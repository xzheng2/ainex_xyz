#!/usr/bin/env python3
"""Publish finished runs from this body into the shared results repository (stdlib only).

WHY A SEPARATE STEP
    The BT node runs inside the ROS container and writes its output to
    ``xyz_behavior/log/runs/<run_id>/``. It knows nothing about git, and should not: ROS
    code that commits to a repository is ROS code that can fail for reasons having
    nothing to do with the robot. Publishing is therefore a host-side step, run when a
    session is finished rather than while the robot is moving.

WHY IT NEVER CONFLICTS
    Every writer writes only under ``results/<body_id>/<study>/<lane>/`` and appends
    only to ``index/<body_id>__<study>__<lane>.jsonl``. Two writers pushing at the same
    time touch disjoint paths, so git has nothing to merge. This is the entire reason
    the results repo is partitioned rather than branched -- see its README.

    Study and lane are part of the partition key, not decoration. The ablation runs four
    arms on ONE robot, so body_id is identical across all four cards; partitioning by
    body alone would put four concurrent writers on one index file and reintroduce
    exactly the conflict the sharding exists to prevent. `study` sits above `lane`
    because a card outlives a study: the same lane `a` card runs exp1 and later exp2,
    and pooling those under one prefix averages across two different experiments.

Usage:
    publish_runs.py                 # copy every unpublished run, update the index
    publish_runs.py --commit        # ... and commit in the results repo
    publish_runs.py --dry-run       # show what would be published
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)                       # .../xyz_behavior
_RUNS_DIR = os.path.join(_PKG_DIR, 'log', 'runs')

DEFAULT_RESULTS_REPO = '/home/pi/experiments/ainex_xyz_result'

#: Fields lifted from run_meta.json into the index. The index exists to be grepped and
#: loaded into a table without opening every run directory, so it stays flat and small.
_INDEX_FIELDS = ('run_id', 'body_id', 'study', 'lane', 'created_utc', 'variant',
                 'trial', 'params', 'note')

#: Fields lifted from metrics.json. Kept as a separate tuple from _INDEX_FIELDS because
#: the two have different sources and different failure modes: run_meta always exists (a
#: run without it is not a run), metrics only exists once someone closed the run.
#:
#: These are the DEPENDENT variables. Without them in the index, an ablation table means
#: opening every run directory in turn -- 30 of them for a 3-variant x 2-body x 5-trial
#: campaign -- which is exactly what the index exists to avoid.
_METRIC_FIELDS = ('outcome', 'interventions', 'failure_mode',
                  'ticks_total', 'first_success_tick')


def discover_runs(runs_dir):
    """Return [(run_dir, meta), ...] for every run directory carrying a run_meta.json."""
    found = []
    if not os.path.isdir(runs_dir):
        return found
    for name in sorted(os.listdir(runs_dir)):
        run_dir = os.path.join(runs_dir, name)
        meta_path = os.path.join(run_dir, 'run_meta.json')
        if not os.path.isfile(meta_path):
            continue
        try:
            with open(meta_path, encoding='utf-8') as fh:
                meta = json.load(fh)
        except (OSError, ValueError) as exc:
            sys.stderr.write('skipping {}: unreadable run_meta.json ({})\n'
                             .format(name, exc))
            continue
        missing = [k for k in ('body_id', 'run_id', 'study', 'lane') if not meta.get(k)]
        if missing:
            # study and lane are as load-bearing as body_id here: together they are the
            # partition key, and a run that cannot say which experiment and which arm
            # produced it cannot be a row in any table.
            sys.stderr.write('skipping {}: run_meta.json has no {}\n'
                             .format(name, '/'.join(missing)))
            continue
        found.append((run_dir, meta))
    return found


def read_metrics(run_dir):
    """Return this run's metrics.json, or None if it was never closed."""
    try:
        with open(os.path.join(run_dir, 'metrics.json'), encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def index_entry(meta, metrics=None):
    entry = {k: meta.get(k) for k in _INDEX_FIELDS}
    git = meta.get('git') or {}
    entry['git_sha'] = git.get('sha')
    entry['git_dirty'] = git.get('dirty')
    # Present-but-null beats absent: a reader loading the shard into a table gets a
    # uniform set of columns, and "not closed" is visible rather than inferred from a
    # missing key.
    for field in _METRIC_FIELDS:
        entry[field] = (metrics or {}).get(field)
    entry['closed'] = metrics is not None
    return entry


def index_shard(repo, body_id, study, lane):
    """This writer's index shard. One writer per shard is what keeps pushes conflict-free."""
    return os.path.join(repo, 'index',
                        '{}__{}__{}.jsonl'.format(body_id, study, lane))


def published_run_ids(index_path):
    """run_ids already present in this writer's index shard."""
    ids = set()
    if not os.path.isfile(index_path):
        return ids
    with open(index_path, encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                ids.add(json.loads(line).get('run_id'))
            except ValueError:
                continue
    return ids


def publish(run_dir, meta, repo, dry_run=False, force=False):
    """Copy one run into the results repo and append its index line.

    Returns 'published', 'skipped' or 'would-publish'.
    """
    body_id, run_id = meta['body_id'], meta['run_id']
    study, lane = meta['study'], meta['lane']
    dest = os.path.join(repo, 'results', body_id, study, lane, run_id)
    index_path = index_shard(repo, body_id, study, lane)

    already = os.path.exists(dest) or run_id in published_run_ids(index_path)
    if already and not force:
        return 'skipped'

    metrics = read_metrics(run_dir)
    if metrics is None:
        # Not a reason to refuse: an exploratory run is still worth keeping. But it has
        # no dependent variables, so it cannot be a row in an ablation table.
        sys.stderr.write(
            'warning: {} has no metrics.json — publishing it, but it carries no outcome '
            'and must not be used as ablation evidence. Close it first:\n'
            '  close_run.py {} --outcome success|failure|aborted\n'
            .format(run_id, run_dir))

    if dry_run:
        return 'would-publish'

    if os.path.exists(dest) and force:
        shutil.rmtree(dest)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.copytree(run_dir, dest)

    if run_id not in published_run_ids(index_path):
        os.makedirs(os.path.dirname(index_path), exist_ok=True)
        with open(index_path, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps(index_entry(meta, metrics), sort_keys=True) + '\n')

    # prune_runs() reads this marker to decide whether deleting an aged-out run is
    # worth warning about. Nothing wrote it until now, so that warning fired on every
    # prune regardless -- a warning that is always on is a warning nobody reads.
    try:
        open(os.path.join(run_dir, '.published'), 'a').close()
    except OSError:
        pass
    return 'published'


def git_commit(repo, count, writers):
    subject = 'results: publish {} run(s) from {}'.format(count, ', '.join(sorted(writers)))
    for cmd in (['git', '-C', repo, 'add', '-A'],
                ['git', '-C', repo, 'commit', '-q', '-m', subject]):
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if proc.returncode != 0:
            sys.stderr.write(proc.stdout.decode('utf-8', 'replace'))
            return False
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs-dir', default=_RUNS_DIR,
                    help='where run directories live (default: %(default)s)')
    ap.add_argument('--results-repo', default=DEFAULT_RESULTS_REPO,
                    help='results repository working copy (default: %(default)s)')
    ap.add_argument('--commit', action='store_true',
                    help='commit in the results repo after publishing')
    ap.add_argument('--dry-run', action='store_true',
                    help='report what would be published, change nothing')
    ap.add_argument('--force', action='store_true',
                    help='republish runs already present (overwrites them)')
    args = ap.parse_args(argv)

    if not os.path.isdir(os.path.join(args.results_repo, '.git')):
        raise SystemExit('not a git repository: {}\n'
                         'Clone the results repo there, or pass --results-repo.'
                         .format(args.results_repo))

    runs = discover_runs(args.runs_dir)
    if not runs:
        print('no runs found under {}'.format(args.runs_dir))
        return 0

    counts = {'published': 0, 'skipped': 0, 'would-publish': 0}
    writers = set()
    for run_dir, meta in runs:
        outcome = publish(run_dir, meta, args.results_repo,
                          dry_run=args.dry_run, force=args.force)
        counts[outcome] += 1
        if outcome != 'skipped':
            writers.add('{} {} lane {}'.format(
                meta['body_id'], meta['study'], meta['lane']))
        print('  {:<14} {}'.format(outcome, meta['run_id']))

    print('\n{} published, {} already present, {} would publish'.format(
        counts['published'], counts['skipped'], counts['would-publish']))

    if args.commit and counts['published']:
        if not git_commit(args.results_repo, counts['published'], writers):
            return 1
        print('committed in {}'.format(args.results_repo))
    return 0


if __name__ == '__main__':
    sys.exit(main())
