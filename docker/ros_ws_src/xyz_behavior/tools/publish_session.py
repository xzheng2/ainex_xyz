#!/usr/bin/env python3
"""Publish a coding session's process data into the results repository (stdlib only).

WHY THIS EXISTS
    ``publish_runs.py`` publishes what the ROBOT did. This publishes what the AGENT did
    getting there: tokens spent, tools called, files reworked, clarifications asked.
    Those four numbers live only in the CLI's session transcripts under
    ``~/.claude/projects/``, which are in no repository and die with the SD card.

    Run it on the HOST, at the end of a working session -- the transcripts are the
    host's, and the results repo is not mounted into any container.

WHY IT IS A SEPARATE TOOL FROM publish_runs.py
    Runs and sessions do not correspond. One session can open several runs, a run can
    span sessions, and most sessions open none at all. Folding this into publish_runs
    would tie an export that should happen when the agent stops to an event that
    happens when the robot stops.

WHY NOT IN .claude/
    The skills+hooks engine is an ablated factor: two of the four lanes will not have a
    ``.claude/`` worth speaking of. Instrumentation that must run identically in all
    four lanes lives in xyz_run_lab, exactly like run_context.

Usage:
    publish_session.py                       # the live session
    publish_session.py --session <id> ...    # named sessions
    publish_session.py --all                 # every session in this project
    publish_session.py --dry-run
"""
import argparse
import json
import os
import subprocess
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)                       # .../xyz_behavior
_WS_SRC = os.path.dirname(_PKG_DIR)                           # .../ros_ws_src
sys.path.insert(0, os.path.join(_WS_SRC, 'xyz_run_lab'))

from run_lab import run_context                               # noqa: E402
from run_lab import session_export                            # noqa: E402

_LOG_DIR = os.path.join(_PKG_DIR, 'log')

DEFAULT_RESULTS_REPO = '/home/pi/experiments/ainex_xyz_result'

#: Fields lifted into the session index, mirroring publish_runs._INDEX_FIELDS: flat and
#: small, so an ablation table is one `cat` away and never has to open a session dir.
_INDEX_FIELDS = ('session_id', 'body_id', 'study', 'lane', 'slug', 'first_ts', 'last_ts',
                 'duration_s', 'api_responses', 'tool_calls_total', 'files_touched',
                 'rework_events', 'rework_files', 'clarification_calls',
                 'clarification_questions', 'human_turns', 'subagent_count',
                 'tool_errors')


def index_shard(repo, body_id, study, lane):
    """One writer per shard, same rule as publish_runs -- but in its own subdirectory
    so the README's `cat index/*.jsonl` keeps meaning "the runs"."""
    return os.path.join(repo, 'index', 'sessions',
                        '{}__{}__{}.jsonl'.format(body_id, study, lane))


def published_session_ids(index_path):
    ids = set()
    if not os.path.isfile(index_path):
        return ids
    try:
        with open(index_path, encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    ids.add(json.loads(line).get('session_id'))
                except ValueError:
                    continue
    except OSError:
        pass
    return ids


def index_entry(meta, metrics, guard):
    entry = {
        'session_id': meta['session_id'],
        'body_id': meta['body_id'],
        'study': meta['study'],
        'lane': meta['lane'],
        'slug': meta.get('slug'),
        'first_ts': metrics.get('first_ts'),
        'last_ts': metrics.get('last_ts'),
        'duration_s': metrics.get('duration_s'),
        'api_responses': metrics.get('api_responses'),
        'tool_calls_total': metrics.get('tool_calls_total'),
        'files_touched': metrics.get('files_touched'),
        'rework_events': metrics.get('rework_events'),
        'rework_files': metrics.get('rework_files'),
        'clarification_calls': metrics.get('clarification_calls'),
        'clarification_questions': metrics.get('clarification_questions'),
        'human_turns': metrics.get('human_turns'),
        'subagent_count': metrics.get('subagent_count'),
        'tool_errors': metrics.get('tool_errors'),
    }
    # Main and subagent totals are kept apart in metrics.json; the index carries the
    # sum, because "what did this session cost" is one number.
    for key in ('input', 'output', 'cache_creation', 'cache_read', 'total'):
        entry['tokens_' + key] = ((metrics.get('tokens') or {}).get(key, 0) +
                                  (metrics.get('tokens_subagent') or {}).get(key, 0))
    entry['guard_events'] = guard.get('events', 0)
    entry['guard_blocked'] = (guard.get('by_action') or {}).get('blocked', 0)
    entry['guard_warned'] = (guard.get('by_action') or {}).get('warned', 0)
    entry['guard_fix_on_first_retry'] = guard.get('fix_on_first_retry')
    return entry


def export_one(session_id, project_path, body_id, study, lane, repo,
               dry_run=False, force=False, guard_log=None):
    """Distil, reduce and write one session. Returns 'published'|'skipped'|'would-publish'."""
    dest = os.path.join(repo, 'sessions', body_id, study, lane, session_id)
    index_path = index_shard(repo, body_id, study, lane)

    if (os.path.exists(dest) or session_id in published_session_ids(index_path)) \
            and not force:
        return 'skipped', None

    lines, info = session_export.distil_session(project_path, session_id)
    if not lines:
        sys.stderr.write('skipping {}: no conversation lines found\n'.format(session_id))
        return 'skipped', None

    metrics = session_export.reduce_session(lines, info)
    events = session_export.guard_events(session_id, guard_log)
    guard = session_export.reduce_guard_events(events)
    metrics['guard'] = guard

    meta = {
        'schema': 'ainex.session_meta/1',
        'session_id': session_id,
        'body_id': body_id,
        'study': study,
        'lane': lane,
        'slug': info.get('slug'),
        'cwd': info.get('cwd'),
        'cli_versions': info.get('versions'),
        'main_transcript': info.get('main_transcript'),
        'subagents': info.get('subagents'),
        'exported_utc': None,
        'git': run_context.code_provenance(),
    }
    import time
    meta['exported_utc'] = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())

    if dry_run:
        return 'would-publish', metrics

    os.makedirs(dest, exist_ok=True)
    with open(os.path.join(dest, 'transcript.jsonl'), 'w', encoding='utf-8') as fh:
        for row in lines:
            fh.write(json.dumps(row, sort_keys=True) + '\n')
    for name, payload in (('session_meta.json', meta),
                          ('session_metrics.json', metrics)):
        with open(os.path.join(dest, name), 'w', encoding='utf-8') as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)
            fh.write('\n')
    # Written even when empty: "this lane had no guards" is a measurement, and an
    # absent file would be indistinguishable from an export that forgot to look.
    with open(os.path.join(dest, 'guard_events.jsonl'), 'w', encoding='utf-8') as fh:
        for event in events:
            fh.write(json.dumps(event, sort_keys=True) + '\n')

    if session_id not in published_session_ids(index_path):
        os.makedirs(os.path.dirname(index_path), exist_ok=True)
        with open(index_path, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps(index_entry(meta, metrics, guard), sort_keys=True) + '\n')
    return 'published', metrics


def git_commit(repo, count, writer):
    subject = 'sessions: publish {} session(s) from {}'.format(count, writer)
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
    ap.add_argument('--session', action='append', metavar='ID',
                    help='session id to export; repeatable (default: the live session)')
    ap.add_argument('--all', action='store_true',
                    help='export every session with a transcript in this project')
    ap.add_argument('--projects-dir', default=session_export.DEFAULT_PROJECTS_DIR,
                    help='Claude Code projects dir (default: %(default)s)')
    ap.add_argument('--project-cwd', default='/home/pi',
                    help='project root whose transcripts to read (default: %(default)s)')
    ap.add_argument('--results-repo', default=DEFAULT_RESULTS_REPO,
                    help='results repository working copy (default: %(default)s)')
    ap.add_argument('--guard-log', default=None,
                    help='guard event log (default: $AINEX_GUARD_LOG or the standard path)')
    ap.add_argument('--log-dir', default=_LOG_DIR,
                    help='project log dir holding .body_id/.study/.lane (default: %(default)s)')
    ap.add_argument('--commit', action='store_true',
                    help='commit in the results repo after publishing')
    ap.add_argument('--dry-run', action='store_true',
                    help='report what would be exported, change nothing')
    ap.add_argument('--force', action='store_true',
                    help='re-export sessions already present (overwrites them)')
    args = ap.parse_args(argv)

    if not os.path.isdir(os.path.join(args.results_repo, '.git')):
        raise SystemExit('not a git repository: {}\n'
                         'Clone the results repo there, or pass --results-repo.'
                         .format(args.results_repo))

    try:
        body_id = run_context.resolve_body_id(args.log_dir)
    except run_context.BodyIdError as exc:
        raise SystemExit(str(exc))
    try:
        study = run_context.resolve_study(args.log_dir)
    except run_context.StudyError as exc:
        raise SystemExit(str(exc))
    try:
        lane = run_context.resolve_lane(args.log_dir)
    except run_context.LaneError as exc:
        raise SystemExit(str(exc))

    project_path = os.path.join(args.projects_dir,
                                session_export.project_dir_name(args.project_cwd))
    if not os.path.isdir(project_path):
        raise SystemExit('no transcripts for {}: {} does not exist'
                         .format(args.project_cwd, project_path))

    if args.all:
        targets = session_export.session_ids_in(project_path)
    elif args.session:
        targets = args.session
    else:
        live = session_export.live_session_ids()
        if not live:
            raise SystemExit('no live session found — pass --session <id> or --all')
        targets = [live[0][0]]
        sys.stderr.write('exporting live session {}\n'.format(targets[0]))

    counts = {'published': 0, 'skipped': 0, 'would-publish': 0}
    for session_id in targets:
        status, metrics = export_one(
            session_id, project_path, body_id, study, lane, args.results_repo,
            dry_run=args.dry_run, force=args.force, guard_log=args.guard_log)
        counts[status] += 1
        if metrics:
            tokens = metrics['tokens']['total'] + metrics['tokens_subagent']['total']
            print('  {:<14} {}  {} tools, {} tokens, {} guard events'.format(
                status, session_id, metrics['tool_calls_total'], tokens,
                metrics['guard']['events']))
        else:
            print('  {:<14} {}'.format(status, session_id))

    print('\n{} published, {} already present, {} would publish'.format(
        counts['published'], counts['skipped'], counts['would-publish']))

    if args.commit and counts['published']:
        if not git_commit(args.results_repo, counts['published'],
                          '{} {} lane {}'.format(body_id, study, lane)):
            return 1
        print('committed in {}'.format(args.results_repo))
    return 0


if __name__ == '__main__':
    sys.exit(main())
