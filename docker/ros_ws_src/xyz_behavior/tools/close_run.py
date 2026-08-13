#!/usr/bin/env python3
"""Close a run: record what happened, alongside what the logs measured (stdlib only).

WHY A SEPARATE STEP FROM new_run.py
    new_run.py records the INDEPENDENT variables before the robot moves -- which commit,
    which body, which parameters. This records the DEPENDENT ones, which only exist once
    the run is over. Two files, because they cannot be written at the same moment:
    run_meta.json must be stamped before anything can drift, metrics.json cannot exist
    until there is something to measure.

WHY A HUMAN IS IN THE LOOP
    The machine half comes from xyz_run_lab/run_lab/run_metrics.py. The other half cannot:
    a behaviour tree only knows what it itself judged. Whether the ball went in, whether
    the robot fell in a pose no L1 node recognised, how many times someone stepped in and
    repositioned it -- no log holds any of that, and it is exactly the ablation's primary
    outcome. So it is typed in here.

RE-RUNNABLE ON PURPOSE
    Machine metrics are recomputed every call and human fields take the latest values, so
    recording `--outcome failure` right after the run and adding `--failure-mode` once
    you have looked at the logs is a supported two-step, not a mistake to work around.

Usage:
    close_run.py --latest --outcome success
    close_run.py <run_dir> --outcome failure --failure-mode lost_target --interventions 2
"""
import argparse
import json
import os
import sys
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)                       # .../xyz_behavior
_WS_SRC = os.path.dirname(_PKG_DIR)                           # .../ros_ws_src
sys.path.insert(0, os.path.join(_WS_SRC, 'xyz_run_lab'))

from run_lab import run_metrics                               # noqa: E402
from run_lab import run_context                               # noqa: E402

_LOG_DIR = os.path.join(_PKG_DIR, 'log')

#: Free-form on purpose. A fixed enum decided up front would be wrong up front: the
#: failure modes worth distinguishing are discovered by running the experiment. Keep them
#: short and reuse them consistently -- they are grouped verbatim during analysis.
_FAILURE_MODE_HINT = 'e.g. lost_target, fell, timeout, missed_kick, operator_error'


def latest_run(log_dir):
    root = run_context.runs_root(log_dir)
    if not os.path.isdir(root):
        return None
    runs = [os.path.join(root, n) for n in os.listdir(root)]
    runs = [p for p in runs if os.path.isdir(p)]
    if not runs:
        return None
    return max(runs, key=lambda p: os.stat(p).st_mtime)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir', nargs='?', help='run directory (or use --latest)')
    ap.add_argument('--latest', action='store_true',
                    help='close the most recently modified run under log/runs/')
    ap.add_argument('--outcome', required=True,
                    choices=['success', 'failure', 'aborted'],
                    help='did the robot accomplish the task? aborted = run not valid '
                         '(equipment, interruption) and excluded from analysis')
    ap.add_argument('--interventions', type=int, default=0,
                    help='times a human physically intervened (repositioned, caught, '
                         'restarted a subsystem)')
    ap.add_argument('--failure-mode', default='',
                    help='short slug describing HOW it failed; ' + _FAILURE_MODE_HINT)
    ap.add_argument('--note', default='', help='free-text observation about this run')
    ap.add_argument('--log-dir', default=_LOG_DIR,
                    help='project log dir holding runs/ (default: %(default)s)')
    args = ap.parse_args(argv)

    if args.latest:
        run_dir = latest_run(args.log_dir)
        if not run_dir:
            raise SystemExit('no runs under {}'.format(run_context.runs_root(args.log_dir)))
    elif args.run_dir:
        run_dir = os.path.abspath(args.run_dir)
    else:
        raise SystemExit('give a run directory, or --latest')

    if not os.path.isdir(run_dir):
        raise SystemExit('not a directory: {}'.format(run_dir))
    if not os.path.isfile(os.path.join(run_dir, 'run_meta.json')):
        raise SystemExit(
            '{} has no run_meta.json — this is not a run directory (runs are opened '
            'with tools/new_run.py)'.format(run_dir))

    if args.outcome == 'failure' and not args.failure_mode:
        sys.stderr.write('note: --outcome failure without --failure-mode; the run is '
                         'recorded, but "why" is what makes failures comparable.\n')

    metrics = run_metrics.reduce_run(run_dir)
    metrics.update({
        'run_id':        os.path.basename(run_dir.rstrip(os.sep)),
        'outcome':       args.outcome,
        'interventions': args.interventions,
        'failure_mode':  args.failure_mode,
        'note':          args.note,
        'closed_utc':    time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()),
    })

    path = os.path.join(run_dir, 'metrics.json')
    revising = os.path.isfile(path)
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
        fh.write('\n')

    if not metrics['log_present']:
        sys.stderr.write(
            'WARNING: no {} in this run — only the fields you typed were recorded. '
            'Did the BT node actually run against this run id?\n'.format(run_metrics.BT_LOG))

    print('{} {}: outcome={} ticks={} interventions={}{}'.format(
        'revised' if revising else 'closed',
        metrics['run_id'], args.outcome, metrics['ticks_total'], args.interventions,
        ' failure_mode=' + args.failure_mode if args.failure_mode else ''))
    print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
