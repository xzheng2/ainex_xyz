#!/usr/bin/env python3
"""Open a new experiment run on this body and stamp what produced it (stdlib only).

WHY THIS EXISTS
    An ablation table is only evidence if every row can be traced back to the exact code
    that produced it. This opens the run's directory and writes ``run_meta.json`` BEFORE
    the robot starts, recording the code commit, whether the tree was dirty, which body,
    which variant, and the checksums of the calibration files that decide how the robot
    moves.

    Run it on the HOST. The provenance it records is the code repository's, and the
    repository is not mounted into the ROS container -- only the workspace source is.
    Hand the run to the node through ``AINEX_RUN_ID`` (this script prints the line).

    All the real work lives in ``bt_observability/run_context.py``, which the
    containerised node imports too, so there is exactly one implementation of "what is a
    run" rather than a host copy and a robot copy drifting apart.

Usage:
    new_run.py --variant ablA --trial 1
    new_run.py --variant baseline --trial 3 --param gait.step_height=0.02 --note "left foot retuned"
    new_run.py --print-body-id
"""
import argparse
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)                       # .../xyz_behavior
sys.path.insert(0, _PKG_DIR)

from bt_observability import run_context                      # noqa: E402

_LOG_DIR = os.path.join(_PKG_DIR, 'log')


def parse_params(pairs):
    """Turn ['a=1', 'b.c=x'] into {'a': 1, 'b.c': 'x'}; numbers/bools are typed."""
    params = {}
    for pair in pairs or []:
        if '=' not in pair:
            raise SystemExit('--param expects key=value, got {!r}'.format(pair))
        key, _, raw = pair.partition('=')
        params[key.strip()] = _coerce(raw.strip())
    return params


def _coerce(raw):
    low = raw.lower()
    if low in ('true', 'false'):
        return low == 'true'
    if low in ('none', 'null'):
        return None
    for cast in (int, float):
        try:
            return cast(raw)
        except ValueError:
            pass
    return raw


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--variant', help='ablation variant name, e.g. ablA / baseline')
    ap.add_argument('--trial', type=int, help='repeat index within this variant, 1-based')
    ap.add_argument('--param', action='append', metavar='KEY=VALUE',
                    help='variant parameter, repeatable')
    ap.add_argument('--note', default='', help='free-text note about this run')
    ap.add_argument('--log-dir', default=_LOG_DIR,
                    help='project log dir holding runs/ (default: %(default)s)')
    ap.add_argument('--max-runs', type=int, default=run_context.DEFAULT_MAX_RUNS,
                    help='run directories to keep (default: %(default)s)')
    ap.add_argument('--print-body-id', action='store_true',
                    help='resolve and cache this body id, then exit')
    args = ap.parse_args(argv)

    try:
        body_id = run_context.resolve_body_id(args.log_dir)
    except run_context.BodyIdError as exc:
        raise SystemExit(str(exc))

    if args.print_body_id:
        print(body_id)
        return 0

    if not args.variant or args.trial is None:
        raise SystemExit('--variant and --trial are required (or use --print-body-id)')

    cfg = run_context.body_config_path(body_id)
    if not os.path.isfile(cfg):
        raise SystemExit(
            'body {!r} has no config file at {}\n'
            'Create it — copy an existing one, see {}/config/bodies/README.md'
            .format(body_id, cfg, _PKG_DIR))

    run_id = run_context.new_run_id(args.variant, args.trial)
    run_dir = run_context.open_run_dir(args.log_dir, run_id=run_id,
                                       max_runs=args.max_runs,
                                       log=lambda m: sys.stderr.write(m + '\n'))
    run_context.write_run_meta(run_dir, body_id, variant=args.variant, trial=args.trial,
                               params=parse_params(args.param), note=args.note)

    meta = run_context.read_run_meta(run_dir) or {}
    git = meta.get('git') or {}
    if git.get('available') and git.get('dirty'):
        sys.stderr.write(
            'WARNING: code tree is dirty ({} changed path(s)) — this run is NOT '
            'reproducible and must not be used as ablation evidence.\n'
            .format(git['dirty_count']))

    # The node adopts this directory only if it sees the id; without it, it opens a run
    # of its own and this one stays behind holding nothing but its meta.
    sys.stderr.write('hand this run to the robot with:\n'
                     '  export AINEX_RUN_ID={}\n'.format(run_id))
    print(run_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
