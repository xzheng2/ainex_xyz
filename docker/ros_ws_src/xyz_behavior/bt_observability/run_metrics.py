#!/usr/bin/env python3
"""
Reduce one run's tick logs to comparable scalars (stdlib only, Python 3.8+).

WHY THIS EXISTS
    An ablation table needs one row of numbers per run. The tick logs are the raw
    evidence -- thousands of lines per run -- and re-deriving numbers from them at
    analysis time has two failure modes: it does not scale (a 3-variant x 2-body x 5-trial
    campaign is 30 logs), and the reduction rules drift, so runs compared against each
    other end up measured with different rulers. Freezing the reduction into a per-run
    artefact fixes both.

    This covers the machine-observable half only. Whether the ball actually went in,
    whether the robot fell in a pose the BT never recognised, how many times a human
    stepped in -- no log knows any of that, and it is exactly the ablation's primary
    outcome. Those fields come from tools/close_run.py, which calls this and merges in
    what the operator recorded.

WHY HERE AND NOT IN THE ROSA TOOLS
    docker/rosa-agent/xyz_agent_tools/bt_analysis/ already reduces these logs, and
    raw_cross_tick.get_raw_cross_tick_bundle() even accepts a log_dir. It is not reused:
    its output is a diagnostic narrative for an LLM (segment detection, drift summaries,
    representative-tick selection), not comparable scalars, and importing it would add a
    "ROS workspace -> diagnostic agent" dependency edge, backwards from the read-only
    downstream role that agent is designed for.

    The package that produces a log format should own reducing it -- bt_debug_visitor.py,
    which writes these events, is this module's neighbour.

EVENT CONTRACT (bt_debug_visitor.py)
    tree_tick_start  {tick_id}
    tree_tick_end    {tick_id, status}
    tick_end         {tick_id, node, type, status}      -- one per behaviour per tick
    bb_write         {tick_id, writer, key, value}

    Note on `value`: the visitor's _safe_repr() passes scalars through but falls back to
    str() for anything else, so a dict lands in the log as a PYTHON REPR STRING, not a
    JSON object. Dwell state is a dict, hence the ast.literal_eval below. Changing that
    fallback to real JSON would be an improvement, and would also break this parser --
    hence _parse_state() accepts both.
"""
import ast
import json
import os

SCHEMA = 'ainex.run_metrics/1'

#: Blackboard namespace LatchedDwellDecorator stores its state under
#: (xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator.NODE_STATE_NS). Each dwell owns
#: /node_state/<state_key> -> {'stable_ticks': int, 'latched': bool, 'last_tick_id': int}.
NODE_STATE_PREFIX = '/node_state/'

#: The BT decision log. The comm log is not reduced here: it describes ROS traffic, which
#: is a transport concern, not an outcome.
BT_LOG = 'bt_debug_lastrun.jsonl'


def _iter_events(path):
    """Yield parsed JSONL objects, skipping unparseable lines.

    A truncated final line is normal: the logger appends during the run and a crash or a
    kill leaves the tail half-written. One bad line must not cost the whole run's metrics.
    """
    try:
        fh = open(path, encoding='utf-8')
    except OSError:
        return
    with fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except ValueError:
                continue


def _parse_state(value):
    """Return a dwell-state dict from a bb_write value, or None.

    Accepts both the current Python-repr string and a real dict, so this keeps working if
    the visitor's _safe_repr is ever fixed to emit JSON.
    """
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        try:
            parsed = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            return None
        if isinstance(parsed, dict):
            return parsed
    return None


def _empty():
    """The zero-valued shape. A run that was opened but never driven is not an error."""
    return {
        'schema':             SCHEMA,
        'log_present':        False,
        'ticks_total':        0,
        'tick_id_first':      None,
        'tick_id_last':       None,
        'first_success_tick': None,
        'root_status_counts': {},
        'node_ticks':         {},
        'node_status_counts': {},
        'dwells':             {},
        'events_total':       0,
    }


def reduce_run(run_dir):
    """Reduce one run directory to a metrics dict. Never raises on missing/short logs."""
    metrics = _empty()
    log_path = os.path.join(run_dir, BT_LOG)
    if not os.path.isfile(log_path):
        return metrics

    tick_ids = set()
    root_status = {}
    first_success = None
    node_ticks = {}
    node_status = {}
    dwell_state = {}          # state_key -> accumulating counters
    dwell_prev = {}           # state_key -> last seen state dict
    events = 0

    for ev in _iter_events(log_path):
        events += 1
        kind = ev.get('event')
        tick_id = ev.get('tick_id')

        if kind == 'tree_tick_start':
            if isinstance(tick_id, int):
                tick_ids.add(tick_id)

        elif kind == 'tree_tick_end':
            status = ev.get('status')
            if status is not None:
                # py_trees stringifies as "Status.SUCCESS"; keep the short name.
                status = str(status).rsplit('.', 1)[-1]
                root_status[status] = root_status.get(status, 0) + 1
                if status == 'SUCCESS' and first_success is None \
                        and isinstance(tick_id, int):
                    first_success = tick_id
            if isinstance(tick_id, int):
                tick_ids.add(tick_id)

        elif kind == 'tick_end':
            node = ev.get('node')
            if node:
                node_ticks[node] = node_ticks.get(node, 0) + 1
                status = str(ev.get('status', '')).rsplit('.', 1)[-1]
                per = node_status.setdefault(node, {})
                per[status] = per.get(status, 0) + 1

        elif kind == 'bb_write':
            key = ev.get('key') or ''
            if not key.startswith(NODE_STATE_PREFIX):
                continue
            state = _parse_state(ev.get('value'))
            if state is None:
                continue
            name = key[len(NODE_STATE_PREFIX):]
            acc = dwell_state.setdefault(name, {
                'latch_count': 0, 'reset_count': 0,
                'ticks_to_first_latch': None, 'max_stable_ticks': 0,
            })
            prev = dwell_prev.get(name)
            stable = state.get('stable_ticks')
            latched = bool(state.get('latched'))

            if isinstance(stable, int):
                acc['max_stable_ticks'] = max(acc['max_stable_ticks'], stable)
                # A reset is the counter falling back to 0 after having climbed. This is
                # the direct observable for the dwell-N ablation: the hypothesis is that a
                # larger N suppresses false detections, and a false detection shows up in
                # the log precisely as the dwell being reset.
                if prev is not None and stable == 0 \
                        and isinstance(prev.get('stable_ticks'), int) \
                        and prev['stable_ticks'] > 0:
                    acc['reset_count'] += 1

            if latched and not (prev is not None and prev.get('latched')):
                acc['latch_count'] += 1
                if acc['ticks_to_first_latch'] is None and isinstance(tick_id, int):
                    acc['ticks_to_first_latch'] = tick_id

            dwell_prev[name] = state

    metrics.update({
        'log_present':        True,
        'ticks_total':        len(tick_ids),
        'tick_id_first':      min(tick_ids) if tick_ids else None,
        'tick_id_last':       max(tick_ids) if tick_ids else None,
        'first_success_tick': first_success,
        'root_status_counts': root_status,
        'node_ticks':         node_ticks,
        'node_status_counts': node_status,
        'dwells':             dwell_state,
        'events_total':       events,
        # Wall-clock duration is not in the log. The file's mtime span is the closest
        # available proxy and is named to say so -- it includes any idle time between the
        # run being opened and the robot actually starting.
        'duration_s_approx':  _mtime_span(run_dir),
    })
    return metrics


def _mtime_span(run_dir):
    times = []
    for name in os.listdir(run_dir):
        try:
            times.append(os.stat(os.path.join(run_dir, name)).st_mtime)
        except OSError:
            continue
    if len(times) < 2:
        return None
    return round(max(times) - min(times), 1)


if __name__ == '__main__':                      # quick look, not a CLI
    import sys
    print(json.dumps(reduce_run(sys.argv[1]), indent=2, sort_keys=True))
