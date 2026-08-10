#!/usr/bin/env python3
"""Correlate ROSA session-log user observations with BT ticks (skill-only helper).

Part of the `xyz-bt-lastrun-diagnose` Claude Code skill. Reads ROSA's JSONL session
transcripts (written by docker/rosa-agent/runtime/session_logger.py) and ties each user
turn to the BT tick_id(s) it was actually about, so the human's runtime observation
("why is the head sweeping but not approaching the ball?") can be folded into the
per-tick BT diagnosis.

WHY tick_id, not wall-clock: the BT free-runs at ~10-15 Hz while a ROSA turn takes
seconds, so a turn's timestamp does NOT map cleanly to a tick. The reliable anchor is the
tick_id that appears in the BT tool calls inside `intermediate_steps` — when the user
asked via analyze_bt_tick / get_bt_tick_raw / cross_tick_analysis / session_digest /
get_bt_status, the tool_input and the (truncated) observation name the exact tick(s).
Turns with no such anchor are kept as *unanchored* (general during-run context, NOT
tick-precise). Timestamps are used only for the coarse job of deciding which session
files overlap the run.

Standalone: stdlib only, host python3, NO dependency on the rosa-agent package (so it
survives a rosa-agent rename/refactor). Run as a CLI or import the functions.
"""
import argparse
import glob
import json
import os
import re
import sys
from datetime import datetime

DEFAULT_SESSION_LOGS_DIR = "/home/pi/docker/rosa-agent/logs"

# BT tools whose calls carry a usable tick anchor.
_ANCHOR_TOOLS = {
    "analyze_bt_tick", "get_bt_tick_raw", "cross_tick_analysis",
    "session_digest", "get_bt_status",
}

# A range wider than this is treated as "broad scope" (e.g. session_digest over the whole
# run) and NOT expanded into per-tick anchors — anchoring an observation to 3000 ticks is
# meaningless.
MAX_ANCHOR_SPAN = 60

# Observation-text patterns that name explicit tick(s).
_OBS_SINGLE = [
    re.compile(r"=== SELECTED TICK\s+(\d+)"),
    re.compile(r"selected_tick_id\s*[:=]\s*(\d+)"),
    re.compile(r"=== TICK\s+(\d+)"),
    re.compile(r"\bto_tick\s*=\s*(\d+)"),
    re.compile(r"\bfrom_tick\s*=\s*(\d+)"),
    re.compile(r"\btick_id\s*[:=]\s*(\d+)"),
]
_OBS_RANGE = re.compile(r"analyzed_tick_ids\s*:\s*(\d+)\s*[-–]\s*(\d+)")
_OBS_LIST = re.compile(r"tick_ids\s*:\s*\[([\d,\s]+)\]")


def expand_tick_selection(sel):
    """Expand a tick_selection string into a set of explicit tick ids.

    Handles "A-B" (incl. en-dash), "A,B,C", and a bare integer. "all" / "latest:N" are
    deliberately NOT expanded (too broad / unresolvable here) and return an empty set.
    """
    if sel is None:
        return set()
    s = str(sel).strip().lower()
    if not s or s == "all" or s.startswith("latest:"):
        return set()
    m = re.fullmatch(r"(\d+)\s*[-–]\s*(\d+)", s)
    if m:
        a, b = int(m.group(1)), int(m.group(2))
        lo, hi = min(a, b), max(a, b)
        if hi - lo > MAX_ANCHOR_SPAN:
            return set()
        return set(range(lo, hi + 1))
    if "," in s:
        out = set()
        for part in s.split(","):
            part = part.strip()
            if part.isdigit():
                out.add(int(part))
        return out
    if s.isdigit():
        return {int(s)}
    return set()


def _anchors_from_input(tool_input):
    """Tick anchors from a tool_input dict (tick_id / tick_selection)."""
    out = set()
    if not isinstance(tool_input, dict):
        return out
    tid = tool_input.get("tick_id")
    if isinstance(tid, int):
        out.add(tid)
    elif isinstance(tid, str) and tid.strip().isdigit():
        out.add(int(tid.strip()))
    out |= expand_tick_selection(tool_input.get("tick_selection"))
    return out


def _anchors_from_observation(observation):
    """Tick anchors parsed from an observation string."""
    out = set()
    if not isinstance(observation, str) or not observation:
        return out
    head = observation  # patterns are near the top, but scanning all is cheap
    for pat in _OBS_SINGLE:
        for m in pat.finditer(head):
            out.add(int(m.group(1)))
    for m in _OBS_RANGE.finditer(head):
        a, b = int(m.group(1)), int(m.group(2))
        lo, hi = min(a, b), max(a, b)
        if hi - lo <= MAX_ANCHOR_SPAN:
            out |= set(range(lo, hi + 1))
    for m in _OBS_LIST.finditer(head):
        listed = {int(part.strip()) for part in m.group(1).split(",")
                  if part.strip().isdigit()}
        # Apply the same breadth guard as the range form: a tool that reports
        # "tick_ids: [...]" for its whole window (cross_tick_analysis does this
        # for any selection <= 40 ticks) would otherwise anchor a general
        # question to every tick in the window and survive any drill-down filter.
        if listed and (max(listed) - min(listed)) <= MAX_ANCHOR_SPAN:
            out |= listed
    return out


def _iso_to_unix(iso):
    """Parse an ISO-8601 UTC string (…+00:00 or …Z) to a unix timestamp, or None."""
    if not iso:
        return None
    try:
        return datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp()
    except Exception:
        return None


def parse_session_log(path):
    """Parse one session JSONL file into a dict.

    Returns {session_id, mode, start_unix, end_unix, turns:[...]} where each turn is
    {turn_index, iso_ts, unix_ts, user_query, response_summary, tools, anchored_tick_ids}.
    """
    session_id = os.path.basename(path)
    mode = None
    start_unix = end_unix = None
    turns = []
    try:
        with open(path, "r") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                ev = rec.get("event")
                if ev == "session_start":
                    session_id = rec.get("session_id", session_id)
                    mode = rec.get("mode")
                    start_unix = _iso_to_unix(rec.get("started_at"))
                elif ev == "session_end":
                    end_unix = _iso_to_unix(rec.get("ended_at"))
                elif ev == "turn":
                    iso = rec.get("timestamp")
                    anchors = set()
                    tools = []
                    for st in (rec.get("intermediate_steps") or []):
                        tool = st.get("tool")
                        tools.append(tool)
                        if tool in _ANCHOR_TOOLS:
                            anchors |= _anchors_from_input(st.get("tool_input"))
                            anchors |= _anchors_from_observation(str(st.get("observation", "")))
                    resp = (rec.get("assistant_response") or "").strip().replace("\n", " ")
                    turns.append({
                        "turn_index": rec.get("turn_index"),
                        "iso_ts": iso,
                        "unix_ts": _iso_to_unix(iso),
                        "user_query": (rec.get("user_query") or "").strip(),
                        "response_summary": resp[:200] + ("…" if len(resp) > 200 else ""),
                        "tools": tools,
                        "anchored_tick_ids": sorted(anchors),
                    })
    except OSError:
        return None
    # Fall back to first/last turn timestamps if start/end markers are missing.
    turn_unix = [t["unix_ts"] for t in turns if t["unix_ts"] is not None]
    if start_unix is None and turn_unix:
        start_unix = min(turn_unix)
    if end_unix is None and turn_unix:
        end_unix = max(turn_unix)
    return {"session_id": session_id, "mode": mode,
            "start_unix": start_unix, "end_unix": end_unix, "turns": turns}


def bt_ts_range(bt_log_path):
    """Return (min_ts, max_ts) over the `ts` epoch fields of a BT JSONL log, or None."""
    if not bt_log_path or not os.path.isfile(bt_log_path):
        return None
    lo = hi = None
    pat = re.compile(rb'"ts":\s*([0-9.]+)')
    try:
        with open(bt_log_path, "rb") as fh:
            for line in fh:
                m = pat.search(line)
                if not m:
                    continue
                try:
                    ts = float(m.group(1))
                except ValueError:
                    continue
                lo = ts if lo is None or ts < lo else lo
                hi = ts if hi is None or ts > hi else hi
    except OSError:
        return None
    if lo is None:
        return None
    return (lo, hi)


def find_overlapping_sessions(logs_dir, bt_range):
    """Return parsed session dicts whose [start,end] intersects the run's BT ts range.

    If bt_range is None, all parseable sessions are returned (no time filter)."""
    out = []
    for path in sorted(glob.glob(os.path.join(logs_dir, "session_*.jsonl"))):
        s = parse_session_log(path)
        if not s:
            continue
        if bt_range is None:
            out.append(s)
            continue
        bt_lo, bt_hi = bt_range
        s_lo, s_hi = s["start_unix"], s["end_unix"]
        if s_lo is None or s_hi is None:
            continue
        if s_hi >= bt_lo and s_lo <= bt_hi:   # ranges intersect
            out.append(s)
    return out


def build_overlay(sessions, tick_filter=None):
    """Collapse sessions into {anchored: [obs…], unanchored: [obs…]}.

    Each obs = {tick_ids, turn_index, session_id, user_query, response_summary, tool}.
    With tick_filter (a set of ints), anchored obs are kept only if their tick set
    intersects the filter, and unanchored obs are dropped."""
    anchored, unanchored = [], []
    for s in sessions:
        for t in s["turns"]:
            obs = {
                "tick_ids": t["anchored_tick_ids"],
                "turn_index": t["turn_index"],
                "session_id": s["session_id"],
                "user_query": t["user_query"],
                "response_summary": t["response_summary"],
                "tool": next((x for x in t["tools"] if x in _ANCHOR_TOOLS), None),
            }
            if t["anchored_tick_ids"]:
                if tick_filter is not None and not (set(t["anchored_tick_ids"]) & tick_filter):
                    continue
                anchored.append(obs)
            else:
                if tick_filter is None:
                    unanchored.append(obs)
    return {"anchored": anchored, "unanchored": unanchored}


def format_overlay(overlay, tick_filter=None):
    """Render the overlay as a text block for the skill / LLM to read."""
    lines = ["--- ROSA USER OBSERVATIONS (tick-anchored) ---"]
    if tick_filter is not None:
        lo, hi = min(tick_filter), max(tick_filter)
        lines.append(f"(filtered to ticks {lo}-{hi})")

    anchored = overlay["anchored"]
    unanchored = overlay["unanchored"]

    if not anchored and not unanchored:
        lines.append("no ROSA session overlapped this run (no user observations to fold in).")
        return "\n".join(lines)

    if anchored:
        # group by the smallest anchored tick for stable ordering
        by_tick = {}
        for o in anchored:
            key = min(o["tick_ids"])
            by_tick.setdefault(key, []).append(o)
        lines.append("\nAnchored to specific tick(s):")
        for key in sorted(by_tick):
            for o in by_tick[key]:
                span = o["tick_ids"]
                tag = f"{span[0]}" if len(span) == 1 else f"{span[0]}-{span[-1]}"
                lines.append(f"\n  [tick {tag}] via {o['tool']} "
                             f"(session {o['session_id']}, turn {o['turn_index']})")
                lines.append(f"    user: {o['user_query']}")
                if o["response_summary"]:
                    lines.append(f"    rosa: {o['response_summary']}")
    else:
        lines.append("\nAnchored to specific tick(s): (none)")

    if unanchored:
        lines.append("\nUnanchored (general during-run observations — NOT tick-precise):")
        for o in unanchored:
            lines.append(f"\n  (session {o['session_id']}, turn {o['turn_index']})")
            lines.append(f"    user: {o['user_query']}")
            if o["response_summary"]:
                lines.append(f"    rosa: {o['response_summary']}")
    return "\n".join(lines)


def _parse_tick_range(s):
    """Parse the --tick-range argument into a set of ticks, or None for no filter.

    Raises ValueError rather than silently returning None for a syntactically
    valid but unusable selection: expand_tick_selection() deliberately refuses
    spans wider than MAX_ANCHOR_SPAN and forms like "all"/"latest:N", and
    quietly turning those into "no filter" would print the entire run under a
    heading that claims it was filtered.
    """
    if not s:
        return None
    ticks = expand_tick_selection(s)
    if not ticks:
        raise ValueError(
            "--tick-range {!r} did not expand to a usable tick set. Use an "
            "explicit range no wider than {} ticks (e.g. 556-561) or a comma "
            "list (e.g. 556,558,560).".format(s, MAX_ANCHOR_SPAN))
    return ticks


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="session_correlate",
        description="Fold ROSA session-log user observations into BT-tick diagnosis.")
    ap.add_argument("--session-logs-dir", default=DEFAULT_SESSION_LOGS_DIR,
                    help="Directory of ROSA session_*.jsonl logs.")
    ap.add_argument("--bt-log", default=None,
                    help="Path to the run's bt_debug_lastrun.jsonl (for ts-overlap filtering).")
    ap.add_argument("--tick-range", default=None,
                    help='Restrict to a tick window, e.g. "556-561" (drill-down).')
    ap.add_argument("--out", choices=["text", "json"], default="text")
    args = ap.parse_args(argv)

    if not os.path.isdir(args.session_logs_dir):
        print(f"error: --session-logs-dir not a directory: {args.session_logs_dir}",
              file=sys.stderr)
        return 2

    bt_range = bt_ts_range(args.bt_log) if args.bt_log else None
    sessions = find_overlapping_sessions(args.session_logs_dir, bt_range)
    try:
        tick_filter = _parse_tick_range(args.tick_range)
    except ValueError as exc:
        ap.error(str(exc))          # exits 2 with a usage message, not a traceback
    overlay = build_overlay(sessions, tick_filter=tick_filter)

    if args.out == "json":
        print(json.dumps({
            "bt_ts_range": bt_range,
            "tick_filter": sorted(tick_filter) if tick_filter else None,
            "overlapping_sessions": [s["session_id"] for s in sessions],
            "overlay": overlay,
        }, default=str))
    else:
        if args.bt_log and bt_range is None:
            print(f"warning: no ts found in {args.bt_log}; showing all sessions unfiltered.",
                  file=sys.stderr)
        print(format_overlay(overlay, tick_filter=tick_filter))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
