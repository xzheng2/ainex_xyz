#!/usr/bin/env python3
"""
ROSA tool: session_digest — coarse whole-run BT overview from the lastrun logs.

This is the LIGHT, beginner-facing fallback for when cross_tick_analysis (which reads
the rolling ~30-tick recent window) does not contain the relevant ticks — e.g. the
failure happened earlier in the run and has scrolled out of the recent window.

It reads the full-session lastrun files (bt_debug_lastrun.jsonl +
bt_ros_comm_debug_lastrun.jsonl) from the container mount and returns a BOUNDED digest:
  - state occupancy profile (which BT states the robot spent time in, and for how long)
  - transition-type histogram (the distinct kinds of behavior change, with counts)

It deliberately omits per-tick snapshots to stay within the LLM context budget. To inspect
a suspect span in detail, pass tick_selection="A-B" (still coarse) or hand the run to the
Claude Code skill `xyz-bt-lastrun-diagnose` for full per-tick forensic drill-down.

Deterministic reduction lives in bt_analysis/raw_cross_tick.py; rendering in
bt_analysis/cross_tick_format.py (shared with cross_tick_analysis and the host CLI).
"""
from langchain.agents import tool

from xyz_agent_tools.bt_analysis.raw_cross_tick import get_raw_cross_tick_bundle
from xyz_agent_tools.bt_analysis.cross_tick_format import format_digest


@tool
def session_digest(tick_selection: str = "all", user_observation: str = "") -> str:
    """
    Summarize a COMPLETED BT run from the full-session lastrun logs (coarse, whole-run view).

    Reads bt_debug_lastrun.jsonl + bt_ros_comm_debug_lastrun.jsonl (the entire run, not the
    30-tick recent window). Returns a bounded digest: a state occupancy profile (which
    execution states the robot was in and for how long) and a transition-type histogram
    (the distinct kinds of behavior change with counts and example tick_ids). Per-tick raw
    snapshots are NOT included — this view stays bounded for runs of any length.

    Args:
        tick_selection: which ticks to aggregate.
            "all"        → the whole run (default)
            "A-B"        → restrict the digest to tick range A..B inclusive
            "latest:N"   → the last N ticks of the run
            "A,B,C"      → explicit ticks
        user_observation: optional natural-language observation to compare against the logs.

    Use this tool when:
      - The user asks about the WHOLE run / "what happened during that run?" / "earlier in
        the run" / "why did it fail" — beyond the recent 30-tick window.
      - cross_tick_analysis reports the relevant ticks are no longer in the recent window.

    This is a coarse overview to LOCATE where behavior diverged. For deep per-tick forensic
    analysis of a completed run, the Claude Code skill `xyz-bt-lastrun-diagnose` (run by a
    developer on the host) is the primary tool — mention it if the user needs more depth.
    """
    bundle = get_raw_cross_tick_bundle(tick_selection, source="lastrun", detail="digest")
    if not bundle["ok"]:
        return f"session_digest error: {bundle['error']}"
    return format_digest(bundle, user_observation)
