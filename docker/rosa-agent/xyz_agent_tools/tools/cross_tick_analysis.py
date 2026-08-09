#!/usr/bin/env python3
"""
ROSA tool for BT cross-tick analysis.

Tool: cross_tick_analysis
  Analyzes BT execution, blackboard, and ROS communication across up to 30 recent
  ticks. Detects stable segments, transitions, and supporting evidence drift.
  All grouping and compression is deterministic; the ROSA LLM writes the report.

Deterministic reduction lives in bt_analysis/raw_cross_tick.py; string rendering lives
in bt_analysis/cross_tick_format.py (shared, langchain-free) so the same scaffold is
reused by the host lastrun CLI and the session_digest tool.
"""
from langchain.agents import tool

from xyz_agent_tools.bt_analysis.raw_cross_tick import get_raw_cross_tick_bundle
from xyz_agent_tools.bt_analysis.cross_tick_format import format_scaffold


# ---------------------------------------------------------------------------
# Tool
# ---------------------------------------------------------------------------

@tool
def cross_tick_analysis(
    tick_selection: str = "all",
    user_observation: str = "",
) -> str:
    """
    Analyze cross-tick BT behavior from recent debug logs.

    Reads bt_debug_recent.jsonl and bt_ros_comm_debug_recent.jsonl (recent only, up to 30 ticks).
    Performs deterministic grouping, transition detection, drift summarization, and
    representative tick selection before returning a scaffold for ROSA to write the report.

    Args:
        tick_selection: how to select ticks from the recent window.
            "all"        → all available tick_ids (default)
            "latest:N"   → last N tick_ids by value
            "A-B"        → tick_ids in range A to B inclusive
            "A,B,C"      → explicit tick_ids (always included in representative set)

        user_observation: optional natural-language physical or UI observation
            spanning multiple ticks. Log-only analysis is complete without it.

    Use this tool when:
      - The user asks about multiple ticks, trends, or episodes.
      - "What changed?", "How long was the BT in FindLine?", "When did the transition happen?"
      - "Summarize recent BT activity."
      For a single specific tick with physical observation, use analyze_bt_tick instead.
      For the WHOLE completed run (beyond the 30-tick recent window), use session_digest.

    TIP — pause/step mode gives the most accurate capture:
      This tool is best suited for observing MULTI-TICK EVOLUTION — transitions,
      stability, and drift across the recent window.  It works especially well
      when the user debugs with BT pause mode (~bt/pause) or step mode (~bt/step):
      pausing freezes the window so it is not overwritten while you analyze, and
      stepping produces a clean, bounded episode.  If the BT is free-running,
      remind the user that pausing or stepping first will make tick capture and
      analysis more accurate.
    """
    bundle = get_raw_cross_tick_bundle(tick_selection)
    if not bundle["ok"]:
        return f"cross_tick_analysis error: {bundle['error']}"
    return format_scaffold(bundle, user_observation)
