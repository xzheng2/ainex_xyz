#!/usr/bin/env python3
"""
ROSA tool for BT cross-tick analysis.

Tool: cross_tick_analysis
  Analyzes BT execution, blackboard, and ROS communication across up to 30 recent
  ticks. Detects stable segments, transitions, and supporting evidence drift.
  All grouping and compression is deterministic; the ROSA LLM writes the report.
"""
import json

from langchain.agents import tool

from ainex_agent_tools.bt_analysis.raw_cross_tick import (
    get_raw_cross_tick_bundle,
    build_signature_digest,
)
from ainex_agent_tools.tools.bt_tick_analysis import _JSONL_SCHEMA_LEGEND

# ---------------------------------------------------------------------------
# Scaffold formatting helpers
# ---------------------------------------------------------------------------

def _fmt_snapshot_block(snap: dict, label: str = "") -> str:
    """Format a PerTickSnapshot as a human-readable block."""
    tid = snap["tick_id"]
    header = f"=== TICK {tid}{(' — ' + label) if label else ''} ==="
    parts = [header]

    # BT node statuses
    parts.append("BT node statuses:")
    for node, info in sorted(snap["bt"]["node_statuses"].items()):
        status = info["status"].replace("Status.", "")
        parts.append(f"  {node} ({info['type']}): {status}")
    if not snap["bt"]["node_statuses"]:
        parts.append("  (none)")

    # Decisions
    if snap["decisions"]:
        parts.append("Decisions:")
        for node, d in sorted(snap["decisions"].items()):
            status = d["status"].replace("Status.", "")
            parts.append(f'  {node}: {status}, reason="{d["reason"]}"')
            if d["inputs"]:
                parts.append(f"    inputs: {json.dumps(d['inputs'], default=str)}")

    # Blackboard
    if snap["blackboard"]:
        parts.append("Blackboard writes:")
        for key, info in sorted(snap["blackboard"].items()):
            parts.append(f"  {key} = {json.dumps(info['value'], default=str)} (writer: {info['writer']})")

    # ros_in
    if snap["ros_in"]:
        parts.append("ros_in (sensor inputs):")
        for src, info in sorted(snap["ros_in"].items()):
            parts.append(f"  {src} → adapter={info['adapter']}, received_count={info['received_count']}")

    # input_state
    if snap["input_state"]:
        parts.append("input_state (adapter BB writes):")
        for adp, info in sorted(snap["input_state"].items()):
            parts.append(f"  {adp}: {json.dumps(info['bb_writes'], default=str)}")

    # ros_out
    if snap["ros_out"]:
        parts.append("ros_out (commands emitted):")
        for r in snap["ros_out"]:
            parts.append(
                f"  {r['bt_node']} / {r['semantic_source']} → {r['target']}"
                f" [{r['comm_type']}] payload={json.dumps(r['payload'], default=str)}"
                f" (confidence={r['attribution_confidence']})"
            )

    return "\n".join(parts)


def _fmt_selected_ticks(bundle: dict) -> str:
    rs = bundle["recent_status"]
    lines = [
        "--- SELECTED TICKS ---",
        f"tick_ids: {bundle['selected_tick_ids']}",
        f"total: {len(bundle['selected_tick_ids'])}",
        f"source: recent only",
    ]
    if rs["bt_age_seconds"] is not None:
        lines.append(f"file_status: bt updated {rs['bt_age_seconds']}s ago")
    if rs["stale_warning"]:
        lines.append(f"STALE WARNING: {rs['stale_warning']}")
    for w in bundle["warnings"]:
        lines.append(f"WARNING: {w}")
    return "\n".join(lines)


def _fmt_segment_digest(bundle: dict) -> str:
    segments = bundle["segments"]
    drift_summaries = bundle["segment_drift_summaries"]
    if not segments:
        return "--- SEGMENT DIGEST ---\n(no segments)"

    lines = ["--- SEGMENT DIGEST ---"]
    for seg in segments:
        idx = seg["index"]
        tids = seg["tick_ids"]
        seg_type = seg["type"]
        tick_range = (
            f"ticks {tids[0]}–{tids[-1]} ({len(tids)} ticks)"
            if len(tids) > 1
            else f"tick {tids[0]}"
        )
        lines.append(f"\n  [{idx}] {seg_type}, {tick_range}")
        for line in seg["signature_digest"].splitlines():
            lines.append(f"    {line}")
        if idx in drift_summaries:
            ds = drift_summaries[idx]
            lines.append(f"    drift: {ds['drift_label']}")
            # Highlight most notable drift items
            notable = []
            for key, v in ds.get("blackboard_value_changes", {}).items():
                if v["changed"]:
                    notable.append(f"{key}: {json.dumps(v['first'], default=str)} → {json.dumps(v['last'], default=str)}")
            for src, v in ds.get("ros_out_summary", {}).items():
                if v["payload_varied"]:
                    notable.append(f"ros_out {src}: payload varied across segment")
            if notable:
                lines.append("    notable changes:")
                for n in notable:
                    lines.append(f"      {n}")

    return "\n".join(lines)


def _fmt_transition_evidence(bundle: dict) -> str:
    transitions = bundle["transitions"]
    if not transitions:
        return "--- TRANSITION EVIDENCE ---\n(no BT transitions detected in selected ticks)"

    snapshots = bundle["snapshots"]
    lines = ["--- TRANSITION EVIDENCE ---"]
    for t in transitions:
        lines.append(
            f"\n  transition [{t['from_segment_index']}] → [{t['to_segment_index']}]:"
            f" from_tick={t['from_tick']} / to_tick={t['to_tick']}"
            + (f" / confirmation_tick={t['confirmation_tick']}" if t["confirmation_tick"] else "")
        )

        if t["changed_bt_nodes"]:
            lines.append("    changed_bt_nodes:")
            for node, chg in t["changed_bt_nodes"].items():
                lines.append(f"      {node}: {chg['before']} → {chg['after']}")

        if t["changed_decisions"]:
            lines.append("    changed_decisions:")
            for node, chg in t["changed_decisions"].items():
                bf = chg["before"]
                af = chg["after"]
                lines.append(f"      {node}:")
                if bf:
                    lines.append(f"        before: inputs={json.dumps(bf.get('inputs'), default=str)}, "
                                 f"{bf.get('status','')}, reason=\"{bf.get('reason','')}\"")
                else:
                    lines.append("        before: [absent]")
                if af:
                    lines.append(f"        after:  inputs={json.dumps(af.get('inputs'), default=str)}, "
                                 f"{af.get('status','')}, reason=\"{af.get('reason','')}\"")
                else:
                    lines.append("        after:  [absent]")

        ev = t["changed_blackboard_or_ros_evidence"]

        if ev.get("blackboard_changes"):
            lines.append("    blackboard_changes:")
            for key, chg in ev["blackboard_changes"].items():
                lines.append(f"      {key}: {json.dumps(chg['before'], default=str)} → "
                             f"{json.dumps(chg['after'], default=str)}")

        if ev.get("ros_in_changes"):
            lines.append("    ros_in_changes:")
            for src, chg in ev["ros_in_changes"].items():
                lines.append(
                    f"      {src}: present {chg['before_present']}→{chg['after_present']}, "
                    f"count {chg['before_count']}→{chg['after_count']}"
                )

        if ev.get("input_state_changes"):
            lines.append("    input_state_changes:")
            for adp, chg in ev["input_state_changes"].items():
                lines.append(f"      {adp}:")
                lines.append(f"        before: {json.dumps(chg['before_bb_writes'], default=str)}")
                lines.append(f"        after:  {json.dumps(chg['after_bb_writes'], default=str)}")

        if ev.get("ros_out_changes"):
            lines.append("    ros_out_changes:")
            for src, chg in ev["ros_out_changes"].items():
                before_str = json.dumps(chg["before"], default=str) if chg["before"] else "[absent]"
                after_str  = json.dumps(chg["after"],  default=str) if chg["after"]  else "[absent]"
                lines.append(f"      {src}:")
                lines.append(f"        before: {before_str}")
                lines.append(f"        after:  {after_str}")

        lines.append(
            f"    evidence_chain: see representative_tick_snapshots for {t['representative_tick_ids']}"
        )

    return "\n".join(lines)


def _fmt_stable_segment_evidence(bundle: dict) -> str:
    segments = bundle["segments"]
    drift_summaries = bundle["segment_drift_summaries"]
    stable = [s for s in segments if s["type"] == "stable"]

    if not stable:
        return "--- STABLE SEGMENT EVIDENCE AND DRIFT ---\n(no stable segments)"

    lines = ["--- STABLE SEGMENT EVIDENCE AND DRIFT ---"]
    for seg in stable:
        idx = seg["index"]
        tids = seg["tick_ids"]
        rep = [t for t in bundle["representative_tick_ids"] if t in set(tids)]

        lines.append(f"\n  segment [{idx}] (ticks {tids[0]}–{tids[-1]}, {len(tids)} ticks):")
        lines.append(f"    representative ticks: {rep}")
        if len(tids) > 2:
            lines.append(
                f"    reason: {len(tids)}-tick stable segment compressed to first two + last two"
            )

        lines.append("    repeated execution signature:")
        for line in seg["signature_digest"].splitlines():
            lines.append(f"      {line}")

        if idx in drift_summaries:
            ds = drift_summaries[idx]
            lines.append(f"    supporting evidence drift: {ds['drift_label']}")

            # decision_input_changes
            changed_dec = {k: v for k, v in ds["decision_input_changes"].items() if v["changed"]}
            if changed_dec:
                lines.append("      decision_input_changes:")
                for node, v in changed_dec.items():
                    lines.append(
                        f"        {node}: first={json.dumps(v['first'], default=str)}, "
                        f"last={json.dumps(v['last'], default=str)}"
                    )

            # blackboard_value_changes
            changed_bb = {k: v for k, v in ds["blackboard_value_changes"].items() if v["changed"]}
            if changed_bb:
                lines.append("      blackboard_value_changes:")
                for key, v in changed_bb.items():
                    lines.append(
                        f"        {key}: {json.dumps(v['first'], default=str)} → "
                        f"{json.dumps(v['last'], default=str)}"
                    )

            # ros_in_changes
            if ds["ros_in_changes"]:
                lines.append("      ros_in_changes:")
                for src, v in ds["ros_in_changes"].items():
                    lines.append(
                        f"        {src}: always_present={v['always_present']}, "
                        f"received_count_range={v['received_count_range']}"
                    )

            # input_state_changes
            changed_is = {k: v for k, v in ds["input_state_changes"].items() if v["bb_writes_changed"]}
            if changed_is:
                lines.append("      input_state_changes:")
                for adp, v in changed_is.items():
                    lines.append(f"        {adp}: first={json.dumps(v['first'], default=str)}, "
                                 f"last={json.dumps(v['last'], default=str)}")

            # ros_out_summary
            if ds["ros_out_summary"]:
                lines.append("      ros_out_summary:")
                for src, v in ds["ros_out_summary"].items():
                    lines.append(
                        f"        {src}: target={v['target']}, count={v['count']}, "
                        f"payload_varied={v['payload_varied']}"
                    )

    return "\n".join(lines)


def _fmt_representative_snapshots(bundle: dict) -> str:
    rep_ids = bundle["representative_tick_ids"]
    snapshots = bundle["snapshots"]
    segments = bundle["segments"]
    transitions = bundle["transitions"]

    if not rep_ids:
        return "--- REPRESENTATIVE TICK SNAPSHOTS ---\n(none)"

    # Build reason labels
    reason_map: dict = {}
    for t in transitions:
        reason_map[t["from_tick"]] = "transition: from_tick"
        reason_map[t["to_tick"]] = "transition: to_tick"
        if t["confirmation_tick"]:
            reason_map[t["confirmation_tick"]] = "transition: confirmation_tick"
    for seg in segments:
        tids = seg["tick_ids"]
        if len(tids) >= 3:
            for t in tids[:2]:
                reason_map.setdefault(t, f"segment [{seg['index']}] first two")
            for t in tids[-2:]:
                reason_map.setdefault(t, f"segment [{seg['index']}] last two")
        else:
            for t in tids:
                reason_map.setdefault(t, f"segment [{seg['index']}] all")

    lines = ["--- REPRESENTATIVE TICK SNAPSHOTS ---"]
    for tid in sorted(rep_ids):
        if tid not in snapshots:
            lines.append(f"\n[tick {tid} not in snapshots]")
            continue
        label = reason_map.get(tid, "representative")
        lines.append("")
        lines.append(_fmt_snapshot_block(snapshots[tid], label=label))

    return "\n".join(lines)


def _fmt_observation_slot(user_observation: str) -> str:
    if not user_observation.strip():
        return (
            "--- OBSERVATION SLOT ---\n"
            "This is log-only analysis. No user_observation was provided.\n"
            "If you need further diagnosis, describe what you physically observed:\n"
            "  - body: walking / stopped / turning / falling\n"
            "  - head: still / sweeping / wrong direction\n"
            "  - visible scene: line / ball / goal visible or not\n"
            "  - optional: rqt/pytreeview node states and blackboard values\n"
            "Only request this context if it would change the diagnosis from log evidence alone."
        )
    return (
        "--- OBSERVATION SLOT ---\n"
        f"user_observation:\n  {user_observation.strip()}\n\n"
        "Compare observation against BT, blackboard, and ROS evidence:\n"
        "  matches:          observation agrees with log evidence\n"
        "  mismatches:       observation contradicts log evidence\n"
        "  missing_evidence: observation mentions something not in logs"
    )


def _format_scaffold(bundle: dict, user_observation: str) -> str:
    """Assemble the full cross-tick analysis scaffold from CrossTickBundle."""
    parts = [
        "=== CROSS_TICK_ANALYSIS PACKAGE ===",
        f"source_mode: {bundle['source_mode']}",
        "",
        _fmt_selected_ticks(bundle),
        "",
        _JSONL_SCHEMA_LEGEND,
        "",
        _fmt_segment_digest(bundle),
        "",
        _fmt_transition_evidence(bundle),
        "",
        _fmt_stable_segment_evidence(bundle),
        "",
        _fmt_representative_snapshots(bundle),
        "",
        _fmt_observation_slot(user_observation),
        "",
        _ANALYSIS_INSTRUCTIONS,
    ]
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# LLM analysis instructions
# ---------------------------------------------------------------------------

_ANALYSIS_INSTRUCTIONS = """\
--- ANALYSIS INSTRUCTIONS ---

RULES:
1. tick_id is the only time unit. Do not expose timestamps, time_start, time_end, or duration.
2. Use readable signature digests. Do not output Python frozenset or tuple repr.
3. Compress stable segments >= 3 ticks to first two + last two representative ticks.
   Explain why intermediate ticks were omitted.
4. Cite tick_ids concretely when comparing evidence.
5. Keep logged ROS command evidence separate from user-provided runtime observation
   when comparing behavior.
6. "Stable BT state with changing supporting evidence" is a distinct finding.
   Report it explicitly when has_drift=True.
7. With user_observation: compare against log evidence (matches, mismatches, missing).
   Without user_observation: produce complete log-only analysis.
   Ask for runtime context only if it would change the diagnosis.

TASK — write the following sections using the data above:

explain_cross_ticks:
  - selected tick ids
  - whether BT execution changed or repeated
  - stable segments or transition points
  - important ROS/blackboard changes related to BT behavior

evidence_chain:
  For each BT transition or stable segment:
  - tick ids involved
  - ros_in / input_state evidence
  - blackboard evidence
  - decision evidence
  - BT node status evidence
  - ros_out consequence evidence

representative_ticks:
  For stable repeated segments >= 3 ticks:
  - show first two ticks and last two ticks
  - explain why intermediate ticks were compressed

transition_details:
  For BT changes (use transition_evidence data above):
  - from_tick / to_tick / confirmation_tick
  - changed BT nodes and decisions
  - changed blackboard and ROS evidence
  - likely interpretation from logs

observation_slot:
  (follow observation_slot instructions above)

compare_followup:
  Offer further comparison only when useful:
  - tick A vs tick B
  - observed behavior vs log evidence
  - UI/rqt state vs bt_debug_recent.jsonl
  - expected behavior vs decision inputs/reasons\
"""


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
    """
    bundle = get_raw_cross_tick_bundle(tick_selection)
    if not bundle["ok"]:
        return f"cross_tick_analysis error: {bundle['error']}"
    return _format_scaffold(bundle, user_observation)
