#!/usr/bin/env python3
"""Langchain-free formatting for cross-tick / session bundles.

Pure string rendering of a CrossTickBundle (from raw_cross_tick.get_raw_cross_tick_bundle).
Kept dependency-free on purpose so it can be imported BOTH inside the rosa-agent container
(by the LangChain tools) AND by host python3.11 (lastrun_digest_cli.py) — the host has no
langchain installed.

Two entry points:
  format_scaffold(bundle, user_observation)  — full report scaffold incl. representative
                                               per-tick snapshots (used by cross_tick_analysis).
  format_digest(bundle, user_observation)    — coarse whole-run view: segment timeline +
                                               transitions + stable-segment drift, NO per-tick
                                               snapshots (used by session_digest + CLI digest).
"""
import json

# ---------------------------------------------------------------------------
# JSONL Schema Legend — single source of truth (also imported by bt_tick_analysis)
# ---------------------------------------------------------------------------

_JSONL_SCHEMA_LEGEND = """\
[BT DECISION LAYER — bt_debug_*.jsonl]
  tree_tick_start : tick begins.
      tick_id=iteration counter, ts=timestamp

  tree_tick_end   : tick ends — overall tree result.
      tick_id, status=root node outcome (SUCCESS|FAILURE|RUNNING), ts

  tick_end        : one BT node was evaluated.
      node=instance name (e.g. "IsRobotStanding"),
      type=Python class (e.g. "L1_Balance_IsStanding" — prefix shows layer/category),
      status=that node's result (SUCCESS|FAILURE|RUNNING), ts

  decision        : a condition node logged its explicit reasoning.
      node=condition name,
      inputs=dict of blackboard keys the condition READ and their values (the evidence),
      status=its result,
      reason=plain-English explanation of WHY it returned SUCCESS or FAILURE, ts

  bb_write        : a blackboard key was written.
      writer=node or infra component that wrote it,
      key=full BB key path (e.g. "/latched/robot_state"),
      value=new value, ts

[ROS COMM LAYER — bt_ros_comm_debug_*.jsonl]
  ros_in          : sensor data received by an InputAdapter — emitted BEFORE tree_tick_start.
      source=ROS topic the adapter subscribes to,
      adapter=adapter class name,
      received_count=number of messages buffered since last snapshot (shows data rate)

  input_state     : what the InputAdapter wrote to the blackboard this tick — emitted BEFORE tree_tick_start.
      adapter=adapter class name,
      bb_writes=dict of BB keys written and their values (sensor data converted to robot semantics)

  ros_out         : outgoing ROS command emitted by comm_facade during tree execution.
      bt_node=which BT action node triggered this command (e.g. "FollowLine"),
      semantic_source=method/intent name describing what the command represents (e.g. "set_walking_speed"),
      target=ROS topic or service that received the command,
      comm_type=how it was sent: topic_publish or service_call,
      payload=full message/request content sent,
      summary=one-line human-readable description of what was sent,
      attribution_confidence=how certain the tracer is that bt_node actually caused this:
          high=directly traced, medium=inferred from context, low=uncertain

[INTRA-TICK ORDER BY TIMESTAMP]
  ros_in → input_state → tree_tick_start → tick_end/decision/bb_write → tree_tick_end → ros_out
  NOTE: ros_in + input_state always appear BEFORE tree_tick_start because InputAdapters
  snapshot and write sensor data in a two-phase latch BEFORE tree.tick() is called.\
"""


# ---------------------------------------------------------------------------
# Per-section formatters
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
    sel = bundle["selected_tick_ids"]
    # A whole-run selection is thousands of ticks; don't dump the full list.
    if len(sel) > 40:
        tick_line = f"tick_ids: {sel[0]}–{sel[-1]} ({len(sel)} ticks, contiguous list elided)"
    else:
        tick_line = f"tick_ids: {sel}"
    lines = [
        "--- SELECTED TICKS ---",
        tick_line,
        f"total: {len(sel)}",
        f"source: {bundle['source_mode']}",
    ]
    if bundle.get("all_tick_ids"):
        atids = bundle["all_tick_ids"]
        lines.append(f"session span: ticks {atids[0]}–{atids[-1]} ({len(atids)} ticks total)")
    # File age only matters for live recent files; a completed lastrun file is always old.
    if bundle["source_mode"] == "recent" and rs["bt_age_seconds"] is not None:
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
  - UI/rqt state vs the source jsonl
  - expected behavior vs decision inputs/reasons\
"""

_DIGEST_INSTRUCTIONS = """\
--- WHOLE-RUN DIGEST INSTRUCTIONS ---

This is a COARSE, whole-session overview: the execution timeline (segments), the
transitions between distinct BT states, and supporting-evidence drift inside stable
segments. Per-tick raw snapshots are NOT included here to keep the view bounded.

Use it to answer "what happened across the whole run?" and to LOCATE where behavior
diverged. To inspect a suspect span in detail, drill into a specific tick range
(per-tick snapshots / raw evidence) rather than asking for the entire session at once.

RULES:
1. tick_id is the only time unit. Do not expose timestamps or durations.
2. Cite tick_ids concretely. Name the transition tick where behavior changed.
3. Report "stable BT state with changing supporting evidence" explicitly when drift is present.
4. Point to the tick range worth drilling into; do not fabricate per-tick detail not shown above.\
"""


# ---------------------------------------------------------------------------
# Public entry points
# ---------------------------------------------------------------------------

def _analyzed_tick_line(bundle: dict) -> str:
    """One greppable line naming the ticks analyzed — kept near the top so it survives
    the session logger's 2000-char observation truncation (used for later correlation)."""
    sel = bundle.get("selected_tick_ids") or []
    if not sel:
        return "analyzed_tick_ids: (none)"
    if len(sel) == 1:
        return f"analyzed_tick_ids: {sel[0]}"
    return f"analyzed_tick_ids: {sel[0]}-{sel[-1]} ({len(sel)} ticks)"


def format_scaffold(bundle: dict, user_observation: str = "") -> str:
    """Assemble the full cross-tick analysis scaffold (with representative snapshots)."""
    parts = [
        "=== CROSS_TICK_ANALYSIS PACKAGE ===",
        _analyzed_tick_line(bundle),
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


def _parse_signature(signature_digest: str):
    """Split a segment signature_digest into {node: status} and a list of decision lines."""
    statuses: dict = {}
    decisions: list = []
    mode = None
    for line in signature_digest.splitlines():
        s = line.strip()
        if s == "BT node statuses:":
            mode = "st"; continue
        if s == "Decisions:":
            mode = "dec"; continue
        if not s:
            continue
        if mode == "st" and ":" in s:
            node, st = s.rsplit(":", 1)
            statuses[node.strip()] = st.strip().replace("Status.", "")
        elif mode == "dec":
            decisions.append(s)
    return statuses, decisions


def _state_label(signature_digest: str) -> list:
    """Compact, human-readable label lines for one execution state (no full node dump)."""
    statuses, decisions = _parse_signature(signature_digest)
    running = [n for n, st in statuses.items() if st == "RUNNING"]
    succeeded = [n for n, st in statuses.items() if st == "SUCCESS"]
    lines = []
    if running:
        lines.append(f"active(RUNNING): {', '.join(sorted(running))}")
    if succeeded:
        lines.append(f"succeeded: {', '.join(sorted(succeeded))}")
    for d in decisions:
        lines.append(f"decision: {d}")
    return lines or ["(no signature)"]


def _fmt_state_profile(bundle: dict, top_n: int = 15) -> str:
    """Occupancy profile: aggregate ticks by execution state. Bounded by distinct states."""
    segments = bundle["segments"]
    if not segments:
        return "--- STATE OCCUPANCY PROFILE ---\n(no segments)"

    agg: dict = {}  # signature_digest -> {ticks, visits, first, last}
    order: list = []
    for seg in segments:
        sd = seg["signature_digest"]
        tids = seg["tick_ids"]
        if sd not in agg:
            agg[sd] = {"ticks": 0, "visits": 0, "first": tids[0], "last": tids[-1], "examples": []}
            order.append(sd)
        a = agg[sd]
        a["ticks"] += len(tids)
        a["visits"] += 1
        a["first"] = min(a["first"], tids[0])
        a["last"] = max(a["last"], tids[-1])
        if len(a["examples"]) < 3:
            a["examples"].append(f"{tids[0]}–{tids[-1]}" if len(tids) > 1 else f"{tids[0]}")

    ranked = sorted(order, key=lambda sd: agg[sd]["ticks"], reverse=True)
    total_ticks = sum(agg[sd]["ticks"] for sd in order)

    lines = [f"--- STATE OCCUPANCY PROFILE (distinct states: {len(order)}; top {min(top_n, len(order))} by ticks) ---"]
    for i, sd in enumerate(ranked[:top_n], 1):
        a = agg[sd]
        pct = (100.0 * a["ticks"] / total_ticks) if total_ticks else 0.0
        lines.append(
            f"\n[State {i}] {a['ticks']} ticks ({pct:.0f}%) across {a['visits']} visit(s); "
            f"span {a['first']}–{a['last']}; e.g. ticks {', '.join(a['examples'])}"
        )
        for lbl in _state_label(sd):
            lines.append(f"    {lbl}")
    if len(order) > top_n:
        rest_ticks = sum(agg[sd]["ticks"] for sd in ranked[top_n:])
        lines.append(f"\n  (+{len(order) - top_n} more states, {rest_ticks} ticks — drill in for detail)")
    return "\n".join(lines)


def _fmt_transition_histogram(bundle: dict, max_types: int = 25) -> str:
    """Transition-type histogram: group transitions by which BT nodes changed status.

    Bounded by the number of DISTINCT transition kinds, not by raw transition count — a
    ball that flickers in/out 100 times collapses to two rows (lost / reacquired)."""
    transitions = bundle["transitions"]
    if not transitions:
        return "--- TRANSITION TYPES ---\n(no BT transitions detected)"

    groups: dict = {}  # key -> {count, ticks, label}
    order: list = []
    for t in transitions:
        changed = t.get("changed_bt_nodes") or {}
        if changed:
            key = ("bt", tuple(sorted((n, chg["before"], chg["after"]) for n, chg in changed.items())))
            label = "; ".join(f"{n}: {b}→{a}" for (n, b, a) in key[1])
        else:
            # no BT status change → keyed by which decision nodes changed
            dec_nodes = tuple(sorted((t.get("changed_decisions") or {}).keys()))
            key = ("dec", dec_nodes)
            label = f"decision change: {', '.join(dec_nodes) or '(none)'}"
        if key not in groups:
            groups[key] = {"count": 0, "ticks": [], "label": label}
            order.append(key)
        g = groups[key]
        g["count"] += 1
        if len(g["ticks"]) < 6:
            g["ticks"].append(t["to_tick"])

    ranked = sorted(order, key=lambda k: groups[k]["count"], reverse=True)
    lines = [f"--- TRANSITION TYPES (distinct kinds: {len(order)}; total transitions: {len(transitions)}) ---"]
    for k in ranked[:max_types]:
        g = groups[k]
        more = "…" if g["count"] > len(g["ticks"]) else ""
        lines.append(f"\n  {g['count']}× {g['label']}")
        lines.append(f"      at to_tick: {g['ticks']}{more}")
    if len(order) > max_types:
        lines.append(f"\n  (+{len(order) - max_types} more transition kinds)")
    return "\n".join(lines)


def format_digest(bundle: dict, user_observation: str = "", top_n: int = 15) -> str:
    """Assemble the COARSE whole-run digest.

    Bounded for any session length: aggregates by execution state (occupancy profile)
    and by transition kind (histogram) instead of dumping per-tick/per-transition evidence.
    Drill into a tick range with detail="full" for the verbose scaffold."""
    parts = [
        "=== SESSION_DIGEST PACKAGE ===",
        _analyzed_tick_line(bundle),
        f"source_mode: {bundle['source_mode']}",
        "",
        _fmt_selected_ticks(bundle),
        "",
        _JSONL_SCHEMA_LEGEND,
        "",
        _fmt_state_profile(bundle, top_n=top_n),
        "",
        _fmt_transition_histogram(bundle),
        "",
        _fmt_observation_slot(user_observation),
        "",
        _DIGEST_INSTRUCTIONS,
    ]
    return "\n".join(parts)
