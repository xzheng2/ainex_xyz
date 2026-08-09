#!/usr/bin/env python3
"""Deterministic multi-tick retrieval and analysis from BT observability files.

Default source is the rolling recent files (bt_debug_recent.jsonl +
bt_ros_comm_debug_recent.jsonl, ~30 ticks) for live/paused sessions. Pass
source="lastrun" to analyze a full completed run (bt_debug_lastrun.jsonl +
bt_ros_comm_debug_lastrun.jsonl) and detail="digest" to keep the return bounded.
log_dir lets the same reducer run on the host against a project log dir.

Pipeline (all deterministic, no LLM):
  1. parse_tick_selection
  2. build_per_tick_snapshot per tick
  3. build_execution_signature per tick
  4. detect_segments (contiguous groups with same execution_signature)
  5. detect_transitions between adjacent segments
  6. summarize_segment_drift per stable segment
  7. select_representative_tick_ids
  8. return CrossTickBundle
"""
import json

# Relative import so this subpackage stays importable on its own (host CLI) without
# triggering xyz_agent_tools/__init__.py, which eagerly imports langchain-based tools.
from .raw_tick import (
    BT_OBS_DIR,
    BT_RECENT_BT,
    BT_RECENT_COMM,
    BT_LASTRUN_BT,
    BT_LASTRUN_COMM,
    _path,
    _file_status,
    _read_jsonl_raw,
    _detect_order,
    _group_by_tick,
)

# ---------------------------------------------------------------------------
# Step 1: parse_tick_selection
# ---------------------------------------------------------------------------

def parse_tick_selection(selection: str, all_tick_ids: list) -> tuple:
    """Parse tick_selection string into (selected_tick_ids, explicit_tick_ids).

    Formats:
      "all"        → all tick_ids; explicit=[]
      "latest:N"   → last N tick_ids by value; explicit=[]
      "A-B"        → tick_ids in [A, B] inclusive; explicit=[]
      "A,B,C"      → explicit list; explicit=[A,B,C]

    Returns (selected_tick_ids, explicit_tick_ids), both sorted ascending.
    Missing tick_ids are silently skipped (caller should add warnings).
    """
    sel = selection.strip()
    explicit: list = []

    if sel.lower() == "all" or not sel:
        return sorted(all_tick_ids), []

    if sel.lower().startswith("latest:"):
        try:
            n = int(sel[7:])
        except ValueError:
            return sorted(all_tick_ids), []
        return sorted(all_tick_ids)[-n:], []

    if "," in sel:
        try:
            ids = [int(x.strip()) for x in sel.split(",")]
        except ValueError:
            return sorted(all_tick_ids), []
        explicit = [i for i in ids if i in set(all_tick_ids)]
        return sorted(explicit), sorted(explicit)

    if "-" in sel:
        parts = sel.split("-", 1)
        try:
            a, b = int(parts[0].strip()), int(parts[1].strip())
        except ValueError:
            return sorted(all_tick_ids), []
        found = [i for i in all_tick_ids if a <= i <= b]
        return sorted(found), []

    # Single integer
    try:
        i = int(sel)
    except ValueError:
        return sorted(all_tick_ids), []
    found = [i] if i in set(all_tick_ids) else []
    return sorted(found), sorted(found)


# ---------------------------------------------------------------------------
# Step 2: build_per_tick_snapshot
# ---------------------------------------------------------------------------

def build_per_tick_snapshot(tick_id: int, bt_entries: list, comm_entries: list) -> dict:
    """Build a deterministic PerTickSnapshot from raw JSONL entry lists for one tick.

    bt_entries:   parsed entries from bt_debug_recent.jsonl for this tick_id.
    comm_entries: parsed entries from bt_ros_comm_debug_recent.jsonl for this tick_id.

    Returns snapshot dict per design spec Section 5.1.
    """
    snap = {
        "tick_id": tick_id,
        "bt": {
            "root_status": None,
            "node_statuses": {},
        },
        "decisions": {},
        "blackboard": {},
        "ros_in": {},
        "input_state": {},
        "ros_out": [],
    }

    for e in bt_entries:
        p = e.get("parsed")
        if not p:
            continue
        ev = p.get("event")
        if ev == "tree_tick_end":
            snap["bt"]["root_status"] = p.get("status")
        elif ev == "tick_end":
            node = p.get("node")
            if node:
                snap["bt"]["node_statuses"][node] = {
                    "type": p.get("type", ""),
                    "status": p.get("status", ""),
                }
        elif ev == "decision":
            node = p.get("node")
            if node:
                snap["decisions"][node] = {
                    "inputs": p.get("inputs", {}),
                    "status": p.get("status", ""),
                    "reason": p.get("reason", ""),
                }
        elif ev == "bb_write":
            key = p.get("key")
            if key:
                snap["blackboard"][key] = {
                    "writer": p.get("writer", ""),
                    "value": p.get("value"),
                }

    for e in comm_entries:
        p = e.get("parsed")
        if not p:
            continue
        ev = p.get("event")
        if ev == "ros_in":
            source = p.get("source")
            if source:
                snap["ros_in"][source] = {
                    "adapter": p.get("adapter", ""),
                    "received_count": p.get("received_count", 0),
                }
        elif ev == "input_state":
            adapter = p.get("adapter")
            if adapter:
                snap["input_state"][adapter] = {
                    "bb_writes": p.get("bb_writes", {}),
                }
        elif ev == "ros_out":
            snap["ros_out"].append({
                "bt_node": p.get("bt_node", p.get("node", "")),
                "semantic_source": p.get("semantic_source", ""),
                "target": p.get("target", ""),
                "comm_type": p.get("comm_type", ""),
                "payload": p.get("payload"),
                "attribution_confidence": p.get("attribution_confidence", ""),
            })

    return snap


# ---------------------------------------------------------------------------
# Step 3: execution_signature and digest
# ---------------------------------------------------------------------------

def build_execution_signature(snapshot: dict) -> tuple:
    """Build a hashable execution_signature from a snapshot.

    Returns (frozenset_tick_end, frozenset_decisions).
    Two ticks have the same signature when all node statuses and decision
    statuses+reasons match.
    """
    tick_end_sig = frozenset(
        (name, info["type"], info["status"])
        for name, info in snapshot["bt"]["node_statuses"].items()
    )
    decision_sig = frozenset(
        (name, d["status"], d["reason"])
        for name, d in snapshot["decisions"].items()
    )
    return (tick_end_sig, decision_sig)


def build_signature_digest(snapshot: dict) -> str:
    """Build a human-readable signature digest (never Python frozenset/tuple repr)."""
    lines = ["BT node statuses:"]
    for name in sorted(snapshot["bt"]["node_statuses"]):
        status = snapshot["bt"]["node_statuses"][name]["status"]
        # Strip "Status." prefix for readability
        short = status.replace("Status.", "") if isinstance(status, str) else str(status)
        lines.append(f"  {name}: {short}")
    if snapshot["decisions"]:
        lines.append("Decisions:")
        for name in sorted(snapshot["decisions"]):
            d = snapshot["decisions"][name]
            status = d["status"].replace("Status.", "") if isinstance(d["status"], str) else str(d["status"])
            reason = d["reason"]
            lines.append(f'  {name}: {status}, reason="{reason}"')
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Step 4: detect_segments
# ---------------------------------------------------------------------------

def detect_segments(snapshots_ordered: list, signatures: dict) -> list:
    """Walk snapshots sorted ascending by tick_id and group same-signature runs.

    Args:
        snapshots_ordered: list of snapshot dicts sorted ascending by tick_id.
        signatures: {tick_id: execution_signature tuple}

    Returns list of Segment dicts:
        {
            "index": int,
            "type": "stable" | "single",
            "tick_ids": [int, ...],
            "execution_signature": tuple,
            "signature_digest": str,
        }
    """
    if not snapshots_ordered:
        return []

    segments = []
    current_sig = signatures[snapshots_ordered[0]["tick_id"]]
    current_ticks = [snapshots_ordered[0]["tick_id"]]
    current_snap = snapshots_ordered[0]

    for snap in snapshots_ordered[1:]:
        tid = snap["tick_id"]
        sig = signatures[tid]
        if sig == current_sig:
            current_ticks.append(tid)
        else:
            segments.append({
                "index": len(segments),
                "type": "stable" if len(current_ticks) >= 2 else "single",
                "tick_ids": list(current_ticks),
                "execution_signature": current_sig,
                "signature_digest": build_signature_digest(current_snap),
            })
            current_sig = sig
            current_ticks = [tid]
            current_snap = snap

    # Final segment
    segments.append({
        "index": len(segments),
        "type": "stable" if len(current_ticks) >= 2 else "single",
        "tick_ids": list(current_ticks),
        "execution_signature": current_sig,
        "signature_digest": build_signature_digest(current_snap),
    })

    return segments


# ---------------------------------------------------------------------------
# Step 5: detect_transitions
# ---------------------------------------------------------------------------

def _diff_dict_values(before: dict, after: dict) -> dict:
    """Return keys where values differ between before and after dicts.

    Includes keys present in either but not both.
    """
    all_keys = set(before) | set(after)
    diff = {}
    for k in all_keys:
        bv = before.get(k)
        av = after.get(k)
        if bv != av:
            diff[k] = {"before": bv, "after": av}
    return diff


def detect_transitions(segments: list, snapshots: dict) -> list:
    """Build a TransitionRecord for each adjacent segment pair with different signatures.

    Args:
        segments: list of Segment dicts (from detect_segments).
        snapshots: {tick_id: snapshot_dict}

    Returns list of TransitionRecord dicts.
    """
    transitions = []
    for i in range(len(segments) - 1):
        seg_a = segments[i]
        seg_b = segments[i + 1]
        if seg_a["execution_signature"] == seg_b["execution_signature"]:
            continue  # adjacent segments with same sig — shouldn't happen but guard it

        from_tick = seg_a["tick_ids"][-1]
        to_tick = seg_b["tick_ids"][0]
        conf_tick = seg_b["tick_ids"][1] if len(seg_b["tick_ids"]) >= 2 else None

        snap_from = snapshots[from_tick]
        snap_to = snapshots[to_tick]

        # changed_bt_nodes: compare node_statuses
        changed_bt = {}
        all_nodes = set(snap_from["bt"]["node_statuses"]) | set(snap_to["bt"]["node_statuses"])
        for node in sorted(all_nodes):
            before_status = snap_from["bt"]["node_statuses"].get(node, {}).get("status", "[absent]")
            after_status  = snap_to["bt"]["node_statuses"].get(node, {}).get("status", "[absent]")
            if before_status != after_status:
                changed_bt[node] = {"before": before_status, "after": after_status}

        # changed_decisions: compare decisions
        changed_dec = {}
        all_dec = set(snap_from["decisions"]) | set(snap_to["decisions"])
        for node in sorted(all_dec):
            bd = snap_from["decisions"].get(node)
            ad = snap_to["decisions"].get(node)
            if bd != ad:
                changed_dec[node] = {"before": bd, "after": ad}

        # changed_blackboard_or_ros_evidence
        # blackboard_changes
        bb_changes = _diff_dict_values(
            {k: v["value"] for k, v in snap_from["blackboard"].items()},
            {k: v["value"] for k, v in snap_to["blackboard"].items()},
        )

        # ros_in_changes
        all_sources = set(snap_from["ros_in"]) | set(snap_to["ros_in"])
        ros_in_changes = {}
        for src in sorted(all_sources):
            bf = snap_from["ros_in"].get(src)
            af = snap_to["ros_in"].get(src)
            bp = bf is not None
            ap = af is not None
            bc = bf["received_count"] if bf else None
            ac = af["received_count"] if af else None
            if bp != ap or bc != ac:
                ros_in_changes[src] = {
                    "before_present": bp, "after_present": ap,
                    "before_count": bc, "after_count": ac,
                }

        # input_state_changes
        all_adapters = set(snap_from["input_state"]) | set(snap_to["input_state"])
        input_state_changes = {}
        for adp in sorted(all_adapters):
            bw = snap_from["input_state"].get(adp, {}).get("bb_writes")
            aw = snap_to["input_state"].get(adp, {}).get("bb_writes")
            if bw != aw:
                input_state_changes[adp] = {"before_bb_writes": bw, "after_bb_writes": aw}

        # ros_out_changes: index by semantic_source (first occurrence per source)
        def _ros_out_by_source(ros_out_list):
            idx = {}
            for r in ros_out_list:
                src = r.get("semantic_source", "")
                if src and src not in idx:
                    idx[src] = {
                        "target": r.get("target"),
                        "comm_type": r.get("comm_type"),
                        "payload": r.get("payload"),
                    }
            return idx

        from_ros = _ros_out_by_source(snap_from["ros_out"])
        to_ros   = _ros_out_by_source(snap_to["ros_out"])
        all_srcs = set(from_ros) | set(to_ros)
        ros_out_changes = {}
        for src in sorted(all_srcs):
            bf = from_ros.get(src)
            af = to_ros.get(src)
            if bf != af:
                ros_out_changes[src] = {"before": bf, "after": af}

        rec = {
            "from_segment_index": seg_a["index"],
            "to_segment_index": seg_b["index"],
            "from_tick": from_tick,
            "to_tick": to_tick,
            "confirmation_tick": conf_tick,
            "changed_bt_nodes": changed_bt,
            "changed_decisions": changed_dec,
            "changed_blackboard_or_ros_evidence": {
                "blackboard_changes": bb_changes,
                "ros_in_changes": ros_in_changes,
                "input_state_changes": input_state_changes,
                "ros_out_changes": ros_out_changes,
            },
            "representative_tick_ids": (
                [from_tick, to_tick, conf_tick] if conf_tick is not None
                else [from_tick, to_tick]
            ),
        }
        transitions.append(rec)

    return transitions


# ---------------------------------------------------------------------------
# Step 6: summarize_segment_drift
# ---------------------------------------------------------------------------

def summarize_segment_drift(segment: dict, snapshots: dict) -> dict:
    """Summarize supporting evidence drift for a stable segment.

    Walks all ticks in the segment and compares first vs last tick for each
    evidence type. Does not expand every tick.
    """
    tick_ids = segment["tick_ids"]
    if not tick_ids:
        return {"segment_index": segment["index"], "has_drift": False, "drift_label": "no ticks"}

    first_snap = snapshots[tick_ids[0]]
    last_snap  = snapshots[tick_ids[-1]]

    # decision_input_changes: compare inputs first vs last per decision node
    decision_input_changes = {}
    all_dec = set(first_snap["decisions"]) | set(last_snap["decisions"])
    for node in sorted(all_dec):
        fi = first_snap["decisions"].get(node, {}).get("inputs")
        li = last_snap["decisions"].get(node, {}).get("inputs")
        decision_input_changes[node] = {"first": fi, "last": li, "changed": fi != li}

    # blackboard_value_changes: compare bb value first vs last per key
    blackboard_value_changes = {}
    all_keys = set(first_snap["blackboard"]) | set(last_snap["blackboard"])
    for key in sorted(all_keys):
        fv = first_snap["blackboard"].get(key, {}).get("value")
        lv = last_snap["blackboard"].get(key, {}).get("value")
        blackboard_value_changes[key] = {"first": fv, "last": lv, "changed": fv != lv}

    # ros_in_changes: track presence and received_count range across ALL ticks
    ros_in_changes: dict = {}
    for tid in tick_ids:
        for src, info in snapshots[tid]["ros_in"].items():
            entry = ros_in_changes.setdefault(src, {"present_count": 0, "counts": []})
            entry["present_count"] += 1
            entry["counts"].append(info["received_count"])
    total_ticks = len(tick_ids)
    ros_in_summary = {}
    for src, entry in ros_in_changes.items():
        ros_in_summary[src] = {
            "always_present": entry["present_count"] == total_ticks,
            "received_count_range": [min(entry["counts"]), max(entry["counts"])],
        }

    # input_state_changes: compare adapter bb_writes first vs last
    input_state_changes = {}
    all_adp = set(first_snap["input_state"]) | set(last_snap["input_state"])
    for adp in sorted(all_adp):
        fw = first_snap["input_state"].get(adp, {}).get("bb_writes")
        lw = last_snap["input_state"].get(adp, {}).get("bb_writes")
        input_state_changes[adp] = {
            "bb_writes_changed": fw != lw,
            "first": fw,
            "last": lw,
        }

    # ros_out_summary: per semantic_source, count emissions and check payload variation
    ros_out_counts: dict = {}  # semantic_source → {target, count, payloads}
    for tid in tick_ids:
        for r in snapshots[tid]["ros_out"]:
            src = r.get("semantic_source", "")
            entry = ros_out_counts.setdefault(src, {
                "target": r.get("target", ""),
                "comm_type": r.get("comm_type", ""),
                "count": 0,
                "payloads": [],
            })
            entry["count"] += 1
            entry["payloads"].append(r.get("payload"))
    ros_out_summary = {}
    for src, entry in ros_out_counts.items():
        payloads = entry["payloads"]
        ros_out_summary[src] = {
            "target": entry["target"],
            "comm_type": entry["comm_type"],
            "count": entry["count"],
            "payload_varied": len(set(
                json.dumps(p, sort_keys=True, default=str) for p in payloads
            )) > 1,
        }

    # Determine has_drift
    has_drift = (
        any(v["changed"] for v in decision_input_changes.values()) or
        any(v["changed"] for v in blackboard_value_changes.values()) or
        any(v["received_count_range"][0] != v["received_count_range"][1]
            for v in ros_in_summary.values()) or
        any(v["bb_writes_changed"] for v in input_state_changes.values()) or
        any(v["payload_varied"] for v in ros_out_summary.values())
    )

    drift_label = (
        "stable BT state with changing supporting evidence"
        if has_drift
        else "stable BT state with stable supporting evidence"
    )

    return {
        "segment_index": segment["index"],
        "tick_range": [tick_ids[0], tick_ids[-1]],
        "decision_input_changes": decision_input_changes,
        "blackboard_value_changes": blackboard_value_changes,
        "ros_in_changes": ros_in_summary,
        "input_state_changes": input_state_changes,
        "ros_out_summary": ros_out_summary,
        "has_drift": has_drift,
        "drift_label": drift_label,
    }


# ---------------------------------------------------------------------------
# Step 7: select_representative_tick_ids
# ---------------------------------------------------------------------------

def select_representative_tick_ids(
    segments: list,
    transitions: list,
    explicit_tick_ids: list,
) -> list:
    """Centralized representative tick selection.

    Rules (applied as a union; result is sorted + deduplicated):
      - For each transition: include from_tick, to_tick, confirmation_tick.
      - For each stable segment (length >= 3): include first two + last two.
      - For stable segment length 1–2 or single segment: include all ticks.
      - Explicit user tick_ids: always include.
    """
    chosen: set = set()

    # Transitions
    for t in transitions:
        chosen.add(t["from_tick"])
        chosen.add(t["to_tick"])
        if t["confirmation_tick"] is not None:
            chosen.add(t["confirmation_tick"])

    # Segments
    for seg in segments:
        tids = seg["tick_ids"]
        if len(tids) >= 3 and seg["type"] == "stable":
            chosen.update(tids[:2])   # first two
            chosen.update(tids[-2:])  # last two
        else:
            chosen.update(tids)       # all (covers single + short stable + single-type)

    # Explicit
    chosen.update(explicit_tick_ids)

    return sorted(chosen)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def get_raw_cross_tick_bundle(tick_selection: str = "all", source: str = "recent",
                              log_dir: str = None, detail: str = "full") -> dict:
    """Execute the full multi-tick pipeline and return a CrossTickBundle dict.

    Args:
        tick_selection: "all" | "latest:N" | "A-B" | "A,B,C"
        source:         "recent" (default, rolling ~30-tick files — live/paused sessions)
                        or "lastrun" (full-session files — post-mortem of a completed run).
        log_dir:        directory holding the JSONL files. None → the rosa-agent container
                        mount (/opt/ainex_bt_log). The host CLI passes a project log dir
                        (e.g. xyz_behavior/<proj>/log) so the same reducer runs on the host.
        detail:         "full" (default) keeps every selected snapshot in the bundle;
                        "digest" trims ["snapshots"] to representative ticks only so the
                        return stays bounded for large (~3000-tick) lastrun sessions.

    Returns CrossTickBundle dict. Always returns a dict; never raises.
    Check ["ok"] first.
    """
    warnings: list = []

    # ── Resolve source tier + file location ──────────────────────────────────
    source = (source or "recent").lower()
    if source == "lastrun":
        bt_file, comm_file = BT_LASTRUN_BT, BT_LASTRUN_COMM
    else:
        source = "recent"
        bt_file, comm_file = BT_RECENT_BT, BT_RECENT_COMM
    base_dir = log_dir or BT_OBS_DIR

    def P(fn: str) -> str:
        return _path(fn, base_dir)

    # ── File status ──────────────────────────────────────────────────────────
    bt_status   = _file_status(bt_file, base_dir)
    comm_status = _file_status(comm_file, base_dir)

    stale_warning = None
    if not bt_status["exists"]:
        return {
            "ok": False,
            "error": (
                f"{bt_file} not found at {P(bt_file)}. "
                + ("Ensure the BT node has run at least once in this session."
                   if source == "recent"
                   else "Ensure a completed run wrote a lastrun file in this log dir.")
            ),
            "source_mode": source,
            "files": {"bt": bt_file, "comm": comm_file},
            "recent_status": {
                "bt_age_seconds": None, "comm_age_seconds": None, "stale_warning": None,
            },
            "order_detected": {"bt": "unknown", "comm": "unknown"},
            "all_tick_ids": [],
            "selected_tick_ids": [],
            "explicit_tick_ids": [],
            "snapshots": {},
            "segments": [],
            "transitions": [],
            "segment_drift_summaries": {},
            "representative_tick_ids": [],
            "warnings": [],
        }

    # Staleness only matters for live recent files; a completed lastrun file is
    # always old by design, so don't warn about it.
    if source == "recent" and bt_status["stale"]:
        stale_warning = (
            f"bt file stale ({bt_status['age_seconds']}s since last write) — "
            "BT node may be paused or stopped."
        )
        warnings.append(stale_warning)

    if source == "recent" and comm_status["exists"] and comm_status["stale"]:
        warnings.append(f"comm file stale ({comm_status['age_seconds']}s since last write).")

    # ── Read entries ─────────────────────────────────────────────────────────
    bt_entries   = _read_jsonl_raw(P(bt_file))
    comm_entries = _read_jsonl_raw(P(comm_file)) if comm_status["exists"] else []

    if bt_entries and bt_entries[0].get("error"):
        return {
            "ok": False,
            "error": bt_entries[0]["error"],
            "source_mode": source,
            "files": {"bt": bt_file, "comm": comm_file},
            "recent_status": {
                "bt_age_seconds": bt_status["age_seconds"],
                "comm_age_seconds": comm_status["age_seconds"],
                "stale_warning": stale_warning,
            },
            "order_detected": {"bt": "unknown", "comm": "unknown"},
            "all_tick_ids": [],
            "selected_tick_ids": [],
            "explicit_tick_ids": [],
            "snapshots": {},
            "segments": [],
            "transitions": [],
            "segment_drift_summaries": {},
            "representative_tick_ids": [],
            "warnings": warnings,
        }

    bt_order   = _detect_order(bt_entries)
    comm_order = _detect_order(comm_entries) if comm_entries else "unknown"

    # Normalize within-tick line order to chronological (oldest-first). A clean-shutdown
    # lastrun file is newest-first on disk; reverse so each tick's lines read in the order
    # they were emitted. (Cross-tick ordering is already guaranteed by the ascending sort
    # of selected_tick_ids below, but per-tick last-write-wins fields need this.)
    if bt_order == "newest-first":
        bt_entries = list(reversed(bt_entries))
    if comm_order == "newest-first":
        comm_entries = list(reversed(comm_entries))

    bt_grouped   = _group_by_tick(bt_entries)
    comm_grouped = _group_by_tick(comm_entries)

    all_tick_ids = sorted(k for k in bt_grouped if k >= 0)

    if not all_tick_ids:
        return {
            "ok": False,
            "error": f"No valid tick_ids found in {bt_file}.",
            "source_mode": source,
            "files": {"bt": bt_file, "comm": comm_file},
            "recent_status": {
                "bt_age_seconds": bt_status["age_seconds"],
                "comm_age_seconds": comm_status["age_seconds"],
                "stale_warning": stale_warning,
            },
            "order_detected": {"bt": bt_order, "comm": comm_order},
            "all_tick_ids": [],
            "selected_tick_ids": [],
            "explicit_tick_ids": [],
            "snapshots": {},
            "segments": [],
            "transitions": [],
            "segment_drift_summaries": {},
            "representative_tick_ids": [],
            "warnings": warnings,
        }

    # ── Step 1: parse tick_selection ─────────────────────────────────────────
    selected_tick_ids, explicit_tick_ids = parse_tick_selection(tick_selection, all_tick_ids)

    if not selected_tick_ids:
        warnings.append(
            f"tick_selection={tick_selection!r} matched no tick_ids. "
            f"Available: {all_tick_ids}"
        )
        selected_tick_ids = all_tick_ids
        explicit_tick_ids = []

    # ── Step 2: build per-tick snapshots ─────────────────────────────────────
    snapshots: dict = {}
    for tid in selected_tick_ids:
        bt_ent   = bt_grouped.get(tid, [])
        comm_ent = comm_grouped.get(tid, [])
        snapshots[tid] = build_per_tick_snapshot(tid, bt_ent, comm_ent)

    # ── Step 3: build execution_signatures ──────────────────────────────────
    signatures = {tid: build_execution_signature(snapshots[tid]) for tid in selected_tick_ids}

    # ── Step 4: detect contiguous segments ──────────────────────────────────
    snapshots_ordered = [snapshots[tid] for tid in selected_tick_ids]
    segments = detect_segments(snapshots_ordered, signatures)

    # ── Step 5: detect transitions ───────────────────────────────────────────
    transitions = detect_transitions(segments, snapshots)

    # ── Step 6: segment drift summaries (stable segments only) ───────────────
    segment_drift_summaries: dict = {}
    for seg in segments:
        if seg["type"] == "stable":
            segment_drift_summaries[seg["index"]] = summarize_segment_drift(seg, snapshots)

    # ── Step 7: representative tick ids ─────────────────────────────────────
    representative_tick_ids = select_representative_tick_ids(
        segments, transitions, explicit_tick_ids
    )

    # ── Digest trim: bound the return for large lastrun sessions ─────────────
    # In "digest" mode keep only the representative snapshots; the coarse view
    # (segments + transitions + drift) is what callers print, so a ~3000-tick
    # session no longer carries ~3000 snapshot dicts in the bundle.
    if detail == "digest":
        rep_set = set(representative_tick_ids)
        snapshots = {tid: snap for tid, snap in snapshots.items() if tid in rep_set}

    return {
        "ok": True,
        "error": None,
        "source_mode": source,
        "detail": detail,
        "files": {"bt": bt_file, "comm": comm_file},
        "recent_status": {
            "bt_age_seconds":   bt_status["age_seconds"],
            "comm_age_seconds": comm_status["age_seconds"],
            "stale_warning":    stale_warning,
        },
        "order_detected": {"bt": bt_order, "comm": comm_order},
        "all_tick_ids":          all_tick_ids,
        "selected_tick_ids":     selected_tick_ids,
        "explicit_tick_ids":     explicit_tick_ids,
        "snapshots":             snapshots,
        "segments":              segments,
        "transitions":           transitions,
        "segment_drift_summaries": segment_drift_summaries,
        "representative_tick_ids": representative_tick_ids,
        "warnings":              warnings,
    }
