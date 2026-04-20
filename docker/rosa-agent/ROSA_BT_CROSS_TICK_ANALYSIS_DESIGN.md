# ROSA BT Cross-Tick Analysis Design

> **Status: IMPLEMENTED 2026-04-20. Prompts sharpened 2026-04-20.**
> References: `ROSA_BT_ONE_TICK_ANALYSIS_DESIGN.md` (implemented 2026-04-11).
> New files: `ainex_agent_tools/bt_analysis/raw_cross_tick.py`, `ainex_agent_tools/tools/cross_tick_analysis.py`.

---

## 1. Purpose

`cross_tick_analysis` analyzes recent BT and ROS communication debug logs across up to 30 ticks. It detects whether BT execution repeated or changed, identifies transitions between BT states, and summarizes supporting evidence drift within stable segments. The tool performs all grouping, comparison, and compression deterministically before returning a scaffold to the ROSA LLM.

---

## 2. Inputs

**`tick_selection`** (str, default `"all"`):
- `"all"` — all tick_ids in recent bt file
- `"latest:N"` — last N tick_ids by value
- `"A-B"` — tick_ids in range A to B inclusive
- `"A,B,C"` — explicit comma-separated tick_ids (always included as `explicit_tick_ids`)

**`user_observation`** (str, default `""`): optional physical or UI observation spanning multiple ticks. Log-only output is complete without it.

**Source files** (recent only, never lastrun):
- `bt_debug_recent.jsonl`
- `bt_ros_comm_debug_recent.jsonl`

---

## 3. Processing Pipeline

```
1. Parse tick_selection
     → selected_tick_ids, explicit_tick_ids

2. Build per-tick snapshots
     → snapshots: {tick_id: PerTickSnapshot}

3. Build execution_signature per tick
     → execution_signatures: {tick_id: tuple}

4. Detect contiguous execution segments
     → segments: [Segment]

5. Detect transitions between adjacent segments with different signatures
     → transitions: [TransitionRecord]

6. For each stable segment, summarize supporting evidence drift
     → segment_drift_summaries: {segment_index: DriftSummary}

7. Select representative ticks:
     - transition: previous / changed / next (from_tick, to_tick, confirmation_tick)
     - stable segment length >= 3: first two / last two
     - stable segment length 1–2: all ticks
     - explicit user tick_ids: always include
     → representative_tick_ids: [int, ...]

8. Return scaffold:
     - selected_ticks
     - segment_digest
     - transition_evidence
     - stable_segment_evidence_and_drift
     - representative_tick_snapshots
     - observation_slot
```

---

## 4. Core Data Structures

### PerTickSnapshot

```python
{
    "tick_id": int,
    "bt": {
        "root_status": str,                    # from tree_tick_end.status
        "node_statuses": {
            "NodeName": {"type": str, "status": str}  # from tick_end events
        }
    },
    "decisions": {
        "NodeName": {"inputs": dict, "status": str, "reason": str}  # from decision events
    },
    "blackboard": {
        "key": {"writer": str, "value": any}   # from bb_write events
    },
    "ros_in": {
        "source_topic": {"adapter": str, "received_count": int}
    },
    "input_state": {
        "AdapterName": {"bb_writes": dict}
    },
    "ros_out": [
        {"bt_node": str, "semantic_source": str, "target": str,
         "comm_type": str, "payload": any, "attribution_confidence": str}
    ]
}
```

### Segment

```python
{
    "index": int,
    "type": "stable" | "single",       # stable: length >= 2 with same signature
    "tick_ids": [int, ...],             # sorted ascending; contiguous in selected set
    "execution_signature": tuple,       # internal; not in scaffold repr
    "signature_digest": str             # human-readable (see Section 5)
}
```

### TransitionRecord

```python
{
    "from_segment_index": int,
    "to_segment_index": int,
    "from_tick": int,                   # last tick of previous segment
    "to_tick": int,                     # first tick of new segment
    "confirmation_tick": int | None,    # second tick of new segment if available
    "changed_bt_nodes": {
        "NodeName": {"before": str, "after": str}  # status changes only
    },
    "changed_decisions": {
        "NodeName": {
            "before": {"inputs": dict, "status": str, "reason": str},
            "after":  {"inputs": dict, "status": str, "reason": str}
        }
    },
    "changed_blackboard_or_ros_evidence": {
        "blackboard_changes": {
            "key": {"before": any, "after": any}   # from bb_write, from_tick vs to_tick
        },
        "ros_in_changes": {
            "source": {
                "before_present": bool, "after_present": bool,
                "before_count": int | None, "after_count": int | None
            }
        },
        "input_state_changes": {
            "AdapterName": {
                "before_bb_writes": dict | None,
                "after_bb_writes": dict | None
            }
        },
        "ros_out_changes": {
            "semantic_source": {
                "before": {"target": str, "comm_type": str, "payload": any} | None,
                "after":  {"target": str, "comm_type": str, "payload": any} | None
            }
        }
    },
    "representative_tick_ids": [int, ...]  # [from_tick, to_tick, confirmation_tick]
}
```

All `changed_*` fields contain only entries that differ between `from_tick` and `to_tick`. Unchanged fields are omitted.

### DriftSummary

```python
{
    "segment_index": int,
    "tick_range": [first_tick, last_tick],
    "decision_input_changes": {
        "NodeName": {"first": dict, "last": dict, "changed": bool}
    },
    "blackboard_value_changes": {
        "key": {"first": any, "last": any, "changed": bool}
    },
    "ros_in_changes": {
        "source": {"always_present": bool, "received_count_range": [min, max]}
    },
    "input_state_changes": {
        "AdapterName": {"bb_writes_changed": bool, "first": dict, "last": dict}
    },
    "ros_out_summary": {
        "semantic_source": {"target": str, "count": int, "payload_varied": bool}
    },
    "has_drift": bool,
    "drift_label": str  # "stable BT state with changing supporting evidence"
                        # or "stable BT state with stable supporting evidence"
}
```

### CrossTickBundle

```python
{
    "ok": bool,
    "error": str | None,
    "source_mode": "recent",
    "files": {"bt": "bt_debug_recent.jsonl", "comm": "bt_ros_comm_debug_recent.jsonl"},
    "recent_status": {
        "bt_age_seconds": float | None,
        "comm_age_seconds": float | None,
        "stale_warning": str | None
    },
    "order_detected": {"bt": str, "comm": str},
    "all_tick_ids": [int, ...],              # sorted ascending, from bt file
    "selected_tick_ids": [int, ...],         # sorted ascending
    "explicit_tick_ids": [int, ...],         # non-empty only for "A,B,C" format
    "snapshots": {tick_id: PerTickSnapshot},
    "segments": [Segment, ...],
    "transitions": [TransitionRecord, ...],
    "segment_drift_summaries": {segment_index: DriftSummary},
    "representative_tick_ids": [int, ...],   # sorted ascending
    "warnings": [str, ...]
}
```

---

## 5. Algorithms

### parse_tick_selection

```
"all"      → all tick_ids from bt file; explicit_tick_ids=[]
"latest:N" → last N tick_ids by value; explicit_tick_ids=[]
"A-B"      → tick_ids where A <= id <= B; explicit_tick_ids=[]
"A,B,C"    → explicit list; explicit_tick_ids=[A,B,C]
Missing tick_ids → warning per ID, continue with found ticks.
Format errors → warning, fallback to "all".
```

### build_execution_signature

```python
tick_end_sig = frozenset(
    (name, info["type"], info["status"])
    for name, info in snapshot["bt"]["node_statuses"].items()
)
decision_sig = frozenset(
    (name, d["status"], d["reason"])
    for name, d in snapshot["decisions"].items()
)
return (tick_end_sig, decision_sig)
```

`build_signature_digest` converts this to a human-readable string (never outputs Python frozenset/tuple repr):

```
BT node statuses:
  IsLineDetected: FAILURE
  FindLine: RUNNING
Decisions:
  IsLineDetected: FAILURE, reason="line_data is None"
```

### detect_segments

```
Walk selected tick_ids sorted ascending.
Group consecutive ticks with equal execution_signature into one segment.
Length >= 2: type="stable". Length = 1: type="single".
Non-consecutive explicit tick_ids each form their own single segment.
```

### detect_transitions

```
For each adjacent segment pair (i, i+1) with different execution_signatures:
  from_tick = last tick of segment i
  to_tick = first tick of segment i+1
  confirmation_tick = second tick of segment i+1 (if available and same signature)
  representative_tick_ids = [from_tick, to_tick, confirmation_tick (if present)]
  changed_bt_nodes: diff node_statuses between from_tick and to_tick snapshots
  changed_decisions: diff decisions between from_tick and to_tick snapshots
  changed_blackboard_or_ros_evidence:
    blackboard_changes: diff bb_write key/value
    ros_in_changes: diff source presence and received_count
    input_state_changes: diff adapter bb_writes
    ros_out_changes: diff semantic_source/target/comm_type/payload
  All changed_* fields: include only differing entries.
```

### summarize_segment_drift

```
For each stable segment:
  Walk all tick_ids in the segment.
  decision_input_changes: per decision node, compare inputs first vs last tick.
  blackboard_value_changes: per bb_write key, compare value first vs last tick.
  ros_in_changes: per source, track presence across all ticks + received_count range.
  input_state_changes: per adapter, check if bb_writes values changed.
  ros_out_summary: per semantic_source, count emissions and check payload variation.
  has_drift = True if any field changed.
  drift_label = "stable BT state with changing supporting evidence" if has_drift else
                "stable BT state with stable supporting evidence"
```

### select_representative_tick_ids

```
Start with empty set.
For each transition: add from_tick, to_tick, confirmation_tick (if present).
For each stable segment:
  length >= 3: add first two + last two tick_ids.
  length 1–2: add all tick_ids.
For each single segment: add the one tick_id.
For each explicit_tick_id: add unconditionally.
Return sorted, deduplicated list.
```

---

## 6. Scaffold Output

`_format_scaffold(bundle, user_observation) -> str` produces six sections.

### selected_ticks

```
selected_ticks:
  tick_ids: [1200, ..., 1218]
  total: 19
  source: recent only
  file_status: bt updated N sec ago [stale warning if applicable]
```

### segment_digest

One entry per segment. Use `signature_digest` (not Python repr). Include `drift_label` from DriftSummary if stable.

```
segment_digest:
  [0] stable, ticks 1200–1215 (16 ticks)
      BT node statuses:
        IsLineDetected: FAILURE
        FindLine: RUNNING
      Decisions:
        IsLineDetected: FAILURE, reason="line_data is None"
      drift: stable BT state with changing supporting evidence

  [1] single, tick 1216
      BT node statuses:
        IsLineDetected: SUCCESS
        FollowLine: RUNNING
      Decisions:
        IsLineDetected: SUCCESS, reason="line_data is not None"

  [2] stable, ticks 1217–1218 (2 ticks)
      [same signature as segment 1]
      drift: stable BT state with stable supporting evidence
```

### transition_evidence

One entry per TransitionRecord. All fields are deterministic tool output.

```
transition_evidence:
  transition [0] → [1]:
    from_tick: 1215 / to_tick: 1216 / confirmation_tick: 1217

    changed_bt_nodes:
      IsLineDetected: FAILURE → SUCCESS
      FindLine: RUNNING → [absent]
      FollowLine: [absent] → RUNNING

    changed_decisions:
      IsLineDetected:
        before: inputs={line_detected: false}, FAILURE, reason="line_data is None"
        after:  inputs={line_detected: true},  SUCCESS, reason="line_data is not None"

    changed_blackboard_or_ros_evidence:
      blackboard_changes:
        /latched/line_data: null → {x: 320, y: 240}
      ros_in_changes:
        /object/pixel_coords: present both ticks; received_count 1 → 1
      input_state_changes:
        LineDetectionAdapter:
          before: {/latched/line_data: null, /latched/last_line_x: null}
          after:  {/latched/line_data: {x:320,y:240}, /latched/last_line_x: 320}
      ros_out_changes:
        stop_walking: before={target:/walking/command, payload:{command:disable}} → after=absent
        walk_forward: before=absent → after={target:/walking/command, payload:{command:enable}}

    evidence_chain: see representative tick snapshots for [1215, 1216, 1217]
```

### stable_segment_evidence_and_drift

One entry per stable segment. Use DriftSummary. Do not expand every tick.

```
stable_segment_evidence_and_drift:
  segment [0] (ticks 1200–1215, 16 ticks):
    representative ticks: 1200, 1201, 1214, 1215
    reason: 16-tick stable segment; first two + last two shown

    repeated execution signature:
      [same as segment_digest entry above]

    supporting evidence drift:
      decision_input_changes:
        IsLineDetected: inputs unchanged (line_detected: false throughout)
      blackboard_value_changes:
        /latched/camera_lost_count: 791 (tick 1200) → 805 (tick 1215), changed=true
        /head_pan_pos: 480 (tick 1200) → 480 (tick 1215), varied between
      ros_in_changes:
        /imu: always present, received_count stable [10, 10]
        /object/pixel_coords: always present, received_count stable [1, 1]
      ros_out_summary:
        stop_walking: emitted 16 ticks, payload stable
        move_head: emitted 16 ticks, payload varied (head_pan_pos 480–510)
      drift_label: stable BT state with changing supporting evidence
```

### representative_tick_snapshots

Full PerTickSnapshot data for each tick in `representative_tick_ids` only.

Label each: `"Snapshot for tick N (representative: <reason>)"`.

### observation_slot

```
observation_slot:
  [If user_observation is absent]
    This is log-only analysis.
    [Ask for specific runtime context only if it would change the diagnosis.
     Do not ask generic questions about physical state.]

  [If user_observation is present]
    Compare observation against BT, blackboard, and ROS evidence:
    - matches: observation agrees with log evidence
    - mismatches: observation contradicts log evidence
    - missing evidence: observation mentions something not in logs
```

---

## 7. LLM Rules

Include in scaffold before analysis instructions:

```
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
```

---

## 8. Implementation Plan

### Phase 1: `ainex_agent_tools/bt_analysis/raw_cross_tick.py`

Reuse from `raw_tick.py` (import directly):
```python
from ainex_agent_tools.bt_analysis.raw_tick import (
    _path, _file_status, _read_jsonl_raw, _detect_order, _group_by_tick
)
```

Implement in order:
1. `parse_tick_selection(selection, all_tick_ids) -> (list, list)`
2. `build_per_tick_snapshot(tick_id, bt_entries, comm_entries) -> dict`
3. `build_execution_signature(snapshot) -> tuple`
4. `build_signature_digest(snapshot) -> str`
5. `detect_segments(snapshots_ordered, signatures) -> list`
6. `detect_transitions(segments, snapshots) -> list`
7. `summarize_segment_drift(segment, snapshots) -> dict`
8. `select_representative_tick_ids(segments, transitions, explicit_tick_ids) -> list`
9. `get_raw_cross_tick_bundle(tick_selection="all") -> dict`

### Phase 2: `ainex_agent_tools/tools/cross_tick_analysis.py`

```python
@tool
def cross_tick_analysis(tick_selection: str = "all", user_observation: str = "") -> str:
    bundle = get_raw_cross_tick_bundle(tick_selection)
    if not bundle["ok"]:
        return f"cross_tick_analysis error: {bundle['error']}"
    return _format_scaffold(bundle, user_observation)
```

`_format_scaffold` produces the six sections from Section 6.
JSONL schema legend is **imported** (not copied) from `bt_tick_analysis.py`:
```python
from ainex_agent_tools.tools.bt_tick_analysis import _JSONL_SCHEMA_LEGEND
```
Inserted at line 355 of `_format_scaffold`, between `_fmt_selected_ticks` and `_fmt_segment_digest`.
LLM rules from Section 7 appended as `_ANALYSIS_INSTRUCTIONS` at end of scaffold.

### Phase 3: Registration ✓

`ainex_agent_tools/__init__.py`:
```python
from ainex_agent_tools.tools.cross_tick_analysis import cross_tick_analysis
# Added to AINEX_TOOLS (total: 8 tools)
```

`ainex_agent_tools/prompts.py` — final text as implemented:

**`critical_instructions` rule 5** (renamed from ONE-TICK DEBUG):
```
SINGLE-TICK DIAGNOSIS: For one specific tick — the latest tick, a named tick_id,
or a paused/stepped tick — call analyze_bt_tick. It diagnoses status, conditions,
and commands for that single moment. If user_observation is provided, output all
three sections: explain_tick, compare_tick, and diagnose_tick. Without observation,
output only explain_tick and ask the user to describe what they physically observed.
```

**`critical_instructions` rule 9** (renamed from CROSS-TICK ANALYSIS):
```
CROSS-TICK RELATIONSHIP: For questions about how BT behavior evolved over the
last ~3 seconds (up to 30 ticks at ~10 Hz) — transitions, stability, drift,
trends — call cross_tick_analysis. Single-moment questions use analyze_bt_tick
instead. Typical two-step workflow: call cross_tick_analysis first to identify
the relevant tick or transition, then call analyze_bt_tick for deep single-tick
diagnosis.
```

**`about_your_capabilities` — `analyze_bt_tick` entry:**
```
Single-tick status diagnosis. Analyzes ONE specific tick in depth: which BT
branch was active, what inputs drove each condition, blackboard state, ROS
commands emitted, and whether that matches user observation.
Use when the question is about a single moment: 'what happened at tick N',
'why did the BT do X in that tick', or diagnosing unexpected behavior at a
specific paused/stepped tick.
tick_id default: 'latest'. With user_observation: explain/compare/diagnose.
Without: explain_tick only. Tool output includes an embedded JSONL schema
legend for interpreting raw evidence.
```

**`about_your_capabilities` — `cross_tick_analysis` entry:**
```
Cross-tick relationship analysis. Covers up to 30 recent ticks (~3 seconds at
~10 Hz BT rate). Detects stable BT execution segments, transitions between
states, and supporting evidence drift within segments.
Use when the question spans time: 'what changed in the last few seconds',
'when did the BT switch state', 'was the BT stable', 'how long has the robot
been in FindLine', or 'summarize recent BT activity'.
tick_selection: 'all' | 'latest:N' | 'A-B' | 'A,B,C'.
user_observation is optional — log-only analysis is complete without it.
Tool output includes an embedded JSONL schema legend for interpreting raw
evidence events (same legend as analyze_bt_tick and get_bt_tick_raw).
Outputs: explain_cross_ticks / evidence_chain / transition_details /
representative_ticks / observation_slot / compare_followup.
```

### Phase 4: Tests ✓

Checklist:
- [x] All three tick_selection formats return correct tick_ids
- [x] Consecutive same-signature ticks grouped as stable segment
- [x] Transition detected with correct from_tick/to_tick/confirmation_tick
- [x] `changed_blackboard_or_ros_evidence` includes all four sub-fields
- [x] Stable segment drift summary correctly populated (camera_lost_count change visible)
- [x] representative_tick_ids: first 2 + last 2 for stable segment ≥ 3
- [x] explicit_tick_ids always appear in representative_tick_ids
- [x] signature_digest is human-readable
- [x] Stale file warning present when applicable
