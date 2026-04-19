#!/usr/bin/env python3
"""
ROSA tools for BT one-tick analysis.

Two tools:
  get_bt_tick_raw   — raw JSONL evidence for a specific tick (recent files only)
  analyze_bt_tick   — staged explain/compare/diagnose scaffold for one tick
"""
from langchain.agents import tool

from ainex_agent_tools.bt_analysis.raw_tick import get_raw_tick_bundle

# ---------------------------------------------------------------------------
# JSONL Schema Legend — embedded in tool output to help the LLM interpret raw events
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
# Default project context (high-level only — no per-node implementation detail)
# ---------------------------------------------------------------------------

_DEFAULT_PROJECT_CONTEXT = (
    "This marathon BT is for Ainex HuroCup line-following behavior. At a high level, "
    "the robot should maintain safe standing state, use vision/blackboard line "
    "information to decide whether to follow or search for the line, and emit ROS "
    "commands through the BT semantic/communication layers. "
    "The raw tick logs are the primary evidence for exact node behavior and payloads."
)

# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def _fmt_entries(entries: list) -> str:
    """Return raw_line for each entry, one per line. Skip file-error sentinels."""
    lines = []
    for e in entries:
        if e.get("error"):
            lines.append(f"[ERROR: {e['error']}]")
        elif e.get("parse_error"):
            lines.append(f"[PARSE ERROR: {e['parse_error']}] {e.get('raw_line', '')}")
        else:
            lines.append(e.get("raw_line", ""))
    return "\n".join(lines) if lines else "(no entries)"


def _fmt_neighbor_block(tick_id: int, data: dict) -> str:
    bt_raw   = _fmt_entries(data["bt_entries"])
    comm_raw = _fmt_entries(data["comm_entries"])
    return (
        f"=== NEIGHBOR TICK {tick_id} "
        f"(context only — do not treat as selected-tick evidence) ===\n"
        f"BT DECISION LAYER:\n{bt_raw}\n\n"
        f"ROS COMM LAYER:\n{comm_raw}"
    )


def _fmt_bundle_header(bundle: dict) -> str:
    parts = [
        f"selected_tick_id : {bundle['selected_tick_id']}",
        f"source_mode      : {bundle['source_mode']}",
    ]
    rs = bundle["recent_status"]
    if rs["bt_age_seconds"] is not None:
        parts.append(f"bt file age      : {rs['bt_age_seconds']}s")
    if rs["comm_age_seconds"] is not None:
        parts.append(f"comm file age    : {rs['comm_age_seconds']}s")
    if rs["stale_warning"]:
        parts.append(f"STALE WARNING    : {rs['stale_warning']}")
    for w in bundle["warnings"]:
        parts.append(f"WARNING          : {w}")
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

@tool
def get_bt_tick_raw(tick_id: str = "latest", include_neighbors: int = 1) -> str:
    """
    Return raw JSONL evidence for a specific BT tick from the recent rolling files.

    Reads bt_debug_recent.jsonl and bt_ros_comm_debug_recent.jsonl ONLY.
    There is no fallback to lastrun files — use this for live or paused sessions.

    Args:
        tick_id:           "latest" (default) or an integer tick_id string.
        include_neighbors: number of adjacent ticks to include as context (default 1).
                           Neighbor ticks are labeled and must not be treated as
                           evidence for the selected tick.

    Use this tool when:
      - The user asks to see raw BT log data for a specific tick.
      - You want to inspect raw evidence before calling analyze_bt_tick.
    """
    bundle = get_raw_tick_bundle(tick_id, include_neighbors)

    if not bundle["ok"]:
        avail = bundle.get("all_tick_ids", [])
        msg = f"ERROR: {bundle['error']}"
        if avail:
            msg += f"\nAvailable tick_ids: {avail}"
        return msg

    header = _fmt_bundle_header(bundle)
    sel = bundle["selected_tick"]
    bt_raw   = _fmt_entries(sel["bt_entries"])
    comm_raw = _fmt_entries(sel["comm_entries"])

    sections = [
        f"=== SELECTED TICK {bundle['selected_tick_id']} (source: recent) ===",
        header,
        "",
        _JSONL_SCHEMA_LEGEND,
        "",
        "--- BT Decision Layer ---",
        bt_raw,
        "",
        "--- ROS Comm Layer ---",
        comm_raw,
    ]

    for nid in sorted(bundle["neighbors"]):
        sections.append("")
        sections.append(_fmt_neighbor_block(nid, bundle["neighbors"][nid]))

    return "\n".join(sections)


@tool
def analyze_bt_tick(
    tick_id: str = "latest",
    project_context: str = "",
    user_observation: str = "",
    rqt_observation: str = "",
    include_neighbors: int = 1,
) -> str:
    """
    Return a one-tick analysis package for ROSA to write explain/compare/diagnose.

    Retrieves raw logs for the selected tick, then assembles a structured package
    containing: project context, observations, raw evidence, and staged analysis
    instructions.  ROSA's main LLM uses this package to write the three-section report.

    Args:
        tick_id:           "latest" (default) or an integer tick_id string.
        project_context:   optional high-level description of the project goal and
                           expected behavior.  If empty, a default marathon context
                           is used.  Keep high-level — detailed behavior must come
                           from the raw tick logs.
        user_observation:  what the user physically observed on the robot during
                           this tick (motion, sound, posture, etc.).  If provided,
                           full analysis (explain + compare + diagnose) is returned.
                           If empty, explain-only mode is used and ROSA will ask
                           the user for their observation.
        rqt_observation:   optional rqt_py_trees / pytreeview observation (tree
                           state, node colors, blackboard panel values).
        include_neighbors: adjacent ticks to include as optional context (default 1).

    Use this tool when:
      - The user reports unexpected robot behavior and you want to diagnose a tick.
      - The robot is in pause or step mode and you are analyzing a specific tick.
      - FIRST CHOICE for one-tick runtime fault diagnosis.
    """
    bundle = get_raw_tick_bundle(tick_id, include_neighbors)

    if not bundle["ok"]:
        avail = bundle.get("all_tick_ids", [])
        msg = f"ERROR: {bundle['error']}"
        if avail:
            msg += f"\nAvailable tick_ids: {avail}"
        return msg

    # Determine analysis mode
    analysis_mode = "full" if user_observation.strip() else "explain_only"

    # Project context
    ctx = project_context.strip() if project_context.strip() else _DEFAULT_PROJECT_CONTEXT

    # File status block
    rs = bundle["recent_status"]
    file_status_lines = []
    if rs["bt_age_seconds"] is not None:
        file_status_lines.append(f"bt:   age={rs['bt_age_seconds']}s")
    if rs["comm_age_seconds"] is not None:
        file_status_lines.append(f"comm: age={rs['comm_age_seconds']}s")

    # Selected tick raw
    sel = bundle["selected_tick"]
    bt_raw   = _fmt_entries(sel["bt_entries"])
    comm_raw = _fmt_entries(sel["comm_entries"])

    # Neighbor raw blocks
    neighbor_blocks = []
    for nid in sorted(bundle["neighbors"]):
        neighbor_blocks.append(_fmt_neighbor_block(nid, bundle["neighbors"][nid]))

    # ── Analysis instructions ────────────────────────────────────────────────
    global_rules = (
        "GLOBAL RULES:\n"
        "1. SELECTED TICK RAW = primary evidence. Use raw field values: bt_node, condition,\n"
        "   inputs, blackboard key/value, semantic_source, target, comm_type, ros_node, payload.\n"
        "2. NEIGHBOR RAW = context only. Do NOT inspect or cite neighbor ticks unless:\n"
        "   (a) the selected tick raw alone cannot explain the observed behavior, OR\n"
        "   (b) the user described a change, ongoing process, or repeated command across ticks.\n"
        "   When neighbor raw IS used, explicitly label the neighbor tick_id and state why it\n"
        "   was needed. Do NOT use neighbor events as facts about the selected tick.\n"
        "3. ros_out in comm log proves command was EMITTED — not that the robot physically\n"
        "   executed it. Never claim motion occurred from ros_out alone.\n"
        "4. diagnose_tick MUST cite concrete raw fields (bt_node, semantic_source, target,\n"
        "   payload, etc.) — do not diagnose in the abstract."
    )

    if analysis_mode == "explain_only":
        task_instructions = (
            "TASK — explain_tick ONLY (no user observation provided):\n"
            "- Use PROJECT CONTEXT + SELECTED TICK RAW only.\n"
            "- Output:\n"
            "    • Active leaf node and execution path through the tree\n"
            "    • Condition inputs and their result (SUCCESS / FAILURE / RUNNING)\n"
            "    • Important blackboard values read or written this tick\n"
            "    • ROS commands emitted with full payloads (from comm layer)\n"
            "    • Theoretical physical behavior the robot should exhibit\n"
            "    • Unknowns: what the raw logs do NOT prove\n"
            "- After explain_tick, ask the user to describe what they physically observed\n"
            "  so you can proceed to compare_tick and diagnose_tick."
        )
    else:
        task_instructions = (
            "TASK — write all three sections:\n"
            "\nexplain_tick:\n"
            "- Use PROJECT CONTEXT + SELECTED TICK RAW only (no user observation).\n"
            "- Output:\n"
            "    • Active leaf node and execution path\n"
            "    • Condition inputs and results\n"
            "    • Blackboard values (read/written)\n"
            "    • ROS commands with full payloads\n"
            "    • Theoretical physical behavior\n"
            "    • Unknowns\n"
            "\ncompare_tick:\n"
            "- Use explain_tick + USER OBSERVATION.\n"
            "- Do NOT diagnose root cause here.\n"
            "- Output:\n"
            "    • Matches (theoretical == observed)\n"
            "    • Mismatches (map theoretical → observed)\n"
            "    • Uncertain items (cannot confirm from available data)\n"
            "\ndiagnose_tick:\n"
            "- Use compare_tick + PROJECT CONTEXT + SELECTED TICK RAW"
            + (" + RQT/PYTREEVIEW OBSERVATION" if rqt_observation.strip() else "")
            + ".\n"
            "- Output:\n"
            "    • Most likely fault layer (0–4) + confidence\n"
            "    • Concrete attribution: bt_node, condition, inputs, blackboard key/value,\n"
            "      semantic_source, target, comm_type, ros_node, payload\n"
            "    • Evidence (cite raw fields explicitly)\n"
            "    • Less-likely layers and why they are less likely\n"
            "    • Next verification questions\n"
            "- Layer guide:\n"
            "    0 = alignment/source mismatch (tick_id, wrong session, stale file, namespace)\n"
            "    1 = BT decision/input (condition result, blackboard value, tree branch)\n"
            "    2 = command generation/emission (leaf ticked but ros_out missing, wrong payload)\n"
            "    3 = ROS controller/comm (ros_out present but controller state unchanged)\n"
            "    4 = physical/hardware (servo, power, mechanical, posture)"
        )

    # ── Assemble output ──────────────────────────────────────────────────────
    parts = [
        "=== ANALYZE_BT_TICK PACKAGE ===",
        f"analysis_mode    : {analysis_mode}",
        f"selected_tick_id : {bundle['selected_tick_id']}",
        f"source_mode      : {bundle['source_mode']}",
        f"include_neighbors: {include_neighbors}",
    ]

    if rs["stale_warning"]:
        parts.append(f"\nSTALE WARNING: {rs['stale_warning']}")
    for w in bundle["warnings"]:
        parts.append(f"WARNING: {w}")

    parts += [
        "",
        "--- RECENT FILE STATUS ---",
        "\n".join(file_status_lines) if file_status_lines else "(unknown)",
        "",
        "--- PROJECT CONTEXT ---",
        ctx,
        "",
        "--- USER OBSERVATION ---",
        user_observation.strip() if user_observation.strip() else "(none provided)",
        "",
        "--- RQT/PYTREEVIEW OBSERVATION ---",
        rqt_observation.strip() if rqt_observation.strip() else "(none provided)",
        "",
        "--- SELECTED TICK RAW (primary evidence) ---",
        _JSONL_SCHEMA_LEGEND,
        "",
        "BT DECISION LAYER:",
        bt_raw,
        "",
        "ROS COMM LAYER:",
        comm_raw,
    ]

    for nb in neighbor_blocks:
        parts.append("")
        parts.append(nb)

    parts += [
        "",
        "--- ANALYSIS INSTRUCTIONS ---",
        global_rules,
        "",
        task_instructions,
    ]

    return "\n".join(parts)
