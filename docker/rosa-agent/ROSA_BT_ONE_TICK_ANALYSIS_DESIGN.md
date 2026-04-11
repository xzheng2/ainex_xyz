# ROSA BT One-Tick Analysis Design

> Scope: ROSA-side runtime analysis for one BT tick.
> Target project: `ainex_behavior/marathon` first, reusable for later BT projects.
> Input logs: `ainex_behavior/<project>/log/*`.
> Goal: compare code-level theoretical behavior with user-observed physical behavior, then diagnose the most likely fault source with raw log evidence.
>
> **Status: IMPLEMENTED — MVP complete 2026-04-11.**
> Implemented files: `ainex_agent_tools/bt_analysis/raw_tick.py`, `ainex_agent_tools/tools/bt_tick_analysis.py`.
> Two tools exported: `get_bt_tick_raw`, `analyze_bt_tick`.
> Prompts updated: `about_your_capabilities` + `critical_instructions` rules 5–7.

---

## 1. Background

The BT observability recorder already emits semantic JSONL logs suitable for LLM reading.
The logs are tick-centered:

- BT decision events and node statuses are written to `bt_debug_*.jsonl`.
- ROS communication events are written to `bt_ros_comm_debug_*.jsonl`.
- Both layers share `tick_id`.
- Business ROS outbound events include `bt_node`, `semantic_source`, `target`, `comm_type`, `ros_node`, and `payload`.
- Sensor inbound events include `target`, `adapter`, and `payload`.
- Infrastructure communication is separated into `infra_comm_manifest_lastrun.json`.

Therefore, ROSA does not need to hide the raw logs behind a fully abstracted model before LLM analysis. The raw tick lines are already the primary evidence.

The ROSA-side responsibility is:

1. Select the correct tick raw logs.
2. Present project context and raw logs to the LLM in a disciplined workflow.
3. Compare theoretical behavior with user observation.
4. Diagnose both by layer and by concrete raw evidence fields.

---

## 2. Core Use Case

The robot is in `pause` or `step` mode. The user observes a specific tick, usually the latest tick, and asks what is wrong.

The user can provide two kinds of observation:

1. Physical-world observation:
   - Robot body moved or did not move.
   - Head moved or did not move.
   - Robot fell, shook, turned, walked forward, stopped, etc.
   - The user can see a line, ball, goal, obstacle, or other environmental condition.

2. UI / replay observation:
   - rqt live display.
   - py_trees view.
   - pytreeview replay result.
   - Optional plugin output.

The analysis compares:

```text
Raw tick logs + project context
  => code-level theoretical behavior

User physical observation
  => real-world behavior

Optional rqt / pytreeview observation
  => runtime UI / replay behavior
```

The final diagnosis identifies where theory and reality diverge.

---

## 3. Inputs

### 3.1 Project Context: `A`

Natural-language project description supplied by the user or by a code agent after reading project code.

Example:

```text
This marathon BT detects the track line, follows it when visible, searches for it when lost, and runs recovery when the robot is not standing.
FindLine should stop walking and move the head to search for the line.
FollowLine should move the head to center and issue walking gait commands.
```

This context helps convert raw fields into expected physical behavior.

### 3.2 Tick Raw Logs: `B`

Raw JSONL lines for the selected tick:

- `bt_debug_recent.jsonl`
- `bt_ros_comm_debug_recent.jsonl`
- Optional surrounding tick raw lines, usually `tick_id - 1` and `tick_id + 1`
- Optional `infra_comm_manifest_lastrun.json` as static interface reference only, not as a tick source

Important rule:

```text
B raw logs are first-class LLM evidence. Do not over-summarize away payload fields.
One-tick analysis reads only recent JSONL files. lastrun files are reserved for later replay / cross-run workflows.
```

The retriever may index or normalize internally, but the LLM should still see the relevant raw JSON lines.

### 3.3 User Observation: `C`

Natural-language physical observation for the same tick.

Example:

```text
The robot was still walking forward. The head did not move. I could see the line on the floor.
```

### 3.4 Optional RQT / Pytreeview Observation: `D`

Natural-language or structured observation from rqt, py_trees view, pytreeview, or plugin output.

Example:

```text
rqt showed active node FindLine. IsLineDetected was FAILURE. line_data was None.
```

---

## 4. Output

The user-facing report always contains three sections:

```text
1. explain_tick
2. compare_tick
3. diagnose_tick
```

The report must support follow-up observations. If the user adds more observation, ROSA can rerun `compare_tick` and `diagnose_tick` against the same `explain_tick` and raw logs.

---

## 5. Processing Flow

```text
A + B raw
  -> explain_tick

explain_tick + C
  -> compare_tick

compare_tick + A + B raw + optional D raw
  -> diagnose_tick

explain_tick + compare_tick + diagnose_tick
  -> final report
```

Key design choice:

```text
explain_tick uses raw logs.
compare_tick uses explain_tick and user observation.
diagnose_tick goes back to raw logs to produce concrete evidence-backed attribution.
```

### 5.1 MVP Execution Model

MVP uses a single ROSA agent answer, not subagents and not nested LLM calls inside tools.

```text
analyze_bt_tick tool
  -> deterministic retrieval from recent JSONL
  -> returns raw tick bundle + staged prompt scaffold

ROSA main agent
  -> writes explain_tick
  -> writes compare_tick if user_observation is present
  -> writes diagnose_tick if user_observation is present
```

This is intentionally simpler than a multi-agent or three-call orchestrator. The staged prompt scaffold still enforces the reasoning order, while keeping the ROSA tool layer read-only and deterministic.

Future versions may replace the scaffold with an orchestrated three-call chain:

```text
explain_prompt(A, B raw)
compare_prompt(explain_tick, C)
diagnose_prompt(compare_tick, A, B raw, D)
```

Subagents are not part of MVP. If later needed, a verifier subagent is a better fit than splitting the primary flow across multiple agents.

### 5.2 Missing Observation Behavior

If `user_observation` is empty, one-tick analysis is allowed but must run in explain-only mode:

```text
Return explain_tick only.
Do not produce compare_tick.
Do not produce diagnose_tick.
Ask the user to add physical observation for the same tick.
```

This preserves the core diagnostic contract: comparison and diagnosis require real-world observation.

---

## 6. Stage 1: `explain_tick`

### 6.1 Inputs

```text
A: Project context
B: Raw JSONL lines for the selected tick
```

### 6.2 Responsibility

Explain the code-level theoretical behavior for this tick.

It should answer:

- Which BT leaf/action/condition is active?
- Why did the BT choose this path?
- Which condition inputs and blackboard values mattered?
- Which ROS commands were emitted?
- What should the robot theoretically do in the physical world?
- What can the logs not prove?

### 6.3 Hard Constraints

`explain_tick` must not use user observation.

It must not say:

```text
The robot actually stopped.
The head actually moved.
The controller executed the command.
```

It may say:

```text
The code requested stop_walking.
The log shows a move_head command was emitted.
The theoretical physical behavior is that the robot should stop and move the head.
```

### 6.4 Output Shape

```text
explain_tick:
  tick:
    tick_id:
    source_files:

  code_state:
    active_leaf:
    active_path:
    tree_status:
    leaf_status:

  decision_reason:
    condition:
    inputs:
    status:
    reason:

  blackboard_context:
    key_values:

  ros_commands:
    - bt_node:
      semantic_source:
      target:
      comm_type:
      ros_node:
      payload:

  theoretical_physical_behavior:
    - ...

  unknowns:
    - ...

  raw_evidence:
    - short quoted/paraphrased field references
```

### 6.5 Example

```text
explain_tick

Tick 1618 is in line-search behavior.
The BT reports IsRobotStanding=SUCCESS with robot_state=stand.
It reports IsLineDetected=FAILURE because line_data is None.
Therefore PatrolControl does not enter FollowLine and instead ticks FindLine.

ROS commands emitted in this tick:
- FindLine / stop_walking -> /walking/command, payload={"command":"disable"}
- FindLine / move_head -> ros_robot_controller/bus_servo/set_position,
  payload={"duration":50,"positions":[[23,320]]}

Theoretical physical behavior:
- The robot should not continue active forward walking.
- The head pan servo 23 should move toward position 320, or already be in the search sweep.
- The robot is searching for the line, not following it.

Unknowns:
- The log proves the commands were emitted, but not that the walking controller or servo physically executed them.
```

---

## 7. Stage 2: `compare_tick`

### 7.1 Inputs

```text
explain_tick output
C: User physical observation
```

### 7.2 Responsibility

Compare the theoretical behavior with the user's observed physical behavior in natural language.

It should answer:

- What matches?
- What conflicts?
- What is unclear?
- Which user observation maps to which theoretical command or state?

### 7.3 Hard Constraints

`compare_tick` should not perform final root-cause diagnosis.

It should avoid jumping from:

```text
head did not move
```

to:

```text
servo is broken
```

Instead, it should say:

```text
This conflicts with the emitted move_head command.
The conflict needs diagnosis using raw payload and feedback evidence.
```

### 7.4 Output Shape

```text
compare_tick:
  matches:
    - ...

  mismatches:
    - theoretical:
      observed:
      related_explain_item:

  uncertain:
    - ...
```

### 7.5 Example

User observation:

```text
The robot was still walking forward. The head did not move.
```

Output:

```text
compare_tick

Matches:
- The user did not report line-following behavior, which is not in conflict with the explained FindLine/search state.

Mismatches:
- explain_tick says stop_walking was emitted and the robot theoretically should not keep walking forward.
  The user observed that the robot was still walking forward.
- explain_tick says move_head was emitted for servo 23.
  The user observed that the head did not move.

Uncertain:
- The user did not report whether rqt/pytreeview also showed FindLine.
- The user did not report walking controller feedback or servo position feedback.
```

---

## 8. Stage 3: `diagnose_tick`

### 8.1 Inputs

```text
compare_tick output
A: Project context
B: Raw JSONL lines for the selected tick
D: Optional rqt / pytreeview observation
```

### 8.2 Responsibility

Diagnose the mismatch by both layer and concrete evidence.

It should answer:

- Which fault layer is most likely?
- Which exact BT node, condition, blackboard key, ROS target, semantic source, and payload are involved?
- Which alternatives are less likely and why?
- What should the user or agent verify next?

### 8.3 Diagnosis Layers

```text
Layer 0: Alignment / source mismatch
  tick_id mismatch, wrong session, recent file stale, rqt display not aligned, namespace mismatch.

Layer 1: BT decision / input layer
  condition result, blackboard value, tree branch selection, state machine logic.

Layer 2: Command generation / emission layer
  leaf ticked but ros_out missing, wrong semantic_source, wrong target, wrong payload.

Layer 3: ROS controller / communication feedback layer
  ros_out exists but controller state does not change, command may not be accepted, competing publisher/service caller.

Layer 4: Physical execution / hardware layer
  controller appears correct but physical action is wrong: servo, power, mechanical jam, slipping, unstable posture.
```

### 8.4 Concrete Attribution Fields

Diagnosis must name concrete raw fields when available:

```text
BT:
  active leaf
  condition node
  condition inputs
  condition status
  blackboard key/value
  node status

ROS:
  bt_node
  semantic_source
  target
  comm_type
  direction
  ros_node
  payload

Observation:
  user physical behavior
  rqt/pytreeview behavior
```

### 8.5 Output Shape

```text
diagnose_tick:
  most_likely_layer:
    layer:
    confidence:
    summary:

  concrete_attribution:
    - object:
      raw_fields:
      why_relevant:

  evidence:
    - ...

  less_likely:
    - layer:
      reason:

  next_verification_questions:
    - ...
```

### 8.6 Example

```text
diagnose_tick

Most likely layer:
Layer 3/4: controller execution or physical execution.
Confidence: medium.

Concrete attribution:
1. Walking stop path
   bt_node=FindLine
   semantic_source=stop_walking
   target=/walking/command
   comm_type=service_call
   ros_node=ainex_controller
   payload={"command":"disable"}

2. Head movement path
   bt_node=FindLine
   semantic_source=move_head
   target=ros_robot_controller/bus_servo/set_position
   comm_type=topic_publish
   ros_node=ros_robot_controller
   payload={"duration":50,"positions":[[23,320]]}

Evidence:
- The BT decision layer reached FindLine consistently with line_data=None.
- The command layer emitted both stop_walking and move_head.
- The user's physical observation says walking continued and the head did not move.
- Therefore the mismatch appears after command emission.

Less likely:
- Layer 1 BT decision problem is less likely if the intended behavior on line loss is FindLine.
- Layer 2 command generation problem is less likely because raw ros_out entries exist with concrete payloads.

Next verification:
1. Check whether /walking/is_walking remained true after the disable command.
2. Check whether another node also commands walking or head servo movement.
3. Check servo 23 actual position, error code, voltage, or controller rosout.
4. If rqt/pytreeview does not show FindLine for the same tick, first resolve tick/session alignment.
```

---

## 9. RQT / Pytreeview Use

RQT and pytreeview observations are not mandatory, but they are valuable for checking alignment.

Use them mainly to answer:

```text
Does the UI/replay agree with the raw log about active BT state?
Does the UI/replay show the same blackboard values?
Is the user looking at the same tick/session/project namespace?
```

Rules:

```text
If rqt/pytreeview agrees with raw logs but physical observation disagrees:
  Focus diagnosis after BT decision, usually Layer 3 or Layer 4.

If rqt/pytreeview disagrees with raw logs:
  Diagnose Layer 0 first: tick alignment, recent file freshness, namespace, display/replay source.

If rqt/pytreeview adds blackboard details missing from raw logs:
  Use it as supporting evidence, but keep raw logs as primary source unless plugin data is more direct.
```

---

## 10. Tool Interface

### 10.1 MVP Tools

Expose two ROSA tools for MVP:

1. `get_bt_tick_raw`
2. `analyze_bt_tick`

`get_bt_tick_raw` is for raw evidence inspection and retrieval debugging. `analyze_bt_tick` is the user-facing one-tick analysis entrypoint.

```python
get_bt_tick_raw(
    tick_id: str = "latest",
    include_neighbors: int = 1,
) -> str
```

Behavior:

1. Resolve `tick_id`.
2. Retrieve selected tick raw lines from recent logs only.
3. Retrieve neighbor raw lines when `include_neighbors > 0`.
4. Return raw JSONL lines grouped by selected tick and optional neighbor ticks.

Default `include_neighbors=1`, but neighbor ticks are optional context. They must not be treated as selected-tick evidence unless explicitly cited by tick id.

```python
analyze_bt_tick(
    tick_id: str = "latest",
    project_context: str = "",
    user_observation: str = "",
    rqt_observation: str = "",
    include_neighbors: int = 1,
) -> str
```

Behavior:

1. Resolve `tick_id`.
2. Retrieve raw lines for that tick from recent logs only.
3. Retrieve optional neighbor raw lines. Default is previous and next tick when present.
4. Return a deterministic analysis package:
   - selected tick metadata
   - recent file status
   - project context
   - user observation
   - optional rqt / pytreeview observation
   - selected tick raw logs
   - optional neighbor raw logs
   - staged prompt scaffold
5. ROSA main agent then writes the user-facing `explain_tick`, `compare_tick`, and `diagnose_tick` sections.

If `user_observation` is empty:

```text
Return explain-only scaffold.
ROSA should output only explain_tick and ask the user for physical observation.
```

### 10.2 Follow-Up Tool

Optional after MVP:

```python
continue_bt_tick_analysis(
    tick_id: str = "latest",
    additional_user_observation: str = "",
    additional_rqt_observation: str = "",
) -> str
```

Behavior:

1. Reuse the same tick raw logs if still available.
2. Append the new observation.
3. Rerun `compare_tick` and `diagnose_tick`.
4. State how the diagnosis changed.

### 10.3 Future Split Tools

After the single tool works, split for debugging and testing:

```python
explain_bt_tick(tick_id, project_context="")
compare_bt_tick(tick_id, user_observation, rqt_observation="")
diagnose_bt_tick(tick_id, user_observation, rqt_observation="")
```

---

## 11. Raw Tick Retrieval

The retriever is deterministic code, not LLM reasoning.

One-tick retrieval reads only:

```text
bt_debug_recent.jsonl
bt_ros_comm_debug_recent.jsonl
```

It must not fall back to lastrun.

If recent files are missing, the tool should report that one-tick runtime analysis is unavailable.

If recent files exist but are stale, the tool should still allow analysis. Staleness is expected in pause/step mode. The output must include a warning such as:

```text
Recent logs are not currently updating. This is allowed for pause/step mode.
Confirm this is the paused tick before treating it as the current runtime state.
```

Implemented module:

```text
ainex_agent_tools/bt_analysis/raw_tick.py
```

Responsibilities:

1. Read recent files only:
   - `bt_debug_recent.jsonl`
   - `bt_ros_comm_debug_recent.jsonl`
   - Do not auto-select lastrun.
   - Do not infer the latest runtime tick from lastrun.

2. Detect file order per file (bt and comm independently):
   - Compare first and last valid `tick_id` in each file.
   - Returns `"newest-first" | "oldest-first" | "single" | "unknown"` per file.

3. Resolve tick:
   - `"latest"` means max `tick_id` available in the bt file.
   - Numeric string or int means exact tick. Error if not found.

4. Extract raw entries:
   - Each entry: `{"raw_line": str, "parsed": dict|None, "parse_error": str|None}`.
   - BT entries for selected tick + optional neighbor ticks.
   - Comm entries matched by `tick_id` from comm file.

Neighbor use policy:

```text
The selected tick is primary evidence.
Neighbor ticks are optional context.
Do not inspect or cite neighbor ticks unless:
  (a) selected tick raw alone cannot explain the observed behavior, OR
  (b) user described a change, ongoing process, or repeated command across ticks.
When used, explicitly label the neighbor tick_id and state why it was needed.
```

5. Return RawTickBundle:

```text
RawTickBundle
  ok: bool
  error: str | None
  selected_tick_id: int | None
  source_mode: "recent"
  files:
    bt: "bt_debug_recent.jsonl"
    comm: "bt_ros_comm_debug_recent.jsonl"
  recent_status:
    bt_age_seconds: float | None
    comm_age_seconds: float | None
    stale_warning: str | None
  order_detected:
    bt: newest-first | oldest-first | single | unknown
    comm: newest-first | oldest-first | single | unknown
  all_tick_ids: [int, ...]  (sorted ascending, from bt file)
  selected_tick:
    bt_entries: list of {raw_line, parsed, parse_error}
    comm_entries: list of {raw_line, parsed, parse_error}
  neighbors:
    <tick_id>:
      bt_entries: list
      comm_entries: list
  warnings: [str, ...]
```

---

## 12. Prompt Contracts

### 12.0 Neighbor Raw Policy

Every prompt that receives neighbor raw lines must follow this policy:

```text
Selected tick raw lines are the primary evidence.
Neighbor raw lines are optional context only.
Do not attribute a neighbor tick command, decision, or blackboard value to the selected tick.
Use neighbor raw only when needed for alignment, continuity, repeated-command context, or incomplete selected-tick evidence.
When using neighbor evidence, explicitly name the neighbor tick_id.
```

### 12.1 `explain_tick` Prompt Contract

System intent:

```text
You explain code-theoretical behavior from project context and raw BT/ROS logs.
Do not use or infer from physical user observation.
Do not diagnose root cause.
Preserve concrete raw fields such as bt_node, semantic_source, target, ros_node, and payload.
State what the logs prove and what they do not prove.
Use selected tick raw as primary evidence. Neighbor raw is optional and must be explicitly labeled if used.
```

### 12.2 `compare_tick` Prompt Contract

System intent:

```text
You compare explain_tick theoretical behavior with user physical observation.
List matches, mismatches, and unclear items.
Do not make final root-cause claims.
Map each mismatch to a specific theoretical behavior item.
If user physical observation is missing, do not produce compare_tick.
```

### 12.3 `diagnose_tick` Prompt Contract

System intent:

```text
You diagnose based on compare_tick plus raw logs.
Always give both layer-level and concrete field-level attribution.
Mention specific BT nodes, conditions, blackboard keys, ROS targets, semantic sources, ros_nodes, and payloads when available.
Explain why less likely layers are less likely.
End with targeted next verification questions.
If user physical observation is missing, do not produce diagnose_tick.
Use selected tick raw as primary evidence; cite neighbor tick_id explicitly if neighbor evidence is needed.
```

---

## 13. User-Facing Report Template

When `user_observation` is present:

```text
Tick <id> Analysis

1. explain_tick
<theoretical behavior from raw logs>

2. compare_tick
<matches / mismatches / uncertain items>

3. diagnose_tick
<likely layer + concrete raw-field attribution + next verification>

You can add more observation for the same tick, for example:
- what rqt/pytreeview showed
- whether /walking/is_walking changed
- whether the head servo physically moved
- whether the line was visible in the camera image
```

When `user_observation` is missing:

```text
Tick <id> Explain Only

1. explain_tick
<theoretical behavior from raw logs>

To continue diagnosis, please describe what the robot physically did at this tick:
- body: stopped / walking / turning / falling / shaking
- head: still / moving / sweeping / wrong direction
- visible scene: line / ball / goal / obstacle visible or not
- optional: rqt/pytreeview active node and blackboard values
```

---

## 14. Implementation Status (COMPLETED 2026-04-11)

All five phases complete.

### Phase 1: Raw Retrieval ✓

Created:

```text
ainex_agent_tools/bt_analysis/__init__.py
ainex_agent_tools/bt_analysis/raw_tick.py
```

Implemented: `_path`, `_file_status`, `_read_jsonl_raw`, `_detect_order`,
`_group_by_tick`, `_resolve_tick_id`, `_select_neighbors`, `get_raw_tick_bundle`.

Key implementation notes:
- Entry fields: `raw_line`, `parsed`, `parse_error` (not `event`).
- `_file_status` returns `exists`, `age_seconds`, `stale` (no `size`/`mtime`).
- `order_detected` is per-file dict: `{"bt": ..., "comm": ...}`.
- `_resolve_tick_id` takes grouped dict, not a set.

### Phase 2: Tool Skeleton ✓

Created: `ainex_agent_tools/tools/bt_tick_analysis.py`

`get_bt_tick_raw` and `analyze_bt_tick` exported from `ainex_agent_tools/__init__.py`.
Inserted after `read_bt_obs`, before disabled stubs. Total: 12 tools.

### Phase 3: MVP Prompt Scaffold ✓

`analyze_bt_tick` returns project context + raw tick bundle + staged instructions.
`analysis_mode = "explain_only"` when `user_observation` is empty; `"full"` otherwise.
Default project context is **high-level only** — no per-node implementation detail.
Neighbor ticks included in output but labeled as context-only; usage gated by rules.

### Phase 4: Prompt Update ✓

Updated `ainex_agent_tools/prompts.py`:
- `about_your_capabilities`: `analyze_bt_tick` and `get_bt_tick_raw` added before `read_bt_obs`.
- `critical_instructions`: rules 5–7 added (one-tick routing, ros_out ≠ execution, neighbor policy).

### Phase 5: Test with Existing Marathon Logs ✓

Verified against `/opt/ainex_bt_log/` (stale recent files, tick_id 1614–1618).
`get_bt_tick_raw.invoke({"tick_id": "latest", "include_neighbors": 1})` returns correct tick 1618 with neighbor 1617.
`analyze_bt_tick.invoke({...})` returns correct `explain_only` / `full` mode depending on `user_observation`.

---

## 15. Design Constraints

1. Raw logs are primary evidence.
   Do not replace them with vague summaries before diagnosis.

2. Theoretical behavior is not physical proof.
   A `ros_out` entry proves command emission, not command execution.

3. Diagnose must be concrete.
   Always include target, payload, semantic_source, and node when available.

4. RQT/pytreeview is alignment evidence.
   If it conflicts with raw logs, resolve source alignment before hardware diagnosis.

5. Follow-up observations should refine diagnosis.
   Additional user observation should not require rereading code unless project context changes.

---

## 16. Future Extension to Cross-Tick Analysis

The one-tick design should remain compatible with cross-tick analysis.

Future cross-tick flow:

```text
raw tick bundle[]
  -> explain_episode
  -> compare_episode
  -> diagnose_episode
```

The same principles apply:

- Raw logs remain primary evidence.
- User observation validates physical trajectory.
- RQT/pytreeview validates BT/UI alignment.
- Diagnosis must include both layer and concrete raw fields.
