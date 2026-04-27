# Skill: `xyz-bt-edu-extend`

Extend the **xyz_bt_edu** shared BT library by adding standard input adapters,
L1 condition nodes, or L2 action/strategy nodes.

This `SKILL.md` is the source of truth for future `xyz_bt_edu` extensions.
Do not treat existing non-conforming legacy code as precedent. New work must follow
the rules below.

**Scope**: this skill targets `xyz_bt_edu` only.
For project-level work under `xyz_behavior/`, use the `xyz-bt-facade-project` skill.

---

## Trigger Phrases

| Phrase | Mode |
|---|---|
| "加L1节点", "add L1 condition", "new condition node", "new L1 node" | Mode 1 — Add L1 node |
| "加L2节点", "add L2 action", "new action node", "new L2 node" | Mode 2 — Add L2 node |
| "加 input adapter", "add adapter", "new sensor", "subscribe to /topic", "新传感器" | Mode 3 — Add input adapter |

If ambiguous, ask:
"你是要给 xyz_bt_edu 加标准 L1 条件节点、L2 动作节点，还是 input adapter？"

---

## Key Paths

Paths are relative to the ROS source workspace root that contains `xyz_bt_edu/`.
In this repo layout that is commonly `docker/ros_ws_src/`.

| Path | Role |
|---|---|
| `xyz_bt_edu/src/xyz_bt_edu/base_node.py` | BT node base classes: `XyzBTNode`, `XyzL1ConditionNode`, `XyzL2ActionNode` |
| `xyz_bt_edu/src/xyz_bt_edu/base_adapter.py` | Input adapter base class: `XyzInputAdapter` |
| `xyz_bt_edu/src/xyz_bt_edu/blackboard_keys.py` | BB key constants; always read first |
| `xyz_bt_edu/src/xyz_bt_edu/base_facade.py` | L2 facade abstract interface |
| `xyz_bt_edu/src/xyz_bt_edu/behaviours/L1_perception/` | L1 condition output dir |
| `xyz_bt_edu/src/xyz_bt_edu/behaviours/L2_locomotion/` | L2 action/strategy output dir |
| `xyz_bt_edu/src/xyz_bt_edu/input_adapters/` | Input adapter output dir |
| `xyz_bt_edu/config/` | Generic calibration/config files owned by `xyz_bt_edu` |
| `xyz_bt_edu/package.xml` | ROS package dependencies |
| `xyz_bt_edu/CMakeLists.txt` | Install config files if needed |
| `xyz_bt_edu/xyz_bt_edu_spec.md` | Must update after any public change |

Templates:
- `assets/templates/l1_node.py.tpl`
- `assets/templates/l2_node.py.tpl`
- `assets/templates/input_adapter.py.tpl`

---

## Design Rules — Adapter / L1 / L2 Responsibility Boundary

`xyz_bt_edu` is a shared low-level BT package. Standard nodes and adapters in this
package must not depend on project-specific code or AiNex application-layer packages.

Allowed dependencies:
- Python standard library
- ROS message/runtime packages declared in `xyz_bt_edu/package.xml`
- `py_trees`
- modules inside `xyz_bt_edu`

Forbidden imports:
- `xyz_behavior.*`
- `ainex_sdk.*`
- `bt_observability.*` from adapter/node files
- competition/project/application-specific modules

Message-only packages such as `ainex_interfaces.msg` are allowed only when declared
in `package.xml`.

If calculation logic is needed, first implement it in the current adapter/node file
as a small side-effect-free helper. If the logic is genuinely generic and reused,
place it inside `xyz_bt_edu`, not in another AiNex package.

Side-effect-free helper means:
- no BB reads/writes
- no ROS publish/subscribe/service calls
- no rospy logging (loginfo / logwarn / logerr / logdebug)
- no facade calls
- no mutation of live adapter state (including callback-thread-only accumulators)
- no project-specific imports

---

## Observability Rules

This skill must stay aligned with `xyz_behavior/bt_observability_framework.md`.

Observability ownership:
- `app/<project>_bt_node.py` creates `DebugEventLogger` and injects `logger` /
  `tick_id_getter` into adapters, nodes, facade, and visitor.
- Adapter/node files in `xyz_bt_edu` must not import `bt_observability.*` directly.
- `logger=None` must be safe and zero-cost; every manual emit must be guarded.
- L1 nodes must inherit `XyzL1ConditionNode`; L2 nodes must inherit
  `XyzL2ActionNode`. Both classes indirectly inherit `XyzBTNode`, so the
  py_trees visitor still sees standard node identity/status behavior.
- `XyzBTNode` is only a node-side helper base: logger storage, tick_id access,
  and `emit_bt` / `emit_decision` helpers. It must not own RUN/PAUSE/STEP control,
  global tick logging, `/bt_node_events`, or `BTDebugVisitor` responsibilities.
- Input adapters must inherit `XyzInputAdapter`. They are not BT nodes, must not
  inherit `XyzBTNode`, and never return `py_trees.common.Status`.
- Adapter `write_snapshot()` is the only business-in emit site and may emit only
  `ros_in` and `input_state` via `XyzInputAdapter` helpers.
- L1/L2 nodes may emit `decision` via `self.emit_decision()`.
- L2 nodes may emit `action_intent` via `self.emit_action_intent()` in
  `initialise()` when useful.
- L1/L2 nodes must never call `logger.emit_comm()`.
- `ros_out` and `ros_result` are emitted only by project `_RuntimeIO`.
- `tick_id` must come from the injected `tick_id_getter`; adapters/nodes must not maintain
  their own tick counters.

---

## Blackboard Rules

Always read `blackboard_keys.py` before adding or using BB keys.

Latched key convention:

```python
ROBOT_STATE_KEY = 'robot_state'                       # short/relative key
ROBOT_STATE     = LATCHED_NS + '/' + ROBOT_STATE_KEY  # '/latched/robot_state'
```

Rules:
- `BB.LATCHED_NS = '/latched'` is already defined; never redefine it.
- For `/latched/` clients, `register_key()` uses `BB.*_KEY`.
- `BB_READS`, `BB_WRITES`, compatibility `BB_LOG_KEYS`, docs, event payload keys,
  and ROSA topic maps use absolute `BB.*`.
- Root namespace keys have no `_KEY` suffix, e.g. `BB.HEAD_PAN_POS = '/head_pan_pos'`.
- Do not hard-code BB string paths in nodes/adapters when a `BB.*` constant should exist.
- If a required key is missing, add both `BB.*_KEY` and `BB.*` following the pattern.

Every new adapter/L1/L2 file should expose machine-checkable declarations:

```python
BB_READS = [...]
BB_WRITES = [...]
FACADE_CALLS = [...]
CONFIG_DEFAULTS = {...}
```

Use empty lists where not applicable.

---

## Defaults / Tuning Rules

Every adapter, L1 condition, and L2 action must follow the two-layer defaults contract:

### 1. CONFIG_DEFAULTS (class-level declaration)
- Must include every threshold, speed, servo center, expected label, tolerance, frame count,
  profile name, ROI, deadband, and any other value that affects runtime classification or output.
- Used for review, documentation, ROSA/LLM inspection, and consistency verification.
- Must stay in sync with `__init__` default arguments.

### 2. `__init__` default arguments (runtime override mechanism)
- Every entry in CONFIG_DEFAULTS must have a matching `__init__` parameter (unless purely
  descriptive and never used in runtime logic, e.g. a string label for documentation only).
- Project trees override standard behavior by passing constructor arguments.
- Constructor stores values on instance fields, e.g. `self._threshold = threshold`.

### Rules
- Runtime logic (`update()`, `_evaluate()`, `_is_xxx()`, `_compute_xxx()`, `_classify_xxx()`,
  helper methods) must use `self._` instance fields, not raw literals or bare class constants.
- Class constants are allowed only for non-tunable symbolic values: state labels like
  `_ST_SWEEP = 'sweep'`, internal protocol names, etc.
- YAML-loaded values: if a value is loaded from config YAML at runtime, CONFIG_DEFAULTS
  must still document the fallback/default value, and the docstring must state that
  config YAML is the single source of truth.
- CONFIG_DEFAULTS and `__init__` defaults must stay in sync; mismatches are a conformance violation.

---

## Input Adapter Rules

Each input adapter converts one ROS input stream into one documented set of blackboard
facts.

Input adapters must inherit `XyzInputAdapter` from
`xyz_bt_edu.base_adapter`. They are independent business-in
components, not BT behaviours:

- no `XyzBTNode` inheritance
- no `update()`
- no `Status.SUCCESS` / `Status.FAILURE` / `Status.RUNNING`
- no `emit_bt()`

Adapters own:
- input normalization
- input filtering
- sensor-level classification/judgement

Adapters must not own:
- BT decisions
- action strategy
- project behavior logic

Each adapter file must include a top-level docstring declaring:

1. ROS topic(s) subscribed.
2. ROS message type(s) received.
3. BB keys written, using `BB.*` names.
4. For each written BB value:
   - source input field(s)
   - extraction/classification helper used
   - decision standard or threshold
   - final BB key written
5. Whether thresholds/calibration values come from CONFIG_DEFAULTS or
   `xyz_bt_edu/config/*.yaml`.

Each adapter must implement explicit side-effect-free helpers that show the full path
from input to BB writes:

```python
def _extract_xxx(self, msg):
    """Extract relevant raw data from the ROS message."""

def _classify_xxx(self, extracted):
    """Apply the documented sensor-level rules/thresholds."""

def _make_bb_writes(self, classified) -> dict:
    """Return {BB.KEY: value}, or equivalent snapshot fields."""
```

`_callback()` should only:
- receive the ROS message
- call extraction/classification helpers
- apply any returned counter/accumulator updates to callback-thread-only fields
  (no lock needed for callback-thread-only state)
- call rospy.loginfo/logwarn if a detected state change warrants logging
- update live shared state under the shared lock via `_apply_live_writes()`
- increment `received_count`

If `_classify_xxx()` manages frame counters (e.g. for hysteresis/debounce), it must
accept current counter values as parameters and return updated counter values together
with the result. Example signature:

    def _classify_xxx(self, extracted, count_a: int, count_b: int) -> tuple:
        # returns (new_count_a, new_count_b, result_or_none)

`_callback()` unpacks the tuple and writes the returned counters to self._count_*
fields before calling `_make_bb_writes()`. This keeps `_classify_xxx()` side-effect-free
while supporting counter-based debounce logic.

`write_snapshot()` should only:
- write the already-computed snapshot to BB
- emit `ros_in` and `input_state` observability events

Every written BB value must be traceable in the same file:

```text
ROS msg input -> helper function -> rule/threshold -> snapshot field -> BB key
```

Hard-coded thresholds are forbidden inside callback logic. Use CONFIG_DEFAULTS
or `xyz_bt_edu/config/*.yaml`, and document default values.

Adapters must keep the two-phase latch protocol:

1. `snapshot_and_reset()` is called while the caller holds the shared lock.
2. `write_snapshot(snap, tick_id)` is called after releasing the lock, on the main thread.

Phase 1 for all adapters must happen inside the same `with lock:` block so the BT tree
sees a consistent frozen view of all sensor inputs per tick.

---

## L1 Node Rules

L1 nodes are pure condition nodes. They answer `Is...`, `Has...`, or `Can...`
questions based on blackboard state.

L1 nodes must:
- use condition-form class names, e.g. `L1_Balance_IsStanding`,
  `L1_Vision_HasTarget`, `L1_Motion_CanStart`
- inherit `XyzL1ConditionNode`
- read BB only for condition evaluation
- never write BB keys
- never call facade methods
- never publish/subscribe ROS
- never dispatch actions or cause external side effects
- never call `logger.emit_comm()`
- normally return only `Status.SUCCESS` or `Status.FAILURE`

Each L1 file must include a top-level docstring declaring:

1. BB keys read, using `BB.*` names.
2. The question/condition being judged.
3. The helper function used for judgement.
4. The `SUCCESS` / `FAILURE` standard.
5. CONFIG_DEFAULTS entries: thresholds, expected states, frame counts, labels,
   center values, tolerances, etc.

Each L1 node must implement one explicit judgement helper in the same file:

```python
def _is_xxx(self, value) -> bool:
    """Return True when the documented condition passes."""
```

or:

```python
def _evaluate(self, ...) -> tuple:
    """Return (passed, reason) for observability logging."""
```

Thresholds, expected labels, expected states, center values, tolerances, and frame
counts must be constructor parameters with safe defaults. Project trees may override
them when instantiating the node.

Avoid hard-coded judgement constants inside `update()`. `update()` should mainly:
- read BB values
- call `_is_xxx()` or `_evaluate()`
- convert the result to `Status.SUCCESS` / `Status.FAILURE`
- emit a `decision` log with `self.emit_decision()`

Any node that writes BB or performs an action is not a valid L1 node.

---

## L2 Node Rules

L2 nodes are action/strategy nodes. They may read BB as action context, compute
strategy, optionally write documented action-state/correction keys, and dispatch
external side effects only through `XyzBTFacade`.

L2 nodes must:
- inherit `XyzL2ActionNode`
- route all hardware/ROS side effects through `XyzBTFacade`
- never directly publish/subscribe ROS
- never call `logger.emit_comm()`
- never import project-specific packages
- keep generic strategy computation in the current node file or inside `xyz_bt_edu`
- expose all tuning values in CONFIG_DEFAULTS so project trees can override them via constructor args
- document every BB read/write and facade method call

Each L2 file must include a top-level docstring declaring:

1. BB keys read.
2. BB keys written, or `none`.
3. Facade method(s) called.
4. The action strategy being computed.
5. Helper function(s) used for computation.
6. CONFIG_DEFAULTS entries: thresholds, speeds, yaw limits, servo centers, state labels,
   frame counts, etc.
7. Return semantics: when it returns `RUNNING`, `SUCCESS`, or `FAILURE`.

Every non-trivial L2 node must implement explicit side-effect-free helpers in the same file:

```python
def _compute_command(self, ...) -> dict:
    """Compute generic action parameters from BB inputs and constructor settings."""
```

or:

```python
def _select_action(self, ...) -> tuple:
    """Return (facade_method_name, kwargs, reason)."""
```

`update()` should mainly:
- read BB values
- call `_compute_command()` or `_select_action()`
- call the selected facade method
- perform documented BB writes
- emit a `decision` log if logger is present
- return the documented `Status`

`initialise()` may emit `action_intent` via `self.emit_action_intent()` for action
start observability. It must not emit `ros_out`; `ros_out` belongs to `_RuntimeIO`.

Hard-coded strategy constants are forbidden inside `update()`. Strategy constants must
live in:
- CONFIG_DEFAULTS (and matching `__init__` args stored on self._)
- `xyz_bt_edu/config/*.yaml`

Allowed L2 BB writes:
- action-state keys owned by the node and documented in the file
- recovery/status correction keys required to keep adapter live state and latched BB
  state consistent
- documented root-level coordination keys read by L1, e.g. head-pan position

L2 nodes must not hide project-specific policy in `RuntimeFacade`.
`RuntimeFacade` translates generic action requests into project ROS communication.
Generic computation such as yaw selection, deadband checks, profile selection, and
sweep state machines belongs in the L2 node or a generic `xyz_bt_edu` helper.
`_RuntimeIO` is the sole raw ROS / manager layer; L2 nodes must never call it directly.

### L2 BT-visible State Synchronization

If a L2 facade call changes robot state that is represented by a standard BB key
and read by other BT nodes, the L2 must either:
1. Write the documented BB key in the same tick as the facade call, or
2. Explicitly document in the docstring why this action does not update BT-visible state.

For command-only state without sensor feedback, the BB value should represent the
commanded target and must be documented as "commanded state, not sensor feedback".

---

## Facade Contract Rule for L2

New L2 nodes should call existing `XyzBTFacade` methods.

Currently available facade methods (full public contract):

Primitives:
- `disable_gait(bt_node, tick_id)` — shut down gait controller (needs enable_gait to restart)
- `enable_gait(bt_node, tick_id)` — start/restart gait controller
- `stop_gait(bt_node, tick_id)` — stop current motion; controller stays up
- `set_step(dsp, x, y, yaw, gait_param, arm_swap, step_num, ...)` — fully-resolved gait step
- `run_action(action_name, bt_node, tick_id)` — named stand-alone motion action
- `set_servos_position(duration_ms, positions, bt_node, tick_id)` — direct servo command
- `publish_buzzer(freq, on_time, off_time, repeat, bt_node, tick_id)` — buzzer pattern (scalars only)

Convenience wrappers:
- `go_step(x, y, yaw, step_num=0, bt_node, tick_id, semantic_source)` — go profile step
- `turn_step(x, y, yaw, step_num=0, bt_node, tick_id, semantic_source)` — turn profile step
- `move_head(pan_pos, bt_node, tick_id)` — head servo position

Removed from contract (do not call):
- `stop_walking` — replaced by `stop_gait`
- `recover_from_fall` — logic now inline in `L2_Balance_RecoverFromFall`
- `follow_line` — removed (algorithm lives in `L2_Gait_FollowLine`)
- `gait_step` — removed (use `go_step` / `turn_step` directly)

If a required facade method does not exist, stop before editing and ask the user to
choose one option:

1. Redesign the L2 node to use existing facade methods.
2. Add a new abstract facade method and perform a breaking interface migration.
3. Move this behavior to project-level code instead of `xyz_bt_edu`.

If the user chooses option 2, the change is a breaking contract migration. It must
update, in the same task:

- `xyz_bt_edu/src/xyz_bt_edu/base_facade.py`
- every existing `xyz_behavior/*/runtime/runtime_facade.py`
- the project scaffold template `runtime_facade.py.tpl` in `xyz-bt-facade-project`
- `_runtime_io.py.tpl` if new ROS communication is also required
- `xyz_bt_edu/xyz_bt_edu_spec.md`
- relevant import/build checks

---

## Mode 1 — Add L1 Condition Node

### Information to collect

1. **name** — `L1_Module_Is/Has/CanDescription`
2. **question** — one sentence, e.g. "is the robot standing?"
3. **BB keys it reads** — from `blackboard_keys.py`
4. **judgement helper** — `_is_xxx()` or `_evaluate()`
5. **CONFIG_DEFAULTS** — thresholds, expected states, labels, frame counts, etc.
6. **SUCCESS/FAILURE semantics**

### Workflow

1. Read `blackboard_keys.py`.
2. Verify every read key exists and follows BB conventions.
3. Scan `input_adapters/` or documented L2 writes to verify something writes each read key.
   If no writer exists, warn and recommend adding an adapter or documented L2 writer first.
4. Expand `assets/templates/l1_node.py.tpl` to generate the node at:
   `xyz_bt_edu/src/xyz_bt_edu/behaviours/L1_perception/{{CLASS_NAME}}.py`
5. Verify structural conformance: the generated file must be structurally
   identical to `assets/templates/l1_node.py.tpl` — same `__init__` parameter
   order, same class-level declarations, same helper structure, same `update()`
   skeleton. Any structural deviation is a conformance violation.
6. Ensure the file satisfies all L1 rules:
   - class name is condition-form
   - `LEVEL = 'L1'`
   - inherits `XyzL1ConditionNode`
   - `BB_READS` and `BB_WRITES = []`
   - `FACADE_CALLS = []`
   - no `Access.WRITE`
   - no facade argument
   - no ROS subscribe/publish
   - no `logger.emit_comm()`
   - has `_is_xxx()` or `_evaluate()`
   - judgement constants are in CONFIG_DEFAULTS and stored on self._
   - no unresolved template `TODO` or `NotImplementedError`
7. Update `L1_perception/__init__.py` only if that package uses explicit imports.
8. Update `xyz_bt_edu_spec.md` with file path, signature, BB reads, judgement rules,
   defaults, and version history.

### L1 Checklist

```text
✅ BB keys use BB.* / BB.*_KEY conventions
✅ Top-level docstring declares reads, question, helper, SUCCESS/FAILURE, defaults
✅ Inherits XyzL1ConditionNode
✅ BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS declared
✅ Reads BB only for condition evaluation; no Access.WRITE
✅ No facade, ROS publish/subscribe, emit_comm, or action dispatch
✅ _is_xxx() or _evaluate() exists in the same file
✅ update() reads BB -> helper -> Status -> self.emit_decision()
✅ CONFIG_DEFAULTS exposes all thresholds/labels/states/tolerances
✅ All judgement thresholds, expected states/labels, centres, tolerances, and frame counts are listed in CONFIG_DEFAULTS and exposed as __init__ args stored on self
✅ _is_xxx() / _evaluate_xxx() helpers use self._ fields, not hard-coded literals
✅ No unresolved template TODO or NotImplementedError remains
✅ xyz_bt_edu_spec.md updated
✅ File structure matches l1_node.py.tpl (param order, class decls, helpers, update skeleton)
```

---

## Mode 2 — Add L2 Action/Strategy Node

### Information to collect

1. **name** — `L2_Module_ActionDescription`
2. **what action/strategy it performs** — one sentence
3. **BB keys it reads**
4. **BB keys it writes** — or `none`
5. **facade method(s) it calls**
6. **strategy helper** — `_compute_command()` or `_select_action()`
7. **CONFIG_DEFAULTS** — thresholds, speeds, yaw limits, servo centers, labels, etc.
8. **return semantics** — `RUNNING`, `SUCCESS`, `FAILURE`

### Workflow

1. Read `blackboard_keys.py`.
2. Read `base_facade.py`.
3. Verify BB read/write keys exist and follow conventions; add missing keys only when
   they are part of the intended public contract.
4. Verify every facade method exists. If not, follow the Facade Contract Rule.
5. Scan `input_adapters/` and documented L2 writes to verify something writes each read key.
6. Expand `assets/templates/l2_node.py.tpl` to generate the node at:
   `xyz_bt_edu/src/xyz_bt_edu/behaviours/L2_locomotion/{{CLASS_NAME}}.py`
7. Verify structural conformance: the generated file must be structurally
   identical to `assets/templates/l2_node.py.tpl` — same `__init__` parameter
   order, same class-level declarations, same helper structure, same `update()`
   skeleton. Any structural deviation is a conformance violation.
8. Ensure the file satisfies all L2 rules:
   - `LEVEL = 'L2'`
   - inherits `XyzL2ActionNode`
   - `BB_READS`, `BB_WRITES`, `FACADE_CALLS`, `CONFIG_DEFAULTS` declared
   - no direct ROS publish/subscribe
   - no `logger.emit_comm()`
   - no project-specific imports
   - non-trivial strategy uses `_compute_command()` or `_select_action()`
   - strategy constants are in CONFIG_DEFAULTS (or config YAML) and stored on self._
   - facade calls are explicit and documented
   - no unresolved template `TODO` or `NotImplementedError`
9. Update `L2_locomotion/__init__.py` only if that package uses explicit imports.
10. Update `xyz_bt_edu_spec.md` with file path, signature, BB reads/writes, facade calls,
    strategy rules, defaults, return semantics, and version history.

### L2 Checklist

```text
✅ BB keys use BB.* / BB.*_KEY conventions
✅ Top-level docstring declares reads, writes, facade calls, strategy, helpers, defaults, returns
✅ Inherits XyzL2ActionNode
✅ BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS declared
✅ All hardware/ROS side effects go through XyzBTFacade
✅ No direct ROS publish/subscribe or emit_comm
✅ No project-specific imports
✅ _compute_command() or _select_action() exists for non-trivial strategy
✅ update() reads BB -> helper -> facade/write BB -> self.emit_decision() -> Status
✅ action_intent, if used, is emitted via self.emit_action_intent() in initialise()
✅ CONFIG_DEFAULTS exposes all tuning values
✅ Facade contract checked; breaking changes synchronized if approved
✅ No unresolved template TODO or NotImplementedError remains
✅ xyz_bt_edu_spec.md updated
✅ If a L2 reads and writes the same BB key, register with Access.WRITE only — py_trees 2.1.6 Access is enum.Enum (not IntFlag), so Access.READ | Access.WRITE raises TypeError; Access.WRITE implicitly grants read
✅ All strategy/tuning constants are listed in CONFIG_DEFAULTS and exposed as __init__ args stored on self
✅ update() and helper methods use self._ instance fields, not raw literals
✅ CONFIG_DEFAULTS matches __init__ defaults (values and keys)
✅ No hard-coded tuning literals in update(), helper, or _compute/_evaluate methods
✅ Project tree can override every tuning/calibration value through constructor args or documented config YAML
✅ Facade actions that change BT-visible state either update the documented BB key in the same tick, or explicitly document why no BB update is performed
✅ Commanded-target BB writes are documented as commanded state, not sensor feedback
✅ When L2 writes BB, emit_decision() includes bb_writes={BB.KEY: value} in extra fields or the write is explicitly noted in reason
✅ File structure matches l2_node.py.tpl (param order, class decls, helpers, update skeleton)
```

---

## Mode 3 — Add Input Adapter

### Information to collect

1. **ROS topic** — e.g. `/camera/depth`
2. **message type** — e.g. `sensor_msgs/Image`
3. **what to extract** — e.g. "average depth of center ROI"
4. **sensor-level judgement/classification standard**
5. **BB keys to write**
6. **adapter class name** — e.g. `DepthCameraAdapter`
7. **CONFIG_DEFAULTS** — thresholds, ROI, offsets, labels, etc.

### Workflow

1. Read `blackboard_keys.py`.
2. Add missing BB key constants:

   ```python
   SOME_VALUE_KEY = 'some_value'
   SOME_VALUE     = LATCHED_NS + '/' + SOME_VALUE_KEY
   ```

3. Check `package.xml` for the message package. If missing, add the required
   `<exec_depend>` and, when needed, `<build_depend>`.
4. If calibration/config values are needed, use CONFIG_DEFAULTS or add a YAML file
   under `xyz_bt_edu/config/`. If adding `config/` for install/deploy, update
   `CMakeLists.txt`:

   ```cmake
   install(DIRECTORY config/
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
   ```

5. Expand `assets/templates/input_adapter.py.tpl` to generate the adapter at:
   `xyz_bt_edu/src/xyz_bt_edu/input_adapters/{{class_name_snake}}.py`
6. Verify structural conformance: the generated file must be structurally
   identical to `assets/templates/input_adapter.py.tpl` — same `__init__`
   parameter order, same class-level declarations, same helper structure,
   same `_callback()` / `snapshot_and_reset()` / `write_snapshot()` skeleton.
   Any structural deviation is a conformance violation.
7. Ensure the file satisfies all Input Adapter rules:
   - inherits `XyzInputAdapter`, not `XyzBTNode`
   - top-level docstring fully traces input -> BB writes
   - `BB_READS = []`, `BB_WRITES`, `FACADE_CALLS = []`, `CONFIG_DEFAULTS` declared
   - `_extract_xxx()`, `_classify_xxx()`, `_make_bb_writes()` exist
   - `_callback()` only receives, calls helpers, updates live state under lock, increments count
   - `write_snapshot()` only writes snapshot and emits observability events
   - only `ros_in` and `input_state` are emitted via `XyzInputAdapter` helpers
   - no BT action strategy
   - no unresolved template `TODO` or `NotImplementedError`
8. Update `input_adapters/__init__.py` only if that package uses explicit exports.
9. Update `xyz_bt_edu_spec.md` with file path, topic, message type, BB writes,
   extraction/classification rules, defaults, integration notes, and version history.

### Adapter Integration Instructions

```text
1. Import in app/<project>_bt_node.py __init__():
   from xyz_bt_edu.input_adapters.<snake_name> import <ClassName>
   self._<var> = <ClassName>(
       lock=self.lock, logger=self._obs_logger,
       tick_id_getter=lambda: self._tick_id)

2. Add to run() two-phase latch block:
   with self.lock:
       ...
       <var>_snap = self._<var>.snapshot_and_reset()
   ...
   self._<var>.write_snapshot(<var>_snap, self._tick_id)

3. Sync bb_ros_bridge.py:
   Add BB mirror topics for each new BB key.

4. Sync infra_manifest.py:
   Add topic_sub record for the ROS topic.

5. Rebuild:
   catkin build xyz_bt_edu xyz_behavior
```

### Adapter Checklist

```text
✅ ROS topic and message type declared
✅ Inherits XyzInputAdapter, not XyzBTNode
✅ Every BB write declared with source fields, helper, rule/threshold, BB key
✅ BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS declared
✅ _extract_xxx() / _classify_xxx() / _make_bb_writes() exist
✅ _classify_xxx() contains no rospy logging and no self-state mutation;
   counter updates and rospy.loginfo() live in _callback()
✅ _callback() has no hidden BB writes or action strategy
✅ write_snapshot() only writes snapshot and emits ros_in/input_state
✅ No ros_out/ros_result emitted by adapter
✅ Thresholds are in CONFIG_DEFAULTS or xyz_bt_edu/config values
✅ Thresholds and calibration values are in CONFIG_DEFAULTS and exposed as __init__ args (or documented as YAML-backed with CONFIG_DEFAULTS providing the fallback)
✅ package.xml updated if message dependency is new
✅ CMakeLists.txt installs config/ if deployment needs it
✅ No unresolved template TODO or NotImplementedError remains
✅ xyz_bt_edu_spec.md updated
✅ File structure matches input_adapter.py.tpl (param order, class decls, helpers, callback/snapshot skeleton)
```

---

## Verification

Run focused verification after adding or modifying adapter/L1/L2 files.

Static checks to perform manually or with `rg`:

```bash
# No project/application package imports inside xyz_bt_edu
rg -n "from xyz_behavior|import xyz_behavior|from ainex_sdk|import ainex_sdk" xyz_bt_edu/src/xyz_bt_edu

# rospy.Subscriber only in input_adapters
rg -n "rospy\\.Subscriber" xyz_bt_edu/src/xyz_bt_edu

# L1 should use the L1 base and should not write BB or use facade
rg -n "class .*\\(XyzBTNode\\)|Access\\.WRITE|_facade|facade:|emit_comm" xyz_bt_edu/src/xyz_bt_edu/behaviours/L1_perception

# L2 should use the L2 base and should not directly publish/subscribe ROS
rg -n "class .*\\(XyzBTNode\\)|rospy\\.Subscriber|\\.publish\\(|ServiceProxy|rospy\\.Publisher|emit_comm" xyz_bt_edu/src/xyz_bt_edu/behaviours/L2_locomotion

# Input adapters should use the input adapter base, not BT node bases
rg -n "XyzBTNode|XyzL1ConditionNode|XyzL2ActionNode" xyz_bt_edu/src/xyz_bt_edu/input_adapters

# Adapter/node files should not import bt_observability directly
rg -n "bt_observability" xyz_bt_edu/src/xyz_bt_edu

# Generated files should not retain template placeholders
rg -n "TODO|NotImplementedError|\\{\\{" xyz_bt_edu/src/xyz_bt_edu
```

Import check:

```bash
cd /home/pi/docker/ros_ws_src
python3 -c "
import sys
sys.path.insert(0, 'xyz_bt_edu/src')
from xyz_bt_edu.behaviours.L1_perception.<ClassName> import <ClassName>
print('import OK')
"
```

Build check:

```bash
docker exec ainex bash -c "
  cd /home/ubuntu/ros_ws && catkin build xyz_bt_edu
"
```

Verification checklist:

```text
✅ No forbidden project/application imports in xyz_bt_edu
✅ rospy.Subscriber appears only in input_adapters
✅ Adapter/node files do not import bt_observability directly
✅ L1 nodes inherit XyzL1ConditionNode
✅ L2 nodes inherit XyzL2ActionNode
✅ Input adapters inherit XyzInputAdapter and do not inherit XyzBTNode
✅ L1 nodes do not register Access.WRITE
✅ L1 nodes do not call facade methods
✅ L1/L2 nodes do not call emit_comm
✅ L2 nodes do not directly publish/subscribe ROS
✅ ros_out/ros_result are emitted only by comm_facade.py
✅ Adapter declared BB_WRITES matches actual snapshot writes
✅ Non-trivial L1/L2 helper functions are present and documented
✅ CONFIG_DEFAULTS covers all thresholds/tuning constants
✅ No unresolved template TODO / NotImplementedError / {{placeholder}} remains
✅ xyz_bt_edu_spec.md is updated
✅ Import/build checks completed or explicitly reported if environment blocks them
```
