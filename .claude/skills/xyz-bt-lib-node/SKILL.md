# Skill: `xyz-bt-lib-node`

Extend the **xyz_bt_lib** shared BT library by adding L1 condition nodes,
L2 action/strategy nodes, or L3 system/process-orchestration nodes.

This `SKILL.md` is the source of truth for L1, L2 and L3 node extensions in `xyz_bt_lib`.
Do not treat existing non-conforming legacy code as precedent. New work must follow
the rules below.

**Scope**: this skill targets `xyz_bt_lib` only.
For input adapters, use the `xyz-bt-lib-adapter` skill.
For project-level work under `xyz_behavior/`, use the `xyz-bt-facade-project` skill.

---

## Trigger Phrases

| Phrase | Mode |
|---|---|
| "加L1节点", "add L1 condition", "new condition node", "new L1 node" | Mode 1 — Add L1 node |
| "加L2节点", "add L2 action", "new action node", "new L2 node" | Mode 2 — Add L2 node |

If ambiguous, ask:
"你是要给 xyz_bt_lib 加标准 L1 条件节点，还是 L2 动作节点？"

---

## Key Paths

Paths are relative to the ROS source workspace root that contains `xyz_bt_lib/`.
In this repo layout that is commonly `docker/ros_ws_src/`.

| Path | Role |
|---|---|
| `xyz_bt_lib/src/xyz_bt_lib/core/base_node.py` | BT node base classes: `XyzBTNode`, `XyzL1ConditionNode`, `XyzL2ActionNode`, `XyzL2GaitActionNode`, `XyzL3ActionNode` |
| `xyz_bt_lib/src/xyz_bt_lib/core/base_adapter.py` | Input adapter base class: `XyzInputAdapter` |
| `xyz_bt_lib/src/xyz_bt_lib/blackboard/blackboard_keys.py` | BB key constants; always read first |
| `xyz_bt_lib/src/xyz_bt_lib/core/base_facade.py` | L2 facade abstract interface |
| `xyz_bt_lib/src/xyz_bt_lib/behaviours/L1_perception/` | L1 condition output dir |
| `xyz_bt_lib/src/xyz_bt_lib/behaviours/L2_locomotion/` | L2 action/strategy output dir |
| `xyz_bt_lib/src/xyz_bt_lib/behaviours/L3_system/` | L3 system/process-orchestration output dir |
| `xyz_bt_lib/src/xyz_bt_lib/adapters/` | Input adapter output dir |
| `xyz_bt_lib/package.xml` | ROS package dependencies |
| `xyz_bt_lib/CMakeLists.txt` | Install launch/scripts if needed |

There is no central spec document. **The module docstring is the authoritative
spec** — after any public change it must fully describe BB reads/writes,
judgement/strategy rules, and CONFIG_DEFAULTS.

Templates:
- `assets/templates/l1_node.py.tpl`
- `assets/templates/l2_node.py.tpl` (carries both the plain and the **gait** variant)
- `assets/templates/l3_node.py.tpl`

---

## Design Rules — Adapter / L1 / L2 Responsibility Boundary

`xyz_bt_lib` is a shared low-level BT package. Standard nodes and adapters in this
package must not depend on project-specific code or AiNex application-layer packages.

Allowed dependencies:
- Python standard library
- ROS message/runtime packages declared in `xyz_bt_lib/package.xml`
- `py_trees`
- modules inside `xyz_bt_lib`

Forbidden imports:
- `xyz_behavior.*`
- `ainex_sdk.*`
- `bt_observability.*` from adapter/node files
- competition/project/application-specific modules

Message-only packages such as `ainex_interfaces.msg` are allowed only when declared
in `package.xml`.

If calculation logic is needed, first implement it in the current adapter/node file
as a small side-effect-free helper. If the logic is genuinely generic and reused,
place it inside `xyz_bt_lib`, not in another AiNex package.

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
- Adapter/node files in `xyz_bt_lib` must not import `bt_observability.*` directly.
- `logger=None` must be safe and zero-cost; every manual emit must be guarded.
- L1 nodes must inherit `XyzL1ConditionNode`; L2 nodes must inherit
  `XyzL2ActionNode` (or `XyzL2GaitActionNode` — see § Gait nodes); L3 nodes must
  inherit `XyzL3ActionNode`. All of these indirectly inherit `XyzBTNode`, so the
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
- Adapters and L1/L2 nodes inside `xyz_bt_lib` must not write BB snapshot files directly.
  The current-state BB file (`xyz_behavior/log/bb_current.json`) is maintained centrally
  by the project/app node entry point (`app/<project>_bt_node.py`) via
  `BlackboardCurrentWriter` from `bt_observability`. Adapter and node code in `xyz_bt_lib`
  has no access to this file and must not import or call `BlackboardCurrentWriter`.
- The BB snapshot file is a current-state mirror only (latest tick), not a history log.
  Do not confuse it with JSONL observability (`bt_debug_*`), which is the correct channel
  for per-tick event sequences.

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
- Root namespace keys have no `_KEY` suffix and no `LATCHED_NS` prefix,
  e.g. `BB.TICK_ID = '/tick_id'`.
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

### Multi-target / composite BB values

- `/latched/` BB values may be scalar, message object, list, or dict.
- When a value is indexed by `target_id`, use plural naming:
    `TRACKED_OBJECTS_KEY = 'tracked_objects'`  — dict, not `'tracked_object'`
- Missing key = target absent. Default empty value for dict keys is `{}`.
  Do NOT use `{target_id: None}` as an absence marker — just omit the key.
- `blackboard_keys.py` is the single source of truth for all BB keys AND
  `ROSA_TOPIC_MAP`. Any new `/latched/` key intended for ROSA monitoring must
  be added to `ROSA_TOPIC_MAP` in the same edit.

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

## L1 Node Rules

L1 nodes are pure condition nodes. They answer `Is...`, `Has...`, or `Can...`
questions based on blackboard state.

L1 nodes are PURE PREDICATES. They must:
- use condition-form class names, e.g. `L1_Balance_IsStanding`,
  `L1_Vision_HasTarget`, `L1_Motion_CanStart`
- inherit `XyzL1ConditionNode`
- read BB only for condition evaluation
- never write BB keys
- never call facade methods
- never publish/subscribe ROS
- never dispatch actions or cause external side effects
- never call `logger.emit_comm()`
- return ONLY `Status.SUCCESS` or `Status.FAILURE` — **never `Status.RUNNING`**
- hold NO cross-tick state (no counters, no dwell, no hysteresis, no
  `initialise()`/`terminate()` bookkeeping)

Timing / stability confirmation is NOT an L1 responsibility. The condition stays a
stateless predicate and gets wrapped at the TREE layer by one of two BB-backed,
reactive-safe decorators — never by a counter inside the node. Do not add
`succeed_dwell_ticks` / `fail_dwell_ticks` or any accumulator to an L1 node.

- `core.latched_dwell.LatchedDwellDecorator` — **N-in / 1-out**. Latches SUCCESS
  after N consecutive passes; a single failing tick unlatches. Returns RUNNING
  while counting. Use for "wait until the world is stable, then commit".
- `core.hysteresis.HysteresisDecorator` — **N-in / M-out**, independent
  `enter_ticks` / `exit_ticks`, and it **never returns RUNNING**. Use when the
  signal flickers and the answer must be held steady (it is the replacement for
  the removed asymmetric `fail_dwell_ticks`).

Both store state at `/node_state/<state_key>`, so a `state_key` must be a
hardcoded literal and unique across every user of that namespace.

If the predicate genuinely needs two samples (e.g. "is the object still?"), the
cross-frame memory belongs in the ADAPTER, not the node — see
`carry_displacement()` in `core/base_adapter.py` and how
`L1_Vision_IsObjectStill` consumes the resulting `displacement` field.

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
- inherit `XyzL2ActionNode` — or `XyzL2GaitActionNode` when the node commands the
  gait (see below)
- route all hardware/ROS side effects through `XyzBTFacade`
- never directly publish/subscribe ROS
- never call `logger.emit_comm()`
- never import project-specific packages
- keep generic strategy computation in the current node file or inside `xyz_bt_lib`
- expose all tuning values in CONFIG_DEFAULTS so project trees can override them via constructor args
- document every BB read/write and facade method call

### Gait nodes: naming IS the contract

**A file named `L2_Gait_*.py` MUST inherit `XyzL2GaitActionNode`** (in
`core/base_node.py`). The prefix means "dispatches gait steps"
(`go_step()` / `turn_step()`), and `xyz_bt_lib_guard.py` enforces the pairing from
the filename alone — no judgement call about whether the node "really" steps.

The converse is equally binding: a node that never dispatches a step must NOT use
the `L2_Gait_` prefix. `L2_Motion_StopGait` and `L2_Motion_PauseAfterTicks` only
call `stop_gait()`, so they live outside the Gait namespace and keep the plain
`XyzL2ActionNode` base. Name new non-stepping nodes `L2_Motion_*` / `L2_Head_*` /
`L2_Balance_*`.

(Three nodes drifted from this rule in Aug 2026 because it existed only as prose
in this file — hence the hook.)

`XyzL2GaitActionNode` owns the gait plumbing that used to be copy-pasted into
every gait node:

```python
class L2_Gait_Foo(XyzL2GaitActionNode):
    CONFIG_DEFAULTS = {'x_speed': 0.01}      # strategy params ONLY

    def __init__(self, name='L2_Gait_Foo', facade=None, logger=None,
                 tick_id_getter=None, x_speed=0.01,
                 period_time_ms=None, dsp_ratio=None, y_swap_amplitude=None,
                 arm_swap=None, step_num=None, gait_param=None):
        super().__init__(name, facade=facade, logger=logger,
                         tick_id_getter=tick_id_getter,
                         period_time_ms=period_time_ms, dsp_ratio=dsp_ratio,
                         y_swap_amplitude=y_swap_amplitude, arm_swap=arm_swap,
                         step_num=step_num, gait_param=gait_param)
        self._x_speed = x_speed

    def update(self):
        self.call_facade('go_step', x=self._x_speed, y=0, yaw=0,
                         semantic_source='foo', **self.gait_kwargs())
```

Rules:
- Do **NOT** re-declare `period_time_ms` / `dsp_ratio` / `y_swap_amplitude` /
  `arm_swap` / `step_num` / `gait_param` in the node's `CONFIG_DEFAULTS` — they
  are inherited from `GAIT_CONFIG_DEFAULTS`. Declare only strategy params.
- Do **NOT** add per-knob arguments such as `step_height=` / `init_yaw_offset=` /
  `hip_pitch_offset=`. Those were removed Aug 2026; every WalkingParam key now
  travels in the single `gait_param` dict, e.g.
  `gait_param={'step_height': 0.03, 'init_yaw_offset': 2}`.
- Build the facade call with `**self.gait_kwargs()`; do not hand-assemble the six
  kwargs, and do not write your own `_effective_gait_param()`.

### L2 Node Kinds

Every L2 node belongs to exactly one of three execution kinds.
The kind must be declared in the top-level docstring as:

  Node kind: dispatch | finite_action | continuous_controller

**dispatch**
  Fire-and-forget. The command is accepted or dispatched this tick.
  `update()` always returns `SUCCESS`. Never returns `RUNNING`.
  Examples: L2_Motion_StopGait, L2_Head_MoveTo, L2_Gait_ControlToObject,
            L2_Motion_RunAction and L2_Balance_RecoverFromFall (their
            run_action() facade call BLOCKS, so they finish inside one tick)

**finite_action**
  Has an internal completion condition. Drives itself to a terminal state.
  `update()` returns `RUNNING` while in progress, `SUCCESS` when done
  (rarely `FAILURE` on unrecoverable error). Does NOT rely on external
  preemption to terminate.
  Examples: L2_Head_SearchSweep, L2_Gait_VisionToObject,
            L2_Gait_AlignBodyToHead, L2_Gait_AlignImuHeading

**continuous_controller**
  Owns continuous control while active. `update()` always returns `RUNNING`.
  Intended to run indefinitely; terminated by the tree structure (e.g. an
  upstream Selector re-evaluates when a condition passes). Never self-SUCCESS.
  Examples: L2_Gait_SearchTurn, L2_Gait_FollowTarget,
            L2_Head_TrackTarget, L2_Motion_PauseAfterTicks

Each L2 file must include a top-level docstring declaring:

1. Node kind: dispatch | finite_action | continuous_controller
2. BB keys read.
3. BB keys written, or `none`.
4. Facade method(s) called.
5. The action strategy being computed.
6. Helper function(s) used for computation.
7. CONFIG_DEFAULTS entries: thresholds, speeds, yaw limits, servo centers, state labels,
   frame counts, etc.
8. Return semantics: when it returns `RUNNING`, `SUCCESS`, or `FAILURE`.

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
live in CONFIG_DEFAULTS (and matching `__init__` args stored on `self._`).
There is no `xyz_bt_lib/config/` directory — it was removed Aug 2026 along with
`line_perception.yaml`. Calibration that a project must own goes in the project
layer and is threaded in as a constructor argument, never read from inside a
library node.

Allowed L2 BB writes:
- action-state keys owned by the node and documented in the file
- recovery/status correction keys required to keep adapter live state and latched BB
  state consistent
- documented root-level coordination keys read by L1, e.g. mission/task flags

### Consuming multi-target BB dicts

- L1/L2 nodes consuming multi-target BB values must accept a `target_id`
  constructor parameter and read `bb_dict.get(target_id)` each tick.
- Missing key (target absent) is the canonical absence representation.
  Do NOT treat `None` or `0` as absence — check `target_id in d`.
- Nodes must not re-interpret or re-compute fields already standardised by
  the adapter (`size`, `error_x`, `error_y`, `lost_count`).
- Nodes must not define a local dict schema different from the adapter's.

L2 nodes must not hide project-specific policy in `RuntimeFacade`.
`RuntimeFacade` translates generic action requests into project ROS communication.
Generic computation such as yaw selection, deadband checks, profile selection, and
sweep state machines belongs in the L2 node or a generic `xyz_bt_lib` helper.
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

**The authoritative method list is `xyz_bt_lib/core/base_facade.py`** — read it before
calling anything. It declares 10 abstract methods (7 primitives + the `go_step` /
`turn_step` / `move_head` wrappers) plus the concrete `start_process` / `stop_process`
pair that L3 nodes use.

This section deliberately does not restate the signatures. It used to, and the copy
drifted: `go_step`'s `step_num` was documented as `0` when the real default is `None`,
the five gait kwargs were missing entirely, `move_head` was missing `tilt_pos`, and
`start_process`/`stop_process` were absent — which would lead an agent doing L3 work to
conclude, wrongly, that the facade methods it needs do not exist. **Python's ABC checks
method NAMES only, never signatures**, so a drifted facade implementation still
instantiates and then fails at runtime.

For a gait node, do not pass gait kwargs by hand at all: inherit `XyzL2GaitActionNode`
and spread `**self.gait_kwargs()` (see § Gait nodes).

Removed from contract (do not call):
- `stop_walking` — replaced by `stop_gait`
- `recover_from_fall` — logic now inline in `L2_Balance_RecoverFromFall`
- `follow_line` — removed (algorithm lives in `L2_Gait_FollowTarget`)
- `gait_step` — removed (use `go_step` / `turn_step` directly)

If a required facade method does not exist, stop before editing and ask the user to
choose one option:

1. Redesign the L2 node to use existing facade methods.
2. Add a new abstract facade method and perform a breaking interface migration.
3. Move this behavior to project-level code instead of `xyz_bt_lib`.

If the user chooses option 2, the change is a breaking contract migration. It must
update, in the same task:

- `xyz_bt_lib/src/xyz_bt_lib/core/base_facade.py`
- every existing `xyz_behavior/*/runtime/runtime_facade.py`
- the project scaffold template `runtime_facade.py.tpl` in `xyz-bt-facade-project`
- `_runtime_io.py.tpl` if new ROS communication is also required
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
3. Scan `adapters/` or documented L2 writes to verify something writes each read key.
   If no writer exists, warn and recommend adding an adapter or documented L2 writer first.
4. Expand `assets/templates/l1_node.py.tpl` to generate the node at:
   `xyz_bt_lib/src/xyz_bt_lib/behaviours/L1_perception/{{CLASS_NAME}}.py`
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
8. Verify the module docstring is the complete spec: BB reads, judgement rules,
   CONFIG_DEFAULTS (there is no central spec doc — the docstring is authoritative).

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
✅ Module docstring complete (BB reads, judgement rules, CONFIG_DEFAULTS) — docstring is the spec
✅ File structure matches l1_node.py.tpl (param order, class decls, helpers, update skeleton)
```

---

## Mode 2 — Add L2 Action/Strategy Node

### Information to collect

1. **name** — `L2_Module_ActionDescription`
2. **node kind** — dispatch | finite_action | continuous_controller
3. **what action/strategy it performs** — one sentence
4. **BB keys it reads**
5. **BB keys it writes** — or `none`
6. **facade method(s) it calls**
7. **strategy helper** — `_compute_command()` or `_select_action()`
8. **CONFIG_DEFAULTS** — thresholds, speeds, yaw limits, servo centers, labels, etc.
9. **return semantics** — `RUNNING`, `SUCCESS`, `FAILURE`

### Workflow

1. Read `blackboard_keys.py`.
2. Read `base_facade.py`.
3. Verify BB read/write keys exist and follow conventions; add missing keys only when
   they are part of the intended public contract.
4. Verify every facade method exists. If not, follow the Facade Contract Rule.
5. Scan `adapters/` and documented L2 writes to verify something writes each read key.
6. Expand `assets/templates/l2_node.py.tpl` to generate the node at:
   `xyz_bt_lib/src/xyz_bt_lib/behaviours/L2_locomotion/{{CLASS_NAME}}.py`
7. Verify structural conformance: the generated file must be structurally
   identical to `assets/templates/l2_node.py.tpl` — same `__init__` parameter
   order, same class-level declarations, same helper structure, same `update()`
   skeleton. Any structural deviation is a conformance violation.
8. Ensure the file satisfies all L2 rules:
   - `LEVEL = 'L2'`
   - inherits `XyzL2ActionNode`
   - Node kind declared in docstring and consistent with update() return values
   - `BB_READS`, `BB_WRITES`, `FACADE_CALLS`, `CONFIG_DEFAULTS` declared
   - no direct ROS publish/subscribe
   - no `logger.emit_comm()`
   - no project-specific imports
   - non-trivial strategy uses `_compute_command()` or `_select_action()`
   - strategy constants are in CONFIG_DEFAULTS (or config YAML) and stored on self._
   - facade calls are explicit and documented
   - no unresolved template `TODO` or `NotImplementedError`
9. Update `L2_locomotion/__init__.py` only if that package uses explicit imports.
10. Verify the module docstring is the complete spec: BB reads/writes, facade calls,
    strategy rules, CONFIG_DEFAULTS, return semantics (no central spec doc — the
    docstring is authoritative).

### L2 Checklist

```text
✅ BB keys use BB.* / BB.*_KEY conventions
✅ Top-level docstring declares reads, writes, facade calls, strategy, helpers, defaults, returns
✅ Node kind declared in docstring (dispatch | finite_action | continuous_controller)
   and consistent with update() return values
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
✅ Module docstring complete (BB reads/writes, facade calls, strategy rules, CONFIG_DEFAULTS, return semantics)
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

### L3 (system / process-orchestration nodes)

L3 nodes orchestrate the lifecycle of *other* programs (start/stop a registered
background process, dispatch a system-level command) rather than commanding robot
motion. They are action-like and reuse the L2 machinery, so the workflow, template,
and checklist are identical to L2 with these substitutions:

- Expand `assets/templates/l3_node.py.tpl` (not the L2 template).
- Generate under `xyz_bt_lib/src/xyz_bt_lib/behaviours/L3_system/{{CLASS_NAME}}.py`.
- `LEVEL = 'L3'` and inherit `XyzL3ActionNode` (which subclasses `XyzL2ActionNode`,
  so `call_facade` / `emit_action_intent` / `emit_decision` all work identically).
- Update `L3_system/__init__.py` only if that package uses explicit imports.
- Everything else (BB_READS/BB_WRITES/FACADE_CALLS/CONFIG_DEFAULTS declared, no
  direct ROS, no emit_comm, no project imports, self._ fields not literals, spec
  update) is the same as the L2 checklist above.
- Reference implementation: `behaviours/L3_system/L3_Process_Control.py`.

---

## Verification

Run focused verification after adding or modifying adapter/L1/L2 files.

Static checks to perform manually or with `rg`:

```bash
# No project/application package imports inside xyz_bt_lib
rg -n "from xyz_behavior|import xyz_behavior|from ainex_sdk|import ainex_sdk" xyz_bt_lib/src/xyz_bt_lib

# rospy.Subscriber only in adapters
rg -n "rospy\\.Subscriber" xyz_bt_lib/src/xyz_bt_lib

# L1 should use the L1 base and should not write BB or use facade
rg -n "class .*\\(XyzBTNode\\)|Access\\.WRITE|_facade|facade:|emit_comm" xyz_bt_lib/src/xyz_bt_lib/behaviours/L1_perception

# L2 should use the L2 base and should not directly publish/subscribe ROS
rg -n "class .*\\(XyzBTNode\\)|rospy\\.Subscriber|\\.publish\\(|ServiceProxy|rospy\\.Publisher|emit_comm" xyz_bt_lib/src/xyz_bt_lib/behaviours/L2_locomotion

# Input adapters should use the input adapter base, not BT node bases
rg -n "XyzBTNode|XyzL1ConditionNode|XyzL2ActionNode" xyz_bt_lib/src/xyz_bt_lib/adapters

# Adapter/node files should not import bt_observability directly
rg -n "bt_observability" xyz_bt_lib/src/xyz_bt_lib

# Generated files should not retain template placeholders
rg -n "TODO|NotImplementedError|\\{\\{" xyz_bt_lib/src/xyz_bt_lib
```

Import check:

```bash
cd /home/pi/docker/ros_ws_src
python3 -c "
import sys
sys.path.insert(0, 'xyz_bt_lib/src')
from xyz_bt_lib.behaviours.L1_perception.<ClassName> import <ClassName>
print('import OK')
"
```

Build check:

```bash
docker exec ainex bash -c "
  cd /home/ubuntu/ros_ws && catkin build xyz_bt_lib
"
```

Verification checklist:

```text
✅ No forbidden project/application imports in xyz_bt_lib
✅ rospy.Subscriber appears only in adapters
✅ Adapter/node files do not import bt_observability directly
✅ L1 nodes inherit XyzL1ConditionNode
✅ L2 nodes inherit XyzL2ActionNode
✅ Input adapters inherit XyzInputAdapter and do not inherit XyzBTNode
✅ L1 nodes do not register Access.WRITE
✅ L1 nodes do not call facade methods
✅ L1/L2 nodes do not call emit_comm
✅ L2 nodes do not directly publish/subscribe ROS
✅ ros_out/ros_result are emitted only by _RuntimeIO
✅ Adapter declared BB_WRITES matches actual snapshot writes
✅ Non-trivial L1/L2 helper functions are present and documented
✅ CONFIG_DEFAULTS covers all thresholds/tuning constants
✅ No unresolved template TODO / NotImplementedError / {{placeholder}} remains
✅ Module docstring is the complete, up-to-date spec
✅ Import/build checks completed or explicitly reported if environment blocks them
```
