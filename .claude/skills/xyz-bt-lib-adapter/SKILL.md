---
name: xyz-bt-lib-adapter
description: Add or modify input adapters in xyz_bt_lib (xyz_bt_lib/src/xyz_bt_lib/adapters/) — the only place a rospy.Subscriber may appear inside the BT process. Use when subscribing to a new ROS topic, feeding new sensor data onto the Blackboard, or adding keys to blackboard_keys.py. 加 input adapter, 新传感器, subscribe to topic.
---

# Skill: `xyz-bt-lib-adapter`

Extend the **xyz_bt_lib** shared BT library by adding standard input adapters.

This `SKILL.md` is the source of truth for input adapter extensions in `xyz_bt_lib`.
Do not treat existing non-conforming legacy code as precedent. New work must follow
the rules below.

**Scope**: this skill targets `xyz_bt_lib` only.
For L1/L2 nodes, use the `xyz-bt-lib-node` skill.
For project-level work under `xyz_behavior/`, use the `xyz-bt-facade-project` skill.

---

## Trigger Phrases

| Phrase | Mode |
|---|---|
| "加 input adapter", "add adapter", "new sensor", "subscribe to /topic", "新传感器" | Mode 3 — Add input adapter |

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
| `xyz_bt_lib/src/xyz_bt_lib/adapters/` | Input adapter output dir |
| `xyz_bt_lib/package.xml` | ROS package dependencies |
| `xyz_bt_lib/CMakeLists.txt` | Install launch/scripts if needed |

There is no central spec document. **The module docstring is the authoritative
spec** — after any public change it must fully describe topic, message type,
BB writes, extraction/classification rules, and CONFIG_DEFAULTS.

Templates:
- `assets/templates/input_adapter.py.tpl`

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

### `tracked_objects` is the one visual-object registry

Every visual detector writes into `BB.TRACKED_OBJECTS` — colour blobs, YOLO
detections, AprilTags **and lines**. Do not invent a per-sensor set of flat keys;
that is exactly what the deleted `LineDetectionAdapter` did (six flat
`line_*` keys parsed from the same topic `ObjectDetectionAdapter` already read).
A new detector adds a `target_specs` entry, not a new BB namespace.

Two schema conventions a tracking adapter must honour:

- **`size` may be `None`.** Compute it per shape: `circle` → π·radius²,
  `rect` → width·height, anything else (a `line`) → `None`. Do not fall back to
  `width * height` for shapes whose `width/height` carry the IMAGE dimensions —
  that yields a constant ~307200 "area" that silently satisfies any `min_size`
  gate. Consumers treat `None` as "this shape has no area", not as zero.
- **`displacement`** — px moved since the previous consecutive detection, filled
  by `carry_displacement(new_entry, old_entry)` from `core/base_adapter.py`. Call
  it in the same merge function that maintains `lost_count`, and write `None`
  when there is no comparable previous sample (first sighting, re-acquisition
  after a loss, or any multi-instance entry with no cross-frame identity).
  This exists so `L1_Vision_IsObjectStill` can stay a pure predicate: motion is a
  two-sample question, and an L1 node may not hold cross-tick state.

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
- No YAML: `xyz_bt_lib` has no `config/` directory (deleted Aug 2026 with
  `line_perception.yaml`). A library adapter must never read a config file. Calibration
  that a project owns is passed in as a constructor argument and documented in
  CONFIG_DEFAULTS — e.g. `ObjectDetectionAdapter(..., center_x_offset=66)`.
- CONFIG_DEFAULTS and `__init__` defaults must stay in sync; mismatches are a conformance violation.

---

## Input Adapter Rules

Each input adapter converts one ROS input stream into one documented set of blackboard
facts.

Input adapters must inherit `XyzInputAdapter` from
`xyz_bt_lib.core.base_adapter`. They are independent business-in
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
   CONFIG_DEFAULTS (there is no `xyz_bt_lib/config/` — see Defaults rules).

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
and document default values. (There is no `xyz_bt_lib/config/`.)

### Composite value adapters

- If an adapter writes a dict or list BB value, the module docstring must
  include a complete schema definition (field names, types, semantics).
- Adapters must produce standardised output fields — `error_x`, `error_y`, `size`,
  `lost_count` — computed once in the adapter. Do not push shape-specific area
  or error calculations to L1/L2 nodes.
- Shape-specific `size` convention:
  - `circle` → `π × radius²` — use `ObjectInfo.radius` field directly (NOT estimated from bounding box)
  - `rect` → `width × height`
- `error_x` / `error_y` must be calculated using calibrated image centre:
  ```
  cx = image_width / 2 + center_x_offset
  cy = image_height / 2 + center_y_offset
  ```
  Mirror the `center_x_offset` pattern from `ObjectDetectionAdapter`.
  (`LineDetectionAdapter` was deleted Aug 2026 — a line is now just a target
  in `tracked_objects` with `shape='line'`; configure it with
  `target_specs={'line': {'shape': 'line'}}, center_x_offset=<cal>`.)

Adapters must keep the two-phase latch protocol:

1. `snapshot_and_reset()` is called while the caller holds the shared lock.
2. `write_snapshot(snap, tick_id)` is called after releasing the lock, on the main thread.

Phase 1 for all adapters must happen inside the same `with lock:` block so the BT tree
sees a consistent frozen view of all sensor inputs per tick.

---

## Facade Contract Rule for L2

New L2 nodes should call existing `XyzBTFacade` methods.

The authoritative contract is `xyz_bt_lib/src/xyz_bt_lib/core/base_facade.py`, which defines
10 abstract methods — the gait / motion / buzzer primitives, plus the `go_step`, `turn_step`
and `move_head` convenience wrappers — plus 2 concrete process-control methods that a project
facade may override. Read the signatures and their Args docstrings there.

**Do not restate the signatures here.** A copy that lived in this file drifted against
`base_facade.py` — it dropped `move_head`'s `tilt_pos` and all five `go_step` / `turn_step`
override params (`period_time_ms`, `dsp_ratio`, `y_swap_amplitude`, `gait_param`, `arm_swap`),
so an L2 node written from it could not reach half the gait tuning surface.

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

## Mode 3 — Add Input Adapter

### Information to collect

1. **ROS topic** — e.g. `/camera/depth`
2. **message type** — e.g. `sensor_msgs/Image`
3. **what to extract** — e.g. "average depth of center ROI"
4. **sensor-level judgement/classification standard**
5. **BB keys to write**
6. **adapter class name** — e.g. `DepthNavAdapter`
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
4. Calibration/config values go in CONFIG_DEFAULTS with matching `__init__` args.
   Do NOT add a YAML file or recreate `xyz_bt_lib/config/` — that directory was
   deleted Aug 2026 and a library adapter must not read config from disk. A value
   the project owns (a camera-centre offset, a per-robot threshold) is threaded in
   from the project layer as a constructor argument.

5. Expand `assets/templates/input_adapter.py.tpl` to generate the adapter at:
   `xyz_bt_lib/src/xyz_bt_lib/adapters/{{class_name_snake}}.py`
6. Verify structural conformance: the generated file must be structurally
   identical to `assets/templates/input_adapter.py.tpl` in the parts that are
   CONTRACT: the class-level declarations, and the
   `_callback()` / `snapshot_and_reset()` / `write_snapshot()` skeleton with its
   locking discipline.

   The conversion helpers are shaped by the sensor, not by the template. Extra
   `__init__` params and differently-named helpers are expected — see
   `ObjectDetectionAdapter`, which uses `_match_targets()` / `_build_entry()` /
   `_compute_size()` / `_compute_errors()` and takes `target_specs` +
   `enable_depth`, or `ImuBalanceStateAdapter`, which adds a public
   `force_state()`. What is NOT negotiable: every helper in the conversion path
   stays side-effect-free, and only `_apply_live_writes()` mutates live state.
7. Ensure the file satisfies all Input Adapter rules:
   - inherits `XyzInputAdapter`, not `XyzBTNode`
   - top-level docstring fully traces input -> BB writes
   - `BB_READS = []`, `BB_WRITES`, `FACADE_CALLS = []`, `CONFIG_DEFAULTS` declared
   - the conversion path is split into side-effect-free helpers (names follow the
     sensor: `_extract_*` / `_classify_*` / `_make_bb_writes` for a scalar adapter,
     or `_match_targets` / `_build_entry` / `_compute_*` for a tracking one)
   - `_callback()` only receives, calls helpers, updates live state under lock,
     increments count — it MAY also read live state under the lock when the
     classification depends on the current state (see `ImuBalanceStateAdapter`)
   - `write_snapshot()` only writes snapshot and emits observability events
   - only `ros_in` and `input_state` are emitted via `XyzInputAdapter` helpers
   - no BT action strategy
   - no unresolved template `TODO` or `NotImplementedError`
8. Update `adapters/__init__.py` only if that package uses explicit exports.
9. Verify the module docstring is the complete spec: topic, message type, BB writes,
   extraction/classification rules, CONFIG_DEFAULTS, integration notes (no central
   spec doc — the docstring is authoritative).

### Adapter Integration Instructions

```text
1. Import in app/<project>_bt_node.py __init__():
   from xyz_bt_lib.adapters.<snake_name> import <ClassName>
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
   catkin build xyz_bt_lib xyz_behavior
```

### Adapter Checklist

```text
✅ ROS topic and message type declared
✅ Inherits XyzInputAdapter, not XyzBTNode
✅ Every BB write declared with source fields, helper, rule/threshold, BB key
✅ BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS declared
✅ conversion path split into side-effect-free helpers (names may follow the sensor)
✅ _classify_xxx() contains no rospy logging and no self-state mutation;
   counter updates and rospy.loginfo() live in _callback()
✅ _callback() has no hidden BB writes or action strategy
✅ write_snapshot() only writes snapshot and emits ros_in/input_state
✅ No ros_out/ros_result emitted by adapter
✅ Thresholds are in CONFIG_DEFAULTS (no config YAML — the directory does not exist)
✅ Thresholds and calibration values are in CONFIG_DEFAULTS and exposed as __init__ args (or documented as YAML-backed with CONFIG_DEFAULTS providing the fallback)
✅ package.xml updated if message dependency is new
✅ No unresolved template TODO or NotImplementedError remains
✅ Module docstring complete (topic, msg type, BB writes, rules, CONFIG_DEFAULTS) — docstring is the spec
✅ File structure matches input_adapter.py.tpl (param order, class decls, helpers, callback/snapshot skeleton)
```

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
from xyz_bt_lib.adapters.<ClassName> import <ClassName>
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
