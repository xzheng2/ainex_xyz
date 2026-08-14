---
name: xyz-bt-facade-project
description: Scaffold or modify a competition behavior project under xyz_behavior/ — tree wiring (tree/*_bt.py plus the matching *_groot.xml), RuntimeFacade and _RuntimeIO, the app entry point, and launch files. Use when creating a new BT project, wiring composites and gate decorators, or changing the runtime/facade layer. 新建BT项目, 创建比赛行为树, scaffold bt project.
---

# xyz-bt-facade-project skill

**This is the only active project template skill.**
`xyz-bt-project` is deprecated and archived — do not use it as a reference or generation baseline.

## Trigger phrases

新建BT项目, 创建比赛行为树, scaffold bt project, create a new BT project,
set up a competition BT, 新建 <name> 项目, scaffold facade project

## Enforcement rules

1. L2 nodes: call_facade() only — never touch _RuntimeIO, managers, or raw ROS
1b. project_bt.py is tree wiring only: node instantiation, composite composition, and decorator
    application. Facade calls are forbidden; facade ref passes only from bootstrap() signature
    to node constructors. See § project_bt.py.
2. RuntimeFacade: holds _RuntimeIO; does NOT hold gait_manager / motion_manager / publishers
3. _RuntimeIO: the ONLY class that calls gait_manager, motion_manager, or ROS publishers
4. publish_buzzer(): scalar params in public contract; BuzzerState constructed inside _RuntimeIO
5. Input adapters (ObjectDetectionAdapter, YoloObjectDetectionAdapter, ImuBalanceStateAdapter, etc.):
   inbound ROS only; outbound goes via _RuntimeIO.
   Detection adapters write BB.TRACKED_OBJECTS + BB.DETECTION_SOURCE ('color' or 'yolo').
   BB.DETECTION_SOURCE is set once at init and never changes at runtime.
6. All L2 nodes must declare BB_READS, BB_WRITES, FACADE_CALLS, CONFIG_DEFAULTS
7. project_bt.py editing rule: always preserve the CONFIG_DEFAULTS block pattern (explicit kwargs
   + `# CONFIG_DEFAULTS — override to tune:` comment). See § project_bt.py.
8. Node naming: launch file must use `name="{{PROJECT}}"` (not `{{PROJECT}}_bt_node`).
   BTExecController, TreeROSPublisher, and BBBridge all use `~` (private namespace);
   ROS resolves `~bt/pause` → `/{node-name}/bt/pause`. Wrong name = services unreachable.
   That runtime resolution is exactly why the first two are shared modules rather than
   per-project copies — the launch file's `name=`, not a template substitution, is what
   scopes them to a project.
9. Composites: use ONLY the semantic factories from `xyz_bt_lib.core.composites` —
   `ReactiveSequence` / `CommittedSequence` / `PrioritySelector` / `CommittedSelector`
   and, for Parallel, `ParallelAll` (SuccessOnAll) / `ParallelAny` (SuccessOnOne).
   NEVER write raw `py_trees.composites.Sequence(memory=...)` / `Selector(memory=...)` /
   `Parallel(policy=...)`.
   Stability confirmation ("condition stable for N ticks") uses `LatchedDwellDecorator`
   wrapping a CONDITION node — never an in-node dwell. See § Stability confirmation.
10. `bb_current.json` is written only by `bt_node` via `BlackboardCurrentWriter`.
    Adapters and L1/L2 nodes must never write it.
11. Groot XML sync: `tree/` must always contain BOTH `{{PROJECT}}_bt.py` and
    `{{PROJECT}}_groot.xml`, in sync at the end of every session, with the XML treated as
    source of truth when they differ. See § Groot XML co-generation and sync.
12. catkin build after CMakeLists changes: a new `catkin_install_python` script or
    `install(DIRECTORY ...)` block MUST be followed by a catkin build before the user is
    asked to `roslaunch` — ROS cannot locate the node until it is installed to
    `devel/lib/<pkg>/`. This is the most common cause of "Cannot locate node of type [...]".
    Command in § Post-generation checklist.
13. TreeNodesModel Control declarations: every `TreeNodesModel` must include BOTH
    `Control ID="Sequence"` and `Control ID="Fallback"` with `<input_port name="memory"/>`,
    even when the tree body uses only one or neither. Missing entries silently hide the
    `memory` port in Groot. `validate_templates.py` only checks the types a tree actually
    uses, so this one still needs a human eye.

## Template self-check

Before relying on these templates — and after ANY edit to a `.tpl` or to a library
contract they depend on (`core/base_facade.py` above all) — run:

```bash
python3 /home/pi/docker/ros_ws_src/xyz_bt_lib/tools/validate_engine.py
```

It renders every template into a throwaway package, checks `bt_node`'s
construction call sites, the facade's signature parity with the base class, and
that `bootstrap()` yields a tree that actually ticks through both branches — plus
the other skills' templates and a guard sweep over the library. Exit 0 = safe to
scaffold.

This exists because five defects once shipped that each crashed a fresh project
before its first tick, and none was visible by reading: the templates and this
document agreed with each other while both disagreed with the library. **A Python
ABC checks method names, not signatures**, so a drifted facade still instantiates.

## Post-generation checklist

- [ ] `validate_templates.py` passes (see § Template self-check) — run it if any template or facade contract changed
- [ ] `CMakeLists.txt` lists `{{PROJECT}}/app/{{PROJECT}}_bt_node.py` (full relative path from package root) under `catkin_install_python` — NOT a scripts/ shim
- [ ] After any `CMakeLists.txt` change (new script in `catkin_install_python`, new `install(DIRECTORY ...)` block): run catkin build inside the `ainex` container and confirm the node is installed:
  ```bash
  docker exec ainex /bin/bash -c \
    "source /opt/ros/noetic/setup.bash && cd /home/ubuntu/ros_ws && catkin build xyz_behavior xyz_bt_lib --no-status"
  ls /home/ubuntu/ros_ws/devel/lib/xyz_behavior/{{PROJECT}}_bt_node.py
  ```
  If a build is abandoned due to a stale package cache, clean it first:
  ```bash
  docker exec ainex /bin/bash -c \
    "source /opt/ros/noetic/setup.bash && cd /home/ubuntu/ros_ws && catkin clean <stale_pkg> -y"
  ```
- [ ] `_LOG_DIR = os.path.join(_PKG_PATH, 'log')` — never a project subdirectory; all projects share `xyz_behavior/log/`
- [ ] `run_dir = open_run_dir(_LOG_DIR, max_runs=_MAX_RUNS, log=rospy.loginfo)`, and **every writer points at `run_dir`**, not `_LOG_DIR`: the 4 `DebugEventLogger` jsonl paths and `BlackboardCurrentWriter`. `_LOG_DIR` keeps only the symlinks `open_run_dir()` refreshes, which is what lets ROSA / `bt_log_read_guard.py` / the diagnose skill go on using the fixed filenames. Writing through a symlink would destroy it — these classes end in `os.replace()`.
- [ ] `_PKG_PATH` uses `rospkg.RosPack().get_path('xyz_behavior')`, not `__file__`-relative
- [ ] a second `sys.path` root, `_RUN_LAB_PATH = rospkg.RosPack().get_path('xyz_run_lab')`, backs `from run_lab.run_context import open_run_dir`. The three observability imports (`debug_event_logger`, `bt_debug_visitor`, `blackboard_current_writer`) still come from `bt_observability` under `_PKG_PATH` — the split is deliberate: instrumentation is present in every experiment lane, the observability architecture is not
- [ ] No `infra/` directory. `TreeROSPublisher` / `BTExecController` come from `bt_observability`; `_RuntimeIO` from `xyz_bt_lib.core.default_runtime_io`; project-local BB keys go to `BlackboardROSBridge(_BB_TOPIC_MAP)`, not to a per-project bridge class
- [ ] `runtime/` holds `runtime_facade.py` only, unless the project retunes the step velocity profile — then a subclass in `runtime/_runtime_io.py`, never a full copy
- [ ] `logger.close()` called on shutdown
- [ ] `app/{{PROJECT}}_bt_node.py` has `if __name__ == '__main__': main()` at bottom
- [ ] `tree/{{PROJECT}}_groot.xml` created alongside `tree/{{PROJECT}}_bt.py` — use `project_bt_groot.xml.tpl`; must mirror tree structure, composite types, node IDs, and CONFIG_DEFAULTS port values
- [ ] `tree/{{PROJECT}}_groot.xml` TreeNodesModel has both `Control ID="Sequence"` and
      `Control ID="Fallback"` with `memory` port (rule #13)
- [ ] `launch/{{PROJECT}}.launch` created (from `project.launch.tpl`): perception nodes + BT node only; never includes bringup. User runs `xyz_bringup.launch` separately before this.
- [ ] Per-project `launch/{{PROJECT}}_bringup.launch` only if hardware differs from `xyz_bringup.launch` (e.g. an event with no camera, avoids sensor_node; most projects use `xyz_bringup.launch` as-is)
- [ ] After any tree edit session: `_bt.py` and `_groot.xml` describe identical structures (rule #11)
- [ ] `python3 xyz_behavior/tools/sync_groot_xml.py tree/{{PROJECT}}_groot.xml` reports **0 changes** on a fresh scaffold (see § Groot XML for what the tool does; warnings are expected only for list/dict-typed params such as `x_range`/`yaw_range`/`gait_param`, which it leaves untouched)

## Architecture

```
L2 nodes
  └─ call_facade() → XyzBTFacade (abstract contract)
                          └─ {{PROJECT_CLASS}}RuntimeFacade (implementation)
                                └─ profile cfg merge (go_cfg / turn_cfg)
                                └─ convenience wrappers (go_step / turn_step / move_head)
                                └─ primitive passthrough (all 7 primitives)
                                      └─ _RuntimeIO (sole raw ROS / manager egress)
                                            └─ gait_manager / motion_manager / publishers
```

Execution chain in bt_node.py:
```
_RuntimeIO(gait_manager, motion_manager, buzzer_pub, ...)
  └─ RuntimeFacade(runtime_io, go_cfg=_GO_CFG, turn_cfg=_TURN_CFG, ...)
        └─ bootstrap(runtime_facade)
              └─ BehaviourTree → TreeROSPublisher(tree)
```

Templates: `bt_node.py.tpl` → `app/{{PROJECT}}_bt_node.py` · `runtime_facade.py.tpl` → `runtime/runtime_facade.py`. `_RuntimeIO` is not generated — it is imported from `xyz_bt_lib.core.default_runtime_io`.

## Public contract (XyzBTFacade)

**The authoritative signature list is `xyz_bt_lib/core/base_facade.py`** — read it,
do not trust a copy. 10 abstract methods (7 primitives + `go_step` / `turn_step` /
`move_head`) plus 2 concrete process-control methods. `runtime_facade.py.tpl`
implements them; that template is the worked example.

This section deliberately does NOT restate the signatures. A previous copy here
drifted from the base class (`move_head` was documented and templated without
`tilt_pos`, which every head node passes), and because the doc and the template
agreed with each other, nothing flagged it until a project crashed on its first
Search tick. **Python's ABC machinery checks method NAMES only, never signatures**,
so a drifted override still instantiates — `validate_templates.py` exists to catch
exactly this and must be run after any facade change.

Facts that live nowhere else, so they stay here:
- `set_step` / `go_step` / `turn_step`: **x and y are in metres**, yaw in degrees.
- `publish_buzzer` takes scalars only; `_RuntimeIO` is what constructs `BuzzerState`.
- `go_step` / `turn_step` merge `go_cfg` / `turn_cfg` with per-call overrides;
  a non-None per-call value wins. See § go_cfg / turn_cfg shape below.

## Output file structure

```
{{PROJECT}}/
  runtime/
    runtime_facade.py    ← {{PROJECT_CLASS}}RuntimeFacade(XyzBTFacade)
    _runtime_io.py       ← OPTIONAL, only to retune the step velocity profile
  tree/
    {{PROJECT}}_bt.py      ← bootstrap(runtime_facade, ...)
    {{PROJECT}}_groot.xml  ← Groot BT visualization (keep in sync with _bt.py)
  app/
    {{PROJECT}}_bt_node.py    ← catkin_install_python entry point (run directly)
  behaviours/
    actions.py           ← project-specific L2 nodes (if needed)
log/                     ← shared runtime log dir (xyz_behavior/log/); all projects write here
  bb_current.json        ← current BB state mirror (latest tick only; not a history log)

launch/                  ← inside xyz_behavior/launch/, shared across all projects
  xyz_bringup.launch     ← shared default hardware bringup (base.launch + camera); run separately before behavior launch
  {{PROJECT}}.launch     ← behavior launch: perception nodes + BT node only; never includes bringup
  {{PROJECT}}_bringup.launch  ← ONLY when project needs non-default hardware (e.g. an event with no camera + avoids sensor_node)
```

**There is no `infra/` any more.** It held four modules and none of them earned a
per-project copy. `TreeROSPublisher` / `BTExecController` are shared modules in
`xyz_behavior/bt_observability/`. `bb_ros_bridge.py` re-implemented what
`BlackboardROSBridge(key_to_topic_map=...)` already does — pass the map instead.
`infra_manifest.py` wrote a JSON file that nothing ever read, declaring an exclusion
from the comm log that was never implemented. Do not re-add any of them as templates.

**`_RuntimeIO` is shared too**, at `xyz_bt_lib.core.default_runtime_io`. A project
generates `runtime/_runtime_io.py` **only** to retune `_GO_STEP_VEL` / `_TURN_STEP_VEL`,
as a subclass — see § go_cfg / turn_cfg shape.

Templates: `project.launch.tpl` → `launch/{{PROJECT}}.launch` (perception nodes + BT node; never includes bringup; TODO block offers Option A (colour detection) or Option B (YOLO detection via `YoloObjectDetectionAdapter`); uncomment the correct block when scaffolding)

Not generated (deprecated paradigm):
- `semantics/semantic_facade.py`
- `comm/comm_facade.py`

## project_bt.py (tree wiring)

Templates: `project_bt.py.tpl` → `tree/{{PROJECT}}_bt.py` · `actions.py.tpl` → `behaviours/actions.py`

`bootstrap()` instantiates each node with all `CONFIG_DEFAULTS` expanded as explicit keyword
arguments, preceded by a comment:

```python
    # CONFIG_DEFAULTS — override to tune:
```

`project_bt.py` is a **wiring-only** file. It must not call facade methods, hold a facade
reference beyond the `bootstrap()` signature, define tree-local helper behaviours that call
facade, or touch _RuntimeIO / managers / raw ROS. All action side effects belong in formal
L2 nodes (either from `xyz_bt_lib` or the project's `behaviours/actions.py`).

`CONFIG_DEFAULTS` is a class-level contract declared by each leaf node (inherited from
`XyzBTNode.CONFIG_DEFAULTS = {}`). The kwargs in `project_bt.py` surface the node's own
defaults at the call site — not project-level config — so the tree assembler can see and
override per-node values in one place.

Nodes with `CONFIG_DEFAULTS = {}` stay compact (no comment block added).

**Memory rule:** a `ReactiveSequence` (`memory=False`) re-initialises its RUNNING children every tick — this resets any inner `CommittedSequence` (`memory=True`) subtree or instance-state dwell counter placed beneath it. A `PrioritySelector` (`memory=False` Selector) does not. So never put a `CommittedSequence` under a `ReactiveSequence` if it must make progress. For "condition stable for N ticks", use `LatchedDwellDecorator` (BB-backed) which survives this re-entry by design — see § Stability confirmation. The authoritative, measured statement of this rule lives in `xyz_bt_lib/core/composites.py`.

When editing an existing `{{PROJECT}}_bt.py`:
- Follow the template pattern exactly.
- Keep the `# CONFIG_DEFAULTS — override to tune:` comment block for nodes that declare `CONFIG_DEFAULTS`.
- Do not collapse tunable kwargs back into implicit defaults.
- If a new node is added that has `CONFIG_DEFAULTS`, expand them the same way.
- Never add facade calls or _RuntimeIO/manager/raw-ROS imports.
- Tree-local helpers that read/write BB or return pure status are permitted
  as wiring aids.  Helpers that call facade are forbidden — extract that logic
  into a formal L2 node (in `xyz_bt_lib` or the project's `behaviours/actions.py`).
- Do not create new tree-local helper classes without asking the user first.

## Groot XML co-generation and sync

Template: `project_bt_groot.xml.tpl` → `tree/{{PROJECT}}_groot.xml`

Every BT project tree/ folder contains two companion files:
- `{{PROJECT}}_bt.py` — executable Python tree wiring
- `{{PROJECT}}_groot.xml` — Groot-compatible BT XML (visual editing + diff)

**Creation**: always generate both files together. The XML mirrors the same tree
structure, node names, composite types, and CONFIG_DEFAULTS port values as bt.py.

**Composite → XML mapping**: each semantic factory maps through its underlying
py_trees composite (`ReactiveSequence` = `Sequence(memory=False)`, and so on — the
full table is in `xyz_bt_lib/core/composites.py`), then to the XML tag:

- `Sequence(memory=…)` → `<Sequence memory="true|false">`
- `Selector(memory=…)` → `<Fallback memory="true|false">`
- `Parallel` → `<Parallel success_threshold="N" failure_threshold="M">`
- `py_trees.decorators.SuccessIsRunning` → `<Decorator ID="SuccessIsRunning">`
- L1 condition class → `<Condition ID="ClassName" port="value" …/>`
- L2 action class → `<Action ID="ClassName" port="value" …/>`

**Groot port registration**: `<TreeNodesModel>` must list **every** node type used in the
tree — not just `Sequence`/`Fallback`. Without a `TreeNodesModel` entry, Groot renders the
node box but shows no configuration port fields (params are invisible even though they exist
as XML attributes on the instance). One `<input_port>` per CONFIG_DEFAULTS key; omit
dict-type params such as `gait_param`.

For the per-node-kind syntax and a complete worked `<TreeNodesModel>`, see
`assets/templates/project_bt_groot.xml.tpl` — it carries both the rules and the
example, so they cannot drift apart. `validate_templates.py` asserts that every
node type used in the tree has a model entry.

Each node instance in `<BehaviorTree>` must use `ID=` (the registered type name matching
its `TreeNodesModel` entry) and optionally `name=` as the display label:
```xml
<Action ID="L2_Gait_VisionToObject" name="VisionToTag" target_id="apriltag" .../>
```
`name=` alone (without `ID=`) is only valid for unnamed composites, not leaf nodes.

**Modification workflow (XML-first policy)**:
Before editing `{{PROJECT}}_bt.py`, always read `{{PROJECT}}_groot.xml` first.

| XML state vs bt.py | Action |
|---|---|
| XML matches bt.py (no user edits) | Apply the change to bt.py, then update the XML to match. |
| XML differs from bt.py (user has edited XML directly) | Treat XML as source of truth. Reconcile bt.py to match the XML's current structure, then apply the requested change to both. |

**Sync invariant**: at the end of every edit session both files must describe
identical tree structures — same composites, node IDs, names, memory= attributes,
and port values.

`xyz_behavior/tools/sync_groot_xml.py` refreshes XML **param values** one-way from
bt.py (matching nodes by `name=`, writing a separate `{{PROJECT}}_groot.synced.xml`;
the panel's BT Tools "Open in Groot" runs it). It never touches the authored XML or
the tree structure, so it does not replace the XML-first policy above. For it to
match, **every node must carry a unique `name=` present on both files**.

## `robot_state` has four values, not three

`/latched/robot_state` ∈ `{'stand', 'lie', 'recline', 'pending'}`. Who writes and
who clears `'pending'`, and why it exists, is documented in
`behaviours/L1_perception/L1_Balance_IsStanding.py`.

Consequence for tree wiring — the part that only matters here:
`L1_Balance_IsStanding` and `L1_Balance_IsFallen` are
**no longer strict inverses** — both return FAILURE while `pending`. That is
deliberate: during `pending` the robot is neither confirmed upright nor confirmed
down, so a safety gate must not assume either, and recovery must not re-trigger
while the IMU is still deciding. Do not write a tree that infers "pending" from
those two nodes; test `robot_state` explicitly if a branch really needs it.

## Stability confirmation

Stability/timing state belongs to a **tree-layer decorator**, never inside an L1
node — L1 nodes are pure predicates (SUCCESS/FAILURE only). Both decorators keep
their counters in the Blackboard at `/node_state/<state_key>`, so they survive the
reactive re-entry a `ReactiveSequence` ancestor causes.

Rules when wiring one:
- **Wrap ONLY something whose status set is {SUCCESS, FAILURE}** — an L1 condition,
  or a composite built purely of L1 conditions. **Never wrap an L2 action.**
- `state_key` MUST be a hardcoded string **literal** — never from a variable,
  constructor arg, or rosparam. Both decorators share the `/node_state/` namespace,
  so it must be unique across BOTH.
- `required_ticks` / `enter_ticks` / `exit_ticks` are NUMBERS injected via a
  `bootstrap()` parameter; read rosparam only in the app/ layer and thread it
  through as an argument.
- Always pass `tick_id_getter` — timing is by tick, never wall clock.

Why each rule exists (the RUNNING-is-not-passed choice, the re-dispatch failure
mode of wrapping an L2, the tick-vs-wall-clock rationale, the Blackboard key
layout) is documented once, in the module docstrings of
`xyz_bt_lib/core/latched_dwell.py` and `xyz_bt_lib/core/hysteresis.py`. Read those
before changing a gate; do not restate them here.

```python
grab_confirmed = LatchedDwellDecorator(
    is_grabbed_condition,                 # a pure L1 predicate
    required_ticks=grab_confirm_ticks,    # NUMBER from a bootstrap() param
    state_key='grab_confirmed',           # hardcoded LITERAL, greppable
    tick_id_getter=tick_id_getter)

grab_seq = CommittedSequence('GrabSeq', children=[
    gate_node,
    grab_confirmed,
    action_node,
])
```

### Which gate: LatchedDwell or Hysteresis

- **`LatchedDwellDecorator`** (N-in / 1-out, RUNNING while counting) — "wait until
  the world is stable, then commit; abandon instantly if it stops being true."
  Safety gates, alignment before a committed motion.
- **`HysteresisDecorator`** (N-in / M-out, **never RUNNING**) — "this measurement is
  noisy; hold the answer steady." Flickering perception, a target hovering at a
  detection boundary. Because it never returns RUNNING it will not hold a
  `ReactiveSequence` branch open while it counts.

```python
ball_visible = HysteresisDecorator(
    is_ball_detected,                     # a pure L1 predicate
    enter_ticks=2,                        # 2 frames to believe it appeared
    exit_ticks=ball_lost_ticks,           # N misses to believe it is gone
    state_key='ball_visible',             # hardcoded LITERAL, greppable
    tick_id_getter=tick_id_getter)
```

## go_cfg / turn_cfg shape

`_GO_CFG` / `_TURN_CFG` hold the **static** per-profile params merged by
`go_step()` / `turn_step()`. **Their values are defined in
`assets/templates/bt_node.py.tpl`** — read them there, do not trust a copy. A copy
lived here until Aug 2026 and had drifted (`arm_swap: False` against the
template's `30`), which is exactly the failure mode this section now avoids.

`_RuntimeIO` owns the **step velocity profile** constants that control gait speed
(`_GO_STEP_VEL` / `_TURN_STEP_VEL`, each `[period_time_ms, dsp_ratio,
y_swap_amplitude]`) — defined in `xyz_bt_lib/src/xyz_bt_lib/core/default_runtime_io.py`,
which every project shares. To retune them, subclass rather than edit:

```python
# <project>/runtime/_runtime_io.py   — write this file ONLY when retuning
from xyz_bt_lib.core.default_runtime_io import _RuntimeIO as _BaseRuntimeIO

class _RuntimeIO(_BaseRuntimeIO):
    # rebind _GO_STEP_VEL / _TURN_STEP_VEL here; the values live in
    # core/default_runtime_io.py, never in this document
```

The `as` alias is required: the subclass keeps the name `_RuntimeIO` so rule #3 and the
guard's `runtime/_runtime_io.py` path key still describe it, which leaves no other way to
name the base.

`gait_manager.set_step(step_vel, x, y, yaw, gait_param, ...)` takes `step_vel` list as first arg.
Per-call overrides (`period_time_ms`, `dsp_ratio`, `y_swap_amplitude`, `arm_swap`, `step_num`,
`gait_param`) passed to `go_step()` / `turn_step()` take precedence over both `_GO_CFG`
and `_GO_STEP_VEL` defaults for that tick only.

Per-tick dynamic params (`x`, `y`, `yaw`) come from the L2 node's `call_facade()` call.
`bt_node` never calls `_RuntimeIO` directly.

x, y units: **meters** (GaitManager native units, `_XY_SCALE = 1`).
Typical values: `x=0.020` fast approach, `x=0.005` fine-tune, `x=0.000` turn-in-place.

`go_step()` smooths executed turn angle via a tunable `_YAW_AVG_WINDOW` (publishes
the window average for stable rotation). See the docstring + comments in
`runtime_facade.py.tpl`.

## Current blackboard snapshot

`bt_node.py.tpl` wires `BlackboardCurrentWriter` as a **default scaffold capability**.
After every valid `tree.tick()`, the node atomically refreshes:

```
xyz_behavior/log/bb_current.json
```

**Schema**: owned by `BlackboardCurrentWriter` in
`xyz_behavior/bt_observability/blackboard_current_writer.py` — read it for the
authoritative field list (`schema_version`, `tick_id`, `ts`, `root_status`,
`blackboard`). Note `robot_state` is one of `stand | lie | recline | pending`
(see § `robot_state` has four values above).

Ownership rules:
- **Owner**: project/app infra (`bt_node.py`). Maintained centrally by the node entry point.
- **Not owned by**: adapters, L1 nodes, L2 nodes, the BB ROS bridge, or any `xyz_bt_lib` component.
- **Not a history log**: this is a single current-state mirror. It does not replace JSONL
  observability (`bt_debug_lastrun.jsonl`, `bt_debug_recent.jsonl`).
- **Not folded into the BB ROS bridge**: BB ROS mirroring and the BB file snapshot are
  separate responsibilities with separate code paths.
- **One file for all projects**: always `xyz_behavior/log/bb_current.json` — no per-project
  filenames, no per-tick snapshots.

