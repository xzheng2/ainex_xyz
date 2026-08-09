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
9. Composites: use ONLY the semantic factories from `xyz_bt_lib.core.composites` —
   `ReactiveSequence` / `CommittedSequence` / `PrioritySelector` / `CommittedSelector`
   and, for Parallel, `ParallelAll` (SuccessOnAll) / `ParallelAny` (SuccessOnOne).
   NEVER write raw `py_trees.composites.Sequence(memory=...)` / `Selector(memory=...)` /
   `Parallel(policy=...)`.
   Stability confirmation ("condition stable for N ticks") uses `LatchedDwellDecorator`
   wrapping a CONDITION node — never an in-node dwell. See § Stability confirmation.
10. `bb_current.json` is written only by `bt_node` via `BlackboardCurrentWriter`.
    Adapters, L1/L2 nodes, and `bb_ros_bridge` must never write it.
11. Groot XML sync: `tree/` must always contain both `{{PROJECT}}_bt.py` and
    `{{PROJECT}}_groot.xml`. Read the XML before editing bt.py; treat XML as source of truth
    if they differ. Both files must be in sync at end of every session. See § Groot XML co-generation and sync.
    A convenience tool `xyz_behavior/tools/sync_groot_xml.py` refreshes XML **param values**
    one-way from bt.py (matches nodes by `name=`, writes a separate `{{PROJECT}}_groot.synced.xml`;
    the panel BT Tools "Open in Groot" runs it) — it never touches the authored XML or tree
    structure, so it does not change this policy. For it to match, **every node must carry a
    unique `name=` present on both files**.
12. catkin build after CMakeLists changes: every time a new script is added to
    `catkin_install_python` or a new `install(DIRECTORY ...)` block is added,
    catkin build MUST be run before the user is asked to `roslaunch`. ROS cannot
    locate the node until it is installed to `devel/lib/<pkg>/`. Never skip this
    step — it is the most common cause of "Cannot locate node of type [...]" errors.
13. TreeNodesModel Control declarations: every `TreeNodesModel` must include BOTH
    `Control ID="Sequence"` and `Control ID="Fallback"` with `<input_port name="memory"/>`,
    even when the specific tree body only uses one or neither composite type.
    Missing entries silently hide the `memory` port in Groot.

## Post-generation checklist

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
- [ ] `_PKG_PATH` uses `rospkg.RosPack().get_path('xyz_behavior')`, not `__file__`-relative
- [ ] `logger.close()` called on shutdown
- [ ] `app/{{PROJECT}}_bt_node.py` has `if __name__ == '__main__': main()` at bottom
- [ ] `tree/{{PROJECT}}_groot.xml` created alongside `tree/{{PROJECT}}_bt.py` — use `project_bt_groot.xml.tpl`; must mirror tree structure, composite types, node IDs, and CONFIG_DEFAULTS port values
- [ ] `tree/{{PROJECT}}_groot.xml` TreeNodesModel has both `Control ID="Sequence"` and
      `Control ID="Fallback"` with `memory` port (rule #13)
- [ ] `launch/{{PROJECT}}.launch` created (from `project.launch.tpl`): perception nodes + BT node only; never includes bringup. User runs `xyz_bringup.launch` separately before this.
- [ ] Per-project `launch/{{PROJECT}}_bringup.launch` only if hardware differs from `xyz_bringup.launch` (e.g. an event with no camera, avoids sensor_node; most projects use `xyz_bringup.launch` as-is)
- [ ] After any tree edit session: `_bt.py` and `_groot.xml` describe identical structures (rule #11)
- [ ] Every node has a unique `name=` present on both `_bt.py` and `_groot.xml`; then `python3 xyz_behavior/tools/sync_groot_xml.py tree/{{PROJECT}}_groot.xml` reports **0 changes** on a fresh scaffold (warnings are expected only for list/dict-typed params such as `x_range`/`yaw_range`/`gait_param`, which the tool leaves untouched) (rule #11 sync tool)

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

Templates: `bt_node.py.tpl` → `app/{{PROJECT}}_bt_node.py` · `runtime_facade.py.tpl` → `runtime/runtime_facade.py` · `_runtime_io.py.tpl` → `runtime/_runtime_io.py`

## Public contract (XyzBTFacade)

Primitives — implemented by RuntimeFacade, executed by _RuntimeIO:
- `disable_gait()` — shut down gait controller (needs enable_gait() to restart)
- `enable_gait()` — start/restart gait controller
- `stop_gait()` — stop current motion; controller stays up for new commands
- `set_step(dsp, x, y, yaw, ...)` — send one fully-resolved gait step; x/y in **meters**
- `run_action(action_name)` — execute a named stand-alone motion action
- `set_servos_position(duration_ms, positions)` — command servos directly
- `publish_buzzer(freq, on_time, off_time, repeat)` — scalar params only; _RuntimeIO constructs BuzzerState

Convenience wrappers — implemented by RuntimeFacade:
- `go_step(x, y, yaw, *, period_time_ms=None, dsp_ratio=None, y_swap_amplitude=None, arm_swap=None, step_num=None, gait_param=None)` — merges go_cfg + per-call overrides + x/y/yaw → set_step(motion_profile='go'); per-call overrides take precedence over _GO_CFG when not None
- `turn_step(x, y, yaw, *, period_time_ms=None, dsp_ratio=None, y_swap_amplitude=None, arm_swap=None, step_num=None, gait_param=None)` — merges turn_cfg + per-call overrides + x/y/yaw → set_step(motion_profile='turn')
- `move_head(pan_pos)` — maps to set_servos_position with HEAD_PAN_SERVO + HEAD_MOVE_DELAY_MS

## Output file structure

```
{{PROJECT}}/
  runtime/
    runtime_facade.py    ← {{PROJECT_CLASS}}RuntimeFacade(XyzBTFacade)
    _runtime_io.py       ← _RuntimeIO (sole raw ROS egress)
  tree/
    {{PROJECT}}_bt.py      ← bootstrap(runtime_facade, ...)
    {{PROJECT}}_groot.xml  ← Groot BT visualization (keep in sync with _bt.py)
  app/
    {{PROJECT}}_bt_node.py    ← catkin_install_python entry point (run directly)
  behaviours/
    actions.py           ← project-specific L2 nodes (if needed)
  infra/
    infra_manifest.py
    bb_ros_bridge.py
    bt_exec_controller.py
    tree_publisher.py
log/                     ← shared runtime log dir (xyz_behavior/log/); all projects write here
  bb_current.json        ← current BB state mirror (latest tick only; not a history log)

launch/                  ← inside xyz_behavior/launch/, shared across all projects
  xyz_bringup.launch     ← shared default hardware bringup (base.launch + camera); run separately before behavior launch
  {{PROJECT}}.launch     ← behavior launch: perception nodes + BT node only; never includes bringup
  {{PROJECT}}_bringup.launch  ← ONLY when project needs non-default hardware (e.g. an event with no camera + avoids sensor_node)
```

Templates: `project.launch.tpl` → `launch/{{PROJECT}}.launch` (perception nodes + BT node; never includes bringup; TODO block offers Option A (colour detection) or Option B (YOLO detection via `YoloObjectDetectionAdapter`); uncomment the correct block when scaffolding) · `infra_manifest.py.tpl` → `infra/infra_manifest.py`

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

py_trees → XML composite mapping (the factories map through their underlying
composite: `PrioritySelector`=`Selector(memory=False)`, `CommittedSelector`=
`Selector(memory=True)`, `ReactiveSequence`=`Sequence(memory=False)`,
`CommittedSequence`=`Sequence(memory=True)`, `ParallelAll`/`ParallelAny`=
`Parallel(SuccessOnAll/SuccessOnOne)`):
- `Selector(memory=True)`  → `<Fallback  memory="true">`
- `Selector(memory=False)` → `<Fallback  memory="false">`
- `Sequence(memory=True)`  → `<Sequence  memory="true">`
- `Sequence(memory=False)` → `<Sequence  memory="false">`
- `py_trees.composites.Parallel` → `<Parallel success_threshold="N" failure_threshold="M">`
- `py_trees.decorators.SuccessIsRunning` → `<Decorator ID="SuccessIsRunning">`
- L1 condition node class → `<Condition ID="ClassName" port="value" …/>`
- L2 action node class → `<Action ID="ClassName" port="value" …/>`

**Groot port registration**: `<TreeNodesModel>` must list **every** node type used in the
tree — not just `Sequence`/`Fallback`. Without a `TreeNodesModel` entry, Groot renders the
node box but shows no configuration port fields (params are invisible even though they exist
as XML attributes on the instance).

- `Sequence` / `Fallback` → `<Control ID="..."><input_port name="memory"/></Control>`
- L1 / L2 library nodes (xyz_bt_lib) → `<Condition ID="ClassName">` / `<Action ID="ClassName">` with one `<input_port name="..."/>` per CONFIG_DEFAULTS key (omit dict-type params such as `gait_param`)
- Project-local action nodes (`behaviours/actions.py`, tree-local helpers) → same pattern
- Decorators → `<Decorator ID="ClassName">` (usually no ports)

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

## Stability confirmation

To require a condition hold for N ticks before proceeding, wrap the CONDITION
node in `LatchedDwellDecorator` (`xyz_bt_lib.core.latched_dwell`). It stores its
counter in the Blackboard at `/node_state/<state_key>`, so — unlike an
instance-state dwell — it survives the reactive re-entry that a `ReactiveSequence`
ancestor causes. Do NOT put stability/timing state inside an L1 node: L1 nodes
are pure predicates (SUCCESS/FAILURE only).

Rules:
- **Wrap ONLY things whose status set is {SUCCESS, FAILURE}** — an L1 condition
  node, or a composite built purely of L1 conditions (`ReactiveSequence` of L1s,
  which never returns RUNNING). **Never wrap an L2 action.** Two ways it breaks:
  a continuous L2 returns RUNNING every tick → treated as not-passed → `stable_ticks`
  never climbs → the branch is unreachable; a one-shot L2 returns SUCCESS every
  tick → the count climbs, but the decorator ticks its child once per tick, so the
  side effect is re-dispatched N times (a 20-tick dwell = 20× `stop_gait`). L1
  predicates are side-effect-free so re-evaluation is safe; L2 actions are not.
- `state_key` MUST be a hardcoded string **literal** — never derived from a
  variable, constructor arg, or rosparam. It is a state identity: it must be
  greppable, and a duplicate must fail loudly at construction.
- `required_ticks` (a NUMBER) is injected via a `bootstrap()` parameter; read
  rosparam only in the app/ layer and thread it through as an argument.
- Timing is by `tick_id` (pass `tick_id_getter`), not wall clock — a blocking
  `run_action` stalls the tree, and tick_id (unlike wall time) does not advance
  during the block, so an in-progress dwell is not falsely reset.

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

## go_cfg / turn_cfg shape

`_GO_CFG` / `_TURN_CFG` hold the **static** per-profile params merged by `go_step()` / `turn_step()`:

```python
_GO_CFG = {
    'dsp':        0.1,    # passed to set_step() for legacy compat (not used by gait_manager directly)
    'gait_param': None,   # GaitParam msg or None (use controller default)
    'arm_swap':   False,
    'step_num':   0,      # 0 = continuous (non-blocking)
}
```

`_RuntimeIO` owns the **step velocity profile** constants that control gait speed:

```python
# In _RuntimeIO — tune per project
_GO_STEP_VEL   = [300, 0.1, 0.01]  # [period_time_ms, dsp_ratio, y_swap_amplitude]
_TURN_STEP_VEL = [400, 0.1, 0.01]
```

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

**Schema** (schema_version 1):
```json
{
    "schema_version": 1,
    "tick_id": 42,
    "ts": 1715789012.345,
    "root_status": "RUNNING",
    "blackboard": {
        "/latched/robot_state": "BALANCED",
        "/latched/tracked_objects": {}
    }
}
```

Ownership rules:
- **Owner**: project/app infra (`bt_node.py`). Maintained centrally by the node entry point.
- **Not owned by**: adapters, L1 nodes, L2 nodes, `bb_ros_bridge.py`, or any `xyz_bt_lib` component.
- **Not a history log**: this is a single current-state mirror. It does not replace JSONL
  observability (`bt_debug_lastrun.jsonl`, `bt_debug_recent.jsonl`).
- **Not folded into `bb_ros_bridge.py`**: BB ROS mirroring and the BB file snapshot are
  separate responsibilities with separate code paths.
- **One file for all projects**: always `xyz_behavior/log/bb_current.json` — no per-project
  filenames, no per-tick snapshots.

Template: `bb_ros_bridge.py.tpl` → `infra/bb_ros_bridge.py`
