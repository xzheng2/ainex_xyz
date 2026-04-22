---
name: ainex-bt-project
description: >
  Use this skill when the user wants to:
  CREATE a new BT project under ainex_behavior ("新建BT项目", "创建比赛行为树",
  "scaffold bt project", "新建 <name> 项目", "create a new BT project",
  "set up a competition BT").
  MODIFY an existing BT project ("新加btnode", "添加节点", "add bt node",
  "给 <project> 加一个条件节点/动作节点", "扩展行为树", "修改 <project> 的bt").
version: 0.1.0
argument-hint: <project_name> [task_description]
---

# ainex-bt-project Skill

Create or modify BT projects under `ainex_behavior/`, fully compliant with the
observability framework and infra manifest framework. All rules are in `references/`.

## Mode detection

| Signal | Mode |
|---|---|
| "新建", "scaffold", "create", no existing project dir | **Mode 1 — Create** |
| existing project name + "加节点", "add node", "修改", "扩展" | **Mode 2 — Modify** |

If ambiguous, ask: "新建项目还是给已有项目加节点？"

---

## System Thinking Principle — Adapter ↔ BB ↔ Behaviour

Every behaviour node that reads the blackboard is part of a **three-component system**.
Before writing or modifying **any single component**, trace the full chain:

```
Input Adapter                 →  blackboard_keys.py (BB.*)  →  Behaviour Node
ainex_bt_edu/input_adapters/     single source of truth         behaviours/
  emits: ros_in + input_state    BB.*_KEY / BB.* / BB.LATCHED_NS  reads BB.LATCHED_NS
```

| Question to answer | Where to look |
|---|---|
| What BB key does the behaviour need? | node's `BB_READS` / `BB_WRITES` / `register_key` |
| Is that key defined in `blackboard_keys.py`? | look for `BB.*_KEY` + `BB.*` |
| Which adapter writes that key? | `ainex_bt_edu/input_adapters/` |
| No adapter yet? → **add adapter first**, then BB key, then node |

**BB key naming convention** (`blackboard_keys.py` — never hardcode strings):

| Use | Pattern | Example |
|---|---|---|
| `attach_blackboard_client` namespace | `BB.LATCHED_NS` | `namespace=BB.LATCHED_NS` |
| `register_key` on `/latched/` client | `BB.*_KEY` (short) | `key=BB.ROBOT_STATE_KEY` |
| `register_key` on root-ns client | `BB.*` (absolute) | `key=BB.HEAD_PAN_POS` |
| `BB_READS` / `BB_WRITES` / `bb_writes` dict | `BB.*` (absolute) | `[BB.LINE_DATA]` |

**Active adapter-written keys** (as of v2.3.2):

| BB key (absolute) | Constant | Written by |
|---|---|---|
| `/latched/robot_state` | `BB.ROBOT_STATE` | `ImuBalanceStateAdapter` |
| `/latched/line_data` | `BB.LINE_DATA` | `LineDetectionAdapter` |
| `/latched/last_line_x` | `BB.LAST_LINE_X` | `LineDetectionAdapter` (raw pixel x) |
| `/latched/line_error_x` | `BB.LINE_ERROR_X` | `LineDetectionAdapter` (x − center, None when lost) |
| `/latched/line_center_x` | `BB.LINE_CENTER_X` | `LineDetectionAdapter` (width/2 + offset) |
| `/latched/last_line_error_x` | `BB.LAST_LINE_ERROR_X` | `LineDetectionAdapter` (sticky signed error) |
| `/latched/camera_lost_count` | `BB.CAMERA_LOST_COUNT` | `LineDetectionAdapter` |
| `/head_pan_pos` | `BB.HEAD_PAN_POS` | `L2_Head_FindLineSweep` (BB write, not adapter) |

> **Legacy warning**: `/locomotion/robot_state` and `/perception/line_data` are
> **deprecated** — do NOT use them in new code.  `behaviours_tem/` nodes that
> reference these keys are legacy archive (v1.2.1) and must not be used as templates.

---

## Mode 1 — Create new BT project

### Trigger

User says any of:
- `新建BT项目 <name>` / `创建比赛行为树 <name>`
- `scaffold bt project <name>`
- `新建 <name> 项目` (where context is ainex_behavior)
- `create a new BT project for <competition>`
- `/ainex-bt-project <name> [description]`

### Information to collect (if not in args)

Ask the user for:
1. **project_name** — snake_case, e.g. `ball_kick`, `sprint`, `penalty_kick`
2. **task_description** — one sentence describing the competition task
3. **base_class** — does the node inherit `color_common.Common`? (default: yes, like marathon)

If args were provided (`/ainex-bt-project ball_kick 足球踢球`), skip asking and proceed.

### 7-Step Workflow

### Step 1 — Load rules

Read `references/bt_observability_rules.md` and `references/bt_infra_manifest_rules.md`
before generating any file. The rules govern every structural and naming decision.

Key constraints to enforce (summary — full rules in references/):
- `tree/` imports ainex_bt_edu standard nodes first; project-specific nodes second
- Project condition nodes inherit `AinexL1ConditionNode`; project action nodes inherit
  `AinexL2ActionNode`. Both indirectly inherit `AinexBTNode`; no node may inherit
  `py_trees.behaviour.Behaviour` directly.
- Input adapters inherit `AinexInputAdapter` and never inherit `AinexBTNode`
- `semantic_facade.py` inherits `AinexBTFacade`
- `comm/comm_facade.py` is the **sole** `emit_comm` exit for business_out
- `ainex_bt_edu/input_adapters/` is the **sole** `emit_comm` exit for business_in (`ros_in` + `input_state`);
  `app/<project>_bt_node.py` must NOT contain `rospy.Subscriber` or direct `emit_comm` calls
- `logger=None` must be zero-cost (all emit calls are no-ops when logger is None)
- `tick_id` increments inside `should_tick()` gate, before `tree.tick()`

### Step 2 — Create directory structure

Create all directories under `ainex_behavior/<project_name>/`:

```
ainex_behavior/<project_name>/
├── __init__.py
├── app/
│   ├── __init__.py
│   ├── <project>_bt_node.py
│   └── ros_msg_utils.py
├── tree/
│   ├── __init__.py
│   └── <project>_bt.py
├── behaviours/
│   ├── __init__.py
│   ├── conditions.py
│   └── actions.py
├── semantics/
│   ├── __init__.py
│   └── semantic_facade.py
├── comm/
│   ├── __init__.py
│   └── comm_facade.py
├── algorithms/
│   └── __init__.py
├── infra/
│   ├── __init__.py
│   ├── infra_manifest.py
│   ├── tree_publisher.py
│   ├── bb_ros_bridge.py
│   └── bt_exec_controller.py
├── log/
│   └── .gitkeep
├── <project>_bt_node.launch
└── <project>_bringup.launch
```

All `__init__.py` files are empty.

**`algorithms/` architecture:**
- Pure computation only: visual error → gait params, target selection, state judgment helpers.
- NO `rospy`, `gait_manager`, `motion_manager`, publisher, service, or `emit_comm` here.
- Called exclusively by `semantics/semantic_facade.py`.
- To surface debug info: return structured dicts (e.g. `{'gait_yaw': 0.3, 'reason': 'line_lost'}`);
  semantic_facade passes them as payload/summary into the comm_facade call;
  comm_facade records them in `ros_out` via `_emit()`.

### Step 3 — Generate skeleton files

Render the following templates from `assets/templates/`, substituting:
- `{{PROJECT}}` → project_name (snake_case), e.g. `ball_kick`
- `{{PROJECT_CLASS}}` → PascalCase, e.g. `BallKick`
- `{{PROJECT_NODE_CLASS}}` → e.g. `BallKickBTNode`
- `{{PROJECT_UPPER}}` → `PROJECT.upper().replace('-','_')`, e.g. `BALL_KICK`
- `{{TASK_DESC}}` → task_description

Files to generate from templates:
| Template | Output |
|---|---|
| `bt_node.py.tpl` | `app/{{PROJECT}}_bt_node.py` |
| `project_bt.py.tpl` | `tree/{{PROJECT}}_bt.py` |
| `semantic_facade.py.tpl` | `semantics/semantic_facade.py` |
| `comm_facade.py.tpl` | `comm/comm_facade.py` |
| `infra_manifest.py.tpl` | `infra/infra_manifest.py` |
| `conditions.py.tpl` | `behaviours/conditions.py` |
| `actions.py.tpl` | `behaviours/actions.py` |
| `bt_exec_controller.py.tpl` | `infra/bt_exec_controller.py` |
| `tree_publisher.py.tpl` | `infra/tree_publisher.py` |
| `ros_msg_utils.py.tpl` | `app/ros_msg_utils.py` |
| `bb_ros_bridge.py.tpl` | `infra/bb_ros_bridge.py` |
| `bringup.launch.tpl` | `{{PROJECT}}_bringup.launch` |

### Step 4 — Node composition check

Verify the generated `tree/{{PROJECT}}_bt.py`:
- [ ] Imports `L1_Balance_IsStanding` from `ainex_bt_edu.behaviours.L1_perception`
- [ ] Imports `L1_Vision_IsLineDetected` from `ainex_bt_edu.behaviours.L1_perception`
- [ ] Imports `L2_Gait_Stop`, `L2_Gait_FindLine`, `L2_Gait_FollowLine`, `L2_Balance_RecoverFromFall`
      from `ainex_bt_edu.behaviours.L2_locomotion`
- [ ] Project-specific nodes come from `<project>.behaviours.*`
- [ ] No node inherits `py_trees.behaviour.Behaviour` directly

Verify BB constant usage across all generated behaviour nodes:
- [ ] All `BB_READS` / `BB_WRITES` / compatibility `BB_LOG_KEYS` use `BB.*` constants
      (never `'/latched/...'` strings)
- [ ] All `attach_blackboard_client` calls use `namespace=BB.LATCHED_NS`
- [ ] All `register_key` calls use `BB.*_KEY` for `/latched/` keys; `BB.HEAD_PAN_POS` for root-ns

### Step 5 — Interface check

Verify the generated `semantics/semantic_facade.py`:
- [ ] Class inherits `AinexBTFacade` (from `ainex_bt_edu.base_facade`)
- [ ] All 7 abstract methods implemented: `stop_walking`,
      `follow_line` (deprecated — no-op stub is acceptable), `gait_step`, `go_step`,
      `turn_step`, `recover_from_fall`, `move_head`
      (`search_line` and `head_sweep_align` removed from interface in v2.3.2)

### Step 6 — Update package.xml

Check `ainex_behavior/package.xml` for `<exec_depend>ainex_bt_edu</exec_depend>`.
If missing, add it before the closing `</package>` tag.

### Step 7 — Print checklist

Output the Section 10 checklist from `references/bt_observability_rules.md` (11 items),
marking which are auto-satisfied by the scaffold and which need manual completion:

```
✅ auto  [node composition] tree/ imports ainex_bt_edu standard nodes
✅ auto  [node composition] project condition nodes inherit AinexL1ConditionNode
✅ auto  [node composition] project action nodes inherit AinexL2ActionNode
✅ auto  [node composition] semantic_facade.py inherits AinexBTFacade
✅ auto  Step 1: DebugEventLogger created in __init__(), paths to log/
✅ auto  Step 2: BTDebugVisitor mounted to tree
✅ auto  Step 3: tick_id increments inside should_tick(), before tree.tick()
✅ auto  Step 4: ImuBalanceStateAdapter + LineDetectionAdapter from ainex_bt_edu/input_adapters/;
                 two-phase latch in run() (snapshot_and_reset under lock, write_snapshot after)
✅ auto  Step 5: comm_facade.py each public method calls _emit()
✅ auto  Step 6: logger.close() called in run() after loop
⬜ TODO  Step 7: infra_manifest.py — add project-specific interfaces
⬜ TODO  Step 8: check_imports.py passes (run after implementing callbacks)
⬜ TODO  [bb_bridge]  Add project-specific BB keys to bb_ros_bridge.py if needed;
                      keep in sync with infra_manifest.py PROJECT_INTERFACES
⬜ TODO  [bringup]    Fill in {{PROJECT}}_bringup.launch with required hardware nodes
⬜ TODO  [services]   Verify BT exec controller:
                        rosservice call /{{PROJECT}}_bt/bt/run   → mode='run'
                        rosservice call /{{PROJECT}}_bt/bt/pause → mode='pause'
                        rosservice call /{{PROJECT}}_bt/bt/step  → one tick, then pause
                        rostopic echo  /{{PROJECT}}_bt/bt/mode
```

### Step 8 — `chmod +x` the node script

```bash
chmod +x /home/pi/docker/ros_ws_src/ainex_behavior/{{PROJECT}}/app/{{PROJECT}}_bt_node.py
```

### Step 9 — Update `CMakeLists.txt`

Edit `docker/ros_ws_src/ainex_behavior/CMakeLists.txt`:

1. Add `{{PROJECT}}/app/{{PROJECT}}_bt_node.py` inside the existing `catkin_install_python(PROGRAMS ...)` block.
2. Append a new `install(DIRECTORY ...)` block after the last existing one:

```cmake
install(DIRECTORY {{PROJECT}}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/{{PROJECT}}
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.launch"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
    PATTERN "log" EXCLUDE)
```

### Step 10 — Launch files

**BT-only launch** (generated from template — `{{PROJECT}}_bt_node.launch`):

```xml
<launch>
  <node pkg="ainex_behavior" type="{{PROJECT}}_bt_node.py" name="{{PROJECT}}_bt"
        output="screen" required="true">
    <param name="start" value="true"/>
    <!-- <param name="bt_mode" value="run"/> -->
    <!-- <param name="max_rolling_ticks" value="30"/> -->
  </node>
</launch>
```

**Full hardware bringup** (`{{PROJECT}}_bringup.launch`) is generated from
`bringup.launch.tpl`. It includes base hardware + camera + BT node. The user
must fill in the TODO sections for project-specific nodes (object detection,
vision pipeline, etc.).

Launch with BT-only (no hardware):
```bash
roslaunch ainex_behavior {{PROJECT}}_bt_node.launch
```

Launch with full hardware:
```bash
roslaunch ainex_behavior {{PROJECT}}_bringup.launch [use_camera:=true]
```

### Step 11 — Build and verify

```bash
# Build
docker exec ainex bash -c "
  source /opt/ros/noetic/setup.bash &&
  cd /home/ubuntu/ros_ws &&
  catkin build ainex_behavior
"

# Verify wrapper installed
docker exec ainex bash -c "
  ls /home/ubuntu/ros_ws/devel/lib/ainex_behavior/{{PROJECT}}_bt_node.py
"
```

Launch with:
```bash
roslaunch ainex_behavior {{PROJECT}}_bt_node.launch
```

---

### Validation commands (Python import checks)

```bash
# 1. Import check (host)
cd /home/pi/docker/ros_ws_src
python3 -c "
import sys
sys.path.insert(0, 'ainex_behavior')
sys.path.insert(0, 'ainex_bt_edu/src')
from {{PROJECT}}.tree.{{PROJECT}}_bt import bootstrap
print('tree import OK')
"

# 2. Interface check (host)
python3 -c "
import sys
sys.path.insert(0, 'ainex_behavior')
sys.path.insert(0, 'ainex_bt_edu/src')
from {{PROJECT}}.infra.infra_manifest import build_infra_manifest
items = build_infra_manifest('{{PROJECT}}_bt')
print('infra manifest OK —', len(items), 'interfaces')
"

# 3. Run inside container after catkin build
docker exec ainex bash -c "
  source /home/ubuntu/ros_ws/devel/setup.bash
  python3 -c 'from {{PROJECT}}.tree.{{PROJECT}}_bt import bootstrap; print(\"OK\")'
"
```

---

## Mode 2 — Modify existing BT project (add / update BT node)

Triggered when user mentions an existing project name + adding or changing nodes.

### Step 1 — Identify target

Ask if not clear:
- Which project? (check `ainex_behavior/` for existing dirs)
- What kind of node? `L1` condition / `L2` action / project-specific
- What does the node do? (one sentence)

### Step 2 — Check ainex_bt_edu first

Read `ainex_bt_edu/src/ainex_bt_edu/behaviours/` to check if an equivalent node
already exists.

- **If yes** → import it directly in `tree/<project>_bt.py`. Stop here.
- **If no, and it is a condition** (L1 perception check, SUCCESS/FAILURE only)
  → add to `behaviours/conditions.py`. Proceed to Step 3.
- **If no, and it is an action** (L2+ actuator command, may return RUNNING)
  → add to `behaviours/actions.py`. Rules for new action nodes:
  - Only allowed when NO generic equivalent exists in `ainex_bt_edu`.
  - Must inherit `AinexL2ActionNode` — never `py_trees.behaviour.Behaviour` directly.
  - No direct `rospy`, `gait_manager`, `motion_manager`, publisher, or service calls.
  - All ROS output via `self._facade.*` → `semantic_facade` → `comm_facade`.
  - `logger=None` must be zero-cost no-op.
  - Emit `decision` and/or `action_intent` via `self.emit_decision()` /
    `self.emit_action_intent()`;
    `ros_out` is emitted ONLY by `comm_facade._emit()` — never in the BT node.
  Proceed to Step 3.

### Step 3 — System trace (required before writing any code)

Apply the **System Thinking Principle**: trace the full adapter → BB → behaviour chain
for every BB key the new node will read or write.

For each BB key:

1. **Check `blackboard_keys.py`** — is the key already defined?
   - Yes → use existing `BB.*_KEY` / `BB.*` constants.  Do not hardcode strings.
   - No → add to `blackboard_keys.py` **before** writing the node:
     ```python
     SOME_KEY_KEY = 'short_name'                            # register_key() argument
     SOME_KEY     = LATCHED_NS + '/' + SOME_KEY_KEY         # BB_READS / BB_WRITES / bb_writes
     ```

2. **Check input_adapters** — which adapter writes that key?
   - Key in active adapter table (see System Thinking section) → adapter exists; no change.
   - New key → create a new adapter in `ainex_bt_edu/input_adapters/` following the
     `AinexInputAdapter` base-class contract and
     two-phase latch protocol (`snapshot_and_reset` under lock, `write_snapshot` after),
     emitting `ros_in` + `input_state`.  Wire it into `app/<project>_bt_node.py`
     `__init__()` + `run()`.

3. **Only then** proceed to Step 4 to write the behaviour node.

> **Never use deprecated keys**: `/locomotion/robot_state` and `/perception/line_data`
> are not written by any active adapter.  Do not create nodes that read them.

### Step 4 — Generate project-specific node

Create file in `<project>/behaviours/` following this checklist:

- [ ] Add `from ainex_bt_edu.blackboard_keys import BB` import
- [ ] L1 condition inherits `AinexL1ConditionNode`; L2 action inherits `AinexL2ActionNode`
      (not `py_trees.behaviour.Behaviour`)
- [ ] Declares `LEVEL = 'L1' | 'L2' | 'L3'`
- [ ] Declares `BB_READS` / `BB_WRITES` and optional compatibility `BB_LOG_KEYS`
      with absolute `BB.*` constants — never `'/latched/...'`
- [ ] Constructor accepts `facade: AinexBTFacade` (if it calls any ROS action)
- [ ] All ROS actions delegated through `facade.*`, no direct rospy calls
- [ ] `attach_blackboard_client(namespace=BB.LATCHED_NS)` — never hardcode `'/latched/'`
- [ ] `register_key(key=BB.SOME_KEY_KEY)` for `/latched/` keys; `key=BB.HEAD_PAN_POS` for root-ns
- [ ] If condition node: optionally emits `"decision"` event via `self.emit_decision()`

Use `assets/templates/conditions.py.tpl` as the starting skeleton for condition nodes.
Use `assets/templates/actions.py.tpl` as the starting skeleton for action nodes.

### Step 5 — Wire into tree

Show the exact diff to `tree/<project>_bt.py` — where the new node slots into the
existing tree structure. Consider:
- Does it replace an existing node?
- Does it gate another node (i.e., insert into a Sequence before it)?
- Does it add a new branch to a Selector?

### Step 6 — Check if SemanticFacade needs a new method

If the new node calls a `facade.*` method that doesn't exist yet:
1. Add the abstract method to `ainex_bt_edu/base_facade.py`
2. Implement it in `<project>/semantics/semantic_facade.py` (delegates to `comm_facade.*`)
3. Add the corresponding `comm_facade.*` method if needed (with `_emit()` call)

Verify `AinexBTFacade` still fully implemented: `isinstance(facade, AinexBTFacade)` → True.

### Step 7 — Check infra_manifest and bb_ros_bridge

If the new node (or its supporting callbacks) introduces new ROS interfaces,
BB keys, or input adapters:

- Determine if the interface is **infra** (framework-level) or **business_in/out**
- If infra → add record to `<project>/infra/infra_manifest.py`
- If business_in → provided by `ainex_bt_edu/input_adapters/` (no `app/` change needed;
  adapters handle `ros_in` + `input_state` automatically)
- If new BB key introduced → also add to `<project>/infra/bb_ros_bridge.py`
  (`{{PROJECT_UPPER}}_BB_TOPIC_MAP`) so ROSA/debug tools can observe it
- These two must stay consistent: any BB key in the bridge must have a
  corresponding record in `infra_manifest.py` PROJECT_INTERFACES
- `write_infra_manifest()` runs automatically on next node startup

### Step 8 — Run import checker

Remind user to verify no layered import violations:

```bash
cd /home/pi/docker/ros_ws_src/ainex_behavior
python3 <project>/check_imports.py
```

If `check_imports.py` doesn't exist yet for this project, copy from:
`marathon/check_imports.py` and adapt the `PROJECT` constant at the top.
