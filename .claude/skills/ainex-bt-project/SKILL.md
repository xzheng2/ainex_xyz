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
- All project nodes inherit `AinexBTNode` (not `py_trees.behaviour.Behaviour`)
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
│   └── conditions.py
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
│   ├── tree_publisher.py      ← copy from marathon/infra/tree_publisher.py
│   ├── bb_ros_bridge.py       ← create project-specific bridge
│   └── bt_exec_controller.py  ← copy from marathon/infra/bt_exec_controller.py
└── log/
    └── .gitkeep
```

All `__init__.py` files are empty.

### Step 3 — Generate skeleton files

Render the following templates from `assets/templates/`, substituting:
- `{{PROJECT}}` → project_name (snake_case), e.g. `ball_kick`
- `{{PROJECT_CLASS}}` → PascalCase, e.g. `BallKick`
- `{{PROJECT_NODE_CLASS}}` → e.g. `BallKickBTNode`
- `{{TASK_DESC}}` → task_description

Files to generate from templates:
| Template | Output |
|---|---|
| `assets/templates/bt_node.py.tpl` | `app/{{PROJECT}}_bt_node.py` |
| `assets/templates/project_bt.py.tpl` | `tree/{{PROJECT}}_bt.py` |
| `assets/templates/semantic_facade.py.tpl` | `semantics/semantic_facade.py` |
| `assets/templates/comm_facade.py.tpl` | `comm/comm_facade.py` |
| `assets/templates/infra_manifest.py.tpl` | `infra/infra_manifest.py` |
| `assets/templates/conditions.py.tpl` | `behaviours/conditions.py` |

Also copy `marathon/app/ros_msg_utils.py` verbatim → `app/ros_msg_utils.py`.

### Step 4 — Node composition check

Verify the generated `tree/{{PROJECT}}_bt.py`:
- [ ] Imports `L1_Balance_IsStanding` from `ainex_bt_edu.behaviours.L1_perception`
- [ ] Imports `L1_Vision_IsLineDetected` from `ainex_bt_edu.behaviours.L1_perception`
- [ ] Imports `L2_Gait_Stop`, `L2_Gait_FindLine`, `L2_Gait_FollowLine`, `L2_Balance_RecoverFromFall`
      from `ainex_bt_edu.behaviours.L2_locomotion`
- [ ] Project-specific nodes come from `<project>.behaviours.*`
- [ ] No node inherits `py_trees.behaviour.Behaviour` directly

### Step 5 — Interface check

Verify the generated `semantics/semantic_facade.py`:
- [ ] Class inherits `AinexBTFacade` (from `ainex_bt_edu.base_facade`)
- [ ] All 6 abstract methods implemented: `stop_walking`, `follow_line`, `search_line`,
     `recover_from_fall`, `move_head`, `head_sweep_align`

### Step 6 — Update package.xml

Check `ainex_behavior/package.xml` for `<exec_depend>ainex_bt_edu</exec_depend>`.
If missing, add it before the closing `</package>` tag.

### Step 7 — Print checklist

Output the Section 10 checklist from `references/bt_observability_rules.md` (11 items),
marking which are auto-satisfied by the scaffold and which need manual completion:

```
✅ auto  [node composition] tree/ imports ainex_bt_edu standard nodes
✅ auto  [node composition] project behaviours/ nodes inherit AinexBTNode
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

### Step 10 — Create launch file

Create `docker/ros_ws_src/ainex_behavior/{{PROJECT}}/{{PROJECT}}_bt_node.launch`:

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

- **If yes** → instruct user to import it directly in `tree/<project>_bt.py`. Stop here.
- **If no** → proceed to Step 3.

### Step 3 — Generate project-specific node

Create file in `<project>/behaviours/` following this checklist:

- [ ] Inherits `AinexBTNode` (not `py_trees.behaviour.Behaviour`)
- [ ] Declares `LEVEL = 'L1' | 'L2' | 'L3'`
- [ ] Declares `BB_LOG_KEYS = ['/latched/...']` for all blackboard keys it reads
- [ ] Constructor accepts `facade: AinexBTFacade` (if it calls any ROS action)
- [ ] All ROS actions delegated through `facade.*`, no direct rospy calls
- [ ] Uses `/latched/` namespace for blackboard clients (keys written by `_latch_inputs`)
- [ ] If condition node: optionally emits `"decision"` event via `self._logger.emit_bt()`

Use `assets/templates/conditions.py.tpl` as the starting skeleton.

### Step 4 — Wire into tree

Show the exact diff to `tree/<project>_bt.py` — where the new node slots into the
existing tree structure. Consider:
- Does it replace an existing node?
- Does it gate another node (i.e., insert into a Sequence before it)?
- Does it add a new branch to a Selector?

### Step 5 — Check if SemanticFacade needs a new method

If the new node calls a `facade.*` method that doesn't exist yet:
1. Add the abstract method to `ainex_bt_edu/base_facade.py`
2. Implement it in `<project>/semantics/semantic_facade.py` (delegates to `comm_facade.*`)
3. Add the corresponding `comm_facade.*` method if needed (with `_emit()` call)

Verify `AinexBTFacade` still fully implemented: `isinstance(facade, AinexBTFacade)` → True.

### Step 6 — Check infra_manifest

If the new node (or its supporting callbacks) introduces new ROS interfaces
(subscriber, publisher, service client):

- Determine if the interface is **infra** (framework-level) or **business_in/out**
- If infra → add record to `<project>/infra/infra_manifest.py`
- If business_in → provided by `ainex_bt_edu/input_adapters/` (no `app/` change needed; adapters handle `ros_in` + `input_state` automatically)
- `write_infra_manifest()` runs automatically on next node startup

### Step 7 — Run import checker

Remind user to verify no layered import violations:

```bash
cd /home/pi/docker/ros_ws_src/ainex_behavior
python3 <project>/check_imports.py
```

If `check_imports.py` doesn't exist yet for this project, copy from:
`marathon/check_imports.py` and adapt the `PROJECT` constant at the top.
