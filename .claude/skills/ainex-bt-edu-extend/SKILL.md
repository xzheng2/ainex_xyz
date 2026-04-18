# Skill: `ainex-bt-edu-extend`

Extend the **ainex_bt_edu** shared BT library by adding standard behaviour nodes
(L1 condition or L2 action) or input adapters.

**Scope**: This skill targets `ainex_bt_edu` only.
For project-level work under `ainex_behavior/`, use the `ainex-bt-project` skill.

---

## Trigger phrases

| Phrase | Mode |
|---|---|
| "加L1节点", "add L1 condition", "new condition node", "new L1 node" | Mode 1 — Add BT node |
| "加L2节点", "add L2 action", "new action node", "new L2 node" | Mode 1 — Add BT node |
| "加 input adapter", "add adapter", "new sensor", "subscribe to /topic", "新传感器" | Mode 2 — Add input adapter |

If ambiguous: "你是要给 ainex_bt_edu 加标准 BT 节点，还是加 input adapter？"

---

## Key paths

| Path | Role |
|---|---|
| `ainex_bt_edu/src/ainex_bt_edu/blackboard_keys.py` | BB key constants — always read first |
| `ainex_bt_edu/src/ainex_bt_edu/base_facade.py` | Facade abstract interface (L2 only) |
| `ainex_bt_edu/src/ainex_bt_edu/behaviours/L1_perception/` | L1 node output dir |
| `ainex_bt_edu/src/ainex_bt_edu/behaviours/L2_locomotion/` | L2 node output dir |
| `ainex_bt_edu/src/ainex_bt_edu/input_adapters/` | Adapter output dir |
| `ainex_bt_edu/package.xml` | ROS package dependencies |
| `ainex_bt_edu/ainex_bt_edu_spec.md` | Always update after any change |

Reference implementations:
- L1: `L1_Balance_IsStanding.py` (simplest, clean pattern)
- L2: `L2_Gait_FindLine.py` (pure RUNNING) + `L2_Balance_RecoverFromFall.py` (BB write + setter hook)
- Adapter: `imu_balance_state_adapter.py` (single key) + `line_detection_adapter.py` (multi-key)

Templates: `assets/templates/l1_node.py.tpl`, `l2_node.py.tpl`, `input_adapter.py.tpl`

---

## Mode 1 — Add standard BT node (L1/L2)

### Information to collect

1. **level** — `L1` (condition, no facade) or `L2` (action, uses facade)
2. **name** — `L1_Module_Description` or `L2_Module_Description` format
3. **what it does** — one sentence
4. **BB keys it reads** — which `/latched/` keys (from `blackboard_keys.py`)
5. **BB keys it writes** (L2 only)
6. **facade method it calls** (L2 only)

### Step 1 — Read `blackboard_keys.py` + check/add BB keys

Always read `blackboard_keys.py` first. Convention (verified):
```python
ROBOT_STATE_KEY = 'robot_state'                       # short/relative key
ROBOT_STATE     = LATCHED_NS + '/' + ROBOT_STATE_KEY  # '/latched/robot_state'
```
- `BB.LATCHED_NS = '/latched'` — already defined, never redefine.
- Root-namespace keys (no `/latched/` prefix) have no `_KEY` suffix:
  `BB.HEAD_PAN_POS = '/head_pan_pos'`
- If required `BB.*_KEY` / `BB.*` constants are missing → add them following the exact pattern above.

### Step 2 — Check adapter writes that BB key

Scan `ainex_bt_edu/src/ainex_bt_edu/input_adapters/` for the adapter that writes the key.
If none → warn: "No adapter writes BB.SOME_KEY yet. The node will always read its initial
value. Consider adding an adapter first (Mode 2)."

### Step 2b — Facade method check (L2 only)

Read `base_facade.py` and verify the required method exists.

- **Method exists** → proceed to Step 3.
- **Method does NOT exist** → **facade contract change sub-process** (plan required):
  1. Add `@abstractmethod` to `base_facade.py`
  2. Implement stub in **all** existing project `semantic_facade.py` files
     (minimum: `raise NotImplementedError(f"{type(self).__name__} does not implement <method>")`)
  3. Update `ainex-bt-project` skill's `semantic_facade.py.tpl`
  4. Update `comm_facade.py.tpl` if a new comm_facade method is also needed

  > **Rule**: New L2 nodes must only use existing facade methods.
  > Adding an abstract method is a breaking change — plan it explicitly.

### Step 3 — Generate node file

Render `l1_node.py.tpl` or `l2_node.py.tpl`.

Key substitution variables:
- `{{CLASS_NAME}}` — e.g. `L1_Balance_IsAtStart`
- `{{DEFAULT_NAME}}` — e.g. `'L1_Balance_IsAtStart'` (with quotes)
- `{{LEVEL}}` — `'L1'` or `'L2'`
- `{{DESCRIPTION}}` — one-sentence docstring
- `{{BB_LOG_KEYS}}` — e.g. `[BB.ROBOT_STATE]`
- `{{FACADE_METHOD}}` — L2 only

**No `import rospy`** in node files. `L2_Gait_FindLine.py` has a legacy `rospy.logdebug` —
do NOT copy it. Use `self._logger.emit_bt()` for all debug output.

Output paths:
- L1: `ainex_bt_edu/src/ainex_bt_edu/behaviours/L1_perception/{{CLASS_NAME}}.py`
- L2: `ainex_bt_edu/src/ainex_bt_edu/behaviours/L2_locomotion/{{CLASS_NAME}}.py`

### Step 4 — Update `__init__.py`

Check `L1_perception/__init__.py` or `L2_locomotion/__init__.py`. If it has explicit
imports, add:
```python
from ainex_bt_edu.behaviours.L1_perception.{{CLASS_NAME}} import {{CLASS_NAME}}
```
(Many `__init__.py` files are empty — check before editing.)

### Step 5 — Update `ainex_bt_edu_spec.md`

- Read the spec: find current node count (`当前 N 个`) and document version (`文档版本`).
- Increment node count by 1.
- Add node entry under the correct heading: file path, signature, BB reads/writes, logic.
- Bump version (minor increment: new node → patch; bigger change → minor) and update date.
- Add version history entry.

### Step 6 — Print checklist

```
✅ auto  BB key constants in blackboard_keys.py
✅ auto  Node file generated with correct imports + BB.* constants
✅ auto  Inherits AinexBTNode; LEVEL + BB_LOG_KEYS declared
✅ auto  setup() calls super().setup(**kwargs)
✅ auto  update() returns Status; logger=None safe; no rospy import
⬜ TODO  Verify adapter writes required BB key (if new key added)
⬜ TODO  Wire node into tree/<project>_bt.py when used in a project
⬜ TODO  catkin build ainex_bt_edu (inside container)
⬜ TODO  Import check: python3 -c "from ainex_bt_edu.behaviours... import ..."
```

---

## Mode 2 — Add input adapter

### Information to collect

1. **ROS topic** — e.g. `/camera/depth`
2. **message type** — e.g. `sensor_msgs/Image`
3. **what to extract** — describe the data (e.g. "average depth of center ROI")
4. **BB keys to write** — new `/latched/` keys (will add to `blackboard_keys.py`)
5. **adapter class name** — e.g. `DepthCameraAdapter`

### Step 1 — Add BB key constants to `blackboard_keys.py`

For each new BB key the adapter will write:
```python
SOME_VALUE_KEY = 'some_value'
SOME_VALUE     = LATCHED_NS + '/' + SOME_VALUE_KEY
```

### Step 1b — Update `package.xml` if new message type needed

Check `ainex_bt_edu/package.xml` for the message package (e.g. `sensor_msgs`, `nav_msgs`).
If `<exec_depend>new_msgs_package</exec_depend>` is missing → add it.
Standard packages already present: `std_msgs`, `sensor_msgs` — verify before adding.

### Step 2 — Generate adapter from template

Render `input_adapter.py.tpl`.

Key substitution variables:
- `{{CLASS_NAME}}` — e.g. `DepthCameraAdapter`
- `{{ROS_TOPIC}}` — e.g. `/camera/depth`
- `{{MSG_TYPE_IMPORT}}` — e.g. `from sensor_msgs.msg import Image`
- `{{MSG_TYPE}}` — e.g. `Image`
- `{{DESCRIPTION}}` — one sentence
- `{{BB_KEY_LIST}}` — list of BB keys written (for docstring)
- `{{BB_KEY_INITS}}` — live state variable declarations
- `{{BB_REGISTER_KEYS}}` — `self._bb.register_key(...)` calls
- `{{BB_INIT_WRITES}}` — initial BB writes before first tick
- `{{CALLBACK_LOGIC}}` — live state update inside `with self._lock:`
- `{{SNAP_FIELDS}}` — snapshot dict fields
- `{{BB_WRITES}}` — BB write statements in `write_snapshot()`
- `{{INPUT_STATE_BB_WRITES}}` — `bb_writes` dict in `input_state` emit

Output path: `ainex_bt_edu/src/ainex_bt_edu/input_adapters/{{class_name_snake}}.py`
(snake_case of class name, e.g. `depth_camera_adapter.py`)

### Step 3 — Update `input_adapters/__init__.py`

If it has explicit exports, add the new class.

### Step 4 — Update `ainex_bt_edu_spec.md`

- Read current version from spec.
- Add adapter entry under `input_adapters/` section: file path, topic, BB keys, logic.
- Bump version and update date.
- Add version history entry.

### Step 5 — Print integration instructions

```
To use this adapter in a BT project:

1. Import in app/<project>_bt_node.py __init__():
   from ainex_bt_edu.input_adapters.<snake_name> import <ClassName>
   self._<var> = <ClassName>(
       lock=self.lock, logger=self._obs_logger,
       tick_id_getter=lambda: self._tick_id)

2. Add to run() two-phase latch block:
   with self.lock:
       ...
       <var>_snap = self._<var>.snapshot_and_reset()
   ...
   self._<var>.write_snapshot(<var>_snap, self._tick_id)

3. Sync bb_ros_bridge.py (infra/<project>/bb_ros_bridge.py):
   Add: BB.SOME_VALUE: '/bt/<project>/bb/some_value'

4. Sync infra_manifest.py — add topic_sub record for <ROS_TOPIC>

5. Rebuild: catkin build ainex_bt_edu ainex_behavior
```

---

## Verification

```bash
# Import check (host — no ROS needed)
cd /home/pi/docker/ros_ws_src
python3 -c "
import sys; sys.path.insert(0, 'ainex_bt_edu/src')
from ainex_bt_edu.behaviours.L1_perception.<ClassName> import <ClassName>
print('import OK')
"

# Build check (inside container)
docker exec ainex bash -c "
  cd /home/ubuntu/ros_ws && catkin build ainex_bt_edu
"
```
