# ainex_bt_edu — Migration Progress

> **Last updated**: 2026-04-18
> **Spec version**: 2.3.1
> **Canonical nodes**: 13 (7 L1 + 6 L2)

---

## Background

`behaviours_tem/` held 25 legacy BT nodes (11 L1, 14 L2) written before the
current architecture was established. They violated several rules:

- `rospy.Subscriber` calls inside node classes (forbidden since v2.3.0)
- Direct `py_trees.blackboard.Blackboard.storage[]` access (non-standard)
- No observability logger injection
- No facade pattern for hardware calls (L2 nodes called managers directly)

A three-phase migration plan was approved to bring all nodes up to spec,
promote them to `behaviours/`, and delete the legacy sources.

---

## Infrastructure Created

### ainex-bt-edu-extend Skill

**Path**: `/home/pi/.claude/skills/ainex-bt-edu-extend/`

Two-mode skill for extending the `ainex_bt_edu` package:

| Mode | What it creates | Template |
|---|---|---|
| Mode 1 | L1 or L2 BT node in `behaviours/` | `l1_node.py.tpl` / `l2_node.py.tpl` |
| Mode 2 | Input adapter in `input_adapters/` | `input_adapter.py.tpl` |

### Hooks (ainex_bt_edu guard pair)

Registered in `/home/pi/.claude/settings.json`:

| Hook | Trigger | Purpose |
|---|---|---|
| `ainex_bt_edu_pre_guard.py` | PreToolUse Write/Edit on `behaviours/` or `input_adapters/` | Injects architecture reminder before edit |
| `ainex_bt_edu_guard.py` | PostToolUse Write/Edit on same paths | Checks node/adapter compliance; flags violations |

---

## Step 0 — Cancelled (6 files, not promoted)

Deleted from `behaviours_tem/` because canonical equivalents already exist or
the node is a near-duplicate of an existing canonical node.

| behaviours_tem file | Reason | Canonical equivalent |
|---|---|---|
| `L1_Balance_IsStanding.py` | Exact duplicate | `behaviours/L1_perception/L1_Balance_IsStanding.py` |
| `L1_Vision_IsLineDetected.py` | Exact duplicate | `behaviours/L1_perception/L1_Vision_IsLineDetected.py` |
| `L2_Balance_RecoverFromFall.py` | Exact duplicate | `behaviours/L2_locomotion/L2_Balance_RecoverFromFall.py` |
| `L2_Gait_FollowLine.py` | Exact duplicate | `behaviours/L2_locomotion/L2_Gait_FollowLine.py` |
| `L2_Gait_Stop.py` | Exact duplicate | `behaviours/L2_locomotion/L2_Gait_Stop.py` |
| `L2_Gait_Disable.py` | Near-duplicate of `L2_Gait_Stop` | calls `gait_manager.disable()` = `facade.stop_walking()` |

---

## Phase 1 — Group A (5 nodes promoted, no blockers)

All 5 nodes use only existing infrastructure (no new adapter or facade method needed).

| Node | BB reads | BB writes | Note |
|---|---|---|---|
| `L1_Balance_IsFallen` | `/latched/robot_state` | — | SUCCESS if state ≠ 'stand' |
| `L1_Motion_ListActions` | — | `/mission/available_actions` | Scans `.d6a` action files |
| `L1_Vision_IsObjectStill` | `/perception/detected_objects` | — | Euclidean stability over N ticks |
| `L1_Vision_IsTargetOnLeft` | `/perception/target_pixel_x` | — | SUCCESS if x < image_width/2 |
| `L2_Head_MoveTo` | — | — | Calls `facade.move_head(pan_pos)` |

**Spec bump**: v2.3.0 → v2.3.1, node count 8 → 13.

---

## What Is Left

### Group B — New input adapter required first

Three adapters + three nodes. Each adapter must be written first, then the node.

#### B1 — ObjectDetectionAdapter → L1_Vision_IsObjectTypeDetected *(planned, next session)*

YOLO publishes to `/object/pixel_coords` (same topic as line detection).
A new `ObjectDetectionAdapter` filters out `type='line'` objects (handled by
`LineDetectionAdapter`) and writes the rest to the `/perception/` BB namespace.

**Adapter details**:

| Item | Value |
|---|---|
| Topic | `/object/pixel_coords` (ObjectsInfo) |
| Filters out | `type == 'line'` |
| Writes: `detected_objects` | ObjectsInfo of non-line objects |
| Writes: `face_detected` | True if any `obj.type=='face'` or `obj.label=='face'` |
| Writes: `target_pixel_x/y` | First object's center (or None) |
| BB namespace | `/perception` |

**YOLO ObjectInfo format** (from `yolo_camera.py`):
- `label` = COCO class name (e.g., `"cat"`, `"stop sign"`, `"bottle"`)
- `type` = always `"rect"`
- `x`, `y` = center pixel
- `width`, `height` = bounding box

**Node** — `L1_Vision_IsObjectTypeDetected(object_type=None)`:
- Replaces 3 merged legacy nodes: `IsObjectDetected`, `IsFaceDetected`, `IsPatternDetected`
- `object_type=None` → SUCCESS if any object detected
- `object_type='cat'` → SUCCESS if any object matches `label=='cat'` OR `type=='cat'`
  (dual check supports both YOLO label-based and legacy type-based detection)

**blackboard_keys.py changes needed**:
```python
PERCEPTION_NS        = '/perception'
DETECTED_OBJECTS_KEY = 'detected_objects'
FACE_DETECTED_KEY    = 'face_detected'
TARGET_PIXEL_X_KEY   = 'target_pixel_x'
TARGET_PIXEL_Y_KEY   = 'target_pixel_y'
```

#### B2 — BatteryAdapter → L1_Battery_IsVoltageOk

| Item | Value |
|---|---|
| Topic | `/ainex/battery` (UInt16, mV) |
| New BB key | `BB.BATTERY_VOLTAGE` / `BB.BATTERY_VOLTAGE_KEY` |
| Node | SUCCESS if voltage_mv ≥ threshold (default 11100 mV) |

#### B3 — ButtonAdapter → L1_System_IsButtonPressed

| Item | Value |
|---|---|
| Topic | `/sensor/button/get_button_state` (Bool) |
| New BB key | `BB.BUTTON_PRESSED` / `BB.BUTTON_PRESSED_KEY` |
| Node | SUCCESS when button is pressed |

---

### Group C — New facade method required first (one session per method)

Each node requires a new `@abstractmethod` added to `base_facade.py`, stub
implementations in all existing `semantic_facade.py` files, and a template
update in the `ainex-bt-project` skill.

| New facade method | Node(s) |
|---|---|
| `enable_gait(bt_node, tick_id)` | `L2_Gait_Enable` |
| `start_walking(bt_node, tick_id)` | `L2_Gait_Start` |
| `set_gait_param(x, y, angle, body_height, bt_node, tick_id)` | `L2_Gait_SetParam` |
| `track_head(target_x, target_y, bt_node, tick_id)` | `L2_Head_TrackObject` |
| `move_arm_to_preset(positions, duration, bt_node, tick_id)` | `L2_Arm_MoveToPreset` |
| `play_action(action_name, bt_node, tick_id)` | `L2_Motion_PlayAction` + `L2_Motion_PlayFromBB` |
| `set_color_target(color, roi, bt_node, tick_id)` | `L2_Vision_SetColorTarget` |
| `update_roi(roi, bt_node, tick_id)` | `L2_Vision_UpdateROI` |

---

## behaviours_tem Remaining Files

### `src/behaviours_tem/L1_perception/` — 2 files

| File | Blocked by |
|---|---|
| `L1_Battery_IsVoltageOk.py` | Group B2 (BatteryAdapter) |
| `L1_System_IsButtonPressed.py` | Group B3 (ButtonAdapter) |

*(3 vision files deleted when B1 merged node is written)*

### `src/behaviours_tem/L2_locomotion/` — 9 files

| File | Blocked by |
|---|---|
| `L2_Gait_Enable.py` | Group C: `enable_gait()` |
| `L2_Gait_Start.py` | Group C: `start_walking()` |
| `L2_Gait_SetParam.py` | Group C: `set_gait_param()` |
| `L2_Head_TrackObject.py` | Group C: `track_head()` |
| `L2_Arm_MoveToPreset.py` | Group C: `move_arm_to_preset()` |
| `L2_Motion_PlayAction.py` | Group C: `play_action()` |
| `L2_Motion_PlayFromBB.py` | Group C: `play_action()` |
| `L2_Vision_SetColorTarget.py` | Group C: `set_color_target()` |
| `L2_Vision_UpdateROI.py` | Group C: `update_roi()` |

### `src/behaviours_tem/L3_mission/` — 11 files

Untouched. L3 nodes require their own planning session (they combine multiple
L1/L2 nodes and may require additional adapters and facade methods).

---

## Canonical behaviours/ — Current State (13 nodes)

### L1_perception/ (7 files)

| File | BB reads | BB writes |
|---|---|---|
| `L1_Balance_IsFallen.py` | `/latched/robot_state` | — |
| `L1_Balance_IsStanding.py` | `/latched/robot_state` | — |
| `L1_Head_IsHeadCentered.py` | `/head_pan_pos` | — |
| `L1_Motion_ListActions.py` | — | `/mission/available_actions` |
| `L1_Vision_IsLineDetected.py` | `/latched/line_data` | — |
| `L1_Vision_IsObjectStill.py` | `/perception/detected_objects` | — |
| `L1_Vision_IsTargetOnLeft.py` | `/perception/target_pixel_x` | — |

### L2_locomotion/ (6 files)

| File | Facade method | BB reads |
|---|---|---|
| `L2_Balance_RecoverFromFall.py` | `play_action_group()` | `/latched/robot_state` |
| `L2_Gait_FindLine.py` | `set_step()` | `/latched/line_data` |
| `L2_Gait_FollowLine.py` | `set_step()` | `/latched/line_data` |
| `L2_Gait_Stop.py` | `stop_walking()` | — |
| `L2_Head_FindLineSweep.py` | `move_head()` | `/latched/line_data`, `/head_pan_pos` |
| `L2_Head_MoveTo.py` | `move_head()` | — |

---

## Key Architecture Rules (v2.3.0+)

1. **No `rospy.*` in BT nodes** — all ROS subscriptions live in `input_adapters/`
2. **Two-phase latch** — `snapshot_and_reset()` under lock, `write_snapshot()` after
3. **BB client access** — always `attach_blackboard_client(namespace=...)` + `register_key()`; never `Blackboard.storage[]`
4. **Facade isolation** — L2 nodes call `self._facade.<method>()`, never managers directly
5. **Logger injection** — `logger=None` is zero-cost; always `if self._logger: ...`
6. **One subscriber per topic** — each ROS topic has exactly one adapter

---

## File Locations

| Path | Description |
|---|---|
| `ainex_bt_edu/ainex_bt_edu_spec.md` | Canonical spec (authoritative, v2.3.1) |
| `ainex_bt_edu/src/ainex_bt_edu/behaviours/` | Promoted canonical BT nodes |
| `ainex_bt_edu/src/ainex_bt_edu/input_adapters/` | ROS→BB adapters |
| `ainex_bt_edu/src/ainex_bt_edu/blackboard_keys.py` | All BB key constants |
| `ainex_bt_edu/src/ainex_bt_edu/base_facade.py` | Facade abstract interface |
| `ainex_bt_edu/src/behaviours_tem/` | Legacy sources (not inside the package) |
| `/home/pi/.claude/skills/ainex-bt-edu-extend/` | Skill for extending this package |
| `/home/pi/.claude/hooks/ainex_bt_edu_guard.py` | PostToolUse compliance check |
| `/home/pi/.claude/hooks/ainex_bt_edu_pre_guard.py` | PreToolUse reminder |
