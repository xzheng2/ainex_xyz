# 技术规格说明：ainex_bt_edu — AiNex 机器人行为树教育框架

---

## 版本信息

| 字段 | 值 |
|------|---|
| 文档版本 | 1.2.1 |
| 日期 | 2026-03-16 |
| ROS 版本 | ROS Noetic (Ubuntu 20.04) |
| 目标包路径 | `ainex_xyz-master/docker/ros_ws_src/ainex_bt_edu/` |
| 依赖基础 | `ainex_behavior/marathon`（现有 BT 实现） |

---

## 背景与目标

为 AiNex 人形机器人（ROS1 Noetic，Python 3）构建一套面向教育场景的 Behavior Tree（BT）框架包 `ainex_bt_edu`。基于 `py_trees` 实现，叠加以下能力层：

1. **自动结构化日志**：每个节点状态转换（任意 → SUCCESS/FAILURE）时，无需修改业务逻辑，自动向 `/rosout` 和 `/bt_node_events` 发布结构化日志。
2. **三级能力分层**：节点携带 `LEVEL` 属性（L1/L2/L3），下游可按 level 过滤可见信息。
3. **Run 生命周期管理**：根节点返回终态时自动发布 `/bt_run_complete` topic。
4. **ROSA 集成**：关键 Blackboard 变量镜像到 `/bt/bb/*` ROS topics，供 ROSA agent 查询。

---

## 依赖关系

- ROS Noetic (Python 3)
- `py_trees >= 2.1`
- `ainex_interfaces`（自定义消息：AppWalkingParam, ObjectsInfo, ColorsDetect, HeadState, FingerPosition）
- `ainex_kinematics`（GaitManager, MotionManager）
- `ainex_sdk`（common, pid）
- `ros_robot_controller`（BuzzerState, SetBusServosPosition）
- `ainex_example`（color_common, pid_track, approach_object, apriltag_common, face_common）

---

## 包结构

```
ainex_bt_edu/
├── CMakeLists.txt                    ← catkin，添加 msg 生成
├── package.xml                       ← 依赖以上各包
├── setup.py                          ← 安装 src/ainex_bt_edu
├── msg/
│   ├── BTNodeEvent.msg               ← 节点状态转换事件（→ /bt_node_events）
│   └── BTRunComplete.msg             ← Run 完成事件（→ /bt_run_complete）
├── src/ainex_bt_edu/
│   ├── __init__.py
│   ├── base_node.py                  ← 核心：AinexBTNode 基类
│   ├── blackboard_keys.py            ← BB 常量 + ROSA topic 映射表
│   ├── bb_ros_bridge.py              ← BB → ROS topic 镜像（ROSA 集成）
│   └── bt_runner.py                  ← 生命周期管理，发布 /bt_run_complete
│   └── behaviours/
│       ├── __init__.py
│       ├── L1_perception/                ← 11 个 Condition 节点
│       ├── __init__.py
│       ├── L1_perception/                ← 11 个 Condition 节点
│       │   ├── __init__.py
│       │   ├── L1_Balance_IsStanding.py
│       │   ├── L1_Balance_IsFallen.py
│       │   ├── L1_Vision_IsLineDetected.py
│       │   ├── L1_Vision_IsObjectDetected.py
│       │   ├── L1_Vision_IsFaceDetected.py
│       │   ├── L1_System_IsButtonPressed.py
│       │   ├── L1_Battery_IsVoltageOk.py
│       │   ├── L1_Motion_ListActions.py
│       │   ├── L1_Vision_IsPatternDetected.py
│       │   ├── L1_Vision_IsObjectStill.py
│       │   └── L1_Vision_IsTargetOnLeft.py
│       ├── L2_locomotion/                ← 14 个 Action 节点
│       │   ├── __init__.py
│       │   ├── L2_Gait_Enable.py
│       │   ├── L2_Gait_Disable.py
│       │   ├── L2_Gait_Start.py
│       │   ├── L2_Gait_Stop.py
│       │   ├── L2_Gait_SetParam.py
│       │   ├── L2_Gait_FollowLine.py
│       │   ├── L2_Balance_RecoverFromFall.py
│       │   ├── L2_Head_MoveTo.py
│       │   ├── L2_Head_TrackObject.py
│       │   ├── L2_Motion_PlayAction.py
│       │   ├── L2_Motion_PlayFromBB.py
│       │   ├── L2_Vision_SetColorTarget.py
│       │   ├── L2_Vision_UpdateROI.py
│       │   └── L2_Arm_MoveToPreset.py
│       └── L3_mission/                   ← 11 个复合任务节点
│           ├── __init__.py
│           ├── L3_Vision_TrackColorLine.py
│           ├── L3_Vision_TrackColorObject.py
│           ├── L3_Vision_TrackFace.py
│           ├── L3_Vision_StartColorDetectFromBB.py
│           ├── L3_Balance_SafeWalk.py
│           ├── L3_Gesture_RecognizeAndAct.py
│           ├── L3_Arm_PickObject.py
│           ├── L3_Arm_PlaceObject.py
│           ├── L3_Mission_TagNavigate.py
│           ├── L3_Mission_ExecuteSequence.py
│           └── L3_Gait_ApproachObject.py
```

---

## 自定义消息定义

### `msg/BTNodeEvent.msg`

```
Header header
string node_name      # 节点名，如 "L2_Gait_FollowLine"
string level          # "L1" | "L2" | "L3"
string prev_status    # "INVALID" | "RUNNING" | "SUCCESS" | "FAILURE"
string curr_status
int32  tick_count     # 该节点累计被 tick 次数
string session_id     # 本次 Run 的 UUID（由 bt_runner 注入）
string bb_snapshot    # JSON 字符串，BB_LOG_KEYS 对应值的快照
```

### `msg/BTRunComplete.msg`

```
Header header
string session_id
string status         # "SUCCESS" | "FAILURE"
float64 duration_sec  # 从 spin() 到终态的时长（秒）
int32  tick_count     # 本次 Run 总 tick 数
string tree_name      # 根节点名
```

---

## 核心类：`AinexBTNode`（`src/ainex_bt_edu/base_node.py`）

### 设计原则

重写 `tick()` 方法，在调用 `super().tick()`（内部透明调用子类 `update()`）前后比对状态变化，**子类业务逻辑零改动**。

```python
#!/usr/bin/env python3
"""
AinexBTNode — 所有 AiNex BT 节点的基类。

子类只需：
  - 声明 LEVEL = 'L1' | 'L2' | 'L3'
  - 声明 BB_LOG_KEYS = ['/domain/key', ...]
  - 在 setup() 中调用 super().setup(**kwargs)
  - 实现 update() → 返回 Status

日志和事件发布完全自动，无需在 update() 中添加任何代码。
"""
import json
import rospy
import py_trees
from py_trees.common import Status
from ainex_bt_edu.msg import BTNodeEvent


class AinexBTNode(py_trees.behaviour.Behaviour):
    LEVEL = 'L1'        # 子类声明：'L1' | 'L2' | 'L3'
    BB_LOG_KEYS = []    # 子类声明：需要快照的 BB 变量路径列表

    # 类级共享 Publisher（第一次 setup() 时初始化，所有实例共用）
    _event_pub = None

    def __init__(self, name: str):
        # Init before super().__init__ because py_trees calls self.tick()
        # during __init__ to create the iterator.
        self._tick_count = 0
        self._session_id = 'unknown'
        self._bb_log_client = None
        super().__init__(name)

    def setup(self, **kwargs):
        """初始化 ROS publishers。子类必须调用 super().setup(**kwargs)。"""
        if AinexBTNode._event_pub is None:
            AinexBTNode._event_pub = rospy.Publisher(
                '/bt_node_events', BTNodeEvent, queue_size=50)
        if self.BB_LOG_KEYS:
            self._bb_log_client = self.attach_blackboard_client(
                name=f'{self.name}_log')
            for key in self.BB_LOG_KEYS:
                self._bb_log_client.register_key(
                    key=key, access=py_trees.common.Access.READ)

    def tick(self):
        """重写 tick（generator）：注入状态变化日志，业务逻辑通过 super().tick() 透明调用。
        注意：py_trees tick() 是 generator，必须 yield，不能 return。"""
        prev = self.status
        self._tick_count += 1
        for node in super().tick():  # 调用子类 initialise/update/terminate
            yield node
        if self.status != prev:
            self._emit_event(prev, self.status)

    def _emit_event(self, prev: Status, curr: Status):
        """发布结构化日志到 /rosout 和 /bt_node_events。"""
        bb = self._snapshot_bb()
        log_fn = rospy.logwarn if curr == Status.FAILURE else rospy.loginfo
        log_fn('[BT][%s][%s] %s→%s tick=%d bb=%s',
               self.LEVEL, self.name,
               prev.name, curr.name, self._tick_count, bb)
        if AinexBTNode._event_pub:
            msg = BTNodeEvent()
            msg.header.stamp = rospy.Time.now()
            msg.node_name    = self.name
            msg.level        = self.LEVEL
            msg.prev_status  = prev.name
            msg.curr_status  = curr.name
            msg.tick_count   = self._tick_count
            msg.session_id   = self._session_id
            msg.bb_snapshot  = json.dumps(bb)
            AinexBTNode._event_pub.publish(msg)

    def _snapshot_bb(self) -> dict:
        """直接读取 Blackboard.storage（仅用于日志，绕过 ACL）。
        注意：py_trees 2.1.6 使用 .storage（公开），不是 ._storage（不存在）。
        BB key 保留前导 /（如 '/locomotion/robot_state'），不要 lstrip。"""
        storage = py_trees.blackboard.Blackboard.storage
        return {
            key: str(storage.get(key, '<unset>'))
            for key in self.BB_LOG_KEYS
        }
```

---

## Blackboard 命名规范

### 格式：`/<domain>/<variable_name>`

| 命名空间 | 变量 | 类型 | 写入方 | ROSA topic |
|---------|------|------|-------|-----------|
| `/perception/` | `detected_objects` | ObjectsInfo | 视觉节点 | `/bt/bb/perception/detected_objects` |
| `/perception/` | `line_data` | ObjectInfo | 视觉节点 | `/bt/bb/perception/line_data` |
| `/perception/` | `target_pixel_x` | float | 视觉节点 | `/bt/bb/perception/target_pixel_x` |
| `/perception/` | `target_pixel_y` | float | 视觉节点 | `/bt/bb/perception/target_pixel_y` |
| `/perception/` | `face_detected` | bool | L1_Vision_IsFaceDetected | `/bt/bb/perception/face_detected` |
| `/perception/` | `gesture_label` | string | L3_Vision_HandGestureCheck | `/bt/bb/perception/gesture_label` |
| `/perception/` | `target_world_pos` | geometry_msgs/Point | L3_Vision_AprilTagAlign | `/bt/bb/perception/target_world_pos` |
| `/locomotion/` | `robot_state` | string | L1_Balance_IsFallen | `/bt/bb/locomotion/robot_state` |
| `/locomotion/` | `gait_enabled` | bool | L2_Gait_Enable/Disable | `/bt/bb/locomotion/gait_enabled` |
| `/locomotion/` | `walk_x` | float | 上层任务或 ROSA | `/bt/bb/locomotion/walk_x` |
| `/locomotion/` | `walk_y` | float | 上层任务或 ROSA | `/bt/bb/locomotion/walk_y` |
| `/locomotion/` | `walk_angle` | float | 上层任务或 ROSA | `/bt/bb/locomotion/walk_angle` |
| `/manipulation/` | `gripper_state` | string (open/closed) | L3_Arm_Open/CloseGripper | `/bt/bb/manipulation/gripper_state` |
| `/manipulation/` | `approach_done` | bool | L3_Arm_ApproachObject | `/bt/bb/manipulation/approach_done` |
| `/mission/` | `current_task` | string | bt_runner | `/bt/bb/mission/current_task` |
| `/mission/` | `session_id` | string | bt_runner | `/bt/bb/mission/session_id` |
| `/mission/` | `target_color` | string | 上层配置 | `/bt/bb/mission/target_color` |
| `/mission/` | `action_name` | string | ROSA / 上层 | — |
| `/mission/` | `available_actions` | list[str] | L1_Motion_ListActions | — |
| `/perception/` | `color_lab_min` | list[int] | 上层配置 | — |
| `/perception/` | `color_lab_max` | list[int] | 上层配置 | — |
| `/perception/` | `color_detect_type` | string | 上层配置 | — |
| `/perception/` | `detect_roi` | list[int] | ROSA / 上层 | — |
| `/locomotion/` | `walk_body_height` | float | 上层任务或 ROSA | — |

### `robot_state` 合法值

| 值 | 含义 |
|---|------|
| `'stand'` | 机器人站立稳定 |
| `'walking'` | 步态运行中 |
| `'lie_to_stand'` | 正面摔倒，需向前起身 |
| `'recline_to_stand'` | 背面摔倒，需向后起身 |

### `blackboard_keys.py` 常量类

```python
class BB:
    # /perception/
    DETECTED_OBJECTS  = '/perception/detected_objects'
    LINE_DATA         = '/perception/line_data'
    TARGET_PIXEL_X    = '/perception/target_pixel_x'
    TARGET_PIXEL_Y    = '/perception/target_pixel_y'
    FACE_DETECTED     = '/perception/face_detected'
    GESTURE_LABEL     = '/perception/gesture_label'
    TARGET_WORLD_POS  = '/perception/target_world_pos'
    # /locomotion/
    ROBOT_STATE       = '/locomotion/robot_state'
    GAIT_ENABLED      = '/locomotion/gait_enabled'
    WALK_X            = '/locomotion/walk_x'
    WALK_Y            = '/locomotion/walk_y'
    WALK_ANGLE        = '/locomotion/walk_angle'
    WALK_BODY_HEIGHT  = '/locomotion/walk_body_height'
    # /perception/ (extended)
    COLOR_LAB_MIN     = '/perception/color_lab_min'
    COLOR_LAB_MAX     = '/perception/color_lab_max'
    COLOR_DETECT_TYPE = '/perception/color_detect_type'
    DETECT_ROI        = '/perception/detect_roi'
    # /manipulation/
    GRIPPER_STATE     = '/manipulation/gripper_state'
    APPROACH_DONE     = '/manipulation/approach_done'
    # /mission/
    CURRENT_TASK       = '/mission/current_task'
    SESSION_ID         = '/mission/session_id'
    TARGET_COLOR       = '/mission/target_color'
    ACTION_NAME        = '/mission/action_name'        # L2_Motion_PlayFromBB 读取
    AVAILABLE_ACTIONS  = '/mission/available_actions'  # L1_Motion_ListActions 写入

    # ROSA 镜像 topic 映射表（供 BlackboardROSBridge 使用）
    ROSA_TOPIC_MAP = {
        ROBOT_STATE:      '/bt/bb/locomotion/robot_state',
        GAIT_ENABLED:     '/bt/bb/locomotion/gait_enabled',
        WALK_X:           '/bt/bb/locomotion/walk_x',
        LINE_DATA:        '/bt/bb/perception/line_data',
        DETECTED_OBJECTS: '/bt/bb/perception/detected_objects',
        TARGET_PIXEL_X:   '/bt/bb/perception/target_pixel_x',
        TARGET_PIXEL_Y:   '/bt/bb/perception/target_pixel_y',
        FACE_DETECTED:    '/bt/bb/perception/face_detected',
        GESTURE_LABEL:    '/bt/bb/perception/gesture_label',
        GRIPPER_STATE:    '/bt/bb/manipulation/gripper_state',
        CURRENT_TASK:     '/bt/bb/mission/current_task',
    }
```

---

## 生命周期管理：`AinexBTRunner`（`src/ainex_bt_edu/bt_runner.py`）

```python
import uuid
import rospy
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.msg import BTRunComplete


class AinexBTRunner:
    """
    管理 BT 的完整生命周期：
    - 树初始化时注入 session_id
    - spin() 驱动 tick 循环
    - 根节点终态时发布 /bt_run_complete（latch=True）
    """

    def __init__(self, tree: py_trees.trees.BehaviourTree,
                 session_id: str = None):
        self.tree = tree
        self.session_id = session_id or str(uuid.uuid4())[:8]
        self._tick_count = 0
        self._start_time = None
        self._complete_pub = rospy.Publisher(
            '/bt_run_complete', BTRunComplete, queue_size=1, latch=True)
        # 将 session_id 注入所有 AinexBTNode 实例
        for node in tree.root.iterate():
            if isinstance(node, AinexBTNode):
                node._session_id = self.session_id
        tree.setup(timeout=15)

    def spin(self, rate_hz: float = 30):
        """驱动 BT tick 循环，直到根节点返回终态。"""
        self._start_time = rospy.Time.now()
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.tree.tick()
            self._tick_count += 1
            if self.tree.root.status in (Status.SUCCESS, Status.FAILURE):
                self._publish_complete()
                break
            rate.sleep()

    def _publish_complete(self):
        duration = (rospy.Time.now() - self._start_time).to_sec()
        msg = BTRunComplete()
        msg.header.stamp = rospy.Time.now()
        msg.session_id   = self.session_id
        msg.status       = self.tree.root.status.name
        msg.duration_sec = duration
        msg.tick_count   = self._tick_count
        msg.tree_name    = self.tree.root.name
        self._complete_pub.publish(msg)
        rospy.loginfo('[BT Runner] COMPLETE session=%s status=%s '
                      'duration=%.2fs ticks=%d',
                      self.session_id, msg.status, duration, self._tick_count)
```

---

## ROSA 集成：`BlackboardROSBridge`（`src/ainex_bt_edu/bb_ros_bridge.py`）

```python
import json
import rospy
import py_trees
from std_msgs.msg import String
from ainex_bt_edu.blackboard_keys import BB


class BlackboardROSBridge:
    """
    以固定频率将 Blackboard 关键变量镜像到 std_msgs/String topic（JSON 值）。
    ROSA agent 通过 ros_master_url 订阅 /bt/bb/* 查询机器人实时状态。

    用法：
        bridge = BlackboardROSBridge()   # 使用 BB.ROSA_TOPIC_MAP 默认映射
        bridge.start(rate_hz=10)
    """

    def __init__(self, key_to_topic_map: dict = None):
        self._map = key_to_topic_map or BB.ROSA_TOPIC_MAP
        self._pubs = {
            key: rospy.Publisher(topic, String, queue_size=1)
            for key, topic in self._map.items()
        }

    def start(self, rate_hz: float = 10):
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._publish_all)

    def _publish_all(self, _):
        storage = py_trees.blackboard.Blackboard.storage
        for key, pub in self._pubs.items():
            val = storage.get(key)
            if val is not None:
                serialized = (val if isinstance(val, (str, int, float, bool))
                              else str(val))
                pub.publish(String(data=json.dumps(serialized)))
```

ROSA 查询示例：
```bash
rostopic echo /bt/bb/locomotion/robot_state     # → '"stand"'
rostopic echo /bt/bb/perception/line_data       # → '"<ObjectInfo x=0.12 ...>"'
```

---

## 完整节点清单（共 36 个）

### 节点命名规则：`{Level}_{Module}_{Action}`

- **Level**：`L1`（感知/只读）/ `L2`（单步动作）/ `L3`（复合任务）
- **Module**：功能模块（Balance/Vision/Gait/Head/Motion/Arm/Mission 等）
- **Action**：动词+名词，对初学者语义透明

---

### L1 层 — 感知/条件节点（11 个，Condition，只读）

| 节点名 | 来源包 | 参考文件 | 核心逻辑 |
|-------|--------|---------|---------|
| `L1_Balance_IsStanding` | `ainex_behavior` | `marathon/behaviours/conditions.py` → `IsRobotStanding` | BB `/locomotion/robot_state == 'stand'`，直接迁移 |
| `L1_Balance_IsFallen` | `ainex_behavior` | `marathon/marathon_bt_node.py` → `_imu_callback` | 提取 IMU 角度阈值逻辑（angle<30 或 >150），写 BB `robot_state` |
| `L1_Vision_IsLineDetected` | `ainex_behavior` | `marathon/behaviours/conditions.py` → `IsLineDetected` | BB `/perception/line_data is not None`，直接迁移 |
| `L1_Vision_IsObjectDetected` | `ainex_example` | `scripts/color_detection/color_detection_node.py`；Topic T16 `/object/pixel_coords` | 订阅 `ObjectsInfo`，写 BB `detected_objects`；非空→SUCCESS |
| `L1_Vision_IsFaceDetected` | `ainex_example` | `scripts/face_track/face_track_node.py` + `src/ainex_example/face_common.py` | 订阅 face detect 结果，写 BB `face_detected`；True→SUCCESS |
| `L1_System_IsButtonPressed` | `ainex_peripherals` | sensor_node；Topic T07 `/sensor/button/get_button_state` | 订阅 Bool topic；True→SUCCESS |
| `L1_Battery_IsVoltageOk` | `ainex_driver/ros_robot_controller` | `ros_robot_controller_node.py`；Topic T03 `/ainex/battery` | 订阅 Float32；电压 ≥ 阈值→SUCCESS，否则 FAILURE |
| `L1_Motion_ListActions` | `ainex_kinematics` | `motion_manager.py`；扫描 `/home/ubuntu/software/ainex_controller/ActionGroups/*.d6a` | 初始化时枚举动作库，写 BB `/mission/available_actions`（list[str]）；供教学可视化和 ROSA 查询 |

**L1 读取/写入的 BB 键：** `/locomotion/robot_state`, `/perception/line_data`, `/perception/detected_objects`, `/perception/face_detected`, `/mission/available_actions`

---

### L2 层 — 动作节点（14 个，Action，直接驱动硬件/服务）

| 节点名 | 来源包 | 参考文件 | 核心逻辑 |
|-------|--------|---------|---------|
| `L2_Gait_Enable` | `ainex_kinematics` | `gait_manager.py`；Service S01 `/walking/command` | 发送 `'enable_control'`，返回 SUCCESS |
| `L2_Gait_Disable` | `ainex_behavior` | `marathon/behaviours/actions.py` → `StopWalking` | 调用 `gait_manager.disable()`，直接迁移 |
| `L2_Gait_Start` | `ainex_kinematics` | `gait_manager.py`；Service S02 | 发送 `'start'` |
| `L2_Gait_Stop` | `ainex_kinematics` | `gait_manager.py`；Service S03 | 发送 `'stop'` |
| `L2_Gait_SetParam` | `ainex_kinematics` | `gait_manager.py`；Topic T09 `/app/set_walking_param` | 发布 `AppWalkingParam`，x/y/angle/height 来自 BB `/locomotion/walk_*` |
| `L2_Gait_FollowLine` | `ainex_behavior` | `marathon/behaviours/actions.py` → `FollowLine` | 读 BB `line_data`，调用 `visual_patrol.process(x, width)` |
| `L2_Balance_RecoverFromFall` | `ainex_behavior` | `marathon/behaviours/actions.py` → `RecoverFromFall` | 蜂鸣→等待2s→disable→run lie/recline action→写 BB `robot_state='stand'` |
| `L2_Head_MoveTo` | `ainex_kinematics` | `motion_manager.py` → `set_servos_position([[23,pan],[24,tilt]])` | servo 23（pan）/24（tilt），目标值由构造注入 |
| `L2_Head_TrackObject` | `ainex_example` | `src/ainex_example/pid_track.py` → `PIDTrack` | 读 BB `target_pixel_x/y`，PID 输出驱动头部 servo |
| `L2_Motion_PlayAction` | `ainex_kinematics` | `motion_manager.py` → `run_action(name)` | 播放 `.d6a` 动作组，动作名由构造注入 |
| `L2_Motion_PlayFromBB` | `ainex_kinematics` | `motion_manager.py` → `run_action(name)`；读 BB `/mission/action_name` | 动作名运行时来自 BB，名称不在库中返回 FAILURE；适合 ROSA 动态下发 |
| `L2_Vision_UpdateROI` | `ainex_example` | `color_detection_node.py`；Topic T19 `/color_detection/update_detect`；`ColorsDetect.roi` 字段 | 从 BB `/perception/detect_roi` 读取新 ROI（x_min/y_min/x_max/y_max），合并到当前 `ColorsDetect` 后重发；ROSA 可实时调整感兴趣区域 |
| `L2_LED_SetState` | `ainex_peripherals` | sensor_node；Topic T08 `/sensor/led/set_led_state` | 发布 Bool，state 由构造注入 |
| `L2_Buzzer_Beep` | `ainex_driver/ros_robot_controller` | `ros_robot_controller_node.py`；Topic T12 | 发布 `BuzzerState(freq, on_time, off_time, repeat)`，参数由构造注入 |

**L2 读取/写入的 BB 键：** `/locomotion/robot_state`, `/perception/line_data`, `/locomotion/walk_x/y/angle`, `/perception/target_pixel_x/y`, `/mission/action_name`, `/perception/detect_roi`

---

### L3 层 — 复合任务节点（11 个，多模块集成）

| 节点名 | 来源包 | 参考文件 | 核心逻辑 |
|-------|--------|---------|---------|
| `L3_Vision_StartColorDetect` | `ainex_example` | `color_common.py` → `enter_func` + `start_srv_callback`；Topic T19 | 配置 `ColorsDetect` + 调用 enter/start 服务 |
| `L3_Vision_StopColorDetect` | `ainex_example` | `color_common.py` → `stop_srv_callback` + `exit_func` | 调用 stop/exit，清空 `/color_detection/update_detect` |
| `L3_Vision_TrackColorLine` | `ainex_behavior` | `marathon_bt_node.py` → `_set_color` | 封装完整颜色配置逻辑，颜色名来自 BB `/mission/target_color` |
| `L3_Vision_AprilTagAlign` | `ainex_example` | `apriltag_track_node.py` + `apriltag_common.py`；Topic T20 `/tag_detections` | 头部跟踪+走近对齐；写 BB `/perception/target_world_pos` |
| `L3_Vision_HandGestureCheck` | `ainex_example` | `hand_gesture_detect_node.py`；`ainex_interfaces/msg/FingerPosition` | 订阅手势检测，写 BB `/perception/gesture_label` |
| `L3_Arm_OpenGripper` | `ainex_driver/ros_robot_controller` | `ros_robot_controller_node.py`；servo ID 21(L)/22(R) | 发送 open_pulse，写 BB `gripper_state='open'` |
| `L3_Arm_CloseGripper` | `ainex_driver/ros_robot_controller` | 同上 | 发送 close_pulse，写 BB `gripper_state='closed'` |
| `L3_Arm_ApproachObject` | `ainex_example` | `src/ainex_example/approach_object.py` → `ApproachObject` | 封装走近闭环，读 BB `detected_objects`，写 BB `approach_done` |
| `L3_Mission_VisualPatrol` | `ainex_example` | `visual_patrol_node.py` + `visual_patrol_pick_up_node.py` | 组合子树：Enable→StartColorDetect→Start→FollowLine loop |
| `L3_Vision_StartColorDetectFromBB` | `ainex_example` | `color_detection_node.py`；`ColorDetection.detect()` → `use_name=False` 路径（`color_detection.py:483-485`）；Topic T19 | 从 BB 读 `color_lab_min/max`（自定义掩码）或 `target_color`（命名颜色），同时读 `color_detect_type` + `detect_roi`，动态构造 `ColorsDetect` 发布；涵盖 OpenCV `use_name=False` 路径 |

**L3 读取/写入的 BB 键：** `/perception/detected_objects`, `/perception/gesture_label`, `/perception/target_world_pos`, `/manipulation/approach_done`, `/manipulation/gripper_state`, `/mission/target_color`, `/perception/color_lab_min`, `/perception/color_lab_max`, `/perception/color_detect_type`, `/perception/detect_roi`

---

### 节点数量汇总

| 层级 | 描述 | Condition | Action | 合计 |
|------|------|-----------|--------|------|
| L1 | 感知/只读，初学者可见 | 11 | 0 | 11 |
| L2 | 单步动作，中级学员 | 0 | 14 | 14 |
| L3 | 复合任务，高级学员 | 0 | 11 | 11 |
| **总计** | | **11** | **25** | **36** |

---

## 节点迁移示范

### 原始 `FollowLine` → `L2_Gait_FollowLine`

```python
# ===== 原：ainex_behavior/marathon/behaviours/actions.py =====
class FollowLine(py_trees.behaviour.Behaviour):
    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/line_data", access=Access.READ)

    def update(self):
        line_data = self.bb.line_data
        self.visual_patrol.process(line_data.x, line_data.width)
        return py_trees.common.Status.SUCCESS


# ===== 新：ainex_bt_edu/behaviours/L2_locomotion/L2_Gait_FollowLine.py =====
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB
from py_trees.common import Access, Status


class L2_Gait_FollowLine(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.LINE_DATA, BB.ROBOT_STATE]   # 新增：日志快照键

    def __init__(self, visual_patrol):
        super().__init__('L2_Gait_FollowLine')
        self.visual_patrol = visual_patrol

    def setup(self, **kwargs):
        super().setup(**kwargs)                     # ← 必须，初始化日志基础设施
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(BB.LINE_DATA, Access.READ)

    def update(self):                               # ← 与原版完全相同
        line_data = self._bb.line_data
        self.visual_patrol.process(line_data.x, line_data.width)
        return Status.SUCCESS
```

状态转换时自动发布到 `/bt_node_events`：
```json
{
  "node_name": "L2_Gait_FollowLine",
  "level": "L2",
  "prev_status": "RUNNING",
  "curr_status": "SUCCESS",
  "tick_count": 42,
  "session_id": "a3f8b2c1",
  "bb_snapshot": {
    "/perception/line_data": "<ObjectInfo x=0.12 width=0.08 ...>",
    "/locomotion/robot_state": "stand"
  }
}
```

---

## Level 过滤机制

Level 是节点元数据属性，**运行时不做过滤**——所有节点事件均发布；消费端按 level 筛选。

```bash
# 只看 L1 层事件（面向初学者的仪表盘）
rostopic echo /bt_node_events | python3 -c \
  "import sys, json
  for line in sys.stdin:
      try:
          d = json.loads(line.strip())
          if d.get('level') == 'L1': print(json.dumps(d, indent=2))
      except: pass"

# 只看 FAILURE 事件（调试用）
rostopic echo /bt_node_events | grep 'curr_status.*FAILURE'
```

---

## `CMakeLists.txt` 关键配置

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(ainex_bt_edu)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs ainex_interfaces message_generation)

catkin_python_setup()

add_message_files(FILES
  BTNodeEvent.msg
  BTRunComplete.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs ainex_interfaces message_runtime)

catkin_install_python(PROGRAMS scripts/bt_edu_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

---

## 验证步骤

```bash
# 1. 构建（在容器内 ros_ws 目录）
cd /home/ubuntu/ros_ws
catkin build ainex_bt_edu
source devel/setup.bash

# 2. Dry-run 测试（需 roscore，不需硬件）
rosrun ainex_bt_edu bt_edu_node.py _mode:=dry_run

# 3. 启动 patrol BT（需 roscore + 硬件 bringup）
rosrun ainex_bt_edu bt_edu_node.py

# 4. 验证结构化日志
rostopic echo /bt_node_events

# 5. 验证生命周期发布
rostopic echo /bt_run_complete

# 6. 验证 ROSA 集成（BB 镜像）
rostopic echo /bt/bb/locomotion/robot_state
rostopic echo /bt/bb/perception/line_data

# 7. Level 过滤验证
rostopic hz /bt_node_events   # 应有事件流
```

---

## 实施阶段

| 阶段 | 内容 |
|------|------|
| Phase 1 | 框架层：`base_node.py`, `blackboard_keys.py`, `bt_runner.py`, `bb_ros_bridge.py`, msg, CMakeLists, package.xml, setup.py |
| Phase 2 | 迁移现有节点：`L1_Balance_*`, `L1_Vision_IsLineDetected`, `L2_Gait_Disable`, `L2_Gait_FollowLine`, `L2_Balance_RecoverFromFall` |
| Phase 3 | 补全 L1/L2 剩余节点 |
| Phase 4 | L3 复合任务节点 |
| Phase 5 | 来自 ainex_example/ainex_tutorial/hurocup2025 的补充节点（+4 节点，修改 1 节点） |

---

## py_trees 2.1.6 兼容性注意事项

实现时发现的关键兼容性问题，已在代码中修复：

| 问题 | 错误做法 | 正确做法 |
|------|---------|---------|
| Blackboard 存储访问 | `Blackboard._storage` | `Blackboard.storage`（公开属性，`_storage` 不存在） |
| `tick()` 方法签名 | `def tick(self) -> None: super().tick()` | `def tick(self): for node in super().tick(): yield node`（generator） |
| `__init__` 中属性初始化 | `super().__init__(name)` 后设置属性 | 属性先于 `super().__init__(name)` 设置（py_trees 在 `__init__` 中调用 `self.tick()`） |
| BB key 前导 `/` | `storage.get(key.lstrip('/'))` | `storage.get(key)`（storage 中 key 保留前导 `/`） |

---

## 节点清单（v1.2.1 — 2026-03-16）

共 **36 个节点**（L1:11 / L2:14 / L3:11），框架层文件全部就绪。
`L2_Gait_SetParam` 新增 `body_height` 参数支持（BB key: `/locomotion/walk_body_height`）。

### 框架层文件清单

| 文件 | 说明 |
|------|------|
| `msg/BTNodeEvent.msg` | 节点状态转换事件消息 |
| `msg/BTRunComplete.msg` | Run 完成事件消息 |
| `src/ainex_bt_edu/base_node.py` | `AinexBTNode` 基类 |
| `src/ainex_bt_edu/blackboard_keys.py` | BB 常量 + ROSA topic 映射表 |
| `src/ainex_bt_edu/bt_runner.py` | `AinexBTRunner` 生命周期管理器 |
| `src/ainex_bt_edu/bb_ros_bridge.py` | `BlackboardROSBridge` ROSA 集成桥 |
| `CMakeLists.txt` | catkin 构建配置 |
| `package.xml` | 包依赖声明 |
| `setup.py` | Python 包安装配置 |

### 节点实现清单

**L1 Condition 节点（11 个）**

| 节点名 | 文件 | 来源 |
|--------|------|------|
| L1_Balance_IsStanding | `behaviours/L1_perception/` | marathon/behaviours/conditions.py |
| L1_Balance_IsFallen | `behaviours/L1_perception/` | marathon/marathon_bt_node.py |
| L1_Vision_IsLineDetected | `behaviours/L1_perception/` | marathon/behaviours/conditions.py |
| L1_Vision_IsObjectDetected | `behaviours/L1_perception/` | ainex_example/color_detection_node.py |
| L1_Vision_IsFaceDetected | `behaviours/L1_perception/` | ainex_example/face_track_node.py |
| L1_System_IsButtonPressed | `behaviours/L1_perception/` | ainex_peripherals/sensor_node.py |
| L1_Battery_IsVoltageOk | `behaviours/L1_perception/` | ainex_driver/ros_robot_controller_node.py |
| L1_Motion_ListActions | `behaviours/L1_perception/` | ainex_kinematics/motion_manager.py |
| L1_Vision_IsPatternDetected | `behaviours/L1_perception/` | ainex_example/color_detection_node.py（ObjectInfo.type 过滤） |
| L1_Vision_IsObjectStill | `behaviours/L1_perception/` | ainex_example/kick_ball_node.py + hurocup2025/penalty_kick_node.py |
| L1_Vision_IsTargetOnLeft | `behaviours/L1_perception/` | ainex_example/kick_ball_node.py + hurocup2025/penalty_kick_node.py |

**L2 Action 节点（14 个）**

| 节点名 | 文件 | 来源 |
|--------|------|------|
| L2_Gait_Enable | `behaviours/L2_locomotion/` | ainex_kinematics/gait_manager.py |
| L2_Gait_Disable | `behaviours/L2_locomotion/` | marathon/behaviours/actions.py |
| L2_Gait_Start | `behaviours/L2_locomotion/` | ainex_kinematics/gait_manager.py |
| L2_Gait_Stop | `behaviours/L2_locomotion/` | ainex_kinematics/gait_manager.py |
| L2_Gait_SetParam | `behaviours/L2_locomotion/` | ainex_kinematics/gait_manager.py |
| L2_Gait_FollowLine | `behaviours/L2_locomotion/` | marathon/behaviours/actions.py |
| L2_Balance_RecoverFromFall | `behaviours/L2_locomotion/` | marathon/behaviours/actions.py |
| L2_Head_MoveTo | `behaviours/L2_locomotion/` | ainex_kinematics/motion_manager.py |
| L2_Head_TrackObject | `behaviours/L2_locomotion/` | ainex_example/pid_track.py |
| L2_Motion_PlayAction | `behaviours/L2_locomotion/` | ainex_kinematics/motion_manager.py |
| L2_Motion_PlayFromBB | `behaviours/L2_locomotion/` | ainex_kinematics/motion_manager.py（新增） |
| L2_Vision_SetColorTarget | `behaviours/L2_locomotion/` | ainex_example/color_detection_node.py |
| L2_Vision_UpdateROI | `behaviours/L2_locomotion/` | ainex_example/color_detection.py（新增） |
| L2_Arm_MoveToPreset | `behaviours/L2_locomotion/` | ainex_kinematics/motion_manager.py |

**L3 复合任务节点（11 个）**

| 节点名 | 文件 | 来源 |
|--------|------|------|
| L3_Vision_TrackColorLine | `behaviours/L3_mission/` | marathon/marathon_bt_node.py |
| L3_Vision_TrackColorObject | `behaviours/L3_mission/` | ainex_example/color_track_node.py |
| L3_Vision_TrackFace | `behaviours/L3_mission/` | ainex_example/face_track_node.py |
| L3_Vision_StartColorDetectFromBB | `behaviours/L3_mission/` | ainex_example/color_detection.py:477-485（新增） |
| L3_Balance_SafeWalk | `behaviours/L3_mission/` | marathon/marathon_bt_node.py |
| L3_Gesture_RecognizeAndAct | `behaviours/L3_mission/` | ainex_example/gesture_control_node.py |
| L3_Arm_PickObject | `behaviours/L3_mission/` | ainex_example/object_pick_node.py |
| L3_Arm_PlaceObject | `behaviours/L3_mission/` | ainex_kinematics/motion_manager.py |
| L3_Mission_TagNavigate | `behaviours/L3_mission/` | ainex_example/apriltag_common.py |
| L3_Mission_ExecuteSequence | `behaviours/L3_mission/` | 新增（ROSA 序列执行） |
| L3_Gait_ApproachObject | `behaviours/L3_mission/` | ainex_example/approach_object.py（步行走近 PID）|
