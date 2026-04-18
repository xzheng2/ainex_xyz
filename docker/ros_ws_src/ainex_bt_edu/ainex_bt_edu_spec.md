# 技术规格说明：ainex_bt_edu — AiNex 机器人行为树标准节点库

---

## 版本信息

| 字段 | 值 |
|------|---|
| 文档版本 | 2.3.1 |
| 日期 | 2026-04-18 |
| ROS 版本 | ROS Noetic (Ubuntu 20.04, aarch64) |
| 包路径（宿主机） | `docker/ros_ws_src/ainex_bt_edu/` |
| 包路径（容器内） | `/home/ubuntu/ros_ws/src/ainex_bt_edu/` |

---

## 定位与设计原则

`ainex_bt_edu` 是 AiNex 人形机器人的 **BT 标准节点库**，供上层项目包（如 `ainex_behavior/marathon`）直接导入。

### 核心设计规则

1. **Facade 隔离**：所有节点只持有 `AinexBTFacade`（抽象接口），不直接调用 ROS publisher/service/gait_manager。ROS I/O 全部由项目层 SemanticFacade → CommFacade 负责。
2. **Latched Blackboard**：节点读写 `/latched/` 命名空间下的 BB 键，由 `input_adapters/` 在每次 tick 前通过两阶段 latch 协议原子性快照，保证同一 tick 内所有节点读取一致的传感器输入。
3. **可观测性注入**：节点构造时接收可选的 `logger`（`DebugEventLogger`）和 `tick_id_getter`（`Callable[[], int]`），均有 None 安全默认值，不影响无观测器场景的使用。
4. **零 ROS 依赖初始化**：节点 `__init__` 不调用任何 ROS API；ROS 资源（BB clients）在 `setup()` 中初始化。
5. **Subscriber 唯一位置**：`rospy.Subscriber` 只允许出现在 `input_adapters/` 中，`app/`、`behaviours/`、`tree/`、`semantics/`、`algorithms/` 均禁止。
6. **算法层纯函数约束**：项目 `algorithms/` 只做纯计算（误差→步态参数、目标选择、状态判断），无 ROS 依赖，返回结构化 dict；由 semantic_facade 将结果放入 comm_facade 调用的 payload，由 `_emit()` 记录在 `ros_out` 中。

---

## 依赖关系

| 依赖 | 用途 |
|------|------|
| `py_trees >= 2.1.6` | BT 运行时 |
| `rospy` | ROS 日志 |
| `ainex_interfaces` | 自定义消息（`BTNodeEvent`, `BTRunComplete`） |
| `ainex_kinematics` | 仅供 `AinexBTRunner`（`bt_runner.py`）参考 |

节点本体**不**直接依赖 `ainex_kinematics`、`ainex_example`、`ainex_sdk`——这些依赖由 SemanticFacade 实现层承担。

---

## 包结构（当前实际）

```
ainex_bt_edu/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── msg/
│   ├── BTNodeEvent.msg           ← 节点状态转换事件
│   └── BTRunComplete.msg         ← Run 完成事件
├── scripts/
│   └── bt_edu_node.py            ← 演示节点（dry-run 用）
└── src/ainex_bt_edu/
    ├── __init__.py
    ├── base_facade.py            ← AinexBTFacade 抽象接口（项目 Facade 必须继承）
    ├── base_node.py              ← AinexBTNode 基类（所有节点继承）
    ├── blackboard_keys.py        ← BB key 唯一真实来源 + ROSA topic 映射表
    ├── bb_ros_bridge.py          ← BlackboardROSBridge（BB→ROS topic 镜像）
    ├── bt_runner.py              ← AinexBTRunner（生命周期管理）
    ├── behaviours/               ← 已上线标准节点（当前 13 个）
    │   ├── __init__.py
    │   ├── L1_perception/
    │   │   ├── __init__.py
    │   │   ├── L1_Balance_IsFallen.py        ← v2.3.1 新增
    │   │   ├── L1_Balance_IsStanding.py
    │   │   ├── L1_Head_IsHeadCentered.py
    │   │   ├── L1_Motion_ListActions.py      ← v2.3.1 新增
    │   │   ├── L1_Vision_IsLineDetected.py
    │   │   ├── L1_Vision_IsObjectStill.py    ← v2.3.1 新增
    │   │   └── L1_Vision_IsTargetOnLeft.py   ← v2.3.1 新增
    │   └── L2_locomotion/
    │       ├── __init__.py
    │       ├── L2_Balance_RecoverFromFall.py
    │       ├── L2_Gait_FindLine.py
    │       ├── L2_Gait_FollowLine.py
    │       ├── L2_Gait_Stop.py
    │       ├── L2_Head_FindLineSweep.py
    │       └── L2_Head_MoveTo.py             ← v2.3.1 新增
    ├── input_adapters/           ← 传感器输入适配层（rospy.Subscriber 唯一合法位置）
    │   ├── __init__.py
    │   ├── imu_balance_state_adapter.py   # /imu → /latched/robot_state
    │   └── line_detection_adapter.py      # /object/pixel_coords → /latched/*
    └── behaviours_tem/           ← 存档：待重构节点（不可 import，不参与构建）
        ├── L1_perception/        ← 5 个待迁移 L1 节点（Group B/C，需新 adapter 或 facade 方法）
        ├── L2_locomotion/        ← 9 个待迁移 L2 节点（Group C，需新 facade 方法）
        └── L3_mission/           ← 11 个旧 L3 节点
```

> **`input_adapters/` 是所有项目共用的传感器适配层。**
> `setup.py` 中已声明 `ainex_bt_edu.input_adapters` 包，直接 import 即可，无需复制到项目。

> **`behaviours_tem/` 是 legacy 存档，不属于 active API。**
> 这些节点（v1.2.1 实现）依赖旧硬件耦合接口（直接持有 gait_manager/motion_manager）。
> **禁止直接 import 或以这些文件为模板编写新节点。**
> 迁移进度（v2.3.1）：
> - 已删除：6 个（5 个 canonical 已存在 + L2_Gait_Disable 近重复于 L2_Gait_Stop）
> - 已升入 behaviours/：5 个（Group A — 无架构阻塞，见 v2.3.1）
> - 待迁移 Group B（需新 input adapter）：5 个 L1 节点
> - 待迁移 Group C（需新 facade 方法）：9 个 L2 节点

---

## `input_adapters/`（传感器输入适配层）

### 定位

`input_adapters/` 是整个 BT 框架中 **`rospy.Subscriber` 的唯一合法位置**。
所有传感器输入 callback 都在此层处理，`app/`、`behaviours/`、`tree/`、`semantics/`、`algorithms/` 均禁止出现 `rospy.Subscriber`。

### 两阶段 latch 协议

每个 tick 前，`app/<project>_bt_node.py` 的 `run()` 执行以下两个阶段：

```
Phase 1（同一 lock 临界区，原子快照所有 adapter）：
    with self.lock:
        imu_snap  = self._imu_adapter.snapshot_and_reset()
        line_snap = self._line_adapter.snapshot_and_reset()   # 按需

Phase 2（无锁，主线程独占，写 BB + emit 日志）：
    self._imu_adapter.write_snapshot(imu_snap,  self._tick_id)
    self._line_adapter.write_snapshot(line_snap, self._tick_id)
```

两阶段分离的目的：同一 tick 内所有节点看到一致的、冻结的传感器快照，消除跨 adapter 竞态。

`write_snapshot()` 负责：
1. 将快照值写入 py_trees blackboard `/latched/` 命名空间
2. `emit_comm("ros_in", ...)` — 仅在 `received_count > 0` 时（本 tick 有新消息）
3. `emit_comm("input_state", ...)` — 每 tick 必有一条，记录 `bb_writes`

### `ImuBalanceStateAdapter`

```
文件    input_adapters/imu_balance_state_adapter.py
订阅    /imu (sensor_msgs/Imu, 100 Hz)
写入 BB /latched/robot_state（ImuBalanceStateAdapter 和 FallRecoveryBTNode 共用）
逻辑    callback 更新内部 pending state（加速度判断 lie/recline/stand）
        snapshot_and_reset() 返回当前 pending state 并重置 received_count
        write_snapshot() 写 BB.ROBOT_STATE + emit ros_in / input_state
额外    force_state(state) — 供 L2_Balance_RecoverFromFall 的 robot_state_setter 钩子调用，
        在恢复动作完成后将状态强制设为 'stand'
```

### `LineDetectionAdapter`

```
文件    input_adapters/line_detection_adapter.py
订阅    /object/pixel_coords (自定义消息, ~30 Hz)
写入 BB /latched/line_data          — ObjectInfo | None
        /latched/last_line_x        — float | None（最后一次有效检测的 x）
        /latched/camera_lost_count  — int（连续未检测到线的 camera 帧数）
逻辑    callback 记录最新 pixel_coords + 递增 received_count + 更新 camera_lost_count
        snapshot_and_reset() 原子返回 (line_data, last_line_x, camera_lost_count) 并重置
        write_snapshot() 写三个 BB key + emit ros_in / input_state
注意    camera_lost_count 以 camera 帧（~30 Hz）计数，与 BT tick_id（~10 Hz）不同
```

### 在项目中集成 adapter

```python
# app/<project>_bt_node.py __init__() 中：
from ainex_bt_edu.input_adapters.imu_balance_state_adapter import ImuBalanceStateAdapter
from ainex_bt_edu.input_adapters.line_detection_adapter import LineDetectionAdapter

self._imu_adapter = ImuBalanceStateAdapter(
    lock=self.lock,
    logger=self._obs_logger,
    tick_id_getter=lambda: self._tick_id,
)
# 只引入项目实际需要的 adapter；不使用的 adapter 不实例化
```

> **项目按需选用**：仅用 IMU 的项目（如 `fall_recovery`）只实例化 `ImuBalanceStateAdapter`；
> 不需要某个 adapter 时不实例化，不影响其他 adapter 正常工作。

---

## `AinexBTFacade`（`base_facade.py`）

所有项目 SemanticFacade 必须继承此抽象基类。节点只持有 `AinexBTFacade`，不知道具体项目细节。

```python
class AinexBTFacade(ABC):
    # ── 行走 ──────────────────────────────────────────────────────────────
    @abstractmethod
    def stop_walking(self, bt_node: str, tick_id: int = None) -> None:
        """立即禁用步态控制器。"""

    @abstractmethod
    def follow_line(self, line_data, bt_node: str, tick_id: int = None) -> None:
        """计算并发送跟线步态指令。line_data 须有 .x 和 .width 字段。"""

    @abstractmethod
    def search_line(self, last_line_x, lost_count: int,
                    bt_node: str, tick_id: int = None) -> int:
        """原地旋转寻线。返回 gait_yaw（供调用方日志用）。"""

    # ── 恢复 ──────────────────────────────────────────────────────────────
    @abstractmethod
    def recover_from_fall(self, robot_state: str,
                          bt_node: str, tick_id: int = None) -> None:
        """完整摔倒恢复序列（蜂鸣→禁步→起身动作）。不写 BB，由节点负责。"""

    # ── 头部 ──────────────────────────────────────────────────────────────
    @abstractmethod
    def move_head(self, pan_pos: int, bt_node: str, tick_id: int = None) -> None:
        """命令头部水平舵机到指定位置（500 = 中心）。"""

    @abstractmethod
    def head_sweep_align(self, head_offset: int,
                         bt_node: str, tick_id: int = None) -> int:
        """按头部偏移量命令身体转向（用于 sweep-align 阶段）。返回 gait_yaw。"""
```

**参考实现**：`ainex_behavior/marathon/semantics/semantic_facade.py` → `MarathonSemanticFacade`

---

## `AinexBTNode`（`base_node.py`）

所有节点的基类，提供自动结构化日志。

```python
class AinexBTNode(py_trees.behaviour.Behaviour):
    LEVEL = 'L1'        # 子类声明：'L1' | 'L2' | 'L3'
    BB_LOG_KEYS = []    # 子类声明：需要快照的 BB 键（供 _emit_event 快照用）

    def __init__(self, name: str): ...

    def setup(self, **kwargs):
        """初始化类级 /bt_node_events publisher 和 BB_LOG_KEYS 客户端。
        子类必须调用 super().setup(**kwargs)。"""

    def tick(self):
        """generator：记录状态变化前后差异，自动发布到 /bt_node_events。
        业务逻辑通过 super().tick() 透明调用，子类无需修改 update()。"""
```

**py_trees 2.1.6 兼容要求**（已在代码中修复，开发时注意）：

| 问题 | 错误做法 | 正确做法 |
|------|---------|---------|
| BB 存储访问 | `Blackboard._storage` | `Blackboard.storage`（公开属性） |
| `tick()` 重写 | `def tick(self) -> None: super().tick()` | `def tick(self): for node in super().tick(): yield node` |
| `__init__` 属性初始化 | `super().__init__(name)` 后设置属性 | 属性先于 `super().__init__(name)` 设置 |
| BB key 前导 `/` | `storage.get(key.lstrip('/'))` | `storage.get(key)`（storage 中 key 保留前导 `/`） |

---

## Blackboard 键（当前实际使用）

> **唯一真实来源**：`ainex_bt_edu/blackboard_keys.py` — 所有 BB key 名称均在此定义，禁止在其他文件中硬编码字符串。

### 命名约定（`blackboard_keys.py`）

| 常量形式 | 用途 |
|---------|------|
| `BB.LATCHED_NS` | `/latched`，传给 `attach_blackboard_client(namespace=...)` |
| `BB.*_KEY` | 相对短 key（如 `'robot_state'`），用于 `register_key(key=BB.*_KEY)` |
| `BB.*`（绝对路径） | 如 `'/latched/robot_state'`，用于 `BB_LOG_KEYS`、`bb_writes` 字典 |
| `BB.HEAD_PAN_POS` | `'/head_pan_pos'`（根命名空间绝对路径），直接用于 `register_key(key=BB.HEAD_PAN_POS)` |
| `BB.ROSA_TOPIC_MAP` | 仅映射 4 个 active `/latched/` key → `/bt/bb/latched/*` topic；旧 `/locomotion/` + `/perception/` 映射已移除（v2.1.0） |

### `/latched/` 命名空间（传感器输入快照）

由 `ImuBalanceStateAdapter` / `LineDetectionAdapter` 在每次 tick 前通过两阶段 latch 协议写入，保证同一 tick 内所有节点读到一致数据。

| BB 键（绝对路径） | 常量 | 类型 | 写入方 | 读取节点 |
|-------|------|------|--------|---------|
| `/latched/robot_state` | `BB.ROBOT_STATE` | `str` | ImuBalanceStateAdapter | L1_Balance_IsStanding, L2_Balance_RecoverFromFall |
| `/latched/line_data` | `BB.LINE_DATA` | `ObjectInfo \| None` | LineDetectionAdapter | L1_Vision_IsLineDetected, L2_Gait_FollowLine, L2_Head_FindLineSweep |
| `/latched/last_line_x` | `BB.LAST_LINE_X` | `float \| None` | LineDetectionAdapter | L2_Gait_FindLine, L2_Head_FindLineSweep |
| `/latched/camera_lost_count` | `BB.CAMERA_LOST_COUNT` | `int` | LineDetectionAdapter | L2_Gait_FindLine |

对应短 key 常量：`BB.ROBOT_STATE_KEY`、`BB.LINE_DATA_KEY`、`BB.LAST_LINE_X_KEY`、`BB.CAMERA_LOST_COUNT_KEY`

### 绝对路径键（BT 内部状态）

| BB 键 | 常量 | 类型 | 写入方 | 读取方 |
|-------|------|------|--------|-------|
| `/head_pan_pos` | `BB.HEAD_PAN_POS` | `int` | L2_Head_FindLineSweep | L1_Head_IsHeadCentered |
| `/tick_id` | — | `int` | MarathonBTNode | BB bridge、ROSA |

### `robot_state` 合法值

| 值 | 含义 |
|---|------|
| `'stand'` | 站立稳定 |
| `'lie_to_stand'` | 正面摔倒，需向前起身 |
| `'recline_to_stand'` | 背面摔倒，需向后起身 |

### 废弃 Key（Legacy — 禁止在新代码中使用）

以下 key 在 v2.0.0 之前存在于旧节点或早期文档中，**v2.1.0 起不再是 active API**：

| 旧 key | 废弃原因 | 替代 |
|--------|---------|------|
| `/locomotion/robot_state` | 已迁移至 `/latched/` 命名空间 | `BB.ROBOT_STATE` = `/latched/robot_state` |
| `/perception/line_data` | 已迁移至 `/latched/` 命名空间 | `BB.LINE_DATA` = `/latched/line_data` |

> **规则**：新节点、新 input_adapter、新项目 behaviours 禁止读写上述废弃 key。
> 如在代码中发现这些 key，须按 v2.1.0 规范重构。

`/locomotion/` 命名空间下的其余 key（`GAIT_ENABLED`、`WALK_X` 等）保留在
`blackboard_keys.py` 供未来 L2/L3 节点扩展使用，但当前**没有任何 input_adapter 写入这些 key**，
也不在 `ROSA_TOPIC_MAP` 中。

---

## 节点清单（当前 behaviours/，共 13 个）

### L1 感知/条件节点（7 个）

#### `L1_Balance_IsFallen`

```
文件    behaviours/L1_perception/L1_Balance_IsFallen.py
签名    __init__(self, name='L1_Balance_IsFallen', logger=None, tick_id_getter=None)
读取 BB /latched/robot_state
逻辑    robot_state != 'stand' → SUCCESS（摔倒状态），'stand' → FAILURE
        fall 检测由 ImuBalanceStateAdapter 负责；本节点只读 BB 结果
```

#### `L1_Balance_IsStanding`

```
文件    behaviours/L1_perception/L1_Balance_IsStanding.py
签名    __init__(self, name='L1_Balance_IsStanding', logger=None, tick_id_getter=None)
读取 BB /latched/robot_state
逻辑    robot_state == 'stand' → SUCCESS，其余 → FAILURE
```

#### `L1_Motion_ListActions`

```
文件    behaviours/L1_perception/L1_Motion_ListActions.py
签名    __init__(self, action_path=None, name='L1_Motion_ListActions', logger=None, tick_id_getter=None)
写入 BB /mission/available_actions（list[str]）
逻辑    在 setup() 扫描 ACTION_PATH（默认 /home/ubuntu/software/ainex_controller/ActionGroups）
        将所有 .d6a 文件名（去后缀）排序后写入 BB，始终返回 SUCCESS
        纯文件系统操作；无 ROS 订阅
```

#### `L1_Head_IsHeadCentered`

```
文件    behaviours/L1_perception/L1_Head_IsHeadCentered.py
签名    __init__(self, name='L1_Head_IsHeadCentered', logger=None, tick_id_getter=None)
读取 BB /head_pan_pos（根命名空间，非 /latched/；由 L2_Head_FindLineSweep 写入）
逻辑    |head_pan_pos - 500| <= 30 → SUCCESS，否则 FAILURE
        用于 head-sweep 模式：阻塞 L2_Gait_FollowLine 直到身体对齐
        HEAD_PAN_CENTER=500，CENTER_THRESHOLD=30（须与 L2_Head_FindLineSweep 一致）
```

#### `L1_Vision_IsLineDetected`

```
文件    behaviours/L1_perception/L1_Vision_IsLineDetected.py
签名    __init__(self, name='L1_Vision_IsLineDetected', logger=None, tick_id_getter=None)
读取 BB /latched/line_data
逻辑    line_data is not None → SUCCESS，None → FAILURE
        logger 会记录 line_x、line_width（有线时）
```

#### `L1_Vision_IsObjectStill`

```
文件    behaviours/L1_perception/L1_Vision_IsObjectStill.py
签名    __init__(self, threshold=2.0, frames=5, name='L1_Vision_IsObjectStill',
                 logger=None, tick_id_getter=None)
读取 BB /perception/detected_objects（ObjectsInfo | None）
逻辑    追踪 objects.data[0] 的质心；当连续 `frames` 帧内位移 < `threshold` 像素 → SUCCESS
        样本不足或检测为空 → FAILURE；对象仍在移动 → FAILURE
        需要 ObjectDetectionAdapter（Group B）提供实时数据
```

#### `L1_Vision_IsTargetOnLeft`

```
文件    behaviours/L1_perception/L1_Vision_IsTargetOnLeft.py
签名    __init__(self, image_width=160, name='L1_Vision_IsTargetOnLeft',
                 logger=None, tick_id_getter=None)
读取 BB /perception/target_pixel_x（float | None）
逻辑    target_pixel_x < image_width / 2 → SUCCESS（目标在画面左侧）
        target_pixel_x >= 中心或无目标 → FAILURE
```

---

### L2 动作节点（6 个）

#### `L2_Gait_Stop`

```
文件    behaviours/L2_locomotion/L2_Gait_Stop.py
签名    __init__(self, name='L2_Gait_Stop', facade=None, tick_id_getter=None)
BB      无
逻辑    facade.stop_walking()，始终返回 SUCCESS
```

#### `L2_Gait_FollowLine`

```
文件    behaviours/L2_locomotion/L2_Gait_FollowLine.py
签名    __init__(self, name='L2_Gait_FollowLine', facade=None, tick_id_getter=None)
读取 BB /latched/line_data
逻辑    每 tick：facade.move_head(500) → 将头拉回中心
                facade.follow_line(line_data) → 发送跟线步态
        始终返回 SUCCESS（上游 IsLineDetected Sequence 控制准入）
注意    move_head 每 tick 调用：防止 head-sweep 模式后头部漂移
```

#### `L2_Balance_RecoverFromFall`

```
文件    behaviours/L2_locomotion/L2_Balance_RecoverFromFall.py
签名    __init__(self, name='L2_Balance_RecoverFromFall',
                 facade=None, robot_state_setter=None, tick_id_getter=None)
读取 BB /latched/robot_state（READ）
写入 BB /latched/robot_state = 'stand'（WRITE，完成后）
逻辑    facade.recover_from_fall(robot_state)（阻塞）
        写 BB 'stand'，调用 robot_state_setter('stand')（同步到 live store）
        始终返回 SUCCESS
```

#### `L2_Gait_FindLine`

```
文件    behaviours/L2_locomotion/L2_Gait_FindLine.py
签名    __init__(self, name='L2_Gait_FindLine', facade=None, tick_id_getter=None)
读取 BB /latched/last_line_x, /latched/camera_lost_count
逻辑    facade.search_line(last_line_x, camera_lost_count) → 原地旋转寻线
        始终返回 RUNNING（Selector 持续 tick 直到 IsLineDetected 成功）
```

#### `L2_Head_FindLineSweep`

```
文件    behaviours/L2_locomotion/L2_Head_FindLineSweep.py
签名    __init__(self, name='L2_Head_FindLineSweep', facade=None, tick_id_getter=None)
读取 BB /latched/line_data, /latched/last_line_x
写入 BB /head_pan_pos（绝对路径，供 IsHeadCentered 读取）

两阶段状态机：
  SWEEP  机器人静止，头部在 [SWEEP_RIGHT_POS=300, SWEEP_LEFT_POS=700] 往返扫描
         检测到 line_data → 切换到 ALIGN
  ALIGN  头部向中心靠拢（ALIGN_STEP=7 units/tick）
         同时 facade.head_sweep_align(head_offset) 命令身体对齐转向
         |head_pan - 500| ≤ CENTER_THRESHOLD(30) → 头部归中 → SUCCESS

关键常量（可在文件顶部调整）：
  HEAD_PAN_CENTER=500, SWEEP_LEFT_POS=700, SWEEP_RIGHT_POS=300
  SWEEP_STEP=10, SWEEP_PAUSE_TICKS=0, CENTER_THRESHOLD=30, ALIGN_STEP=7

memory=False Selector 适配（_fresh_start 标志）：
  initialise() 每 tick 被调用，但只在真正首次启动或 ALIGN SUCCESS 后才重置方向
  mid-sweep reinit 保留 _sweep_dir（防止每 tick 重置方向导致振荡）
  stop_walking() 只在 line_data is None 时调用（SWEEP 阶段静止扫描）
  line_data 存在时（ALIGN 阶段步态运行），initialise() 跳过 stop_walking()
```

#### `L2_Head_MoveTo`

```
文件    behaviours/L2_locomotion/L2_Head_MoveTo.py
签名    __init__(self, pan_pos=500, name='L2_Head_MoveTo', facade=None, tick_id_getter=None)
BB      无读写
逻辑    facade.move_head(pan_pos=self._pan_pos) — 将头部水平舵机移至指定位置
        500 = 中心；fire-and-forget，始终返回 SUCCESS
注意    仅控制 pan（舵机 23）；tilt（舵机 24）不由此节点控制
```

---

## 项目专属节点规范

当 `ainex_bt_edu.behaviours/` 中**没有通用等价节点**时，项目可在自己的 `behaviours/` 目录下新增节点。

### 文件分工

| 文件 | 节点类型 | 说明 |
|------|---------|------|
| `behaviours/conditions.py` | L1 感知条件节点 | 只读 BB，返回 SUCCESS/FAILURE；不调用 facade |
| `behaviours/actions.py` | L2+ 执行器动作节点 | 可返回 RUNNING；调用 `self._facade.*` |

### 项目专属节点强制规则

1. **必须继承 `AinexBTNode`** — 禁止直接继承 `py_trees.behaviour.Behaviour`
2. **禁止 rospy / manager 直接调用** — 所有 ROS 输出经 `facade → semantic_facade → comm_facade`
3. **`logger=None` 零成本 no-op** — 每次 `emit_bt()` 前先检查 `if self._logger:`
4. **debug log 归因链**：
   - 条件节点 / 动作节点：可 emit `"decision"` 和（动作节点）`"action_intent"`
   - `"ros_out"` **仅由 `comm/comm_facade.py` 的 `_emit()` 负责** — 节点层禁止直接 emit
   - 完整链：BT node `action_intent` → `semantic_facade.*` → `comm_facade.*` → `_emit(ros_out)`
5. **`algorithms/` 结果传递方式**：算法层返回结构化 dict；semantic_facade 将其放入 comm_facade 调用的 payload/summary；由 `_emit()` 记录在 `ros_out` 中 — 不在节点层直接记录算法中间结果

### marathon 向后兼容别名

`IsHeadCentered` 已在 v2.2.0 升级为标准节点 `L1_Head_IsHeadCentered`。
`marathon/behaviours/conditions.py` 保留同名别名：

```python
from ainex_bt_edu.behaviours.L1_perception.L1_Head_IsHeadCentered import (
    L1_Head_IsHeadCentered as IsHeadCentered,
)
```

---

## 框架组件

### `AinexBTRunner`（`bt_runner.py`）

管理 BT 完整生命周期：注入 session_id → spin() tick 循环 → 根节点终态时发布 `/bt_run_complete`（latch）。

```python
class AinexBTRunner:
    def __init__(self, tree, session_id=None): ...
    def spin(self, rate_hz=30): ...          # 驱动 tick；根节点终态时自动停止
```

### `BlackboardROSBridge`（`bb_ros_bridge.py`）

以 10 Hz 将 BB 键镜像到 `std_msgs/String` topic（JSON 格式），供 ROSA agent 订阅。

```python
bridge = BlackboardROSBridge()   # 使用 BB.ROSA_TOPIC_MAP 默认映射
bridge.start(rate_hz=10)
```

### 自定义消息

**`BTNodeEvent.msg`**
```
Header header
string node_name      # 节点名
string level          # "L1" | "L2" | "L3"
string prev_status    # "INVALID" | "RUNNING" | "SUCCESS" | "FAILURE"
string curr_status
int32  tick_count
string session_id
string bb_snapshot    # JSON 字符串（BB_LOG_KEYS 快照）
```

**`BTRunComplete.msg`**
```
Header header
string session_id
string status         # "SUCCESS" | "FAILURE"
float64 duration_sec
int32  tick_count
string tree_name
```

---

## 节点构造函数规范

所有 `behaviours/` 中的节点遵循统一构造函数约定：

```python
from ainex_bt_edu.blackboard_keys import BB

class LX_Module_Action(AinexBTNode):
    LEVEL = 'LX'
    BB_LOG_KEYS = [BB.ROBOT_STATE]   # 绝对路径常量 BB.*

    def __init__(self,
                 name: str = 'LX_Module_Action',
                 facade: AinexBTFacade = None,   # L2 节点必须；L1 可省略
                 logger=None,                    # L1 节点用于 emit_bt()；L2 可省略
                 tick_id_getter=None):            # 所有节点均支持
        super().__init__(name)
        self._facade = facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)

    def setup(self, **kwargs):
        super().setup(**kwargs)                  # 必须调用，初始化日志基础设施
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.ROBOT_STATE_KEY, access=Access.READ)  # 短 key 常量 BB.*_KEY

    def update(self) -> Status:
        ...
        return Status.SUCCESS | Status.FAILURE | Status.RUNNING
```

---

## 构建与验证

```bash
# 在容器内构建
docker exec ainex bash -c "cd /home/ubuntu/ros_ws && catkin build ainex_bt_edu ainex_behavior"

# 验证导入（需 source）
docker exec ainex bash -c "
source /home/ubuntu/ros_ws/devel/setup.bash
python3 -c '
import sys, rospkg
sys.path.insert(0, rospkg.RosPack().get_path(\"ainex_behavior\"))
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from ainex_bt_edu.behaviours.L1_perception.L1_Vision_IsLineDetected import L1_Vision_IsLineDetected
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_Stop import L2_Gait_Stop
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_FollowLine import L2_Gait_FollowLine
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_FindLine import L2_Gait_FindLine
from ainex_bt_edu.behaviours.L2_locomotion.L2_Head_FindLineSweep import L2_Head_FindLineSweep
from ainex_bt_edu.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall
print(\"OK\")
'
"
```

---

## 历史版本对比

| 版本 | 日期 | 变更概要 |
|------|------|---------|
| v1.2.1 | 2026-03-16 | 初始实现：36 节点（L1:11 / L2:14 / L3:11）；节点直接持有 gait_manager/motion_manager |
| v2.0.0 | 2026-04-15 | **Facade 重构**：引入 `AinexBTFacade` 抽象接口；节点与 ROS I/O 完全解耦；BB 改用 `/latched/` 命名空间；新增 `L2_Gait_FindLine` + `L2_Head_FindLineSweep`；36 节点中仅 7 个完成重构（其余存入 `behaviours_tem/` 待后续迁移） |
| v2.1.0 | 2026-04-17 | **BB key 标准化**：`blackboard_keys.py` 成为唯一 key source of truth；新增 `BB.*_KEY` 短 key 常量 + `BB.LATCHED_NS`；所有节点和 input_adapters 改用 `BB.*` 常量（禁止硬编码字符串）；`ROSA_TOPIC_MAP` 更新为 `/bt/bb/latched/*` |
| v2.2.0 | 2026-04-17 | **L1_Head_IsHeadCentered 升级**：从 marathon 专属节点提升为标准库节点；继承 `AinexBTNode`；支持 `logger` + `tick_id_getter` 可观测性；marathon `conditions.py` 保留同名别名；`bb_ros_bridge.py` 新增 `/head_pan_pos` 镜像 |
| **v2.3.1** | **2026-04-18** | **behaviours_tem Group A 迁移**：`behaviours_tem/` 中 6 个重复/近重复节点删除（5 个已有 canonical + `L2_Gait_Disable` 近重复于 `L2_Gait_Stop`）；5 个 Group A 节点重构后升入 `behaviours/`：`L1_Balance_IsFallen`（纯 BB 读取，移除 rospy.Subscriber）、`L1_Motion_ListActions`（文件系统扫描，改用 BB client）、`L1_Vision_IsObjectStill`（BB client + 状态稳定检测）、`L1_Vision_IsTargetOnLeft`（BB client + 位置判断）、`L2_Head_MoveTo`（facade.move_head()）；`behaviours_tem/` 剩余 14 个节点分 Group B（需新 adapter）/ Group C（需新 facade 方法） |
| **v2.3.0** | **2026-04-17** | **`input_adapters/` 层正式化**：`ImuBalanceStateAdapter` + `LineDetectionAdapter` 迁移至 `ainex_bt_edu/input_adapters/`（`setup.py` 声明包）；两阶段 latch 协议（`snapshot_and_reset` under lock / `write_snapshot` after lock）正式成为框架规范；`rospy.Subscriber` 唯一合法位置约束写入设计原则；项目专属节点分工规范化：条件节点 → `behaviours/conditions.py`，动作节点 → `behaviours/actions.py`；debug log 归因链约束（`ros_out` 仅由 `comm_facade._emit()` 负责）和 `algorithms/` 纯函数约束写入文档 |
