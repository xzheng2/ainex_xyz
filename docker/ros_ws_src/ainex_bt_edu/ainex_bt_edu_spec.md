# 技术规格说明：ainex_bt_edu — AiNex 机器人行为树标准节点库

---

## 版本信息

| 字段 | 值 |
|------|---|
| 文档版本 | 2.0.0 |
| 日期 | 2026-04-15 |
| ROS 版本 | ROS Noetic (Ubuntu 20.04, aarch64) |
| 包路径（宿主机） | `docker/ros_ws_src/ainex_bt_edu/` |
| 包路径（容器内） | `/home/ubuntu/ros_ws/src/ainex_bt_edu/` |

---

## 定位与设计原则

`ainex_bt_edu` 是 AiNex 人形机器人的 **BT 标准节点库**，供上层项目包（如 `ainex_behavior/marathon`）直接导入。

### 核心设计规则

1. **Facade 隔离**：所有节点只持有 `AinexBTFacade`（抽象接口），不直接调用 ROS publisher/service/gait_manager。ROS I/O 全部由项目层 SemanticFacade → CommFacade 负责。
2. **Latched Blackboard**：节点读写 `/latched/` 命名空间下的 BB 键，该命名空间由上层节点（MarathonBTNode）在每次 tick 前原子性快照一次，保证同一 tick 内所有节点读取一致的传感器输入。
3. **可观测性注入**：节点构造时接收可选的 `logger`（`DebugEventLogger`）和 `tick_id_getter`（`Callable[[], int]`），均有 None 安全默认值，不影响无观测器场景的使用。
4. **零 ROS 依赖初始化**：节点 `__init__` 不调用任何 ROS API；ROS 资源（BB clients）在 `setup()` 中初始化。

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
    ├── blackboard_keys.py        ← BB 常量 + ROSA topic 映射表
    ├── bb_ros_bridge.py          ← BlackboardROSBridge（BB→ROS topic 镜像）
    ├── bt_runner.py              ← AinexBTRunner（生命周期管理）
    ├── behaviours/               ← 已上线标准节点（当前 7 个）
    │   ├── __init__.py
    │   ├── L1_perception/
    │   │   ├── __init__.py
    │   │   ├── L1_Balance_IsStanding.py
    │   │   └── L1_Vision_IsLineDetected.py
    │   └── L2_locomotion/
    │       ├── __init__.py
    │       ├── L2_Balance_RecoverFromFall.py
    │       ├── L2_Gait_FindLine.py
    │       ├── L2_Gait_FollowLine.py
    │       ├── L2_Gait_Stop.py
    │       └── L2_Head_FindLineSweep.py
    └── behaviours_tem/           ← 存档：待重构节点（不可 import，不参与构建）
        ├── L1_perception/        ← 9 个旧 L1 节点（原 v1.2.1 版本）
        ├── L2_locomotion/        ← 11 个旧 L2 节点
        └── L3_mission/           ← 11 个旧 L3 节点
```

> **`behaviours_tem/`**：原 v1.2.1 中已实现但尚未按新 Facade 架构重构的节点存档。
> 这些文件**不应直接 import**——它们依赖旧的直接硬件耦合接口。
> 未来按需从此处取用后，按新规范重构后移入 `behaviours/`。

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

### `/latched/` 命名空间（传感器输入快照）

由 `MarathonBTNode._latch_inputs()` 在每次 tick 前写入，保证同一 tick 内所有节点读到一致数据。

| BB 键 | 类型 | 写入方 | 读取节点 |
|-------|------|--------|---------|
| `/latched/robot_state` | `str` | MarathonBTNode | L1_Balance_IsStanding, L2_Balance_RecoverFromFall |
| `/latched/line_data` | `ObjectInfo \| None` | MarathonBTNode | L1_Vision_IsLineDetected, L2_Gait_FollowLine, L2_Head_FindLineSweep |
| `/latched/last_line_x` | `float \| None` | MarathonBTNode | L2_Gait_FindLine, L2_Head_FindLineSweep |
| `/latched/camera_lost_count` | `int` | MarathonBTNode | L2_Gait_FindLine |

### 绝对路径键（BT 内部状态）

| BB 键 | 类型 | 写入方 | 读取方 |
|-------|------|--------|-------|
| `/head_pan_pos` | `int` | L2_Head_FindLineSweep | marathon/behaviours/conditions.py → IsHeadCentered |
| `/tick_id` | `int` | MarathonBTNode | BB bridge、ROSA |

### `robot_state` 合法值

| 值 | 含义 |
|---|------|
| `'stand'` | 站立稳定 |
| `'lie_to_stand'` | 正面摔倒，需向前起身 |
| `'recline_to_stand'` | 背面摔倒，需向后起身 |

---

## 节点清单（当前 behaviours/，共 7 个）

### L1 感知/条件节点（2 个）

#### `L1_Balance_IsStanding`

```
文件    behaviours/L1_perception/L1_Balance_IsStanding.py
签名    __init__(self, name='L1_Balance_IsStanding', logger=None, tick_id_getter=None)
读取 BB /latched/robot_state
逻辑    robot_state == 'stand' → SUCCESS，其余 → FAILURE
```

#### `L1_Vision_IsLineDetected`

```
文件    behaviours/L1_perception/L1_Vision_IsLineDetected.py
签名    __init__(self, name='L1_Vision_IsLineDetected', logger=None, tick_id_getter=None)
读取 BB /latched/line_data
逻辑    line_data is not None → SUCCESS，None → FAILURE
        logger 会记录 line_x、line_width（有线时）
```

---

### L2 动作节点（5 个）

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

---

## marathon 专属节点（非本库）

以下节点位于 `ainex_behavior/marathon/behaviours/conditions.py`，不属于 `ainex_bt_edu`：

| 节点 | 位置 | 说明 |
|------|------|------|
| `IsHeadCentered` | `marathon/behaviours/conditions.py` | 读 BB `/head_pan_pos`；\|pan-500\| ≤ 30 → SUCCESS；仅 head-sweep 模式使用 |

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
class LX_Module_Action(AinexBTNode):
    LEVEL = 'LX'
    BB_LOG_KEYS = ['/latched/...']   # 状态转换时快照的 BB 键

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
            name=self.name, namespace='/latched')
        self._bb.register_key(key='...', access=Access.READ)

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
| **v2.0.0** | **2026-04-15** | **Facade 重构**：引入 `AinexBTFacade` 抽象接口；节点与 ROS I/O 完全解耦；BB 改用 `/latched/` 命名空间；新增 `L2_Gait_FindLine` + `L2_Head_FindLineSweep`；36 节点中仅 7 个完成重构（其余存入 `behaviours_tem/` 待后续迁移） |
