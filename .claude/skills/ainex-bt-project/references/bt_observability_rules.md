# BT 可观测性系统框架

> 适用范围：所有基于 `py_trees` + ROS 的 BT 项目
> 参考实现：`marathon/`、`fall_recovery/`（`ainex_behavior` 包）
> 共享模块：`ainex_behavior/bt_observability/`
> 节点库：`ainex_bt_edu/behaviours/`（所有标准 BT 节点的唯一来源）
> 最后更新：2026-04-17

---

## 1. 设计目标

为每个 BT 项目提供统一、可插拔的可观测性能力：

- **BT 决策层日志**：每个 tick 内节点执行状态、条件判断输入/输出、BB 写入
- **ROS 通信层日志**：业务出站（`ros_out`）+ 传感器输入适配（`ros_in`）
- **双层存储**：ROS topic（实时）+ JSONL 文件（离线回放）
- **ROSA/LLM 可查询**：结构化 JSON，工具可直接消费
- **零成本回退**：`logger=None` 时所有调用为 no-op，不影响无日志环境

---

## 2. 共享模块

`bt_observability/` 是包级共享模块，**不属于任何单个 BT 项目**，所有项目共用：

```
ainex_behavior/
└── bt_observability/
    ├── __init__.py
    ├── debug_event_logger.py   # 统一输出层（ROS topic + JSONL）
    ├── bt_debug_visitor.py     # BT 决策层 Visitor
    └── ros_comm_tracer.py      # ⚠️ LEGACY / optional — 新项目不引入
```

`ainex_bt_edu/input_adapters/` 是传感器输入适配层，**共享，不属于任何单个项目**：

```
ainex_bt_edu/src/ainex_bt_edu/
└── input_adapters/
    ├── __init__.py
    ├── base_adapter.py               # AinexInputAdapter（business_in 基类）
    ├── imu_balance_state_adapter.py    # /imu → /latched/robot_state
    └── line_detection_adapter.py       # /object/pixel_coords → /latched/*
```

> **`rospy.Subscriber` 唯一合法位置**：`ainex_bt_edu/input_adapters/`。
> `app/`、`behaviours/`、`tree/`、`semantics/`、`algorithms/` 均禁止出现 `rospy.Subscriber`。

新 BT 项目**直接 import，不复制**：

```python
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor
```

> **`ros_comm_tracer.py` 标注说明**：
> `ROSCommTracer` 和 `ManagerProxy` 是 legacy 工具类，**新项目默认不使用，不得引入业务主链**。
> `BTDebugVisitor` 内部使用 `ROSCommTracer._msg_to_dict` 做 BB value 序列化（这是共享模块内部实现细节，不属于公开 API）。
> 新项目业务出站通信唯一出口是 `comm/comm_facade.py`，禁止通过 `ManagerProxy` 或 `ROSCommTracer` 替代。

---

## 3. 新项目目录结构模板

```
ainex_bt_edu/                        # 标准节点库 + 输入适配层（所有项目共用，只读 import，不修改）
├── base_node.py                     # AinexBTNode + AinexL1ConditionNode + AinexL2ActionNode
├── behaviours/
│   ├── L1_perception/               # 通用感知条件节点（继承 AinexL1ConditionNode）
│   └── L2_locomotion/               # 通用运动动作节点（继承 AinexL2ActionNode）
└── input_adapters/                  # 传感器输入适配（rospy.Subscriber 唯一合法位置）
    ├── base_adapter.py              # AinexInputAdapter；不是 BT node
    ├── imu_balance_state_adapter.py # /imu → /latched/robot_state
    └── line_detection_adapter.py    # /object/pixel_coords → /latched/*

ainex_behavior/
└── <project_name>/
    ├── __init__.py
    ├── app/
    │   ├── <project>_bt_node.py    # node 入口；创建 logger；装配 adapters/facade/infra
    │   └── ros_msg_utils.py        # msg_to_dict（来自模板，无 rospy 依赖）
    ├── tree/
    │   └── <project>_bt.py         # BT 树结构定义；只做节点组装
    │                                #   - 优先 import ainex_bt_edu 标准节点
    │                                #   - 项目专属节点从本项目 behaviours/ import
    ├── behaviours/
    │   ├── conditions.py           # ⚠️ 项目专属 L1 条件节点（perception 判断）
    │   │                            #   - 仅放 ainex_bt_edu 里无对应的节点
    │   │                            #   - 所有节点必须继承 AinexL1ConditionNode
    │   │                            #   - 禁止重新实现 ainex_bt_edu 已有节点
    │   │                            #   - 可通过 self.emit_decision() 主动上报 decision
    │   └── actions.py              # ⚠️ 项目专属 L2+ 动作节点（执行器指令）
    │                                #   - 继承 AinexL2ActionNode；接受 facade 参数
    │                                #   - 只调用 facade.*；禁止 rospy/manager 直接调用
    │                                #   - 可通过 helper emit decision + action_intent
    │                                #   - ros_out 由 comm_facade 负责；节点禁止直接 emit ros_out
    ├── semantics/
    │   └── semantic_facade.py      # 业务语义层；继承 AinexBTFacade；只调用 comm_facade
    ├── comm/
    │   └── comm_facade.py          # 唯一 ros_out / ros_result 日志出口
    ├── algorithms/                 # 纯算法层（无 ROS 依赖）
    │                                #   - 纯计算：误差→步态参数、目标选择、状态判断等
    │                                #   - 禁止 rospy / gait_manager / emit_comm
    │                                #   - 只由 semantic_facade 调用
    │                                #   - 返回结构化结果 dict；semantic_facade 放入
    │                                #     comm_facade payload/summary 由 _emit() 记录
    ├── infra/
    │   ├── __init__.py
    │   ├── infra_manifest.py       # 基础设施通信清单（见 bt_infra_manifest_rules.md）
    │   ├── tree_publisher.py       # py_trees_msgs 发布（来自模板）
    │   ├── bb_ros_bridge.py        # BB → ROS topic 镜像（来自模板，按项目定制）
    │   └── bt_exec_controller.py   # RUN/PAUSE/STEP 控制（来自模板）
    │                                #   service names: ~bt/run, ~bt/pause, ~bt/step
    │                                #   resolved: /<project>_bt/bt/run 等
    ├── <project>_bt_node.launch    # BT-only launch
    ├── <project>_bringup.launch    # 完整硬件 bringup（来自模板，按项目定制）
    ├── check_imports.py            # AST 分层 import 约束检查（可选但推荐）
    └── log/                        # 运行时生成，git 按需跟踪
        ├── bt_debug_lastrun.jsonl
        ├── bt_debug_recent.jsonl
        ├── bt_ros_comm_debug_lastrun.jsonl
        ├── bt_ros_comm_debug_recent.jsonl
        └── infra_comm_manifest_lastrun.json
```

---

## 4. 架构分层与日志责任

```
ainex_bt_edu/input_adapters/     ← rospy.Subscriber 唯一合法位置；adapter 继承 AinexInputAdapter
    ImuBalanceStateAdapter            每 tick：snapshot_and_reset()（under lock）
    LineDetectionAdapter                       + write_snapshot()（after lock）
    │  emits: ros_in（received_count>0 时）+ input_state（每 tick 必有）
    ▼
app/<project>_bt_node.py         ← 装配层；禁止 rospy.Subscriber 和 emit_comm
    │  创建 adapters，装配 tree/facade/infra；调用 adapter.snapshot_and_reset/write_snapshot
    ▼
ainex_bt_edu/behaviours/         ← 标准节点库（L1/L2；继承 L1/L2 基类；实现 AinexBTFacade 接口）
    ▼ import（tree/ 直接引用）
tree/<project>_bt.py             ← 只做组装：import ainex_bt_edu 节点 + 项目专属节点
    ▼
behaviours/conditions.py         ← 项目专属 L1 条件节点（仅放无通用等价物的节点）
    │  继承 AinexL1ConditionNode；不调用 facade
    │  可选 self.emit_decision(...)
behaviours/actions.py            ← 项目专属 L2+ 动作节点（仅放无通用等价物的节点）
    │  继承 AinexL2ActionNode；接受 AinexBTFacade 类型的 facade
    │  只调用 semantic_facade.*
    │  可选 self.emit_decision(...) 和 self.emit_action_intent(...)
    │  ⚠️  禁止直接 emit ros_out — ros_out 由 comm_facade 负责
    ▼
semantics/semantic_facade.py     ← 实现 AinexBTFacade；只调用 comm_facade.*
    ▼
comm/comm_facade.py              ← 唯一 business_out（ros_out / ros_result）emit_comm 出口
    ▼
runtime（GaitManager / Publisher / ServiceProxy）
```

`bt_observability/` 横切所有层，由 `app` 层初始化后注入，**不在各层直接 import**。
`ainex_bt_edu.base_node.AinexBTNode` 只提供节点侧 helper；节点级 `tick_end`
由 `BTDebugVisitor` 自动记录。L1/L2 节点必须分别继承
`AinexL1ConditionNode` / `AinexL2ActionNode`，二者间接继承 `AinexBTNode`。
Input adapter 继承 `AinexInputAdapter`，不继承任何 BT node 基类。

### 通信边界三类分类

| 边界类型 | 定义 | 记录位置 | 日志事件 |
|---|---|---|---|
| **business_out** | BT leaf 触发，经 behaviours → semantics → comm → runtime 发出的出站通信 | `comm/comm_facade.py` | `ros_out` / `ros_result` |
| **business_in** | 传感器或外部输入 callback 收到的消息事件（输入适配） | `ainex_bt_edu/input_adapters/` | `ros_in` + `input_state` |
| **infra** | 服务 node 框架本身的非业务通信（树可视化、BB 镜像、exec 控制、生命周期等） | `infra_comm_manifest_lastrun.json` | 不进入 `bt_ros_comm_debug` |

**传感器订阅的双重性**（重要）：
- 传感器 subscriber **接口注册**（topic 名、消息类型是否存在）→ 属于 **infra**，写入 `infra_comm_manifest_lastrun.json`
- 传感器 callback **收到消息**的每次采样事件（每 tick 记一次）→ 属于 **business_in**，在 `input_adapters/` 中记录为 `ros_in` + `input_state`，写入 `bt_ros_comm_debug_*.jsonl`

### 日志责任归属原则

| 事件类型 | 边界类型 | 谁负责 emit | 说明 |
|---|---|---|---|
| `ros_in` | business_in | `input_adapters/` 的 `write_snapshot()` | received_count>0 时每 tick 记一次；received_count 是自上次 latch 后收到的消息数 |
| `input_state` | business_in | `input_adapters/` 的 `write_snapshot()` | 每 tick 必有一条；bb_writes 记录本 tick 实际写入 py_trees BB 的值 |
| `ros_out` | business_out | `comm/comm_facade.py` 各公开方法 | 在 runtime 调用**前**写，表示通信意图 |
| `ros_result` | business_out | `comm/comm_facade.py` 各公开方法 | 有返回值或异常时另写，与 ros_out 配对 |
| `decision` | — | `behaviours/conditions.py` 或 `actions.py`（可选） | condition/action 节点通过 `self.emit_decision()` 主动上报判断输入/结果 |
| `action_intent` | — | `behaviours/actions.py`（可选） | action 节点 `initialise()` 通过 `self.emit_action_intent()` 上报执行意图；ros_out 归属由 comm_facade 负责 |
| `tree_tick_start/end` | — | `BTDebugVisitor`（自动） | 挂载后自动记录，无需手动 |
| `tick_end`（节点级） | — | `BTDebugVisitor`（自动） | 同上 |
| `bb_write` | — | `BTDebugVisitor`（自动） | flush 时自动记录 BB 写入去重结果 |

---

## 5. 集成步骤

### Step 1：初始化 `DebugEventLogger`

在 `app/<project>_bt_node.py` 的 `__init__()` 中，其他对象创建之前：

```python
import os
from bt_observability.debug_event_logger import DebugEventLogger

_LOG_DIR = os.path.join(_PACKAGE_DIR, '<project_name>', 'log')
os.makedirs(_LOG_DIR, exist_ok=True)

self._tick_id = 0
self._obs_logger = DebugEventLogger(
    bt_topic="/<project>_bt_debug",                      # 项目专属 topic 名，避免冲突
    comm_topic="/<project>_bt_ros_comm_debug",
    bt_lastrun_jsonl=os.path.join(_LOG_DIR, 'bt_debug_lastrun.jsonl'),
    comm_lastrun_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_lastrun.jsonl'),
    rolling_bt_jsonl=os.path.join(_LOG_DIR, 'bt_debug_recent.jsonl'),
    rolling_comm_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_recent.jsonl'),
    max_rolling_ticks=rospy.get_param('~max_rolling_ticks', 30),
    tick_id_getter=lambda: self._tick_id,
)
```

### Step 2：挂载 `BTDebugVisitor`

在 BT 树构建完成后立刻挂载：

```python
from bt_observability.bt_debug_visitor import BTDebugVisitor

self._bt_visitor = BTDebugVisitor(self._obs_logger, lambda: self._tick_id)
self.tree.visitors.append(self._bt_visitor)
self.tree.pre_tick_handlers.append(self._bt_visitor.on_tree_tick_start)
self.tree.post_tick_handlers.append(self._bt_visitor.on_tree_tick_end)
```

### Step 3：在 run() 中递增 tick_id 并调用 adapter 两阶段协议

`tick_id` 必须在 `tree.tick()` 之前递增，且只在 `should_tick()` 通过时递增：

```python
def run(self):
    rate = rospy.Rate(10)
    while self.running and not rospy.is_shutdown():
        if self.start and self._exec_ctrl.should_tick():
            self._tick_id += 1
            self._bb_meta.tick_id = self._tick_id   # 写入 BB（可选，供 BB bridge 使用）

            # Phase 1: 在同一 lock 临界区内原子 snapshot 所有 adapters
            # 保证 BT 树看到一致的、已冻结的传感器输入快照（无跨 adapter 竞态）
            with self.lock:
                imu_snap  = self._imu_adapter.snapshot_and_reset()
                line_snap = self._line_adapter.snapshot_and_reset()
            # Phase 2: 写 BB + emit ros_in / input_state（无锁，主线程独占）
            self._imu_adapter.write_snapshot(imu_snap,  self._tick_id)
            self._line_adapter.write_snapshot(line_snap, self._tick_id)

            self.tree.tick()
        rate.sleep()
```

### Step 4：使用 input_adapters 两阶段协议（`ros_in` + `input_state` 由 adapter 负责）

`rospy.Subscriber` 和 `ros_in` / `input_state` emit 均由 `ainex_bt_edu/input_adapters/` 负责。
`app/<project>_bt_node.py` **不得**直接创建 `rospy.Subscriber` 或调用 `emit_comm`。
所有 input adapter 必须继承 `AinexInputAdapter`。它们是 business-in 组件，不是
BT node，不继承 `AinexBTNode`，不返回 `Status`。

在 `__init__()` 中创建 adapters（在 `self._obs_logger` 之后、`self._comm_facade` 之前）：

```python
from ainex_bt_edu.input_adapters.imu_balance_state_adapter import ImuBalanceStateAdapter
from ainex_bt_edu.input_adapters.line_detection_adapter import LineDetectionAdapter

# ── Input adapters ────────────────────────────────────────────────────
# rospy.Subscriber 位于 ainex_bt_edu/input_adapters/ 内，不在 app/ 层
# 两阶段协议：snapshot_and_reset()（under lock）+ write_snapshot()（after lock）
self._imu_adapter = ImuBalanceStateAdapter(
    lock=self.lock,
    logger=self._obs_logger,
    tick_id_getter=lambda: self._tick_id,
)
self._line_adapter = LineDetectionAdapter(
    lock=self.lock,
    logger=self._obs_logger,
    tick_id_getter=lambda: self._tick_id,
)
```

`bootstrap()` 的 `robot_state_setter` 参数传 `self._imu_adapter.force_state`（供 `L2_Balance_RecoverFromFall` 调用）。

### Step 5：在 `comm_facade.py` 中添加 `ros_out` 日志

`comm_facade.py` 每个公开业务方法的标准写法：
1. 裁剪 payload
2. `_emit(ros_out)` — 表示通信意图，**在 runtime 调用前写**
3. 执行 runtime 调用
4. 如有返回值或异常，另写 `ros_result`

> **时序说明**：`ros_out` 记录的是"即将发出"的意图，不代表调用已成功。
> 如需区分是否成功，在 runtime 调用后额外 emit `ros_result`。

```python
def _emit(self, bt_node, semantic_source, target, comm_type,
          direction, ros_node, payload, summary='', tick_id=None):
    if self._logger is None:
        return
    self._logger.emit_comm({
        "event":            "ros_out",
        "ts":               time.time(),
        "tick_id":          tick_id if tick_id is not None else self._tick_id_getter(),
        "phase":            "tick",
        "bt_node":          bt_node,
        "ros_node":         ros_node,
        "semantic_source":  semantic_source,
        "target":           target,
        "comm_type":        comm_type,
        "direction":        direction,
        "payload":          payload,
        "summary":          summary,
        "attribution_confidence": "high",
        "node":             bt_node,   # legacy alias
    })

def some_business_method(self, bt_node, ...):
    # 1. 裁剪 payload（如需）
    payload = {...}
    # 2. emit ros_out — 在 runtime 调用之前
    self._emit(bt_node, "some_business_method", "/target_topic", "topic_publish", "out",
               "target_ros_node", payload)
    # 3. 执行 runtime 调用
    self._runtime.do_something(...)
    # 4. 如有返回值或异常，emit ros_result（可选）
```

### Step 6：condition 节点可选主动上报决策；action 节点可上报意图

**Condition 节点（`conditions.py`）**：

```python
def update(self):
    result = ...  # 判断逻辑
    status = py_trees.common.Status.SUCCESS if result else py_trees.common.Status.FAILURE
    self.emit_decision(
        inputs={"key": value, ...},
        status=status,
        reason="描述为什么得出这个结论",
    )
    return status
```

**Action 节点（`actions.py`）**：

```python
def initialise(self):
    # 上报执行意图 — action_intent
    self.emit_action_intent(
        action="描述即将执行的动作",
        inputs={},
    )

def update(self):
    # 关键分支决策可选 decision 事件
    self.emit_decision(
        inputs={},
        status=Status.RUNNING,
        reason="...",
    )
    # ⚠️  通过 facade 调用，ros_out 由 comm_facade._emit() 负责
    self._facade.some_action(...)
    return Status.RUNNING
```

> **关键约束**：action 节点不得直接调用 `emit_comm("ros_out", ...)`。
> 完整归因链：BT node `action_intent` → `semantic_facade.*` → `comm_facade.*` → `_emit(ros_out)`。

### Step 7：在 shutdown 中调用 `logger.close()`

```python
def run(self):
    # ... main loop ...
    self._obs_logger.close()    # 倒序 lastrun 文件为 newest-first，停止 rosbag
    rospy.signal_shutdown('shutdown')
```

---

## 6. 双层 JSONL 存储规则

| 文件 | 内容 | 写入策略 | crash 后状态 |
|---|---|---|---|
| `bt_debug_lastrun.jsonl` | 本次 session 全部 BT 事件 | append-only（oldest-first）；`close()` 原子倒序 | oldest-first（内容完整） |
| `bt_debug_recent.jsonl` | 最近 N tick BT 事件 | 每 tick 原子重写，newest-first | newest-first（最近 N tick） |
| `bt_ros_comm_debug_lastrun.jsonl` | 本次 session 全部通信事件 | 同 lastrun | 同 lastrun |
| `bt_ros_comm_debug_recent.jsonl` | 最近 N tick 通信事件 | 同 recent | newest-first |

**ROSA 工具读取规则**：
- BT node 运行中 → 优先读 `*_recent.jsonl`（实时反映最新状态）
- BT node 停止后 → 读 `*_lastrun.jsonl`（完整 session）
- 读 lastrun 时需将行重新倒序（newest-first → oldest-first）后传给 summarizer，保证时序正确

---

## 7. 事件 schema 速查

### BT 决策层（`/bt_debug` + `bt_debug_*.jsonl`）

```json
{"event": "tree_tick_start", "tick_id": 42, "ts": 1712345678.0}
{"event": "tree_tick_end",   "tick_id": 42, "status": "SUCCESS", "ts": 1712345678.1}
{"event": "tick_end",  "node": "NodeName", "type": "ClassName", "status": "Status.SUCCESS", "tick_id": 42, "ts": ...}
{"event": "decision",  "node": "CondName", "inputs": {...}, "status": "...", "reason": "...", "tick_id": 42, "ts": ...}
{"event": "bb_write",  "tick_id": 42, "writer": "ClientName", "key": "/ns/key", "value": "...", "ts": ...}
```

> **`bb_write` 说明**：每条 BB 写入对应一条独立记录（`writer`/`key`/`value` 三字段）。
> `BTDebugVisitor` 对同一 tick 内同一 key 的连续相同值进行了压缩（保留每段连续值的 first 和 last），
> 因此同一 key 可能在一个 tick 中出现多条记录。无 `writes: [...]` 聚合字段。

### 通信层（`/bt_ros_comm_debug` + `bt_ros_comm_debug_*.jsonl`）

```json
{
  "event": "ros_out",
  "tick_id": 42, "ts": ..., "phase": "tick",
  "bt_node": "NodeName", "semantic_source": "method_name",
  "target": "/topic_or_service", "comm_type": "topic_publish|service_call",
  "direction": "out", "ros_node": "target_ros_node",
  "payload": {...}, "summary": "", "attribution_confidence": "high"
}
{
  "event": "ros_in",
  "tick_id": 42, "ts": ...,
  "source": "/sensor_topic",
  "adapter": "ImuBalanceStateAdapter|LineDetectionAdapter",
  "received_count": 3
}
{
  "event": "input_state",
  "tick_id": 42, "ts": ...,
  "adapter": "ImuBalanceStateAdapter|LineDetectionAdapter",
  "bb_writes": {"robot_state": "STANDING"}
}
```

> `ros_in`：只在 `received_count > 0` 时 emit（本 tick 内 adapter 收到至少一条消息）；
> 无 payload 字段（只记录消息数，不记录消息内容）；无 `bt_node` 字段。
>
> `input_state`：每 tick 必有一条（无论是否收到新消息）；`bb_writes` 记录本 tick 实际写入 py_trees BB 的键值对。

---

## 8. 分层 import 约束

通过 `check_imports.py`（AST 检查）强制执行，建议每个新项目复制并按需修改：

```bash
python3 <project>/check_imports.py
```

各层禁止规则（模板）：

| 层 | 禁止 import | 禁止调用 |
|---|---|---|
| `app/` | — | `rospy.Subscriber`, `.emit_comm()` 直接调用 |
| `behaviours/` | `<project>.comm`, `<project>.infra` | `rospy.Publisher()`, `rospy.ServiceProxy()`, `.emit_comm()` |
| `semantics/` | `<project>.infra`, `bt_observability.ros_comm_tracer` | 同上 |
| `algorithms/` | `rospy`, `<project>.comm`, `<project>.infra`, `bt_observability` | `.emit_comm()` |
| `tree/` | `<project>.comm`, `<project>.infra`, `<project>.algorithms` | `rospy.Publisher()`, `CommFacade` |

> `app/` 层负责装配（创建 logger、adapters、facade、infra），但传感器订阅和 emit 均委托给 `ainex_bt_edu/input_adapters/`。

**节点组合附加规则**（新项目必须遵守）：

| 规则 | 说明 |
|---|---|
| 优先使用 ainex_bt_edu 节点 | `tree/` 中优先 import `ainex_bt_edu.behaviours.*`，有标准节点时禁止在项目 `behaviours/` 中重新实现 |
| condition 节点 → `conditions.py` | 项目专属感知判断节点（L1），无 facade 参数 |
| action 节点 → `actions.py` | 项目专属执行器指令节点（L2+），接受 facade 参数；只在 ainex_bt_edu 无对应时才建 |
| 项目节点必须继承 L1/L2 基类 | `conditions.py` 节点继承 `AinexL1ConditionNode`，`actions.py` 节点继承 `AinexL2ActionNode`；二者间接继承 `AinexBTNode`，不得直接继承 `py_trees.behaviour.Behaviour` |
| SemanticFacade 必须实现接口 | `semantics/semantic_facade.py` 必须继承 `ainex_bt_edu.base_facade.AinexBTFacade` |
| 节点不直接做 ROS 通信 | 节点内所有 ROS 动作（步态、舵机、动作播放）通过 `facade.*` 委托，禁止直接调用 manager / publisher |
| action 节点不 emit ros_out | `ros_out` 仅由 `comm/comm_facade.py` 的 `_emit()` 负责；action 节点只 emit `decision` / `action_intent` |
| bb_ros_bridge 与 infra_manifest 同步 | 新增 BB key 时，必须同步更新 `infra/bb_ros_bridge.py` 的 topic map 和 `infra/infra_manifest.py` 的接口记录 |

`emit_comm()` 允许出现的位置：

| 位置 | 用途 | 说明 |
|---|---|---|
| `comm/comm_facade.py` | business_out（`ros_out` / `ros_result`） | 唯一业务出站出口 |
| `ainex_bt_edu/input_adapters/` | business_in（`ros_in` + `input_state`） | 传感器输入适配；`rospy.Subscriber` 唯一合法位置；`app/` 层禁止直接调用 |
| `bt_observability/debug_event_logger.py` 内部 | `ros_topology_snapshot` | 共享观测模块内部事件；不属于业务通信链；业务层禁止直接调用 |

> **legacy 提示**：`bt_observability.ros_comm_tracer`（`ROSCommTracer` / `ManagerProxy`）已标为 legacy，
> 新项目禁止在 `semantics/` 及以上各层 import。`check_imports.py` 中应将其列为 `semantics/` 禁止 import。

---

## 9. 配置参数约定

建议每个新项目暴露以下标准 rosparam（与 marathon 保持一致）：

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `~max_rolling_ticks` | int | `30` | rolling 文件保留 tick 数 |
| `~bt_mode` | string | `run` | 启动模式：`run` / `pause` / `step` |
| `~start` | bool | `true` | 是否自动启动 BT ticking |
| `~bt_jsonl` | string | （见 Step 1）| 覆盖 lastrun BT 日志路径 |
| `~comm_jsonl` | string | （见 Step 1）| 覆盖 lastrun comm 日志路径 |

项目专属参数在此列表之外自行添加。

---

## 10. 新项目接入检查清单

| 步骤 | 检查项 | 完成标志 |
|---|---|---|
| **节点组合** | `tree/` 中所有通用节点（感知条件、步态动作、跌倒恢复）从 `ainex_bt_edu.behaviours` import | grep 项目无自定义等价重写 |
| **节点组合** | 项目 `conditions.py` 节点继承 `AinexL1ConditionNode`，`actions.py` 节点继承 `AinexL2ActionNode`，声明 `LEVEL` 和 `BB_READS` / `BB_WRITES` | 类定义可见 `LEVEL = 'L1'|'L2'` |
| **节点组合** | `semantic_facade.py` 继承 `AinexBTFacade`，实现所有抽象方法 | `isinstance(facade, AinexBTFacade)` 为 True |
| Step 1 | `DebugEventLogger` 在 `__init__()` 中创建，路径指向项目 `log/` | logger 对象存在，文件路径正确 |
| Step 2 | `BTDebugVisitor` 挂载到 tree（visitors + pre/post_tick_handlers） | 启动后 `/bt_debug` topic 有输出 |
| Step 3 | `run()` 中 `tick_id` 在 `should_tick()` 之后、`tree.tick()` 之前递增 | `bt_debug_lastrun.jsonl` 中 tick_id 单调递增 |
| Step 4 | 项目所需的 input adapter（`ainex_bt_edu/input_adapters/`）已集成；`run()` 使用两阶段协议：同一 lock 临界区内 `snapshot_and_reset()` + 释放锁后 `write_snapshot()` | `bt_ros_comm_debug_*.jsonl` 中有 `ros_in` + `input_state` 事件 |
| Step 5 | `comm_facade.py` 每个公开方法调用 `_emit()`；action 节点不直接 emit ros_out | `bt_ros_comm_debug_*.jsonl` 中有 `ros_out` 事件，无节点层直接 emit |
| Step 6 | `shutdown` / `run()` 末尾调用 `logger.close()` | 正常关闭后 lastrun 文件为 newest-first |
| Step 7 | `infra_manifest.py` 写入 `infra_comm_manifest_lastrun.json` | 启动后 JSON 文件存在，接口数量正确 |
| Step 7 | `bb_ros_bridge.py` topic map 与 `infra_manifest.py` PROJECT_INTERFACES 保持同步 | 新 BB key 两处均有记录 |
| Step 8 | `check_imports.py` 通过，无分层违规 | `python3 check_imports.py` 输出 OK |

---

## 11. 已知约束与注意事项

- **`BTDebugVisitor` `full=False`**：只 visit 被实际 tick 到的节点，未执行节点不产生 `tick_end` 事件。
- **BB write 去重**：同一 key 在一个 tick 内多次写入相同值时，只记 first 和 last，中间重复压缩。跨 tick 不保留状态。
- **`ros_topology_snapshot`** 异步执行（daemon thread），可能与当前 tick 错位一个 tick。
- **crash 场景**：`close()` 不会被调用，`lastrun` 文件保持 oldest-first，内容完整但未倒序。ROSA 的 `read_bt_obs` 工具需在读取时主动倒序。
- **rosbag 录制**：默认关闭；需要时在 `DebugEventLogger` 构造时传入 `rosbag_topics` 列表。
- **`logger=None`**：所有 `emit_bt` / `emit_comm` 调用均为 no-op，可安全用于无日志的单元测试环境。

---

## 12. 参考实现

| 文件 | 说明 |
|---|---|
| `bt_observability/debug_event_logger.py` | 共享模块，所有项目共用 |
| `bt_observability/bt_debug_visitor.py` | 共享模块，所有项目共用 |
| `marathon/app/marathon_bt_node.py` | 完整集成示例（Steps 1–7，含两个 adapter） |
| `fall_recovery/app/fall_recovery_bt_node.py` | 简化集成示例（单 adapter：ImuBalanceStateAdapter） |
| `marathon/comm/comm_facade.py` | `ros_out` 日志实现参考 |
| `fall_recovery/infra/bt_exec_controller.py` | `BTExecController` 实现（service names: ~bt/run/pause/step） |
| `marathon/check_imports.py` | 分层约束 AST 检查参考 |
| `bt_infra_manifest_rules.md` | 基础设施通信清单框架（Step 7 详情） |
| `ainex_bt_edu/src/ainex_bt_edu/base_facade.py` | `AinexBTFacade` 抽象接口定义 |
| `ainex_bt_edu/src/ainex_bt_edu/base_node.py` | BT node 基类族：`AinexBTNode` / `AinexL1ConditionNode` / `AinexL2ActionNode` |
| `ainex_bt_edu/src/ainex_bt_edu/input_adapters/base_adapter.py` | `AinexInputAdapter` business-in 基类 |
| `ainex_bt_edu/src/ainex_bt_edu/behaviours/` | 标准节点库（L1/L2；以 marathon 节点为第一批参考实现） |
| `marathon/tree/marathon_bt.py` | 节点组合模式参考实现（tree/ 只做组装） |
| `marathon/semantics/semantic_facade.py` | `AinexBTFacade` 实现参考 |
