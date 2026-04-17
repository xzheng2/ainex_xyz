# BT 可观测性系统框架 — ADR

> 适用范围：所有基于 `py_trees` + ROS 的 BT 项目
> 最后更新：2026-04-16
> **完整规则同步维护于 skill**: `~/.claude/skills/ainex-bt-project/references/bt_observability_rules.md`

---

## 决策背景

每个 BT 项目需要统一、可插拔的可观测性能力，覆盖 BT 决策层日志、ROS 通信层日志、
双层存储（ROS topic 实时 + JSONL 离线）、以及 ROSA/LLM 可查询的结构化输出。

---

## 核心约束（5 条）

1. **共享模块直接 import，不复制**
   ```python
   from bt_observability.debug_event_logger import DebugEventLogger
   from bt_observability.bt_debug_visitor import BTDebugVisitor
   ```

2. **通信边界三类**：`business_out`（唯一出口：`comm/comm_facade.py`）、
   `business_in`（唯一出口：`ainex_bt_edu/input_adapters/`，`app/` 层禁止 `rospy.Subscriber` 和 `ros_in` emit）、
   `infra`（写入 `infra_comm_manifest_lastrun.json`，不进 `bt_ros_comm_debug`）

3. **节点组合约束**：`tree/` 优先 import `ainex_bt_edu` 标准节点；项目节点继承
   `AinexBTNode`；`semantic_facade.py` 继承 `AinexBTFacade`；节点不直接调用 rospy

4. **`logger=None` 零成本**：所有 `emit_bt` / `emit_comm` 为 no-op，不影响无日志环境

5. **`tick_id`** 在 `should_tick()` 之后、`tree.tick()` 之前递增；`ros_in` 按 tick 去重

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
    ├── imu_balance_state_adapter.py    # /imu → /latched/robot_state
    └── line_detection_adapter.py       # /object/pixel_coords → /latched/{line_data,...}
```

`rospy.Subscriber` **只允许** 出现在 `input_adapters/` 中，不允许出现在 `app/` 或任何 BT 层。

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
ainex_behavior/
└── <project_name>/
    ├── __init__.py
    ├── app/
    │   ├── <project>_bt_node.py    # node 入口；创建 logger；ros_in 适配日志
    │   └── ros_msg_utils.py        # msg_to_dict（从 marathon 复制或共用）
    ├── tree/
    │   └── <project>_bt.py         # BT 树结构定义
    ├── behaviours/
    │   ├── actions.py
    │   └── conditions.py           # condition 节点可主动 emit_bt decision 事件
    ├── semantics/
    │   └── semantic_facade.py      # 业务语义层；只调用 comm_facade
    ├── comm/
    │   └── comm_facade.py          # 唯一 ros_out / ros_result 日志出口
    ├── algorithms/                 # 纯算法层（无 ROS 依赖）
    ├── infra/
    │   ├── __init__.py
    │   ├── infra_manifest.py       # 基础设施通信清单（见 bt_infra_manifest_framework.md）
    │   ├── tree_publisher.py
    │   ├── bb_ros_bridge.py
    │   └── bt_exec_controller.py
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
ainex_bt_edu/input_adapters/        ← rospy.Subscriber 唯一合法位置
    ImuBalanceStateAdapter              每 tick：snapshot_and_reset()（under lock）
    LineDetectionAdapter                         + write_snapshot()（after lock）
    │  emits: ros_in（received_count>0 时）+ input_state（每 tick 必有）
    ▼
app/<project>_bt_node.py            ← 装配层；禁止 rospy.Subscriber 和 emit_comm
    │  创建 adapters，装配 tree/facade/infra；调用 adapter.snapshot_and_reset/write_snapshot
    ▼
tree/<project>_bt.py
    ▼
behaviours/actions.py + conditions.py
    │  只调用 semantic_facade.*
    │  conditions 可选 emit_bt("decision", ...)
    ▼
semantics/semantic_facade.py
    │  只调用 comm_facade.*
    ▼
comm/comm_facade.py              ← 唯一 business_out（ros_out / ros_result）emit_comm 出口
    ▼
runtime（GaitManager / Publisher / ServiceProxy）
```

`bt_observability/` 横切所有层，由 `app` 层初始化后注入，**不在各层直接 import**。

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
| `input_state` | business_in | `input_adapters/` 的 `write_snapshot()` | 每 tick 必有一条；bb_writes 记录本 tick latch_to_blackboard() 实际写入 py_trees BB 的值 |
| `ros_out` | business_out | `comm/comm_facade.py` 各公开方法 | 在 runtime 调用**前**写，表示通信意图 |
| `ros_result` | business_out | `comm/comm_facade.py` 各公开方法 | 有返回值或异常时另写，与 ros_out 配对 |
| `decision` | — | `behaviours/conditions.py`（可选） | condition 节点主动上报判断输入/结果 |
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

### Step 3：在 run() 中递增 tick_id

`tick_id` 必须在 `tree.tick()` 之前递增，且只在 `should_tick()` 通过时递增：

```python
def run(self):
    rate = rospy.Rate(10)
    while self.running and not rospy.is_shutdown():
        if self.start and self._exec_ctrl.should_tick():
            self._tick_id += 1
            self._bb_meta.tick_id = self._tick_id   # 写入 BB（可选，供 BB bridge 使用）
            self._latch_inputs()
            self.tree.tick()
        rate.sleep()
```

### Step 4：使用 InputAdapter 适配传感器输入

不在 `app/` 层直接创建 `rospy.Subscriber` 或 emit `ros_in`。
每个传感器对应一个 `ainex_bt_edu/input_adapters/` 中的 Adapter 类。

`app/<project>_bt_node.py` 中：

```python
from ainex_bt_edu.input_adapters.imu_balance_state_adapter import ImuBalanceStateAdapter
from ainex_bt_edu.input_adapters.line_detection_adapter import LineDetectionAdapter

# 在 __init__() 中创建 adapters（在 obs_logger 创建之后）：
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

在 `run()` 的 `tree.tick()` 之前，两阶段 latch（保证同一 tick 快照一致性）：

```python
# Phase 1：在同一 lock 临界区内原子 snapshot 所有 adapters
with self.lock:
    imu_snap  = self._imu_adapter.snapshot_and_reset()
    line_snap = self._line_adapter.snapshot_and_reset()
# Phase 2：lock 释放后写 BB + emit 日志（主线程，无竞争）
self._imu_adapter.write_snapshot(imu_snap,  self._tick_id)
self._line_adapter.write_snapshot(line_snap, self._tick_id)
self.tree.tick()
```

`write_snapshot()` 内部：
- 若 `received_count > 0`：emit `ros_in`（简化 schema，含 `source` / `received_count`，不含完整 payload）
- 写 BB（`self._bb.<key> = snap[...]`）
- 无论 `received_count` 为多少：emit `input_state`（`bb_writes` 记录实际写入 BB 的值）

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

### Step 6：condition 节点可选主动上报决策

```python
def update(self):
    result = ...  # 判断逻辑
    status = py_trees.common.Status.SUCCESS if result else py_trees.common.Status.FAILURE
    if self._logger:
        self._logger.emit_bt({
            "event":  "decision",
            "node":   self.name,
            "inputs": {"key": value, ...},
            "status": str(status),
            "reason": "描述为什么得出这个结论",
        })
    return status
```

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
  "source": "/imu",
  "adapter": "ImuBalanceStateAdapter",
  "received_count": 16
}
{
  "event": "input_state",
  "tick_id": 42, "ts": ...,
  "adapter": "ImuBalanceStateAdapter",
  "bb_writes": {"/latched/robot_state": "stand"}
}
{
  "event": "input_state",
  "tick_id": 42, "ts": ...,
  "adapter": "LineDetectionAdapter",
  "bb_writes": {
    "/latched/line_data": {"x": 80.0, "width": 12.0},
    "/latched/last_line_x": 80.0,
    "/latched/camera_lost_count": 0
  }
}
```

> `ros_in`：`received_count` = 自上次 latch 后该 adapter 收到的 ROS 消息数（每次 latch 后置 0）；不含 payload，无 `bt_node` 字段（callback 在 BT 执行上下文之外运行）。
>
> `input_state`：每 tick 必有一条（即使 `received_count==0`）；`bb_writes` 记录 `write_snapshot()` 实际写入 py_trees blackboard 的 key-value，代表该 tick BT 节点实际读到的输入状态。

---

## 8. 分层 import 约束

通过 `check_imports.py`（AST 检查）强制执行，建议每个新项目复制并按需修改：

```bash
python3 <project>/check_imports.py
```

各层禁止规则（模板）：

| 层 | 禁止 import | 禁止调用 |
|---|---|---|
| `app/` | — | `rospy.Subscriber()`, `.emit_comm()` |
| `behaviours/` | `<project>.comm`, `<project>.infra` | `rospy.Publisher()`, `rospy.Subscriber()`, `rospy.ServiceProxy()`, `.emit_comm()` |
| `semantics/` | `<project>.infra`, `bt_observability.ros_comm_tracer` | `rospy.Publisher()`, `rospy.ServiceProxy()`, `.emit_comm()` |
| `algorithms/` | `rospy`, `<project>.comm`, `<project>.infra`, `bt_observability` | `.emit_comm()` |
| `tree/` | `<project>.comm`, `<project>.infra`, `<project>.algorithms` | `rospy.Publisher()`, `CommFacade` |

`emit_comm()` 允许出现的位置：

| 位置 | 用途 | 说明 |
|---|---|---|
| `comm/comm_facade.py` | business_out（`ros_out` / `ros_result`） | 唯一业务出站出口 |
| `ainex_bt_edu/input_adapters/` | business_in（`ros_in` + `input_state`） | 传感器输入适配；`app/` 层禁止直接 emit |
| `bt_observability/debug_event_logger.py` 内部 | `ros_topology_snapshot` | 共享观测模块内部事件；不属于业务通信链；业务层禁止直接调用 |

> **`rospy.Subscriber` 唯一合法位置**：`ainex_bt_edu/input_adapters/`。
> `app/`、`behaviours/`、`tree/`、`semantics/`、`algorithms/` 均禁止出现 `rospy.Subscriber`。

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
| Step 1 | `DebugEventLogger` 在 `__init__()` 中创建，路径指向项目 `log/` | logger 对象存在，文件路径正确 |
| Step 2 | `BTDebugVisitor` 挂载到 tree（visitors + pre/post_tick_handlers） | 启动后 `/bt_debug` topic 有输出 |
| Step 3 | `run()` 中 `tick_id` 在 `should_tick()` 之后、`tree.tick()` 之前递增 | `bt_debug_lastrun.jsonl` 中 tick_id 单调递增 |
| Step 4 | 每个传感器有对应 InputAdapter 类；`snapshot_and_reset()` 在 lock 下与其他 adapter 一起原子调用；`write_snapshot()` emit `ros_in`（received_count>0 时）+ `input_state`（每 tick 必有） | `bt_ros_comm_debug_*.jsonl` 中有 `ros_in` 和 `input_state` 事件 |
| Step 5 | `comm_facade.py` 每个公开方法调用 `_emit()` | `bt_ros_comm_debug_*.jsonl` 中有 `ros_out` 事件 |
| Step 6 | `shutdown` / `run()` 末尾调用 `logger.close()` | 正常关闭后 lastrun 文件为 newest-first |
| Step 7 | `infra_manifest.py` 写入 `infra_comm_manifest_lastrun.json` | 启动后 JSON 文件存在，接口数量正确 |
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
| `marathon/app/marathon_bt_node.py` | 完整集成示例（Steps 1–7） |
| `marathon/comm/comm_facade.py` | `ros_out` 日志实现参考 |
| `marathon/check_imports.py` | 分层约束 AST 检查参考 |
| `bt_infra_manifest_framework.md` | 基础设施通信清单框架（Step 7 详情） |
| 完整规则（12 节）| `~/.claude/skills/ainex-bt-project/references/bt_observability_rules.md` |
| 新项目脚手架 | `/ainex-bt-project <project_name>` (Claude Code skill) |
