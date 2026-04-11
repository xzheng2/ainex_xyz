# BT 可观测性系统框架

> 适用范围：所有基于 `py_trees` + ROS 的 BT 项目
> 参考实现：`marathon/`（`ainex_behavior` 包）
> 共享模块：`ainex_behavior/bt_observability/`
> 最后更新：2026-04-10

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
    └── ros_comm_tracer.py      # 工具类（ROSCommTracer）
```

新 BT 项目**直接 import，不复制**：

```python
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor
```

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
app/<project>_bt_node.py
    │  ros_in（输入适配日志）← 每个传感器 callback 在首次 tick 时 emit_comm
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
comm/comm_facade.py              ← 唯一 ros_out / ros_result emit_comm 出口
    ▼
runtime（GaitManager / Publisher / ServiceProxy）
```

`bt_observability/` 横切所有层，由 `app` 层初始化后注入，**不在各层直接 import**。

### 日志责任归属原则

| 事件类型 | 谁负责 emit | 说明 |
|---|---|---|
| `ros_in` | `app/<project>_bt_node.py` callback | 每 tick 第一次收到时记录一次，避免 30 Hz 刷屏 |
| `ros_out` / `ros_result` | `comm/comm_facade.py` 各公开方法 | 唯一出口，不可绕过 |
| `decision` | `behaviours/conditions.py`（可选） | condition 节点主动上报判断输入/结果 |
| `tree_tick_start/end` | `BTDebugVisitor`（自动） | 挂载后自动记录，无需手动 |
| `tick_end`（节点级） | `BTDebugVisitor`（自动） | 同上 |
| `bb_write` | `BTDebugVisitor`（自动） | flush 时自动记录 BB 写入去重结果 |

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

### Step 4：在传感器 callback 中添加 `ros_in` 日志

每个传感器 callback 记录一次，使用 `_last_*_emit_tick` 去重（同一 tick 只记一条）：

```python
self._last_<sensor>_emit_tick = -1

def _<sensor>_callback(self, msg):
    if self._tick_id != self._last_<sensor>_emit_tick:
        self._last_<sensor>_emit_tick = self._tick_id
        self._obs_logger.emit_comm({
            "event":     "ros_in",
            "ts":        time.time(),
            "tick_id":   self._tick_id,
            "comm_type": "topic_subscribe",
            "direction": "in",
            "target":    "/<sensor_topic>",
            "ros_node":  None,
            "adapter":   "<sensor>_callback",
            "payload":   msg_to_dict(msg),
        })
    # ... 实际业务逻辑
```

### Step 5：在 `comm_facade.py` 中添加 `ros_out` 日志

每个公开业务方法执行后调用 `_emit()`：

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
        "node":             bt_node,
    })

def some_business_method(self, bt_node, ...):
    # 执行 ROS 调用
    self._gait_manager.do_something(...)
    # 记录日志
    self._emit(bt_node, "some_business_method", "/target_topic", "topic_publish", "out", ...)
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
{"event": "tick_end",  "node": "NodeName", "status": "Status.SUCCESS", "tick_id": 42, "ts": ...}
{"event": "decision",  "node": "CondName", "inputs": {...}, "status": "...", "reason": "...", "tick_id": 42, "ts": ...}
{"event": "bb_write",  "tick_id": 42, "writes": [{"key": "/ns/key", "value_repr": "...", "position": "first|only|last"}], "ts": ...}
```

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
  "comm_type": "topic_subscribe", "direction": "in",
  "target": "/sensor_topic", "ros_node": null,
  "adapter": "callback_name", "payload": {...}
}
```

> `ros_in` 无 `bt_node` 字段 — callback 在 BT 执行上下文之外运行。

---

## 8. 分层 import 约束

通过 `check_imports.py`（AST 检查）强制执行，建议每个新项目复制并按需修改：

```bash
python3 <project>/check_imports.py
```

各层禁止规则（模板）：

| 层 | 禁止 import | 禁止调用 |
|---|---|---|
| `behaviours/` | `<project>.comm`, `<project>.infra` | `rospy.Publisher()`, `rospy.ServiceProxy()`, `.emit_comm()` |
| `semantics/` | `<project>.infra`, `bt_observability.ros_comm_tracer` | 同上 |
| `algorithms/` | `rospy`, `<project>.comm`, `<project>.infra`, `bt_observability` | `.emit_comm()` |
| `tree/` | `<project>.comm`, `<project>.infra`, `<project>.algorithms` | `rospy.Publisher()`, `CommFacade` |

`emit_comm()` 只允许出现在两处：
- `comm/comm_facade.py`（业务出站）
- `app/<project>_bt_node.py`（传感器输入适配）

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
| Step 4 | 每个传感器 callback 有 `_last_*_emit_tick` 去重的 `ros_in` 日志 | `bt_ros_comm_debug_*.jsonl` 中有 `ros_in` 事件 |
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
| `marathon/ainex_bt_observability_execution_plan.md` | marathon 专项技术参考 |
| `bt_infra_manifest_framework.md` | 基础设施通信清单框架（Step 7 详情） |
