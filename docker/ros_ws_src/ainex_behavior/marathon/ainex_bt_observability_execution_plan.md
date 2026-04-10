# Ainex BT 可观测性系统 — 技术参考

> **适用范围：** `ainex_behavior` 包下所有 BT 项目（marathon 为首个落地项目）
>
> **本文档描述当前已实现的系统现状，不包含历史实施步骤。**

---

## 1. 目录结构

```
ainex_behavior/
├── bt_observability/               # 共享可观测性模块（不属于 marathon 层级）
│   ├── __init__.py
│   ├── debug_event_logger.py       # 统一输出层：ROS topic + JSONL 双层存储
│   ├── bt_debug_visitor.py         # BT 决策层 Visitor
│   └── ros_comm_tracer.py          # ROSCommTracer（工具类）；ManagerProxy（已从业务主链移除）
└── marathon/
    ├── __init__.py
    ├── app/
    │   ├── marathon_bt_node.py     # 节点入口；ros_in 输入适配日志
    │   └── ros_msg_utils.py        # msg_to_dict 独立工具
    ├── tree/marathon_bt.py
    ├── behaviours/
    │   ├── actions.py
    │   └── conditions.py
    ├── semantics/semantic_facade.py
    ├── comm/
    │   └── comm_facade.py          # 唯一业务 ros_out 日志出口
    ├── algorithms/visual_patrol.py
    ├── infra/
    │   ├── bb_ros_bridge.py
    │   ├── bt_exec_controller.py
    │   ├── tree_publisher.py
    │   └── infra_manifest.py
    ├── check_imports.py            # AST 分层约束检查
    └── log/                        # 运行时日志（git 忽略）
        ├── bt_debug_lastrun.jsonl
        ├── bt_debug_recent.jsonl
        ├── bt_ros_comm_debug_lastrun.jsonl
        ├── bt_ros_comm_debug_recent.jsonl
        └── infra_comm_manifest_lastrun.json
```

---

## 2. 架构分层

```
app/marathon_bt_node.py
    │  ros_in (输入适配日志)
    ▼
tree/marathon_bt.py
    ▼
behaviours/actions.py + conditions.py
    │  只调用 semantic_facade.*
    ▼
semantics/semantic_facade.py
    │  只调用 comm_facade.*
    ▼
comm/comm_facade.py              ← 唯一业务 ros_out / ros_result 日志出口
    │
    ▼
runtime (GaitManager / MotionManager / Publisher)
```

`bt_observability/` 横切所有层，由 `app` 层初始化后注入。

---

## 3. 核心组件

### 3.1 `debug_event_logger.py`

统一事件输出层，由 `app` 层创建，注入到 BT visitor 和 comm_facade。

```python
DebugEventLogger(
    bt_topic="/bt_debug",                    # ROS String topic（BT 决策层）
    comm_topic="/bt_ros_comm_debug",         # ROS String topic（通信层）
    bt_lastrun_jsonl="log/bt_debug_lastrun.jsonl",
    comm_lastrun_jsonl="log/bt_ros_comm_debug_lastrun.jsonl",
    rolling_bt_jsonl="log/bt_debug_recent.jsonl",
    rolling_comm_jsonl="log/bt_ros_comm_debug_recent.jsonl",
    max_rolling_ticks=30,
    tick_id_getter=lambda: self._tick_id,
    rosbag_topics=None,   # 可选：传入 topic 列表开启 rosbag 录制
    rosbag_dir=None,
)
```

**双层 JSONL 存储：**

| 文件 | 内容 | 存储策略 |
|---|---|---|
| `bt_debug_lastrun.jsonl` | 本次 session 全部 BT 决策事件 | 内存缓冲，每 tick 原子重写，newest-first |
| `bt_debug_recent.jsonl` | 最近 `max_rolling_ticks` 个 tick 的 BT 事件 | 每 tick 滚动写，newest-first |
| `bt_ros_comm_debug_lastrun.jsonl` | 本次 session 全部通信事件 | 同上 |
| `bt_ros_comm_debug_recent.jsonl` | 最近 N 个 tick 的通信事件 | 同上 |

**所有文件 newest-first**（tick_id 最大的记录在文件顶部）。

**Rosbag 录制（可选）：**
- 传入 `rosbag_topics` 列表时，node 启动自动在后台开启 rosbag 录制
- 60 秒分割，最多保留 30 个分割文件（约 30 分钟）
- `_stop_rosbag()` 在 node 关闭时调用

**公开 API：**

```python
logger.emit_bt(payload: dict)         # 发布 BT 事件
logger.emit_comm(payload: dict)       # 发布通信事件
logger.begin_tick(tick_id: int)       # 每 tick 开始时清空缓冲
logger.end_tick(tick_id: int)         # 每 tick 结束时 flush 到文件
logger.snapshot_ros_topology(tick_id) # 异步收集 rosnode/rostopic 快照
logger.close()                        # no-op（文件无需显式关闭）
```

---

### 3.2 `bt_debug_visitor.py`

`BTDebugVisitor(py_trees.visitors.VisitorBase)` — BT 决策层 Visitor。

**接入方式（`app/marathon_bt_node.py`）：**

```python
self._bt_visitor = BTDebugVisitor(self._obs_logger, lambda: self._tick_id)
self.tree.visitors.append(self._bt_visitor)
self.tree.pre_tick_handlers.append(self._bt_visitor.on_tree_tick_start)
self.tree.post_tick_handlers.append(self._bt_visitor.on_tree_tick_end)
```

**生成事件：**

| 事件 | 触发时机 | 目标流 |
|---|---|---|
| `tree_tick_start` | 每 tick 开始 | bt |
| `tree_tick_end` | 每 tick 结束 | bt |
| `tick_end` | 每个被实际 tick 的节点执行后 | bt |
| `decision` | 由 condition 节点主动调用 `logger.emit_bt()` | bt |
| `bb_write` | flush 当 tick 内 BB 写入（去重后） | bt |
| `ros_topology_snapshot` | root 状态变化时异步触发 | bt |

**BB write 去重：** 同一 key 在一个 tick 内多次写入相同值时，只保留 first 和 last，中间重复值被压缩。

---

### 3.3 `comm_facade.py`

**业务通信日志唯一出口。** 所有 `ros_out` 事件只在此处生成。

`_emit()` 私有方法，每个公开方法调用后触发：

```python
def _emit(self, bt_node, semantic_source, target, comm_type,
          direction, ros_node, payload, summary='', tick_id=None)
```

**6 个公开方法 → 各自的 `ros_out` 日志：**

| 方法 | target | comm_type | direction |
|---|---|---|---|
| `disable_gait` | `/walking/command` | `service_call` | `call` |
| `enable_gait` | `/walking/command` | `service_call` | `call` |
| `set_step` | `walking/set_param` | `topic_publish` | `out` |
| `run_action` | `ros_robot_controller/bus_servo/set_position` | `topic_publish` | `out` |
| `set_servos_position` | `ros_robot_controller/bus_servo/set_position` | `topic_publish` | `out` |
| `publish_buzzer` | `/ros_robot_controller/set_buzzer` | `topic_publish` | `out` |

**`ManagerProxy` 已从业务主链移除。** `CommFacade` 直接持有真实 manager 对象并自行记录日志。

---

### 3.4 `ros_comm_tracer.py`

包含三个组件，目前使用情况：

| 组件 | 状态 | 说明 |
|---|---|---|
| `ProxyContext` | 已移除出业务主链 | 原用于 ManagerProxy 归因，现 comm_facade 直接传参 |
| `ManagerProxy` | 已移除出业务主链 | 类仍保留但不在 marathon 中使用 |
| `ROSCommTracer` | 保留（工具类） | `_msg_to_dict` 静态方法已迁移至 `app/ros_msg_utils.py` |

---

### 3.5 `ros_msg_utils.py`（`app/` 层独立工具）

```python
from marathon.app.ros_msg_utils import msg_to_dict
payload = msg_to_dict(ros_msg)   # 递归转 dict，支持 ainex_interfaces
```

---

## 4. 事件类型定义

### 4.1 BT 决策层（`/bt_debug` + `bt_debug_*.jsonl`）

**`tree_tick_start`**
```json
{"event": "tree_tick_start", "tick_id": 42, "ts": 1712345678.0}
```

**`tree_tick_end`**
```json
{"event": "tree_tick_end", "tick_id": 42, "status": "SUCCESS", "ts": 1712345678.1}
```

**`tick_end`**（每个被 tick 到的节点）
```json
{
  "event": "tick_end",
  "node": "FollowLine",
  "status": "Status.SUCCESS",
  "tick_id": 42,
  "ts": 1712345678.05
}
```

**`decision`**（由 condition 节点主动发出）
```json
{
  "event": "decision",
  "node": "IsLineDetected",
  "inputs": {"line_detected": true, "line_x": 78.3, "line_width": 32},
  "status": "Status.SUCCESS",
  "reason": "line_data present",
  "tick_id": 42,
  "ts": 1712345678.02
}
```

**`bb_write`**
```json
{
  "event": "bb_write",
  "tick_id": 42,
  "writes": [
    {"key": "/latched/line_data", "value_repr": "ObjectInfo(x=78.3...)", "position": "first"},
    {"key": "/latched/camera_lost_count", "value_repr": "0", "position": "only"}
  ],
  "ts": 1712345678.1
}
```

---

### 4.2 通信层（`/bt_ros_comm_debug` + `bt_ros_comm_debug_*.jsonl`）

**`ros_out`**（业务出站，由 `comm_facade.py` 生成）
```json
{
  "event": "ros_out",
  "ts": 1712345678.05,
  "tick_id": 42,
  "phase": "tick",
  "bt_node": "FollowLine",
  "ros_node": "ainex_controller",
  "semantic_source": "follow_line",
  "target": "walking/set_param",
  "comm_type": "topic_publish",
  "direction": "out",
  "payload": {"dsp": [300, 0.2, 0.02], "x": 0.015, "y": 0, "yaw": -2},
  "summary": "",
  "attribution_confidence": "high",
  "node": "FollowLine"
}
```

**`ros_in`**（业务输入适配，由 `app/marathon_bt_node.py` callbacks 生成）
```json
{
  "event": "ros_in",
  "ts": 1712345678.01,
  "tick_id": 42,
  "comm_type": "topic_subscribe",
  "direction": "in",
  "target": "/object/pixel_coords",
  "ros_node": null,
  "adapter": "objects_callback",
  "payload": {...}
}
```

> `ros_in` 无 `bt_node` 字段 — callback 运行在 BT 执行上下文之外。

---

## 5. 非业务通信 — infra manifest

非业务 ROS 接口（`tree_publisher`、`bb_ros_bridge`、`bt_exec_controller`、固定订阅/发布）不进入业务 comm 日志，在 node 启动时写入静态 manifest：

```
marathon/log/infra_comm_manifest_lastrun.json
```

由 `infra/infra_manifest.py` 的 `build_infra_manifest(node_name)` + `write_infra_manifest(path, items)` 生成。

每条记录：
```json
{
  "component": "BTExecController",
  "kind": "service_server",
  "name": "~bt/pause",
  "resolved_name": "/marathon_bt/bt/pause",
  "msg_or_srv_type": "std_srvs/Empty",
  "purpose": "pause BT ticking",
  "bt_decision_related": false,
  "excluded_from_generic_ros_facade": true
}
```

---

## 6. 分层 import 约束

通过 `marathon/check_imports.py`（AST 检查）强制执行：

```bash
python3 marathon/check_imports.py
```

各层禁止项摘要：

| 层 | 禁止 import | 禁止调用 |
|---|---|---|
| `behaviours/` | `marathon.comm`, `marathon.infra` | `rospy.Publisher()`, `rospy.ServiceProxy()`, `.emit_comm()` |
| `semantics/` | `marathon.infra`, `bt_observability.ros_comm_tracer` | 同上 + `ManagerProxy` |
| `algorithms/` | `rospy`, `marathon.comm`, `marathon.infra`, `bt_observability` | `.emit_comm()`, `.set_step()`, `.run_action()` |
| `tree/` | `marathon.comm`, `marathon.infra`, `marathon.algorithms` | `rospy.Publisher()`, `CommFacade`, `GaitManager` |

`emit_comm()` 只允许出现在：
- `comm/comm_facade.py`（`ros_out` 业务出站）
- `app/marathon_bt_node.py`（`ros_in` 输入适配）

---

## 7. 配置参数（rosparam）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `~bt_jsonl` | `log/bt_debug_lastrun.jsonl` | BT 决策层 lastrun 路径 |
| `~comm_jsonl` | `log/bt_ros_comm_debug_lastrun.jsonl` | 通信层 lastrun 路径 |
| `~max_rolling_ticks` | `30` | rolling 文件保留 tick 数 |
| `~bt_mode` | `run` | 启动模式：`run` / `pause` / `step` |
| `~find_line` | `gait` | FindLine 变体：`gait` / `head_sweep` |
| `~start` | `true` | 是否自动启动 BT |
| `~color` | `black` | 目标线颜色 |

---

## 8. 接入其他 BT 项目

最小接入步骤：

```python
# 1. 初始化 logger
logger = DebugEventLogger(
    bt_lastrun_jsonl=".../log/bt_debug_lastrun.jsonl",
    comm_lastrun_jsonl=".../log/bt_ros_comm_debug_lastrun.jsonl",
    rolling_bt_jsonl=".../log/bt_debug_recent.jsonl",
    rolling_comm_jsonl=".../log/bt_ros_comm_debug_recent.jsonl",
    tick_id_getter=lambda: self._tick_id,
)

# 2. 挂载 visitor
bt_visitor = BTDebugVisitor(logger, lambda: self._tick_id)
tree.visitors.append(bt_visitor)
tree.pre_tick_handlers.append(bt_visitor.on_tree_tick_start)
tree.post_tick_handlers.append(bt_visitor.on_tree_tick_end)

# 3. 将 logger 传入 CommFacade
comm_facade = CommFacade(..., logger=logger, tick_id_getter=...)

# 4. logger=None 时所有 emit_* 调用均为 no-op，向后兼容
```

Condition 节点可选主动上报决策：

```python
if self._logger:
    self._logger.emit_bt({
        "event": "decision",
        "node": self.name,
        "inputs": {"robot_state": state},
        "status": str(status),
        "reason": "...",
    })
```

---

## 9. ROSA Agent 接口

ROSA 通过以下工具消费可观测性数据：

| 工具 | 数据来源 | 用途 |
|---|---|---|
| `read_bt_obs` | JSONL 文件（4 个） | 统一入口；BT 节点运行中自动选 recent，停止后选 lastrun |
| `get_bt_status` | BB bridge topics `/bt/marathon/bb/*` | 实时 BB key 快照（无文件读取） |
| `read_last_run_summary` | `bt_debug_lastrun.jsonl` | 全 session 摘要 |

`read_bt_obs` 读取 JSONL 时会重新倒序（newest-first → oldest-first）后传给 summarizer，保证时序正确。

---

## 10. 已知约束

- `bt_debug_visitor.py` 的 `full=False`：只 visit 实际被 tick 到的节点，未执行节点不产生 `tick_end` 事件。
- BB write 去重在每个 tick 内独立处理，跨 tick 不保留状态。
- `ros_topology_snapshot` 异步执行（daemon thread），不阻塞主循环，可能与当前 tick 错位一个 tick。
- `close()` 为 no-op；最后一个 tick 的数据已在 `end_tick()` 时原子写入，node 被 SIGINT 终止时不会丢失已写入内容。
- rosbag 录制需手动在 `DebugEventLogger` 构造时传入 `rosbag_topics`，默认关闭。
