# BT 可观测性系统 — 录制侧完善执行方案
## 在现有 bt_observability 基础上的增量修改

> **前置依赖**：`ainex_bt_observability_execution_plan.md`（已实现的基础方案必须先完成）
> **本文档范围**：仅描述录制侧的增量改动，不涉及回放侧工具

---

## 0. 变更总览

| 文件 | 操作 | 核心变化 |
|------|------|---------|
| `bt_observability/bt_debug_visitor.py` | **替换** | 增加 BB write flush；拓扑快照触发改为 root status 变化时 |
| `bt_observability/debug_event_logger.py` | **扩展** | 增加 `snapshot_ros_topology`；增加 rosbag 后台录制 |
| `marathon/behaviours/conditions.py` | **修改** | Subscriber 回调里 emit `ros_topic_received` 事件 |
| `marathon/marathon_bt_node.py` | **修改** | 传入 `rosbag_topics` / `rosbag_dir` 参数 |
| `marathon/log/rosbag/` | **新建目录** | 运行时自动创建，加入 `.gitignore` |

---

## 1. 新增事件类型定义

完善后，comm log 包含以下事件类型：

```
event 字段值                   方向                    触发时机
────────────────────────────────────────────────────────────────
ros_topology_snapshot          系统级快照              root status 变化时（低频）
ros_comm (topic_publish)       BT → ROS node           BT action update() 内同步 pub
ros_comm (service_call)        BT → ROS node           BT action update() 内同步 call
ros_comm_result (service)      ROS node → BT           service_call 返回时
ros_topic_received             外部 ROS node → BT      Subscriber 回调触发时
bb_write                       BT 内部（BB 写入）       tick 结束时统一 flush
```

**关于 `ros_node` 字段的设计决策：**
- `ros_topic_received` 事件的 `ros_node` 固定为 `null`
  — ROS1 subscriber 回调不提供发布方信息
- LLM 可从最近的 `ros_topology_snapshot.node_details` 中查询该 topic 的发布方
- BB 写入（`bb_write`）以 BB 实际变化的 tick 为准，
  比 subscriber 接收原始消息晚约 1 tick，这是设计选择而非缺陷

---

## 2. 文件一：`bt_debug_visitor.py` — 完整替换

**替换整个文件**（不是增量修改）。

```python
#!/usr/bin/env python3
"""
BTDebugVisitor: py_trees visitor for BT decision-layer observability.

Records:
  - tree_tick_start / tree_tick_end       via pre/post tick handlers
  - per-node tick_end                     via run()
  - bb_write events per tick              via activity_stream flush
  - ros_topology_snapshot                 when root status changes
"""
import py_trees


class BTDebugVisitor(py_trees.visitors.VisitorBase):

    def __init__(self, logger, tick_id_getter):
        super().__init__(full=False)   # full=False: 只 visit 实际被 tick 的节点
        self._logger = logger
        self._tick_id_getter = tick_id_getter
        self._last_root_status = None

        # 启用 py_trees 内置 blackboard activity stream
        # maximum_size 须大于单 tick 内最多 BB 操作数
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=500)

    # ── tree-level hooks ──────────────────────────────────────────────────────

    def on_tree_tick_start(self, tree):
        """Assigned to tree.pre_tick_handler."""
        tick_id = self._tick_id_getter()
        self._logger.begin_tick(tick_id)
        self._logger.emit_bt({
            "event": "tree_tick_start",
            "tick_id": tick_id,
        })

    def on_tree_tick_end(self, tree):
        """Assigned to tree.post_tick_handler."""
        tick_id = self._tick_id_getter()
        current_status = str(tree.root.status)

        # 1. tree_tick_end
        self._logger.emit_bt({
            "event": "tree_tick_end",
            "tick_id": tick_id,
            "status": current_status,
        })

        # 2. flush BB write events（本 tick 所有 BB 写入）
        self._flush_blackboard_writes(tick_id)

        # 3. root status 变化 → 触发拓扑快照（低频，仅状态切换时）
        if current_status != self._last_root_status:
            self._logger.snapshot_ros_topology(tick_id)
            self._last_root_status = current_status

        # 4. 清空 activity stream，准备下一 tick
        py_trees.blackboard.Blackboard.activity_stream.clear()

        self._logger.end_tick(tick_id)

    # ── per-node hook ─────────────────────────────────────────────────────────

    def run(self, behaviour):
        """Called by py_trees after behaviour.update() completes."""
        self._logger.emit_bt({
            "event": "tick_end",
            "tick_id": self._tick_id_getter(),
            "node": behaviour.name,
            "type": type(behaviour).__name__,
            "status": str(behaviour.status),
        })

    # ── internal ──────────────────────────────────────────────────────────────

    def _flush_blackboard_writes(self, tick_id: int):
        """
        读取本 tick 内 Blackboard 的所有 WRITE 活动并 emit bb_write 事件。

        设计说明：
          - 使用 py_trees 内置 activity_stream，无需修改任何 BT 节点
          - 仅记录 WRITE，READ 不记录（避免噪音）
          - 若 BB 值未变化，py_trees 不产生 WRITE activity，天然去重
          - Subscriber 回调写 BB 发生在本 tick 的 update() 内，
            bb_write 的 tick_id 是"BB 实际被写入的 tick"，
            比 subscriber 接收原始消息晚约 1 tick，以 BB 变化 tick 为准
        """
        stream = py_trees.blackboard.Blackboard.activity_stream
        if not stream or not stream.data:
            return

        for activity in stream.data:
            # ActivityType 在不同 py_trees 版本名称略有不同
            act_type = str(activity.activity_type)
            if "WRITE" not in act_type.upper():
                continue

            self._logger.emit_bt({
                "event":  "bb_write",
                "tick_id": tick_id,
                "writer": getattr(activity, "client_name", "unknown"),
                "key":    getattr(activity, "variable_name", "unknown"),
                "value":  self._safe_repr(getattr(activity, "value", None)),
            })

    @staticmethod
    def _safe_repr(value):
        """将 BB 值转为可 JSON 序列化的对象。"""
        if value is None:
            return None
        if isinstance(value, (bool, int, float, str)):
            return value
        if hasattr(value, "__slots__"):          # ROS message 对象
            from bt_observability.ros_comm_tracer import ROSCommTracer
            return ROSCommTracer._msg_to_dict(value)
        return str(value)
```

---

## 3. 文件二：`debug_event_logger.py` — 增量修改

在现有类上**追加**以下内容，`__init__` 追加两个可选参数。

### 3.1 `__init__` 新增参数（追加到参数列表末尾）

```python
def __init__(
    self,
    bt_topic="/bt_debug",
    comm_topic="/bt_ros_comm_debug",
    bt_lastrun_jsonl="bt_debug_lastrun.jsonl",
    comm_lastrun_jsonl="bt_ros_comm_debug_lastrun.jsonl",
    rolling_bt_jsonl="bt_debug_recent.jsonl",
    rolling_comm_jsonl="bt_ros_comm_debug_recent.jsonl",
    max_rolling_ticks=100,
    rosbag_topics=None,    # list[str] | None，为 None 时不启动 rosbag
    rosbag_dir=None,       # str | None，rosbag 文件存放目录
):
    # ... 原有初始化代码不变 ...

    # rosbag 后台录制（新增）
    self._rosbag_proc = None
    if rosbag_topics and rosbag_dir:
        self._start_rosbag(rosbag_topics, rosbag_dir)
```

### 3.2 新增方法（追加到类末尾）

```python
import subprocess
import os
import signal

# ── rosbag 录制 ────────────────────────────────────────────────────────────────

def _start_rosbag(self, topics: list, rosbag_dir: str):
    """
    后台启动 rosbag record，录制指定 topic 列表。

    用途：
      - 全量录制 BT 已知会用到的 topic（含 action /goal /feedback /result）
      - 覆盖 topic publish 对端响应 + ROS node 间内部通信（BT 视角不可见部分）
      - LLM 通过回放侧工具（后续实现）查询具体消息内容

    文件命名：rosbag_dir/bt_session_<timestamp>.bag
    """
    os.makedirs(rosbag_dir, exist_ok=True)
    cmd = (
        ["rosbag", "record",
         "--output-prefix", os.path.join(rosbag_dir, "bt_session")]
        + topics
    )
    self._rosbag_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,   # 独立进程组，close() 时可整组 kill
    )
    import rospy
    rospy.loginfo(f"[DebugEventLogger] rosbag started: {topics}")

def _stop_rosbag(self):
    if self._rosbag_proc is not None:
        try:
            os.killpg(os.getpgid(self._rosbag_proc.pid), signal.SIGINT)
            self._rosbag_proc.wait(timeout=5)
        except Exception:
            pass
        self._rosbag_proc = None

# ── 拓扑快照 ───────────────────────────────────────────────────────────────────

def snapshot_ros_topology(self, tick_id: int):
    """
    采集完整 ROS 系统拓扑快照，写入 comm log。

    内容：
      node_details   : {node_name: {publications, subscriptions, services}}
      topic_details  : {topic_name: type_string}
      service_details: {service_name: type_string}

    触发时机：由 BTDebugVisitor 在 root status 变化时调用，非周期性。

    耗时说明：
      rosnode info 对每个 node 各调用一次，节点多时可能耗时 1~3 秒。
      因触发频率低（仅状态变化时），对 30Hz tick loop 影响可接受。
      若实测影响 tick 时序，可改为异步线程执行（后续优化项）。
    """
    import rospy
    snapshot = {
        "event":   "ros_topology_snapshot",
        "tick_id": tick_id,
        "ts":      rospy.Time.now().to_sec(),
    }

    # 1. node 列表 + 每个 node 的 pub/sub/service 详情
    node_names = self._run_cmd("rosnode list")
    node_details = {}
    for node in node_names:
        if not node.startswith("/"):
            continue
        info_lines = self._run_cmd(f"rosnode info {node}")
        node_details[node] = self._parse_rosnode_info(info_lines)
    snapshot["node_details"] = node_details

    # 2. topic 列表 + 每个 topic 的消息类型
    topic_names = self._run_cmd("rostopic list")
    topic_details = {}
    for t in topic_names:
        if not t.startswith("/"):
            continue
        t_type = self._run_cmd(f"rostopic type {t}")
        topic_details[t] = t_type[0] if t_type else "unknown"
    snapshot["topic_details"] = topic_details

    # 3. service 列表 + 每个 service 的类型
    svc_names = self._run_cmd("rosservice list")
    svc_details = {}
    for s in svc_names:
        if not s.startswith("/"):
            continue
        s_type = self._run_cmd(f"rosservice type {s}")
        svc_details[s] = s_type[0] if s_type else "unknown"
    snapshot["service_details"] = svc_details

    self.emit_comm(snapshot)

# ── close 替换（替换原有 close 方法）──────────────────────────────────────────

def close(self):
    """关闭文件句柄并停止 rosbag 录制。"""
    self._bt_file.close()
    self._comm_file.close()
    self._stop_rosbag()

# ── 工具函数（追加）──────────────────────────────────────────────────────────

@staticmethod
def _run_cmd(cmd: str, timeout: int = 3) -> list:
    """执行 shell 命令，返回非空行列表。失败返回 ["error: ..."]。"""
    try:
        out = subprocess.check_output(
            cmd.split(), timeout=timeout, stderr=subprocess.DEVNULL
        )
        return [l for l in out.decode().strip().split("\n") if l.strip()]
    except Exception as e:
        return [f"error: {e}"]

@staticmethod
def _parse_rosnode_info(lines: list) -> dict:
    """
    解析 rosnode info 输出，提取 pub/sub/service 三个 section。

    rosnode info 输出格式：
      Publications:
       * /topic_name [pkg/MsgType]
      Subscriptions:
       * /topic_name [pkg/MsgType]
      Services:
       * /service_name

    返回：
      {
        "publications":  [{"name": "/topic", "type": "pkg/Msg"}, ...],
        "subscriptions": [{"name": "/topic", "type": "pkg/Msg"}, ...],
        "services":      [{"name": "/service"}, ...],
      }
    """
    result = {"publications": [], "subscriptions": [], "services": []}
    section = None
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("Publications:"):
            section = "publications"
        elif stripped.startswith("Subscriptions:"):
            section = "subscriptions"
        elif stripped.startswith("Services:"):
            section = "services"
        elif stripped.startswith("*") and section:
            parts = stripped.lstrip("* ").split(" ", 1)
            entry = {"name": parts[0]}
            if len(parts) > 1:
                entry["type"] = parts[1].strip("[]")
            result[section].append(entry)
    return result
```

---

## 4. 文件三：`conditions.py` — Subscriber 回调增量修改

**原则：** 每个 Condition 节点在 Subscriber 回调里增加一次 `emit_comm`，
记录外部 ROS → BT 方向的原始消息。

**`__init__` 修改：** 新增 `tick_id_getter` 参数（原来只有 `logger`）：

```python
def __init__(self, name, logger=None, tick_id_getter=None):
    super().__init__(name=name)
    self._logger = logger
    self._tick_id_getter = tick_id_getter or (lambda: -1)
```

**`marathon_bt.py` 同步修改：** 传入 `tick_id_getter`：

```python
IsRobotStanding("IsRobotStanding", logger=logger, tick_id_getter=tick_id_getter)
IsLineDetected("IsLineDetected",   logger=logger, tick_id_getter=tick_id_getter)
```

**回调模板（以 `IsRobotStanding` 为例）：**

```python
import threading
from bt_observability.ros_comm_tracer import ROSCommTracer

class IsRobotStanding(py_trees.behaviour.Behaviour):

    def __init__(self, name, logger=None, tick_id_getter=None):
        super().__init__(name=name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._imu_cache = None
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        self._sub = rospy.Subscriber(
            "/imu/data", Imu, self._imu_cb, queue_size=1
        )

    def _imu_cb(self, msg):
        # 记录外部 → BT 方向的原始消息
        if self._logger:
            self._logger.emit_comm({
                "event":     "ros_topic_received",
                "direction": "subscribe",        # 外部 ROS node → BT node
                "target":    "/imu/data",        # 订阅的 topic 名
                "ros_node":  None,               # ROS1 不提供发布方信息
                                                 # 可查 ros_topology_snapshot
                                                 # 的 node_details 找发布方
                "node":      self.name,          # 哪个 BT node 在订阅
                "tick_id":   self._tick_id_getter(),
                "payload":   ROSCommTracer._msg_to_dict(msg),
            })
        with self._lock:
            self._imu_cache = msg

    def update(self):
        with self._lock:
            cache = self._imu_cache
        # 读缓存 → 写 BB
        # BTDebugVisitor 在 tick 结束时从 activity_stream 捞到 bb_write 事件
        ...
```

**`IsLineDetected` 同理**，回调里填写对应的 `target`（如 `/line_camera/result`）。

---

## 5. 文件四：`marathon_bt_node.py` — 初始化修改

在 `DebugEventLogger` 初始化前定义 `ROSBAG_TOPICS`，并传入新参数：

```python
# ── rosbag 录制 topic 清单 ────────────────────────────────────────────────────
# 包含：BT 已知会用到的所有 topic + action 的 /goal /feedback /result
# 目的：覆盖 topic publish 对端响应 + ROS node 间内部通信（BT 视角不可见部分）
ROSBAG_TOPICS = [
    "/walking/set_param",
    "/walking/command",
    "/ros_robot_controller/bus_servo/set_position",
    "/ros_robot_controller/set_buzzer",
    "/imu/data",
    # action 示例（按实际接口补充）：
    # "/some_action/goal",
    # "/some_action/feedback",
    # "/some_action/result",
]

self._obs_logger = DebugEventLogger(
    bt_topic="/bt_debug",
    comm_topic="/bt_ros_comm_debug",
    bt_lastrun_jsonl=rospy.get_param(
        '~bt_jsonl', os.path.join(_LOG_DIR, 'bt_debug_lastrun.jsonl')),
    comm_lastrun_jsonl=rospy.get_param(
        '~comm_jsonl', os.path.join(_LOG_DIR, 'bt_ros_comm_debug_lastrun.jsonl')),
    rolling_bt_jsonl=os.path.join(_LOG_DIR, 'bt_debug_recent.jsonl'),
    rolling_comm_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_recent.jsonl'),
    max_rolling_ticks=rospy.get_param('~max_rolling_ticks', 100),
    # ── 新增 ────────────────────────────────────────────────────────────────
    rosbag_topics=ROSBAG_TOPICS,
    rosbag_dir=os.path.join(_LOG_DIR, 'rosbag'),
)
```

---

## 6. 完整数据流示意

```
外部 ROS node
  │  publish /imu/data
  ▼
IsRobotStanding._imu_cb()
  │  emit_comm: ros_topic_received  target=/imu/data  ros_node=null
  │             tick_id=N  payload={orientation: ...}
  │  写 self._imu_cache
  ▼
IsRobotStanding.update()  [tick N 或 N+1]
  │  读 cache → 写 Blackboard["is_standing"] = True
  ▼
BTDebugVisitor.on_tree_tick_end()
  │  flush activity_stream
  │  emit_bt: bb_write  tick_id=N  writer=IsRobotStanding
  │           key=is_standing  value=True
  │
  │  若 root status 变化（如 RUNNING → SUCCESS）：
  │  emit_comm: ros_topology_snapshot  tick_id=N
  │             node_details={/ainex_controller: {publications:[...], ...}, ...}
  │             topic_details={/imu/data: "sensor_msgs/Imu", ...}
  ▼
StopWalking.update()  [同 tick]
  │  proxy_context.current_node = "StopWalking"
  │  gait_manager.disable()  → ManagerProxy 拦截
  │  emit_comm: ros_comm  comm_type=service_call  target=/walking/command
  │             ros_node=ainex_controller  node=StopWalking  tick_id=N
  │             payload={command: disable}
  │  service 返回
  │  emit_comm: ros_comm_result  target=/walking/command  result={success: true}
  ▼
rosbag（后台持续录制）
  ← /walking/command  request + response
  ← /imu/data         每条原始消息
  ← 所有 ROSBAG_TOPICS 的完整消息流
```

---

## 7. JSONL 新增事件示例

**BB 写入事件：**
```jsonl
{"event":"bb_write","tick_id":42,"writer":"IsRobotStanding","key":"is_standing","value":true,"ts":...}
{"event":"bb_write","tick_id":42,"writer":"IsLineDetected","key":"line_data","value":{"x":82,"width":640},"ts":...}
```

**Subscriber 接收事件：**
```jsonl
{"event":"ros_topic_received","direction":"subscribe","target":"/imu/data","ros_node":null,
 "node":"IsRobotStanding","tick_id":42,"payload":{"orientation":{"x":0.0,"y":0.01,"z":0.0,"w":1.0},...},"ts":...}
```

**拓扑快照（root status 变化时，格式简化示意）：**
```jsonl
{"event":"ros_topology_snapshot","tick_id":150,
 "node_details":{
   "/ainex_controller":{
     "publications":[{"name":"/walking/state","type":"ainex_interfaces/WalkingState"}],
     "subscriptions":[{"name":"/walking/set_param","type":"ainex_interfaces/WalkingParam"}],
     "services":[{"name":"/walking/command"}]
   }
 },
 "topic_details":{"/imu/data":"sensor_msgs/Imu","/walking/set_param":"ainex_interfaces/WalkingParam"},
 "service_details":{"/walking/command":"ainex_interfaces/WalkingCommand"},
 "ts":...}
```

---

## 8. LLM 可回答的问题边界（录制完成后）

| 问题 | 数据来源 | 可否回答 |
|------|---------|---------|
| 第 N tick BT 做了什么决策 | bt_debug JSONL | ✅ |
| 哪个 BT node 发出了哪个 ROS 调用，内容是什么 | comm JSONL `ros_comm` | ✅ |
| service call 的返回值 | comm JSONL `ros_comm_result` | ✅ |
| 外部 topic 发给 BT 的原始内容 | comm JSONL `ros_topic_received` | ✅ |
| BB 在哪个 tick 被哪个节点写入了什么值 | bt JSONL `bb_write` | ✅ |
| 系统里有哪些 ROS node，各有什么 pub/sub/service | comm JSONL `ros_topology_snapshot` | ✅ |
| 某 topic 的发布方是哪个 node | `ros_topology_snapshot.node_details` | ✅ |
| topic publish 对端是否收到 | rosbag（消息存在即投递）| ⚠️ 推断 |
| action feedback/result 详情 | rosbag | ✅（需回放侧工具）|
| ROS node 间（BT 之外）的通信内容 | rosbag | ✅（需回放侧工具）|

---

## 9. 实施顺序

1. **替换** `bt_debug_visitor.py`（完整替换，见第 2 节）
2. **扩展** `debug_event_logger.py`（追加方法 + 修改 `__init__` + 替换 `close`，见第 3 节）
3. **修改** `conditions.py`（各 Condition 节点加 `tick_id_getter` 参数 + 回调 emit，见第 4 节）
4. **修改** `marathon_bt.py`（`bootstrap()` 传入 `tick_id_getter` 给 Condition 节点）
5. **修改** `marathon_bt_node.py`（加 `ROSBAG_TOPICS` + 传入新参数，见第 5 节）
6. **验证**：
   ```bash
   # 运行 marathon BT，检查 recent JSONL 中出现 bb_write 事件
   grep '"event":"bb_write"' marathon/log/bt_debug_recent.jsonl

   # 检查 ros_topic_received 事件
   grep '"event":"ros_topic_received"' marathon/log/bt_ros_comm_debug_recent.jsonl

   # 触发 root status 变化（如跌倒），检查拓扑快照
   grep '"event":"ros_topology_snapshot"' marathon/log/bt_ros_comm_debug_lastrun.jsonl

   # 检查 rosbag 文件生成
   ls marathon/log/rosbag/
   ```
