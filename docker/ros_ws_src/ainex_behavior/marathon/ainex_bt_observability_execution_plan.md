# Ainex BT 可观测性系统 — 技术执行方案
## 基于 py_trees + ManagerProxy + ROSCommTracer 的全链路 BT 决策与通信追踪

> **适用范围：** `ainex_behavior` 包下所有 BT 项目（marathon 为首个落地项目，后续可扩展至任意新建 BT task）
>
> **依赖的设计文档：**
> - `bt_debug_topic_technical_report.md` — Debug Topic 与装饰器设计
> - `bt_ros_observability_plan.md` — BT + ROS 双层可观测性架构
> - `ros_comm_tracer_execution_plan.md` — ROSCommTracer 实现规范
>
> **实施状态：** 全部 Phase 已完成（Phase 1-3 基础方案 + Phase 4 录制侧完善）

---

## 0. 背景与现状分析

### 0.1 当前项目结构（marathon 为例）

```
ainex_behavior/
├── marathon/
│   ├── marathon_bt_node.py      # ROS 节点入口，15Hz tick loop
│   ├── marathon_bt.py           # BT 工厂函数 bootstrap()
│   └── behaviours/
│       ├── conditions.py        # IsRobotStanding, IsLineDetected
│       └── actions.py           # StopWalking, FollowLine, RecoverFromFall
```

### 0.2 现有 ROS 通信汇总（marathon）

| BT 节点 | Python 封装调用 | 实际 ROS 接口 | 接口类型 | 目标 ROS 节点 |
|---------|---------------|-------------|---------|-------------|
| `RecoverFromFall` | 直接 pub | topic `/ros_robot_controller/set_buzzer` | topic publish | `ros_robot_controller` |
| `RecoverFromFall` | `gait_manager.disable()` | service `/walking/command` | service call | `ainex_controller` |
| `RecoverFromFall` | `motion_manager.run_action()` | topic `ros_robot_controller/bus_servo/set_position` | topic publish | `ros_robot_controller` |
| `StopWalking` | `gait_manager.disable()` | service `/walking/command` | service call | `ainex_controller` |
| `FollowLine` | `visual_patrol.process()` → `gait_manager.set_step()` | topic `walking/set_param` | topic publish | `ainex_controller` |

### 0.3 关键框架差异（py_trees vs 自定义 BT）

原设计文档基于自定义 `tick()` 方法。py_trees 的对应关系：

| 原设计 | py_trees 对应 |
|--------|-------------|
| `tick()` 方法 | `update()` 方法 |
| `@trace_tick` 装饰器 | `BTDebugVisitor`（Visitor 模式）|
| Tree Runner 的 tick 计数 | 自增 `self._tick_id` |
| 树级 tick_start/end | `tree.pre_tick_handlers` / `post_tick_handlers` |

py_trees 的 Visitor 模式可以在**不修改任何现有节点**的情况下捕获每个节点的执行状态。

### 0.4 封装层透明性问题与解决方案

`gait_manager` / `motion_manager` / `visual_patrol` 是 Python 封装类，直接调用无法在日志中看到实际 ROS 接口名（topic/service 名、目标节点名）。

**解决方案：** `ManagerProxy` — 通用透明代理，用 `__getattr__` 动态拦截，配合 `ros_map` 配置表将 Python 方法调用翻译为真实 ROS 接口事件记录，且对被代理对象和调用方**完全透明**。

---

## 1. 目标

### 层一：BT 决策层可观测性
- 记录每次 tick 中哪些节点被执行、执行结果（SUCCESS/FAILURE/RUNNING）
- 记录 Condition 节点的判定输入值与原因
- 记录 Blackboard 写入事件（bb_write），含写入方、key、值，同一 tick 内连续相同值自动压缩
- 发布到 `/bt_debug` topic + 写入 `bt_debug_lastrun.jsonl`（全量）/ `bt_debug_recent.jsonl`（滚动）

### 层二：ROS 通信层可观测性
- 记录 Action 节点向 ROS 系统发出的具体命令
- 记录外部 ROS topic 首次到达 BT 的时间（`ros_topic_received`，每 tick 每 topic 仅记录一次）
- 记录 root status 变化时的完整 ROS 系统拓扑快照（异步，不阻塞 tick loop）
- **清晰区分** topic publish / service call / action goal，并记录目标 ROS 节点名
- 发布到 `/bt_ros_comm_debug` topic + 写入 `bt_ros_comm_debug_lastrun.jsonl`（全量）/ `bt_ros_comm_debug_recent.jsonl`（滚动）

### 层三：旁路全量录制（rosbag）
- 后台录制所有 BT 相关 topic，覆盖 BT 视角不可见的对端响应
- 60s 一个 bag 文件，保留最近 30 分钟（30 个文件），自动滚动删除

### 全链路覆盖

```
tick_id
  → 哪些节点执行了（Visitor）
  → 为什么执行（Condition 决策输入 + bb_write）
  → 发出了什么 ROS 命令（ROSCommTracer + ManagerProxy）
      → 具体 topic/service 名、消息内容、目标 ROS 节点
  → 外部收到了什么（ros_topic_received，每 tick 每 topic 首次）
  → 系统拓扑（ros_topology_snapshot，root status 变化时异步采集）
  → 完整消息流（rosbag，60s 滚动，30min 保留）
```

---

## 2. 新增模块结构

```
ainex_behavior/
├── bt_observability/                  ← 共享模块
│   ├── __init__.py
│   ├── debug_event_logger.py          ← DebugEventLogger（统一输出 + rosbag）
│   ├── bt_debug_visitor.py            ← py_trees Visitor（决策层 + bb_write）
│   └── ros_comm_tracer.py             ← ROSCommTracer + ProxyContext + ManagerProxy
│
├── marathon/
│   ├── marathon_bt_node.py            ← 修改：注入 logger + visitor + ManagerProxy + rosbag
│   ├── marathon_bt.py                 ← 修改：传递 logger + tick_id_getter 到所有节点
│   ├── behaviours/
│   │   ├── conditions.py              ← 修改：emit decision 事件；tick_id_getter 参数
│   │   ├── actions.py                 ← 修改：proxy_context + ROSCommTracer（仅 buzzer）
│   │   └── visual_patrol.py           ← 新建：从 ainex_example 复制（不依赖外部路径）
│   └── log/                           ← 日志落盘目录（marathon/log/ 已加入 .gitignore）
│       ├── bt_debug_lastrun.jsonl     ← 全量，newest-first，启动清空
│       ├── bt_ros_comm_debug_lastrun.jsonl
│       ├── bt_debug_recent.jsonl      ← 最近 30 tick，newest-first，原子覆写
│       ├── bt_ros_comm_debug_recent.jsonl
│       └── rosbag/                    ← 60s 分片 bag 文件，保留最近 30 分钟
│           └── bt_session_<ts>.bag
```

**两层 JSONL 存储说明：**

| 文件 | 写入方式 | 内容 | 排列顺序 | 用途 |
|------|---------|------|---------|------|
| `bt_debug_lastrun.jsonl` | 全量内存缓冲，每 tick 原子覆写 | 本次 roscore 全量 | newest-first | 完整复盘 |
| `bt_ros_comm_debug_lastrun.jsonl` | 同上 | 本次 roscore 全量 | newest-first | 完整复盘 |
| `bt_debug_recent.jsonl` | 滚动 deque，每 tick 原子覆写 | 最近 30 tick | newest-first | ROSA/LLM 上下文 |
| `bt_ros_comm_debug_recent.jsonl` | 同上 | 最近 30 tick | newest-first | ROSA/LLM 上下文 |

**注意：** 所有文件均为 newest-first（最高 tick_id 在最前）。ROSA 工具读取 lastrun 时需先反转行序再送入摘要函数（摘要函数假设 oldest-first 输入）。

---

## 3. 核心组件实现

### 3.1 `debug_event_logger.py` — 统一输出层

```python
#!/usr/bin/env python3
"""
DebugEventLogger: publish ROS topic + write two-tier JSONL storage.

Two-tier storage:
  last-run  — fixed filename, cleared on each roscore start, all ticks buffered
              in memory and atomically rewritten newest-first on each tick_end.
  rolling   — fixed filename, last N ticks only, atomically rewritten
              on each tick_end. For ROSA/LLM context window.

Both files are always newest-first (highest tick_id at top).
"""
import os
import json
import signal
import subprocess
import collections
import rospy
from std_msgs.msg import String


class DebugEventLogger:

    def __init__(
        self,
        bt_topic="/bt_debug",
        comm_topic="/bt_ros_comm_debug",
        bt_lastrun_jsonl="bt_debug_lastrun.jsonl",
        comm_lastrun_jsonl="bt_ros_comm_debug_lastrun.jsonl",
        rolling_bt_jsonl="bt_debug_recent.jsonl",
        rolling_comm_jsonl="bt_ros_comm_debug_recent.jsonl",
        max_rolling_ticks=30,
        tick_id_getter=None,
        rosbag_topics=None,    # list[str] | None — None disables rosbag
        rosbag_dir=None,       # str | None — directory for rosbag files
    ):
        self._tick_id_getter = tick_id_getter or (lambda: None)
        self._bt_pub = rospy.Publisher(bt_topic, String, queue_size=100)
        self._comm_pub = rospy.Publisher(comm_topic, String, queue_size=100)

        # last-run: cleared on startup, all ticks buffered in memory,
        # atomically rewritten newest-first on each tick_end
        self._lastrun_bt_path = bt_lastrun_jsonl
        self._lastrun_comm_path = comm_lastrun_jsonl
        self._bt_all_ticks = []    # list of tick line-lists, oldest first
        self._comm_all_ticks = []
        open(bt_lastrun_jsonl, "w").close()
        open(comm_lastrun_jsonl, "w").close()

        # rolling: fixed path, atomically rewritten each tick
        self._rolling_bt_path = rolling_bt_jsonl
        self._rolling_comm_path = rolling_comm_jsonl
        self._bt_rolling = collections.deque(maxlen=max_rolling_ticks)
        self._comm_rolling = collections.deque(maxlen=max_rolling_ticks)

        # per-tick buffer (cleared by begin_tick each tick)
        self._bt_tick_buf = []
        self._comm_tick_buf = []

        # rosbag background recording
        self._rosbag_proc = None
        if rosbag_topics and rosbag_dir:
            self._start_rosbag(rosbag_topics, rosbag_dir)

    # --- Public API ---

    def emit_bt(self, payload: dict):
        payload.setdefault("tick_id", self._tick_id_getter())
        payload.setdefault("ts", rospy.Time.now().to_sec())
        line = json.dumps(payload, ensure_ascii=False)
        self._bt_pub.publish(line)
        self._bt_tick_buf.append(line)

    def emit_comm(self, payload: dict):
        payload.setdefault("tick_id", self._tick_id_getter())
        payload.setdefault("ts", rospy.Time.now().to_sec())
        line = json.dumps(payload, ensure_ascii=False)
        self._comm_pub.publish(line)
        self._comm_tick_buf.append(line)

    def begin_tick(self, tick_id: int):
        """Called by BTDebugVisitor.on_tree_tick_start(). Clears per-tick buffers."""
        self._bt_tick_buf = []
        self._comm_tick_buf = []

    def end_tick(self, tick_id: int):
        """Called by BTDebugVisitor.on_tree_tick_end(). Pushes to buffers and flushes."""
        if self._bt_tick_buf:
            self._bt_rolling.append(list(self._bt_tick_buf))
            self._bt_all_ticks.append(list(self._bt_tick_buf))
        if self._comm_tick_buf:
            self._comm_rolling.append(list(self._comm_tick_buf))
            self._comm_all_ticks.append(list(self._comm_tick_buf))
        self._flush_rolling()
        self._flush_lastrun()

    def close(self):
        """Stop rosbag recording. (JSONL files already flushed by last end_tick.)"""
        self._stop_rosbag()

    # --- rosbag recording ---

    def _start_rosbag(self, topics: list, rosbag_dir: str):
        """
        Start rosbag record in background.

        Split strategy:
          --split --duration 60   one .bag file per 60 s
          --max-splits 30         keep only the last 30 files (= 30 min rolling window)
                                  rosbag deletes the oldest split automatically

        File naming: rosbag_dir/bt_session_<timestamp>.bag
        Uses a separate process group so close() can kill the whole group via SIGINT.
        """
        os.makedirs(rosbag_dir, exist_ok=True)
        cmd = [
            "rosbag", "record",
            "--split",
            "--duration", "60",
            "--max-splits", "30",
            "--output-prefix", os.path.join(rosbag_dir, "bt_session"),
        ] + topics
        self._rosbag_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        rospy.loginfo("[DebugEventLogger] rosbag started (60s splits, 30min window): %s", topics)

    def _stop_rosbag(self):
        if self._rosbag_proc is not None:
            try:
                os.killpg(os.getpgid(self._rosbag_proc.pid), signal.SIGINT)
                self._rosbag_proc.wait(timeout=5)
            except Exception:
                pass
            self._rosbag_proc = None

    # --- ROS topology snapshot ---

    def snapshot_ros_topology(self, tick_id: int):
        """
        Collect full ROS system topology snapshot, write to comm log.

        Called from BTDebugVisitor via a daemon thread when root status changes.
        Runs synchronously inside that thread — rosnode info over all nodes
        can take 5-10 s, hence the async dispatch.

        Content:
          node_details   : {node_name: {publications, subscriptions, services}}
          topic_details  : {topic_name: type_string}
          service_details: {service_name: type_string}
        """
        snapshot = {
            "event":   "ros_topology_snapshot",
            "tick_id": tick_id,
            "ts":      rospy.Time.now().to_sec(),
        }
        node_names = self._run_cmd("rosnode list")
        node_details = {}
        for node in node_names:
            if not node.startswith("/"):
                continue
            info_lines = self._run_cmd("rosnode info {}".format(node))
            node_details[node] = self._parse_rosnode_info(info_lines)
        snapshot["node_details"] = node_details

        topic_names = self._run_cmd("rostopic list")
        topic_details = {}
        for t in topic_names:
            if not t.startswith("/"):
                continue
            t_type = self._run_cmd("rostopic type {}".format(t))
            topic_details[t] = t_type[0] if t_type else "unknown"
        snapshot["topic_details"] = topic_details

        svc_names = self._run_cmd("rosservice list")
        svc_details = {}
        for s in svc_names:
            if not s.startswith("/"):
                continue
            s_type = self._run_cmd("rosservice type {}".format(s))
            svc_details[s] = s_type[0] if s_type else "unknown"
        snapshot["service_details"] = svc_details

        self.emit_comm(snapshot)

    # --- Internal ---

    def _flush_rolling(self):
        self._atomic_write(
            self._rolling_bt_path,
            [line for tick in reversed(self._bt_rolling) for line in tick],
        )
        self._atomic_write(
            self._rolling_comm_path,
            [line for tick in reversed(self._comm_rolling) for line in tick],
        )

    def _flush_lastrun(self):
        self._atomic_write(
            self._lastrun_bt_path,
            [line for tick in reversed(self._bt_all_ticks) for line in tick],
        )
        self._atomic_write(
            self._lastrun_comm_path,
            [line for tick in reversed(self._comm_all_ticks) for line in tick],
        )

    @staticmethod
    def _atomic_write(path, lines):
        """Write to tmp then os.replace — prevents ROSA reading a partial file."""
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            if lines:
                f.write("\n".join(lines) + "\n")
        os.replace(tmp, path)

    @staticmethod
    def _run_cmd(cmd: str, timeout: int = 3) -> list:
        """Run shell command, return non-empty lines. Returns ['error: ...'] on failure."""
        try:
            out = subprocess.check_output(
                cmd.split(), timeout=timeout, stderr=subprocess.DEVNULL
            )
            return [l for l in out.decode().strip().split("\n") if l.strip()]
        except Exception as e:
            return ["error: {}".format(e)]

    @staticmethod
    def _parse_rosnode_info(lines: list) -> dict:
        """
        Parse rosnode info output, extracting pub/sub/service sections.

        Returns:
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

### 3.2 `bt_debug_visitor.py` — py_trees 决策层 Visitor

py_trees 2.x 的 Visitor 在每个节点 `update()` 执行后自动被调用，无需修改节点代码即可捕获执行状态。

**py_trees 2.1.6 ActivityItem 实际字段名（通过实测验证）：**

| 字段 | 含义 |
|------|------|
| `client_name` | 写入方 BB Client 名 |
| `key` | BB key（含前缀斜线，如 `/line_data`）|
| `current_value` | 写入后的值 |
| `previous_value` | 写入前的值 |
| `activity_type` | `'WRITE'`（更新）或 `'INITIALISED'`（首次写入）|

```python
#!/usr/bin/env python3
"""
BTDebugVisitor: py_trees visitor for BT decision-layer observability.

Records:
  - tree_tick_start / tree_tick_end       via pre/post tick handlers
  - per-node tick_end                     via run()
  - bb_write events per tick              via activity_stream flush
  - ros_topology_snapshot                 when root status changes (daemon thread)
"""
import threading
import py_trees


class BTDebugVisitor(py_trees.visitors.VisitorBase):

    def __init__(self, logger, tick_id_getter):
        super().__init__(full=False)   # full=False: only visit actually-ticked nodes
        self._logger = logger
        self._tick_id_getter = tick_id_getter
        self._last_root_status = None

        # Enable py_trees built-in blackboard activity stream.
        # maximum_size=500 must exceed max BB ops per tick.
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=500)

    # ── tree-level hooks ──────────────────────────────────────────────────────

    def on_tree_tick_start(self, tree):
        """Assigned to tree.pre_tick_handlers."""
        tick_id = self._tick_id_getter()
        self._logger.begin_tick(tick_id)
        self._logger.emit_bt({
            "event": "tree_tick_start",
            "tick_id": tick_id,
        })

    def on_tree_tick_end(self, tree):
        """Assigned to tree.post_tick_handlers."""
        tick_id = self._tick_id_getter()
        current_status = str(tree.root.status)

        # 1. tree_tick_end
        self._logger.emit_bt({
            "event": "tree_tick_end",
            "tick_id": tick_id,
            "status": current_status,
        })

        # 2. flush BB write events (all writes this tick)
        self._flush_blackboard_writes(tick_id)

        # 3. root status changed → topology snapshot in daemon thread
        #    Running async: rosnode info over all nodes takes 5-10 s synchronously.
        if current_status != self._last_root_status:
            t = threading.Thread(
                target=self._logger.snapshot_ros_topology,
                args=(tick_id,),
                daemon=True,
            )
            t.start()
            self._last_root_status = current_status

        # 4. clear activity stream for next tick
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
        Read all WRITE activities from Blackboard this tick and emit bb_write events.

        Compression: subscriber callbacks run at 30-100 Hz against a 15 Hz tick,
        so many writes to the same key accumulate per tick.  For each key, group
        consecutive same-value writes into runs and emit only (first, last) per run.
        This preserves every value transition while suppressing mid-run duplicates.

        Example for /line_data: [A, B, null, null, null]
          → emits: A, B, null[first], null[last]   (4 lines instead of 5)

        activity_type values (py_trees 2.1.6):
          'WRITE'       — update to existing key
          'INITIALISED' — first-ever write to a key
        """
        stream = py_trees.blackboard.Blackboard.activity_stream
        if not stream or not stream.data:
            return

        # Group write activities per key (preserving insertion order, Python 3.7+)
        key_activities = {}
        for activity in stream.data:
            act_type = str(activity.activity_type)
            if act_type not in ("WRITE", "INITIALISED"):
                continue
            key_activities.setdefault(activity.key, []).append(activity)

        # For each key: collapse consecutive same-value runs to (first, last)
        for key, activities in key_activities.items():
            runs = []   # list of lists; each inner list = one same-value run
            for act in activities:
                val_key = repr(act.current_value)
                if runs and repr(runs[-1][-1].current_value) == val_key:
                    runs[-1].append(act)
                else:
                    runs.append([act])

            for run in runs:
                self._emit_bb_write(tick_id, run[0])
                if len(run) > 1:
                    self._emit_bb_write(tick_id, run[-1])

    def _emit_bb_write(self, tick_id: int, activity):
        self._logger.emit_bt({
            "event":   "bb_write",
            "tick_id": tick_id,
            "writer":  activity.client_name,
            "key":     activity.key,
            "value":   self._safe_repr(activity.current_value),
        })

    @staticmethod
    def _safe_repr(value):
        """Convert BB value to JSON-serializable object."""
        if value is None:
            return None
        if isinstance(value, (bool, int, float, str)):
            return value
        if hasattr(value, "__slots__"):          # ROS message object
            from bt_observability.ros_comm_tracer import ROSCommTracer
            return ROSCommTracer._msg_to_dict(value)
        return str(value)
```

**py_trees 挂载方式（在 `marathon_bt_node.py` 中）：**

```python
self._bt_visitor = BTDebugVisitor(self._obs_logger, lambda: self._tick_id)
self.tree.visitors.append(self._bt_visitor)
self.tree.pre_tick_handlers.append(self._bt_visitor.on_tree_tick_start)
self.tree.post_tick_handlers.append(self._bt_visitor.on_tree_tick_end)
```

注意：使用 `pre_tick_handlers.append()` / `post_tick_handlers.append()`（列表追加），不是赋值。

---

### 3.3 `ros_comm_tracer.py` — ROS 通信层追踪

包含三个组件：`ROSCommTracer`（直接 ROS 调用追踪）、`ProxyContext`（共享上下文）、`ManagerProxy`（封装层代理）。

```python
#!/usr/bin/env python3
"""
ROS 通信层追踪工具集。

JSONL 事件字段语义：
  comm_type : "topic_publish" | "service_call" | "action_goal"
  direction : "publish" | "call" | "goal"
  target    : ROS topic 名 / service 名 / action 名（完整路径）
  ros_node  : 对端 ROS 节点名，无法确定时为 null
  source    : Python 调用路径（ManagerProxy 事件专用）
  node      : 发起调用的 BT 节点名
  tick_id   : 当前 tick 编号（用于跨层关联）
"""
import rospy


class ProxyContext:
    def __init__(self):
        self.current_node: str = "unknown"

proxy_context = ProxyContext()   # 模块级单例


class ROSCommTracer:
    def __init__(self, logger, node_name_getter=None, tick_id_getter=None):
        self._logger = logger
        self._node_name = node_name_getter or (lambda: "unknown")
        self._tick_id = tick_id_getter or (lambda: -1)

    def _emit(self, payload: dict):
        payload["node"] = self._node_name()
        payload["tick_id"] = self._tick_id()
        self._logger.emit_comm(payload)

    def publish(self, pub, msg, topic_name, topic_type=None, reason=None, ros_node=None):
        self._emit({
            "event":       "ros_comm",
            "comm_type":   "topic_publish",
            "direction":   "publish",
            "target":      topic_name,
            "target_type": topic_type,
            "ros_node":    ros_node,
            "payload":     self._msg_to_dict(msg),
            "reason":      reason,
        })
        pub.publish(msg)

    def service_call(self, client, req, service_name, reason=None, ros_node=None):
        self._emit({
            "event":     "ros_comm",
            "comm_type": "service_call",
            "direction": "call",
            "target":    service_name,
            "ros_node":  ros_node,
            "payload":   self._msg_to_dict(req),
            "reason":    reason,
        })
        resp = client(req)
        self._emit({
            "event":     "ros_comm_result",
            "comm_type": "service_call",
            "direction": "call",
            "target":    service_name,
            "result":    self._msg_to_dict(resp),
        })
        return resp

    def send_goal(self, action_client, goal, action_name,
                  goal_id=None, reason=None, ros_node=None):
        self._emit({
            "event":     "ros_comm",
            "comm_type": "action_goal",
            "direction": "goal",
            "target":    action_name,
            "ros_node":  ros_node,
            "goal_id":   goal_id,
            "payload":   self._msg_to_dict(goal),
            "reason":    reason,
        })
        action_client.send_goal(goal)

    @staticmethod
    def _msg_to_dict(msg):
        if msg is None:
            return None
        if hasattr(msg, "__slots__"):
            result = {}
            for slot in msg.__slots__:
                value = getattr(msg, slot)
                if hasattr(value, "__slots__"):
                    result[slot] = ROSCommTracer._msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    result[slot] = [
                        ROSCommTracer._msg_to_dict(v) if hasattr(v, "__slots__") else v
                        for v in value
                    ]
                else:
                    result[slot] = value
            return result
        return str(msg)


class ManagerProxy:
    """
    Generic transparent proxy for Python manager classes that wrap ROS interfaces.

    Intercepts methods listed in ros_map and emits ros_comm events.
    Non-listed methods pass through unchanged.
    Reads proxy_context.current_node for BT node attribution.
    """

    def __init__(self, real_manager, logger, tick_id_getter,
                 ros_map: dict, proxy_name: str = "proxy"):
        object.__setattr__(self, '_real',    real_manager)
        object.__setattr__(self, '_logger',  logger)
        object.__setattr__(self, '_tick_id', tick_id_getter)
        object.__setattr__(self, '_map',     ros_map)
        object.__setattr__(self, '_name',    proxy_name)

    def __getattr__(self, name):
        real_method = getattr(object.__getattribute__(self, '_real'), name)
        ros_map     = object.__getattribute__(self, '_map')
        if name not in ros_map:
            return real_method
        ros_info   = ros_map[name]
        logger     = object.__getattribute__(self, '_logger')
        tick_id_fn = object.__getattribute__(self, '_tick_id')
        proxy_name = object.__getattribute__(self, '_name')

        def traced(*args, **kwargs):
            payload_fn = ros_info.get("payload_fn", lambda a, k: {})
            logger.emit_comm({
                "event":     "ros_comm",
                "comm_type": ros_info["comm_type"],
                "direction": ros_info["direction"],
                "target":    ros_info["target"],
                "ros_node":  ros_info.get("ros_node"),
                "source":    "{}.{}".format(proxy_name, name),
                "payload":   payload_fn(args, kwargs),
                "node":      proxy_context.current_node,
                "tick_id":   tick_id_fn(),
            })
            return real_method(*args, **kwargs)

        return traced

    def __setattr__(self, name, value):
        setattr(object.__getattribute__(self, '_real'), name, value)
```

---

## 4. Marathon 项目接入（具体改动）

### 4.1 `marathon/behaviours/visual_patrol.py` — 新建本地拷贝

从 `ainex_example/visual_patrol.py` 复制到 `marathon/behaviours/visual_patrol.py`，增加 `stop()` 方法，不再依赖外部路径。`VisualPatrol.__init__` 接受 `gait_manager` 可以是真实对象或 `ManagerProxy`，`process()` 内部的 `gait_manager.set_step()` 自动被拦截记录。

---

### 4.2 `marathon_bt_node.py` 修改

```python
# --- 新增 import ---
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor
from bt_observability.ros_comm_tracer import ManagerProxy, ROSCommTracer

_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log')

# Topics recorded via rosbag for full message-level observability.
_ROSBAG_TOPICS = [
    "/walking/set_param",
    "/walking/command",
    "/ros_robot_controller/bus_servo/set_position",
    "/ros_robot_controller/set_buzzer",
    "/imu",
    "/object/pixel_coords",
]

class MarathonBTNode(Common):
    def __init__(self, name):
        # ... 现有初始化代码 ...

        # === 可观测性系统初始化 ===
        os.makedirs(_LOG_DIR, exist_ok=True)
        self._tick_id = 0
        # Per-topic dedup: only emit ros_topic_received once per BT tick
        self._last_imu_emit_tick = -1
        self._last_obj_emit_tick = -1

        self._obs_logger = DebugEventLogger(
            bt_topic="/bt_debug",
            comm_topic="/bt_ros_comm_debug",
            bt_lastrun_jsonl=rospy.get_param(
                '~bt_jsonl', os.path.join(_LOG_DIR, 'bt_debug_lastrun.jsonl')),
            comm_lastrun_jsonl=rospy.get_param(
                '~comm_jsonl', os.path.join(_LOG_DIR, 'bt_ros_comm_debug_lastrun.jsonl')),
            rolling_bt_jsonl=os.path.join(_LOG_DIR, 'bt_debug_recent.jsonl'),
            rolling_comm_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_recent.jsonl'),
            max_rolling_ticks=rospy.get_param('~max_rolling_ticks', 30),
            tick_id_getter=lambda: self._tick_id,
            rosbag_topics=_ROSBAG_TOPICS,
            rosbag_dir=os.path.join(_LOG_DIR, 'rosbag'),
        )

        self._gait_proxy   = ManagerProxy(self.gait_manager,   self._obs_logger,
                                          lambda: self._tick_id, _GAIT_ROS_MAP,   "gait_manager")
        self._motion_proxy = ManagerProxy(self.motion_manager, self._obs_logger,
                                          lambda: self._tick_id, _MOTION_ROS_MAP, "motion_manager")
        self.visual_patrol = VisualPatrol(self._gait_proxy)

        self.tree = bootstrap(
            self._motion_proxy, self._gait_proxy, self.visual_patrol, self.buzzer_pub,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
        )

        self._bt_visitor = BTDebugVisitor(self._obs_logger, lambda: self._tick_id)
        self.tree.visitors.append(self._bt_visitor)
        self.tree.pre_tick_handlers.append(self._bt_visitor.on_tree_tick_start)
        self.tree.post_tick_handlers.append(self._bt_visitor.on_tree_tick_end)
        # BTDebugVisitor.__init__ calls enable_activity_stream(500) — no separate call needed

    def _imu_callback(self, msg):
        """Detect falls. Emits ros_topic_received once per BT tick (dedup)."""
        if self._tick_id != self._last_imu_emit_tick:
            self._last_imu_emit_tick = self._tick_id
            self._obs_logger.emit_comm({
                "event":     "ros_topic_received",
                "direction": "subscribe",
                "target":    "/imu",
                "ros_node":  None,
                "node":      self.name,
                "tick_id":   self._tick_id,
                "payload":   ROSCommTracer._msg_to_dict(msg),
            })
        # ... fall detection logic ...

    def _objects_callback(self, msg):
        """Write line detection to BB. Emits ros_topic_received once per BT tick (dedup)."""
        if self._tick_id != self._last_obj_emit_tick:
            self._last_obj_emit_tick = self._tick_id
            self._obs_logger.emit_comm({
                "event":     "ros_topic_received",
                "direction": "subscribe",
                "target":    "/object/pixel_coords",
                "ros_node":  None,
                "node":      self.name,
                "tick_id":   self._tick_id,
                "payload":   ROSCommTracer._msg_to_dict(msg),
            })
        # ... BB write logic ...

    def run(self):
        rate = rospy.Rate(15)
        while self.running and not rospy.is_shutdown():
            if self.start and self._exec_ctrl.should_tick():
                self._tick_id += 1
                self._bb.tick_id = self._tick_id
                self.tree.tick()
                # Note: do NOT read/clear activity_stream here.
                # BTDebugVisitor.on_tree_tick_end() already flushes and clears it
                # inside tree.tick(). Reading it here would always see an empty stream.
                if self._snapshot_visitor.changed:
                    rospy.loginfo('\n' + py_trees.display.unicode_tree(
                        self.tree.root, show_status=True))
            rate.sleep()

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        self._obs_logger.close()
        rospy.signal_shutdown('shutdown')
```

**ros_topic_received 去重设计：** subscriber 回调以 30-100 Hz 运行，BT tick 为 15 Hz，每 tick 内同一 topic 可能有多次回调。`_last_imu_emit_tick` / `_last_obj_emit_tick` 跟踪上次 emit 的 tick_id，确保每个 tick 每个 topic 最多记录一行（首次到达的那条消息）。

---

### 4.3 `marathon_bt.py` 修改

`bootstrap()` 接受 `logger` 和 `tick_id_getter`，并将 `tick_id_getter` 传给所有 Condition 节点：

```python
safety_gate.add_children([
    IsRobotStanding("IsRobotStanding", logger=logger,
                    tick_id_getter=tick_id_getter),   # ← tick_id_getter 必传
    recovery,
])

line_following.add_children([
    IsLineDetected("IsLineDetected", logger=logger,
                   tick_id_getter=tick_id_getter),    # ← tick_id_getter 必传
    FollowLine("FollowLine", visual_patrol,
               logger=logger, tick_id_getter=tick_id_getter),
])
```

---

### 4.4 `conditions.py` 修改 — 决策输入记录

两个 Condition 节点均增加 `tick_id_getter` 参数（为后续 subscriber 回调中 emit 使用预留）：

```python
class IsRobotStanding(py_trees.behaviour.Behaviour):
    def __init__(self, name="IsRobotStanding", logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/robot_state", access=Access.READ)

    def update(self):
        state = self.bb.robot_state
        passed = (state == 'stand')
        status = py_trees.common.Status.SUCCESS if passed else py_trees.common.Status.FAILURE
        if self._logger:
            self._logger.emit_bt({
                "event": "decision",
                "node": self.name,
                "inputs": {"robot_state": state},
                "status": str(status),
                "reason": "robot_state == 'stand'" if passed else f"robot_state == '{state}'",
            })
        return status


class IsLineDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name="IsLineDetected", logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/line_data", access=Access.READ)

    def update(self):
        line_data = self.bb.line_data
        passed = (line_data is not None)
        status = py_trees.common.Status.SUCCESS if passed else py_trees.common.Status.FAILURE
        if self._logger:
            inputs = {"line_detected": passed}
            if passed:
                inputs["line_x"] = line_data.x
                inputs["line_width"] = line_data.width
            self._logger.emit_bt({
                "event": "decision",
                "node": self.name,
                "inputs": inputs,
                "status": str(status),
                "reason": "line_data present" if passed else "line_data is None",
            })
        return status
```

**注意：** Condition 节点本身没有 subscriber，subscriber 在 `MarathonBTNode._imu_callback` 和 `_objects_callback` 中；`tick_id_getter` 在 Condition 内为未来扩展预留。

---

### 4.5 `actions.py` 修改 — proxy_context + ROSCommTracer（仅 buzzer）

```python
from bt_observability.ros_comm_tracer import ROSCommTracer, proxy_context

class StopWalking(py_trees.behaviour.Behaviour):
    def update(self):
        proxy_context.current_node = self.name
        self.gait_manager.disable()   # ManagerProxy 自动追踪
        return py_trees.common.Status.SUCCESS

class FollowLine(py_trees.behaviour.Behaviour):
    def update(self):
        line_data = self.bb.line_data
        proxy_context.current_node = self.name
        self.visual_patrol.process(line_data.x, line_data.width)  # set_step 自动追踪
        return py_trees.common.Status.SUCCESS

class RecoverFromFall(py_trees.behaviour.Behaviour):
    def __init__(self, name, motion_manager, gait_manager, buzzer_pub,
                 logger=None, tick_id_getter=None):
        # 仅 buzzer（直接 ROS publish，非 manager 封装）需要 ROSCommTracer
        self._tracer = ROSCommTracer(logger, lambda: self.name, tick_id_getter) \
            if logger else None

    def update(self):
        proxy_context.current_node = self.name
        # Buzzer — 直接 pub，用 ROSCommTracer
        if self._tracer:
            self._tracer.publish(self.buzzer_pub, buzzer_msg,
                                 "/ros_robot_controller/set_buzzer", ...)
        # manager 调用 — ManagerProxy 自动追踪
        self.gait_manager.disable()
        self.motion_manager.run_action(action_name)
```

---

## 5. 完整事件类型定义

### BT 决策层事件（`/bt_debug` + `bt_debug_*.jsonl`）

| event | 触发时机 | 关键字段 |
|-------|---------|---------|
| `tree_tick_start` | 每 tick 开始 | `tick_id` |
| `tree_tick_end` | 每 tick 结束 | `tick_id`, `status` |
| `tick_end` | 每个被 tick 的节点执行后 | `tick_id`, `node`, `type`, `status` |
| `decision` | Condition 节点 `update()` 结束 | `tick_id`, `node`, `inputs`, `status`, `reason` |
| `bb_write` | 每 tick 结束时从 activity_stream flush | `tick_id`, `writer`, `key`, `value` |

**bb_write 压缩规则：** 同 key 连续相同值 → 仅保留首尾两条（若只有一条则保留一条），每次值变化都记录。

### ROS 通信层事件（`/bt_ros_comm_debug` + `bt_ros_comm_debug_*.jsonl`）

| event | 触发时机 | 方向 | 关键字段 |
|-------|---------|------|---------|
| `ros_comm` (`topic_publish`) | BT action `update()` 内同步发布 | BT → ROS | `target`, `ros_node`, `source`, `payload`, `node`, `tick_id` |
| `ros_comm` (`service_call`) | BT action `update()` 内同步调用 | BT → ROS | `target`, `ros_node`, `source`, `payload`, `node`, `tick_id` |
| `ros_comm_result` | service_call 返回时 | ROS → BT | `target`, `result` |
| `ros_topic_received` | subscriber 回调首次触发（每 tick 每 topic 仅一次）| 外部 ROS → BT | `target`, `ros_node`=null, `node`, `tick_id`, `payload` |
| `ros_topology_snapshot` | root status 变化时（daemon 线程异步）| 系统快照 | `node_details`, `topic_details`, `service_details`, `tick_id` |

**`ros_topic_received` 的 `ros_node` 固定为 `null`**：ROS1 subscriber 回调不提供发布方信息，可从同 tick 最近的 `ros_topology_snapshot.node_details` 中查询。

---

## 6. 典型 JSONL 输出示例

### `bt_debug_*.jsonl`（决策层）

```jsonl
{"event":"tree_tick_start","tick_id":42,"ts":1710000000.100}
{"event":"decision","node":"IsRobotStanding","inputs":{"robot_state":"stand"},"status":"Status.SUCCESS","reason":"robot_state == 'stand'","tick_id":42,"ts":1710000000.101}
{"event":"tick_end","node":"IsRobotStanding","type":"IsRobotStanding","status":"Status.SUCCESS","tick_id":42,"ts":1710000000.102}
{"event":"decision","node":"IsLineDetected","inputs":{"line_detected":true,"line_x":82,"line_width":640},"status":"Status.SUCCESS","reason":"line_data present","tick_id":42,"ts":1710000000.103}
{"event":"tick_end","node":"FollowLine","type":"FollowLine","status":"Status.SUCCESS","tick_id":42,"ts":1710000000.115}
{"event":"bb_write","tick_id":42,"writer":"MarathonBTNode","key":"/line_data","value":{"x":82,"width":640,...},"ts":1710000000.116}
{"event":"bb_write","tick_id":42,"writer":"MarathonBTNode","key":"/last_line_x","value":82,"ts":1710000000.116}
{"event":"bb_write","tick_id":42,"writer":"MarathonBTNode","key":"/camera_lost_count","value":0,"ts":1710000000.117}
{"event":"tree_tick_end","tick_id":42,"status":"Status.SUCCESS","ts":1710000000.118}
```

### `bt_ros_comm_debug_*.jsonl`（通信层）

**FollowLine tick（巡线中）：**
```jsonl
{"event":"ros_topic_received","direction":"subscribe","target":"/object/pixel_coords","ros_node":null,"node":"marathon_bt","tick_id":42,"payload":{...},"ts":1710000000.099}
{"event":"ros_comm","comm_type":"topic_publish","direction":"publish","target":"walking/set_param","ros_node":"ainex_controller","source":"gait_manager.set_step","payload":{"x":0.01,"y":0,"yaw":-3},"node":"FollowLine","tick_id":42,"ts":1710000000.110}
```

**root status 変化时（跌倒检测后，异步写入）：**
```jsonl
{"event":"ros_topology_snapshot","tick_id":150,"node_details":{"/ainex_controller":{"publications":[...],"subscriptions":[...],"services":[...]},...},"topic_details":{"/imu":"sensor_msgs/Imu",...},"service_details":{"/walking/command":"ainex_interfaces/WalkingCommand",...},"ts":...}
```

---

## 7. 配置参数（rosparam）

```xml
<node pkg="ainex_behavior" type="marathon_bt_node.py" name="marathon_bt">
  <param name="start" value="true"/>
  <param name="color" value="white"/>
  <!-- 可观测性参数（均有默认值，可选覆盖） -->
  <!-- <param name="max_rolling_ticks" value="30"/> -->
  <!-- <param name="bt_jsonl"   value="/custom/path/bt_debug_lastrun.jsonl"/> -->
  <!-- <param name="comm_jsonl" value="/custom/path/bt_ros_comm_debug_lastrun.jsonl"/> -->
</node>
```

---

## 8. 扩展到其他 ainex BT 项目

### 8.1 通用接入模式

**Step 1：** 在 ROS 节点入口初始化 logger + visitor + ManagerProxy，传入 `rosbag_topics` + `rosbag_dir`。

**Step 2：** 为项目的 manager 类编写对应的 `ros_map`，列出所有命令型方法及其实际 ROS 接口。

**Step 3：** 在所有 Condition 节点的 `update()` 末尾添加 `emit_bt(decision)`；`__init__` 加 `tick_id_getter` 参数。

**Step 4：** 在所有 Action 节点的 `update()` 开头设置 `proxy_context.current_node = self.name`；直接 `pub.publish()` 的调用仍用 `ROSCommTracer` 追踪。

**Step 5：** 在节点入口的 subscriber 回调开头加 `emit_comm(ros_topic_received)` + tick 级去重逻辑。

### 8.2 logger=None 保持向后兼容

所有节点的 `logger` 参数默认 `None`；不传时行为与原始代码完全一致，零开销。

---

## 9. 实施步骤

### Phase 1：基础设施 ✅
- [x] 创建 `ainex_behavior/bt_observability/` 目录及 `__init__.py`
- [x] 实现 `debug_event_logger.py`（atomic write + newest-first）
- [x] 实现 `bt_debug_visitor.py`
- [x] 实现 `ros_comm_tracer.py`（含 `ProxyContext` + `ManagerProxy`）
- [x] 在 Docker 容器内验证文件可被 import

### Phase 2：Marathon 接入 ✅
- [x] 新建 `marathon/behaviours/visual_patrol.py`（本地拷贝 + stop()）
- [x] 修改 `marathon_bt_node.py`：GAIT_ROS_MAP / MOTION_ROS_MAP / ManagerProxy / logger / visitor / tick_id
- [x] 修改 `marathon_bt.py`：bootstrap() 新增参数
- [x] 修改 `conditions.py`：emit decision 事件
- [x] 修改 `actions.py`：proxy_context 上下文 + ROSCommTracer 仅用于 buzzer

### Phase 3：验证 ✅
- [x] `rostopic echo /bt_debug` 确认事件流
- [x] `rostopic echo /bt_ros_comm_debug` 确认 comm 事件有 direction/ros_node 字段
- [x] FollowLine tick 中出现 `walking/set_param` 事件，payload 含 x/y/yaw
- [x] 手动触发跌倒恢复，验证 RecoverFromFall tick 完整事件序列

### Phase 4：录制侧完善 ✅
- [x] `bt_debug_visitor.py`：增加 bb_write flush；修正 ActivityItem 字段名（`key`/`current_value`）；WRITE+INITIALISED 两种 activity type；同 key 连续相同值压缩；异步拓扑快照 daemon thread
- [x] `debug_event_logger.py`：新增 `snapshot_ros_topology`、rosbag 后台录制（60s 分片，30min 保留）
- [x] `marathon_bt_node.py`：`ros_topic_received` emit + tick 级去重（`_last_imu_emit_tick`/`_last_obj_emit_tick`）；删除 `run()` 中 activity stream 读取块；`ROSCommTracer` import
- [x] `marathon_bt.py`：`tick_id_getter` 传给 `IsRobotStanding` + `IsLineDetected`
- [x] `conditions.py`：两个 Condition 节点加 `tick_id_getter` 参数

### 验证命令

```bash
# bb_write 事件（含正确 key 和 value）
grep '"event":"bb_write"' marathon/log/bt_debug_recent.jsonl

# ros_topic_received（每 tick 每 topic 仅一行）
grep '"event":"ros_topic_received"' marathon/log/bt_ros_comm_debug_recent.jsonl

# 触发跌倒，等待 root status 变化，检查拓扑快照
grep '"event":"ros_topology_snapshot"' marathon/log/bt_ros_comm_debug_lastrun.jsonl

# rosbag 文件
ls marathon/log/rosbag/
```

---

## 10. 局限性与注意事项

| 问题 | 说明 | 状态 |
|------|------|------|
| `proxy_context` 非线程安全 | 单线程 BT tick loop，无问题；多线程场景需 `threading.local()` | 当前架构单线程，无需处理 |
| `RecoverFromFall.update()` 阻塞（sleep 2s）| 阻塞整个 tick loop | 现有设计如此，可观测性不引入额外阻塞 |
| `snapshot_ros_topology` 耗时 5-10s | rosnode info 每节点约 0.2-0.5s | 已用 daemon thread 异步，不影响 tick loop |
| `ros_topic_received` 仅记录首次 | 每 tick 只记录到达的第一条消息 payload | 设计决策：去重降噪；需完整消息流用 rosbag |
| `ros_topic_received.ros_node` 为 null | ROS1 subscriber 回调不提供发布方 | 可从同 tick 的 `ros_topology_snapshot` 查询 |
| rosbag 占用磁盘 | 30min × 所有 topic 流量 | `--max-splits 30` 自动删旧；按需裁减 `_ROSBAG_TOPICS` |
| bb_write 压缩丢弃中间值 | 同 key 连续相同值只保留首尾 | 中间值通常为噪声；需精确时用 rosbag |
| `visual_patrol.py` 需同步更新 | 若源文件有 bug fix，需手动同步本地拷贝 | 保持版本注释，便于 diff |

---

## 11. 后续扩展方向

- **Foxglove/Web UI**：订阅 `/bt_debug` + `/bt_ros_comm_debug` 渲染 BT 时序图
- **ROSA Agent 接入**：见第 12 节
- **Structured Msg**：将 JSON String topic 升级为自定义 `.msg` 类型（`BTEvent.msg`）
- **rosbag 回放侧工具**：ROSA `read_rosbag` 工具，按 tick_id 范围查询具体消息内容

---

## 12. ROSA Agent 接入方案

### 12.1 Docker Mount

`docker/docker-compose.yml` 的 `rosa-agent.volumes` 新增一行（只读）：

```yaml
volumes:
  - rosa_agent_logs:/opt/rosa-agent/logs
  - /home/pi/docker/ros_log:/root/.ros/log:ro
  - /home/pi/docker/ros_ws_src/ainex_behavior/marathon/log:/opt/ainex_bt_log:ro  # ← 新增
```

---

### 12.2 新建工具文件 `bt_obs.py`

**文件路径：** `docker/rosa-agent/ainex_agent_tools/tools/bt_obs.py`

核心工具（已实现，见 `read_bt_obs` unified tool）：
- `read_bt_recent_obs` — 读取 rolling JSONL（实时）
- `read_bt_lastrun_obs` — 读取 lastrun JSONL 并生成摘要（会话结束后）
- `read_bt_obs` — 统一入口，自动路由：BT 节点运行中选 recent，否则选 lastrun

**ROSA 工具说明（`about_your_capabilities` prompt 中 PRIORITY 1）：**
- `read_bt_obs` 自动路由：检查 BB bridge topic（`/bt/marathon/bb/tick_id`）是否活跃，活跃选 recent pair，否则选 lastrun pair
- LLM 应将 tick_id / 每 tick 决策查询路由到此工具

---

### 12.3 注册工具

`docker/rosa-agent/ainex_agent_tools/__init__.py` 末尾追加：

```python
from ainex_agent_tools.tools.bt_obs import read_bt_recent_obs, read_bt_lastrun_obs
```

---

### 12.4 验证步骤

```bash
# 1. 重启 ROSA 容器
docker compose up -d rosa-agent

# 2. 验证挂载
docker exec rosa-agent ls /opt/ainex_bt_log/

# 3. 验证工具可导入
docker exec rosa-agent python3 -c \
  "from ainex_agent_tools.tools.bt_obs import read_bt_recent_obs; print('ok')"

# 4. 功能验证（marathon 节点运行中）
#    问 ROSA："Show me the current BT behavior"  → read_bt_recent_obs
#    问 ROSA："Summarize the last marathon run"   → read_bt_lastrun_obs
```

---

## 附录：文件改动汇总

| 文件 | 操作 | 主要改动 |
|------|------|---------|
| `bt_observability/__init__.py` | **新建** | 空文件 |
| `bt_observability/debug_event_logger.py` | **新建** | DebugEventLogger；atomic write + newest-first；rosbag 60s 分片 30min 保留；snapshot_ros_topology |
| `bt_observability/bt_debug_visitor.py` | **新建** | BTDebugVisitor；bb_write flush（正确 ActivityItem 字段）；same-value 压缩；异步拓扑快照 |
| `bt_observability/ros_comm_tracer.py` | **新建** | ROSCommTracer + ProxyContext + proxy_context + ManagerProxy |
| `marathon/behaviours/visual_patrol.py` | **新建** | 从 ainex_example 复制，加 stop() 方法 |
| `marathon/marathon_bt_node.py` | **修改** | _ROSBAG_TOPICS；DebugEventLogger 新参数；ROSCommTracer import；ros_topic_received emit + tick 去重；删除 activity stream 读取块 |
| `marathon/marathon_bt.py` | **修改** | bootstrap() / MarathonBT.__init__() 传 logger + tick_id_getter；Condition 节点加 tick_id_getter |
| `marathon/behaviours/conditions.py` | **修改** | IsRobotStanding + IsLineDetected 加 tick_id_getter 参数；emit_bt(decision) |
| `marathon/behaviours/actions.py` | **修改** | proxy_context.current_node；ROSCommTracer 仅用于 buzzer |
| `.gitignore` | **已有** | `marathon/log/` 已排除（含 rosbag/ 子目录）|
