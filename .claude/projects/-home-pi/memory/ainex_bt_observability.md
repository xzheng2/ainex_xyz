# Ainex BT Observability System

**Added**: Apr 5 2026
**Module**: `docker/ros_ws_src/ainex_behavior/bt_observability/`
**Log dir**: `docker/ros_ws_src/ainex_behavior/marathon/log/` (excluded from git)

---

## Overview

Two-tier JSONL logging for the marathon behavior tree:
- **last-run** (`*_lastrun.jsonl`): full session; all ticks buffered in memory, atomically rewritten newest-first on each `end_tick()`
- **rolling** (`*_recent.jsonl`): last 30 ticks, atomically overwritten each tick via `.tmp` + `os.replace`

**All 4 files are newest-first** (highest tick_id at top of file).

Two data streams:
- **BT decision layer** — tree lifecycle and node status (`bt_debug_*.jsonl`)
- **ROS comm layer** — topic publishes, service calls, action goals, manager methods (`bt_ros_comm_debug_*.jsonl`)

---

## JSONL File Paths

| File | Content | Tier |
|------|---------|------|
| `marathon/log/bt_debug_lastrun.jsonl` | BT events, full session, newest-first | last-run |
| `marathon/log/bt_ros_comm_debug_lastrun.jsonl` | ROS comm events, full session, newest-first | last-run |
| `marathon/log/bt_debug_recent.jsonl` | BT events, last 30 ticks, newest-first | rolling |
| `marathon/log/bt_ros_comm_debug_recent.jsonl` | ROS comm events, last 30 ticks, newest-first | rolling |

ROSA container mount: `/home/pi/docker/ros_ws_src/ainex_behavior/marathon/log` → `/opt/ainex_bt_log:ro`

---

## BT Debug JSONL Event Schema (`bt_debug_*.jsonl`)

All events share common fields:
```json
{"event": "<type>", "tick_id": 42, "ts": 1743832111.234}
```

### `tree_tick_start`
Emitted at the start of each tree tick (before any node runs).
```json
{"event": "tree_tick_start", "tick_id": 42, "ts": 1743832111.234}
```

### `tree_tick_end`
Emitted at the end of each tree tick (after all nodes complete).
```json
{"event": "tree_tick_end", "tick_id": 42, "ts": 1743832111.251, "status": "Status.RUNNING"}
```
`status` values: `Status.SUCCESS`, `Status.FAILURE`, `Status.RUNNING`

### `tick_end` (per-node)
Emitted by `BTDebugVisitor.run()` after each node's `update()`. Only nodes that were actually ticked (not pruned by Selector/Sequence short-circuit) appear.
```json
{"event": "tick_end", "tick_id": 42, "ts": 1743832111.240, "node": "IsLineDetected", "type": "IsLineDetected", "status": "Status.FAILURE"}
```

### `decision` (condition nodes)
Emitted by condition nodes themselves (in `conditions.py`). Captures the decision inputs.
```json
{"event": "decision", "tick_id": 42, "ts": 1743832111.238, "node": "IsLineDetected",
 "inputs": {"line_data": null, "lost_count": 5}, "reason": "no line detected",
 "status": "Status.FAILURE"}
```

---

## ROS Comm JSONL Event Schema (`bt_ros_comm_debug_*.jsonl`)

### `ros_comm` — outgoing call
```json
{"event": "ros_comm", "comm_type": "topic_publish", "target": "/ros_robot_controller/set_buzzer",
 "target_type": null, "payload": {"on": true, "freq": 1000, "duration": 0.2}, "reason": "fall detected",
 "node": "RecoverFromFall", "tick_id": 51, "ts": 1743832130.1}
```
`comm_type` values: `topic_publish`, `service_call`, `action_goal`, `method_call`

### `ros_comm_result` — service call response
```json
{"event": "ros_comm_result", "comm_type": "service_call", "target": "/walking/command",
 "result": {}, "node": "RecoverFromFall", "tick_id": 51, "ts": 1743832130.3}
```

---

## Module Structure

### `bt_observability/debug_event_logger.py` — `DebugEventLogger`
Central logger. Constructed once in `marathon_bt_node.py`.

```python
logger = DebugEventLogger(
    bt_topic="/bt_debug",
    comm_topic="/bt_ros_comm_debug",
    bt_lastrun_jsonl=".../log/bt_debug_lastrun.jsonl",
    comm_lastrun_jsonl=".../log/bt_ros_comm_debug_lastrun.jsonl",
    rolling_bt_jsonl=".../log/bt_debug_recent.jsonl",
    rolling_comm_jsonl=".../log/bt_ros_comm_debug_recent.jsonl",
    max_rolling_ticks=30,
    tick_id_getter=lambda: self._tick_id,   # auto-injects tick_id into all events
)
```

**Storage design:**
- Lastrun: `_bt_all_ticks` / `_comm_all_ticks` lists grow in memory; atomically rewritten newest-first on each `end_tick()`; no streaming file handles
- Rolling: `_bt_rolling` / `_comm_rolling` deques (maxlen=30); atomically rewritten newest-first on each `end_tick()`

Key methods:
- `emit_bt(payload)` — write BT event; auto-injects `tick_id` and `ts` via `setdefault`
- `emit_comm(payload)` — write ROS comm event; same auto-injection
- `begin_tick(tick_id)` — called by visitor at tick start; clears per-tick buffers
- `end_tick(tick_id)` — called by visitor at tick end; pushes to deques + all_ticks lists; flushes both rolling + lastrun files newest-first
- `close()` — **no-op** (last `end_tick()` already wrote all files)

### `bt_observability/bt_debug_visitor.py` — `BTDebugVisitor`
py_trees `VisitorBase` (full=False — only visits nodes that were actually ticked).

**Attach to tree**:
```python
visitor = BTDebugVisitor(logger, lambda: self._tick_id)
tree.visitors.append(visitor)
tree.pre_tick_handlers.append(visitor.on_tree_tick_start)   # plural list! use .append()
tree.post_tick_handlers.append(visitor.on_tree_tick_end)
```
**CRITICAL**: py_trees 2.1.6 `pre_tick_handlers` / `post_tick_handlers` are **lists** — must use `.append()`, not assignment (assignment silently creates instance attribute, handlers never called).

### `bt_observability/ros_comm_tracer.py` — `ROSCommTracer`
Per-node helper for tracing ROS calls from action nodes.

```python
self._tracer = ROSCommTracer(
    logger=logger,
    node_name_getter=lambda: self.name,
    tick_id_getter=tick_id_getter,
) if logger else None
```

Methods:
- `tracer.publish(pub, msg, "/topic_name", reason="...")` — record then publish
- `tracer.service_call(client, req, "/service_name")` — record request + response
- `tracer.send_goal(client, goal, "/action_name")` — record action goal
- `tracer.method_call("gait_manager.disable", {}, reason="...")` — record manager calls

---

## Marathon Integration Points

- **`marathon_bt_node.py`**: constructs logger, BTDebugVisitor, attaches pre/post handlers; `self._tick_id += 1` then `self._bb.tick_id = self._tick_id` before each `tree.tick()`; `logger.close()` is no-op on shutdown
- **`marathon_bt.py`**: `bootstrap(logger=None, tick_id_getter=None)` passes to all condition and action node constructors
- **`behaviours/conditions.py`**: `IsRobotStanding`, `IsLineDetected` call `logger.emit_bt({"event": "decision", ...})` with their inputs
- **`behaviours/actions.py`**: `StopWalking`, `FollowLine`, `RecoverFromFall`, `FindLine` create `ROSCommTracer` if logger provided; use `_tracer.publish()` / `_tracer.method_call()` instead of direct calls
- **`logger=None` fallback**: all constructors accept `logger=None`; tracer is `None` when disabled; zero-cost when not used

---

## Blackboard Bridge

`bb_ros_bridge.py` publishes 5 BB keys to ROS topics at 10 Hz:

| BB key | ROS topic | Description |
|--------|-----------|-------------|
| `/tick_id` | `/bt/marathon/bb/tick_id` | BT iteration counter (15 Hz, increments only when should_tick()=True) |
| `/robot_state` | `/bt/marathon/bb/robot_state` | Stand/fall state string |
| `/line_data` | `/bt/marathon/bb/line_data` | Current line detection object or null |
| `/last_line_x` | `/bt/marathon/bb/last_line_x` | Last known line x position |
| `/camera_lost_count` | `/bt/marathon/bb/camera_lost_count` | Consecutive camera frames (30 Hz) without line detection; resets to 0 on detection. NOT BT ticks. |

**Key distinction**: `tick_id` (BT ticks, 15 Hz, ~hundreds per minute) ≠ `camera_lost_count` (camera frames, 30 Hz, resets on any detection).

---

## ROSA Agent Tools

File: `docker/rosa-agent/ainex_agent_tools/tools/bt_obs.py`

### `read_bt_obs` (unified, Apr 6 2026)
Single auto-routing tool replacing the former `read_bt_recent_obs` + `read_bt_lastrun_obs` pair.

**Detection**: `_bt_is_running()` checks `os.path.getmtime("bt_debug_recent.jsonl")`;
if modified within 10 s → BT node is live.

**BT node running** → returns raw JSONL from `*_recent.jsonl` files (newest-first, last 30 ticks)
with header `"# SOURCE: _recent files (BT node is live). ..."`

**BT node not running** → returns structured summary from `*_lastrun.jsonl` files
with header `"# SOURCE: _lastrun files (BT node not running). ..."`
- Re-reverses lines before summarizing (files are newest-first; summarizers expect oldest-first)
- `_summarize_bt_generic()`: tick count, duration, tree result distribution, per-node counts, last `decision` inputs
- `_summarize_comm_generic()`: per-target call count + last payload

**`_SUMMARY_INSTRUCTIONS`** — populated (Apr 6 2026) with description of all 4 files and reading rule;
injected into `_summarize_bt_generic` output under `[Analysis focus: ...]`.

Listed in PRIORITY 1 of ROSA `about_your_capabilities` prompt.

Both tools:
- `@tool` decorator (langchain), `_input: str = ""` (unused), return formatted string
- Read from `/opt/ainex_bt_log/` (container path)
- `[]` on file-not-found (no exception)

---

## ROS Topics Published

| Topic | Type | Content |
|-------|------|---------|
| `/bt_debug` | `std_msgs/String` | BT events as JSON string (one per message) |
| `/bt_ros_comm_debug` | `std_msgs/String` | ROS comm events as JSON string |

Subscribe with: `rostopic echo /bt_debug | python3 -c "import sys,json; [print(json.dumps(json.loads(l),indent=2)) for l in sys.stdin]"`
