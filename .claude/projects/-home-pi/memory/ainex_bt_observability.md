# xyz_behavior BT Observability System

**Rewritten Aug 9 2026** — the previous version described the pre-rename
(`ainex_behavior`) module and was stale on two factual points (rolling-file
ordering, and a third log file that didn't exist yet). Verified against actual
source this pass.

**Module**: `docker/ros_ws_src/xyz_behavior/bt_observability/`
**Log dir**: `docker/ros_ws_src/xyz_behavior/log/` (flat, not per-project — will
need revisiting once sprint/basketball/penalty_kick each need their own log dir)

---

## Three files, not two

| File | Writer | Order | Refresh |
|------|--------|-------|---------|
| `bt_debug_lastrun.jsonl` / `bt_ros_comm_debug_lastrun.jsonl` | `DebugEventLogger` | append-only oldest-first during run; **reversed to newest-first in `close()`** | on every `end_tick()` |
| `bt_debug_recent.jsonl` / `bt_ros_comm_debug_recent.jsonl` | `DebugEventLogger` | **oldest-first** (natural deque order — correction: old doc said newest-first, that was wrong) | throttled: rewritten at most every `flush_min_interval_secs` (default **2.0s**), not every tick |
| `bb_current.json` | `BlackboardCurrentWriter` | n/a (single snapshot) | every tick, atomic (`.tmp` + `fsync` + `os.replace`) |

`bb_current.json` is new since the last write-up: `{schema_version, tick_id, ts,
root_status, blackboard: {key: value, ...}}` — full current BB state, not a
history log. Ownership is the `app/*_bt_node.py` entry point only; adapters/L1/L2
nodes/`bb_ros_bridge` must never write it.

Rolling files use `max(tick_id)` to resolve "latest" so write order doesn't
affect correctness even though they're oldest-first.

---

## Event schema (`bt_debug_*.jsonl`)

Same base fields as before (`event`, `tick_id`, `ts`) plus two event types that
didn't exist in the last write-up:

- **`bb_write`** — one per Blackboard key write, sourced from py_trees'
  built-in activity stream (`enable_activity_stream`), flushed at
  `on_tree_tick_end`. Compressed: consecutive same-value writes to a key
  collapse to (first, last) of the run — e.g. `[0,0,1,2]` emits `0(first),
  0(last), 1, 2`, not all four. Fields: `writer` (client name), `key`, `value`.
- **`ros_topology_snapshot`** — captured **once per session**, in a daemon
  thread, ~2s after startup (settle delay so the graph is complete). Uses
  `rosgraph.Master` XML-RPC directly (`getSystemState()` + `getTopicTypes()`,
  two calls) instead of spawning per-node/per-service CLI subprocesses. Fields:
  `node_details` (pub/sub/services per node), `topic_details` (topic→type),
  `service_details` (service→provider nodes; no bulk service-type API exists).
  Written to the **comm** log, not the BT log.

Unchanged: `tree_tick_start`, `tree_tick_end` (with `status`), `tick_end`
(per-node, only actually-ticked nodes since `VisitorBase(full=False)`),
`decision` (emitted by condition nodes via `emit_decision()`).

## ROS comm schema (`bt_ros_comm_debug_*.jsonl`)

Unchanged: `ros_comm` (outgoing, `comm_type` ∈ topic_publish/service_call/
action_goal/method_call) + `ros_comm_result`. `ros_comm_tracer.py` is now a thin
message-serialization helper (`_msg_to_dict`) — the tracer object itself lives
elsewhere in the calling node.

## Optional rosbag recording

New since last write-up: `DebugEventLogger(rosbag_topics=[...], rosbag_dir=...)`
starts `rosbag record --split --duration 60 --max-splits 30` in a background
process group (killable via `SIGINT` to the whole group in `close()`). 60s
splits × 30 files = 30 min rolling window, oldest auto-deleted by rosbag itself.
`None`/`None` (default) disables it entirely.

---

## Module structure

- `debug_event_logger.py` — `DebugEventLogger`: two-tier JSONL + optional
  rosbag + topology snapshot capture. `emit_bt()`/`emit_comm()` auto-inject
  `tick_id`/`ts`.
- `bt_debug_visitor.py` — `BTDebugVisitor(logger, tick_id_getter)`, py_trees
  `VisitorBase`. Attach via `tree.visitors.append(...)` +
  `tree.pre_tick_handlers.append(visitor.on_tree_tick_start)` +
  `tree.post_tick_handlers.append(visitor.on_tree_tick_end)` — **lists**, must
  `.append()`, not assign (py_trees 2.1.6 pitfall, unchanged from before).
  A `bt_debug_visitor_original.py` sits alongside it on disk (untracked, not in
  git) — looks like a pre-rewrite backup, not imported by anything current.
- `ros_comm_tracer.py` — `_msg_to_dict()` message serialization only.
- `blackboard_current_writer.py` — `BlackboardCurrentWriter`, writes
  `bb_current.json` (see above).

## ROSA-side reading

`read_bt_obs` (mentioned in older notes) **does not exist anymore**. Current
tools: `session_digest` (bounded whole-run digest from lastrun files),
`bt_tick_analysis` (`get_bt_tick_raw`/`analyze_bt_tick`, live/paused sessions
only, no lastrun fallback), `cross_tick_analysis` (multi-tick episode
analysis). Full tool details: [[ainex_rosa_agent]].

## ROS Topics Published
| Topic | Type | Content |
|-------|------|---------|
| `/bt_debug` | `std_msgs/String` | BT events as JSON string |
| `/bt_ros_comm_debug` | `std_msgs/String` | ROS comm events as JSON string |
