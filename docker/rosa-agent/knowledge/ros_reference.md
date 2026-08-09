# ROSA-XYZ progressively loaded reference

Sections in this file are appended to a query as `[Reference context]` when the
query matches a section's keyword line (see `xyz_agent_tools/knowledge.py`).
Each section must start with a `<!-- keywords: ... -->` comment line.

## ROS Topics
<!-- keywords: topic, topics, subscribe, publish, publisher, message, imu, battery, voltage, camera, image, vision, pixel_coords, apriltag, tag, detection, rosout, walking, gait, hz, echo -->

Verified topics on the Ainex robot (ROS Noetic, master at 127.0.0.1:11311):

| Topic | Description |
|---|---|
| `/ros_robot_controller/imu_raw` | Raw 9-axis IMU (sensor_msgs/Imu, 100 Hz) |
| `/imu` | Filtered orientation via complementary filter (sensor_msgs/Imu, 100 Hz) |
| `/ros_robot_controller/battery` | Battery voltage (std_msgs/UInt16, millivolts, ~1 Hz) |
| `/camera/image_raw` | Raw RGB camera (sensor_msgs/Image, 30 Hz) |
| `/walking/is_walking` | Gait engine on/off (std_msgs/Bool, on change) |
| `/object/pixel_coords` | Unified vision detections (ainex_interfaces/ObjectsInfo) |
| `/tag_detections` | AprilTag detections (apriltag_ros/AprilTagDetectionArray, 30 Hz) |
| `/rosout_agg` | Aggregated ROS logs |

## ROS Services
<!-- keywords: service, services, rosservice, get_state, servo state, walking command, get_param, enable_control, start, stop -->

Verified services — these are handled by custom tools, do NOT call them directly:

| Service | Purpose | Use instead |
|---|---|---|
| `/ros_robot_controller/bus_servo/get_state` | servo health | `get_robot_health` |
| `/walking/command` | gait arm/start/stop (enable_control, start, stop, disable_control) | read-only: never call |
| `/walking/is_walking` | walking state | `get_walking_state` |
| `/walking/get_param` | gait params | `get_walking_state` |

## BT Observability
<!-- keywords: bt, behavior tree, behaviour tree, tick, tick_id, blackboard, bb, snapshot, tree, latched, pause, step, jsonl, debug log, observability, py_trees, groot, rqt -->

BT-related topics (the `{project}` segment is the active BT node name):

| Topic | Description |
|---|---|
| `/{project}/log/tree` | BT snapshot (py_trees_msgs/BehaviourTree, latched) |
| `/{project}/ascii/snapshot` | ASCII tree rendering (std_msgs/String, latched) |
| `/bt/bb/latched/*` | Shared blackboard mirrors (std_msgs/String JSON, 10 Hz): robot_state, detected_count, etc. |
| `/bt/{project}/bb/*` | Project-specific blackboard mirrors |

ROSA auto-detects the active project by scanning for a `/{project}/ascii/snapshot` topic.

BT execution control (operator runs these, agent is read-only): services
`~bt/run`, `~bt/pause`, `~bt/step` on the BT node switch RUN/PAUSE/STEP mode.
Pause/step mode gives the most accurate tick capture for analyze_bt_tick and
cross_tick_analysis.

## Tool Reference (detailed)
<!-- keywords: tool, tools, capability, capabilities, health, servo, torque, temperature, walking state, bt status, analyze, analysis, diagnose, diagnosis, raw, evidence, cross tick, cross-tick, transition, drift, episode -->

PRIORITY 1 — XYZ custom tools (detailed reference):

- `get_robot_health` — battery voltage, per-servo temperature/voltage/torque-enable
  state, IMU roll/pitch. torque=off means the servo is limp — the robot collapses
  regardless of BT commands. Call after any Layer 4 (physical/hardware) diagnosis,
  or whenever robot posture does not match the blackboard state.
- `get_walking_state` — walking controller state, gait parameters, walking offset.
- `get_bt_status` — live BT tree snapshot, active node, blackboard mirror, tick_id.
  Use for current real-time robot state and live ROS topology questions.
- `analyze_bt_tick` — single-tick status diagnosis. Analyzes ONE specific tick in
  depth: which BT branch was active, what inputs drove each condition, blackboard
  state, ROS commands emitted, and whether that matches user observation.
  Best for a single paused/stepped tick. tick_id default: 'latest'.
  With user_observation: explain/compare/diagnose. Without: explain_tick only.
  Tool output includes an embedded JSONL schema legend for interpreting raw evidence.
- `get_bt_tick_raw` — raw JSONL evidence for a specific tick from the recent log.
  Use when the user asks to see raw log data, or to inspect raw evidence before
  calling analyze_bt_tick. Output includes the same JSONL schema legend.
- `cross_tick_analysis` — cross-tick relationship analysis covering up to 30 recent
  ticks (~3 seconds at ~10 Hz BT rate). Detects stable BT execution segments,
  transitions between states, and supporting evidence drift within segments.
  tick_selection: 'all' | 'latest:N' | 'A-B' | 'A,B,C'. user_observation optional.
  Outputs: explain_cross_ticks / evidence_chain / transition_details /
  representative_ticks / observation_slot / compare_followup.
- `stop_current_behavior` / `stand_safe` — disabled stubs in read-only mode.

PRIORITY 2 — ROSA built-in read tools (only if no custom tool covers the query):
rosgraph_get, rostopic_list, rosnode_list, rostopic_info, rostopic_echo,
rosnode_info, rosservice_list, rosservice_info, rosmsg_info, rossrv_info,
rosparam_list, rosparam_get, rospkg_list, roslaunch_list.

## Runtime Environment
<!-- keywords: docker, container, environment, network, master, ros_master_uri, mount, log file, logs, noetic, where, path, deployment -->

- The Ainex robot runs ROS Noetic inside a Docker container named `ainex`.
- This agent runs in the `rosa-agent` container with host networking and
  `ROS_MASTER_URI=http://127.0.0.1:11311`.
- BT debug logs are mounted read-only at `/opt/ainex_bt_log`
  (bt_debug_recent.jsonl, bt_ros_comm_debug_recent.jsonl and *_lastrun variants).
- ROS node logs are mounted read-only at `/root/.ros/log`.
- Agent project root inside the container: `/opt/rosa-agent` (live mount of
  `/home/pi/docker/rosa-agent` on the host).
