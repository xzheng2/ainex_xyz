# ROSA Agent — Ainex Integration Notes

## Overview

`rosa-agent` is a standalone Docker container running NASA JPL ROSA in read-only mode,
connected to the `ainex` container's ROS graph via host networking.

- ROSA repo: https://github.com/nasa-jpl/rosa
- PyPI package: `jpl-rosa` (>=1.0.9)
- Python requirement: 3.9+ (ROSA constraint)
- Phase 1: read-only diagnostics only

## Directory Layout

```
/home/pi/docker/
├── docker-compose.yml                  # manages rosa-agent; ainex managed manually
└── rosa-agent/
    ├── Dockerfile                      # Ubuntu 20.04 + Python 3.9 + ROS Noetic + ROSA
    ├── requirements.txt
    ├── setup.py
    ├── .env.example                    # copy to .env; set LLM_PROVIDER etc.
    ├── ainex_agent.py                  # main entry (CLI + ROS node mode)
    ├── llm_config.py                   # Ollama / OpenAI / Azure
    ├── summarize_ros_logs.py           # log summarizer (finds newest session by mtime)
    ├── ainex_agent_tools/
    │   ├── __init__.py                 # exports AINEX_TOOLS list (8 tools)
    │   ├── prompts.py                  # RobotSystemPrompts for Ainex
    │   └── tools/
    │       ├── health.py               # get_robot_health (battery + all 24 servos + IMU)
    │       ├── behavior.py             # get_current_behavior
    │       ├── walking.py              # get_walking_state
    │       ├── detections.py           # get_latest_detections
    │       ├── logs.py                 # read_recent_ros_logs + read_last_run_summary
    │       └── disabled.py             # stop_current_behavior, stand_safe (stubs)
    ├── config/
    │   ├── readonly.yaml               # per-operation read/write switches
    │   ├── blacklist.yaml              # ROSA built-in tool blacklist
    │   ├── expected_nodes.yaml         # expected nodes per launch (missing-node detection)
    │   └── rqt_marathon_prompt.md      # LLM guide for rqt perspective recommendations
    └── vendor/
        └── rosa/                       # patched ROSA source (fixes rosservice_list bug)
```

## Docker Network & Mounts

- **Host networking**: both `ainex` (`--net host`) and `rosa-agent` (`network_mode: host`) share the host network stack
- No bridge network needed — `ainex_ros_net` is obsolete (can be removed with `docker network rm ainex_ros_net`)
- `rosa-agent` env: `ROS_MASTER_URI=http://127.0.0.1:11311` (no `ROS_HOSTNAME` needed)
- Ollama: `OLLAMA_BASE_URL=http://localhost:11434` (`host.docker.internal` does not resolve in host mode)
- **ROS log mount** (added Mar 14 2026): ainex writes to `/home/ubuntu/.ros/log` → host `/home/pi/docker/ros_log` → rosa-agent reads at `/root/.ros/log:ro`; symlink `/root/.ros/log` → `/home/ubuntu/.ros/log` inside ainex so both root and ubuntu users share the mount
- **Log symlink fix** (Mar 14 2026): ROS recreates `/home/ubuntu/.ros/log/latest` with absolute path on every restart, which is invalid inside rosa-agent. Fixed by making `read_last_run_summary` resolve newest session dir by mtime instead of following the `latest` symlink. No manual symlink fixes needed anymore.

## ROSA Instantiation Pattern

```python
from rosa import ROSA
agent = ROSA(
    ros_version=1,
    llm=get_llm(),
    tools=AINEX_TOOLS,          # list of @tool decorated functions
    tool_packages=[],
    blacklist=load_blacklist(),  # from config/blacklist.yaml
    prompts=get_ainex_prompts(),
    verbose=False,
    streaming=False,
    max_iterations=30,
    max_execution_time=60,      # 60s timeout per query (added Mar 24 2026)
)
```

## ROSA Built-in ROS1 Tool Names (for blacklist reference)

Read (kept): `rosgraph_get`, `rostopic_list`, `rosnode_list`, `rostopic_info`,
`rostopic_echo`, `rosnode_info`, `rosservice_list`, `rosservice_info`,
`rosmsg_info`, `rossrv_info`, `rosparam_list`, `rosparam_get`,
`rospkg_list`, `rospkg_info`, `rospkg_roots`, `roslog_list`, `roslaunch_list`

Write (blacklisted): `rosparam_set`, `rosservice_call`, `roslaunch`, `rosnode_kill`

## Read-Only Tools Implemented

| Tool | ROS interface | Notes |
|------|--------------|-------|
| `get_robot_health` | `/ros_robot_controller/battery` (UInt16, mV), `/ros_robot_controller/bus_servo/get_state` (GetBusServoState), `/ros_robot_controller/imu_raw` (Imu) | battery <10500mV = LOW; queries **all 24 servos** grouped by Legs/Arms/Head; reads temperature, voltage, **and torque state** (`get_torque_state=1`); servo >60°C = HOT; `get_servo_torque` was merged into this tool (Mar 2026) |
| `get_current_behavior` | rosgraph.Master.getSystemState() + `/walking/enable_control` param | checks node names for behavior patterns |
| `get_walking_state` | `/walking/state` (GetWalkingState.srv), `/walking/param` (GetWalkingParam.srv), `/walking/offset` (GetWalkingOffset.srv) | all read-only service calls |
| `get_latest_detections` | `/object_detect/objects` (ObjectsInfo), `/color_detect/objects` (ObjectsInfo), `/tag_detections` (AprilTagDetectionArray) | subscribe 1 msg, no publish |
| `read_recent_ros_logs` | `/rosout_agg` (rosgraph_msgs/Log) | collects 3s of messages, filterable by level + node |
| `read_last_run_summary` | `/root/.ros/log/<session>/*.log` (file read, mounted ro from ainex) | Finds newest session dir by mtime (bypasses broken `latest` symlink); runs `summarize_ros_logs.py` to produce condensed Markdown report (~5-15KB); deduplicates ROSA polling, camera restarts; checks `expected_nodes.yaml` for missing nodes |

## Disabled Write Tools (Phase 1)

| Tool | Would call | Guard |
|------|-----------|-------|
| `stop_current_behavior` | `/walking/command "stop"` | stub returns disabled msg |
| `stand_safe` | `/walking/command "stand"` | stub returns disabled msg |

Enable in Phase 2: set `AINEX_WRITE_ENABLED=true` + implement real logic in `disabled.py`.

## Dockerfile Key Points

- Base: `ubuntu:20.04`
- Python 3.9 via `ppa:deadsnakes/ppa`
- ROS Noetic via apt (`ros-noetic-ros-base` + user-space Python tools)
- ROS Python packages exposed to Python 3.9 via `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages`
- `ainex_interfaces` **and** `ros_robot_controller` both built with `catkin_make --pkg ainex_interfaces ros_robot_controller` in `/opt/ainex_msgs_ws/`
  - `ros_robot_controller` source: `COPY ros_ws_src/ainex_driver/ros_robot_controller`
  - Generated pure-Python stubs added to PYTHONPATH
  - Build context is `docker/` so both COPY paths work
  - **Bug fixed Mar 2026**: omitting `ros_robot_controller` caused `No module named 'ros_robot_controller'` when health tool called `GetBusServoState` service
- ROSA installed from `vendor/rosa/` if present, else from PyPI (`jpl-rosa`)
- **Vendor detection fix (Mar 24 2026)**: Old Dockerfile checked `vendor/rosa/src` (pip layout) but vendor is flat (`vendor/rosa/rosa.py`). Fixed detection to check `rosa.py` directly + added `ENV PYTHONPATH="/opt/rosa-agent/vendor:${PYTHONPATH}"` so vendor ROSA takes priority over PyPI
- `summarize_ros_logs.py` COPY'd into `/opt/rosa-agent/` (added Mar 14 2026)
- `ainex_agent_tools` installed as editable package (`pip install -e /opt/rosa-agent`)

## Desktop Shortcut

`/home/pi/Desktop/rosa_agent.desktop` — double-click to launch ROSA in a host terminal.
Calls `/home/pi/docker/run_rosa.sh` which:
1. Checks `ainex` container is running
2. Checks ROS master is reachable (must source `/opt/ros/noetic/setup.bash` before the `python3` check — bug fixed Mar 2026)
3. Runs `docker compose run --rm rosa-agent python3.9 ainex_agent.py`

**Known bug (fixed)**: original `run_rosa.sh` ran `docker exec ainex python3 -c "import rosgraph ..."` without sourcing ROS, causing false "ROS master not responding" error even when master was running. Fix: use `bash -c "source /opt/ros/noetic/setup.bash && python3 ..."`.

**Desktop shortcut fix (Mar 2026)**: `Terminal=true` in the `.desktop` file did not run the script — terminal opened but dropped to a bare shell prompt. Fix: set `Terminal=false` and invoke `lxterminal` explicitly:
```
Exec=lxterminal -e bash -c "bash /home/pi/docker/run_rosa.sh; read -p 'Press Enter to close...'"
```
The trailing `read` keeps the window open after ROSA exits.

## After Reboot — How to Start ROSA

Prerequisites: `ainex` container must be running with roscore up.

```bash
# 1. Confirm ainex is running
docker ps | grep ainex

# 2. Start ROSA interactive CLI
cd /home/pi/docker
docker compose run --rm rosa-agent python3.9 ainex_agent.py
```

Then type queries at the `You:` prompt, e.g.:
- `List the currently running ROS nodes`
- `What is the robot's battery voltage?`
- `What is the robot's current health status?`

```bash
# Single query (non-interactive)
docker compose run --rm rosa-agent python3.9 ainex_agent.py \
  --query "List the currently running ROS nodes"

# Verify ROS master reachable (sanity check)
docker compose run --rm rosa-agent python3.9 -c "
import rosgraph; m = rosgraph.Master('/test'); print('PID:', m.getPid())"
```

No rebuild needed after reboot — image `ainex-rosa-agent:latest` persists.

## One-Time Setup (already done)

```bash
# .env is configured at /home/pi/docker/rosa-agent/.env
# LLM_PROVIDER=openai, OPENAI_MODEL=gpt-4o, ROS_MASTER_URI=http://127.0.0.1:11311

# Build (only needed after Dockerfile changes)
cd /home/pi/docker && docker compose build rosa-agent
```

## Build & Run (full reference)

```bash
# Run interactive CLI
docker compose run --rm rosa-agent python3.9 ainex_agent.py

# Single query
docker compose run --rm rosa-agent python3.9 ainex_agent.py --query "Battery status?"

# Background service
docker compose up -d rosa-agent
docker compose logs -f rosa-agent
docker compose stop rosa-agent
```

## LLM Providers (llm_config.py)

- `LLM_PROVIDER=ollama` (default) — needs Ollama on host, `OLLAMA_BASE_URL`, `OLLAMA_MODEL`
- `LLM_PROVIDER=openai` — needs `OPENAI_API_KEY`, `OPENAI_MODEL`
- `LLM_PROVIDER=azure` — needs `AZURE_OPENAI_*` vars

## Bug Fixes (Mar 2026)

- **`rosservice_list` crash with `include_nodes=True`**: ROSA built-in `ros1.py` `rosservice_list()` called `.startswith()` on list items when `include_nodes=True` (items are `[service_name, [nodes]]` not strings). Fixed in `vendor/rosa/tools/ros1.py` with `_name()` helper. Must be copied into container at `/usr/local/lib/python3.9/dist-packages/rosa/tools/ros1.py` after rebuild.
- **`get_servo_torque` merged into `get_robot_health`**: Separate `servos.py` tool removed; `get_robot_health` now queries all 24 servos (was 10 representative) with temperature, voltage, and torque state.
- **`rostopic_echo` actual_count bug (Mar 24)**: `return_echoes` default changed from `False` to `True` in `vendor/rosa/tools/ros1.py` — fixes bug where received messages weren't returned (actual_count always 0 because messages were collected but discarded)
- **`max_execution_time` support (Mar 24)**: Added `max_execution_time` param to vendor `rosa.py`, passed through to LangChain `AgentExecutor`; set to 60s in `ainex_agent.py` to prevent runaway queries
- **Dockerfile vendor detection (Mar 24)**: Fixed `vendor/rosa/src` check → `vendor/rosa/rosa.py`; added `PYTHONPATH` prepend so vendor ROSA overrides PyPI
- **Prompts rewritten (Mar 24)**: `critical_instructions` now says "use custom tools FIRST"; `about_your_environment` rewritten with verified topic names and full 24-DOF servo layout; `about_your_capabilities` uses PRIORITY 1 (custom) / PRIORITY 2 (built-in) structure

## Known Risks / Phase 2 TODOs

- Python 3.9 + ROS Noetic: pure Python ROS packages work; C extensions (roslz4) may not but not critical
- ainex_interfaces compiled at build time in container (not from ainex container's devel/)
- Phase 2: implement real stop_current_behavior / stand_safe after write audit
- Phase 2: vision streaming diagnostics (/camera/image_raw)
- Phase 2: ainex_behavior BT state query
