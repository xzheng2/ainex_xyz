# ROSA Agent — Ainex Integration Notes

**Rewritten Aug 9 2026** — previous version described the pre-rename
(`ainex_agent.py`/`ainex_agent_tools`) layout and an 11-tool set that no longer
matches; four of those tools (behavior/detections/logs/`read_bt_obs`) are gone
from the registered tool list. Verified against actual source this pass;
deep Dockerfile/bugfix history from the old doc was trimmed rather than
re-verified line-by-line (compressed, not necessarily still 100% accurate on
every historical detail — treat the "Bug Fixes" section as changelog, not spec).

`rosa-agent` is a standalone Docker container running NASA JPL ROSA in
read-only mode, connected to the `ainex` container's ROS graph via host
networking. Image: **`ainex-rosa-agent:latest`** (unchanged despite the
ainex→xyz renames elsewhere — this name wasn't part of that rename).

- ROSA repo: https://github.com/nasa-jpl/rosa · PyPI: `jpl-rosa` (>=1.0.9) · Python 3.9+
- Entry point: **`xyz_agent.py`** (was `ainex_agent.py`)
- Tool package: **`xyz_agent_tools`** (was `ainex_agent_tools`)
- Phase 1: read-only diagnostics only

## Directory Layout (current)

```
/home/pi/docker/
├── docker-compose.yml          # service `rosa-agent`, image ainex-rosa-agent:latest
└── rosa-agent/
    ├── Dockerfile
    ├── requirements.txt
    ├── setup.py
    ├── .env / .env.example
    ├── xyz_agent.py             # main entry (CLI + ROS node mode)
    ├── llm_config.py            # Ollama / OpenAI / Azure
    ├── summarize_ros_logs.py
    ├── runtime/
    │   └── session_logger.py    # new since last write-up, not independently verified
    ├── knowledge/
    │   └── ros_reference.md     # progressive-loaded reference (see knowledge.py)
    ├── xyz_agent_tools/
    │   ├── __init__.py          # exports XYZ_TOOLS list (10 tools, see below)
    │   ├── prompts.py
    │   ├── knowledge.py         # new: appends `[Reference context]` sections from
    │   │                        #   ros_reference.md when a query matches a keyword line
    │   ├── camera.py            # new, not independently verified this pass
    │   ├── bt_analysis/         # shared (langchain-free) helpers for the BT tools:
    │   │                        #   raw_tick.py, raw_cross_tick.py, cross_tick_format.py,
    │   │                        #   lastrun_digest_cli.py (host CLI)
    │   └── tools/
    │       ├── health.py             # get_robot_health
    │       ├── walking.py            # get_walking_state
    │       ├── servo_positions.py    # get_servo_positions
    │       ├── bt_monitor.py         # get_bt_status
    │       ├── bt_tick_analysis.py   # get_bt_tick_raw, analyze_bt_tick
    │       ├── cross_tick_analysis.py# cross_tick_analysis
    │       ├── session_digest.py     # session_digest
    │       └── disabled.py           # stop_current_behavior, stand_safe (stubs)
    ├── config/
    │   ├── readonly.yaml
    │   ├── blacklist.yaml
    │   ├── expected_nodes.yaml
    │   └── python_logging.yaml   # new since last write-up
    └── vendor/rosa/               # patched ROSA source
```

**Gone since the last write-up** (not in current `XYZ_TOOLS` export, and their
old source files — `behavior.py`, `detections.py`, `logs.py`, `bt_obs.py` — are
not imported by `__init__.py` regardless of whether the files still exist on
disk): `get_current_behavior`, `get_latest_detections`, `read_recent_ros_logs`,
`read_last_run_summary`, `read_bt_obs`. `rqt_project_prompt.md` (old config
entry) not re-checked.

## Read-Only Tools (current — 10, from `xyz_agent_tools/__init__.py`)

| Tool | ROS interface (all READ) |
|------|--------------------------|
| `get_robot_health` | `/ros_robot_controller/battery` (mV), `/ros_robot_controller/bus_servo/get_state`, `/imu` (fused) |
| `get_walking_state` | `/walking/state`, `/walking/param`, `/walking/offset` (srv), `/walking/set_param` (topic, best-effort) |
| `get_servo_positions` | `/ros_robot_controller/bus_servo/get_position` (all 24 servos) |
| `get_bt_status` | auto-detects active project via `/{project}/ascii/snapshot` (latched); also reads `/{project}/log/tree` (py_trees_msgs/BehaviourTree) + `/bt/bb/latched/*` (shared) + `/bt/{project}/bb/*` (project keys) |
| `get_bt_tick_raw` / `analyze_bt_tick` | raw JSONL evidence for one tick / staged explain-compare-diagnose scaffold — **live or paused sessions only, no lastrun fallback** |
| `cross_tick_analysis` | multi-tick episode analysis across up to 30 recent ticks — see [[ainex_bt_observability]] |
| `session_digest` | bounded whole-run digest from the lastrun JSONL files |
| `stop_current_behavior`, `stand_safe` | disabled stubs — see below |

## Disabled Write Tools (Phase 1)

Guarded by `XYZ_WRITE_ENABLED` env var (renamed from `AINEX_WRITE_ENABLED`) —
unset/false returns a fixed "disabled in read-only mode" message instead of
calling ROS. To enable: set `XYZ_WRITE_ENABLED=true` in `.env`, remove the
guard in `disabled.py`, implement the real call, update `config/readonly.yaml`.

## Docker Network & Mounts (not re-verified this pass — kept from prior notes)

- Host networking: both `ainex` and `rosa-agent` share the host network stack;
  `rosa-agent` env `ROS_MASTER_URI=http://127.0.0.1:11311`
  (`ROS_HOSTNAME` not needed)
- Ollama: `OLLAMA_BASE_URL=http://localhost:11434`
  (`host.docker.internal` doesn't resolve in host mode)
  Source live-mount: `/home/pi/docker/rosa-agent` → `/opt/rosa-agent`; edits on
  host are instantly live in container, no rebuild needed
- ROS log mount: `/home/ubuntu/.ros/log` (in `ainex`) → host
  `/home/pi/docker/ros_log` → `rosa-agent` reads at `/root/.ros/log:ro`
- BT observability log mount: `docker/ros_ws_src/xyz_behavior/log` →
  `/opt/ainex_bt_log:ro` inside `rosa-agent` (path constant name kept from the
  original mount setup; contents are the current 3 files, see
  [[ainex_bt_observability]])

## Starting ROSA

```bash
# prerequisite: ainex container running with roscore up
docker ps | grep ainex
cd /home/pi/docker
docker compose run --rm rosa-agent python3.9 xyz_agent.py
# single query:
docker compose run --rm rosa-agent python3.9 xyz_agent.py --query "Battery status?"
# background service:
docker compose up -d rosa-agent && docker compose logs -f rosa-agent
```

Desktop shortcut `/home/pi/Desktop/rosa_agent.desktop` → `run_rosa.sh` (checks
`ainex` is running + ROS master reachable, then runs the above). Not
re-verified this pass whether the script itself still references the old
entry-point filename — check `run_rosa.sh` before relying on the shortcut if
things seem stale.

## LLM Providers (`llm_config.py`)
- `LLM_PROVIDER=ollama` (default) / `openai` / `azure` — see `.env.example` for
  the required vars per provider.

## Known gaps in this rewrite
- `runtime/session_logger.py` and `xyz_agent_tools/camera.py` exist but weren't
  read this pass — purpose not documented here yet.
- Dockerfile internals (build steps, vendor detection, exact COPY paths) were
  not re-verified; the old doc's bugfix narratives for those are plausible
  history but unconfirmed against current file contents.
- `config/expected_nodes.yaml` / `blacklist.yaml` / `readonly.yaml` contents
  not re-read; assume still accurate in spirit (read/write tool split) but
  don't rely on exact old field names without checking.
