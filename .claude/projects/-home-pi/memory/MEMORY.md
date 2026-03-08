# Claude Memory - Ainex Robot (Raspberry Pi)

## Projects

### Ainex Humanoid Robot
- Robot runs ROS Noetic inside Docker container named `ainex`
- Docker src mount: `/home/pi/docker/src` → `/home/ubuntu/share/src`
- ROS workspace inside container: `/home/ubuntu/ros_ws/`
- Main launch: `roslaunch ainex_bringup bringup.launch`
- **17 ROS packages** — full inventory in `ainex_architecture.md`
- **24 DOF humanoid**: 12 leg + 10 arm + 2 head servos (RS485 via STM32, /dev/ttyAMA0)
- Key walking service: `rosservice call /walking/command "command: 'enable_control'"` then `rosservice call /walking/command "command: 'start'"` — **NOT `/ainex/set_walking_command`** (deprecated, does not exist)
- Servo IDs 1–12 legs (interleaved L/R, odd=L even=R, ankle→hip order), 13–22 arms, 23–24 head — **canonical table in `ainex_truth_spec.md`** (`ainex_architecture.md` servo table is WRONG)
- Gait config: `ainex_driver/ainex_kinematics/config/walking_param.yaml`
- Missing in repo (need to create): `ainex_control` (safety/watchdog), `ainex_perception` (unified vision), `ainex_behavior` (FSM), `ainex_navigation` (gait commander)
- Competition code: `hurocup2025/` (marathon, penalty kick, sprint, triple jump, weight lift)
- Simulation: `ainex_simulations/ainex_gazebo/` + `ainex_description/`, flag `gazebo_sim:=true`

### Ainex Controller GUI
- Source: `/home/ubuntu/software/ainex_controller/main.py`
- **Manual button work in progress** — see `ainex_manual_button.md`

## Topic Files
- `ainex_docker_mount.md` — Planned: mount ros_ws/src to host for direct editing (UIDs match, low risk, steps documented)
- `ainex_manual_button.md` — Manual button in Ainex Controller GUI, ROS walking API, servo IDs
- `ainex_architecture.md` — Full repo inventory, package list, node table, TF tree, config locations, proposed production architecture, MVP launch sequence
- **`ainex_truth_spec.md`** — CANONICAL source of truth: topic table, service table, servo ID table (authoritative)
- `ainex_conflict_matrix.md` — All conflicts between docs, decisions, and dispositions
- `ainex_migration_map.md` — Legacy-to-canonical name mapping
- `ainex_validation_checklist.md` — 24-command acceptance checklist (30 min bringup validation)
