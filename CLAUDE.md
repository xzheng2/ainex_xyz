## Environment

**Host**: Raspberry Pi 5, Linux 6.12 rpi-2712, user `pi`, home `/home/pi`, shell `zsh`

### ROS1 Container (`ainex`)
**Image**: `ainex-backup:20260630`, user `ubuntu`, home `/home/ubuntu`

**User**: robot runs as `ubuntu` (pip `--user` pkgs — py_trees, zmq, scipy — live in `/home/ubuntu/.local`); `docker exec ainex` defaults to **root** which can't see them — use `docker exec -u ubuntu ainex` to check runtime deps.

**Bind mounts (host → container)**:
- `/home/pi/docker/ros_ws_src` → `/home/ubuntu/ros_ws/src` (ROS1 workspace source)
- `/home/pi/docker/ros_log` → `/home/ubuntu/.ros/log` (ROS log dir)
- `/home/pi/docker/src` → `/home/ubuntu/share/src`
- `/home/pi/docker/tmp` → `/home/ubuntu/share/tmp`
- `/home/pi/docker/software/lab_tool` → `/home/ubuntu/software/lab_tool`
- `/dev` → `/dev`
- `/tmp/.X11-unix` → `/tmp/.X11-unix`

**`--ipc host` is REQUIRED** — the container shares host `/dev/shm` for zero-copy Gemini 305 camera frames. Container is started manually (not compose); include this flag on recreation.

**ROS workspace**: built inside container at `/home/ubuntu/ros_ws/devel/setup.bash`; source edited on host at `/home/pi/docker/ros_ws_src/`

**Rules**:
- ROS nodes run inside the container, not on the host
- Never assume a Python package is missing until verified inside the container:
  `docker exec ainex python3 -c "import <module>"`
- **Never execute `roslaunch` or `rosrun` commands** (directly or via `docker exec`) to run the robot. Instead, show the command and ask the user to run it themselves. Example: _"Run this to start the node:"_ followed by the command in a code block.

## Learner Scaffolding

Users of `xyz_bt_lib`, `xyz_behavior`, and ROSA may be beginners with no prior BT or ROS
knowledge. When helping these users:

- Assume no BT or ROS knowledge unless the user demonstrates it
- **BT concepts are the default priority** — always scaffold BT first
- **ROS concepts only when triggered**: explain ROS communication, topics, input adapters,
  or message payloads only when the user explicitly mentions ROS comm, input from a ROS node,
  output/publish, or payload — not for general BT questions
- Introduce concepts as they arise naturally — do not front-load everything
- **Prefer persistent file edits over one-time args**: when        
  suggesting a configuration change (e.g. detect colour, shape, area threshold), 
  edit the relevant file (`.launch` default, YAML config, Python constant) rather than passing command-line overrides. 
  One-time args are only appropriate when the user explicitly wants a temporary/test run.      
- Calibrate depth to the active project (see detection table below)

### Active project detection

| Files / names in conversation | User is working on |
|---|---|
| `xyz_bt_lib`, `L1_*`, `L2_*`, `XyzBTNode`, `input_adapters` | Shared BT library (educational) |
| `xyz_behavior`, `RuntimeFacade`, `_RuntimeIO` | Competition behavior project |
| ROSA, `bt_debug_*.jsonl`, `bb_current.json`, `bt_observability` | Runtime debug / observability |

### Behavior tree concepts

A behavior tree is ticked from the root downward on every cycle. Each node
returns one of three statuses: SUCCESS, FAILURE, or RUNNING.

**Sequence** (→) — "do each step in order; stop if any step fails"
All children must succeed for the Sequence to succeed.
Think of it as a recipe: chop → mix → bake. If chopping fails, don't bake.

**Selector** (also called Fallback) (?) — "try each option until one works"
Succeeds as soon as one child succeeds; only moves to the next child on failure.
Think of it as a backup plan: try door → try window → try alarm.

**Parallel** — ticks all children on every tick simultaneously.
Succeeds or fails based on a policy (e.g. succeed when all succeed, or when one succeeds).
Think of it as doing multiple things at the same time and deciding when the group is done.

**Condition leaf (L1 node)** — asks a yes/no question about the world.
Reads from the Blackboard; returns SUCCESS or FAILURE instantly; **never RUNNING**,
no side effects, no cross-tick state (a pure predicate).
Example: "Is the target visible in the camera frame?"
Stability confirmation ("condition held for N ticks") is NOT done inside the L1
node — wrap the condition at the tree layer in `LatchedDwellDecorator`
(`xyz_bt_lib.core.latched_dwell`), which stores its counter in the Blackboard so
it survives a reactive parent's per-tick re-entry.

**Action leaf (L2 node)** — does something that takes time.
May return RUNNING across many ticks while the work is in progress.
Example: "Walk toward the target until close enough."

**Blackboard (BB)** — a shared in-memory key-value store inside the BT process.
Input adapters write sensor data here. L1 nodes read from here. Not ROS.

Robot example — a generic "approach target and act" tree:
```
Selector('MainSel')             ← try each child until one succeeds
  IsTaskDone()                  ← option 1: already done? → exit immediately
  Sequence('ApproachAct')       ← option 2: approach + act sequence
    L1_IsTargetDetected         ←   step 1: target in frame? (gate condition)
    L2_WalkToTarget             ←   step 2: walk toward target (RUNNING while walking)
    LatchedDwell(               ←   step 3: confirm positioned for 5 ticks —
      L1_IsTargetPositioned, 5)      the dwell decorator wraps the pure L1 condition
                                     (L1 itself stays SUCCESS/FAILURE only)
    L2_RunAction('Act')         ←   step 4: execute the task motion
    L2_RunAction('Stand')       ←   step 5: recover to standing
    MarkTaskDone()              ←   step 6: set done flag → Sequence SUCCESS
  L2_PauseAfterTicks('Stop')    ← option 3: no target → stop gait, keep looping
```
Reading: MainSel checks option 1 first. If the task isn't done yet, tries option 2.
Inside option 2, if step 1 fails (target not visible), the whole Sequence fails
and MainSel falls through to option 3 (stop gait). When the target appears, option 2
runs all steps in order.

### ROS basics (explain only when user mentions ROS comm, topics, input/output, or payload)

A ROS **node** is a running program. Nodes communicate via **topics** — named
message channels using publish/subscribe. Any node can publish; any node can subscribe.

A **launch file** starts multiple nodes together with their parameters.

How ROS connects to the BT in this project:
```
Hardware/sensors → ROS topic → Input adapter (xyz_bt_lib/input_adapters/)
  → Blackboard → L1 node → L2 node → RuntimeFacade → _RuntimeIO → ROS topic → Robot
```
Key insight: BT nodes (L1, L2) never touch ROS directly. Input adapter is the sole
ROS subscriber feeding the tree; `_RuntimeIO` is the sole publisher sending commands out.

### Project structure

```
xyz_bt_lib/                    ← portable shared library (no robot-specific code)
  behaviours/L1_*/             ← condition nodes: read BB → SUCCESS/FAILURE
  behaviours/L2_*/             ← action nodes: call facade → SUCCESS/FAILURE/RUNNING
  input_adapters/              ← ROS topic → BB fact writers
  blackboard_keys.py           ← all BB key constants (single source of truth)

xyz_behavior/<proj>/           ← robot-specific competition project
  tree/<proj>_bt.py            ← BT wiring only (Selector/Sequence + node instances)
  app/<proj>_bt_node.py        ← ROS node entry point (starts tree + adapters)
  runtime/runtime_facade.py    ← abstract interface L2 nodes call
  runtime/_RuntimeIO.py        ← sole ROS publisher (all ROS egress here)
  behaviours/                  ← project-specific L2 nodes (calls facade)
  log/                         ← runtime logs: bt_debug_*.jsonl, bb_current.json
```

Why the facade exists: L2 nodes call abstract methods (`go_step()`, `run_action()`, etc.).
Only `_RuntimeIO` publishes to ROS — tree logic is decoupled from the robot's ROS interface.

### Implementation skills

When creating or extending BT nodes in `xyz_bt_lib`, use the `xyz-bt-lib-node` skill (input adapters: `xyz-bt-lib-adapter`).
When scaffolding a new `xyz_behavior` project, use the `xyz-bt-facade-project` skill.

## Git workflow

- **Monorepo, and it must stay one.** `.claude/` and `docker/ros_ws_src/` reference each other
  by hardcoded path in both directions (`xyz_bt_lib/tools/validate_engine.py` → `.claude/skills`,
  `.claude/hooks/*` → `xyz_bt_lib/src/...` path patterns). Splitting it breaks both sides at once —
  never propose it.
- **Trunk-based**: commit straight to `master`. No Git Flow, no release branches. A short-lived
  branch is only for multi-session risky work, or to freeze `master` while an experiment campaign
  runs; delete it after merging.
- **Experiment baselines are annotated tags, not branches** (`abl-<YYYYMMDD>-<campaign>`).
- **`package.xml` version numbers are decorative** — nothing reads them, there is no CI or release
  process. Don't "fix" them or start bumping them.

Operational detail (the pre-push gate, tag commands, staging checklist) is injected by
`.claude/hooks/git_workflow_guard.py` when a git command actually runs — it is not repeated here.
