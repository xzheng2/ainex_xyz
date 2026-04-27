# xyz-bt-facade-project skill

**This is the only active project template skill.**
`xyz-bt-project` is deprecated and archived — do not use it as a reference or generation baseline.

## Trigger phrases

新建BT项目, 创建比赛行为树, scaffold bt project, create a new BT project,
set up a competition BT, 新建 <name> 项目, scaffold facade project

## Architecture

```
L2 nodes
  └─ call_facade() → XyzBTFacade (abstract contract)
                          └─ {{PROJECT_CLASS}}RuntimeFacade (implementation)
                                └─ profile cfg merge (go_cfg / turn_cfg)
                                └─ convenience wrappers (go_step / turn_step / move_head)
                                └─ primitive passthrough (all 7 primitives)
                                      └─ _RuntimeIO (sole raw ROS / manager egress)
                                            └─ gait_manager / motion_manager / publishers
```

Execution chain in bt_node.py:
```
_RuntimeIO(gait_manager, motion_manager, buzzer_pub, ...)
  └─ RuntimeFacade(runtime_io, go_cfg=_GO_CFG, turn_cfg=_TURN_CFG, ...)
        └─ bootstrap(runtime_facade)
```

## Public contract (XyzBTFacade)

Primitives — implemented by RuntimeFacade, executed by _RuntimeIO:
- `disable_gait()` — shut down gait controller (needs enable_gait() to restart)
- `enable_gait()` — start/restart gait controller
- `stop_gait()` — stop current motion; controller stays up for new commands
- `set_step(dsp, x, y, yaw, ...)` — send one fully-resolved gait step
- `run_action(action_name)` — execute a named stand-alone motion action
- `set_servos_position(duration_ms, positions)` — command servos directly
- `publish_buzzer(freq, on_time, off_time, repeat)` — scalar params only; _RuntimeIO constructs BuzzerState

Convenience wrappers — implemented by RuntimeFacade:
- `go_step(x, y, yaw)` — merges go_cfg (static) + x/y/yaw (per-tick) → set_step(motion_profile='go')
- `turn_step(x, y, yaw)` — merges turn_cfg + x/y/yaw → set_step(motion_profile='turn')
- `move_head(pan_pos)` — maps to set_servos_position with HEAD_PAN_SERVO + HEAD_MOVE_DELAY_MS

## Output file structure

```
{{PROJECT}}/
  runtime/
    runtime_facade.py    ← {{PROJECT_CLASS}}RuntimeFacade(XyzBTFacade)
    _runtime_io.py       ← _RuntimeIO (sole raw ROS egress)
  tree/
    {{PROJECT}}_bt.py    ← bootstrap(runtime_facade, ...)
  app/
    {{PROJECT}}_bt_node.py
  behaviours/
    actions.py           ← project-specific L2 nodes (if needed)
  infra/
    infra_manifest.py
    bb_ros_bridge.py
    bt_exec_controller.py
    tree_publisher.py
  log/                   ← git-tracked, written at runtime
```

Not generated (deprecated paradigm):
- `semantics/semantic_facade.py`
- `comm/comm_facade.py`

## go_cfg / turn_cfg shape

Keys are the **static** parameters of `_RuntimeIO.set_step()` / `gait_manager.set_step()`:

```python
_GO_CFG = {
    'dsp':        0.1,    # double-support phase duration (s) — tune per project
    'gait_param': None,   # GaitParam msg or None (use controller default)
    'arm_swap':   False,
    'step_num':   1,
}
```

Per-tick dynamic params (`x`, `y`, `yaw`) come from the L2 node's `call_facade()` call.
`RuntimeFacade.go_step(x, y, yaw)` merges these at runtime; bt_node never calls _RuntimeIO.

## Templates

| Template | Generates |
|---|---|
| `runtime_facade.py.tpl` | `{{PROJECT}}/runtime/runtime_facade.py` |
| `_runtime_io.py.tpl`    | `{{PROJECT}}/runtime/_runtime_io.py` |
| `bt_node.py.tpl`        | `{{PROJECT}}/app/{{PROJECT}}_bt_node.py` |
| `project_bt.py.tpl`     | `{{PROJECT}}/tree/{{PROJECT}}_bt.py` |
| `actions.py.tpl`        | `{{PROJECT}}/behaviours/actions.py` |
| `infra_manifest.py.tpl` | `{{PROJECT}}/infra/infra_manifest.py` |

Reuse from `xyz-bt-project` (unchanged templates):
- `bb_ros_bridge.py.tpl`
- `bt_exec_controller.py.tpl`
- `tree_publisher.py.tpl`
- `bringup.launch.tpl`
- `ros_msg_utils.py.tpl`

## Enforcement rules

1. L2 nodes and tree code: call_facade() only — never touch _RuntimeIO, managers, or raw ROS
2. RuntimeFacade: holds _RuntimeIO; does NOT hold gait_manager / motion_manager / publishers
3. _RuntimeIO: the ONLY class that calls gait_manager, motion_manager, or ROS publishers
4. publish_buzzer(): scalar params in public contract; BuzzerState constructed inside _RuntimeIO
5. Input adapters (ImuBalanceStateAdapter, etc.): inbound ROS only; outbound goes via _RuntimeIO
6. All L2 nodes must declare BB_READS, BB_WRITES, FACADE_CALLS, CONFIG_DEFAULTS
