"""Pre-write reminder texts. No judgement, no content inspection.

These are what the two PreToolUse reminder guards emit. They evaluate nothing, which is
why their events are logged as `reminder` rather than `pass` -- calling them a passed
check would claim a check that never ran.

The _FILE_TYPE_HINTS keys are ALSO a path classification, the fourth one; the selection
order lives in path_spec._HINT_ORDER, which differs from the category order for two
pairs. Look there before assuming dict order here is meaningful.
"""
from __future__ import annotations

NODE_REMINDER = (
    "[xyz_bt_lib node guard · pre-write reminder]\n"
    "Detected write to xyz_bt_lib behaviours/L*/... BT node file.\n"
    "Ensure content complies with xyz_bt_lib node conventions (xyz-bt-lib-node skill):\n"
    "  1. class L1_Foo(XyzL1ConditionNode) / L2_Foo(XyzL2ActionNode) / L3_Foo(XyzL3ActionNode)  — correct base\n"
    "  2. LEVEL = 'L1'|'L2'|'L3'          — class variable declaring node level\n"
    "  3. BB_READS / BB_WRITES / FACADE_CALLS / CONFIG_DEFAULTS declared\n"
    "  4. setup() calls super().setup(**kwargs)\n"
    "  5. Do not import rospy or call rospy.*()  — use self._logger.emit_bt() instead\n"
    "Run the xyz-bt-lib-node skill first if you need a template."
)

ADAPTER_REMINDER = (
    "[xyz_bt_lib adapter guard · pre-write reminder]\n"
    "Detected write to xyz_bt_lib adapters/... adapter file.\n"
    "Ensure content complies with the two-phase lock protocol (xyz-bt-lib-adapter skill):\n"
    "  1. snapshot_and_reset()          — Phase 1: call inside with lock:, return snapshot and reset received_count\n"
    "  2. write_snapshot(snap, tick_id) — Phase 2: call after releasing lock, write BB + emit ros_in/input_state event\n"
    "  3. rospy.Subscriber created only in __init__()\n"
    "  4. Initial BB write in __init__() (ensures keys exist before first tick, prevents KeyError in BT nodes)\n"
    "Run the xyz-bt-lib-adapter skill first if you need a template."
)

FILE_TYPE_HINTS = {
    'bt_ros_io/default_runtime_io.py': (
        '[xyz_behavior guard · bt_ros_io/default_runtime_io.py — SHARED by every project]\n'
        '  1. BuzzerState must be imported from ros_robot_controller.msg (not ainex_interfaces.msg)\n'
        '  2. Class name must be _RuntimeIO\n'
        '  3. This is the ONLY class holding gait_manager / motion_manager / publishers\n'
        '  4. To retune _GO_STEP_VEL / _TURN_STEP_VEL for ONE project, do NOT edit this\n'
        '     file — subclass it in <project>/runtime/_runtime_io.py'
    ),
    'bt_ros_io/runtime_facade.py': (
        '[xyz_behavior guard · bt_ros_io/runtime_facade.py — SHARED by every project]\n'
        '  1. Class XyzRuntimeFacade must inherit XyzBTFacade and implement every abstract method\n'
        '  2. Must NOT hold gait_manager / motion_manager / publishers — all ROS I/O via self._io\n'
        '  3. Every per-project knob is a CONSTRUCTOR ARGUMENT (go_cfg / turn_cfg /\n'
        '     yaw_avg_window), never a class constant — that is what lets one class serve\n'
        '     every project. Do not hardcode project values here, and do not add a class\n'
        '     constant that a project would have to subclass to change\n'
        '  4. Nothing subclasses this. There is no <project>/runtime/runtime_facade.py'
    ),
    'runtime/_runtime_io.py': (
        '[xyz_behavior guard · <project>/runtime/_runtime_io.py — OPTIONAL subclass]\n'
        '  1. Write this file ONLY to retune _GO_STEP_VEL / _TURN_STEP_VEL. Never a full copy\n'
        '  2. from bt_ros_io.default_runtime_io import _RuntimeIO as _BaseRuntimeIO\n'
        '     class _RuntimeIO(_BaseRuntimeIO):  — the alias is required, the subclass keeps the name\n'
        '  3. BuzzerState must be imported from ros_robot_controller.msg (not ainex_interfaces.msg)\n'
        '  4. The base lives at xyz_behavior/bt_ros_io/default_runtime_io.py'
    ),
    '_groot.xml': (
        '[xyz_behavior guard · tree/*_groot.xml]\n'
        '  1. Co-authoritative with companion *_bt.py — you wrote this file, so update *_bt.py to match\n'
        '  2. <root main_tree_to_execute="{{PROJECT}}"> must match BehaviorTree ID\n'
        '  3. <TreeNodesModel> must declare every node ID used in the tree'
    ),
    'tree/': (
        '[xyz_behavior guard · tree/*_bt.py]\n'
        '  1. Must define bootstrap(facade, logger=None, tick_id_getter=None, ...) function\n'
        '  2. Nodes with CONFIG_DEFAULTS must expand to explicit kwargs + "# CONFIG_DEFAULTS — override to tune:" comment\n'
        '  3. For L1/L2 nodes, pass logger+tick_id_getter via kw and facade via kw_f\n'
        '  4. See skill templates: project_bt.py.tpl (tree wiring) + project_bt_groot.xml.tpl (Groot XML)\n'
        '  5. Co-authoritative with companion _groot.xml — you wrote this file, so update _groot.xml to match'
    ),
    'app/': (
        '[xyz_behavior guard · app/*_bt_node.py]\n'
        '  1. Must define main() function\n'
        "  2. Log dir: _LOG_DIR = os.path.join(_PKG_PATH, 'log') (shared dir, not per-project subdir)\n"
        '  3. Two-phase lock: with lock: snap = adapter.snapshot_and_reset(); ...; adapter.write_snapshot(snap, tick_id)\n'
        '  4. See skill template assets/templates/bt_node.py.tpl'
    ),
    'launch/': (
        '[xyz_behavior guard · launch/*.launch]\n'
        '  1. node name must match PROJECT name (no _bt_node suffix)\n'
        '  2. type points to script under scripts/\n'
        '  3. Must be valid XML\n'
    ),
    'scripts/': (
        '[xyz_behavior guard · scripts/*_bt_node.py]\n'
        '  1. Thin entry file (3-5 lines)\n'
        '  2. Add PKG_PATH to sys.path\n'
        '  3. Import main from the corresponding app module\n'
        '  4. See scripts/approachhoop_bt_node.py for reference'
    ),
    'behaviours/': (
        '[xyz_behavior guard · behaviours/actions.py]\n'
        '  1. Each behaviour class must inherit XyzL2ActionNode\n'
        '  2. Must have class variables LEVEL and BB_LOG_KEYS\n'
        '  3. Must not call rospy.*() directly\n'
        '  4. File may be empty if no custom behaviour nodes are needed'
    ),
}

#: Emitted when a path matches xyz_behavior/ but no specific hint key.
DEFAULT_BEHAVIOR_HINT = (
    '[xyz_behavior guard · pre-write reminder]\n'
    'Detected write to xyz_behavior/ project file.\n'
    'Ensure compliance with the xyz-bt-facade-project skill.\n'
    'Consult the skill templates if in doubt.'
)
