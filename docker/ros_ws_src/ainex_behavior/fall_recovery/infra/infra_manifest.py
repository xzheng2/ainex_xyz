#!/usr/bin/env python3
"""Infrastructure communication manifest for fall_recovery BT node.

Defines and writes a static JSON manifest that declares all non-business
ROS interfaces belonging to fall_recovery node infrastructure components.
These interfaces are excluded from the Generic ROS Facade and its business
comm log (bt_ros_comm_debug*.jsonl).

节点组合约束（须在项目创建前确认）：
  - tree/fall_recovery_bt.py 中优先使用 ainex_bt_edu.behaviours 标准节点
  - 项目 behaviours/ 中的节点继承 ainex_bt_edu.base_node.AinexBTNode
  - semantic_facade.py 继承 ainex_bt_edu.base_facade.AinexBTFacade

Output: fall_recovery/log/infra_comm_manifest_lastrun.json
Written once at node startup, overwriting any previous file.
"""
import json
import os
import time


def build_infra_manifest(node_name: str) -> list:
    """Build the static infra interface list.

    Args:
        node_name: ROS node name (e.g. 'fall_recovery_bt'), used to resolve
                   relative topic/service names starting with '~'.

    Returns:
        List of interface record dicts.
    """
    def resolve(name):
        if name.startswith('~'):
            return '/' + node_name + '/' + name[1:]
        return name

    records = [
        # ── TreeROSPublisher ──────────────────────────────────────────────
        {
            "component": "TreeROSPublisher",
            "kind": "topic_pub",
            "name": "~log/tree",
            "resolved_name": resolve("~log/tree"),
            "msg_or_srv_type": "py_trees_msgs/BehaviourTree",
            "purpose": "publish BT tree snapshot for rqt_py_trees visualization",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "TreeROSPublisher",
            "kind": "topic_pub",
            "name": "~ascii/snapshot",
            "resolved_name": resolve("~ascii/snapshot"),
            "msg_or_srv_type": "std_msgs/String",
            "purpose": "publish unicode tree display string",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "TreeROSPublisher",
            "kind": "topic_pub",
            "name": "~tip",
            "resolved_name": resolve("~tip"),
            "msg_or_srv_type": "py_trees_msgs/Behaviour",
            "purpose": "publish currently executing tip node",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        # ── FallRecoveryBBBridge ──────────────────────────────────────────
        {
            "component": "FallRecoveryBBBridge",
            "kind": "topic_pub",
            "name": "/bt/fall_recovery/bb/robot_state",
            "msg_or_srv_type": "std_msgs/String",
            "purpose": "mirror latched BB key for ROSA diagnostics",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "FallRecoveryBBBridge",
            "kind": "topic_pub",
            "name": "/bt/fall_recovery/bb/tick_id",
            "msg_or_srv_type": "std_msgs/String",
            "purpose": "mirror BT tick counter for ROSA diagnostics",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        # ── BTExecController ──────────────────────────────────────────────
        {
            "component": "BTExecController",
            "kind": "topic_pub",
            "name": "~bt/mode",
            "resolved_name": resolve("~bt/mode"),
            "msg_or_srv_type": "std_msgs/String",
            "purpose": "publish current BT exec mode (RUN/PAUSE/STEP), latched",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "BTExecController",
            "kind": "service_server",
            "name": "~bt/run",
            "resolved_name": resolve("~bt/run"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "set BT exec mode to RUN",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "BTExecController",
            "kind": "service_server",
            "name": "~bt/pause",
            "resolved_name": resolve("~bt/pause"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "pause BT ticking",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "BTExecController",
            "kind": "service_server",
            "name": "~bt/step",
            "resolved_name": resolve("~bt/step"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "queue one BT tick then revert to PAUSE",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        # ── FallRecoveryBTNode fixed infra ────────────────────────────────
        # IMPORTANT: FallRecoveryBTNode inherits color_common.Common.
        # Common.__init__() creates additional interfaces not visible in
        # FallRecoveryBTNode directly.  Any change to Common.__init__() must
        # be reflected here.
        # Source: ainex_example/src/ainex_example/color_common.py
        {
            "component": "FallRecoveryBTNode",
            "kind": "topic_sub",
            "name": "/imu",
            "msg_or_srv_type": "sensor_msgs/Imu",
            "purpose": "fall detection — updates robot_state in live store",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "topic_pub",
            "name": "/ros_robot_controller/set_buzzer",
            "msg_or_srv_type": "ros_robot_controller/BuzzerState",
            "purpose": "buzzer publisher (node-level infra object); business calls logged by comm_facade",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "publisher object owned here; actual business emit_comm logging in comm_facade.publish_buzzer",
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "topic_pub",
            "name": "/color_detection/update_detect",
            "msg_or_srv_type": "ainex_interfaces/ColorsDetect",
            "purpose": "detect_pub created by color_common.Common.__init__()",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "self.detect_pub — created in color_common.Common.__init__(); see ainex_example/src/ainex_example/color_common.py",
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "lifecycle_action",
            "name": "walk_ready",
            "msg_or_srv_type": "action",
            "purpose": "initial posture on node startup",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "lifecycle_action",
            "name": "stand",
            "msg_or_srv_type": "action",
            "purpose": "final posture on node shutdown",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_server",
            "name": "~start",
            "resolved_name": resolve("~start"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "enable BT ticking (via Common base class)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "created by color_common.Common.__init__()",
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_server",
            "name": "~stop",
            "resolved_name": resolve("~stop"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "disable BT ticking (via Common base class)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "created by color_common.Common.__init__()",
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_server",
            "name": "~enter",
            "resolved_name": resolve("~enter"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "run init_action (via Common base class)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "created by color_common.Common.__init__()",
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_server",
            "name": "~exit",
            "resolved_name": resolve("~exit"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "stop (via Common base class)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "created by color_common.Common.__init__()",
        },
        # ── color_detection_node service clients (via color_common.Common) ──
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_client",
            "name": "/color_detection/enter",
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "activate color_detection_node (via Common)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_client",
            "name": "/color_detection/exit",
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "deactivate color_detection_node (via Common)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_client",
            "name": "/color_detection/start",
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "start color detection result publishing (via Common)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "FallRecoveryBTNode",
            "kind": "service_client",
            "name": "/color_detection/stop",
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "stop color detection result publishing (via Common)",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
    ]
    return records


def write_infra_manifest(path: str, items: list) -> None:
    """Write the infra manifest to a JSON file (overwrites on each call)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    doc = {
        "schema_version": "1.0",
        "generated_ts": time.time(),
        "description": "Static fall_recovery BT infrastructure communication manifest. "
                       "All items are excluded from the Generic ROS Facade business log.",
        "interfaces": items,
    }
    with open(path, 'w') as f:
        json.dump(doc, f, indent=2)
