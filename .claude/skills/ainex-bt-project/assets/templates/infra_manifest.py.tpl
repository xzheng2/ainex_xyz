#!/usr/bin/env python3
"""Infrastructure communication manifest for {{PROJECT_CLASS}} BT node.

Defines and writes a static JSON manifest that declares all non-business
ROS interfaces belonging to {{PROJECT_CLASS}} node infrastructure components.
These interfaces are excluded from the Generic ROS Facade business log.

Node composition constraints (confirm before modifying):
  - tree/{{PROJECT}}_bt.py uses ainex_bt_edu.behaviours standard nodes first
  - behaviours/conditions.py nodes inherit ainex_bt_edu.base_node.AinexL1ConditionNode
  - behaviours/actions.py nodes inherit ainex_bt_edu.base_node.AinexL2ActionNode
  - semantic_facade.py inherits ainex_bt_edu.base_facade.AinexBTFacade

Output: {{PROJECT}}/log/infra_comm_manifest_lastrun.json
Written once at node startup, overwriting any previous file.
"""
import json
import os
import time


def build_infra_manifest(node_name: str) -> list:
    """Build the static infra interface list.

    Args:
        node_name: ROS node name (e.g. '{{PROJECT}}_bt'), used to resolve
                   relative topic/service names starting with '~'.

    Returns:
        List of interface record dicts.
    """
    def resolve(name):
        if name.startswith('~'):
            return '/' + node_name + '/' + name[1:]
        return name

    records = [
        # ── TreeROSPublisher ─────────────────────────────────────────────
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
        # ── {{PROJECT_CLASS}}BBBridge ────────────────────────────────────
        # TODO: add entries for each BB key mirrored by {{PROJECT_CLASS}}BBBridge
        {
            "component": "{{PROJECT_CLASS}}BBBridge",
            "kind": "topic_pub",
            "name": "/bt/{{PROJECT}}/bb/robot_state",
            "msg_or_srv_type": "std_msgs/String",
            "purpose": "mirror latched BB key for ROSA diagnostics",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "{{PROJECT_CLASS}}BBBridge",
            "kind": "topic_pub",
            "name": "/bt/{{PROJECT}}/bb/tick_id",
            "msg_or_srv_type": "std_msgs/String",
            "purpose": "mirror BT tick counter for ROSA diagnostics",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        # ── BTExecController ─────────────────────────────────────────────
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
        # ── {{PROJECT_NODE_CLASS}} infra ─────────────────────────────────
        # IMPORTANT: {{PROJECT_NODE_CLASS}} inherits color_common.Common.
        # Common.__init__() creates the following interfaces — not visible in
        # {{PROJECT_NODE_CLASS}} code directly.  Any change to Common.__init__()
        # must be reflected here.
        # Source: ainex_example/src/ainex_example/color_common.py
        #
        # Note: /imu and /object/pixel_coords subscribers are owned by
        # ainex_bt_edu/input_adapters/ adapters (not by {{PROJECT_NODE_CLASS}} directly).
        {
            "component": "ImuBalanceStateAdapter",
            "kind": "topic_sub",
            "name": "/imu",
            "msg_or_srv_type": "sensor_msgs/Imu",
            "purpose": "fall detection — updates robot_state in live store; "
                       "owned by ainex_bt_edu/input_adapters/imu_balance_state_adapter.py",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "LineDetectionAdapter",
            "kind": "topic_sub",
            "name": "/object/pixel_coords",
            "msg_or_srv_type": "ainex_interfaces/ObjectsInfo",
            "purpose": "line detection — updates line_data, last_line_x, camera_lost_count; "
                       "owned by ainex_bt_edu/input_adapters/line_detection_adapter.py",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "{{PROJECT_NODE_CLASS}}",
            "kind": "topic_pub",
            "name": "/ros_robot_controller/set_buzzer",
            "msg_or_srv_type": "ros_robot_controller/BuzzerState",
            "purpose": "buzzer publisher; business calls logged by comm_facade",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "{{PROJECT_NODE_CLASS}}",
            "kind": "topic_pub",
            "name": "/color_detection/update_detect",
            "msg_or_srv_type": "ainex_interfaces/ColorsDetect",
            "purpose": "configure color_detection_node at startup",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "self.detect_pub — created in color_common.Common.__init__()",
        },
        {
            "component": "{{PROJECT_NODE_CLASS}}",
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
            "component": "{{PROJECT_NODE_CLASS}}",
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
            "component": "{{PROJECT_NODE_CLASS}}",
            "kind": "service_server",
            "name": "~enter",
            "resolved_name": resolve("~enter"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "run init_action + call /color_detection/enter",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "created by color_common.Common.__init__()",
        },
        {
            "component": "{{PROJECT_NODE_CLASS}}",
            "kind": "service_server",
            "name": "~exit",
            "resolved_name": resolve("~exit"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "stop + call /color_detection/exit",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "created by color_common.Common.__init__()",
        },
        {
            "component": "{{PROJECT_NODE_CLASS}}",
            "kind": "service_client",
            "name": "/color_detection/enter",
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "activate color_detection_node processing pipeline",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        {
            "component": "{{PROJECT_NODE_CLASS}}",
            "kind": "service_client",
            "name": "/color_detection/exit",
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "deactivate color_detection_node processing pipeline",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },
        # TODO: add any project-specific interfaces here
    ]
    return records


def write_infra_manifest(path: str, items: list) -> None:
    """Write the infra manifest to a JSON file (overwrites on each call)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    doc = {
        "schema_version": "1.0",
        "generated_ts": time.time(),
        "description": "Static {{PROJECT_CLASS}} BT infrastructure communication manifest. "
                       "All items are excluded from the Generic ROS Facade business log.",
        "interfaces": items,
    }
    with open(path, 'w') as f:
        json.dump(doc, f, indent=2)
