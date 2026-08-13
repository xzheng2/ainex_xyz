#!/usr/bin/env python3
"""Infrastructure communication manifest for {{PROJECT_CLASS}} BT node.

Defines and writes a static JSON manifest that declares all non-business
ROS interfaces belonging to {{PROJECT_CLASS}} node infrastructure components.
These interfaces are excluded from the _RuntimeIO business log.

Node composition (confirm before modifying):
  - tree/{{PROJECT}}_bt.py wires xyz_bt_lib standard nodes
  - behaviours/actions.py nodes inherit xyz_bt_lib.core.base_node.XyzL2ActionNode
  - runtime/runtime_facade.py inherits xyz_bt_lib.core.base_facade.XyzBTFacade
  - runtime/_runtime_io.py is the sole raw ROS egress layer

Output: log/infra_comm_manifest_lastrun.json
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
    def _abs(name):
        # '~bt/run' -> '/<node_name>/bt/run'. The leading slash is not cosmetic: without
        # it the value is a RELATIVE name, which resolves against the caller's namespace
        # rather than naming the node's own interface. Every target here was missing it.
        return '/' + node_name + '/' + name.lstrip('~').lstrip('/')

    return [
        # ── BT execution controller ──────────────────────────────────────
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'bt_observability/bt_exec_controller.py',
            'comm_type':  'service_server',
            'direction':  'in',
            'target':     _abs('~bt/run'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Set BT mode to RUN',
        },
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'bt_observability/bt_exec_controller.py',
            'comm_type':  'service_server',
            'direction':  'in',
            'target':     _abs('~bt/pause'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Set BT mode to PAUSE',
        },
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'bt_observability/bt_exec_controller.py',
            'comm_type':  'service_server',
            'direction':  'in',
            'target':     _abs('~bt/step'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Queue one tick then PAUSE',
        },
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'bt_observability/bt_exec_controller.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     _abs('~bt/mode'),
            'ros_node':   node_name,
            'payload':    {'mode': '<RUN|PAUSE|STEP>'},
            'summary':    'Current BT execution mode (latched)',
        },
        # ── BB ROS bridge — project-specific ────────────────────────────
        {
            'component':  '{{PROJECT_CLASS}}BBBridge',
            'layer':      'infra',
            'file':       'infra/bb_ros_bridge.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     '/bt/{{PROJECT}}/bb/*',
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Project-local BB keys for ROSA (10 Hz)',
        },
        # ── BB ROS bridge — shared (xyz_bt_lib) ─────────────────────────
        {
            'component':  'BlackboardROSBridge',
            'layer':      'infra',
            'file':       'xyz_bt_lib/blackboard/bb_ros_bridge.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     '/bt/bb/latched/*',
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Shared BB keys for ROSA (robot_state, tracked_objects, detection_source, ...) at 10 Hz',
        },
        # ── Tree publisher ───────────────────────────────────────────────
        {
            'component':  'TreeROSPublisher',
            'layer':      'infra',
            'file':       'bt_observability/tree_publisher.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     _abs('~log/tree'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'py_trees_msgs BehaviourTree for rqt_py_trees',
        },
        {
            'component':  'TreeROSPublisher',
            'layer':      'infra',
            'file':       'bt_observability/tree_publisher.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     _abs('~ascii/snapshot'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'unicode ascii tree display after each tick (std_msgs/String)',
        },
        {
            'component':  'TreeROSPublisher',
            'layer':      'infra',
            'file':       'bt_observability/tree_publisher.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     _abs('~tip'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'currently executing tip node after each tick (py_trees_msgs/Behaviour)',
        },
        # ── Runtime layers ───────────────────────────────────────────────
        {
            'component':  '{{PROJECT_CLASS}}RuntimeFacade',
            'layer':      'runtime_facade',
            'file':       'runtime/runtime_facade.py',
            'comm_type':  'none',
            'direction':  'none',
            'target':     'n/a',
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Public BT-facing runtime interface (XyzBTFacade)',
        },
        {
            'component':  '_RuntimeIO',
            'layer':      'runtime_io',
            'file':       'runtime/_runtime_io.py',
            'comm_type':  'multiple',
            'direction':  'out',
            'target':     'gait_manager / motion_manager / buzzer_pub',
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Sole raw ROS / manager egress; sole ros_out log outlet',
        },
    ]


def write_infra_manifest(records: list, log_dir: str) -> None:
    """Write the infra manifest JSON to log_dir.

    Args:
        records:  Output of build_infra_manifest().
        log_dir:  Directory to write infra_comm_manifest_lastrun.json.
    """
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, 'infra_comm_manifest_lastrun.json')
    with open(path, 'w') as f:
        json.dump({
            'written_at': time.time(),
            'interfaces': records,
        }, f, indent=2)
    import rospy
    rospy.loginfo(f'[infra_manifest] written: {path}')
