#!/usr/bin/env python3
"""Infrastructure communication manifest for {{PROJECT_CLASS}} BT node.

Defines and writes a static JSON manifest that declares all non-business
ROS interfaces belonging to {{PROJECT_CLASS}} node infrastructure components.
These interfaces are excluded from the _RuntimeIO business log.

Node composition (confirm before modifying):
  - tree/{{PROJECT}}_bt.py wires xyz_bt_edu standard nodes
  - behaviours/actions.py nodes inherit xyz_bt_edu.base_node.XyzL2ActionNode
  - runtime/runtime_facade.py inherits xyz_bt_edu.base_facade.XyzBTFacade
  - runtime/_runtime_io.py is the sole raw ROS egress layer

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
    def _abs(name):
        return name.replace('~', node_name)

    return [
        # ── BT execution controller ──────────────────────────────────────
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'infra/bt_exec_controller.py',
            'comm_type':  'service_server',
            'direction':  'in',
            'target':     _abs('~/bt/run'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Set BT mode to RUN',
        },
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'infra/bt_exec_controller.py',
            'comm_type':  'service_server',
            'direction':  'in',
            'target':     _abs('~/bt/pause'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Set BT mode to PAUSE',
        },
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'infra/bt_exec_controller.py',
            'comm_type':  'service_server',
            'direction':  'in',
            'target':     _abs('~/bt/step'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Queue one tick then PAUSE',
        },
        {
            'component':  'BTExecController',
            'layer':      'infra',
            'file':       'infra/bt_exec_controller.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     _abs('~/bt/mode'),
            'ros_node':   node_name,
            'payload':    {'mode': '<RUN|PAUSE|STEP>'},
            'summary':    'Current BT execution mode (latched)',
        },
        # ── BB ROS bridge ────────────────────────────────────────────────
        {
            'component':  '{{PROJECT_CLASS}}BBBridge',
            'layer':      'infra',
            'file':       'infra/bb_ros_bridge.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     '/bt/{{PROJECT}}/bb/*',
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'Mirror BB keys for ROSA / debug tools (10 Hz)',
        },
        # ── Tree publisher ───────────────────────────────────────────────
        {
            'component':  'TreeROSPublisher',
            'layer':      'infra',
            'file':       'infra/tree_publisher.py',
            'comm_type':  'topic_publish',
            'direction':  'out',
            'target':     _abs('~/log/tree'),
            'ros_node':   node_name,
            'payload':    {},
            'summary':    'py_trees_msgs BehaviourTree for rqt_py_trees',
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
