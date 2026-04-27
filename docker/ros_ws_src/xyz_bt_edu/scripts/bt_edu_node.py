#!/usr/bin/env python3
"""
Example: Run a behavior tree using xyz_bt_edu.

This demo builds a simple safety-patrol tree:

    Sequence(root)
    ├── L1_Battery_IsVoltageOk     (guard: skip if battery low)
    └── Selector(patrol)
        ├── Sequence(line_follow)
        │   ├── L1_Vision_IsLineDetected
        │   └── L2_Gait_FollowLine
        └── L2_Gait_Stop            (fallback: stop if no line)

Usage (inside container, with roscore + hardware running):
    rosrun xyz_bt_edu bt_edu_node.py

Or standalone test (just verifies the framework, no hardware):
    rosrun xyz_bt_edu bt_edu_node.py _mode:=dry_run
"""
import sys
import os
import rospy
import py_trees

from xyz_bt_edu import XyzBTRunner, BlackboardROSBridge, BB
from xyz_bt_edu.behaviours.L1_perception import (
    L1_Battery_IsVoltageOk,
    L1_Vision_IsLineDetected,
)
from xyz_bt_edu.behaviours.L2_locomotion import (
    L2_Gait_FollowLine,
    L2_Gait_Stop,
)


def build_dry_run_tree():
    """Minimal tree for testing the framework without hardware.
    Uses only BB reads — no subscribers, no services."""
    from xyz_bt_edu.behaviours.L1_perception import L1_Balance_IsStanding

    # Pre-populate blackboard
    py_trees.blackboard.Blackboard.storage[BB.ROBOT_STATE] = 'stand'

    root = py_trees.composites.Sequence(name='DryRunTest', memory=False,
                                        children=[L1_Balance_IsStanding()])
    return root


def build_patrol_tree(gait_manager, visual_patrol):
    """Full patrol tree with battery guard + line following."""
    battery_ok = L1_Battery_IsVoltageOk(threshold_mv=10000)
    line_detected = L1_Vision_IsLineDetected()
    follow_line = L2_Gait_FollowLine(visual_patrol)
    stop = L2_Gait_Stop(gait_manager)

    line_follow_seq = py_trees.composites.Sequence(
        name='LineFollow', memory=False,
        children=[line_detected, follow_line])

    patrol = py_trees.composites.Selector(
        name='Patrol', memory=False,
        children=[line_follow_seq, stop])

    root = py_trees.composites.Sequence(
        name='SafePatrol', memory=False,
        children=[battery_ok, patrol])

    return root


def main():
    rospy.init_node('bt_edu_node')
    mode = rospy.get_param('~mode', 'patrol')

    if mode == 'dry_run':
        # --- Dry-run: no hardware needed, just proves the framework ---
        rospy.loginfo('[bt_edu_node] DRY RUN mode — no hardware required')
        root = build_dry_run_tree()
        tree = py_trees.trees.BehaviourTree(root)
        runner = XyzBTRunner(tree)

        # Start ROSA bridge (publishes BB state to /bt/bb/*)
        bridge = BlackboardROSBridge()
        bridge.start(rate_hz=2)

        rospy.loginfo('[bt_edu_node] Running tree...')
        runner.spin(rate_hz=10)
        rospy.loginfo('[bt_edu_node] Tree finished.')

    else:
        # --- Full patrol: requires roscore + bringup ---
        rospy.loginfo('[bt_edu_node] PATROL mode — needs hardware/bringup')

        # Import heavy dependencies only when needed
        from ainex_kinematics.gait_manager import GaitManager
        from ainex_kinematics.motion_manager import MotionManager

        # VisualPatrol is in hurocup2025 package
        import rospkg
        _path = rospkg.RosPack().get_path('hurocup2025')
        sys.path.insert(0, os.path.join(_path, 'scripts', 'marathon'))
        from visual_patrol import VisualPatrol

        gait_manager = GaitManager()
        motion_manager = MotionManager()
        visual_patrol = VisualPatrol(gait_manager)

        # Init blackboard with defaults
        py_trees.blackboard.Blackboard.storage[
            BB.ROBOT_STATE] = 'stand'
        py_trees.blackboard.Blackboard.storage[
            BB.LINE_DATA] = None

        root = build_patrol_tree(gait_manager, visual_patrol)
        tree = py_trees.trees.BehaviourTree(root)
        runner = XyzBTRunner(tree)

        bridge = BlackboardROSBridge()
        bridge.start(rate_hz=10)

        motion_manager.run_action('walk_ready')
        rospy.loginfo('[bt_edu_node] Running patrol tree...')
        runner.spin(rate_hz=30)


if __name__ == '__main__':
    main()
