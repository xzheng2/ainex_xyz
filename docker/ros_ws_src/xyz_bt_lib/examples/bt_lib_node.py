#!/usr/bin/env python3
"""
Example: Run a behavior tree using xyz_bt_lib.

This demo builds a simple safety-patrol tree with the semantic composite
factories (never raw Sequence/Selector(memory=...)):

    ReactiveSequence(SafePatrol)
    ├── L1_Balance_IsStanding        (guard: only patrol while standing)
    └── PrioritySelector(Patrol)
        ├── ReactiveSequence(LineFollow)
        │   ├── L1_Vision_IsObjectDetected
        │   └── L2_Gait_FollowTarget
        └── L2_Motion_StopGait       (fallback: stop if no line)

Nodes are imported by full module path (the behaviours/L*/__init__.py packages
do not re-export). L2 action nodes take a facade (XyzBTFacade); a project
supplies its own concrete RuntimeFacade, and StubFacade is a no-op stand-in for
offline structure checks.

Usage (inside container, with roscore running):
    rosrun xyz_bt_lib bt_lib_node.py                 # dry_run: ticks an L1-only tree
    rosrun xyz_bt_lib bt_lib_node.py _mode:=patrol   # builds + prints the patrol tree
"""
import rospy
import py_trees

from xyz_bt_lib import (
    XyzBTRunner, BlackboardROSBridge, BB,
    ReactiveSequence, PrioritySelector, StubFacade, render_unicode,
)
from xyz_bt_lib.behaviours.L1_perception.L1_Balance_IsStanding import (
    L1_Balance_IsStanding,
)
from xyz_bt_lib.behaviours.L1_perception.L1_Vision_IsObjectDetected import (
    L1_Vision_IsObjectDetected,
)
from xyz_bt_lib.behaviours.L2_locomotion.L2_Gait_FollowTarget import (
    L2_Gait_FollowTarget,
)
from xyz_bt_lib.behaviours.L2_locomotion.L2_Motion_StopGait import L2_Motion_StopGait


def build_dry_run_tree():
    """Minimal tree for testing the framework without hardware or a facade.
    Uses only a BB read — no subscribers, no services, no facade calls."""
    # Pre-populate the blackboard so the L1 read succeeds.
    py_trees.blackboard.Blackboard.storage[BB.ROBOT_STATE] = 'stand'

    return ReactiveSequence('DryRunTest', children=[L1_Balance_IsStanding()])


def build_patrol_tree(facade, logger=None, tick_id_getter=None):
    """Full safety-patrol tree. `facade` is any XyzBTFacade (StubFacade offline)."""
    line_follow = ReactiveSequence('LineFollow', children=[
        L1_Vision_IsObjectDetected(target_id='line', logger=logger,
                                   tick_id_getter=tick_id_getter),
        L2_Gait_FollowTarget(target_id='line', facade=facade, logger=logger,
                             tick_id_getter=tick_id_getter),
    ])

    patrol = PrioritySelector('Patrol', children=[
        line_follow,
        L2_Motion_StopGait(facade=facade, logger=logger, tick_id_getter=tick_id_getter),
    ])

    return ReactiveSequence('SafePatrol', children=[
        L1_Balance_IsStanding(logger=logger, tick_id_getter=tick_id_getter),
        patrol,
    ])


def main():
    rospy.init_node('bt_lib_node')
    mode = rospy.get_param('~mode', 'dry_run')

    if mode == 'dry_run':
        # No hardware / facade needed — just proves the framework runs.
        rospy.loginfo('[bt_lib_node] DRY RUN mode — no hardware required')
        tree = py_trees.trees.BehaviourTree(build_dry_run_tree())
        runner = XyzBTRunner(tree)

        # Mirror BB state to /bt/bb/* for ROSA.
        BlackboardROSBridge().start(rate_hz=2)

        rospy.loginfo('[bt_lib_node] Running tree...')
        runner.spin(rate_hz=10)
        rospy.loginfo('[bt_lib_node] Tree finished.')

    else:
        # The patrol tree drives gait via L2 nodes, so it needs a project
        # RuntimeFacade (XyzBTFacade) plus the input adapters that populate the
        # /latched/ line-detection keys. xyz_bt_lib is portable and ships no
        # concrete facade, so here we build the tree with StubFacade (a no-op:
        # it issues NO motion) and print its structure. To actually run it,
        # pass your project's RuntimeFacade to build_patrol_tree() instead.
        rospy.loginfo('[bt_lib_node] PATROL mode — structure only (StubFacade, '
                      'no motion). Supply a project RuntimeFacade to run for real.')
        root = build_patrol_tree(StubFacade())
        rospy.loginfo('[bt_lib_node] patrol tree:\n%s', render_unicode(root))


if __name__ == '__main__':
    main()
