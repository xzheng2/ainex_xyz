#!/usr/bin/env python3
"""Publish py_trees 2.1.x tree state as py_trees_msgs for rqt_py_trees."""
import uuid

import rospy
import py_trees
import unique_id
import py_trees_msgs.msg as pt_msgs
from std_msgs.msg import String

# Map py_trees 2.1.x Status to py_trees_msgs constants
_STATUS_MAP = {
    py_trees.common.Status.INVALID: pt_msgs.Behaviour.INVALID,
    py_trees.common.Status.RUNNING: pt_msgs.Behaviour.RUNNING,
    py_trees.common.Status.SUCCESS: pt_msgs.Behaviour.SUCCESS,
    py_trees.common.Status.FAILURE: pt_msgs.Behaviour.FAILURE,
}

_BLACKBOX_MAP = {
    py_trees.common.BlackBoxLevel.DETAIL: pt_msgs.Behaviour.BLACKBOX_LEVEL_DETAIL,
    py_trees.common.BlackBoxLevel.COMPONENT: pt_msgs.Behaviour.BLACKBOX_LEVEL_COMPONENT,
    py_trees.common.BlackBoxLevel.BIG_PICTURE: pt_msgs.Behaviour.BLACKBOX_LEVEL_BIG_PICTURE,
    py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX: pt_msgs.Behaviour.BLACKBOX_LEVEL_NOT_A_BLACKBOX,
}

_ZERO_UUID = uuid.UUID(int=0)


def _behaviour_type(behaviour):
    if isinstance(behaviour, py_trees.composites.Sequence):
        return pt_msgs.Behaviour.SEQUENCE
    elif isinstance(behaviour, py_trees.composites.Selector):
        return pt_msgs.Behaviour.SELECTOR
    elif isinstance(behaviour, py_trees.composites.Parallel):
        return pt_msgs.Behaviour.PARALLEL
    elif isinstance(behaviour, py_trees.decorators.Decorator):
        return pt_msgs.Behaviour.DECORATOR
    else:
        return pt_msgs.Behaviour.BEHAVIOUR


def behaviour_to_msg(behaviour):
    """Convert a py_trees 2.1.x Behaviour to py_trees_msgs/Behaviour."""
    msg = pt_msgs.Behaviour()
    msg.name = behaviour.name
    msg.class_name = "{}.{}".format(
        behaviour.__module__, type(behaviour).__name__)
    msg.own_id = unique_id.toMsg(behaviour.id)
    msg.parent_id = unique_id.toMsg(
        behaviour.parent.id if behaviour.parent else _ZERO_UUID)

    # iterate(direct_descendants=True) includes self in py_trees 2.1.6
    msg.child_ids = [
        unique_id.toMsg(child.id)
        for child in behaviour.iterate(direct_descendants=True)
        if child is not behaviour
    ]

    tip = behaviour.tip()
    msg.tip_id = unique_id.toMsg(tip.id if tip else behaviour.id)
    msg.type = _behaviour_type(behaviour)
    msg.blackbox_level = _BLACKBOX_MAP.get(
        behaviour.blackbox_level,
        pt_msgs.Behaviour.BLACKBOX_LEVEL_NOT_A_BLACKBOX)
    msg.status = _STATUS_MAP.get(
        behaviour.status, pt_msgs.Behaviour.INVALID)
    msg.message = behaviour.feedback_message
    msg.is_active = behaviour.status != py_trees.common.Status.INVALID
    return msg


class TreeROSPublisher:
    """Publish py_trees 2.1.x tree snapshots on ROS topics for rqt_py_trees."""

    def __init__(self, tree):
        self.tree = tree
        self._pub_tree = rospy.Publisher(
            '~log/tree', pt_msgs.BehaviourTree,
            queue_size=2, latch=True)
        self._pub_ascii = rospy.Publisher(
            '~ascii/snapshot', String,
            queue_size=2, latch=True)
        self._pub_tip = rospy.Publisher(
            '~tip', pt_msgs.Behaviour,
            queue_size=2, latch=True)
        tree.add_post_tick_handler(self._publish)

    def _publish(self, tree):
        now = rospy.Time.now()

        # Full tree snapshot
        behaviours = [behaviour_to_msg(node) for node in tree.root.iterate()]
        msg = pt_msgs.BehaviourTree()
        msg.header.stamp = now
        msg.behaviours = behaviours
        self._pub_tree.publish(msg)

        # ASCII snapshot
        ascii_tree = py_trees.display.unicode_tree(
            tree.root, show_status=True)
        self._pub_ascii.publish(String(data=ascii_tree))

        # Tip
        tip = tree.root.tip()
        if tip:
            self._pub_tip.publish(behaviour_to_msg(tip))
