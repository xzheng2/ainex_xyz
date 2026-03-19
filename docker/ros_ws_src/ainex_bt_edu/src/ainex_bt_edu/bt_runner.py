#!/usr/bin/env python3
"""AinexBTRunner -- BT lifecycle manager with run-complete publishing."""
import uuid
import rospy
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.msg import BTRunComplete


class AinexBTRunner:
    """
    Manages a BT's full lifecycle:
    - Injects session_id into all AinexBTNode instances at init
    - spin() drives the tick loop
    - Publishes /bt_run_complete (latch=True) when root reaches terminal state
    """

    def __init__(self, tree: py_trees.trees.BehaviourTree,
                 session_id: str = None):
        self.tree = tree
        self.session_id = session_id or str(uuid.uuid4())[:8]
        self._tick_count = 0
        self._start_time = None
        self._complete_pub = rospy.Publisher(
            '/bt_run_complete', BTRunComplete, queue_size=1, latch=True)
        for node in tree.root.iterate():
            if isinstance(node, AinexBTNode):
                node._session_id = self.session_id
        tree.setup(timeout=15)

    def spin(self, rate_hz: float = 30):
        """Drive BT tick loop until root returns terminal state."""
        self._start_time = rospy.Time.now()
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.tree.tick()
            self._tick_count += 1
            if self.tree.root.status in (Status.SUCCESS, Status.FAILURE):
                self._publish_complete()
                break
            rate.sleep()

    def _publish_complete(self):
        duration = (rospy.Time.now() - self._start_time).to_sec()
        msg = BTRunComplete()
        msg.header.stamp = rospy.Time.now()
        msg.session_id   = self.session_id
        msg.status       = self.tree.root.status.name
        msg.duration_sec = duration
        msg.tick_count   = self._tick_count
        msg.tree_name    = self.tree.root.name
        self._complete_pub.publish(msg)
        rospy.loginfo('[BT Runner] COMPLETE session=%s status=%s '
                      'duration=%.2fs ticks=%d',
                      self.session_id, msg.status, duration, self._tick_count)
