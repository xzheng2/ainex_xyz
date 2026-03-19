#!/usr/bin/env python3
"""
AinexBTNode -- base class for all AiNex BT nodes.

Subclasses only need to:
  - Declare LEVEL = 'L1' | 'L2' | 'L3'
  - Declare BB_LOG_KEYS = ['/domain/key', ...]
  - Call super().setup(**kwargs) in setup()
  - Implement update() -> return Status

Logging and event publishing are fully automatic.
"""
import json
import rospy
import py_trees
from py_trees.common import Status
from ainex_bt_edu.msg import BTNodeEvent


class AinexBTNode(py_trees.behaviour.Behaviour):
    LEVEL = 'L1'
    BB_LOG_KEYS = []

    _event_pub = None

    def __init__(self, name: str):
        # Init before super().__init__ because py_trees calls self.tick()
        # during __init__ to create the iterator.
        self._tick_count = 0
        self._session_id = 'unknown'
        self._bb_log_client = None
        super().__init__(name)

    def setup(self, **kwargs):
        """Initialize ROS publishers. Subclasses must call super().setup(**kwargs)."""
        if AinexBTNode._event_pub is None:
            AinexBTNode._event_pub = rospy.Publisher(
                '/bt_node_events', BTNodeEvent, queue_size=50)
        if self.BB_LOG_KEYS:
            self._bb_log_client = self.attach_blackboard_client(
                name=f'{self.name}_log')
            for key in self.BB_LOG_KEYS:
                self._bb_log_client.register_key(
                    key=key, access=py_trees.common.Access.READ)

    def tick(self):
        """Override tick (generator): inject state-change logging transparently."""
        prev = self.status
        self._tick_count += 1
        for node in super().tick():
            yield node
        if self.status != prev:
            self._emit_event(prev, self.status)

    def _emit_event(self, prev: Status, curr: Status):
        """Publish structured log to /rosout and /bt_node_events."""
        bb = self._snapshot_bb()
        log_fn = rospy.logwarn if curr == Status.FAILURE else rospy.loginfo
        log_fn('[BT][%s][%s] %s->%s tick=%d bb=%s',
               self.LEVEL, self.name,
               prev.name, curr.name, self._tick_count, bb)
        if AinexBTNode._event_pub:
            msg = BTNodeEvent()
            msg.header.stamp = rospy.Time.now()
            msg.node_name    = self.name
            msg.level        = self.LEVEL
            msg.prev_status  = prev.name
            msg.curr_status  = curr.name
            msg.tick_count   = self._tick_count
            msg.session_id   = self._session_id
            msg.bb_snapshot  = json.dumps(bb)
            AinexBTNode._event_pub.publish(msg)

    def _snapshot_bb(self) -> dict:
        """Read Blackboard.storage directly (log-only, bypasses ACL)."""
        storage = py_trees.blackboard.Blackboard.storage
        return {
            key: str(storage.get(key, '<unset>'))
            for key in self.BB_LOG_KEYS
        }
