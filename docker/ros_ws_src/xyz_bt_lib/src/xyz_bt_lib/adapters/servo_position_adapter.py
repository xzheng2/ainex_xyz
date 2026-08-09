#!/usr/bin/env python3
"""Input adapter that subscribes to the servo-position topic and writes positions to the BB.

BB.SERVO_POSITIONS → dict {servo_id (int): position (int) or None}
None means the servo did not appear in the most recent publisher message.

Requires ros_robot_controller_node started with state_servo_ids, e.g.:
  rosrun ros_robot_controller ros_robot_controller_node.py _state_servo_ids:=[23,24] _state_rate_hz:=10.0
"""
import rospy
from py_trees.common import Access

from ros_robot_controller.msg import SetBusServosPosition
from xyz_bt_lib.core.base_adapter import XyzInputAdapter
from xyz_bt_lib.blackboard.blackboard_keys import BB

_DEFAULT_TOPIC = '/ros_robot_controller/bus_servo/state'


class ServoPositionAdapter(XyzInputAdapter):
    """Subscribes to the bus-servo state topic and writes all positions to the BB."""

    ADAPTER_NAME = 'ServoPositionAdapter'
    ROS_TOPIC = _DEFAULT_TOPIC
    BB_WRITES = [BB.SERVO_POSITIONS]
    CONFIG_DEFAULTS = {'topic': _DEFAULT_TOPIC}

    DEFAULT_SERVO_IDS = list(range(1, 25))  # IDs 1-24

    def __init__(self, lock, logger=None, tick_id_getter=None,
                 servo_ids=None, topic=_DEFAULT_TOPIC):
        super().__init__(lock, logger, tick_id_getter)
        self._servo_ids = list(servo_ids or self.DEFAULT_SERVO_IDS)

        # Live state: {servo_id: position or None}
        self._live_positions = {sid: None for sid in self._servo_ids}

        # BB client (latched namespace)
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.SERVO_POSITIONS_KEY, access=Access.WRITE)
        self._bb.servo_positions = dict(self._live_positions)

        # ROS subscriber replaces the timer+service pattern
        rospy.Subscriber(topic, SetBusServosPosition, self._callback, queue_size=1)

    # ------------------------------------------------------------------
    # Subscriber callback (runs on ROS spin thread)
    # ------------------------------------------------------------------

    def _callback(self, msg):
        with self._lock:
            for entry in msg.position:
                if entry.id in self._live_positions:
                    self._live_positions[entry.id] = entry.position
            if msg.position:
                self._received_count += 1

    # ------------------------------------------------------------------
    # Two-phase latch protocol
    # ------------------------------------------------------------------

    def snapshot_and_reset(self) -> dict:
        snap = {
            'positions': dict(self._live_positions),
            'received_count': self._received_count,
        }
        self._received_count = 0
        return snap

    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        self.emit_ros_in(
            tick_id=tick_id,
            received_count=snap['received_count'],
            source=self.ROS_TOPIC,
        )
        self._bb.servo_positions = snap['positions']
        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={BB.SERVO_POSITIONS: snap['positions']},
        )
