#!/usr/bin/env python3
# encoding: utf-8
# ROS node version of serial_servo_move_demo.py
# 让一个总线舵机循环经过 4 个位置 (cycle one bus servo through a 4-waypoint sequence)
#
# Unlike the original demo, this node does NOT grab the serial Board directly.
# It publishes SetBusServosPosition messages to the controller topic, so it can
# run alongside ros_robot_controller_node (which owns the serial port and applies
# the per-servo position limits).
import rospy
from ros_robot_controller.msg import SetBusServosPosition, BusServoPosition

TOPIC = '/ros_robot_controller/bus_servo/set_position'

# Default 4-waypoint sequence: (position, move_duration_s, stay_still_s)
DEFAULT_WAYPOINTS = [
    (301, 0.1, 1.0),
    (300, 0.1, 1.0),
    (369, 0.1, 1.0),
    (370, 0.1, 1.0),
]


class SerialServoMove:
    def __init__(self):
        rospy.init_node('serial_servo_move')

        self.servo_id = rospy.get_param('~servo_id', 24)
        self.exit_position = rospy.get_param('~exit_position', 370)

        # Build the 4 waypoints; each position has its own move + stay-still time.
        self.waypoints = []
        for i, (pos, move, stay) in enumerate(DEFAULT_WAYPOINTS, start=1):
            self.waypoints.append((
                rospy.get_param('~position_%d' % i, pos),
                rospy.get_param('~move_%d' % i, move),
                rospy.get_param('~stay_%d' % i, stay),
            ))

        self.pub = rospy.Publisher(TOPIC, SetBusServosPosition, queue_size=1)
        # Give the publisher connection a moment to register before the first send.
        rospy.sleep(0.5)

        # Send the exit position once when the node shuts down (any reason, incl. Ctrl-C).
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo('serial_servo_move: servo %d cycling through %s',
                      self.servo_id, [w[0] for w in self.waypoints])

    def send(self, position, duration):
        msg = SetBusServosPosition()
        msg.duration = duration
        msg.position = [BusServoPosition(id=self.servo_id, position=position)]
        self.pub.publish(msg)

    def move_and_hold(self, position, move, stay):
        # Drive toward `position` over `move` seconds, then hold it for `stay` seconds.
        self.send(position, move)
        rospy.sleep(move)
        self.send(position, stay)
        rospy.sleep(stay)

    def _on_shutdown(self):
        # Best-effort: park the servo at the exit position.
        try:
            self.send(self.exit_position, 0.03)
        except Exception:
            pass

    def run(self):
        while not rospy.is_shutdown():
            for position, move, stay in self.waypoints:
                if rospy.is_shutdown():
                    break
                self.move_and_hold(position, move, stay)


if __name__ == '__main__':
    try:
        SerialServoMove().run()
    except rospy.ROSInterruptException:
        pass
