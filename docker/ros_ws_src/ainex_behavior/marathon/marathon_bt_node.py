#!/usr/bin/env python3
# encoding: utf-8
# Marathon behavior tree node for Ainex humanoid robot (HuroCup 2025).
# Replaces the two-node marathon_node + fall_rise_node coupling with a
# py_trees behavior tree whose structure enforces safety priority.

import os
import sys
import math
import time
import signal

# --- local import paths ---
# Allow "from behaviours.x import ..." and "from marathon_bt import ..."
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Locate visual_patrol.py via rospkg so we don't hard-code paths
import rospkg
_hurocup_path = rospkg.RosPack().get_path('hurocup2025')
sys.path.insert(0, os.path.join(_hurocup_path, 'scripts', 'marathon'))

import rospy
import py_trees
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_interfaces.msg import ObjectsInfo, ColorDetect
from ros_robot_controller.msg import BuzzerState
from visual_patrol import VisualPatrol  # type: ignore  (found via sys.path)

from marathon_bt import bootstrap  # type: ignore


class MarathonBTNode(Common):
    head_pan_init = 500
    head_tilt_init = 340
    FALL_COUNT_THRESHOLD = 100

    # Line detection ROI (same as VisualPatrolNode)
    line_roi = [
        (5 / 12, 6 / 12, 1 / 4, 3 / 4),
        (6 / 12, 7 / 12, 1 / 4, 3 / 4),
        (7 / 12, 8 / 12, 1 / 4, 3 / 4),
    ]
    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.count_lie = 0
        self.count_recline = 0

        # Common provides: gait_manager, motion_manager, detect_pub, lock,
        # self.start flag, enter_func, start_srv_callback, stop_srv_callback,
        # init_action
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        # Blackboard writer (global keys — no namespace prefix)
        self._bb = py_trees.blackboard.Client(name="MarathonBTNode")
        self._bb.register_key(
            key="/robot_state", access=py_trees.common.Access.WRITE)
        self._bb.register_key(
            key="/line_data", access=py_trees.common.Access.WRITE)
        self._bb.robot_state = 'stand'
        self._bb.line_data = None

        # Buzzer publisher (passed into RecoverFromFall action)
        self.buzzer_pub = rospy.Publisher(
            '/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)

        # Visual patrol helper (uses gait_manager from Common)
        self.visual_patrol = VisualPatrol(self.gait_manager)

        # Build and setup the behavior tree
        self.tree = bootstrap(
            self.motion_manager,
            self.gait_manager,
            self.visual_patrol,
            self.buzzer_pub,
        )

        # Subscribers
        rospy.Subscriber('/imu', Imu, self._imu_callback)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo,
                         self._objects_callback)

        # Play ready posture
        self.motion_manager.run_action('walk_ready')

        # Auto-start if requested
        if rospy.get_param('~start', True):
            target_color = rospy.get_param('~color', 'black')
            self.enter_func(None)
            self._set_color(target_color)
            self.start_srv_callback(None)
            common.loginfo('Marathon BT: start tracking %s lane' % target_color)

        signal.signal(signal.SIGINT, self.shutdown)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _imu_callback(self, msg):
        """Detect forward/backward falls and update robot_state on blackboard."""
        angle = abs(int(
            math.degrees(
                math.atan2(msg.linear_acceleration.y,
                           msg.linear_acceleration.z))))

        with self.lock:
            current_state = self._bb.robot_state

        if current_state != 'stand':
            # Recovery in progress — don't accumulate fall counts
            return

        # Count consecutive frames where robot appears fallen
        if angle < 30:
            self.count_lie += 1
        else:
            self.count_lie = 0

        if angle > 150:
            self.count_recline += 1
        else:
            self.count_recline = 0

        if self.count_lie > self.FALL_COUNT_THRESHOLD:
            self.count_lie = 0
            self.count_recline = 0
            with self.lock:
                self._bb.robot_state = 'lie_to_stand'
            rospy.loginfo('[Marathon BT] fall detected: lie_to_stand')
        elif self.count_recline > self.FALL_COUNT_THRESHOLD:
            self.count_lie = 0
            self.count_recline = 0
            with self.lock:
                self._bb.robot_state = 'recline_to_stand'
            rospy.loginfo('[Marathon BT] fall detected: recline_to_stand')

    def _objects_callback(self, msg):
        """Write line detection result to blackboard."""
        line_data = None
        for obj in msg.data:
            if obj.type == 'line':
                line_data = obj
                break
        with self.lock:
            self._bb.line_data = line_data

    # ------------------------------------------------------------------
    # Color detection setup (mirrors VisualPatrolNode.set_color_srv_callback)
    # ------------------------------------------------------------------

    def _set_color(self, color_name):
        param = ColorDetect()
        param.color_name = color_name
        param.use_name = True
        param.detect_type = 'line'
        param.image_process_size = self.image_process_size

        param.line_roi.up.y_min = int(
            self.line_roi[0][0] * self.image_process_size[1])
        param.line_roi.up.y_max = int(
            self.line_roi[0][1] * self.image_process_size[1])
        param.line_roi.up.x_min = int(
            self.line_roi[0][2] * self.image_process_size[0])
        param.line_roi.up.x_max = int(
            self.line_roi[0][3] * self.image_process_size[0])

        param.line_roi.center.y_min = int(
            self.line_roi[1][0] * self.image_process_size[1])
        param.line_roi.center.y_max = int(
            self.line_roi[1][1] * self.image_process_size[1])
        param.line_roi.center.x_min = int(
            self.line_roi[1][2] * self.image_process_size[0])
        param.line_roi.center.x_max = int(
            self.line_roi[1][3] * self.image_process_size[0])

        param.line_roi.down.y_min = int(
            self.line_roi[2][0] * self.image_process_size[1])
        param.line_roi.down.y_max = int(
            self.line_roi[2][1] * self.image_process_size[1])
        param.line_roi.down.x_min = int(
            self.line_roi[2][2] * self.image_process_size[0])
        param.line_roi.down.x_max = int(
            self.line_roi[2][3] * self.image_process_size[0])

        param.min_area = 1
        param.max_area = (self.image_process_size[0]
                          * self.image_process_size[1])

        self.detect_pub.publish([param])
        common.loginfo('%s _set_color: %s' % (self.name, color_name))

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def shutdown(self, signum, frame):
        with self.lock:
            self.motion_manager.run_action('stand')
            self.running = False
        common.loginfo('%s shutdown' % self.name)

    def run(self):
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            if self.start:
                self.tree.tick()
            rate.sleep()

        # Clean up
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')


if __name__ == '__main__':
    MarathonBTNode('marathon_bt').run()
