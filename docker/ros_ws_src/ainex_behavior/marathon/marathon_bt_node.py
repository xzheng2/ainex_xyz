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
# Allow "from bt_observability.x import ..." (ainex_behavior/ parent dir)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import py_trees
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_interfaces.msg import ObjectsInfo, ColorDetect
from ros_robot_controller.msg import BuzzerState
from behaviours.visual_patrol import VisualPatrol  # type: ignore  (local copy)

from marathon_bt import bootstrap  # type: ignore
from behaviours.actions import FindLineHeadSweep  # type: ignore
from tree_publisher import TreeROSPublisher  # type: ignore
from bb_ros_bridge import MarathonBBBridge  # type: ignore
from bt_exec_controller import BTExecController  # type: ignore
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor
from bt_observability.ros_comm_tracer import ManagerProxy, ROSCommTracer

_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log')

# Topics to record via rosbag for full message-level observability.
# Covers all BT-relevant topics: gait control, servo, IMU, line detection.
_ROSBAG_TOPICS = [
    "/walking/set_param",
    "/walking/command",
    "/ros_robot_controller/bus_servo/set_position",
    "/ros_robot_controller/set_buzzer",
    "/imu",
    "/object/pixel_coords",
]


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
        self._bb.register_key(
            key="/last_line_x", access=py_trees.common.Access.WRITE)
        self._bb.register_key(
            key="/camera_lost_count", access=py_trees.common.Access.WRITE)
        self._bb.register_key(
            key="/tick_id", access=py_trees.common.Access.WRITE)
        self._bb.robot_state = 'stand'
        self._bb.line_data = None
        self._bb.last_line_x = None
        self._bb.camera_lost_count = 0
        self._bb.tick_id = 0

        # Buzzer publisher (passed into RecoverFromFall action)
        self.buzzer_pub = rospy.Publisher(
            '/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)

        # Select FindLine variant via ROS param (default: gait-based)
        _find_line_mode = rospy.get_param('~find_line', 'gait')
        _find_line_cls = FindLineHeadSweep if _find_line_mode == 'head_sweep' else None

        # Observability: logger + visitor (tick_id managed in run())
        os.makedirs(_LOG_DIR, exist_ok=True)
        self._tick_id = 0
        # Per-topic dedup: only emit ros_topic_received once per BT tick
        self._last_imu_emit_tick = -1
        self._last_obj_emit_tick = -1
        self._obs_logger = DebugEventLogger(
            bt_topic="/bt_debug",
            comm_topic="/bt_ros_comm_debug",
            bt_lastrun_jsonl=rospy.get_param(
                '~bt_jsonl', os.path.join(_LOG_DIR, 'bt_debug_lastrun.jsonl')),
            comm_lastrun_jsonl=rospy.get_param(
                '~comm_jsonl', os.path.join(_LOG_DIR, 'bt_ros_comm_debug_lastrun.jsonl')),
            rolling_bt_jsonl=os.path.join(_LOG_DIR, 'bt_debug_recent.jsonl'),
            rolling_comm_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_recent.jsonl'),
            max_rolling_ticks=rospy.get_param('~max_rolling_ticks', 30),
            tick_id_getter=lambda: self._tick_id,
            rosbag_topics=_ROSBAG_TOPICS,
            rosbag_dir=os.path.join(_LOG_DIR, 'rosbag'),
        )

        # ROS interface maps for ManagerProxy — maps Python method names to the
        # actual ROS topic/service they communicate with.
        _GAIT_ROS_MAP = {
            "disable": {
                "comm_type": "service_call", "direction": "call",
                "target": "/walking/command", "ros_node": "ainex_controller",
                "payload_fn": lambda args, kwargs: {"command": "disable"},
            },
            "enable": {
                "comm_type": "service_call", "direction": "call",
                "target": "/walking/command", "ros_node": "ainex_controller",
                "payload_fn": lambda args, kwargs: {"command": "enable"},
            },
            "stop": {
                "comm_type": "service_call", "direction": "call",
                "target": "/walking/command", "ros_node": "ainex_controller",
                "payload_fn": lambda args, kwargs: {"command": "stop"},
            },
            "set_step": {
                "comm_type": "topic_publish", "direction": "publish",
                "target": "walking/set_param", "ros_node": "ainex_controller",
                "payload_fn": lambda args, kwargs: {
                    "dsp": args[0] if len(args) > 0 else None,
                    "x":   args[1] if len(args) > 1 else None,
                    "y":   args[2] if len(args) > 2 else None,
                    "yaw": args[3] if len(args) > 3 else None,
                },
            },
        }
        _MOTION_ROS_MAP = {
            "run_action": {
                "comm_type": "topic_publish", "direction": "publish",
                "target": "ros_robot_controller/bus_servo/set_position",
                "ros_node": "ros_robot_controller",
                "payload_fn": lambda args, kwargs: {"action": args[0] if args else None},
            },
            "set_servos_position": {
                "comm_type": "topic_publish", "direction": "publish",
                "target": "ros_robot_controller/bus_servo/set_position",
                "ros_node": "ros_robot_controller",
                "payload_fn": lambda args, kwargs: {
                    "duration":  args[0] if len(args) > 0 else None,
                    "positions": args[1] if len(args) > 1 else None,
                },
            },
        }

        # Wrap managers with tracing proxies — visual_patrol and bootstrap
        # receive the proxies so all their manager calls are automatically logged.
        self._gait_proxy   = ManagerProxy(self.gait_manager,   self._obs_logger,
                                          lambda: self._tick_id, _GAIT_ROS_MAP,
                                          "gait_manager")
        self._motion_proxy = ManagerProxy(self.motion_manager, self._obs_logger,
                                          lambda: self._tick_id, _MOTION_ROS_MAP,
                                          "motion_manager")
        self.visual_patrol = VisualPatrol(self._gait_proxy)

        # Build and setup the behavior tree
        self.tree = bootstrap(
            self._motion_proxy,
            self._gait_proxy,
            self.visual_patrol,
            self.buzzer_pub,
            find_line_cls=_find_line_cls,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
        )

        # Debug visitor: records per-node tick events to obs_logger
        self._bt_visitor = BTDebugVisitor(self._obs_logger, lambda: self._tick_id)
        self.tree.visitors.append(self._bt_visitor)
        self.tree.pre_tick_handlers.append(self._bt_visitor.on_tree_tick_start)
        self.tree.post_tick_handlers.append(self._bt_visitor.on_tree_tick_end)

        # Snapshot visitor: detects when any node changes status
        self._snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.visitors.append(self._snapshot_visitor)

        # Publish tree state on ROS topics for rqt_py_trees and ROSA
        self._tree_publisher = TreeROSPublisher(self.tree)
        self._bb_bridge = MarathonBBBridge()
        self._bb_bridge.start(rate_hz=10)

        _bt_mode = rospy.get_param('~bt_mode', 'run')
        self._exec_ctrl = BTExecController(self.lock, initial_mode=_bt_mode)

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
        if self._tick_id != self._last_imu_emit_tick:
            self._last_imu_emit_tick = self._tick_id
            self._obs_logger.emit_comm({
                "event":     "ros_topic_received",
                "direction": "subscribe",
                "target":    "/imu",
                "ros_node":  None,
                "node":      self.name,
                "tick_id":   self._tick_id,
                "payload":   ROSCommTracer._msg_to_dict(msg),
            })
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
        """Write line detection result to blackboard.

        Also maintains /last_line_x and /camera_lost_count for FindLine recovery.
        """
        if self._tick_id != self._last_obj_emit_tick:
            self._last_obj_emit_tick = self._tick_id
            self._obs_logger.emit_comm({
                "event":     "ros_topic_received",
                "direction": "subscribe",
                "target":    "/object/pixel_coords",
                "ros_node":  None,
                "node":      self.name,
                "tick_id":   self._tick_id,
                "payload":   ROSCommTracer._msg_to_dict(msg),
            })
        line_data = None
        for obj in msg.data:
            if obj.type == 'line':
                line_data = obj
                break

        if line_data is not None:
            rospy.loginfo('[BB] line detected x=%.1f', line_data.x)
        else:
            rospy.loginfo('[BB] line lost (count=%d)',
                          self._bb.camera_lost_count + 1)

        with self.lock:
            self._bb.line_data = line_data
            if line_data is not None:
                self._bb.last_line_x = line_data.x
                self._bb.camera_lost_count = 0
            else:
                self._bb.camera_lost_count = self._bb.camera_lost_count + 1

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
        rate = rospy.Rate(15)
        while self.running and not rospy.is_shutdown():
            if self.start and self._exec_ctrl.should_tick():
                self._tick_id += 1
                self._bb.tick_id = self._tick_id
                self.tree.tick()

                # Print tree snapshot only when a node changed status
                if self._snapshot_visitor.changed:
                    rospy.loginfo('\n' + py_trees.display.unicode_tree(
                        self.tree.root, show_status=True))

            rate.sleep()

        # Clean up
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        self._obs_logger.close()
        rospy.signal_shutdown('shutdown')


if __name__ == '__main__':
    MarathonBTNode('marathon_bt').run()
