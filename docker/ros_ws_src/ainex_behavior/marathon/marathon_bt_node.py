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
from comm_facade import CommFacade  # type: ignore
from semantic_facade import MarathonSemanticFacade  # type: ignore
from behaviours.actions import FindLineHeadSweep  # type: ignore
from tree_publisher import TreeROSPublisher  # type: ignore
from bb_ros_bridge import MarathonBBBridge  # type: ignore
from bt_exec_controller import BTExecController  # type: ignore
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor
from bt_observability.ros_comm_tracer import ManagerProxy, ROSCommTracer

_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log')


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

        # ── Live (pending) inputs — written only by async ROS callbacks ────
        # Callbacks update this dict exclusively.  _latch_inputs() copies a
        # snapshot to the BB once per iteration, just before tree.tick(), so
        # every BT node in a single tick sees a frozen, consistent view of
        # sensor inputs (no mid-tick async mutation).
        self._live = {
            'robot_state':       'stand',
            'line_data':         None,
            'last_line_x':       None,
            'camera_lost_count': 0,
        }

        # ── Latched BB client — written only by _latch_inputs() ─────────
        # BT condition/action nodes read these keys via their own BB clients
        # using namespace="/latched", so they resolve to /latched/<key>.
        # /tick_id is BT-internal (not a sensor input) and kept at /tick_id.
        self._bb = py_trees.blackboard.Client(
            name="MarathonBTNode_latched", namespace="/latched")
        self._bb.register_key(
            key="robot_state", access=py_trees.common.Access.WRITE)
        self._bb.register_key(
            key="line_data", access=py_trees.common.Access.WRITE)
        self._bb.register_key(
            key="last_line_x", access=py_trees.common.Access.WRITE)
        self._bb.register_key(
            key="camera_lost_count", access=py_trees.common.Access.WRITE)
        # Initialise BB to match live store so nodes never hit KeyError
        # before the first _latch_inputs() call.
        self._bb.robot_state       = self._live['robot_state']
        self._bb.line_data         = self._live['line_data']
        self._bb.last_line_x       = self._live['last_line_x']
        self._bb.camera_lost_count = self._live['camera_lost_count']

        # /tick_id lives at an absolute path (no namespace) — separate client
        # avoids attribute-name ambiguity when the latched client has namespace.
        self._bb_meta = py_trees.blackboard.Client(name="MarathonBTNode_meta")
        self._bb_meta.register_key(
            key="/tick_id", access=py_trees.common.Access.WRITE)
        self._bb_meta.tick_id = 0

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

        # Wrap managers with tracing proxies — visual_patrol receives the proxies
        # so all internal gait calls (FollowLine via visual_patrol.process) are
        # automatically logged.  CommFacade also receives the same proxies so
        # explicit leaf-node comms share the same logging path.
        self._gait_proxy   = ManagerProxy(self.gait_manager,   self._obs_logger,
                                          lambda: self._tick_id, _GAIT_ROS_MAP,
                                          "gait_manager")
        self._motion_proxy = ManagerProxy(self.motion_manager, self._obs_logger,
                                          lambda: self._tick_id, _MOTION_ROS_MAP,
                                          "motion_manager")
        self.visual_patrol = VisualPatrol(self._gait_proxy)

        # Generic ROS Facade — unified ROS communication exit and final log outlet.
        # All outbound ROS calls from the semantic facade flow through here.
        self._comm_facade = CommFacade(
            gait_manager=self._gait_proxy,
            motion_manager=self._motion_proxy,
            buzzer_pub=self.buzzer_pub,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
        )

        # Project Semantic Facade — translates leaf-node business intent into
        # project-semantic commands and delegates ROS I/O to CommFacade.
        self._semantic_facade = MarathonSemanticFacade(
            visual_patrol=self.visual_patrol,
            comm_facade=self._comm_facade,
            tick_id_getter=lambda: self._tick_id,
        )

        # Build and setup the behavior tree — only semantic_facade is injected;
        # CommFacade and VisualPatrol are internal implementation details.
        self.tree = bootstrap(
            self._semantic_facade,
            find_line_cls=_find_line_cls,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
            robot_state_setter=self._set_live_robot_state,
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
            current_state = self._live['robot_state']

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
                self._live['robot_state'] = 'lie_to_stand'
            rospy.loginfo('[Marathon BT] fall detected: lie_to_stand')
        elif self.count_recline > self.FALL_COUNT_THRESHOLD:
            self.count_lie = 0
            self.count_recline = 0
            with self.lock:
                self._live['robot_state'] = 'recline_to_stand'
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
                          self._live['camera_lost_count'] + 1)

        with self.lock:
            self._live['line_data'] = line_data
            if line_data is not None:
                self._live['last_line_x'] = line_data.x
                self._live['camera_lost_count'] = 0
            else:
                self._live['camera_lost_count'] = self._live['camera_lost_count'] + 1

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
    # Live → Latched input management
    # ------------------------------------------------------------------

    def _latch_inputs(self):
        """Snapshot the live (async) input dict into the latched BB keys.

        Called once per BT iteration, *before* self.tree.tick(), so every
        node in a tick reads a consistent, frozen view of sensor inputs.
        The copy is done under self.lock to avoid a torn read, then the
        BB writes happen outside the lock (the BB is only touched from the
        main thread during ticks).

        ── Pause/Step hook ─────────────────────────────────────────────
        This is the natural injection point for step-mode inspection:
        after _latch_inputs() the BB reflects exactly what the upcoming
        tick will see.  A future debugger could snapshot or publish the
        latched state here before calling tree.tick().
        """
        with self.lock:
            snap = dict(self._live)
        self._bb.robot_state       = snap['robot_state']
        self._bb.line_data         = snap['line_data']
        self._bb.last_line_x       = snap['last_line_x']
        self._bb.camera_lost_count = snap['camera_lost_count']

    def _set_live_robot_state(self, state):
        """Callback for RecoverFromFall to write recovery completion into live.

        RecoverFromFall already writes 'stand' to the latched BB key so the
        remainder of the current tick (PatrolControl) sees 'stand'.  It also
        calls this setter so the *next* pre-tick latch does not overwrite the
        BB with the stale pre-recovery live value that the IMU callback set.
        """
        with self.lock:
            self._live['robot_state'] = state

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def shutdown(self, signum, frame):
        with self.lock:
            self.motion_manager.run_action('stand')
            self.running = False
        common.loginfo('%s shutdown' % self.name)

    def run(self):
        rate = rospy.Rate(10)
        while self.running and not rospy.is_shutdown():
            if self.start and self._exec_ctrl.should_tick():
                self._tick_id += 1
                self._bb_meta.tick_id = self._tick_id
                # Snapshot async inputs into latched BB keys before any BT
                # node runs.  All nodes in this tick share this frozen view;
                # new callbacks arriving mid-tick only affect the next tick.
                self._latch_inputs()
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
