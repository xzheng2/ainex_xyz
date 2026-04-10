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

# ── Package path via rospkg (stable across catkin_install_python wrappers) ──
# rospkg.get_path() returns the source directory regardless of whether the
# node is launched from source or from an installed wrapper script.
import rospkg as _rospkg
_AINEX_BEHAV_DIR = _rospkg.RosPack().get_path('ainex_behavior')
sys.path.insert(0, _AINEX_BEHAV_DIR)

_LOG_DIR = os.path.join(_AINEX_BEHAV_DIR, 'marathon', 'log')

import rospy
import py_trees
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_interfaces.msg import ObjectsInfo, ColorDetect
from ros_robot_controller.msg import BuzzerState

from marathon.tree.marathon_bt import bootstrap
from marathon.comm.comm_facade import CommFacade
from marathon.semantics.semantic_facade import MarathonSemanticFacade
from marathon.algorithms.visual_patrol import VisualPatrol
from marathon.behaviours.actions import FindLineHeadSweep
from marathon.infra.tree_publisher import TreeROSPublisher
from marathon.infra.bb_ros_bridge import MarathonBBBridge
from marathon.infra.bt_exec_controller import BTExecController
from marathon.infra.infra_manifest import build_infra_manifest, write_infra_manifest
from marathon.app.ros_msg_utils import msg_to_dict
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor


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

        # Buzzer publisher (passed into CommFacade, used by RecoverFromFall via
        # semantic_facade → comm_facade.publish_buzzer)
        self.buzzer_pub = rospy.Publisher(
            '/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)

        # Select FindLine variant via ROS param (default: gait-based)
        _find_line_mode = rospy.get_param('~find_line', 'gait')
        _find_line_cls = FindLineHeadSweep if _find_line_mode == 'head_sweep' else None

        # Observability: logger + visitor (tick_id managed in run())
        os.makedirs(_LOG_DIR, exist_ok=True)
        self._tick_id = 0
        # Per-topic dedup: only emit ros_in once per BT tick
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

        # ── Build algorithm + facade layers ─────────────────────────────
        # Gait parameters: loaded at app layer and passed as pure data to
        # VisualPatrol so the algorithm layer has zero runtime dependency.
        _go_param = self.gait_manager.get_gait_param()
        _go_param['body_height']      = 0.025
        _go_param['step_height']      = 0.015
        _go_param['hip_pitch_offset'] = 25
        _go_param['z_swap_amplitude'] = 0.006

        _turn_param = self.gait_manager.get_gait_param()
        _turn_param['body_height']      = 0.025
        _turn_param['step_height']      = 0.015
        _turn_param['hip_pitch_offset'] = 25
        _turn_param['z_swap_amplitude'] = 0.006
        _turn_param['pelvis_offset']    = 8

        self.visual_patrol = VisualPatrol(
            go_gait_param=_go_param,
            turn_gait_param=_turn_param,
        )

        # Generic ROS Facade — sole business ROS exit and log outlet.
        # Receives real manager objects (not proxies).
        self._comm_facade = CommFacade(
            gait_manager=self.gait_manager,
            motion_manager=self.motion_manager,
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

        # ── Infra components ────────────────────────────────────────────
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

        # Write static infra comm manifest (overwrites on each startup)
        manifest_path = os.path.join(_LOG_DIR, 'infra_comm_manifest_lastrun.json')
        write_infra_manifest(manifest_path, build_infra_manifest(self.name))

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
                "event":     "ros_in",
                "ts":        time.time(),
                "tick_id":   self._tick_id,
                "comm_type": "topic_subscribe",
                "direction": "in",
                "target":    "/imu",
                "ros_node":  None,
                "adapter":   "imu_callback",
                "payload":   msg_to_dict(msg),
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
                "event":     "ros_in",
                "ts":        time.time(),
                "tick_id":   self._tick_id,
                "comm_type": "topic_subscribe",
                "direction": "in",
                "target":    "/object/pixel_coords",
                "ros_node":  None,
                "adapter":   "objects_callback",
                "payload":   msg_to_dict(msg),
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
