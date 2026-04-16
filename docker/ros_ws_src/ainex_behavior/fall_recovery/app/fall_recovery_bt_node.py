#!/usr/bin/env python3
# encoding: utf-8
# Fall-recovery behavior tree node for Ainex humanoid robot.
# Monitors IMU for falls and executes stand-up recovery via py_trees BT.
# No line following, camera, or gait-search logic.

import os
import sys
import math
import time
import signal

# ── Package path via rospkg (stable across catkin_install_python wrappers) ──
import rospkg as _rospkg
_AINEX_BEHAV_DIR = _rospkg.RosPack().get_path('ainex_behavior')
sys.path.insert(0, _AINEX_BEHAV_DIR)

_LOG_DIR = os.path.join(_AINEX_BEHAV_DIR, 'log')

import rospy
import py_trees
from sensor_msgs.msg import Imu
from ainex_sdk import common
from ainex_example.color_common import Common
from ros_robot_controller.msg import BuzzerState

from fall_recovery.tree.fall_recovery_bt import bootstrap
from fall_recovery.comm.comm_facade import CommFacade
from fall_recovery.semantics.semantic_facade import FallRecoverySemanticFacade
from fall_recovery.infra.tree_publisher import TreeROSPublisher
from fall_recovery.infra.bb_ros_bridge import FallRecoveryBBBridge
from fall_recovery.infra.bt_exec_controller import BTExecController
from fall_recovery.infra.infra_manifest import build_infra_manifest, write_infra_manifest
from fall_recovery.app.ros_msg_utils import msg_to_dict
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor


class FallRecoveryBTNode(Common):
    head_pan_init = 500
    head_tilt_init = 340
    FALL_COUNT_THRESHOLD = 100

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
        self._live = {
            'robot_state': 'stand',
        }

        # ── Latched BB client — written only by _latch_inputs() ─────────
        self._bb = py_trees.blackboard.Client(
            name="FallRecoveryBTNode_latched", namespace="/latched")
        self._bb.register_key(
            key="robot_state", access=py_trees.common.Access.WRITE)
        self._bb.robot_state = self._live['robot_state']

        # /tick_id lives at an absolute path (no namespace)
        self._bb_meta = py_trees.blackboard.Client(name="FallRecoveryBTNode_meta")
        self._bb_meta.register_key(
            key="/tick_id", access=py_trees.common.Access.WRITE)
        self._bb_meta.tick_id = 0

        # Buzzer publisher
        self.buzzer_pub = rospy.Publisher(
            '/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)

        # Observability: logger + visitor (tick_id managed in run())
        os.makedirs(_LOG_DIR, exist_ok=True)
        self._tick_id = 0
        self._last_imu_emit_tick = -1
        self._obs_logger = DebugEventLogger(
            bt_topic="/fall_recovery_bt_debug",
            comm_topic="/fall_recovery_bt_ros_comm_debug",
            bt_lastrun_jsonl=rospy.get_param(
                '~bt_jsonl', os.path.join(_LOG_DIR, 'bt_debug_lastrun.jsonl')),
            comm_lastrun_jsonl=rospy.get_param(
                '~comm_jsonl', os.path.join(_LOG_DIR, 'bt_ros_comm_debug_lastrun.jsonl')),
            rolling_bt_jsonl=os.path.join(_LOG_DIR, 'bt_debug_recent.jsonl'),
            rolling_comm_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_recent.jsonl'),
            max_rolling_ticks=rospy.get_param('~max_rolling_ticks', 30),
            tick_id_getter=lambda: self._tick_id,
        )

        # ── Build facade layers ──────────────────────────────────────────
        self._comm_facade = CommFacade(
            gait_manager=self.gait_manager,
            motion_manager=self.motion_manager,
            buzzer_pub=self.buzzer_pub,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
        )

        self._semantic_facade = FallRecoverySemanticFacade(
            comm_facade=self._comm_facade,
            tick_id_getter=lambda: self._tick_id,
        )

        # Build the behavior tree
        self.tree = bootstrap(
            self._semantic_facade,
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
        self._tree_publisher = TreeROSPublisher(self.tree)
        self._bb_bridge = FallRecoveryBBBridge()
        self._bb_bridge.start(rate_hz=10)

        _bt_mode = rospy.get_param('~bt_mode', 'run')
        self._exec_ctrl = BTExecController(self.lock, initial_mode=_bt_mode)

        # Subscribers
        rospy.Subscriber('/imu', Imu, self._imu_callback)

        # Write static infra comm manifest (overwrites on each startup)
        manifest_path = os.path.join(_LOG_DIR, 'infra_comm_manifest_lastrun.json')
        write_infra_manifest(manifest_path, build_infra_manifest(self.name))

        # Play ready posture
        self.motion_manager.run_action('walk_ready')

        # Auto-start if requested
        if rospy.get_param('~start', True):
            self.enter_func(None)
            self.start_srv_callback(None)
            common.loginfo('Fall Recovery BT: started')

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
            rospy.loginfo('[FallRecovery BT] fall detected: lie_to_stand')
        elif self.count_recline > self.FALL_COUNT_THRESHOLD:
            self.count_lie = 0
            self.count_recline = 0
            with self.lock:
                self._live['robot_state'] = 'recline_to_stand'
            rospy.loginfo('[FallRecovery BT] fall detected: recline_to_stand')

    # ------------------------------------------------------------------
    # Live → Latched input management
    # ------------------------------------------------------------------

    def _latch_inputs(self):
        """Snapshot the live (async) input dict into the latched BB keys.

        Called once per BT iteration, *before* self.tree.tick(), so every
        node in a tick reads a consistent, frozen view of sensor inputs.
        """
        with self.lock:
            snap = dict(self._live)
        self._bb.robot_state = snap['robot_state']

    def _set_live_robot_state(self, state):
        """Callback for RecoverFromFall to write recovery completion into live."""
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
                self._latch_inputs()
                self.tree.tick()

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
    FallRecoveryBTNode('fall_recovery_bt').run()
