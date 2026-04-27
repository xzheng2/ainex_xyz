#!/usr/bin/env python3
# encoding: utf-8
# {{PROJECT_CLASS}} behavior tree node — {{TASK_DESC}}

import os
import sys
import signal

import rospkg as _rospkg
_XYZ_BEHAV_DIR = _rospkg.RosPack().get_path('xyz_behavior')
sys.path.insert(0, _XYZ_BEHAV_DIR)

_LOG_DIR = os.path.join(_XYZ_BEHAV_DIR, 'log')

import rospy
import py_trees
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_interfaces.msg import ColorDetect
from ros_robot_controller.msg import BuzzerState

from {{PROJECT}}.tree.{{PROJECT}}_bt import bootstrap
from {{PROJECT}}.comm.comm_facade import CommFacade
from {{PROJECT}}.semantics.semantic_facade import {{PROJECT_CLASS}}SemanticFacade
from {{PROJECT}}.infra.tree_publisher import TreeROSPublisher
from {{PROJECT}}.infra.bb_ros_bridge import {{PROJECT_CLASS}}BBBridge
from {{PROJECT}}.infra.bt_exec_controller import BTExecController
from {{PROJECT}}.infra.infra_manifest import build_infra_manifest, write_infra_manifest
from xyz_bt_edu.input_adapters.imu_balance_state_adapter import ImuBalanceStateAdapter
from xyz_bt_edu.input_adapters.line_detection_adapter import LineDetectionAdapter
from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor


class {{PROJECT_NODE_CLASS}}(Common):
    head_pan_init = 500
    head_tilt_init = 340

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True

        # Common provides: gait_manager, motion_manager, detect_pub, lock,
        # self.start flag, enter_func, start_srv_callback, stop_srv_callback,
        # init_action
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        # /tick_id lives at absolute path (no namespace) — separate client
        self._bb_meta = py_trees.blackboard.Client(name="{{PROJECT_NODE_CLASS}}_meta")
        self._bb_meta.register_key(key="/tick_id", access=py_trees.common.Access.WRITE)
        self._bb_meta.tick_id = 0

        self.buzzer_pub = rospy.Publisher(
            '/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)

        # ── Observability ─────────────────────────────────────────────────
        os.makedirs(_LOG_DIR, exist_ok=True)
        self._tick_id = 0
        self._obs_logger = DebugEventLogger(
            bt_topic="/{{PROJECT}}_bt_debug",
            comm_topic="/{{PROJECT}}_bt_ros_comm_debug",
            bt_lastrun_jsonl=rospy.get_param(
                '~bt_jsonl', os.path.join(_LOG_DIR, 'bt_debug_lastrun.jsonl')),
            comm_lastrun_jsonl=rospy.get_param(
                '~comm_jsonl', os.path.join(_LOG_DIR, 'bt_ros_comm_debug_lastrun.jsonl')),
            rolling_bt_jsonl=os.path.join(_LOG_DIR, 'bt_debug_recent.jsonl'),
            rolling_comm_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_recent.jsonl'),
            max_rolling_ticks=rospy.get_param('~max_rolling_ticks', 30),
            tick_id_getter=lambda: self._tick_id,
        )

        # ── Input adapters ────────────────────────────────────────────────
        # rospy.Subscriber lives in xyz_bt_edu/input_adapters/, not here.
        # Two-phase latch: snapshot_and_reset() under lock, write_snapshot() after.
        # Both adapters share self.lock so all live states are snapshotted atomically.
        self._imu_adapter = ImuBalanceStateAdapter(
            lock=self.lock,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
        )
        self._line_adapter = LineDetectionAdapter(
            lock=self.lock,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
        )

        # ── Facade layers ─────────────────────────────────────────────────
        self._comm_facade = CommFacade(
            gait_manager=self.gait_manager,
            motion_manager=self.motion_manager,
            buzzer_pub=self.buzzer_pub,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
        )
        self._semantic_facade = {{PROJECT_CLASS}}SemanticFacade(
            comm_facade=self._comm_facade,
            tick_id_getter=lambda: self._tick_id,
        )

        # ── BT tree ───────────────────────────────────────────────────────
        self.tree = bootstrap(
            self._semantic_facade,
            logger=self._obs_logger,
            tick_id_getter=lambda: self._tick_id,
            robot_state_setter=self._imu_adapter.force_state,
        )

        self._bt_visitor = BTDebugVisitor(self._obs_logger, lambda: self._tick_id)
        self.tree.visitors.append(self._bt_visitor)
        self.tree.pre_tick_handlers.append(self._bt_visitor.on_tree_tick_start)
        self.tree.post_tick_handlers.append(self._bt_visitor.on_tree_tick_end)

        self._snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.visitors.append(self._snapshot_visitor)

        # ── Infra components ──────────────────────────────────────────────
        self._tree_publisher = TreeROSPublisher(self.tree)
        self._bb_bridge = {{PROJECT_CLASS}}BBBridge()
        self._bb_bridge.start(rate_hz=10)

        _bt_mode = rospy.get_param('~bt_mode', 'run')
        self._exec_ctrl = BTExecController(self.lock, initial_mode=_bt_mode)

        # ── Infra comm manifest ───────────────────────────────────────────
        manifest_path = os.path.join(_LOG_DIR, 'infra_comm_manifest_lastrun.json')
        write_infra_manifest(manifest_path, build_infra_manifest(self.name))

        self.motion_manager.run_action('walk_ready')

        if rospy.get_param('~start', True):
            target_color = rospy.get_param('~color', 'black')
            self.enter_func(None)
            self._set_color(target_color)
            self.start_srv_callback(None)
            common.loginfo('{{PROJECT_CLASS}} BT: start tracking %s lane' % target_color)

        signal.signal(signal.SIGINT, self.shutdown)

    # ── Color setup ────────────────────────────────────────────────────────

    def _set_color(self, color_name):
        # TODO: configure ROI for this competition if different from marathon
        param = ColorDetect()
        param.color_name = color_name
        param.use_name = True
        param.detect_type = 'line'
        param.image_process_size = [160, 120]
        self.detect_pub.publish([param])
        common.loginfo('%s _set_color: %s' % (self.name, color_name))

    # ── Lifecycle ──────────────────────────────────────────────────────────

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

                # Phase 1: atomic snapshot of ALL adapters under one lock acquisition.
                # Ensures the BT tree sees a consistent, frozen view of all sensor
                # inputs for this tick (no mid-snapshot async mutation between adapters).
                with self.lock:
                    imu_snap  = self._imu_adapter.snapshot_and_reset()
                    line_snap = self._line_adapter.snapshot_and_reset()

                # Phase 2: write BB + emit ros_in / input_state (no lock needed,
                # main thread only, no async mutation possible after Phase 1).
                self._imu_adapter.write_snapshot(imu_snap,  self._tick_id)
                self._line_adapter.write_snapshot(line_snap, self._tick_id)

                self.tree.tick()
                if self._snapshot_visitor.changed:
                    rospy.loginfo('\n' + py_trees.display.unicode_tree(
                        self.tree.root, show_status=True))
            rate.sleep()

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        self._obs_logger.close()
        rospy.signal_shutdown('shutdown')


if __name__ == '__main__':
    {{PROJECT_NODE_CLASS}}('{{PROJECT}}_bt').run()
