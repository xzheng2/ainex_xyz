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
from ros_robot_controller.msg import BuzzerState

from {{PROJECT}}.tree.{{PROJECT}}_bt import bootstrap
from {{PROJECT}}.runtime.runtime_facade import {{PROJECT_CLASS}}RuntimeFacade
from {{PROJECT}}.runtime._runtime_io import _RuntimeIO
from {{PROJECT}}.infra.tree_publisher import TreeROSPublisher
from {{PROJECT}}.infra.bb_ros_bridge import {{PROJECT_CLASS}}BBBridge
from {{PROJECT}}.infra.bt_exec_controller import BTExecController
from {{PROJECT}}.infra.infra_manifest import build_infra_manifest, write_infra_manifest
from xyz_bt_edu.input_adapters.imu_balance_state_adapter import ImuBalanceStateAdapter
# TODO: import additional adapters as needed, e.g.:
# from xyz_bt_edu.input_adapters.line_detection_adapter import LineDetectionAdapter

# ── Project gait profile constants ───────────────────────────────────────────
# Shape: static params of set_step() / gait_manager.set_step().
# RuntimeFacade.go_step(x, y, yaw) merges these with per-tick x/y/yaw.
# TODO: tune dsp and other params for this project's walking characteristics.
_GO_CFG = {
    'dsp':        0.1,    # double-support phase duration (s) — tune per project
    'gait_param': None,   # GaitParam msg or None (use controller default)
    'arm_swap':   False,
    'step_num':   1,
}
_TURN_CFG = {
    'dsp':        0.1,    # TODO: may differ from go profile for sharper turns
    'gait_param': None,
    'arm_swap':   False,
    'step_num':   1,
}


class {{PROJECT_CLASS}}Node(common.CommonAiNex):

    def __init__(self):
        rospy.init_node('{{PROJECT}}_bt', log_level=rospy.INFO)
        super().__init__(common.RunningMode.REAL)
        self._logger         = None
        self._tree           = None
        self._runtime_facade = None
        self._runtime_io     = None
        self._adapters       = []
        self._adapter_lock   = __import__('threading').Lock()
        self._tick_id        = [0]
        signal.signal(signal.SIGINT,  self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)

    def _get_tick_id(self):
        return self._tick_id[0]

    def start(self):
        # ── Observability logger ─────────────────────────────────────────
        # TODO: wire up DebugEventLogger if observability is needed
        # from xyz_behavior.bt_observability.debug_event_logger import DebugEventLogger
        # self._logger = DebugEventLogger(log_dir=_LOG_DIR, session_id='{{PROJECT}}')

        buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer',
                                     BuzzerState, queue_size=1)

        # ── Execution chain: _RuntimeIO → RuntimeFacade → bootstrap ─────
        self._runtime_io = _RuntimeIO(
            gait_manager=self.gait_manager,
            motion_manager=self.motion_manager,
            buzzer_pub=buzzer_pub,
            logger=self._logger,
            tick_id_getter=self._get_tick_id,
        )
        self._runtime_facade = {{PROJECT_CLASS}}RuntimeFacade(
            runtime_io=self._runtime_io,
            go_cfg=_GO_CFG,
            turn_cfg=_TURN_CFG,
            tick_id_getter=self._get_tick_id,
        )

        # ── Input adapters ───────────────────────────────────────────────
        imu_adapter = ImuBalanceStateAdapter(
            lock=self._adapter_lock,
            logger=self._logger,
            tick_id_getter=self._get_tick_id,
        )
        self._adapters = [imu_adapter]
        # TODO: add project-specific adapters here, e.g.:
        # line_adapter = LineDetectionAdapter(lock=self._adapter_lock, ...)
        # self._adapters.append(line_adapter)

        # ── Tree bootstrap ───────────────────────────────────────────────
        self._tree = bootstrap(
            runtime_facade=self._runtime_facade,
            robot_state_setter=imu_adapter.force_state,
            logger=self._logger,
            tick_id_getter=self._get_tick_id,
        )
        self._tree.setup(timeout=15)

        # ── Infra manifest ───────────────────────────────────────────────
        manifest = build_infra_manifest(rospy.get_name())
        write_infra_manifest(manifest, log_dir=_LOG_DIR)

        # ── Walk ready ───────────────────────────────────────────────────
        # TODO: use runtime_facade if stand/walk_ready are added to the facade contract
        self.gait_manager.walk_ready()
        rospy.sleep(0.5)

        rospy.loginfo('[{{PROJECT_CLASS}}Node] BT node ready')
        rate = rospy.Rate(15)
        while not rospy.is_shutdown() and self.is_running:
            # Two-phase latch: snapshot all adapters under lock, then write BB
            with self._adapter_lock:
                snaps = [a.snapshot_and_reset() for a in self._adapters]
            for adapter, snap in zip(self._adapters, snaps):
                adapter.write_snapshot(snap, self._tick_id[0])

            self._tree.tick()
            self._tick_id[0] += 1
            rate.sleep()

    def _on_shutdown(self, *_):
        self.is_running = False
        rospy.loginfo('[{{PROJECT_CLASS}}Node] shutdown')


if __name__ == '__main__':
    {{PROJECT_CLASS}}Node().start()
