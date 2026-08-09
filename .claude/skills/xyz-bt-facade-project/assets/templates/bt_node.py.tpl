#!/usr/bin/env python3
# encoding: utf-8
# {{PROJECT_CLASS}} behavior tree node — {{TASK_DESC}}

import os
import sys
import threading

import rospkg as _rospkg
_XYZ_BEHAV_DIR = _rospkg.RosPack().get_path('xyz_behavior')
sys.path.insert(0, _XYZ_BEHAV_DIR)

_LOG_DIR = os.path.join(_XYZ_BEHAV_DIR, 'log')

import rospy
import py_trees

from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor
from bt_observability.blackboard_current_writer import BlackboardCurrentWriter

from {{PROJECT}}.tree.{{PROJECT}}_bt import bootstrap
from {{PROJECT}}.runtime.runtime_facade import {{PROJECT_CLASS}}RuntimeFacade
from {{PROJECT}}.runtime._runtime_io import _RuntimeIO
from {{PROJECT}}.infra.tree_publisher import TreeROSPublisher
from {{PROJECT}}.infra.bb_ros_bridge import {{PROJECT_CLASS}}BBBridge
from {{PROJECT}}.infra.bt_exec_controller import BTExecController
from {{PROJECT}}.infra.infra_manifest import build_infra_manifest, write_infra_manifest
from xyz_bt_lib import BlackboardROSBridge
# TODO: import the appropriate detection adapter:
# Colour/OpenCV: from xyz_bt_lib.adapters.object_detection_adapter import ObjectDetectionAdapter
# YOLO:          from xyz_bt_lib.adapters.yolo_object_detection_adapter import YoloObjectDetectionAdapter
# from xyz_bt_lib.adapters.servo_position_adapter import ServoPositionAdapter
# from xyz_bt_lib.adapters.imu_balance_state_adapter import ImuBalanceStateAdapter

_ACTION_GROUPS_PATH = '/home/ubuntu/ros_ws/src/ActionGroups'

# TODO: define target specs for ObjectDetectionAdapter / YoloObjectDetectionAdapter if needed, e.g.:
# _TARGET_SPECS = {
#     'object': {'label': 'color', 'shape': 'shape'},
# }


def main():
    py_trees.blackboard.Blackboard.storage.clear()
    rospy.init_node('{{PROJECT}}', log_level=rospy.INFO)
    rospy.loginfo('[{{PROJECT_CLASS}}] Starting')

    # ── Step 1: tick_id getter ────────────────────────────────────────────
    _tick_id = [0]
    tick_id_getter = lambda: _tick_id[0]

    # ── Step 2: observability logger ──────────────────────────────────────
    logger = DebugEventLogger(
        bt_lastrun_jsonl=os.path.join(_LOG_DIR, 'bt_debug_lastrun.jsonl'),
        comm_lastrun_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_lastrun.jsonl'),
        rolling_bt_jsonl=os.path.join(_LOG_DIR, 'bt_debug_recent.jsonl'),
        rolling_comm_jsonl=os.path.join(_LOG_DIR, 'bt_ros_comm_debug_recent.jsonl'),
        tick_id_getter=tick_id_getter,
    )

    # ── Step 3: adapters ─────────────────────────────────────────────────
    lock = threading.Lock()
    # TODO: create project-specific adapter(s).  Choose ONE detection pipeline:
    # Colour/OpenCV:
    # adapter = ObjectDetectionAdapter(
    #     lock=lock, target_specs=_TARGET_SPECS,
    #     image_width=640, image_height=480,
    #     logger=logger, tick_id_getter=tick_id_getter,
    # )
    # YOLO:
    # adapter = YoloObjectDetectionAdapter(
    #     lock=lock, target_specs=_TARGET_SPECS,
    #     image_width=640, image_height=480,
    #     logger=logger, tick_id_getter=tick_id_getter,
    # )

    # ── Step 4: hardware managers ─────────────────────────────────────────
    from ainex_kinematics.gait_manager import GaitManager
    from ainex_kinematics.motion_manager import MotionManager

    gait_manager   = GaitManager()
    motion_manager = MotionManager(action_path=_ACTION_GROUPS_PATH)

    runtime_io = _RuntimeIO(gait_manager, motion_manager,
                             logger=logger, tick_id_getter=tick_id_getter)
    facade     = {{PROJECT_CLASS}}RuntimeFacade(runtime_io)

    # ── Step 5: hardware init ─────────────────────────────────────────────
    facade.enable_gait()
    motion_manager.run_action('walk_ready')
    rospy.loginfo('[{{PROJECT_CLASS}}] Hardware ready')

    # ── Step 6: build tree ────────────────────────────────────────────────
    root = bootstrap(facade, logger=logger, tick_id_getter=tick_id_getter)
    tree = py_trees.trees.BehaviourTree(root)

    _tree_pub = TreeROSPublisher(tree)

    # ── Step 7: wire BTDebugVisitor ───────────────────────────────────────
    visitor = BTDebugVisitor(logger=logger, tick_id_getter=tick_id_getter)
    tree.pre_tick_handlers.append(visitor.on_tree_tick_start)
    tree.post_tick_handlers.append(visitor.on_tree_tick_end)
    tree.visitors.append(visitor)

    # ── Step 8: start BB bridges ──────────────────────────────────────────
    BlackboardROSBridge().start(rate_hz=10)
    {{PROJECT_CLASS}}BBBridge().start(rate_hz=10)

    # ── Step 9: infra comm manifest ───────────────────────────────────────
    _manifest_path = os.path.join(_LOG_DIR, 'infra_comm_manifest_lastrun.json')
    write_infra_manifest(_manifest_path, build_infra_manifest('{{PROJECT}}_bt_node'))
    rospy.loginfo('[{{PROJECT_CLASS}}] Infra manifest written')

    # ── Step 10: BT exec controller ───────────────────────────────────────
    exec_lock = threading.Lock()
    exec_ctrl = BTExecController(exec_lock)

    # ── BB current-state snapshot writer ─────────────────────────────────
    bb_writer = BlackboardCurrentWriter(os.path.join(_LOG_DIR, 'bb_current.json'))

    # ── Step 11: tick loop with per-tick adapter latch ────────────────────
    rospy.loginfo('[{{PROJECT_CLASS}}] Running BT at 10 Hz')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # TODO: drain all adapters under the lock
        # with lock:
        #     snap = adapter.snapshot_and_reset()

        if exec_ctrl.should_tick():
            _tick_id[0] += 1
            # TODO: write snapshots before tree.tick()
            # adapter.write_snapshot(snap, _tick_id[0])
            tree.tick()
            bb_writer.write_current(_tick_id[0], str(tree.root.status))
            if root.status == py_trees.common.Status.SUCCESS:
                rospy.loginfo('[{{PROJECT_CLASS}}] Done — shutting down')
                break

        rate.sleep()

    logger.close()
    rospy.loginfo('[{{PROJECT_CLASS}}] Shutdown')


if __name__ == '__main__':
    main()
