#!/usr/bin/env python3
# encoding: utf-8
# {{PROJECT_CLASS}} behavior tree node — {{TASK_DESC}}

import os
import sys
import threading
import time

import rospkg as _rospkg
_PKG_PATH = _rospkg.RosPack().get_path('xyz_behavior')
sys.path.insert(0, _PKG_PATH)

_LOG_DIR = os.path.join(_PKG_PATH, 'log')

import rospy
import py_trees
from ros_robot_controller.msg import BuzzerState

from bt_observability.debug_event_logger import DebugEventLogger
from bt_observability.bt_debug_visitor import BTDebugVisitor
from bt_observability.blackboard_current_writer import BlackboardCurrentWriter
# Shared, not per-project: neither takes a project name, a topic or a node name, and
# both scope themselves through the node's `~` private namespace at runtime.
from bt_observability.tree_publisher import TreeROSPublisher
from bt_observability.bt_exec_controller import BTExecController

from {{PROJECT}}.tree.{{PROJECT}}_bt import bootstrap
# The binding layer. Shared by every project and NOT scaffolded: neither class has
# anything per-event in it. Every per-project knob of the facade travels as a
# constructor argument from this file (_GO_CFG / _TURN_CFG / _YAW_AVG_WINDOW below),
# so no project ever needs to subclass or copy XyzRuntimeFacade.
from bt_ros_io.default_runtime_io import _RuntimeIO
from bt_ros_io.runtime_facade import XyzRuntimeFacade
# Full path, not `from xyz_bt_lib import ...`: the package __init__ is an eager
# aggregator that pulls in bt_runner and its catkin-built xyz_bt_lib.msg, so the short
# form drags ROS message artefacts into anything that imports this module.
from xyz_bt_lib.blackboard.bb_ros_bridge import BlackboardROSBridge
# TODO: import the appropriate detection adapter:
# Colour/OpenCV: from xyz_bt_lib.adapters.object_detection_adapter import ObjectDetectionAdapter
# YOLO:          from xyz_bt_lib.adapters.yolo_object_detection_adapter import YoloObjectDetectionAdapter
# from xyz_bt_lib.adapters.servo_position_adapter import ServoPositionAdapter
# from xyz_bt_lib.adapters.imu_balance_state_adapter import ImuBalanceStateAdapter

_ACTION_GROUPS_PATH = '/home/ubuntu/ros_ws/src/ActionGroups'

# Pruning old run directories belongs to tools/new_run.py, which owns the run lifecycle.
# This node only writes into the directory it is given; deciding which runs survive is
# not an application concern, and doing it here would mean two places implementing the
# same retention rule.

# TODO: define target specs for ObjectDetectionAdapter / YoloObjectDetectionAdapter.
# Keys are the target_id a tree refers to; values filter detections by the
# ObjectInfo fields (None = match anything). A line is just another target:
# _TARGET_SPECS = {
#     'ball': {'label': 'red',  'shape': 'circle'},
#     'line': {'label': None,   'shape': 'line'},
# }

# Static per-profile gait params consumed by the facade wrappers.
# TODO: tune per project. Keys must match XyzRuntimeFacade's go_cfg/turn_cfg contract.
# THIS is the per-project gait tuning surface — it stays here precisely so the facade
# itself has nothing project-specific in it and can be shared.
_GO_CFG   = {'dsp': 0.1, 'gait_param': None, 'arm_swap': 30, 'step_num': 0}
_TURN_CFG = {'dsp': 0.1, 'gait_param': None, 'arm_swap': 30, 'step_num': 0}

# go_step() publishes the average of this many ticks' raw yaw, so the gait executes
# one stable rotation per window instead of chasing every tick.
# TODO: tune to gait_cycle_ms / bt_tick_ms (e.g. 300 ms / 100 ms = 3).
_YAW_AVG_WINDOW = 3

# TODO (only if this project has BB keys of its own): declare them here and add the
# matching second bridge in Step 8. Keys already in BB.ROSA_TOPIC_MAP are published by
# the no-arg bridge — do NOT repeat them.
#
# Never pass an EMPTY map: BlackboardROSBridge does `key_to_topic_map or ROSA_TOPIC_MAP`,
# so `BlackboardROSBridge({})` silently re-publishes the shared map — a second set of
# publishers and a second timer on the same topics. Absent beats empty.
#
# _BB_TOPIC_MAP = {
#     '/{{PROJECT}}/some_flag': '/bt/{{PROJECT}}/bb/some_flag',
# }


def main():
    py_trees.blackboard.Blackboard.storage.clear()
    rospy.init_node('{{PROJECT}}', log_level=rospy.INFO)
    rospy.loginfo('[{{PROJECT_CLASS}}] Starting')

    # ── Step 1: tick_id getter ────────────────────────────────────────────
    _tick_id = [0]
    tick_id_getter = lambda: _tick_id[0]

    # ── Step 2: observability logger ──────────────────────────────────────
    # Every run gets its own directory so runs stop overwriting each other. Writers must
    # target run_dir, never the legacy names in log/: those are symlinks, and these
    # classes finish with os.replace(), which would turn a symlink into a regular file.
    #
    # The directory is INJECTED, not computed. tools/new_run.py resolves it and exports
    # AINEX_RUN_DIR; this node only needs a directory to write into, not the rule for
    # deciding what a run is. That is what keeps a generated project free of
    # xyz_run_lab, and therefore runnable outside the repository it was generated in.
    run_dir = os.environ.get('AINEX_RUN_DIR', '').strip()
    if run_dir:
        rospy.loginfo('[{{PROJECT_CLASS}}] Run directory (tracked run): %s', run_dir)
    else:
        # Standalone fallback. Still PER-RUN, deliberately: the observability filenames
        # are fixed and truncated at startup (DebugEventLogger opens them "w"), so
        # falling back to a shared directory would overwrite the previous run's logs
        # every time — losing data silently, which is worse than any error.
        run_dir = os.path.join(_LOG_DIR, 'runs',
                               time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()))
        rospy.logwarn(
            '[{{PROJECT_CLASS}}] AINEX_RUN_DIR is not set — running UNTRACKED. No '
            'run_meta.json, no provenance, and the legacy symlinks in log/ are NOT '
            'refreshed, so ROSA and the diagnose skill still point at the previous '
            'run. Writing to %s. Open a tracked run with tools/new_run.py.', run_dir)
    os.makedirs(run_dir, exist_ok=True)

    logger = DebugEventLogger(
        bt_lastrun_jsonl=os.path.join(run_dir, 'bt_debug_lastrun.jsonl'),
        comm_lastrun_jsonl=os.path.join(run_dir, 'bt_ros_comm_debug_lastrun.jsonl'),
        rolling_bt_jsonl=os.path.join(run_dir, 'bt_debug_recent.jsonl'),
        rolling_comm_jsonl=os.path.join(run_dir, 'bt_ros_comm_debug_recent.jsonl'),
        tick_id_getter=tick_id_getter,
    )

    # ── Step 3: adapters ─────────────────────────────────────────────────
    lock = threading.Lock()
    # TODO: create project-specific adapter(s).  Choose ONE detection pipeline:
    # Colour/OpenCV:
    # adapter = ObjectDetectionAdapter(
    #     lock=lock, target_specs=_TARGET_SPECS,
    #     image_width=640, image_height=480,
    #     center_x_offset=0.0,   # line following needs the calibrated value here
    #     enable_depth=False,    # True = populate depth_mm/cam_*_m per target
    #     logger=logger, tick_id_getter=tick_id_getter,
    # )
    # YOLO:
    # adapter = YoloObjectDetectionAdapter(
    #     lock=lock, target_specs=_TARGET_SPECS,
    #     image_width=640, image_height=480,
    #     enable_depth=False,
    #     logger=logger, tick_id_getter=tick_id_getter,
    # )

    # ── Step 4: hardware managers ─────────────────────────────────────────
    from ainex_kinematics.gait_manager import GaitManager
    from ainex_kinematics.motion_manager import MotionManager

    gait_manager   = GaitManager()
    motion_manager = MotionManager(action_path=_ACTION_GROUPS_PATH)

    # _RuntimeIO is the SOLE raw-ROS egress, so the buzzer publisher is created
    # here and handed to it — never published from a BT node.
    buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer',
                                 BuzzerState, queue_size=1)

    runtime_io = _RuntimeIO(gait_manager, motion_manager, buzzer_pub,
                            logger=logger, tick_id_getter=tick_id_getter)
    facade     = XyzRuntimeFacade(runtime_io, _GO_CFG, _TURN_CFG,
                                  tick_id_getter=tick_id_getter,
                                  yaw_avg_window=_YAW_AVG_WINDOW)

    # ── Step 5: hardware init ─────────────────────────────────────────────
    facade.enable_gait()
    motion_manager.run_action('walk_ready')
    rospy.loginfo('[{{PROJECT_CLASS}}] Hardware ready')

    # ── Step 6: build tree ────────────────────────────────────────────────
    # bootstrap() already returns a py_trees.trees.BehaviourTree — do not re-wrap.
    # TODO: pass robot_state_setter=<adapter>.force_state so RecoverFromFall can
    # sync the IMU adapter's live store when it writes robot_state='pending'.
    tree = bootstrap(facade, logger=logger, tick_id_getter=tick_id_getter)

    _tree_pub = TreeROSPublisher(tree)

    # ── Step 7: wire BTDebugVisitor ───────────────────────────────────────
    visitor = BTDebugVisitor(logger=logger, tick_id_getter=tick_id_getter)
    tree.pre_tick_handlers.append(visitor.on_tree_tick_start)
    tree.post_tick_handlers.append(visitor.on_tree_tick_end)
    tree.visitors.append(visitor)

    # ── Step 8: start BB bridge ───────────────────────────────────────────
    # No argument = the shared BB.ROSA_TOPIC_MAP.
    BlackboardROSBridge().start(rate_hz=10)
    # Add alongside it, only when _BB_TOPIC_MAP above is defined and non-empty:
    # BlackboardROSBridge(_BB_TOPIC_MAP).start(rate_hz=10)

    # ── Step 9: BT exec controller ────────────────────────────────────────
    exec_lock = threading.Lock()
    exec_ctrl = BTExecController(exec_lock)

    # ── BB current-state snapshot writer ─────────────────────────────────
    bb_writer = BlackboardCurrentWriter(os.path.join(run_dir, 'bb_current.json'))

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
            if tree.root.status == py_trees.common.Status.SUCCESS:
                rospy.loginfo('[{{PROJECT_CLASS}}] Done — shutting down')
                break

        rate.sleep()

    logger.close()
    rospy.loginfo('[{{PROJECT_CLASS}}] Shutdown')


if __name__ == '__main__':
    main()
