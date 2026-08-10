#!/usr/bin/env python3
"""Example tree showcasing the semantic composite factories + offline rendering.

Builds a small line-following tree using the composite factories from
:mod:`xyz_bt_lib.core.composites`, wired with real L1/L2 nodes. It takes a
``facade`` (plus optional ``logger`` / ``tick_id_getter``) so it can be built
off-robot with :class:`~xyz_bt_lib.core.stub_facade.StubFacade` and printed by
``tools/dump_tree.py`` — no ROS, no hardware.

    python3 tools/dump_tree.py demo_composites_tree:build_tree -p examples

The tree shape (and what each factory expresses):

    PrioritySelector(Main)                 ← recovery may preempt the task
    ├── ParallelAll(SensorMonitor)         ← both L1 checks each tick (SuccessOnAll)
    │   ├── L1_Balance_IsStanding
    │   └── L1_Vision_IsObjectDetected
    ├── ReactiveSequence(Guarded)          ← re-checks the guard every tick
    │   ├── L1_Balance_IsStanding          ←   guard: only run task while standing
    │   └── CommittedSequence(FollowTask)  ←   once started, run to completion
    │       ├── L1_Vision_IsObjectDetected
    │       └── L2_Gait_FollowTarget
    └── L2_Motion_StopGait                 ← fallback: stop if guard fails

The SensorMonitor ParallelAll is included so the renderer's Parallel handling is
exercised (glyph ``/_/``, and no ``[memory=...]`` tag since Parallel has none).
"""
from xyz_bt_lib.core.composites import (
    ReactiveSequence, CommittedSequence, PrioritySelector, ParallelAll,
)
from xyz_bt_lib.behaviours.L1_perception.L1_Balance_IsStanding import (
    L1_Balance_IsStanding,
)
from xyz_bt_lib.behaviours.L1_perception.L1_Vision_IsObjectDetected import (
    L1_Vision_IsObjectDetected,
)
from xyz_bt_lib.behaviours.L2_locomotion.L2_Gait_FollowTarget import (
    L2_Gait_FollowTarget,
)
from xyz_bt_lib.behaviours.L2_locomotion.L2_Motion_StopGait import L2_Motion_StopGait


def build_tree(facade=None, logger=None, tick_id_getter=None):
    """Build the demo line-following tree. Pass a facade (StubFacade for offline)."""
    follow_task = CommittedSequence('FollowTask', children=[
        L1_Vision_IsObjectDetected(target_id='line', logger=logger,
                                   tick_id_getter=tick_id_getter),
        L2_Gait_FollowTarget(target_id='line', facade=facade, logger=logger,
                             tick_id_getter=tick_id_getter),
    ])

    guarded = ReactiveSequence('Guarded', children=[
        L1_Balance_IsStanding(logger=logger, tick_id_getter=tick_id_getter),
        follow_task,
    ])

    # ParallelAll = Parallel(policy=SuccessOnAll): both checks each tick.
    sensor_monitor = ParallelAll('SensorMonitor', children=[
        L1_Balance_IsStanding(logger=logger, tick_id_getter=tick_id_getter),
        L1_Vision_IsObjectDetected(target_id='line', logger=logger,
                                   tick_id_getter=tick_id_getter),
    ])

    return PrioritySelector('Main', children=[
        sensor_monitor,
        guarded,
        L2_Motion_StopGait(facade=facade, logger=logger, tick_id_getter=tick_id_getter),
    ])
