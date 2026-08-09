#!/usr/bin/env python3
"""
{{PROJECT_CLASS}} behavior tree wiring and bootstrap factory.

All leaf nodes receive only the Project RuntimeFacade (XyzBTFacade).
No node may hold gait_manager, motion_manager, or raw ROS references.

Composites: use ONLY the semantic factories from xyz_bt_lib.core.composites —
ReactiveSequence / CommittedSequence / PrioritySelector / CommittedSelector.
NEVER write raw py_trees.composites.Sequence(memory=...) / Selector(memory=...):
the factory name states the contract, and a typo becomes a NameError.

Tree structure:
  ReactiveSequence({{PROJECT_CLASS}}BT)
      PrioritySelector(SafetyGate)
          --> L1_Balance_IsStanding
          ReactiveSequence(Recovery)
              --> L2_Gait_Stop
              --> L2_Balance_RecoverFromFall
      PrioritySelector(PatrolControl)
          ReactiveSequence(LineFollowing)
              --> L1_Vision_IsLineDetected
              --> L2_Gait_FollowLine
          --> L2_Gait_FindLine
          --> L2_Gait_Stop
"""
import py_trees

# Semantic composite factories (NEVER raw Sequence/Selector/Parallel).
# For Parallel use ParallelAll (SuccessOnAll) / ParallelAny (SuccessOnOne).
from xyz_bt_lib.core.composites import (
    ReactiveSequence, CommittedSequence, PrioritySelector, CommittedSelector,
    ParallelAll, ParallelAny,
)
# Reactive-safe stability gate (BB-backed dwell); wrap a CONDITION node with it
# when a tree needs "condition stable for N ticks".
from xyz_bt_lib.core.latched_dwell import LatchedDwellDecorator

# xyz_bt_lib standard library nodes
from xyz_bt_lib.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from xyz_bt_lib.behaviours.L1_perception.L1_Vision_IsLineDetected import L1_Vision_IsLineDetected
from xyz_bt_lib.behaviours.L2_locomotion.L2_Gait_Stop import L2_Gait_Stop
from xyz_bt_lib.behaviours.L2_locomotion.L2_Gait_FollowLine import L2_Gait_FollowLine
from xyz_bt_lib.behaviours.L2_locomotion.L2_Gait_FindLine import L2_Gait_FindLine
from xyz_bt_lib.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall
# TODO: import project-specific nodes from behaviours/actions.py if needed


def bootstrap(runtime_facade, robot_state_setter=None,
              logger=None, tick_id_getter=None,
              stand_confirm_ticks=5):
    """Build and return the {{PROJECT_CLASS}} behaviour tree.

    Args:
        runtime_facade:     {{PROJECT_CLASS}}RuntimeFacade instance (XyzBTFacade).
        robot_state_setter: callable(str) to sync ImuBalanceStateAdapter live store.
        logger:             BT observability logger (None = zero-cost no-op).
        tick_id_getter:     callable → current tick_id (int).
        stand_confirm_ticks: numeric tuning for a LatchedDwellDecorator, injected
                            as a build parameter. Numeric knobs (required_ticks
                            etc.) are passed in here; rosparam is read ONLY in the
                            app/ layer and threaded through as an argument — never
                            read rosparam inside tree wiring.

    Returns:
        py_trees.trees.BehaviourTree
    """
    # ── Nodes ────────────────────────────────────────────────────────────
    is_standing = L1_Balance_IsStanding(
        'IsStanding',
        logger=logger, tick_id_getter=tick_id_getter,
        # CONFIG_DEFAULTS — override to tune:
        expected_stand_label='stand',
    )

    gait_stop_recovery = L2_Gait_Stop(
        'L2_Gait_Stop_recovery',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter)

    recover_from_fall = L2_Balance_RecoverFromFall(
        'RecoverFromFall',
        facade=runtime_facade,
        robot_state_setter=robot_state_setter,
        logger=logger, tick_id_getter=tick_id_getter,
        # CONFIG_DEFAULTS — override to tune:
        lie_action='lie_to_stand',
        recline_action='recline_to_stand',
        buzzer_freq=1900,
        buzzer_on_time=0.1,
        buzzer_off_time=0.01,
        buzzer_repeat=1,
        pre_action_delay_s=2.0,
        post_action_delay_s=0.5,
    )

    is_line_detected = L1_Vision_IsLineDetected(
        'IsLineDetected',
        logger=logger, tick_id_getter=tick_id_getter)

    follow_line = L2_Gait_FollowLine(
        'FollowLine',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter,
        # CONFIG_DEFAULTS — override to tune:
        x_range=[0, 0.015],
        yaw_range=[-8, 10],
        deadband_px=10,
        go_turn_threshold=4,
        head_pan_center=500,
        hi_yaw_threshold=6,
        x_hi_yaw=0.008,
    )

    find_line = L2_Gait_FindLine(
        'FindLine',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter,
        # CONFIG_DEFAULTS — override to tune:
        base_turn_deg=3,
        max_turn_deg=7,
        count_scale_at=30,
        default_turn_deg=3,
        right_turn_deg=5,
    )

    gait_stop_fallback = L2_Gait_Stop(
        name='L2_Gait_Stop_fallback',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter)

    # ── Tree (semantic factories only — never raw Sequence/Selector) ─────
    recovery_seq = ReactiveSequence('Recovery',
                                    children=[gait_stop_recovery, recover_from_fall])

    # SafetyGate: recovery may preempt the task at any tick → PrioritySelector.
    #
    # If you need "confirm the robot is stably standing for N ticks" before
    # proceeding, wrap the CONDITION (not an action) in LatchedDwellDecorator.
    # It is BB-backed, so it survives the reactive re-entry that would reset an
    # instance-counter dwell. Rules:
    #   - state_key MUST be a hardcoded string LITERAL — never from a variable,
    #     constructor arg, or rosparam. It is a state identity: it must be
    #     greppable, and a duplicate must fail loudly at construction.
    #   - required_ticks (a NUMBER) comes from a bootstrap() parameter.
    #
    #   stand_confirmed = LatchedDwellDecorator(
    #       is_standing, required_ticks=stand_confirm_ticks,
    #       state_key='safety_stand_confirmed',   # hardcoded literal
    #       tick_id_getter=tick_id_getter)
    #   safety_gate = PrioritySelector('SafetyGate',
    #                                  children=[stand_confirmed, recovery_seq])
    safety_gate = PrioritySelector('SafetyGate',
                                   children=[is_standing, recovery_seq])

    line_following_seq = ReactiveSequence('LineFollowing',
                                          children=[is_line_detected, follow_line])

    patrol_control = PrioritySelector(
        'PatrolControl',
        children=[line_following_seq, find_line, gait_stop_fallback])

    root = ReactiveSequence('{{PROJECT_CLASS}}BT',
                            children=[safety_gate, patrol_control])

    return py_trees.trees.BehaviourTree(root)
