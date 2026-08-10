#!/usr/bin/env python3
"""
{{PROJECT_CLASS}} behavior tree wiring and bootstrap factory.

All leaf nodes receive only the Project RuntimeFacade (XyzBTFacade).
No node may hold gait_manager, motion_manager, or raw ROS references.

Composites: use ONLY the semantic factories from xyz_bt_lib.core.composites —
ReactiveSequence / CommittedSequence / PrioritySelector / CommittedSelector.
NEVER write raw py_trees.composites.Sequence(memory=...) / Selector(memory=...):
the factory name states the contract, and a typo becomes a NameError.

Tree structure (a generic "find a target, then approach it" task — replace the
target and the task branch with your event's own behaviour):
  ReactiveSequence({{PROJECT_CLASS}}BT)
      PrioritySelector(SafetyGate)          <- recovery may preempt at any tick
          --> L1_Balance_IsStanding
          ReactiveSequence(Recovery)
              --> L2_Motion_StopGait
              --> L2_Balance_RecoverFromFall
      PrioritySelector(TaskControl)
          ReactiveSequence(Approach)        <- target visible: walk onto it
              --> L1_Vision_IsObjectDetected(target_id='ball')
              --> L2_Gait_VisionToObject
          ReactiveSequence(Search)          <- target lost: stand still, sweep head
              --> L2_Motion_StopGait
              --> L2_Head_SearchSweep

Note how the search/track state machine lives in the TREE, not inside a node:
L2_Head_SearchSweep only sweeps and reports SUCCESS when the target appears, so
the PrioritySelector is what decides to go back to approaching. Keep new
behaviour in this shape — the tree is the layer humans and agents read.
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
# Reactive-safe asymmetric debounce (N-in / M-out); wrap a CONDITION node with it
# when a signal FLICKERS and the answer must be held steady. Never returns RUNNING.
from xyz_bt_lib.core.hysteresis import HysteresisDecorator

# xyz_bt_lib standard library nodes
from xyz_bt_lib.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from xyz_bt_lib.behaviours.L1_perception.L1_Vision_IsObjectDetected import L1_Vision_IsObjectDetected
from xyz_bt_lib.behaviours.L2_locomotion.L2_Motion_StopGait import L2_Motion_StopGait
from xyz_bt_lib.behaviours.L2_locomotion.L2_Gait_VisionToObject import L2_Gait_VisionToObject
from xyz_bt_lib.behaviours.L2_locomotion.L2_Head_SearchSweep import L2_Head_SearchSweep
from xyz_bt_lib.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall
# TODO: import project-specific nodes from behaviours/actions.py if needed


def bootstrap(runtime_facade, robot_state_setter=None,
              logger=None, tick_id_getter=None,
              target_id='ball',
              stand_confirm_ticks=5):
    """Build and return the {{PROJECT_CLASS}} behaviour tree.

    Args:
        runtime_facade:     {{PROJECT_CLASS}}RuntimeFacade instance (XyzBTFacade).
        robot_state_setter: callable(str) to sync ImuBalanceStateAdapter live store.
        logger:             BT observability logger (None = zero-cost no-op).
        tick_id_getter:     callable → current tick_id (int).
        target_id:          key in /latched/tracked_objects to act on. Must match a
                            key in the adapter's target_specs (see app/ layer).
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

    gait_stop_recovery = L2_Motion_StopGait(
        'StopGait_recovery',
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

    is_object_detected = L1_Vision_IsObjectDetected(
        'IsObjectDetected',
        logger=logger, tick_id_getter=tick_id_getter,
        target_id=target_id,
        lost_count_threshold=0,
    )

    approach_object = L2_Gait_VisionToObject(
        'ApproachObject',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter,
        target_id=target_id,
        # Pixel alignment target. align_y is the distance proxy: without it the
        # node aligns laterally and never advances.
        align_x=320, align_y=360,
        x_error_threshold=30, y_error_threshold=30,
        x_speed=0.010,
        # Gait pass-through goes in ONE dict (XyzL2GaitActionNode); never as
        # per-knob arguments:
        gait_param={'step_height': 0.03},
    )

    gait_stop_search = L2_Motion_StopGait(
        'StopGait_search',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter)

    search_sweep = L2_Head_SearchSweep(
        'SearchSweep',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter,
        target_id=target_id,
        # state_key MUST be a hardcoded string LITERAL — sweep progress lives at
        # /node_state/<state_key> so it resumes across Search/Approach flicker.
        # Must be unique across every node using /node_state/, including the
        # LatchedDwell / Hysteresis decorators.
        state_key='target_search',
        sweep_left_pos=700, sweep_right_pos=300, sweep_step=10,
    )

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

    approach_seq = ReactiveSequence('Approach',
                                    children=[is_object_detected, approach_object])

    # Standing still while sweeping is the TREE's decision, not the head node's —
    # L2_Head_SearchSweep never commands the gait itself.
    search_seq = ReactiveSequence('Search',
                                  children=[gait_stop_search, search_sweep])

    task_control = PrioritySelector('TaskControl',
                                    children=[approach_seq, search_seq])

    root = ReactiveSequence('{{PROJECT_CLASS}}BT',
                            children=[safety_gate, task_control])

    return py_trees.trees.BehaviourTree(root)
