#!/usr/bin/env python3
"""{{PROJECT_CLASS}}-specific action behaviour nodes.

Only add nodes here when NO generic equivalent exists in xyz_bt_lib.
Check xyz_bt_lib/behaviours/ first — if a generic node exists, import it
directly in tree/{{PROJECT}}_bt.py instead of reimplementing here.

Generic nodes available in xyz_bt_lib (see each file's docstring for the full
contract — there is no central spec document):
  L1_perception/  L1_Balance_IsStanding, L1_Balance_IsFallen, L1_Head_IsHeadCentered,
                  L1_Vision_IsObjectDetected, L1_Vision_IsObjectPositioned,
                  L1_Vision_IsObjectStill, L1_Vision_IsTargetOnSide,
                  L1_Nav_HasPath, L1_Nav_IsObstacleNear, L1_Nav_IsObstacleDetected,
                  L1_Nav_IsHeadingAligned
  L2_locomotion/  L2_Motion_StopGait, L2_Gait_StepNum, L2_Motion_PauseAfterTicks,
                  L2_Gait_FollowTarget, L2_Gait_SearchTurn, L2_Gait_VisionToObject,
                  L2_Gait_ControlToObject, L2_Gait_AlignBodyToHead,
                  L2_Gait_AlignImuHeading, L2_Gait_FollowNavPath,
                  L2_Head_MoveTo, L2_Head_SearchSweep, L2_Head_TrackTarget,
                  L2_Motion_RunAction, L2_Balance_RecoverFromFall
  L3_system/      L3_Process_Control (base: XyzL3ActionNode — orchestrates other
                  programs, not motion)

There is no "line" node family any more: a detected line is just a target in
/latched/tracked_objects (shape='line'), so L1_Vision_IsObjectDetected(
target_id='line') + L2_Gait_FollowTarget(target_id='line') do line following.

Rules for project-specific action nodes (enforce strictly):

  1. Must inherit XyzL2ActionNode — NEVER py_trees.behaviour.Behaviour directly.
     If the node commands the gait (go_step / turn_step), inherit
     XyzL2GaitActionNode instead: it carries the shared gait pass-through knobs
     and gait_kwargs(), so do NOT redeclare period_time_ms / dsp_ratio /
     y_swap_amplitude / arm_swap / step_num / gait_param yourself.
  2. No direct ROS calls, gait_manager, motion_manager, publisher, or service calls.
     All ROS output via: self.call_facade('<method>') → RuntimeFacade → _RuntimeIO.
  3. logger=None must be zero-cost no-op (base helpers handle this).
  4. Debug log chain:
       - BT node: self.emit_decision() and/or self.emit_action_intent() only.
       - ros_out is emitted ONLY by _RuntimeIO._emit() — never here.
  5. Declare BB_READS, BB_WRITES, FACADE_CALLS, CONFIG_DEFAULTS as class attributes.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode, XyzL2GaitActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


# ── Example project-specific action ──────────────────────────────────────────
# Remove or replace with your actual project-specific node.

class L2_Example_Action(XyzL2ActionNode):
    """Example project-specific L2 action node.

    Replace this with your actual node.
    """

    LEVEL        = 'L2'
    BB_READS     = []
    BB_WRITES    = []
    FACADE_CALLS = ['go_step']   # list every RuntimeFacade method called
    CONFIG_DEFAULTS = {
        'example_param': 1.0,
    }

    def __init__(self, name: str = 'L2_Example_Action',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 example_param: float = 1.0):
        """
        CONFIG_DEFAULTS:
            example_param: 1.0
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._example_param = example_param

    def initialise(self):
        self.emit_action_intent(action='example_action')

    def update(self) -> Status:
        # All ROS output through call_facade → RuntimeFacade → _RuntimeIO.
        # Never call self._facade._io directly or use rospy here.
        self.call_facade('go_step', x=self._example_param, y=0, yaw=0,
                         semantic_source='example_action')
        self.emit_decision(
            inputs={'example_param': self._example_param},
            status=Status.SUCCESS,
            reason='example step dispatched',
        )
        return Status.SUCCESS
