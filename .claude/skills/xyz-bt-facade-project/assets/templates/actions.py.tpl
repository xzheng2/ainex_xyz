#!/usr/bin/env python3
"""{{PROJECT_CLASS}}-specific action behaviour nodes.

Only add nodes here when NO generic equivalent exists in xyz_bt_edu.
Check xyz_bt_edu/behaviours/ first — if a generic node exists, import it
directly in tree/{{PROJECT}}_bt.py instead of reimplementing here.

Generic nodes available in xyz_bt_edu:
  L1_perception/L1_Balance_IsStanding
  L1_perception/L1_Head_IsHeadCentered
  L1_perception/L1_Vision_IsLineDetected
  L2_locomotion/L2_Gait_Stop
  L2_locomotion/L2_Gait_FollowLine
  L2_locomotion/L2_Gait_FindLine
  L2_locomotion/L2_Head_FindLineSweep
  L2_locomotion/L2_Balance_RecoverFromFall

Rules for project-specific action nodes (enforce strictly):

  1. Must inherit XyzL2ActionNode — NEVER py_trees.behaviour.Behaviour directly.
  2. No direct ROS calls, gait_manager, motion_manager, publisher, or service calls.
     All ROS output via: self.call_facade('<method>') → RuntimeFacade → _RuntimeIO.
  3. logger=None must be zero-cost no-op (base helpers handle this).
  4. Debug log chain:
       - BT node: self.emit_decision() and/or self.emit_action_intent() only.
       - ros_out is emitted ONLY by _RuntimeIO._emit() — never here.
  5. Declare BB_READS, BB_WRITES, FACADE_CALLS, CONFIG_DEFAULTS as class attributes.
"""
from py_trees.common import Access, Status
from xyz_bt_edu.base_node import XyzL2ActionNode
from xyz_bt_edu.base_facade import XyzBTFacade
from xyz_bt_edu.blackboard_keys import BB


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
