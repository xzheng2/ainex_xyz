#!/usr/bin/env python3
"""{{PROJECT_CLASS}}-specific action behaviour nodes.

Only add nodes here when NO generic equivalent exists in ainex_bt_edu.
Check ainex_bt_edu/behaviours/ first — if a generic node exists, import it
directly in tree/{{PROJECT}}_bt.py instead of reimplementing here.

Generic nodes available in ainex_bt_edu:
  L1_perception/L1_Balance_IsStanding
  L1_perception/L1_Head_IsHeadCentered
  L1_perception/L1_Vision_IsLineDetected
  L2_locomotion/L2_Gait_Stop
  L2_locomotion/L2_Gait_FollowLine
  L2_locomotion/L2_Gait_FindLine
  L2_locomotion/L2_Head_FindLineSweep
  L2_locomotion/L2_Balance_RecoverFromFall

Rules for project-specific action nodes (enforce strictly):

  1. Must inherit AinexL2ActionNode — NEVER py_trees.behaviour.Behaviour directly.
  2. No direct rospy, gait_manager, motion_manager, publisher, or service calls.
     All ROS output via: self._facade.<method>() → semantic_facade → comm_facade.
  3. logger=None must be zero-cost no-op (base helpers handle this).
  4. Debug log chain:
       - BT node: self.emit_decision() and/or self.emit_action_intent() only.
       - ros_out is emitted ONLY by comm_facade._emit() — never here.
       - Full attribution: BT intent → semantic_facade → comm_facade → ros_out.
  5. algorithms/ results surface via semantic_facade payload/summary into
     comm_facade's _emit() call — not directly in the BT node.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL2ActionNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


# ── Example project-specific action ──────────────────────────────────────────
# Remove or replace with your actual project-specific node.

# class DoSomethingAction(AinexL2ActionNode):
#     """RUNNING while executing; SUCCESS on completion; FAILURE on error.
#
#     Project-specific: no generic equivalent in ainex_bt_edu.
#
#     System Thinking Checklist (complete BEFORE writing this node):
#       1. Confirm the BB keys this node reads are defined in blackboard_keys.py.
#       2. Confirm the input_adapters that write those keys are wired in bt_node.py.
#       3. Add the semantic_facade method this node will call (if it doesn't exist).
#       4. Add the comm_facade method if needed (with _emit() call for ros_out).
#     """
#     LEVEL = 'L2'
#     BB_READS = [BB.ROBOT_STATE]      # absolute path constants — never '/latched/...'
#     BB_WRITES = []
#     FACADE_CALLS = ['do_something']
#     BB_LOG_KEYS = BB_READS           # compatibility alias during migration
#
#     def __init__(self, name: str, facade: AinexBTFacade, logger=None, tick_id_getter=None):
#         super().__init__(name, facade=facade, logger=logger, tick_id_getter=tick_id_getter)
#         self._bb = None
#
#     def setup(self, **kwargs):
#         super().setup(**kwargs)
#         self._bb = self.attach_blackboard_client(name=self.name, namespace=BB.LATCHED_NS)
#         self._bb.register_key(key=BB.ROBOT_STATE_KEY, access=Access.READ)
#
#     def initialise(self):
#         """Called once when node transitions from INVALID → RUNNING."""
#         self.emit_action_intent(
#             action="start do_something",
#             inputs={},
#         )
#
#     def update(self) -> Status:
#         robot_state = self._bb.robot_state
#         # Decision log (optional — only for significant branch choices)
#         self.emit_decision(
#             inputs={"robot_state": str(robot_state)},
#             status=Status.RUNNING,
#             reason="executing do_something",
#         )
#         # All ROS output goes via facade — comm_facade will emit ros_out
#         self.call_facade('do_something')
#         return Status.RUNNING
#
#     def terminate(self, new_status):
#         """Called when node exits RUNNING (success, failure, or preempt)."""
#         pass
