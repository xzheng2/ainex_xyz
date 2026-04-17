#!/usr/bin/env python3
"""{{PROJECT_CLASS}}-specific condition behaviours.

Only place nodes here that have NO generic equivalent in ainex_bt_edu.
Check ainex_bt_edu/behaviours/ first — if a generic node exists, use it
from tree/{{PROJECT}}_bt.py instead of reimplementing here.

Generic nodes available in ainex_bt_edu:
  L1_perception/L1_Balance_IsStanding
  L1_perception/L1_Vision_IsLineDetected
  L2_locomotion/L2_Gait_Stop
  L2_locomotion/L2_Gait_FollowLine
  L2_locomotion/L2_Gait_FindLine
  L2_locomotion/L2_Head_FindLineSweep
  L2_locomotion/L2_Balance_RecoverFromFall
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode


# ── Example project-specific condition ───────────────────────────────────────
# Remove or replace with your actual project-specific node.

# class IsAtStartingPosition(AinexBTNode):
#     """SUCCESS when the robot is detected at the starting position.
#
#     Project-specific: reads /bt/{{PROJECT}}/bb/start_marker BB key.
#     There is no generic equivalent in ainex_bt_edu.
#     """
#     LEVEL = 'L1'
#     BB_LOG_KEYS = ['/latched/start_marker']
#
#     def __init__(self, name: str, logger=None, tick_id_getter=None):
#         super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
#         self.bb = None
#
#     def setup(self, **kwargs):
#         self.bb = self.attach_blackboard_client(name=self.name)
#         self.bb.register_key(key="/latched/start_marker", access=Access.READ)
#
#     def update(self) -> Status:
#         marker = self.bb.start_marker
#         result = marker is not None and marker.detected
#         status = Status.SUCCESS if result else Status.FAILURE
#         if self._logger:
#             self._logger.emit_bt({
#                 "event":  "decision",
#                 "node":   self.name,
#                 "inputs": {"start_marker": str(marker)},
#                 "status": str(status),
#                 "reason": "start marker detected" if result else "no start marker",
#             })
#         return status
