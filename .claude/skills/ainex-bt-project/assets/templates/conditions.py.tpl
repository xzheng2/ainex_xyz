#!/usr/bin/env python3
"""{{PROJECT_CLASS}}-specific condition (and action) behaviour nodes.

This file is for **condition** nodes (L1 perception checks that return
SUCCESS/FAILURE without executing actuator commands).

For **action** nodes (L2+ actuator commands, RUNNING/SUCCESS/FAILURE):
  → use behaviours/actions.py (actions.py.tpl).

Only place nodes here that have NO generic equivalent in ainex_bt_edu.
Check ainex_bt_edu/behaviours/ first — if a generic node exists, use it
from tree/{{PROJECT}}_bt.py instead of reimplementing here.

Generic nodes available in ainex_bt_edu:
  L1_perception/L1_Balance_IsStanding
  L1_perception/L1_Head_IsHeadCentered
  L1_perception/L1_Vision_IsLineDetected
  L2_locomotion/L2_Gait_Stop
  L2_locomotion/L2_Gait_FollowLine
  L2_locomotion/L2_Gait_FindLine
  L2_locomotion/L2_Head_FindLineSweep
  L2_locomotion/L2_Balance_RecoverFromFall
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


# ── Example project-specific condition ───────────────────────────────────────
# Remove or replace with your actual project-specific node.

# class IsAtStartingPosition(AinexBTNode):
#     """SUCCESS when the robot is detected at the starting position.
#
#     Project-specific: no generic equivalent in ainex_bt_edu.
#
#     System Thinking Checklist (complete BEFORE writing this node):
#       1. Add to ainex_bt_edu/blackboard_keys.py:
#            START_MARKER_KEY = 'start_marker'
#            START_MARKER     = BB.LATCHED_NS + '/' + START_MARKER_KEY
#       2. Verify an input_adapter writes BB.START_MARKER every tick.
#          If none exists, create one in ainex_bt_edu/input_adapters/ first.
#       3. Wire the adapter into app/<project>_bt_node.py __init__() + run().
#     """
#     LEVEL = 'L1'
#     BB_LOG_KEYS = [BB.START_MARKER]       # absolute path constant — never '/latched/...'
#
#     def __init__(self, name: str, logger=None, tick_id_getter=None):
#         super().__init__(name)
#         self._logger = logger
#         self._tick_id_getter = tick_id_getter or (lambda: -1)
#         self._bb = None
#
#     def setup(self, **kwargs):
#         super().setup(**kwargs)
#         self._bb = self.attach_blackboard_client(name=self.name, namespace=BB.LATCHED_NS)
#         self._bb.register_key(key=BB.START_MARKER_KEY, access=Access.READ)  # short key constant
#
#     def update(self) -> Status:
#         marker = self._bb.start_marker
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
