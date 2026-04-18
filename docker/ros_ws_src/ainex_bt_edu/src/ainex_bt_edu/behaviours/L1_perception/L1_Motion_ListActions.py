#!/usr/bin/env python3
"""L1 Action: scan action file directory and write list to blackboard.

Scans ACTION_PATH for .d6a action group files, writes the sorted name list
to BB.AVAILABLE_ACTIONS every tick.

Always returns SUCCESS (even if the directory is missing — writes empty list).
No ROS subscription needed; this is a pure filesystem scan.
"""
import os
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB

_MISSION_NS = '/mission'
_AVAILABLE_ACTIONS_KEY = 'available_actions'


class L1_Motion_ListActions(AinexBTNode):
    """Scan action files and write available action names to BB."""

    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.AVAILABLE_ACTIONS]

    ACTION_PATH = '/home/ubuntu/software/ainex_controller/ActionGroups'

    def __init__(self, action_path: str = None,
                 name: str = 'L1_Motion_ListActions',
                 logger=None, tick_id_getter=None):
        super().__init__(name)
        self._action_path = action_path or self.ACTION_PATH
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None
        self._actions = []

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=_MISSION_NS)
        self._bb.register_key(key=_AVAILABLE_ACTIONS_KEY, access=Access.WRITE)
        self._scan_actions()

    def _scan_actions(self) -> None:
        if os.path.isdir(self._action_path):
            self._actions = sorted([
                f[:-4] for f in os.listdir(self._action_path)
                if f.endswith('.d6a')
            ])
        else:
            self._actions = []

    def update(self) -> Status:
        self._bb.available_actions = self._actions

        if self._logger:
            self._logger.emit_bt({
                'event':  'decision',
                'node':   self.name,
                'inputs': {'action_count': len(self._actions)},
                'status': str(Status.SUCCESS),
                'reason': f"listed {len(self._actions)} actions from {self._action_path}",
            })

        return Status.SUCCESS
