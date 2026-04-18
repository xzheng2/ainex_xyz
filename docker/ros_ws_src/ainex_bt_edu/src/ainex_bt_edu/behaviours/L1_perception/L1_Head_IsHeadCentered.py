#!/usr/bin/env python3
"""L1_Head_IsHeadCentered — condition: head pan is within threshold of centre."""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Head_IsHeadCentered(AinexBTNode):
    """SUCCESS when /head_pan_pos is within CENTER_THRESHOLD of HEAD_PAN_CENTER.

    Written by L2_Head_FindLineSweep (root-namespace BB key, not /latched/).
    Used to gate L2_Gait_FollowLine in head-sweep mode until body is aligned.
    CENTER_THRESHOLD must match L2_Head_FindLineSweep.CENTER_THRESHOLD (30).
    """

    LEVEL            = 'L1'
    BB_LOG_KEYS      = [BB.HEAD_PAN_POS]
    HEAD_PAN_CENTER  = 500
    CENTER_THRESHOLD = 30

    def __init__(self, name: str = 'L1_Head_IsHeadCentered',
                 logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        # HEAD_PAN_POS lives at root namespace, NOT /latched/ — no namespace arg
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key=BB.HEAD_PAN_POS, access=Access.READ)

    def update(self) -> Status:
        head_pan = self._bb.head_pan_pos
        offset   = abs(head_pan - self.HEAD_PAN_CENTER)
        passed   = (offset <= self.CENTER_THRESHOLD)
        status   = Status.SUCCESS if passed else Status.FAILURE

        if self._logger:
            self._logger.emit_bt({
                'event':    'decision',
                'node':     self.name,
                'tick_id':  self._tick_id_getter(),
                'inputs':   {'head_pan_pos': head_pan, 'offset': offset},
                'status':   str(status),
                'reason':   f'offset {offset} <= {self.CENTER_THRESHOLD}' if passed
                            else f'offset {offset} > {self.CENTER_THRESHOLD}',
            })

        return status
