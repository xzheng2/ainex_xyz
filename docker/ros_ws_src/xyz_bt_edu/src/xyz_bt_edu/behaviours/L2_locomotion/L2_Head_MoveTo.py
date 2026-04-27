#!/usr/bin/env python3
"""L2 Action: move the head-pan servo to a fixed position.

Node kind: dispatch

BB reads:  none
BB writes: BB.HEAD_PAN_POS (/head_pan_pos) — commanded target (not sensor feedback)
Facade:    move_head(pan_pos)  — convenience wrapper over set_servos_position()
Strategy:  none (trivial: always dispatches the configured pan position)

Returns:
    SUCCESS → head command dispatched (fire-and-forget)

CONFIG_DEFAULTS:
    pan_pos: 500  — servo centre position

Note: this node controls head pan only. Servo 23 (pan) is driven by the
facade; servo 24 (tilt) is not addressed here and retains its current position.
"""
from py_trees.common import Access, Status
from xyz_bt_edu.base_node import XyzL2ActionNode
from xyz_bt_edu.base_facade import XyzBTFacade
from xyz_bt_edu.blackboard_keys import BB


class L2_Head_MoveTo(XyzL2ActionNode):
    """Move head-pan to a fixed position via facade. Always returns SUCCESS."""

    LEVEL        = 'L2'
    BB_READS     = []
    BB_WRITES    = [BB.HEAD_PAN_POS]
    FACADE_CALLS = ['move_head']
    CONFIG_DEFAULTS = {'pan_pos': 500}

    def __init__(self, name: str = 'L2_Head_MoveTo',
                 facade: XyzBTFacade = None,
                 pan_pos: int = 500,
                 logger=None,
                 tick_id_getter=None):
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._pan_pos = pan_pos
        self._bb_pan  = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb_pan = self.attach_blackboard_client(name=self.name)
        self._bb_pan.register_key(key=BB.HEAD_PAN_POS, access=Access.WRITE)

    def initialise(self):
        self.emit_action_intent(action='move_head', inputs={'pan_pos': self._pan_pos})

    def update(self) -> Status:
        self.call_facade('move_head', pan_pos=self._pan_pos)
        self._bb_pan.head_pan_pos = self._pan_pos  # commanded state, not sensor feedback
        self.emit_decision(
            inputs={'pan_pos': self._pan_pos},
            status=Status.SUCCESS,
            reason='head command dispatched',
            bb_writes={BB.HEAD_PAN_POS: self._pan_pos},
        )
        return Status.SUCCESS
