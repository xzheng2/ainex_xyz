#!/usr/bin/env python3
"""L2 Action: move the head servo(s) to fixed position(s).

Node kind: dispatch

BB reads:  none
BB writes: none
Facade:    move_head(pan_pos, tilt_pos)  — convenience wrapper over set_servos_position()
Strategy:  none (trivial: always dispatches the configured position(s))

Returns:
    SUCCESS → head command dispatched (fire-and-forget)

CONFIG_DEFAULTS:
    pan_pos:  500  — servo centre position
    tilt_pos: None — when None, tilt servo is not commanded

Note: servo 23 (pan) is always commanded. Servo 24 (tilt) is commanded only
when tilt_pos is provided; otherwise the tilt servo retains its current position.
"""
from py_trees.common import Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Head_MoveTo(XyzL2ActionNode):
    """Move head to fixed position(s) via facade. Always returns SUCCESS."""

    LEVEL        = 'L2'
    BB_READS     = []
    BB_WRITES    = []
    FACADE_CALLS = ['move_head']
    CONFIG_DEFAULTS = {'pan_pos': 500, 'tilt_pos': None}

    def __init__(self, name: str = 'L2_Head_MoveTo',
                 facade: XyzBTFacade = None,
                 pan_pos: int = 500,
                 tilt_pos: int = None,
                 logger=None,
                 tick_id_getter=None):
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._pan_pos  = pan_pos
        self._tilt_pos = tilt_pos

    def initialise(self):
        inputs = {'pan_pos': self._pan_pos}
        if self._tilt_pos is not None:
            inputs['tilt_pos'] = self._tilt_pos
        self.emit_action_intent(action='move_head', inputs=inputs)

    def update(self) -> Status:
        self.call_facade('move_head', pan_pos=self._pan_pos, tilt_pos=self._tilt_pos)
        inputs = {'pan_pos': self._pan_pos}
        if self._tilt_pos is not None:
            inputs['tilt_pos'] = self._tilt_pos
        self.emit_decision(
            inputs=inputs,
            status=Status.SUCCESS,
            reason='head command dispatched',
        )
        return Status.SUCCESS
