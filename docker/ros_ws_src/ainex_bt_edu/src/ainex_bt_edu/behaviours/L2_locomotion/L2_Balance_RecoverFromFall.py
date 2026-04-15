#!/usr/bin/env python3
"""L2 Action: blocking fall-recovery sequence.

Reads  /latched/robot_state to select the correct stand-up action.
Writes /latched/robot_state = 'stand' on completion.

Delegates the full recovery sequence (buzzer → disable gait → stand-up action)
to the project SemanticFacade; this node only owns blackboard I/O and the
optional post-recovery setter hook.

Reference implementation: marathon/behaviours/actions.py :: RecoverFromFall
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade


class L2_Balance_RecoverFromFall(AinexBTNode):
    """Execute fall recovery via facade, then mark robot_state='stand'."""

    LEVEL = 'L2'
    BB_LOG_KEYS = ['/latched/robot_state']

    def __init__(self, name: str = 'L2_Balance_RecoverFromFall',
                 facade: AinexBTFacade = None,
                 robot_state_setter=None,
                 tick_id_getter=None):
        super().__init__(name)
        self._facade = facade
        # Called after writing 'stand' to the latched BB so the live store is
        # also updated — prevents the next pre-tick latch from overwriting
        # the latched key with the stale pre-recovery value.
        self._robot_state_setter = robot_state_setter
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace='/latched')
        self._bb.register_key(key='robot_state', access=Access.WRITE)

    def update(self) -> Status:
        state = self._bb.robot_state
        self._facade.recover_from_fall(
            robot_state=state,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        self._bb.robot_state = 'stand'
        # Push 'stand' into the live store so the next pre-tick latch
        # does not overwrite the BB with the stale pre-recovery live value.
        if self._robot_state_setter is not None:
            self._robot_state_setter('stand')
        return Status.SUCCESS
