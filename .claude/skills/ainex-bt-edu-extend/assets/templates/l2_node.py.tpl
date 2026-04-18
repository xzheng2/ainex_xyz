#!/usr/bin/env python3
"""{{CLASS_NAME}} — {{DESCRIPTION}}

BB reads:  <list /latched/ keys read>
BB writes: <list /latched/ keys written, or 'none'>

Returns:
    RUNNING  → action dispatched; waiting for next tick (or SUCCESS/FAILURE below)
    SUCCESS  → <fill in success condition>
    FAILURE  → <fill in failure condition, or remove if always RUNNING>

Debug log: this node emits 'action_intent' in initialise() and 'decision' in
update().  ros_out is emitted exclusively by comm_facade._emit() — never here.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class {{CLASS_NAME}}(AinexBTNode):
    """{{DESCRIPTION}}"""

    LEVEL = 'L2'
    BB_LOG_KEYS = {{BB_LOG_KEYS}}

    def __init__(self, name: str = {{DEFAULT_NAME}},
                 facade: AinexBTFacade = None,
                 tick_id_getter=None):
        super().__init__(name)
        self._facade = facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        # Register each BB key this node reads or writes:
        # self._bb.register_key(key=BB.SOME_KEY, access=Access.READ)
        # self._bb.register_key(key=BB.OTHER_KEY, access=Access.WRITE)
        raise NotImplementedError("Fill in register_key() calls and remove this line")

    def initialise(self):
        """Called once when this node transitions from IDLE → RUNNING."""
        if self._logger:
            self._logger.emit_bt({
                'event':  'action_intent',
                'node':   self.name,
                'action': '{{CLASS_NAME}}',    # fill in action description
                'inputs': {},                  # fill in: {'key': value, ...}
            })

    def update(self) -> Status:
        # Read required BB keys, e.g.:
        # value = self._bb.some_key
        # Call the facade method (must already exist in base_facade.py):
        # result = self._facade.some_method(
        #     param=value,
        #     bt_node=self.name,
        #     tick_id=self._tick_id_getter(),
        # )
        raise NotImplementedError("Fill in update() logic and remove this line")

        # Optional: write back to BB
        # self._bb.some_key = new_value

        status = Status.RUNNING   # or SUCCESS / FAILURE based on result

        if self._logger:
            self._logger.emit_bt({
                'event':  'decision',
                'node':   self.name,
                'inputs': {},        # fill in
                'status': str(status),
                'reason': '',        # fill in
            })

        return status

    def terminate(self, new_status: Status):
        """Called when the node stops ticking (SUCCESS, FAILURE, or tree interrupt)."""
        pass   # Add cleanup here if needed (e.g. stop gait)
