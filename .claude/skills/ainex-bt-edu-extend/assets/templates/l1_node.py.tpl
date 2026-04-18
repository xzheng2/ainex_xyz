#!/usr/bin/env python3
"""{{CLASS_NAME}} — {{DESCRIPTION}}

Reads from the blackboard (all keys under /latched/).

SUCCESS  → <fill in SUCCESS condition>
FAILURE  → <fill in FAILURE condition>
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class {{CLASS_NAME}}(AinexBTNode):
    """{{DESCRIPTION}}"""

    LEVEL = 'L1'
    BB_LOG_KEYS = {{BB_LOG_KEYS}}

    def __init__(self, name: str = {{DEFAULT_NAME}},
                 logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        # Register each BB key this node reads:
        # self._bb.register_key(key=BB.SOME_KEY, access=Access.READ)
        raise NotImplementedError("Fill in register_key() calls and remove this line")

    def update(self) -> Status:
        # Read required BB keys, e.g.:
        # value = self._bb.some_key
        # passed = (value == expected)
        # status = Status.SUCCESS if passed else Status.FAILURE
        raise NotImplementedError("Fill in update() logic and remove this line")

        if self._logger:
            self._logger.emit_bt({
                'event':  'decision',
                'node':   self.name,
                'inputs': {},        # fill in: {'key': value, ...}
                'status': str(status),
                'reason': '',        # fill in: human-readable reason string
            })

        return status
