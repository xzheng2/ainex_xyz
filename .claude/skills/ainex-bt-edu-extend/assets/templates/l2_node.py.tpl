#!/usr/bin/env python3
"""{{CLASS_NAME}} - {{DESCRIPTION}}

L2 action/strategy node.

BB reads:
  TODO: list BB.* constants read by this node.

BB writes:
  TODO: list BB.* constants written by this node, or 'none'.

Facade calls:
  TODO: list existing AinexBTFacade methods called by this node.

Action strategy:
  {{DESCRIPTION}}

Strategy helper:
  _select_action(...)

Constructor defaults:
  TODO: list every threshold, speed, yaw limit, servo center, state label,
  frame count, etc. Project trees may override these constructor defaults.

Returns:
  RUNNING: TODO
  SUCCESS: TODO
  FAILURE: TODO

Observability:
  May emit 'action_intent' and 'decision' via base-node helpers.
  Never emits ros_out/ros_result or any comm event; those belong to comm_facade.py.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL2ActionNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class {{CLASS_NAME}}(AinexL2ActionNode):
    """{{DESCRIPTION}}"""

    LEVEL = 'L2'
    BB_READS = {{BB_READS}}
    BB_WRITES = [
        # TODO: BB.SOME_OUTPUT, or leave empty
    ]
    FACADE_CALLS = [
        # TODO: 'go_step', 'turn_step', 'move_head', ...
    ]
    CONFIG_DEFAULTS = {
        # TODO: replace with explicit documented defaults, e.g.
        # 'yaw_limit': 8,
        # 'speed': 0.015,
    }
    BB_LOG_KEYS = BB_READS

    def __init__(self, name: str = {{DEFAULT_NAME}},
                 facade: AinexBTFacade = None,
                 tick_id_getter=None,
                 logger=None):
        """
        Args:
            name: BT node name.
            facade: Project semantic facade implementing AinexBTFacade.
            tick_id_getter: Callable returning current tick_id.
            logger: DebugEventLogger-compatible object, or None.

        TODO: add explicit constructor default args for every strategy setting
        instead of hard-coding constants in update().
        """
        super().__init__(
            name,
            facade=facade,
            logger=logger,
            tick_id_getter=tick_id_getter,
        )
        self._bb = None

        # TODO: store constructor defaults on self, e.g.
        # self._yaw_limit = yaw_limit
        # self._speed = speed

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        # Register every BB key this node reads/writes.
        # Use BB.*_KEY for /latched keys and BB.* for root namespace keys.
        # Examples:
        # self._bb.register_key(key=BB.SOME_INPUT_KEY, access=Access.READ)
        # self._bb.register_key(key=BB.SOME_OUTPUT_KEY, access=Access.WRITE)
        raise NotImplementedError("Fill in register_key() calls")

    def initialise(self):
        """Optionally emit action_intent when this action starts.

        This method must not emit ros_out. ros_out belongs to comm_facade.py.
        """
        self.emit_action_intent(
            action='{{CLASS_NAME}}',
            inputs={},  # TODO: fill with relevant current inputs if safe.
        )

    def _select_action(self, *values) -> tuple:
        """Return (facade_method_name, kwargs, reason).

        facade_method_name must name an existing AinexBTFacade method unless the
        user approved a breaking facade migration.

        Use this helper for both simple selection and parameter computation.
        If the computation grows large, split out a separate side-effect-free
        _compute_command(...) helper and call it from here.

        kwargs should contain only facade method parameters owned by the action.
        update() appends bt_node and tick_id.

        This helper must be side-effect-free:
        - no BB reads/writes
        - no facade calls
        - no ROS calls
        - no logger calls
        """
        raise NotImplementedError("Fill in side-effect-free action selection")

    def update(self) -> Status:
        # Read BB values, select facade call, then dispatch.
        # Example:
        # value = self._bb.some_key
        # method_name, kwargs, reason = self._select_action(value)
        # self.call_facade(method_name, **kwargs)
        # status = Status.RUNNING
        # inputs = {'some_key': value}
        raise NotImplementedError("Fill in update() orchestration")

        self.emit_decision(
            inputs=inputs,
            status=status,
            reason=reason,
        )

        return status

    def terminate(self, new_status: Status):
        """Optional cleanup when this node stops ticking."""
        pass
