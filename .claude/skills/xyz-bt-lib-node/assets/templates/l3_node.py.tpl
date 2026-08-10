#!/usr/bin/env python3
"""{{CLASS_NAME}} - {{DESCRIPTION}}

L3 system / process-orchestration node.

Node kind: <dispatch | finite_action | continuous_controller>

L3 sits ABOVE the L2 locomotion/action tier: it orchestrates the lifecycle of
*other programs* (start/stop a registered background process, issue a
system-level command) rather than commanding robot motion. If your node moves
the robot, it is an L2 — use l2_node.py.tpl instead.

BB reads:  none
  Most L3 nodes touch no blackboard at all — the reference node
  (L3_Process_Control) declares BB_READS = BB_WRITES = [] and has no setup().
  Only add a setup() + blackboard client if this node genuinely reads or writes
  a documented BB key.

BB writes: none

Facade calls:
  TODO: list the XyzBTFacade methods this node calls, e.g. start_process /
  stop_process. These two are CONCRETE on XyzBTFacade (they raise
  NotImplementedError by default), so a project that needs them overrides both
  and holds a ProcessManager — see core/process_manager.py.

Action strategy:
  {{DESCRIPTION}}

Strategy helper:
  A class-level dispatch map (see _ACTIONS below) is usually clearer than a
  _select_action() method for L3: the choice is normally a fixed enum, and a map
  lets the constructor reject an invalid value immediately.

Returns:
  SUCCESS: TODO
  RUNNING: TODO (usually never — most L3 nodes dispatch once)
  FAILURE: TODO

Idempotence warning:
  If the facade calls are idempotent (start is a no-op when already running,
  etc.) this node should return SUCCESS exactly once per activation, and the
  tree must not re-tick it continuously — otherwise the target program gets
  repeatedly (re)started or stopped.

Observability:
  May emit 'action_intent' and 'decision' via base-node helpers.
  Never emits ros_out/ros_result or any comm event; those belong to _RuntimeIO.
"""
from py_trees.common import Status
from xyz_bt_lib.core.base_node import XyzL3ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade


class {{CLASS_NAME}}(XyzL3ActionNode):
    """{{DESCRIPTION}}"""

    LEVEL        = 'L3'
    BB_READS     = []
    BB_WRITES    = []
    FACADE_CALLS = [
        # TODO: 'start_process', 'stop_process', ...
    ]
    CONFIG_DEFAULTS = {
        'target': '',        # example: registry key of the program to control
        'action': 'start',   # example: which operation to perform
        # TODO: replace with this node's real orchestration params.
        # These must match __init__ default args. No hard-coded literals in update().
        # NOTE: gait/servo tuning knobs (speed, yaw_limit, head_pan_center, ...)
        # do NOT belong here — those are L2 concerns.
    }

    # Fixed choice -> facade method. Lets __init__ reject a typo immediately
    # instead of failing at the first tick.
    _ACTIONS = {
        'start': 'start_process',
        'stop':  'stop_process',
    }

    # Param order matches the reference node: name, facade, logger,
    # tick_id_getter, then domain params.
    def __init__(self, name: str = {{DEFAULT_NAME}},
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target: str = '',
                 action: str = 'start'):
        """
        Args:
            name: BT node name.
            facade: Project semantic facade implementing XyzBTFacade.
            logger: DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
            target: Registry key of the program to control. Must be provided by
                    the project tree; the empty default is invalid on purpose.
            action: One of _ACTIONS.

        Every CONFIG_DEFAULTS entry must have a matching __init__ arg stored on self._.
        Runtime logic must use self._ fields, not raw literals.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        if action not in self._ACTIONS:
            raise ValueError(
                "{{CLASS_NAME}} action must be one of {}, got {!r}".format(
                    sorted(self._ACTIONS), action))
        self._target = target
        self._action = action

    # No setup() — this node touches no blackboard. Add one ONLY if that changes,
    # and then it must call super().setup(**kwargs) first.

    def initialise(self):
        """Emit action_intent when this action starts.

        Emit the semantic action (what the node is doing), not the class name.
        This method must not emit ros_out. ros_out belongs to _RuntimeIO.
        """
        self.emit_action_intent(
            action=self._action,
            inputs={'target': self._target},
        )

    def update(self) -> Status:
        method = self._ACTIONS[self._action]
        self.call_facade(method, target=self._target)
        self.emit_decision(
            inputs={'target': self._target, 'action': self._action},
            status=Status.SUCCESS,
            reason='{} {!r} dispatched'.format(self._action, self._target),
        )
        return Status.SUCCESS
