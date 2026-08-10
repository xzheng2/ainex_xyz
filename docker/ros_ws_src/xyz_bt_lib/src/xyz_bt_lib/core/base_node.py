#!/usr/bin/env python3
"""Base classes for AiNex behaviour-tree nodes.

The observable-BT runtime owns global tick logging through BTDebugVisitor.
These classes only provide node-side helpers for structured semantic events
and keep the L1/L2 responsibility boundary explicit.
"""
import py_trees
from py_trees.common import Status


class XyzBTNode(py_trees.behaviour.Behaviour):
    """Common base for AiNex BT behaviours.

    This class intentionally does not override ``tick()`` and does not publish
    ``/bt_node_events``. Per-node tick status is captured by BTDebugVisitor.
    Subclasses can use ``emit_bt()`` / ``emit_decision()`` for optional semantic
    events that are not available from the py_trees visitor alone.
    """

    LEVEL = 'BT'
    BB_READS = []
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    # Compatibility alias for existing nodes and older docs/templates.
    BB_LOG_KEYS = []

    def __init__(self, name: str, logger=None, tick_id_getter=None):
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._session_id = 'unknown'
        super().__init__(name)

    def setup(self, **kwargs):
        """Compatibility no-op.

        Existing nodes call ``super().setup(**kwargs)``. Observable BT logging is
        mounted at the tree level, so the base node has no ROS publishers or
        blackboard clients to create here.
        """
        pass

    @property
    def tick_id(self):
        """Return the current externally-owned BT tick id."""
        return self._tick_id_getter()

    def emit_bt(self, payload: dict) -> None:
        """Emit a BT-layer semantic event if a logger was injected."""
        if self._logger is None:
            return
        event = dict(payload)
        event.setdefault('tick_id', self.tick_id)
        event.setdefault('node', self.name)
        event.setdefault('level', self.LEVEL)
        if self._session_id != 'unknown':
            event.setdefault('session_id', self._session_id)
        self._logger.emit_bt(event)

    def emit_decision(self, *, inputs, status, reason, **extra) -> None:
        """Emit a standard decision event.

        Args:
            inputs: Structured input values used by this node's decision.
            status: py_trees Status or status string.
            reason: Human-readable reason for the selected status.
            **extra: Optional additional JSON-serializable fields.
        """
        event = {
            'event': 'decision',
            'inputs': inputs,
            'status': getattr(status, 'name', str(status)),
            'reason': reason,
        }
        event.update(extra)
        self.emit_bt(event)


class XyzL1ConditionNode(XyzBTNode):
    """Base class for L1 condition nodes.

    L1 nodes are PURE PREDICATES: they read blackboard state and return SUCCESS
    or FAILURE only. They must never return RUNNING, hold cross-tick state,
    dispatch actions, call facades, publish/subscribe ROS, or emit
    communication-layer events.

    Stability / timing confirmation (dwell) is NOT an L1 responsibility. When a
    tree needs "condition stable for N ticks", wrap the condition at the tree
    layer in xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator — keep the
    condition node itself a stateless predicate.
    """

    LEVEL = 'L1'
    BB_WRITES = []
    FACADE_CALLS = []

    def status_from_bool(self, passed: bool) -> Status:
        """Convert a boolean judgement to a BT Status: SUCCESS if passed else FAILURE.

        L1 predicates never return RUNNING; dwell/hysteresis belongs to a
        tree-layer decorator (see the class docstring).
        """
        return Status.SUCCESS if passed else Status.FAILURE


class XyzL2ActionNode(XyzBTNode):
    """Base class for L2 action/strategy nodes.

    L2 nodes may read/write documented blackboard keys and dispatch side effects
    only through the injected XyzBTFacade runtime interface.

    All ROS output flows: L2 node → call_facade() → RuntimeFacade → _RuntimeIO.
    L2 nodes must never call _RuntimeIO, managers, or raw ROS APIs directly.
    """

    LEVEL = 'L2'

    def __init__(self, name: str, logger=None, tick_id_getter=None, facade=None):
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._facade = facade

    def emit_action_intent(self, *, action, inputs=None, **extra) -> None:
        """Emit a standard action_intent event for action start observability."""
        event = {
            'event': 'action_intent',
            'action': action,
            'inputs': inputs or {},
        }
        event.update(extra)
        self.emit_bt(event)

    def call_facade(self, method_name: str, **kwargs):
        """Call a facade method with standard BT attribution fields."""
        if self._facade is None:
            raise RuntimeError(
                '{} requires a facade before calling {}'.format(
                    self.name, method_name))
        kwargs.setdefault('bt_node', self.name)
        kwargs.setdefault('tick_id', self.tick_id)
        return getattr(self._facade, method_name)(**kwargs)


class XyzL2GaitActionNode(XyzL2ActionNode):
    """Base for L2 nodes that command the gait via go_step() / turn_step().

    Exists to remove a real duplication: the same block of gait pass-through
    knobs was declared in five nodes, and ``_effective_gait_param()`` was copied
    verbatim into three of them. Every gait node needs the same plumbing and
    none of it is strategy, so it lives here once.

    Two kinds of gait tuning, deliberately kept separate:

    1. **Step kwargs** (``period_time_ms``, ``dsp_ratio``, ``y_swap_amplitude``,
       ``arm_swap``, ``step_num``) — passed as their own arguments to
       ``go_step()`` / ``turn_step()``. ``None`` = use the project default.
    2. **``gait_param``** — a partial WalkingParam dict merged by ``_RuntimeIO``
       onto the controller's current parameters. Keys mirror
       ``GaitManager.get_gait_param()``: ``step_height``, ``hip_pitch_offset``,
       ``init_yaw_offset``, ``init_x_offset``, ``init_y_offset``,
       ``body_height``, ``init_roll_offset``, ``init_pitch_offset``,
       ``step_fb_ratio``, ``z_swap_amplitude``, ``pelvis_offset``.

    Subclasses declare ONLY their strategy parameters in ``CONFIG_DEFAULTS``;
    the gait knobs above are inherited from ``GAIT_CONFIG_DEFAULTS`` and must
    not be repeated. In a project tree, tune them by name inside the dict::

        L2_Gait_StepNum('Forward', facade=f, x=0.02,
                        gait_param={'step_height': 0.03, 'init_yaw_offset': 2})

    Note (Aug 2026): the per-knob constructor arguments (``step_height=``,
    ``init_yaw_offset=``, ``init_z_offset=`` …) that three of these nodes used
    to accept are gone — they only ever merged into ``gait_param``, so the dict
    is now the single way in. ``L2_Gait_FollowNavPath``'s ``init_z_offset``
    alias for ``body_height`` is likewise gone; write ``body_height`` directly.

    Subclasses build their facade call as::

        self.call_facade('go_step', x=x, y=y, yaw=yaw,
                         **self.gait_kwargs(), semantic_source='...')
    """

    GAIT_CONFIG_DEFAULTS = {
        'period_time_ms':   None,   # gait cycle (ms); None = project default
        'dsp_ratio':        None,   # double-support fraction (0-1)
        'y_swap_amplitude': None,   # lateral body swing (m)
        'arm_swap':         None,   # arm swing amplitude (deg)
        'step_num':         None,   # steps per tick (0 = continuous)
        'gait_param':       None,   # partial WalkingParam dict (see class docstring)
    }

    def __init__(self, name: str, logger=None, tick_id_getter=None, facade=None,
                 period_time_ms=None, dsp_ratio=None, y_swap_amplitude=None,
                 arm_swap=None, step_num=None, gait_param=None):
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter,
                         facade=facade)
        self._period_time_ms   = period_time_ms
        self._dsp_ratio        = dsp_ratio
        self._y_swap_amplitude = y_swap_amplitude
        self._arm_swap         = arm_swap
        self._step_num         = step_num
        self._gait_param       = gait_param

    def _effective_gait_param(self):
        """Return a defensive copy of gait_param, or None when nothing overrides.

        Copied rather than passed by reference so a node can never mutate the
        dict the tree handed it (which may be shared between nodes).
        """
        return dict(self._gait_param) if self._gait_param else None

    def gait_kwargs(self) -> dict:
        """Return the standard gait kwargs for go_step() / turn_step()."""
        return {
            'period_time_ms':   self._period_time_ms,
            'dsp_ratio':        self._dsp_ratio,
            'y_swap_amplitude': self._y_swap_amplitude,
            'arm_swap':         self._arm_swap,
            'step_num':         self._step_num,
            'gait_param':       self._effective_gait_param(),
        }


class XyzL3ActionNode(XyzL2ActionNode):
    """Base class for L3 system / process-orchestration nodes.

    L3 nodes sit above the L2 locomotion/action tier: they orchestrate the
    lifecycle of *other* programs (start/stop a registered background process,
    dispatch a system-level command) rather than commanding robot motion.

    They are action-like, so they reuse the full XyzL2ActionNode machinery
    (facade injection, call_facade(), emit_action_intent()); all
    side effects still flow: L3 node → call_facade() → RuntimeFacade → _RuntimeIO.
    L3 nodes must never call _RuntimeIO, managers, or raw ROS APIs directly.
    """

    LEVEL = 'L3'
