#!/usr/bin/env python3
"""{{CLASS_NAME}} - {{DESCRIPTION}}

L2 action/strategy node.

Node kind: <dispatch | finite_action | continuous_controller>

BB reads:
  TODO: list BB.* constants read by this node.

BB writes:
  TODO: list BB.* constants written by this node, or 'none'.

Facade calls:
  TODO: list existing XyzBTFacade methods called by this node.

Action strategy:
  {{DESCRIPTION}}

Strategy helper:
  _select_action(...)

CONFIG_DEFAULTS:
  TODO: list every threshold, speed, yaw limit, servo center, state label,
  frame count, etc. Project trees may override these via constructor args.

Returns:
  RUNNING: TODO
  SUCCESS: TODO
  FAILURE: TODO

Observability:
  May emit 'action_intent' and 'decision' via base-node helpers.
  Never emits ros_out/ros_result or any comm event; those belong to _RuntimeIO.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode, XyzL2GaitActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


# ── Which base class? Naming IS the contract ─────────────────────────────────
#
#   File named L2_Gait_*.py   -> MUST inherit XyzL2GaitActionNode
#   Anything else             -> inherit XyzL2ActionNode
#
# The L2_Gait_ prefix means "dispatches gait steps" (go_step / turn_step), and
# xyz_bt_lib_guard.py enforces the pairing from the filename alone. If your node
# only stops the gait or never steps at all, do NOT use the Gait prefix — name it
# L2_Motion_* / L2_Head_* / L2_Balance_* instead (see L2_Motion_StopGait).
#
# XyzL2GaitActionNode carries the six gait pass-through knobs
# (period_time_ms, dsp_ratio, y_swap_amplitude, arm_swap, step_num, gait_param)
# plus gait_kwargs(). A gait node therefore:
#   1. does NOT repeat those six in its own CONFIG_DEFAULTS,
#   2. accepts them in __init__ and forwards them to super(),
#   3. builds its facade call with **self.gait_kwargs(),
#   4. never adds per-knob args (step_height=, init_yaw_offset=, ...) — every
#      WalkingParam key travels inside the single gait_param dict.
# See L2_Gait_StepNum.py for the reference implementation, and the gait variant
# at the bottom of this file for the skeleton.


class {{CLASS_NAME}}(XyzL2ActionNode):
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
        'speed': 0.015,             # example: linear speed (m/step)
        'yaw_limit': 8,             # example: maximum yaw angle (deg)
        'head_pan_center': 500,     # example: head servo center position
        # TODO: add all strategy/tuning constants here.
        # These must match __init__ default args. No hard-coded literals in update().
    }
    BB_LOG_KEYS = BB_READS

    # Param order: name, facade, then domain params, then logger, tick_id_getter.
    def __init__(self, name: str = {{DEFAULT_NAME}},
                 facade: XyzBTFacade = None,
                 speed: float = 0.015,
                 yaw_limit: int = 8,
                 head_pan_center: int = 500,
                 logger=None,
                 tick_id_getter=None):
        """
        Args:
            name: BT node name.
            facade: Project semantic facade implementing XyzBTFacade.
            speed: Linear speed (m/step). Project trees may override.
            yaw_limit: Maximum yaw angle (deg). Project trees may override.
            head_pan_center: Head servo center position.
            logger: DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.

        Every CONFIG_DEFAULTS entry must have a matching __init__ arg stored on self._.
        Runtime logic must use self._ fields, not raw literals.
        """
        super().__init__(
            name,
            facade=facade,
            logger=logger,
            tick_id_getter=tick_id_getter,
        )
        self._bb = None
        self._speed = speed
        self._yaw_limit = yaw_limit
        self._head_pan_center = head_pan_center

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        # Register every BB key this node reads/writes.
        # Use BB.*_KEY for /latched keys and BB.* for root namespace keys.
        # Examples:
        # self._bb.register_key(key=BB.SOME_INPUT_KEY, access=Access.READ)
        # self._bb.register_key(key=BB.SOME_OUTPUT_KEY, access=Access.WRITE)
        # TODO(DELETE THIS LINE once the register_key() calls above are filled in)
        pass

    def initialise(self):
        """Optionally emit action_intent when this action starts.

        This method must not emit ros_out. ros_out belongs to _RuntimeIO.
        """
        self.emit_action_intent(
            action='{{CLASS_NAME}}',
            inputs={},  # TODO: fill with relevant current inputs if safe.
        )

    def _select_action(self, *values) -> tuple:
        """Return (facade_method_name, kwargs, reason).

        facade_method_name must name an existing XyzBTFacade method unless the
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
        # TODO(REPLACE): choose the facade method and its kwargs. Placeholder
        # keeps the unedited template syntactically complete.
        return 'stop_gait', {}, 'not implemented'

    def update(self) -> Status:
        # Read BB values, select facade call, then dispatch.
        # Example:
        # value = self._bb.some_key
        # method_name, kwargs, reason = self._select_action(value)
        # self.call_facade(method_name, **kwargs)
        # status = Status.RUNNING
        # inputs = {'some_key': value}
        #
        # BT-visible state sync: if the facade call changes state represented by a
        # BB key, write that key in the same tick and document it in emit_decision:
        # self.emit_decision(inputs=inputs, status=status, reason=reason,
        #                    bb_writes={BB.SOME_KEY: value_written})
        # TODO(REPLACE): read BB, call _select_action(), dispatch via call_facade(),
        # and set the real status / inputs. The placeholders below keep the
        # unedited template syntactically complete and every name defined.
        method_name, kwargs, reason = self._select_action()
        self.call_facade(method_name, **kwargs)
        status = Status.RUNNING
        inputs = {}

        self.emit_decision(
            inputs=inputs,
            status=status,
            reason=reason,
        )

        return status

    def terminate(self, new_status: Status):
        """Optional cleanup when this node stops ticking."""
        pass


# ─────────────────────────────────────────────────────────────────────────────
# GAIT VARIANT — use this skeleton instead when the file is named L2_Gait_*.py
# ─────────────────────────────────────────────────────────────────────────────
#
# class {{CLASS_NAME}}(XyzL2GaitActionNode):
#     """{{DESCRIPTION}}"""
#
#     LEVEL = 'L2'
#     BB_READS = {{BB_READS}}
#     BB_WRITES = []
#     FACADE_CALLS = ['go_step']          # or 'turn_step'
#     # Gait pass-through knobs are inherited from
#     # XyzL2GaitActionNode.GAIT_CONFIG_DEFAULTS — do NOT repeat them here.
#     # Declare strategy params only.
#     CONFIG_DEFAULTS = {
#         'x_speed': 0.010,
#     }
#
#     def __init__(self, name: str = '{{CLASS_NAME}}',
#                  facade: XyzBTFacade = None,
#                  logger=None,
#                  tick_id_getter=None,
#                  x_speed: float = 0.010,
#                  period_time_ms: int = None,
#                  dsp_ratio: float = None,
#                  y_swap_amplitude: float = None,
#                  arm_swap: int = None,
#                  step_num: int = None,
#                  gait_param: dict = None):
#         """
#         CONFIG_DEFAULTS:
#             x_speed: forward step magnitude (m).
#
#         Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude /
#         arm_swap / step_num / gait_param): see XyzL2GaitActionNode. Tune
#         WalkingParam keys through the dict, e.g.
#         gait_param={'step_height': 0.03, 'init_yaw_offset': 2}.
#         """
#         super().__init__(
#             name,
#             facade=facade,
#             logger=logger,
#             tick_id_getter=tick_id_getter,
#             period_time_ms=period_time_ms,
#             dsp_ratio=dsp_ratio,
#             y_swap_amplitude=y_swap_amplitude,
#             arm_swap=arm_swap,
#             step_num=step_num,
#             gait_param=gait_param,
#         )
#         self._x_speed = x_speed
#
#     def update(self) -> Status:
#         self.call_facade('go_step', x=self._x_speed, y=0, yaw=0,
#                          semantic_source='{{NODE_NAME}}',
#                          **self.gait_kwargs())
#         self.emit_decision(inputs={'x': self._x_speed},
#                            status=Status.SUCCESS,
#                            reason='step dispatched')
#         return Status.SUCCESS
