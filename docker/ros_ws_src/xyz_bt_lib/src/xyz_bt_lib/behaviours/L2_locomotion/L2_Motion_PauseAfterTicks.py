#!/usr/bin/env python3
"""L2_Gait_PauseAfterTicks — stop gait after N consecutive active ticks; always returns RUNNING.

L2 action/strategy node.

Node kind: continuous_controller

BB reads:
  none

BB writes:
  none

Facade calls:
  stop_gait()

Action strategy:
  Sits at the bottom of a Selector as a fallback sentinel.  The node counts
  consecutive active ticks; once the count reaches trigger_ticks it dispatches
  stop_gait() exactly once via the facade, then continues returning RUNNING
  indefinitely.

  Delay semantics: when trigger_ticks > 1 the first (N-1) ticks do NOT call
  stop_gait(), giving upstream conditions time to re-gate before stopping the
  gait.  This avoids unnecessary stop/restart cycles in selectors that switch
  branches rapidly (e.g. momentary object loss in vision-based control).

  stop_gait() is dispatched at most once per activation.  After dispatch the
  node continues to return RUNNING so the parent Selector never succeeds
  through this sentinel.

Strategy helper:
  (none — logic is trivial: counter, threshold comparison, one-shot dispatch)

CONFIG_DEFAULTS:
  trigger_ticks: 1  — number of consecutive active ticks before dispatching
                       stop_gait().  1 = dispatch on first active tick.

Returns:
  RUNNING: always (continuous_controller — never self-terminates)

Observability:
  May emit 'action_intent' and 'decision' via base-node helpers.
  Never emits ros_out/ros_result or any comm event; those belong to _RuntimeIO.
"""
from py_trees.common import Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade


class L2_Gait_PauseAfterTicks(XyzL2ActionNode):
    """Stop gait after N consecutive active ticks; always returns RUNNING."""

    LEVEL        = 'L2'
    BB_READS     = []
    BB_WRITES    = []
    FACADE_CALLS = ['stop_gait']
    CONFIG_DEFAULTS = {
        'trigger_ticks': 1,  # consecutive ticks before dispatching stop_gait()
    }
    BB_LOG_KEYS = BB_READS

    # Param order: name, facade, then domain params, then logger, tick_id_getter.
    def __init__(self, name: str = 'L2_Gait_PauseAfterTicks',
                 facade: XyzBTFacade = None,
                 trigger_ticks: int = 1,
                 logger=None,
                 tick_id_getter=None):
        """
        Args:
            name: BT node name.
            facade: Project runtime facade implementing XyzBTFacade.
            trigger_ticks: Consecutive active ticks before stop_gait() is dispatched.
                           1 = dispatch on first tick.  Tree may override.
            logger: DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
        """
        super().__init__(
            name,
            facade=facade,
            logger=logger,
            tick_id_getter=tick_id_getter,
        )
        self._trigger_ticks = trigger_ticks
        self._active_ticks = 0
        self._stop_sent = False

    def initialise(self):
        """Reset counters on (re-)activation."""
        self._active_ticks = 0
        self._stop_sent = False
        self.emit_action_intent(
            action='pause_after_ticks',
            inputs={'trigger_ticks': self._trigger_ticks},
        )

    def update(self) -> Status:
        """Count tick; dispatch stop_gait() once when threshold reached.

        Always returns RUNNING — this is a continuous_controller sentinel.
        """
        self._active_ticks += 1

        if self._active_ticks == self._trigger_ticks and not self._stop_sent:
            self.call_facade('stop_gait')
            self._stop_sent = True
            reason = (
                f'threshold reached (tick {self._active_ticks}/'
                f'{self._trigger_ticks}), gait stop dispatched'
            )
        elif self._stop_sent:
            reason = 'idle — gait already stopped'
        else:
            reason = (
                f'waiting for threshold (tick {self._active_ticks}/'
                f'{self._trigger_ticks})'
            )

        self.emit_decision(
            inputs={
                'active_ticks': self._active_ticks,
                'trigger_ticks': self._trigger_ticks,
                'stop_sent': self._stop_sent,
            },
            status=Status.RUNNING,
            reason=reason,
        )
        return Status.RUNNING

    def terminate(self, new_status: Status):
        """Clean up counters when this node stops ticking."""
        self._active_ticks = 0
        self._stop_sent = False
