#!/usr/bin/env python3
"""LatchedDwellDecorator — reactive-safe stability gate for xyz_bt_lib.

Problem it solves
-----------------
A dwell decorator that keeps its counter in instance state (as the removed
``DwellDecorator`` did) has that counter wiped by ``initialise()`` on re-entry,
so a ``memory=False`` (reactive) ancestor resets it every tick and it can never
climb — see the measured proof in :mod:`xyz_bt_lib.core.composites`
(ancestor–descendant memory rule) and ``examples/demo_memory_interaction.py``.

This decorator moves the dwell timer + latch into the Blackboard, so it
survives py_trees lifecycle churn while still re-checking the wrapped condition
every tick (reactive-safe). It is the reactive-safe stability gate for the
library, replacing instance-state dwell decorators. Wrapping a condition
in this decorator also keeps L1 condition nodes pure per the spec's L1 contract
(instantaneous predicate, no side effects, no cross-tick RUNNING state) — the
dwell/latch responsibility lives here, not in the condition.

What you may wrap (status set MUST be {SUCCESS, FAILURE})
--------------------------------------------------------
Only wrap something whose per-tick result is strictly SUCCESS or FAILURE — an
L1 condition node, or a composite built purely of L1 conditions (no RUNNING
child, so the composite never returns RUNNING either). Do NOT wrap an L2 action.
Two ways wrapping an L2 breaks:
  - A continuous L2 (e.g. ``L2_Gait_ControlToObject``) returns RUNNING every
    tick. RUNNING is treated as not-passed (choice 4), so ``stable_ticks`` is
    zeroed every tick and the branch can never latch — it is unreachable.
  - A one-shot L2 (e.g. ``L2_Gait_Stop``) returns SUCCESS every tick, so the
    count does climb — but the decorator ticks its child ONCE PER TICK, so the
    action's side effect is re-dispatched N times (a 20-tick dwell = 20×
    ``stop_gait``). L1 predicates are side-effect-free, so re-evaluating them
    every tick is safe; an L2 has side effects, so this floods the actuator.

Positive example — wrap a pure-L1 composite:

    LatchedDwellDecorator(
        child=ReactiveSequence('BallReady', children=[
            L1_Vision_IsObjectDetected(...),
            L1_Vision_IsObjectPositioned(...)]),
        required_ticks=10, state_key='shoot/ball_ready',
        tick_id_getter=tick_id_getter)

Four deliberate design choices
------------------------------
1. Timing is by ``tick_id``, never wall clock.
   Dwell progress and the continuity test both use an externally-injected
   ``tick_id`` (the existing ``tick_id_getter`` pattern). ``run_action`` is a
   BLOCKING call: while it runs the whole tree stalls for seconds. A wall clock
   would read the post-block resume as "reactivated after a long gap" and wipe a
   legitimately in-progress dwell. ``tick_id`` does not advance while the tree is
   blocked, so the gap stays 1 and the dwell is correctly treated as continuing.
   (Also: a dwell confirms *observation confidence*, which comes from the number
   of samples, not from elapsed time.)

2. One Blackboard key per node, value is a dict.
   Key ``/node_state/<state_key>`` → ``{'stable_ticks': int, 'latched': bool,
   'last_tick_id': int}``. Not 2N/3N separate keys (declaration bloat); not one
   global key holding a nested dict (every dwell node would write the same key,
   making static conflict analysis all false-positives). NOT the ``/latched/``
   namespace — that is reserved for the adapters' two-phase sensor snapshot, and
   putting node-internal state there would break the "one consistent world
   snapshot per tick" invariant.
   ``last_tick_id`` is mandatory: with only stable/latched, ``latched=True``
   would live forever, and re-entering the node in a different tree phase would
   return SUCCESS immediately with the dwell skipped entirely.

3. ``state_key`` is required at construction, no default, never derived from
   ``node.name``. py_trees does not guarantee unique node names; same-named
   nodes would silently share state. Requiring it surfaces key collisions at
   construction time.

4. A RUNNING child does NOT count as passed.
   ``passed = (child_status == SUCCESS)``; RUNNING is treated exactly like
   FAILURE. A dwell is a safety-confirmation mechanism, so "uncertain" must be
   handled as "not satisfied" — otherwise an unconfirmed state would be counted
   toward the stable duration and satisfy the gate early. FAILURE is the
   safe-fail direction.

No rospy. Blackboard access is intentional (this is a stateful gate, unlike a
pure instance-state dwell decorator).

Known gap — falling-edge hysteresis has NO decorator replacement
----------------------------------------------------------------
The removed node-level ``fail_dwell_ticks`` was asymmetric debounce: enter on
1 passing tick, exit only after N failing ticks (1-in / N-out). This class is
the opposite shape (N-in / 1-out): a single failing tick resets the latch, so
it CANNOT express falling-edge debounce. No current tree needs it; add a
separate ``HysteresisDecorator`` (independent enter/exit thresholds, BB-backed
like this one) only when a real tree does — do not emulate it with this class.
"""
import py_trees
from py_trees.common import Access, Status


class LatchedDwellDecorator(py_trees.decorators.Decorator):
    """Latch SUCCESS once the wrapped condition holds for ``required_ticks``.

    Dwell state persists in the Blackboard at ``/node_state/<state_key>`` so it
    survives reactive-ancestor invalidation. See the module docstring for the
    four design choices (tick_id timing, per-node dict key, required state_key,
    RUNNING-is-not-passed).

    Args:
        child:               the wrapped condition Behaviour (its SUCCESS = passed).
        required_ticks:      consecutive passing ticks required before latching.
        state_key:           REQUIRED unique key; state at /node_state/<state_key>.
        gap_threshold_ticks: a tick_id gap greater than this since the node was
                             last evaluated resets the dwell (stale re-entry).
                             Default 1 (consecutive ticks have gap 1 = continuous).
        name:                optional node name (default derived for display only).
        tick_id_getter:      callable → current tick_id (int). Inject the tree's
                             real getter; the gap protection is inert without it.
    """

    NODE_STATE_NS = '/node_state'

    # Blackboard resource declarations, mirroring the L1/L2 node convention so a
    # future static resource-conflict analyser can see this decorator's BB usage
    # (it writes BB, unlike a pure instance-state dwell decorator). The path depends on the
    # per-instance state_key, so the concrete values are filled in __init__ —
    # these class-level defaults document the contract. It reads then writes its
    # OWN /node_state/<state_key> dict; two instances sharing a state_key is the
    # collision the required unique key exists to prevent (see scenario 7).
    BB_READS = []
    BB_WRITES = []

    def __init__(self, child, required_ticks, state_key,
                 gap_threshold_ticks=1, name=None, tick_id_getter=None):
        if not state_key or not isinstance(state_key, str):
            raise ValueError(
                'LatchedDwellDecorator requires a non-empty string state_key '
                '(no default, not derived from node.name) so Blackboard key '
                'collisions surface at construction time.')
        super().__init__(
            child,
            name=name or 'LatchedDwell[{},{}t]'.format(state_key, required_ticks))
        self._required_ticks      = required_ticks
        self._state_key           = state_key
        self._gap_threshold_ticks = gap_threshold_ticks
        self._tick_id_getter      = tick_id_getter or (lambda: -1)
        self._state_path          = '{}/{}'.format(self.NODE_STATE_NS, state_key)
        self._bb                  = None
        # Per-instance concrete declarations (the resolved key this node touches).
        self.BB_READS  = [self._state_path]
        self.BB_WRITES = [self._state_path]

    # ── Blackboard plumbing ───────────────────────────────────────────────

    def _ensure_client(self):
        if self._bb is None:
            self._bb = self.attach_blackboard_client(name=self.name)
            # WRITE implicitly grants read in py_trees 2.1.6.
            self._bb.register_key(key=self._state_path, access=Access.WRITE)

    def setup(self, **kwargs):
        """Register the Blackboard key (idiomatic); update() also lazy-inits."""
        self._ensure_client()

    def _read_state(self) -> dict:
        self._ensure_client()
        try:
            val = self._bb.get(self._state_path)
        except KeyError:
            return {}
        # Defensive copy so in-place mutation never aliases the stored dict.
        return dict(val) if isinstance(val, dict) else {}

    def _write_state(self, st: dict) -> None:
        self._ensure_client()
        self._bb.set(self._state_path, st)

    # ── Lifecycle: deliberately does NOT touch dwell state ─────────────────
    # The timer/latch lives in the Blackboard precisely so it survives py_trees
    # lifecycle churn. initialise() / terminate() cannot distinguish structural
    # invalidation (reactive re-entry) from logical completion, so resetting here
    # would reintroduce the exact reactive-ancestor bug this class exists to fix.

    def initialise(self):
        pass

    def terminate(self, new_status):
        pass

    # ── Core state machine (continuity test BEFORE evaluating the child) ───

    def update(self) -> Status:
        tick_id = self._tick_id_getter()
        st = self._read_state()

        last = st.get('last_tick_id')
        if last is None or (tick_id - last) > self._gap_threshold_ticks:
            st = {'stable_ticks': 0, 'latched': False}
        st['last_tick_id'] = tick_id

        passed = (self.decorated.status == Status.SUCCESS)

        if not passed:
            st['stable_ticks'] = 0
            st['latched']      = False
            result = Status.FAILURE
        elif st['latched']:
            result = Status.SUCCESS                       # idempotent hold
        else:
            st['stable_ticks'] += 1
            if st['stable_ticks'] >= self._required_ticks:
                st['latched'] = True
                result = Status.SUCCESS
            else:
                result = Status.RUNNING

        self._write_state(st)
        return result
