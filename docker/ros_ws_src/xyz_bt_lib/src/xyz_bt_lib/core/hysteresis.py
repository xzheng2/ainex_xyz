#!/usr/bin/env python3
"""HysteresisDecorator — asymmetric (N-in / M-out) debounce gate for xyz_bt_lib.

Problem it solves
-----------------
:class:`~xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator` is symmetric in the
wrong direction for flicker: it needs N passing ticks to engage but unlatches on
the FIRST failing tick (N-in / 1-out). That is correct for a safety confirmation
("only proceed once the world has been stable"), but wrong for a signal that
blinks — a detector that drops one frame out of twenty would tear down a branch
that is, in reality, still valid.

This decorator gives the two edges independent thresholds: ``enter_ticks``
passing ticks to engage, ``exit_ticks`` failing ticks to disengage. Setting
``enter_ticks=1, exit_ticks=N`` reproduces exactly the falling-edge debounce of
the removed node-level ``fail_dwell_ticks`` parameter (1-tick enter, N-tick
exit, anti-flicker on target loss) — the capability gap that class's docstring
records as deliberately deferred until a real tree needed it.

Like LatchedDwellDecorator, the state lives in the Blackboard, so it survives the
per-tick invalidation a ``memory=False`` (reactive) ancestor inflicts on RUNNING
descendants — see the measured proof in :mod:`xyz_bt_lib.core.composites` and
``examples/demo_memory_interaction.py``. Wrapping a condition here also keeps L1
nodes pure per the L1 contract (instantaneous predicate, no cross-tick state):
the debounce responsibility lives in this decorator, not in the condition.

Which one to use
----------------
- **LatchedDwellDecorator** — "do not act until the world has been stable for N
  ticks, and abandon immediately the moment it is not." Safety gates, precision
  alignment before a committed motion.
- **HysteresisDecorator** — "this signal flickers; hold the answer steady."
  Perception whose ground truth is continuous but whose measurement is noisy
  (intermittent detector dropouts, a target crossing a detection boundary).

They share the ``/node_state/`` namespace, so a ``state_key`` must be unique
across BOTH decorators, not just within one of them.

What you may wrap (status set MUST be {SUCCESS, FAILURE})
--------------------------------------------------------
Identical to LatchedDwellDecorator: only an L1 condition node, or a composite
built purely of L1 conditions (no RUNNING child, so the composite never returns
RUNNING). Do NOT wrap an L2 action. Two ways wrapping an L2 breaks:
  - A continuous L2 (e.g. ``L2_Gait_ControlToObject``) returns RUNNING every
    tick. RUNNING is treated as not-passed (choice 4), so the gate can never
    engage — the branch is unreachable.
  - A one-shot L2 (e.g. ``L2_Motion_StopGait``) returns SUCCESS every tick, so the gate
    does engage — but the decorator ticks its child EVERY tick, engaged or not,
    so the action's side effect is re-dispatched on every single tick for as long
    as the branch is entered. L1 predicates are side-effect-free, so re-evaluating
    them every tick is safe; an L2 has side effects, so this floods the actuator.

Positive example — hold a flickering detection steady:

    HysteresisDecorator(
        child=L1_Vision_IsObjectDetected(target_id='ball'),
        enter_ticks=2,      # 2 consecutive frames to believe it appeared
        exit_ticks=10,      # 10 consecutive misses to believe it is gone
        state_key='shoot/ball_visible',
        tick_id_getter=tick_id_getter)

Five deliberate design choices
------------------------------
1. Timing is by ``tick_id``, never wall clock.
   Identical rationale to LatchedDwellDecorator: ``run_action`` is a BLOCKING
   call, and while it runs the whole tree stalls for seconds. A wall clock would
   read the post-block resume as "reactivated after a long gap" and wipe a
   legitimately in-progress debounce run. ``tick_id`` does not advance while the
   tree is blocked, so the gap stays 1 and the run correctly continues. (Also: a
   debounce counts *samples*, and samples are ticks, not seconds.)

2. One Blackboard key per node, value is a dict.
   Key ``/node_state/<state_key>`` → ``{'engaged': bool, 'enter_count': int,
   'exit_count': int, 'last_tick_id': int}``. Two counters, not one shared
   counter: a single signed counter would make "3 passes then 2 fails" ambiguous
   with "1 pass", and each edge must be able to cancel the other's run outright.
   NOT the ``/latched/`` namespace — that is reserved for the adapters' two-phase
   sensor snapshot, and putting node-internal state there would break the "one
   consistent world snapshot per tick" invariant.
   ``last_tick_id`` is mandatory: without it a stale re-entry in a different tree
   phase would resume a half-finished debounce run against unrelated history.

3. ``state_key`` is required at construction, no default, never derived from
   ``node.name``. py_trees does not guarantee unique node names; same-named
   nodes would silently share state. Requiring it surfaces key collisions at
   construction time.

4. A RUNNING child does NOT count as passed.
   ``passed = (child_status == SUCCESS)``; RUNNING is treated exactly like
   FAILURE. Consistent with LatchedDwellDecorator: "uncertain" must never be
   counted as evidence that the condition holds.

5. This decorator NEVER returns RUNNING — and that is the deliberate difference
   from LatchedDwellDecorator, which returns RUNNING while counting up.
   A hysteresis gate is a *filtered view of a boolean*, so its output must be as
   binary as its input: SUCCESS while engaged, FAILURE while not. Emitting
   RUNNING during the enter run would make a ``ReactiveSequence`` guard hold the
   whole branch RUNNING (blocking its siblings) purely because a debounce was
   mid-count. It also means the output of this decorator is itself a legal
   {SUCCESS, FAILURE} signal, so it may be nested inside another pure-L1
   composite (or, if a tree genuinely needs it, wrapped by a LatchedDwell).
   Consequence: before the first successful enter run the gate reports FAILURE
   ("not yet confirmed" = "not satisfied", the safe-fail direction).

No rospy. Blackboard access is intentional (this is a stateful gate, unlike a
pure instance-state debounce).
"""
import py_trees
from py_trees.common import Access, Status


class HysteresisDecorator(py_trees.decorators.Decorator):
    """Debounce the wrapped condition with independent enter/exit thresholds.

    Engages (SUCCESS) after ``enter_ticks`` consecutive passing ticks; disengages
    (FAILURE) after ``exit_ticks`` consecutive failing ticks. A single tick of the
    opposite result cancels the run in progress — that cancellation IS the
    anti-flicker property. State persists in the Blackboard at
    ``/node_state/<state_key>`` so it survives reactive-ancestor invalidation.
    See the module docstring for the five design choices (tick_id timing,
    per-node dict key, required state_key, RUNNING-is-not-passed, never-RUNNING
    output).

    Args:
        child:               the wrapped condition Behaviour (its SUCCESS = passed).
        enter_ticks:         consecutive passing ticks required to engage.
                             1 = engage immediately (falling-edge-only debounce).
        exit_ticks:          consecutive failing ticks required to disengage.
                             1 = drop immediately (rising-edge-only debounce).
        state_key:           REQUIRED unique key; state at /node_state/<state_key>.
                             Must be unique across LatchedDwell nodes too — they
                             share this namespace.
        gap_threshold_ticks: a tick_id gap greater than this since the node was
                             last evaluated resets the gate to disengaged (stale
                             re-entry). Default 1 (consecutive ticks have gap 1
                             = continuous).
        name:                optional node name (default derived for display only).
        tick_id_getter:      callable → current tick_id (int). Inject the tree's
                             real getter; the gap protection is inert without it.
    """

    NODE_STATE_NS = '/node_state'

    # Blackboard resource declarations, mirroring the L1/L2 node convention so a
    # future static resource-conflict analyser can see this decorator's BB usage
    # (it writes BB, unlike a pure instance-state debounce). The path depends on
    # the per-instance state_key, so the concrete values are filled in __init__ —
    # these class-level defaults document the contract. It reads then writes its
    # OWN /node_state/<state_key> dict; two instances sharing a state_key is the
    # collision the required unique key exists to prevent.
    BB_READS = []
    BB_WRITES = []

    def __init__(self, child, enter_ticks, exit_ticks, state_key,
                 gap_threshold_ticks=1, name=None, tick_id_getter=None):
        if not state_key or not isinstance(state_key, str):
            raise ValueError(
                'HysteresisDecorator requires a non-empty string state_key '
                '(no default, not derived from node.name) so Blackboard key '
                'collisions surface at construction time.')
        if enter_ticks < 1 or exit_ticks < 1:
            raise ValueError(
                'HysteresisDecorator requires enter_ticks >= 1 and '
                'exit_ticks >= 1 (got {}/{}); a threshold of 0 would mean '
                '"engage/disengage on no evidence at all".'.format(
                    enter_ticks, exit_ticks))
        super().__init__(
            child,
            name=name or 'Hysteresis[{},{}in/{}out]'.format(
                state_key, enter_ticks, exit_ticks))
        self._enter_ticks         = enter_ticks
        self._exit_ticks          = exit_ticks
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

    # ── Lifecycle: deliberately does NOT touch debounce state ──────────────
    # The counters/engaged flag live in the Blackboard precisely so they survive
    # py_trees lifecycle churn. initialise() / terminate() cannot distinguish
    # structural invalidation (reactive re-entry) from logical completion, so
    # resetting here would reintroduce the exact reactive-ancestor bug this class
    # exists to avoid.

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
            st = {'engaged': False, 'enter_count': 0, 'exit_count': 0}
        st['last_tick_id'] = tick_id

        passed = (self.decorated.status == Status.SUCCESS)

        if st['engaged']:
            if passed:
                st['exit_count'] = 0                  # a pass cancels the exit run
            else:
                st['exit_count'] += 1
                if st['exit_count'] >= self._exit_ticks:
                    st['engaged']     = False
                    st['enter_count'] = 0
        else:
            if passed:
                st['enter_count'] += 1
                if st['enter_count'] >= self._enter_ticks:
                    st['engaged']    = True
                    st['exit_count'] = 0
            else:
                st['enter_count'] = 0                 # a fail cancels the enter run

        self._write_state(st)
        return Status.SUCCESS if st['engaged'] else Status.FAILURE
