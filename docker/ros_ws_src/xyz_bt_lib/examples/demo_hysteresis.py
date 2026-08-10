#!/usr/bin/env python3
"""Empirical validation of HysteresisDecorator (offline, no ROS/hardware).

Ticks the decorator through nine scenarios, printing per-tick state and
asserting the outcome. This is the executable proof that asymmetric
(N-in / M-out) debounce behaves as specified — run it, don't trust prose.

    python3 examples/demo_hysteresis.py

Scenario 2 is the one that justifies the class existing at all: a flickering
signal that LatchedDwellDecorator would tear down on the first dropped frame is
held steady here. Scenario 3 proves enter_ticks=1 reproduces the removed
node-level ``fail_dwell_ticks``. Scenario 8 pins the never-RUNNING contract.

The gate wraps a *condition*, which needs no facade, so no StubFacade is
required here (it would be, only if the subtree contained L2 action nodes).
"""
import py_trees
from py_trees.common import Status

from xyz_bt_lib.core.composites import ReactiveSequence
from xyz_bt_lib.core.hysteresis import HysteresisDecorator

BBSTORE = py_trees.blackboard.Blackboard.storage

S, F, R = Status.SUCCESS, Status.FAILURE, Status.RUNNING


class Cond(py_trees.behaviour.Behaviour):
    """Controllable condition leaf: .set(status) drives what it returns."""
    def __init__(self, name='cond', status=Status.SUCCESS):
        super().__init__(name=name)
        self._status = status

    def set(self, status):
        self._status = status

    def update(self):
        return self._status


def peek(state_key):
    return BBSTORE.get('/node_state/{}'.format(state_key), {})


def fresh():
    BBSTORE.clear()


def hdr(title):
    print('\n===== {} ====='.format(title))


def run(d, cond, clock, seq, state_key, label='child'):
    """Tick d once per entry in seq, printing state. Returns list of statuses."""
    out = []
    for cs in seq:
        clock['t'] += 1
        cond.set(cs)
        d.tick_once()
        st = peek(state_key)
        out.append(d.status)
        print('  tick_id={t} {lbl}={c:7s} enter={i} exit={o} engaged={e:5s} -> {r}'.format(
            t=clock['t'], lbl=label, c=cs.name, i=st.get('enter_count'),
            o=st.get('exit_count'), e=str(st.get('engaged')), r=d.status.name))
    return out


# 1. baseline asymmetry: 2 passes to engage, 3 fails to drop
def s1_baseline_asymmetry():
    hdr('1. baseline asymmetry (enter=2, exit=3)')
    fresh()
    clock = {'t': 0}
    cond = Cond()
    d = HysteresisDecorator(cond, enter_ticks=2, exit_ticks=3,
                            state_key='h1', tick_id_getter=lambda: clock['t'])
    out = run(d, cond, clock, [S, S, F, F, F], 'h1')
    assert out == [F, S, S, S, F], out
    print('  PASS: engaged on the 2nd pass, held through 2 fails, dropped on the 3rd.')


# 2. THE POINT: a flickering signal never disengages (LatchedDwell would)
def s2_flicker_rejection():
    hdr('2. flicker rejection (enter=1, exit=3, signal blinks)')
    fresh()
    clock = {'t': 0}
    cond = Cond()
    d = HysteresisDecorator(cond, enter_ticks=1, exit_ticks=3,
                            state_key='h2', tick_id_getter=lambda: clock['t'])
    out = run(d, cond, clock, [S, F, S, F, F, S], 'h2')
    assert all(s == Status.SUCCESS for s in out), out
    assert peek('h2')['engaged'] is True
    print('  PASS: stayed SUCCESS through every dropout — a single pass cancels')
    print('        the exit run. LatchedDwellDecorator would have dropped at tick 2.')


# 3. enter_ticks=1 reproduces the removed node-level fail_dwell_ticks
def s3_old_fail_dwell_equivalence():
    hdr('3. old fail_dwell_ticks equivalence (enter=1, exit=3)')
    fresh()
    clock = {'t': 0}
    cond = Cond()
    d = HysteresisDecorator(cond, enter_ticks=1, exit_ticks=3,
                            state_key='h3', tick_id_getter=lambda: clock['t'])
    out = run(d, cond, clock, [S, F, F, F], 'h3')
    assert out == [S, S, S, F], out
    print('  PASS: 1-tick enter / 3-tick exit == the deleted fail_dwell_ticks=3.')


# 4. a single fail cancels an in-progress enter run
def s4_enter_run_cancelled():
    hdr('4. enter run cancelled by one fail (enter=3, exit=1)')
    fresh()
    clock = {'t': 0}
    cond = Cond()
    d = HysteresisDecorator(cond, enter_ticks=3, exit_ticks=1,
                            state_key='h4', tick_id_getter=lambda: clock['t'])
    run(d, cond, clock, [S, S], 'h4')
    assert peek('h4')['enter_count'] == 2
    run(d, cond, clock, [F], 'h4')
    assert peek('h4')['enter_count'] == 0, 'one fail must cancel the enter run'
    out = run(d, cond, clock, [S, S, S], 'h4')
    assert out[-1] == Status.SUCCESS and peek('h4')['engaged'] is True
    print('  PASS: enter run zeroed at the fail, recounted 1->3, then engaged.')


# 5. reactive-safe: accumulates under a memory=False ancestor
def s5_under_reactive():
    hdr('5. reactive-safe under ReactiveSequence (enter=3)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = HysteresisDecorator(cond, enter_ticks=3, exit_ticks=1,
                            state_key='h5', tick_id_getter=lambda: clock['t'])
    root = ReactiveSequence('R', children=[d])
    counts = []
    for _ in range(3):
        clock['t'] += 1
        root.tick_once()
        st = peek('h5')
        counts.append(st.get('enter_count'))
        print('  tick_id={t} enter={i} engaged={e:5s} root={r}'.format(
            t=clock['t'], i=st.get('enter_count'),
            e=str(st.get('engaged')), r=root.status.name))
    assert counts == [1, 2, 3], counts
    assert root.status == Status.SUCCESS
    print('  PASS: BB-backed counters survived reactive re-entry (instance state would not).')


# 6. RUNNING child is NOT passed
def s6_running_not_passed():
    hdr('6. RUNNING child treated as not-passed (enter=2)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.RUNNING)
    d = HysteresisDecorator(cond, enter_ticks=2, exit_ticks=2,
                            state_key='h6', tick_id_getter=lambda: clock['t'])
    out = run(d, cond, clock, [R, R], 'h6')
    assert out == [F, F], out
    assert peek('h6')['enter_count'] == 0
    print('  PASS: RUNNING never advanced the enter run (safe-fail).')


# 7. stale re-entry (tick_id gap) resets to disengaged
def s7_stale_reentry():
    hdr('7. stale re-entry resets (enter=3, gap_threshold=1)')
    fresh()
    clock = {'t': 0}
    cond = Cond()
    d = HysteresisDecorator(cond, enter_ticks=3, exit_ticks=5,
                            state_key='h7', gap_threshold_ticks=1,
                            tick_id_getter=lambda: clock['t'])
    run(d, cond, clock, [S, S, S], 'h7')
    assert peek('h7')['engaged'] is True and d.status == Status.SUCCESS
    print('  (engaged; now jump tick_id 3 -> 20, gap 17 > threshold 1)')
    clock['t'] = 19          # run() increments to 20
    out = run(d, cond, clock, [S], 'h7')
    assert peek('h7')['engaged'] is False
    assert peek('h7')['enter_count'] == 1, 'must recount from 1 after the gap'
    assert out[0] == Status.FAILURE
    print('  PASS: stale re-entry disengaged the gate and recounted from 1.')


# 8. contract: this decorator NEVER returns RUNNING
def s8_never_running():
    hdr('8. never returns RUNNING (contrast with LatchedDwellDecorator)')
    fresh()
    clock = {'t': 0}
    cond = Cond()
    d = HysteresisDecorator(cond, enter_ticks=5, exit_ticks=5,
                            state_key='h8', tick_id_getter=lambda: clock['t'])
    out = run(d, cond, clock, [S, S, R, F, S, S, S, S, S, F, R], 'h8')
    assert all(s in (Status.SUCCESS, Status.FAILURE) for s in out), out
    print('  PASS: every tick was SUCCESS or FAILURE — mid-count never leaked RUNNING')
    print('        (a LatchedDwell in the same run would emit RUNNING while counting).')


# 9. key isolation: two state_keys do not interfere
def s9_key_isolation():
    hdr('9. key isolation (two distinct state_keys)')
    fresh()
    clock = {'t': 0}
    ca, cb = Cond(name='a'), Cond(name='b')
    da = HysteresisDecorator(ca, enter_ticks=2, exit_ticks=1,
                             state_key='iso_ha', tick_id_getter=lambda: clock['t'])
    db = HysteresisDecorator(cb, enter_ticks=2, exit_ticks=1,
                             state_key='iso_hb', tick_id_getter=lambda: clock['t'])
    for cs_a in (S, S, F):
        clock['t'] += 1
        ca.set(cs_a); cb.set(S)
        da.tick_once(); db.tick_once()
        print('  tick_id={t} A(engaged={ea:5s},{ra}) B(engaged={eb:5s},{rb})'.format(
            t=clock['t'], ea=str(peek('iso_ha').get('engaged')), ra=da.status.name,
            eb=str(peek('iso_hb').get('engaged')), rb=db.status.name))
    assert peek('iso_ha')['engaged'] is False, 'A failed -> A disengaged'
    assert peek('iso_hb')['engaged'] is True and db.status == Status.SUCCESS
    print('  PASS: A dropping did not disturb B.')


def main():
    for fn in (s1_baseline_asymmetry, s2_flicker_rejection,
               s3_old_fail_dwell_equivalence, s4_enter_run_cancelled,
               s5_under_reactive, s6_running_not_passed,
               s7_stale_reentry, s8_never_running, s9_key_isolation):
        fn()
    print('\nALL 9 SCENARIOS PASSED.')


if __name__ == '__main__':
    main()
