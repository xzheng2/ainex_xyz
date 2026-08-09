#!/usr/bin/env python3
"""Empirical validation of LatchedDwellDecorator (offline, no ROS/hardware).

Ticks the decorator through seven scenarios, printing per-tick state and
asserting the outcome. This is the executable proof that the reactive-safe
dwell behaves as specified — run it, don't trust prose.

    python3 examples/demo_latched_dwell.py

The dwell wraps a *condition*, which needs no facade, so no StubFacade is
required here (it would be, only if the subtree contained L2 action nodes).
"""
import py_trees
from py_trees.common import Status

from xyz_bt_lib.core.composites import ReactiveSequence
from xyz_bt_lib.core.latched_dwell import LatchedDwellDecorator

BBSTORE = py_trees.blackboard.Blackboard.storage


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


# 1. baseline: condition holds → SUCCESS exactly at required_ticks
def s1_baseline():
    hdr('1. baseline (required=3, condition always SUCCESS)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = LatchedDwellDecorator(cond, required_ticks=3, state_key='s1',
                              tick_id_getter=lambda: clock['t'])
    statuses = []
    for _ in range(5):
        clock['t'] += 1
        d.tick_once()
        st = peek('s1')
        statuses.append(d.status)
        print('  tick_id={t} stable={s} latched={l} -> {r}'.format(
            t=clock['t'], s=st.get('stable_ticks'), l=st.get('latched'),
            r=d.status.name))
    assert statuses[0] == Status.RUNNING and statuses[1] == Status.RUNNING
    assert statuses[2] == Status.SUCCESS, 'should latch at required_ticks (3)'
    assert statuses[3] == Status.SUCCESS, 'latch holds'
    print('  PASS: SUCCESS first at tick 3, held after.')


# 2. E1: mid FAILURE zeroes + unlatches, recovery recounts from 1
def s2_failure_reset():
    hdr('2. E1 mid-failure reset (required=3)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = LatchedDwellDecorator(cond, required_ticks=3, state_key='s2',
                              tick_id_getter=lambda: clock['t'])
    seq = [Status.SUCCESS, Status.SUCCESS, Status.FAILURE,
           Status.SUCCESS, Status.SUCCESS, Status.SUCCESS]
    stables = []
    for cs in seq:
        clock['t'] += 1
        cond.set(cs)
        d.tick_once()
        st = peek('s2')
        stables.append(st.get('stable_ticks'))
        print('  tick_id={t} child={c} stable={s} latched={l} -> {r}'.format(
            t=clock['t'], c=cs.name, s=st.get('stable_ticks'),
            l=st.get('latched'), r=d.status.name))
    assert stables[2] == 0, 'FAILURE must zero stable_ticks'
    assert stables[3] == 1, 'recovery recounts from 1'
    assert d.status == Status.SUCCESS, 'reaches SUCCESS again at end'
    print('  PASS: failure zeroed, recovery recounted from 1.')


# 3. E2 regression: under a ReactiveSequence it STILL accumulates + succeeds
def s3_under_reactive():
    hdr('3. E2 regression under ReactiveSequence (required=3)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = LatchedDwellDecorator(cond, required_ticks=3, state_key='s3',
                              tick_id_getter=lambda: clock['t'])
    root = ReactiveSequence('R', children=[d])
    stables = []
    for _ in range(3):
        clock['t'] += 1
        root.tick_once()
        st = peek('s3')
        stables.append(st.get('stable_ticks'))
        print('  tick_id={t} stable={s} latched={l} root={r}'.format(
            t=clock['t'], s=st.get('stable_ticks'), l=st.get('latched'),
            r=root.status.name))
    assert stables == [1, 2, 3], 'dwell must accumulate despite reactive re-entry'
    assert root.status == Status.SUCCESS
    print('  PASS: survived reactive invalidation, latched at 3.')


# 4. E3: after latched, a tick_id JUMP (gap>threshold) resets, not SUCCESS
def s4_stale_reentry():
    hdr('4. E3 stale re-entry after latch (required=2, gap_threshold=1)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = LatchedDwellDecorator(cond, required_ticks=2, state_key='s4',
                              gap_threshold_ticks=1,
                              tick_id_getter=lambda: clock['t'])
    for _ in range(2):
        clock['t'] += 1
        d.tick_once()
    assert peek('s4')['latched'] is True and d.status == Status.SUCCESS
    print('  after ticks 1,2: latched=True -> SUCCESS')
    clock['t'] = 10          # jump: gap 8 > threshold 1
    d.tick_once()
    st = peek('s4')
    print('  tick_id=10 (jumped) stable={s} latched={l} -> {r}'.format(
        s=st['stable_ticks'], l=st['latched'], r=d.status.name))
    assert st['latched'] is False and st['stable_ticks'] == 1
    assert d.status == Status.RUNNING, 'stale re-entry must recount, not SUCCESS'
    print('  PASS: stale re-entry cleared the latch and recounted.')


# 5. blocking: tick_id gap stays 1 across a stall → dwell continues (the point)
def s5_blocking_continuity():
    hdr('5. blocking continuity (tick_id gap==1 across a stall)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = LatchedDwellDecorator(cond, required_ticks=3, state_key='s5',
                              tick_id_getter=lambda: clock['t'])
    clock['t'] = 1; d.tick_once(); assert peek('s5')['stable_ticks'] == 1
    clock['t'] = 2; d.tick_once(); assert peek('s5')['stable_ticks'] == 2
    print('  (simulate run_action blocking: tree stalls, tick_id does NOT advance)')
    clock['t'] = 3           # resume: gap == 1, NOT a jump
    d.tick_once()
    st = peek('s5')
    print('  resume tick_id=3 gap=1 stable={s} -> {r}'.format(
        s=st['stable_ticks'], r=d.status.name))
    assert st['stable_ticks'] == 3 and d.status == Status.SUCCESS
    print('  PASS: gap==1 kept the dwell alive across the block (core motive).')


# 6. RUNNING child is NOT passed → zero + FAILURE
def s6_running_not_passed():
    hdr('6. RUNNING child treated as not-passed (required=3)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.RUNNING)
    d = LatchedDwellDecorator(cond, required_ticks=3, state_key='s6',
                              tick_id_getter=lambda: clock['t'])
    clock['t'] = 1
    d.tick_once()
    st = peek('s6')
    print('  tick_id=1 child=RUNNING stable={s} -> {r}'.format(
        s=st['stable_ticks'], r=d.status.name))
    assert st['stable_ticks'] == 0 and d.status == Status.FAILURE
    print('  PASS: RUNNING zeroed dwell and returned FAILURE (safe-fail).')


# 7. key isolation: two state_keys do not interfere
def s7_key_isolation():
    hdr('7. key isolation (two distinct state_keys)')
    fresh()
    clock = {'t': 0}
    ca, cb = Cond(name='a'), Cond(name='b')
    da = LatchedDwellDecorator(ca, required_ticks=3, state_key='iso_a',
                               tick_id_getter=lambda: clock['t'])
    db = LatchedDwellDecorator(cb, required_ticks=3, state_key='iso_b',
                               tick_id_getter=lambda: clock['t'])
    for cs_a in (Status.SUCCESS, Status.SUCCESS, Status.FAILURE):
        clock['t'] += 1
        ca.set(cs_a); cb.set(Status.SUCCESS)
        da.tick_once(); db.tick_once()
        print('  tick_id={t} A(stable={sa},{ra}) B(stable={sb},{rb})'.format(
            t=clock['t'], sa=peek('iso_a')['stable_ticks'], ra=da.status.name,
            sb=peek('iso_b')['stable_ticks'], rb=db.status.name))
    assert peek('iso_a')['stable_ticks'] == 0, 'A failed → A zeroed'
    assert peek('iso_b')['stable_ticks'] == 3 and db.status == Status.SUCCESS
    print('  PASS: A failing did not disturb B.')


# 8. latched, THEN the condition breaks → unlatch + FAILURE (ball rolled away)
def s8_latched_then_fail():
    hdr('8. latch then condition breaks (required=2)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = LatchedDwellDecorator(cond, required_ticks=2, state_key='s8',
                              tick_id_getter=lambda: clock['t'])
    clock['t'] = 1; d.tick_once()
    clock['t'] = 2; d.tick_once()
    assert peek('s8')['latched'] is True and d.status == Status.SUCCESS
    print('  ticks 1,2: latched -> SUCCESS')
    clock['t'] = 3; cond.set(Status.FAILURE); d.tick_once()
    st = peek('s8')
    print('  tick_id=3 child=FAILURE stable={s} latched={l} -> {r}'.format(
        s=st['stable_ticks'], l=st['latched'], r=d.status.name))
    assert st['latched'] is False and st['stable_ticks'] == 0
    assert d.status == Status.FAILURE, 'broken condition after latch must unlock + FAILURE'
    clock['t'] = 4; cond.set(Status.SUCCESS); d.tick_once()
    assert peek('s8')['stable_ticks'] == 1, 'recovery recounts from 1'
    print('  tick_id=4 child=SUCCESS recount stable=1 -> {}'.format(d.status.name))
    print('  PASS: latch broke on FAILURE, then recounted from 1.')


# 9. gap_threshold_ticks is a real parameter (not hardcoded 1)
def s9_gap_threshold_configurable():
    hdr('9. gap_threshold_ticks configurable (=3, not hardcoded 1)')
    fresh()
    clock = {'t': 0}
    cond = Cond(status=Status.SUCCESS)
    d = LatchedDwellDecorator(cond, required_ticks=5, state_key='s9',
                              gap_threshold_ticks=3,
                              tick_id_getter=lambda: clock['t'])
    clock['t'] = 1; d.tick_once()
    assert peek('s9')['stable_ticks'] == 1
    clock['t'] = 4; d.tick_once()            # gap 3 == threshold → NOT > → continue
    s_gap3 = peek('s9')['stable_ticks']
    print('  tick_id 1->4 (gap 3 == threshold) stable={}'.format(s_gap3))
    assert s_gap3 == 2, 'gap == threshold must NOT reset'
    clock['t'] = 8; d.tick_once()            # gap 4 > threshold → reset
    s_gap4 = peek('s9')['stable_ticks']
    print('  tick_id 4->8 (gap 4 > threshold) stable={}'.format(s_gap4))
    assert s_gap4 == 1, 'gap > threshold must reset'
    print('  PASS: threshold=3 honored (gap 3 continued, gap 4 reset) -> configurable.')


def main():
    for fn in (s1_baseline, s2_failure_reset, s3_under_reactive,
               s4_stale_reentry, s5_blocking_continuity,
               s6_running_not_passed, s7_key_isolation,
               s8_latched_then_fail, s9_gap_threshold_configurable):
        fn()
    print('\nALL 9 SCENARIOS PASSED.')


if __name__ == '__main__':
    main()
