#!/usr/bin/env python3
"""Empirical proof of the ancestor-descendant memory interaction rule.

Runs two minimal trees tick-by-tick and prints an INSTANCE-STATE counter
decorator's count, so the rule stated in ``xyz_bt_lib.core.composites`` is
*measured*, not asserted. No ROS, no hardware — pure py_trees.

    python3 examples/demo_memory_interaction.py

The counter decorator below (_InstanceCounterDwell) keeps its count in an
instance attribute and resets it in initialise() — the classic dwell pattern
that this proof shows is defeated by a reactive ancestor. (This is exactly why
the library ships LatchedDwellDecorator, which stores its counter in the
Blackboard instead; see examples/demo_latched_dwell.py scenario 3.)

Expected (py_trees 2.1.6, n_ticks=3, child always SUCCESS):

    tick                                          1   2   3   4   5
    CommittedSequence[ Dwell ]                    1   2   3*  1   2     accumulates, SUCCEEDS at 3
    ReactiveSequence[ CommittedSequence[Dwell] ]  1   1   1   1   1     pinned at 1, NEVER succeeds

The reactive (memory=False) ancestor invalidates its RUNNING committed
descendant every tick, so the decorator's initialise() re-runs and the counter
can never climb past 1.
"""
import py_trees
import py_trees.behaviours as bh
from py_trees.common import Status

from xyz_bt_lib.core.composites import ReactiveSequence, CommittedSequence

N_TICKS = 3
TICKS = 5


class _InstanceCounterDwell(py_trees.decorators.Decorator):
    """Minimal instance-state dwell: SUCCESS after n_ticks consecutive child SUCCESS.

    Deliberately keeps its counter in an instance attribute and resets it in
    initialise() — the fragile pattern this demo exposes. Not for production use;
    real trees use xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator.
    """
    def __init__(self, child, n_ticks, name='Dwell'):
        super().__init__(child, name=name)
        self._n_ticks = n_ticks
        self._count = 0

    def initialise(self):
        self._count = 0

    def update(self):
        if self.decorated.status == Status.SUCCESS:
            self._count += 1
        else:
            self._count = 0
        return Status.SUCCESS if self._count >= self._n_ticks else Status.RUNNING


def _build_control():
    dwell = _InstanceCounterDwell(bh.Success(name='OK'), n_ticks=N_TICKS)
    return dwell, CommittedSequence('C', children=[dwell])


def _build_reactive():
    dwell = _InstanceCounterDwell(bh.Success(name='OK'), n_ticks=N_TICKS)
    committed = CommittedSequence('C', children=[dwell])
    return dwell, ReactiveSequence('R', children=[committed])


def _run(dwell, root):
    counts, statuses = [], []
    for _ in range(TICKS):
        root.tick_once()
        counts.append(dwell._count)
        statuses.append('S' if root.status == py_trees.common.Status.SUCCESS else '.')
    return counts, statuses


def main():
    ctrl_c, ctrl_s = _run(*_build_control())
    reac_c, reac_s = _run(*_build_reactive())

    hdr = '  '.join('{:>2}'.format(t) for t in range(1, TICKS + 1))
    print('tick                                          ' + hdr)

    def row(label, counts, statuses):
        cells = '  '.join('{:>2}'.format(c) for c in counts)
        print('{:<44}{}'.format(label, cells))
        print('{:<44}{}'.format('  (root SUCCESS?)',
                                 '  '.join('{:>2}'.format(s) for s in statuses)))

    row('CommittedSequence[ Dwell ]', ctrl_c, ctrl_s)
    row('ReactiveSequence[ CommittedSequence[Dwell] ]', reac_c, reac_s)


if __name__ == '__main__':
    main()
