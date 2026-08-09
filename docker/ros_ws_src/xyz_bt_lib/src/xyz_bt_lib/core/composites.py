#!/usr/bin/env python3
"""Semantic composite factories for xyz_bt_lib trees.

py_trees composites take a ``memory`` boolean whose name describes the
*mechanism* (whether the composite remembers the index of the child that was
RUNNING last tick) rather than the *contract* the tree author actually cares
about. That naming gap is a recurring source of bugs: ``memory`` gets written
one way in a hand-drawn docstring tree and the opposite way in code, and a
typo like ``memory=false`` (lower-case, not a Python literal) fails silently as
a NameError only if you're lucky.

These factories put the contract in the *name*, so the intent is explicit at
the call site and a misspelled factory name is an immediate ``NameError``:

    memory=False  →  reactive / guard semantics   (re-evaluate every tick)
    memory=True   →  committed semantics           (check once, then advance)

    ReactiveSequence   = Sequence(memory=False)
    CommittedSequence  = Sequence(memory=True)
    PrioritySelector   = Selector(memory=False)
    CommittedSelector  = Selector(memory=True)
    ParallelAll        = Parallel(policy=SuccessOnAll)
    ParallelAny        = Parallel(policy=SuccessOnOne)

Parallel uses a *policy*, not memory, so the two Parallel factories name the
success policy instead. (There is no reactive/committed axis for Parallel.)

Ancestor–descendant interaction rule (READ THIS — MEASURED, not asserted)
-------------------------------------------------------------------------
A ``memory=False`` parent (ReactiveSequence / PrioritySelector) invalidates
its RUNNING child every tick and re-enters it from scratch. When such a parent
sits *above* a ``memory=True`` descendant (CommittedSequence) or a dwell /
confirm decorator, the descendant's RUNNING position is discarded each tick and
its dwell / confirmation counter is reset, so it can never climb.

This is the measured behaviour (py_trees 2.1.6), ticking an instance-state dwell
decorator with ``n_ticks=3`` whose child always returns SUCCESS. Reproduce it
exactly with ``python3 examples/demo_memory_interaction.py``:

    tick                                          1   2   3   4   5
    CommittedSequence[ Dwell ]                    1   2   3   1   2
      root SUCCESS?                               .   .   S   .   .
    ReactiveSequence[ CommittedSequence[Dwell] ]  1   1   1   1   1
      root SUCCESS?                               .   .   .   .   .

The committed control accumulates to 3 and SUCCEEDS at tick 3, then re-arms.
Wrapped in a reactive ancestor the counter is pinned at 1 forever and the tree
never succeeds — the reactive ancestor silently defeats the committed
descendant's memory. Note the minimal case reproduces this: no failing guard is
needed; a reactive ancestor alone is sufficient.

Practical consequences:
  - A dwell ("hold SUCCESS for N ticks") or a committed action chain will
    never make progress if any ancestor between it and the root is reactive.
  - If you need a guard that can abort a committed sub-tree, put the guard
    *condition* as an earlier child of the SAME CommittedSequence (or use a
    reactive guard that wraps the whole committed sub-tree and returns
    FAILURE to tear it down deliberately) — do not rely on a reactive
    ancestor to "watch" a committed descendant tick-by-tick.

This is the single authoritative statement of that rule; prefer linking here
over re-explaining it per tree.

py_trees version note
---------------------
Written against py_trees 2.1.6, where the constructors are::

    Selector(name='Selector', memory=False, children=None)
    Sequence(name='Sequence', memory=True,  children=None)

Note the differing ``memory`` *defaults* (Selector False, Sequence True) —
another reason to prefer these explicit factories over the raw constructors.
All factories pass ``memory`` and ``children`` by keyword to be robust against
positional-order changes between py_trees point releases.

No ROS, no Blackboard access — pure tree wiring, same as ``decorators.py``.
"""
import py_trees


def ReactiveSequence(name, children=None):
    """Sequence that re-evaluates from the first child every tick (reactive guard).

    Semantics: on every tick, start again at the first child. Every preceding
    child (typically a condition / gate) must keep succeeding for execution to
    reach the later children; the moment one stops being true the sequence
    abandons the later work. Use for guarded sequences that must bail out as
    soon as a precondition no longer holds.

    Equivalent to ``py_trees.composites.Sequence(name, memory=False, children=children)``.

    Warning: as a reactive parent, this resets the RUNNING position and any
    dwell counter of a CommittedSequence / dwell decorator placed beneath it —
    see the module docstring's ancestor–descendant rule.
    """
    return py_trees.composites.Sequence(name=name, memory=False,
                                        children=children)


def CommittedSequence(name, children=None):
    """Sequence that resumes at the last RUNNING child (committed action chain).

    Semantics: remember which child was RUNNING; next tick continue from there
    without re-evaluating the earlier children. Use for an action chain that,
    once started, must run to completion — including blocking actions and
    dwell / confirmation steps that need to accumulate across ticks.

    Equivalent to ``py_trees.composites.Sequence(name, memory=True, children=children)``.

    Do not place under a reactive (memory=False) ancestor if the chain must
    make progress — see the module docstring's ancestor–descendant rule.
    """
    return py_trees.composites.Sequence(name=name, memory=True,
                                        children=children)


def PrioritySelector(name, children=None):
    """Selector that re-evaluates from the highest-priority child every tick.

    Semantics: on every tick, try children in order from the top; a
    higher-priority child may preempt whatever was RUNNING below it. Use for
    the standard priority structure where safety / recovery branches sit above
    task branches and must be able to interrupt them at any tick.

    Equivalent to ``py_trees.composites.Selector(name, memory=False, children=children)``.
    """
    return py_trees.composites.Selector(name=name, memory=False,
                                        children=children)


def CommittedSelector(name, children=None):
    """Selector that stays on the current RUNNING branch (no preemption).

    Semantics: remember the RUNNING child and keep ticking it without
    re-evaluating higher-priority children. Rare — use only when you
    explicitly do NOT want a higher-priority branch to preempt the branch in
    progress.

    Equivalent to ``py_trees.composites.Selector(name, memory=True, children=children)``.
    """
    return py_trees.composites.Selector(name=name, memory=True,
                                        children=children)


def ParallelAll(name, children=None, synchronise=True):
    """Parallel that succeeds only when ALL children succeed (SuccessOnAll).

    Semantics: tick every child each tick; SUCCESS when all children have
    succeeded, FAILURE as soon as one fails, RUNNING otherwise. Use for
    "do these concurrently, all must complete".

    ``synchronise`` (py_trees 2.1.6 default True) is passed through to the
    policy: when True, a child that has already returned SUCCESS is not
    re-ticked (its success is held) while the others catch up; when False,
    every child is re-ticked every tick. Left as the default here.

    Equivalent to ``py_trees.composites.Parallel(name,
    policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise), children=children)``.
    """
    return py_trees.composites.Parallel(
        name=name,
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=synchronise),
        children=children)


def ParallelAny(name, children=None):
    """Parallel that succeeds as soon as ONE child succeeds (SuccessOnOne).

    Semantics: tick every child each tick; SUCCESS the moment any child
    succeeds, FAILURE only when all have failed, RUNNING otherwise. Use for
    "race these concurrently, first to finish wins".

    ``SuccessOnOne`` takes no ``synchronise`` argument in py_trees 2.1.6.

    Equivalent to ``py_trees.composites.Parallel(name,
    policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=children)``.
    """
    return py_trees.composites.Parallel(
        name=name,
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=children)
