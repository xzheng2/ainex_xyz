---
name: hysteresis-falling-edge-gap
description: "Known gap: falling-edge hysteresis (old fail_dwell_ticks, 1-enter/N-exit debounce) has NO decorator replacement after the Aug 2026 dwell cleanup"
metadata: 
  node_type: memory
  type: project
  originSessionId: d055617e-ad8f-471b-982b-d8918d8a3d1a
---

The Aug 2026 dwell cleanup removed `XyzL1ConditionNode`'s built-in
`succeed_dwell_ticks`/`fail_dwell_ticks`. `fail_dwell_ticks` was **falling-edge
hysteresis** ("1-tick enter, N-tick exit": from stable SUCCESS, require N
consecutive False frames before FAILURE — anti-flicker on target loss).

`LatchedDwellDecorator` ([[bt-steady-confirm-memory-true]]) does the opposite
asymmetry — **N-enter, 1-exit**: it latches after `required_ticks` passes but
unlatches on the FIRST non-pass. So it **cannot** express falling-edge debounce.

**Status: known gap, no replacement, intentionally deferred.** No node currently
depends on it — the six formerly-hysteresis L1 nodes all defaulted the params to 0,
and the only callers were the wiped `xyz_behavior` trees. If a real tree later needs
falling-edge debounce, add a BB-backed `HysteresisDecorator` with independent
enter/exit thresholds (design + empirical tests mirroring `LatchedDwellDecorator`) —
do NOT reintroduce dwell state into an L1 node (L1 stays a pure predicate). Recorded
in the `core/latched_dwell.py` module docstring ("Known gap" section) — spec.md was
deleted Aug 9 2026 ([[docstring-is-the-spec]]).
