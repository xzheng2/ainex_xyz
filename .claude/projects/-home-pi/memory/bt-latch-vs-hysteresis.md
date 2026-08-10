---
name: bt-latch-vs-hysteresis
description: "Two BB-backed stability gates in xyz_bt_lib and when to use each: LatchedDwellDecorator (N-in/1-out) vs HysteresisDecorator (N-in/M-out, added Aug 9 2026)"
metadata: 
  node_type: memory
  type: project
  originSessionId: d055617e-ad8f-471b-982b-d8918d8a3d1a
  modified: 2026-08-09T21:40:01.575Z
---

`xyz_bt_lib` has **two** tree-layer gates for turning a noisy/unsteady condition
into a trustworthy one. Both are BB-backed at `/node_state/<state_key>`, both
time by `tick_id`, both wrap ONLY {SUCCESS, FAILURE} things (never an L2).
Picking the wrong one is the mistake this note exists to prevent.

- **`LatchedDwellDecorator`** (`core/latched_dwell.py`) — **N-in / 1-out**.
  Latches SUCCESS after `required_ticks` consecutive passes; **one** failing tick
  unlatches. Returns RUNNING while counting up.
  Use for: *"wait until the world is stable, then commit; abandon instantly."*
  Safety gates, precision alignment before a committed motion.

- **`HysteresisDecorator`** (`core/hysteresis.py`, added Aug 9 2026) —
  **N-in / M-out**, independent `enter_ticks` / `exit_ticks`. **Never returns
  RUNNING** — output is a debounced boolean, so it won't hold a ReactiveSequence
  branch RUNNING while counting.
  Use for: *"this measurement is noisy; hold the answer steady."* Flickering
  perception (a detector dropping one frame in twenty), a target hovering at a
  detection boundary.

**Why both:** the latch's 1-out edge is correct for safety but wrong for flicker
— a single dropped detection frame tears down a branch that is still valid.
`enter_ticks=1, exit_ticks=N` on the hysteresis gate reproduces exactly the
removed node-level `fail_dwell_ticks` (the capability gap that was deliberately
deferred from Aug 2026 until a real need appeared; now closed).

**How to apply:**
- `state_key` must be a hardcoded literal AND unique across BOTH decorators —
  they share the `/node_state/` namespace. The `xyz_bt_tree_pre_guard.py` hook
  enforces the literal requirement for both automatically (its regex targets the
  `state_key=` parameter name, not a class name — no hook change was needed).
- Thresholds are NUMBERS injected via `bootstrap()` params, never rosparam read
  inside tree wiring.
- Never put either kind of state inside an L1 node — L1 stays a pure predicate
  ([[bt-steady-confirm-memory-true]]).
- Executable proof: `examples/demo_latched_dwell.py` and
  `examples/demo_hysteresis.py` (9 scenarios each, both pass).
