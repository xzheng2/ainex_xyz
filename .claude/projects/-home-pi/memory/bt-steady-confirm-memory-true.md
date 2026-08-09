---
name: bt-steady-confirm-memory-true
description: "For steady-detection confirmation in xyz_bt_lib BTs, wrap the condition in LatchedDwellDecorator (BB-backed); the old built-in node dwell + DwellDecorator were removed Aug 2026"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 114b9cae-a404-40a9-88bc-237539048f2f
  modified: 2026-08-09T16:28:35.454Z
---

For "detect steadily for N ticks then act" gates, use
`LatchedDwellDecorator` (`xyz_bt_lib.core.latched_dwell`) wrapping the pure L1
condition. Its dwell counter lives in the Blackboard at `/node_state/<state_key>`,
so it survives a reactive (`memory=False`) ancestor's per-tick re-entry — it works
anywhere in the tree, no special parent shape required.

**Superseded (Aug 2026 cleanup):** both the base-node built-in dwell
(`XyzL1ConditionNode.status_from_bool` + `succeed_dwell_ticks`/`fail_dwell_ticks`/`_sc`)
and `DwellDecorator` (`core/decorators.py`) were **deleted**. L1 nodes are now pure
predicates (SUCCESS/FAILURE only, no cross-tick state). The old `memory=True`
confirm-sequence workaround is no longer needed and no longer possible.

**Why the old approaches failed:** an instance-state counter (built-in `_sc`, or
DwellDecorator's `_count`) is wiped by `initialise()`/`terminate()` when a
`memory=False` ancestor re-invalidates its RUNNING descendant each tick, pinning the
count at 1 forever. Measured proof: `examples/demo_memory_interaction.py`.
`LatchedDwellDecorator` fixes this by storing state in the BB instead — measured in
`examples/demo_latched_dwell.py` (scenario 3, under a ReactiveSequence).

**How to apply:**
- `LatchedDwellDecorator(condition, required_ticks=N, state_key='<literal>', tick_id_getter=…)`.
- `state_key` MUST be a hardcoded string literal (greppable; duplicate fails at construction).
- `required_ticks` is a number injected via the `build_tree`/`bootstrap` param; rosparam
  only in the app/ layer.
- Timing is by `tick_id` (survives blocking `run_action`), not wall clock.
- See [[bt-extend-not-new-node]] and the composites factories (`ReactiveSequence` etc.).
