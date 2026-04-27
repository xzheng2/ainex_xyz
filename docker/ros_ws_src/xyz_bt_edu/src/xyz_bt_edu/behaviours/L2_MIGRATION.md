# L2 Locomotion Node Migration — Conformance Backlog

> **Created**: 2026-04-22
> **Completed**: 2026-04-22 (v2.5.0)
> **Context**: Phase b L1 conformance is complete (v2.4.0).
> This document tracks the equivalent work for the 6 canonical L2 nodes.

---

## What Was Done for L1 (reference)

Each L1 node had the same set of violations; all were fixed in v2.4.0:

| Fix | Before | After |
|---|---|---|
| Base class | `XyzBTNode` | `XyzL1ConditionNode` |
| `__init__` | `super().__init__(name)` + manual field assigns | `super().__init__(name, logger=..., tick_id_getter=...)` |
| Logging | `self._logger.emit_bt({...})` | `self.emit_decision(inputs=..., status=..., reason=...)` |
| Helper | logic inline in `update()` | explicit `_is_xxx()` / `_evaluate_xxx()` method |
| Declarations | `BB_LOG_KEYS` only | `BB_READS`, `BB_WRITES`, `FACADE_CALLS`, `CONFIG_DEFAULTS` |
| Docstring | partial | full required fields |

---

## L2 Files — Current Violations (all 6)

Verified by grep. **Every** L2 file has:

1. `from xyz_bt_edu.base_node import XyzBTNode` → must be `XyzL2ActionNode`
2. `class L2_xxx(XyzBTNode)` → `class L2_xxx(XyzL2ActionNode)`
3. `super().__init__(name)` — does not pass `logger`, `tick_id_getter`, or `facade`
4. Manual `self._tick_id_getter = tick_id_getter or (lambda: -1)` in `__init__`
5. Missing `BB_READS`, `BB_WRITES`, `FACADE_CALLS`, `CONFIG_DEFAULTS` class declarations
6. `self._logger.emit_bt({...})` pattern instead of `self.emit_decision()` /
   `self.emit_action_intent()`
7. Missing `_compute_command()` or `_select_action()` helper for non-trivial strategy
8. Docstring incomplete (no BB reads/writes, facade calls, strategy, helpers, defaults, returns)

---

## Recommended Edit Order (simplest → most complex)

### 1. `L2_Gait_Stop.py` ✅ trivial
- Calls `facade.stop_walking()` only; no BB reads; no strategy helper needed.
- `update()` always returns `Status.SUCCESS`.

### 2. `L2_Head_MoveTo.py` ✅ simple
- One constructor param `pan_pos`; calls `facade.move_head(pan_pos)`.
- `update()` always returns `Status.SUCCESS`.

### 3. `L2_Balance_RecoverFromFall.py` ✅ moderate
- Reads `BB.ROBOT_STATE`; calls `facade.recover_from_fall(robot_state)`.
- Returns `RUNNING` (delegate; facade manages the sequence internally).
- `initialise()` should emit `action_intent`.

### 4. `L2_Gait_FindLine.py` ✅ moderate-complex
- Reads `BB.LAST_LINE_ERROR_X`, `BB.CAMERA_LOST_COUNT`.
- Inlines direction+magnitude calc → calls `facade.turn_step(...)`.
- Strategy helper: `_compute_search_turn(last_error_x, lost_count) -> dict`.
- Returns `RUNNING` always (search until line found by L1 condition above it).

### 5. `L2_Gait_FollowLine.py` ✅ complex
- Reads `BB.LINE_DATA`, `BB.LINE_ERROR_X`.
- Inlines visual-patrol yaw algorithm → calls `facade.go_step` or `facade.turn_step`.
- Strategy helper: `_compute_follow_step(line_error_x, line_data) -> dict`.
- Returns `RUNNING` always.

### 6. `L2_Head_FindLineSweep.py` ✅ most complex (state machine)
- SWEEP → ALIGN two-phase state machine; `_fresh_start` bool flag guard in `initialise()`.
- Reads `BB.LINE_DATA`, `BB.LAST_LINE_ERROR_X`, `BB.HEAD_PAN_POS`.
- Writes `BB.HEAD_PAN_POS`.
- Calls `facade.move_head()`, `facade.go_step()`, `facade.turn_step()`.
- Strategy helpers:
  - `_compute_sweep_step(head_pan, direction) -> dict`
  - `_compute_align_turn(last_error_x) -> dict`
- Returns `RUNNING` (SWEEP/ALIGN in progress), `SUCCESS` (ALIGN complete), `FAILURE` (unused).
- **Critical invariants** — must not break:
  - `gait_manager.disable()` in `initialise()` guarded by `line_data is None`
    (prevents gait stop/restart at 30 Hz during ALIGN turns)
  - SWEEP_STEP must not be a factor of 200 (half-range), to avoid exact center hit
  - ALIGN turn uses `go_step`/`turn_step` profile switch at threshold ≈ 2°
  - `_fresh_start` reset only in `__init__` and on ALIGN SUCCESS (not on every initialise call)

---

## `XyzL2ActionNode.__init__` Signature

```python
super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
```

`facade` is the injected `XyzBTFacade` implementation. All project trees
pass `facade=self._facade` when constructing L2 nodes.

---

## Logging Pattern for L2

`initialise()` (action start):
```python
self.emit_action_intent(
    action='stop_walking',   # or whichever facade method
    inputs={...},
)
```

`update()` (decision / outcome):
```python
self.emit_decision(
    inputs={...},
    status=status,
    reason='...',
)
```

Never call `self.emit_bt({...})` directly or `self._logger.emit_bt(...)`.

---

## Declarations Template for L2

```python
LEVEL        = 'L2'
BB_READS     = [BB.SOME_KEY, ...]
BB_WRITES    = []           # or list keys written
FACADE_CALLS = ['method_name']
CONFIG_DEFAULTS = {
    'param': default_value,
}
```

---

## Notes

- `L2_Head_FindLineSweep` is the riskiest edit. Read the full tick-interruption
  fix pattern in `xyz_bt_edu.md` (memory) before touching it.
- After editing each file, run the import check inside the container:
  ```bash
  docker exec ainex bash -c "source /home/ubuntu/ros_ws/devel/setup.bash && \
    python3 -c 'from xyz_bt_edu.behaviours.L2_locomotion.<ClassName> import <ClassName>; print(\"OK\")'"
  ```
- Run the full compliance grep after finishing all 6:
  ```bash
  grep -rn "XyzBTNode\|emit_bt\b\|self\._logger\s*=" \
    docker/ros_ws_src/xyz_bt_edu/src/xyz_bt_edu/behaviours/L2_locomotion/
  ```
  Should return no matches.
