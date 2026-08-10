---
name: docstring-is-the-spec
description: xyz_bt_lib has NO central spec doc since Aug 9 2026 — docs/spec.md deleted; module docstrings are the authoritative spec. Do not recreate spec.md.
metadata: 
  node_type: memory
  type: project
  originSessionId: d055617e-ad8f-471b-982b-d8918d8a3d1a
  modified: 2026-08-09T21:40:10.844Z
---

`xyz_bt_lib/docs/spec.md` (1314 lines) was **deleted Aug 9 2026** (user decision,
after evaluation showed it was a hand-maintained mirror that had drifted: node
catalog said 22 nodes vs 27 actual files, stale dwell_ticks descriptions).
`docs/plan_d_phase3_migration.md` deleted too (completed one-time plan); the
whole `docs/` dir is gone. Both recoverable from git history.

**Why:** every fact in spec.md had a closer source of truth — node/adapter/module
docstrings (BB reads/writes, rules, CONFIG_DEFAULTS), `blackboard_keys.py`,
`base_facade.py`, the skills, git history. Maintaining the mirror cost 4-way sync
and it drifted anyway.

**How to apply:**
- The module docstring IS the authoritative spec. After any public change, the
  docstring must fully describe BB reads/writes, judgement/strategy rules,
  CONFIG_DEFAULTS. Never recreate a central spec.md.
- Unique content was migrated, not lost: deprecated-BB-key list → comment block
  in `blackboard/blackboard_keys.py` (/locomotion/ section); falling-edge
  hysteresis gap note → `core/latched_dwell.py` docstring (the gap itself was
  closed Aug 9 2026 by `core/hysteresis.py`, see [[bt-latch-vs-hysteresis]]).
- `src/xyz_bt_lib/README.md` is the human entry point: package nav +
  docstring-as-spec pointers + build/verify commands + pseudocode templates.
- Skills `xyz-bt-lib-node` / `xyz-bt-lib-adapter` no longer contain "update
  docs/spec.md" steps — their checklists now require docstring completeness.
