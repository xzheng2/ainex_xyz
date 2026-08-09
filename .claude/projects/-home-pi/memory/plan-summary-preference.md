---
name: plan-summary-preference
description: How the user wants plans/changes presented before executing
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 371521b0-8aff-418d-a54b-a8818ff61d63
---

When presenting a plan for approval, show ONLY the current round's proposed changes — a
short, focused list — not the whole accumulated plan file.

**Why:** The plan file grows with addenda across many turns; re-showing the entire thing
is noise. The user tracks work round-by-round.

**How to apply:** Before ExitPlanMode, write a concise per-round change summary (files +
what changes, verification). Keep full detail in the plan file, but the message to the
user should be just this round's diff-level intent. Related: [[MEMORY]].
