---
name: bt-extend-not-new-node
description: Prefer extending an existing BT node with tunable config over creating a near-duplicate node
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6bb93a1e-ed5e-4e03-b588-029d446fdce7
---

When a task needs a variant of an existing xyz_bt_edu BT node (e.g. aligning the
body to a non-zero heading), **extend the existing node with a tunable config
param** rather than creating a new near-duplicate node.

Example: instead of a new `L2_Gait_AlignHeadingToTarget`, extend
`L2_Gait_AlignHeading` with `target_deg` (default 0, so existing callers are
unchanged) plus an optional `target_bb_key` to read the target from the
blackboard live.

**Why:** avoids duplicate gait/logic nodes drifting apart; keeps the shared
library small; default value preserves backward compatibility.

**How to apply:** add the new behavior as a `CONFIG_DEFAULTS` entry with a
no-op default; support a readable BB key when the value must vary at runtime.
Related: [[bt-steady-confirm-memory-true]].
