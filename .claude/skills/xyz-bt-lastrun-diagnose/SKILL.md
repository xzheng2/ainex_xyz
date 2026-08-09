# Skill: `xyz-bt-lastrun-diagnose`

Diagnose a **completed** Behavior-Tree run from the full-session *lastrun* observability
logs. This is the primary, full-depth forensic path for "why did the robot do X during
that run?" — the developer-facing complement to ROSA's live tools.

**When to use this skill:**
- The user asks to diagnose a whole run / a past run / a post-mortem: "why did it never
  grab the ball", "what happened during that run", "where did it go wrong", "it failed
  earlier in the run".
- The relevant ticks are beyond the live 30-tick recent window (ROSA's `cross_tick_analysis`
  and `analyze_bt_tick` only read the rolling `_recent` files; this skill reads the full
  `_lastrun` files — thousands of ticks).

**When NOT to use:**
- Live / paused-session questions about the last ~30 ticks → that is ROSA's
  `cross_tick_analysis` / `analyze_bt_tick`, not this skill.

## What the logs are

Each `xyz_behavior` project writes 4 JSONL files in its `log/` dir. The two `_lastrun`
files hold the **entire run** (often thousands of ticks, 10s of MB):
- `bt_debug_lastrun.jsonl` — BT decision layer: `tree_tick_start/end`, `tick_end` (per-node
  status), `decision` (condition reasoning + inputs), `bb_write`.
- `bt_ros_comm_debug_lastrun.jsonl` — ROS comm layer: `ros_in` (sensor in), `input_state`
  (adapter BB writes), `ros_out` (commands emitted).

Never `cat`/read a whole lastrun file into context — it is far too large. Always go through
the deterministic reducer below, then drill into a bounded tick range.

## The reducer (single source of truth)

The host CLI wraps the same deterministic reducer ROSA uses. Run it with **host python3**
(no container, no langchain needed):

```
CLI=/home/pi/docker/rosa-agent/xyz_agent_tools/bt_analysis/lastrun_digest_cli.py
```

It has two modes:
- `--detail digest` — bounded whole-run overview (state occupancy profile + transition-type
  histogram). Stays small (~thousands of tokens) for runs of any length. **Start here.**
- `--detail full --tick-selection A-B` — verbose per-tick scaffold (node statuses, decisions,
  blackboard, ros_in/out) for a *bounded* tick range. Use to drill into a suspect span.

`--out text` (default) is human/LLM-readable; `--out json` returns the raw bundle.

## Procedure

### 1. Locate the run

Auto-detect the most recently modified lastrun across all projects (no live ROS topic exists
post-mortem, so pick by mtime):

```bash
ls -t /home/pi/docker/ros_ws_src/xyz_behavior/*/log/bt_debug_lastrun.jsonl \
      /home/pi/docker/ros_ws_src/xyz_behavior/log/bt_debug_lastrun.jsonl 2>/dev/null | head -1
```

The directory of that file is your `LOG_DIR`. If the user named a project, use its
`xyz_behavior/<proj>/log` instead. Confirm the LOG_DIR you picked in your reply.

### 2. Whole-run digest first

```bash
python3 "$CLI" --log-dir "$LOG_DIR" --detail digest
```

Read the **STATE OCCUPANCY PROFILE** (which BT states dominated, for how many ticks/visits)
and the **TRANSITION TYPES** histogram (the distinct kinds of behavior change, with example
`to_tick`s). Use these to form a hypothesis and pick the suspect tick range / transition.
For example: a state where `IsBallTracked` is always FAILURE that occupies most of the run
points to a perception problem; a transition kind that never fires points to a gate that was
never satisfied.

Tighten the occupancy view with `--top-n N`, or scope the digest to a phase with
`--tick-selection A-B`.

### 2b. Pull ROSA user observations (human evidence)

If the user talked to the ROSA agent during the run, those turns are gold — they hold the
human's runtime observation ("why is the head sweeping but not approaching the ball?") and
which tick(s) it was about. Fold them in:

```bash
SC=/home/pi/.claude/skills/xyz-bt-lastrun-diagnose/session_correlate.py
python3 "$SC" --bt-log "$LOG_DIR/bt_debug_lastrun.jsonl"
```

This prints **tick-anchored** observations (when the user asked via `analyze_bt_tick` /
`get_bt_tick_raw` / `cross_tick_analysis` / `get_bt_status`, the tool call names the exact
tick) and **unanchored** ones (general during-run context — NOT tick-precise). It auto-selects
only ROSA session logs whose time range overlaps the run. If it prints "no ROSA session
overlapped this run", there is simply no human evidence for this run — proceed on logs alone.

**Trust model:** an anchored observation pins a human statement to specific tick(s) — treat it
as primary alongside the BT evidence. Wall-clock time is NOT used to map turns to ticks (the
BT free-runs while a turn takes seconds), so never infer a tick from a turn's timestamp.

### 3. Drill into the suspect span

```bash
python3 "$CLI" --log-dir "$LOG_DIR" --detail full --tick-selection 556-561
# observations anchored to just this window:
python3 "$SC" --bt-log "$LOG_DIR/bt_debug_lastrun.jsonl" --tick-range 556-561
```

Keep the range small (a few to a few dozen ticks). The digest CLI returns representative
per-tick snapshots with the full decision inputs, blackboard, and emitted ROS commands, plus
transition evidence (before/after) for that window; the correlator surfaces any user
observation tied to those ticks (fills the digest's OBSERVATION SLOT).

### 4. Raw evidence (optional, bounded)

To read exact raw lines for specific ticks (e.g. a single `decision` or `ros_out` payload),
filter the JSONL directly with `jq` on `tick_id` — **always bounded**, never the whole file:

```bash
jq -c 'select(.tick_id>=556 and .tick_id<=561)' "$LOG_DIR/bt_debug_lastrun.jsonl"
jq -c 'select(.tick_id>=556 and .tick_id<=561)' "$LOG_DIR/bt_ros_comm_debug_lastrun.jsonl"
```

(Lastrun files are newest-first after a clean shutdown; `jq` preserves file order. Tick
ordering does not affect the reducer — it sorts internally.) Cross-reference `ros_out`
payloads and `bb_write`/`input_state` drift against the `decision` reasons to confirm the
root cause.

### 5. Report

Tie the diagnosis to concrete `tick_id`s and the transition where behavior diverged. State
what the robot was doing, what evidence drove (or failed to drive) the key decision, and
which BT node/condition is responsible. Where a ROSA user observation is anchored to the
divergence tick(s), corroborate or contradict it against the BT evidence explicitly (matches
/ mismatches / not provable from logs).

## Scaffolding for beginners

The user may be new to behavior trees. When useful, explain in plain terms: a Selector tries
its children until one succeeds; a Sequence runs steps in order and stops on the first
failure; a node returns SUCCESS / FAILURE / RUNNING each tick; an L1 condition reads the
Blackboard and answers a yes/no question and returns only SUCCESS/FAILURE (never RUNNING —
RUNNING comes from an L2 action or from a `LatchedDwellDecorator`); an L2 action does
something that takes time. Frame
the diagnosis around which condition gated the tree and why, not just raw log fields. Explain
ROS-layer details (topics, adapters, payloads) only if the user asks about input/output.

## Blackboard key namespaces in the logs

- `/latched/<key>` — per-tick sensor snapshot written by input adapters (the tick's
  consistent world view). This is what L1 predicates read.
- `/node_state/<state_key>` — a `LatchedDwellDecorator`'s own persisted dwell state,
  value `{'stable_ticks', 'latched', 'last_tick_id'}`. When a stability gate seems
  stuck or fires early, read this: `stable_ticks` shows how far the dwell has counted,
  `latched` whether it has fired, and a jump in `last_tick_id` (gap > threshold) means
  the dwell was reset by a stale re-entry. `<state_key>` is the hardcoded literal from
  the tree wiring, so it is greppable across the tree and the logs.

## Notes

- The reducer is deterministic (no LLM); the CLI is stdlib-only and imports the `bt_analysis`
  subpackage directly, bypassing `xyz_agent_tools/__init__.py` (which needs langchain).
- ROSA's `session_digest` tool runs the *same* digest in-container for a beginner operator
  during runtime; this skill is the deeper, iterative drill-down path for a developer.
- `session_correlate.py` (in this skill dir) is standalone stdlib — no rosa-agent import; it
  parses ROSA's `logs/session_*.jsonl` transcripts and anchors user observations to tick_ids.
- Source: reducer `xyz_agent_tools/bt_analysis/raw_cross_tick.py`, formatter
  `xyz_agent_tools/bt_analysis/cross_tick_format.py`, CLI `…/lastrun_digest_cli.py` under
  `/home/pi/docker/rosa-agent/`.
