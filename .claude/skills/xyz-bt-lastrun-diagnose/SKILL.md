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
  status), `decision` (reasoning + inputs — emitted by L1 **and** L2 nodes),
  `action_intent` (L2 action start), `bb_write`.
- `bt_ros_comm_debug_lastrun.jsonl` — ROS comm layer: `ros_in` (sensor in), `input_state`
  (adapter BB writes), `ros_out` (commands emitted), and `ros_topology_snapshot`.
  **Beware the topology snapshot**: it is one enormous line (the whole ROS graph — every
  node's pubs/subs/services) written once, a couple of seconds into the run. A `jq` range
  near the start of the run will pull the entire graph into context despite being
  "bounded" — exclude it with `select(.event != "ros_topology_snapshot")`.

Never `cat`/read a whole lastrun file into context — it is far too large. Always go through
the deterministic reducer below, then drill into a bounded tick range.

## The reducer (single source of truth)

The host CLI wraps the same deterministic reducer ROSA uses. Run it with **host python3**
(no container, no langchain needed):

```
/home/pi/docker/rosa-agent/xyz_agent_tools/bt_analysis/lastrun_digest_cli.py
```

**Write these paths out in full in every command.** Each Bash call runs in a fresh
shell, so a `CLI=...` assignment made in one call is gone by the next — the command
would silently become `python3 "" ...`. Worse for `session_correlate.py`: with the
variable empty the command no longer contains that filename, so `bt_log_read_guard`
sees a bare read of `bt_debug_lastrun.jsonl` and blocks it.

It has two modes:
- `--detail digest` — bounded whole-run overview (state occupancy profile + transition-type
  histogram). Stays small (~thousands of tokens) for runs of any length. **Start here.**
- `--detail full --tick-selection A-B` — verbose per-tick scaffold (node statuses, decisions,
  blackboard, ros_in/out) for a *bounded* tick range. Use to drill into a suspect span.

`--out text` (default) is human/LLM-readable; `--out json` returns the raw bundle.
Also available: `--source {lastrun,recent}` (the only way to point the CLI at the rolling
30-tick tier) and `--user-observation "<text>"` (fills the digest's OBSERVATION SLOT).

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

**If that command prints nothing, stop and ask** which project or log directory to use —
do not carry on with an empty `LOG_DIR`, which fails obscurely several steps later. (No
project directories exist under `xyz_behavior/` until an event is rebuilt, so today the
glob legitimately matches nothing.)

### 2. Whole-run digest first

```bash
LOG_DIR=<the directory you just confirmed>
python3 /home/pi/docker/rosa-agent/xyz_agent_tools/bt_analysis/lastrun_digest_cli.py --log-dir "$LOG_DIR" --detail digest
```

Read the **STATE OCCUPANCY PROFILE** (which BT states dominated, for how many ticks/visits)
and the **TRANSITION TYPES** histogram (the distinct kinds of behavior change, with example
`to_tick`s). Use these to form a hypothesis and pick the suspect tick range / transition.
For example: a state where `L1_Vision_IsObjectDetected` is always FAILURE that occupies most of the run
points to a perception problem; a transition kind that never fires points to a gate that was
never satisfied.

Tighten the occupancy view with `--top-n N`, or scope the digest to a phase with
`--tick-selection A-B`.

### 2b. Pull ROSA user observations (human evidence)

If the user talked to the ROSA agent during the run, those turns are gold — they hold the
human's runtime observation ("why is the head sweeping but not approaching the ball?") and
which tick(s) it was about. Fold them in:

```bash
LOG_DIR=<the directory you confirmed in step 1>
python3 /home/pi/.claude/skills/xyz-bt-lastrun-diagnose/session_correlate.py --bt-log "$LOG_DIR/bt_debug_lastrun.jsonl"
```

This prints **tick-anchored** observations (the user's question went through a tool call
that named tick(s) — `analyze_bt_tick`, `get_bt_tick_raw`, `cross_tick_analysis`,
`session_digest`, `get_bt_status`) and **unanchored** ones (general during-run context — NOT tick-precise). It auto-selects
only ROSA session logs whose time range overlaps the run. If it prints "no ROSA session
overlapped this run", there is simply no human evidence for this run — proceed on logs alone.

**Trust model:** an anchored observation pins a human statement to specific tick(s) — treat it
as primary alongside the BT evidence. But check the WIDTH of the anchor before calling it
pinpoint: a `cross_tick_analysis` turn typically anchors to its whole ~30-tick window, and
raw-tick dumps also absorb the neighbour ticks they include as context. A wide anchor
locates the observation to a span, not to one tick. Wall-clock time is NOT used to map turns to ticks (the
BT free-runs while a turn takes seconds), so never infer a tick from a turn's timestamp.

### 3. Drill into the suspect span

```bash
LOG_DIR=<the directory you confirmed in step 1>
python3 /home/pi/docker/rosa-agent/xyz_agent_tools/bt_analysis/lastrun_digest_cli.py --log-dir "$LOG_DIR" --detail full --tick-selection 556-561
# observations anchored to just this window:
LOG_DIR=<the directory you confirmed in step 1>
python3 /home/pi/.claude/skills/xyz-bt-lastrun-diagnose/session_correlate.py --bt-log "$LOG_DIR/bt_debug_lastrun.jsonl" --tick-range 556-561
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

(Lastrun ordering is NOT guaranteed: files are appended oldest-first during the run and
only reversed to newest-first by `close()` on a clean shutdown — a crash leaves them
oldest-first. `jq` preserves whatever order is on disk. This never affects the reducer or
`session_correlate.py`, both of which are order-independent, but it does change what
`jq | head` shows you.) Cross-reference `ros_out`
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
- `/node_state/<state_key>` — BB-backed state owned by a single node or decorator.
  `<state_key>` is the hardcoded literal from the tree wiring, so it is greppable
  across both the tree and the logs. Three kinds live here:
  - `LatchedDwellDecorator` → `{'stable_ticks', 'latched', 'last_tick_id'}`. When a
    stability gate seems stuck or fires early, read this: `stable_ticks` shows how far
    the dwell has counted, `latched` whether it has fired, and a jump in
    `last_tick_id` (gap > threshold) means it was reset by a stale re-entry.
  - `HysteresisDecorator` → `{'engaged', 'enter_count', 'exit_count', 'last_tick_id'}`.
    For a flickering signal, `engaged` is the debounced answer; `exit_count` climbing
    but never reaching `exit_ticks` means a pass keeps cancelling the exit run — that
    is the anti-flicker behaviour working, not a stuck gate.
  - `L2_Head_SearchSweep` → `{'pan', 'dir', 'tilt', 'pause'}`. If the head appears to
    restart its sweep, check whether `pan` is resuming or resetting to an endpoint.

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
