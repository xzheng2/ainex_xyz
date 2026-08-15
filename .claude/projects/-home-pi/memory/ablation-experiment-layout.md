---
name: ablation-experiment-layout
description: "Ablation experiments use a second git repo outside the code repo, partitioned by body_id AND lane, plus a parallel session-transcript pipeline"
metadata: 
  node_type: memory
  type: project
  originSessionId: f1539859-db40-4b19-9d76-a19013065b02
  modified: 2026-08-15T19:52:54.755Z
---

Multi-body ablation experiments (set up Aug 11 2026) split across **two** repositories:

- code — the existing `/home/pi` repo (`xzheng2/ainex_xyz`)
- results — **`/home/pi/experiments/ainex_xyz_result`** (singular "result"), a separate git
  repo on `master`, remote `https://github.com/xzheng2/ainex_xyz_result.git` (**public**,
  created Aug 11 2026, mirrors ainex_xyz's settings). **Invisible from the code repo** —
  the deny-all `.gitignore` swallows `experiments/`, so `git status` in `/home/pi` will
  never show it and it is easy to forget it exists.

Results are partitioned `results/<body_id>/<study>/<lane>/<run_id>/` with a per-writer index
shard `index/<body_id>__<study>__<lane>.jsonl`. Each writer only ever writes under its own
prefix, so concurrent pushes cannot conflict — that is why there are **no per-writer
branches** (they never merge back and block code improvements from propagating).

**A writer is a body AND a study AND a lane.** The four-lane ablation runs four SD cards on
ONE robot, so `body_id` is identical across all four; keying the partition on body alone
would put four concurrent writers on one index file, the exact conflict the sharding
prevents. `study` (`exp1`/`exp2`/…) sits ABOVE `lane` for a different reason — not conflict
but conflation: a card outlives a study, the same lane-`a` card runs exp1 then exp2, and
pooling those under one prefix silently averages two experiments. Always filter by study
before aggregating; `cat index/*__exp1__*.jsonl` is the intended idiom.

Both resolve like `body_id`: `AINEX_STUDY` → `xyz_behavior/log/.study`, `AINEX_LANE` →
`log/.lane` (**not** in `xyz_run_lab/` — that package must be byte-identical in every lane).
`new_run.py --study exp1 --lane a` sets both, `--study exp2` re-sets it when the card moves
on. `lane` ∈ a closed `a|b|c|d`; `study` is a slug pattern instead, because a closed set
would mean editing the instrumentation package to start a third experiment. Missing either
is a hard failure, like a missing body_id.

`body_id` is the robot's **WiFi access point SSID** (this body: `HW-ROBOPARKS676EF55C`),
never the hostname — every stock Pi answers to `raspberrypi`. Resolution lives in
`xyz_run_lab/run_lab/run_context.py`: `AINEX_BODY_ID` → `log/.body_id` cache →
nmcli probe for the connection whose `802-11-wireless.mode` is `ap`. **The probe only
works on the host**: nmcli exists in the ainex container but cannot reach NetworkManager's
D-Bus, which is the whole reason the cache file exists.

Per-run flow: `new_run.py` (independent variables, BEFORE the run) → robot runs →
**`close_run.py --latest --outcome ...`** (dependent variables: machine metrics reduced by
`xyz_run_lab/run_lab/run_metrics.py` + operator-recorded outcome/interventions/failure_mode
→ `metrics.json`) → `publish_runs.py --commit`. Skipping close does not block publishing but
marks the run `closed: false` in the index; such runs are excluded from ablation tables
exactly like `dirty: true` ones.

**Process data is a second, parallel pipeline**: `publish_session.py` → `sessions/<body_id>/
<study>/<lane>/<session_id>/` + `index/sessions/<body_id>__<study>__<lane>.jsonl`. It exports a *distilled*
Claude Code transcript (tokens, tool calls, rework, clarifications) plus that session's
slice of `~/.claude/logs/guard_events.jsonl`. Runs and sessions do not correspond 1:1,
hence two tools. Three traps, all measured: the CLI writes one line per content block and
repeats `usage` on each, so **dedupe by `message.id`** (2.56× inflation otherwise);
subagent tokens live in separate `<sessionId>/subagents/agent-*.jsonl` files and are
invisible in the main transcript; and the results repo is **public**, so the transcript is
distilled rather than copied — raw ones carry every file read and every command's output.

`run_meta.json`'s `git.dirty` is **scoped to `docker/ros_ws_src` only** (`RUNTIME_PATHS` in
`run_context.py`, recorded per-run in `git.dirty_scope`, schema `ainex.run_meta/2`).
Editing `.claude/` hooks, skills or templates does NOT mark a run irreproducible — none of
it is loaded by a running node, and counting it would leave the flag permanently on.
`git.repo_dirty` keeps the whole-repo signal as information, without paths.

Each run gets `xyz_behavior/log/runs/<run_id>/`; the legacy fixed filenames in `log/`
become **relative** symlinks to the newest run, which is what keeps ROSA
(`BT_OBS_DIR=/opt/ainex_bt_log`), `bt_log_read_guard.py` and the diagnose skill working
unchanged. Relative, because that directory has a different absolute path in each
container. Writers must target the run dir, never the symlinks — `os.replace()` would
turn a symlink into a regular file. Only the newest 10 runs survive, deleted at node
startup published or not, so `tools/publish_runs.py` is the step that makes a run
permanent.

The BT observability layer that writes into those run dirs is documented by its own module
docstrings under `docker/ros_ws_src/xyz_behavior/bt_observability/` — the `ainex_bt_observability`
memory that used to be linked here was deleted Aug 15 2026 as re-derivable from that code.
