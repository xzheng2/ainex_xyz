---
name: ablation-experiment-layout
description: "Ablation experiments use a second git repo outside the code repo, plus hotspot-SSID body ids and per-run log directories"
metadata: 
  node_type: memory
  type: project
  originSessionId: f1539859-db40-4b19-9d76-a19013065b02
  modified: 2026-08-11T09:01:07.945Z
---

Multi-body ablation experiments (set up Aug 11 2026) split across **two** repositories:

- code — the existing `/home/pi` repo (`xzheng2/ainex_xyz`)
- results — **`/home/pi/experiments/ainex_xyz_result`** (singular "result"), a separate git
  repo on `master`, remote `https://github.com/xzheng2/ainex_xyz_result.git` (**public**,
  created Aug 11 2026, mirrors ainex_xyz's settings). **Invisible from the code repo** —
  the deny-all `.gitignore` swallows `experiments/`, so `git status` in `/home/pi` will
  never show it and it is easy to forget it exists.

Results are partitioned `results/<body_id>/<run_id>/` with a per-body index shard
`index/<body_id>.jsonl`. Bodies only ever write under their own prefix, so concurrent
pushes cannot conflict — that is why there are **no per-body branches** (they never merge
back and block code improvements from propagating).

`body_id` is the robot's **WiFi access point SSID** (this body: `HW-ROBOPARKS676EF55C`),
never the hostname — every stock Pi answers to `raspberrypi`. Resolution lives in
`xyz_behavior/bt_observability/run_context.py`: `AINEX_BODY_ID` → `log/.body_id` cache →
nmcli probe for the connection whose `802-11-wireless.mode` is `ap`. **The probe only
works on the host**: nmcli exists in the ainex container but cannot reach NetworkManager's
D-Bus, which is the whole reason the cache file exists.

Per-run flow: `new_run.py` (independent variables, BEFORE the run) → robot runs →
**`close_run.py --latest --outcome ...`** (dependent variables: machine metrics reduced by
`bt_observability/run_metrics.py` + operator-recorded outcome/interventions/failure_mode →
`metrics.json`) → `publish_runs.py --commit`. Skipping close does not block publishing but
marks the run `closed: false` in the index; such runs are excluded from ablation tables
exactly like `dirty: true` ones.

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

Related: [[ainex_bt_observability]], [[docstring-is-the-spec]]
