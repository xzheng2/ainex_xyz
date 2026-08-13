# Per-body configuration

One YAML file per physical robot body: `bodies/<body_id>.yaml`.

## Why a config layer and not a git branch

Bodies differ — servo zero offsets, gait trim, camera mounting. The tempting fix is
"one git branch per robot". Don't: a per-body branch never merges back, so every code
improvement made on one robot has to be cherry-picked onto every other, and the
branches drift until they are separate projects. Body differences are **data**, so they
live in versioned data files that every body shares and any body can read.

## Resolving `body_id` — the WiFi hotspot SSID

A body's id is the SSID of the access point it runs, e.g. `HW-ROBOPARKS676EF55C`. That
name already encodes the unit and exists before anyone configures anything. `hostname`
is deliberately not used: every stock Raspberry Pi answers to `raspberrypi`, and two
bodies sharing an id would silently merge their results under one
`results/<body_id>/` prefix — discoverable only long after the experiment.

`xyz_run_lab/run_lab/run_context.py: resolve_body_id()` resolves in this order:

1. environment variable `AINEX_BODY_ID`
2. the cache file `xyz_behavior/log/.body_id`
3. probe: the active NetworkManager connection whose `802-11-wireless.mode` is `ap`
   (a body usually also has an `infrastructure` uplink, so the mode is what picks the
   right one) — the result is then written to the cache

There is no fourth step. An unidentified body raises `BodyIdError` rather than guessing.

**Why the cache exists.** Step 3 only works on the host. `nmcli` is installed inside the
ainex container but cannot reach NetworkManager's D-Bus there, so the containerised BT
node reads the cache the host wrote. `log/.body_id` sits inside the bind mount and is
gitignored, which is exactly right: it is machine state, not source.

Renaming the access point therefore means: rename `<body_id>.yaml`, delete
`log/.body_id`, and restart. Or pin it explicitly and skip the probe entirely:

```bash
echo 'export AINEX_BODY_ID=HW-ROBOPARKS676EF55C' >> ~/.zshrc
```

## Adding a body

1. On the new robot, read its AP name: `python3 …/xyz_run_lab/run_lab/run_context.py`
2. Copy an existing `<body_id>.yaml` to `<that SSID>.yaml`.
3. Fill in the identity block and whatever genuinely differs on that body.
4. Commit the file — every body gets it, which is the point: calibration becomes
   reviewable and recoverable instead of living only on one SD card.

## What belongs in here

Values that differ **because of the physical body**: servo trim, gait offsets, camera
extrinsics, worn-out-joint workarounds. Anything that is the same on every body belongs
in the normal package config (`ainex_kinematics/config/`, launch defaults, node
`CONFIG_DEFAULTS`) — putting shared values here just creates N copies to drift apart.

Experiment variant parameters do **not** belong here either: they vary per run, not per
body, and are recorded in each run's `run_meta.json` in the results repo.
