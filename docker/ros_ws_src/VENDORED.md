# Vendored third-party ROS packages

Two packages in this workspace come from upstream repositories but are tracked here
as **plain files**, not as git submodules.

## Why vendored rather than submodules

They used to be submodules — badly. `git ls-files -s` recorded them as mode `160000`
gitlinks while **no `.gitmodules` file existed**, so the URLs were nowhere in the
repository. A fresh `git clone` produced two empty directories and
`git submodule update --init` failed with no URL to fetch from; the catkin workspace
could not be built on a new robot at all.

Vendoring fixes that and buys two more things this project actually needs:

- **The ROS Noetic arm64 apt repository is gone (404).** These packages *must* be built
  from source, so the source has to travel with the workspace regardless.
- **Local patches survive.** `rqt_py_trees` carries a fix that exists in no upstream
  (see below). As a gitlink it lived only inside the nested `.git` on one machine and
  would have been lost the moment the repo was cloned elsewhere.

Cost: ~650 KB in the parent repo, and upstream updates become a manual re-vendor
instead of a `git submodule update`. Both are acceptable at this size and update rate.

## Inventory

### `py_trees_msgs`

| | |
|---|---|
| Upstream | https://github.com/stonier/py_trees_msgs.git |
| Vendored at | `b431dc580c3b0c1447592712552202bb288a2b2c` (tag `0.3.7`) |
| Local changes | none |

### `rqt_py_trees`

| | |
|---|---|
| Upstream | https://github.com/splintered-reality/rqt_py_trees.git (branch `devel`) |
| Upstream base | `0b3a324` (tag `0.4.1`) |
| Vendored at | `71e1bbf49579ee7f7080f810054b91ea66361506` |
| Local changes | **1 commit on top of upstream** — see patch below |

**Local patch** — `71e1bbf` "Fix save_settings to use visibility_level.value
(py_trees 2.1.6 enum compat)":

```diff
--- a/src/rqt_py_trees/behaviour_tree.py
+++ b/src/rqt_py_trees/behaviour_tree.py
@@ -552,7 +552,7 @@ class RosBehaviourTree(QObject):
     def save_settings(self, plugin_settings, instance_settings):
-        instance_settings.set_value('visibility_level', self.visibility_level)
+        instance_settings.set_value('visibility_level', self.visibility_level.value)
```

`rqt_py_trees` 0.4.1 predates py_trees 2.x, where `visibility_level` became an
`enum.IntEnum`. Qt's `QSettings.set_value()` cannot serialise the enum object, so
saving the rqt perspective raised on shutdown until the `.value` was passed instead.
This workspace pins py_trees **2.1.6** (pip, not apt — see `CLAUDE.md`), so the patch
must be kept through any re-vendor.

## Re-vendoring from upstream

```bash
cd /tmp && git clone <upstream-url> pkg && cd pkg && git checkout <new-ref>
rm -rf .git
rsync -a --delete /tmp/pkg/ /home/pi/docker/ros_ws_src/<pkg>/
# reapply the local patch above if the package is rqt_py_trees, then:
cd /home/pi && git add docker/ros_ws_src/<pkg> && git commit
```

Update the table above in the same commit — the vendored ref is only knowable from
this file once the nested `.git` is gone.
