---
name: catkin-build-needs-ros-sourced
description: catkin build in the ainex container needs /opt/ros/noetic/setup.bash sourced explicitly — the shell profile does not do it
metadata: 
  node_type: memory
  type: project
  originSessionId: 41f63775-0fe9-4893-9cfd-d77a300f51fc
  modified: 2026-08-13T08:50:27.504Z
---

`docker exec -u ubuntu ainex bash -lc 'cd /home/ubuntu/ros_ws && catkin build'`
fails at `catkin_tools_prebuild` with "catkin not found" whenever `build/` and
`devel/` have just been wiped. The container's `~/.bashrc` has no ROS source
line and the workspace has no `.catkin_tools/` profile pinning an extend path,
so a login shell starts with no ROS environment at all. Prefix the command:

    docker exec -u ubuntu ainex bash -lc \
      'source /opt/ros/noetic/setup.bash && cd /home/ubuntu/ros_ws && catkin build'

Incremental builds hide this — an existing `devel/setup.bash` supplies the
environment — so it only bites on the clean rebuild that verifies dependency
changes. A full clean rebuild of all 23 packages takes ~2m15s on this Pi.

Related: [[ablation-experiment-layout]].
