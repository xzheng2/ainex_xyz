# rqt Marathon Perspective — Setup Guide

Use this document after analyzing the last run's log summary to recommend
which rqt plugins the user should open for the next run.

## Available rqt Plugins (33 installed in ainex container)

| Plugin ID | Name | Use Case |
|-----------|------|----------|
| rqt_console/Console | Console | Live rosout messages with level/node filtering |
| rqt_image_view/ImageView | Image View | View camera/image topics (raw, compressed, annotated) |
| rqt_plot/Plot | Plot | Time-series plots of numeric topics (IMU, battery, etc.) |
| rqt_graph/RosGraph | Node Graph | Interactive node/topic connection graph |
| rqt_topic/Topic | Topic Monitor | Topic list with publication Hz rates |
| rqt_robot_monitor/RobotMonitor | Robot Monitor | Diagnostics aggregator |
| rqt_tf_tree/TfTree | TF Tree | Transform tree visualization |
| rqt_reconfigure/Reconfigure | Dynamic Reconfigure | Live parameter tuning GUI |
| rqt_logger_level/LoggerLevel | Logger Level | Change per-node log verbosity at runtime |
| rqt_service_caller/ServiceCaller | Service Caller | Call ROS services from GUI |
| rqt_bag/Bag | Bag Recorder | Record/playback rosbag files |
| rqt_top/Top | Process Monitor | CPU/memory usage per node |
| rqt_launch/Launch | Launch | Start/stop launch files from GUI |

## How to Recommend Plugins

After reading the log summary from `read_last_run_summary`, identify:
1. **What failed** — recommend plugins that would make failures visible
2. **What's important to monitor** — based on which nodes/topics are active
3. **What the user cares about** — marathon = walking + vision + safety

### Example Recommendations Based on Common Issues

**Camera crashes** → ImageView (`/camera/image_raw`) + Console (filter ERROR level)
**Missing nodes** → Node Graph (shows disconnected graph) + Topic Monitor (Hz = 0)
**Walking issues** → Plot (`/imu/linear_acceleration/*`) + Console (filter `/walking`)
**BT not running** → Console (filter `marathon_bt`) + Node Graph
**General monitoring** → Console + ImageView + Node Graph + Topic Monitor

## How to Launch rqt

```bash
docker exec -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 -u ubuntu ainex bash -c \
  'source /opt/ros/noetic/setup.bash && source /home/ubuntu/ros_ws/devel/setup.bash && rqt'
```

Then add plugins via the **Plugins** menu in rqt.

## Perspective File Format (for automation)

Save as `.perspective` and load with `rqt --perspective-file <path>`:

```ini
[Perspective]
count=N
name=marathon

[Plugin 0]
plugin=rqt_console/Console

[Plugin 1]
plugin=rqt_image_view/ImageView

[Plugin 2]
plugin=rqt_plot/Plot
args_str=/imu/linear_acceleration

[Plugin 3]
plugin=rqt_graph/RosGraph
```

Load with:
```bash
docker exec -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 -u ubuntu ainex bash -c \
  'source /opt/ros/noetic/setup.bash && source /home/ubuntu/ros_ws/devel/setup.bash && \
   rqt --perspective-file /path/to/marathon.perspective'
```

## Key Topics for Marathon Monitoring

| Topic | Type | Why Monitor |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Camera health — Hz drops to 0 on crash |
| `/color_detection/image_result` | sensor_msgs/Image | Annotated vision output (line detection) |
| `/object/pixel_coords` | ainex_interfaces/ObjectsInfo | Line detection data consumed by BT |
| `/walking/is_walking` | std_msgs/Bool | Whether gait engine is active |
| `/imu/linear_acceleration` | geometry_msgs/Vector3 | Fall detection input |
| `/ros_robot_controller/battery` | std_msgs/UInt16 | Battery voltage (mV), LOW < 10500 |
| `/rosout_agg` | rosgraph_msgs/Log | All node log messages |
