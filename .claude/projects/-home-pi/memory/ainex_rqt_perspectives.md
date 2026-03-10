# Ainex rqt Perspectives

## Config File Location (inside container)
```
/home/ubuntu/.config/ros.org/rqt_gui.ini
```

## Launch Command
```bash
docker exec -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 -u ubuntu ainex bash -c \
  'source /opt/ros/noetic/setup.bash && rqt --perspective <name>'
```

## Current Perspectives

### 11111
Active plugins: rqt_plot/Plot, rqt_image_view/ImageView, rqt_graph/RosGraph
- ImageView topic: `/color_detection/image_result`
- Plot topics: `/imu/linear_acceleration/x`, `/imu/linear_acceleration/y`, `/imu/linear_acceleration/z`
- Node Graph: enabled

## How to Set Perspectives via Python

```python
from PyQt5.QtCore import QSettings
s = QSettings("/home/ubuntu/.config/ros.org/rqt_gui.ini", QSettings.IniFormat)

# Set active plugins for a perspective
s.setValue("perspective/11111/pluginmanager/running-plugins", {
    "rqt_plot/Plot": [1],
    "rqt_image_view/ImageView": [1],
    "rqt_graph/RosGraph": [1],
})

# Set default perspective on startup
s.setValue("General/current-perspective", "11111")

s.sync()
```

## Key INI Keys per Perspective
```
perspective/<name>\mainwindow\geometry         # window size/position (ByteArray)
perspective/<name>\mainwindow\state            # dock layout (ByteArray)
perspective/<name>\pluginmanager\running-plugins  # active plugins dict {plugin_id: [instance_id]}
perspective/<name>\pluginmanager\plugin__<plugin_id>__<N>\plugin\<key>  # plugin settings
```

## Plugin IDs
- Node Graph:  `rqt_graph/RosGraph`
- Image View:  `rqt_image_view/ImageView`
- Plot:        `rqt_plot/Plot`
- Console:     `rqt_console/Console`
- Topic Monitor: `rqt_topic/Topic`

## Plugin-Specific Settings Keys
| Plugin | Key | Example |
|--------|-----|---------|
| ImageView | `plugin\topic` | `/color_detection/image_result` |
| Plot | `plugin\topics` | `/imu/linear_acceleration/x, /imu/linear_acceleration/y` |
| Plot | `plugin\autoscroll` | `true` |
| Plot | `plugin\plot_type` | `1` (MatPlot) |

## Notes
- Instance IDs (`__1`, `__2`) in plugin settings must match the instance list in `running-plugins`
- `perspective-file` flag uses JSON export format (different from INI) — avoid, use `--perspective` instead
- `DISPLAY=:1` and `LIBGL_ALWAYS_SOFTWARE=1` always required (see ainex_display_fix.md)
