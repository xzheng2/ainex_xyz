# xyz_perception/apriltag

AprilTag detection node + configuration.
Independent of `ainex_example` — all settings live here.

## What it does

Launches `apriltag_ros_continuous_node` with its own detector settings and tag
database, publishing detected tags on `/tag_detections`
(`apriltag_ros/AprilTagDetectionArray`).

## Directory layout

```
xyz_perception/
  config/apriltag/
    settings.yaml   ← detector parameters (tag family, threads, etc.)
    tags.yaml       ← tag database: which IDs to detect and their physical size
  launch/
    apriltag_detection.launch   ← launch file (include from your project launch)
  src/xyz_perception/apriltag/
    apriltag_detection.py       ← OpenCV detector (used by the ZMQ node below)
    apriltag_detection_node.py  ← ZMQ-fed alternative to apriltag_ros
```

## Adding a tag

Edit `config/tags.yaml`:

```yaml
standalone_tags:
  [
    {id: 0, size: 0.10, name: 'tag_0'},   # example
    {id: 7, size: 0.15, name: 'my_new_tag'},       # add your tag here
  ]
```

- `id` — integer printed on the tag image (e.g. `tag36h11_id7.png` → id 7)
- `size` — physical side length of the solid black/white border **in metres**
- `name` — optional label used for the TF frame

Changes take effect on the next `roslaunch`.

## Tag images (tag36h11 family)

Download from: https://github.com/AprilRobotics/apriltag-imgs
Folder: `tag36h11/`
File naming: `tag36h11_id<N>.png`

Print at the physical size specified in `size`. Include a white quiet zone
(border) of at least one tag cell width around the tag.

## Changing the tag family

Edit `config/settings.yaml`, field `tag_family`. Available options:

```
tag36h11  tag25h9  tag16h5
tagStandard52h13  tagStandard41h12
tagCircle21h7  tagCircle49h12  tagCustom48h12
```

`tag36h11` is recommended: highest robustness, low false-positive rate.
Make sure the printed tags match the selected family.

## /tag_detections message type

```
apriltag_ros/AprilTagDetectionArray
  std_msgs/Header header
  AprilTagDetection[] detections
    int32[]   id                          # e.g. [0]
    float64[] size                        # physical size in metres, e.g. [0.10]
    geometry_msgs/Point[4] corners        # pixel corners: [0]=TL [1]=TR [2]=BR [3]=BL
                                          # .x .y = pixels; .z = 0
    geometry_msgs/PoseWithCovarianceStamped pose   # 3D pose in camera frame
```
