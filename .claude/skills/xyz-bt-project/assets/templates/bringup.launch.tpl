<!--
  {{PROJECT}} full hardware bringup.

  This template is a STARTING POINT. Activate only the ROS nodes your project
  requires; leave unused blocks commented out.

  Minimal launch order:
    1. Base hardware (servos, IMU, walking controller)
    2. Sensors required by this project (camera, etc.)
    3. Vision/perception pipeline if needed
    4. BT node

  To launch BT-only (no hardware): use {{PROJECT}}_bt_node.launch directly.
-->
<launch>
  <!-- ── Args ──────────────────────────────────────────────────────────── -->
  <arg name="use_camera"    default="true"/>
  <arg name="use_web_video" default="false"/>

  <!-- ── 1. Base hardware bringup ──────────────────────────────────────── -->
  <include file="$(find ainex_bringup)/launch/base.launch"/>

  <!-- ── 2. Camera ─────────────────────────────────────────────────────── -->
  <group if="$(arg use_camera)">
    <!-- TODO: configure usb_cam params for this project -->
    <!-- <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
      <param name="video_device"     value="/dev/video0"/>
      <param name="image_width"      value="640"/>
      <param name="image_height"     value="480"/>
      <param name="framerate"        value="30"/>
      <param name="camera_frame_id"  value="camera"/>
      <param name="io_method"        value="mmap"/>
    </node> -->
  </group>

  <!-- ── 3. Project-specific perception / vision pipeline ──────────────── -->
  <!-- Add object detection, line detection, apriltag_ros, etc. here -->
  <!-- Example:
  <node name="color_detect" pkg="ainex_perception" type="color_detect_node.py"
        output="screen"/>
  -->

  <!-- ── 4. Web video server (optional debug stream) ───────────────────── -->
  <group if="$(arg use_web_video)">
    <node name="web_video_server" pkg="web_video_server" type="web_video_server"
          output="screen">
      <param name="port" value="8080"/>
    </node>
  </group>

  <!-- ── 5. BT node ────────────────────────────────────────────────────── -->
  <include file="$(find xyz_behavior)/{{PROJECT}}/{{PROJECT}}_bt_node.launch"/>

</launch>
