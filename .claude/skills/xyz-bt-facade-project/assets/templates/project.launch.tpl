<launch>
  <!-- TODO: add project-specific perception nodes above the BT node.
       Option A — colour detection (detect_type: circle | rect | line):
  <include file="$(find xyz_perception)/launch/color_detection_min.launch">
    <arg name="enable_camera" value="false"/>
    <arg name="detect_color"  value="blue"/>
    <arg name="detect_type"   value="circle"/>
  </include>
       Option B — YOLO detection:
  <node pkg="xyz_perception" type="yolo_detection_node.py"
        name="yolo_detection" output="screen">
    <param name="zmq_host" value="localhost"/>
    <param name="zmq_port" value="5551"/>
  </node>
  -->

  <!-- {{PROJECT_CLASS}} behavior tree node.
       name="{{PROJECT}}" is required: BTExecController uses ~ (private namespace),
       so ~bt/pause resolves to /{{PROJECT}}/bt/pause. -->
  <node pkg="xyz_behavior" type="{{PROJECT}}_bt_node.py"
        name="{{PROJECT}}" output="screen"/>
</launch>
