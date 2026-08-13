<launch>
  <!-- TODO: add project-specific perception nodes above the BT node.
       Option A — colour detection:
  <node pkg="ainex_example" type="color_detection_node.py"
        name="color_detection_node" output="screen"/>
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
