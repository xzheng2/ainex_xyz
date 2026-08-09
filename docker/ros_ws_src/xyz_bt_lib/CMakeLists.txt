cmake_minimum_required(VERSION 3.0.2)
project(xyz_bt_lib)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs ainex_interfaces message_generation)

catkin_python_setup()

add_message_files(FILES
  BTNodeEvent.msg
  BTRunComplete.msg
  DepthNavState.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs ainex_interfaces message_runtime)

catkin_install_python(PROGRAMS
  examples/bt_lib_node.py
  src/xyz_bt_lib/sensors/color/color_detection_min_node.py
  src/xyz_bt_lib/sensors/yolo/yolo_detection_node.py
  src/xyz_bt_lib/sensors/apriltag/apriltag_detection_node.py
  src/xyz_bt_lib/sensors/depth_nav/depth_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
