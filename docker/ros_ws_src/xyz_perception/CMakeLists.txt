cmake_minimum_required(VERSION 3.0.2)
project(xyz_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs ainex_interfaces message_generation)

catkin_python_setup()

add_message_files(FILES
  DepthNavState.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs ainex_interfaces message_runtime)

catkin_install_python(PROGRAMS
  src/xyz_perception/color/color_detection_min_node.py
  src/xyz_perception/yolo/yolo_detection_node.py
  src/xyz_perception/apriltag/apriltag_detection_node.py
  src/xyz_perception/depth_nav/depth_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# config/ is installed, unlike in the old layout: apriltag_detection.launch loads its
# two yaml files through $(find xyz_perception), which resolves to the share dir once
# the package is installed rather than run from a devel workspace.
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
