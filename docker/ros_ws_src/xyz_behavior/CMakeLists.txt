cmake_minimum_required(VERSION 3.0.2)
project(xyz_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  ainex_interfaces)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs ainex_interfaces)

# Per-project entry nodes are added here as projects are (re)built, e.g.:
# catkin_install_python(PROGRAMS
#   <proj>/app/<proj>_bt_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")
