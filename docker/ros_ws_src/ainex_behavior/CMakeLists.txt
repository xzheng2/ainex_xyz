cmake_minimum_required(VERSION 3.0.2)
project(ainex_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  ainex_interfaces
  ainex_example)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs ainex_interfaces ainex_example)

catkin_install_python(PROGRAMS
  marathon/marathon_bt_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY marathon/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/marathon
  FILES_MATCHING PATTERN "*.launch")
