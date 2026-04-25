cmake_minimum_required(VERSION 3.0.2)
project(xyz_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  ainex_interfaces
  ainex_example)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs ainex_interfaces ainex_example)

catkin_install_python(PROGRAMS
  marathon/app/marathon_bt_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY marathon/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/marathon
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.launch"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
    PATTERN "log" EXCLUDE)
