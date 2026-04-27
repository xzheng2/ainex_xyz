cmake_minimum_required(VERSION 3.0.2)
project(xyz_bt_edu)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs ainex_interfaces message_generation)

catkin_python_setup()

add_message_files(FILES
  BTNodeEvent.msg
  BTRunComplete.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs ainex_interfaces message_runtime)

catkin_install_python(PROGRAMS scripts/bt_edu_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
