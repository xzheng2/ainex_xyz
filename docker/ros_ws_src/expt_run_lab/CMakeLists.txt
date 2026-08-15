cmake_minimum_required(VERSION 3.0.2)
project(expt_run_lab)

find_package(catkin REQUIRED)

catkin_package()

# Deliberately no catkin_python_setup(): run_lab/ is never installed onto PYTHONPATH.
# Callers reach it with sys.path.insert() against $(rospack find expt_run_lab), exactly
# the mechanism xyz_behavior already uses for bt_observability. Installing it would be a
# second, divergent import path for the same files.
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
