cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_maneuvering)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/send_maneuver_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
