cmake_minimum_required(VERSION 3.7.2)
project(scms_test)

find_package(catkin)
catkin_package()

install(
  PROGRAMS
    scripts/disco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
