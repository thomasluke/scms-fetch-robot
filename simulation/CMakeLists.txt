cmake_minimum_required(VERSION 3.7.2)
project(bar_simulation)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

install(
  PROGRAMS
    scripts/disco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)