cmake_minimum_required(VERSION 3.7.2)
project(arm_movement)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(arm_movement src/main.cpp)
target_link_libraries(arm_movement ${catkin_LIBRARIES})

install(
  PROGRAMS
    scripts/disco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)