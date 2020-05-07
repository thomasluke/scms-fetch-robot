cmake_minimum_required(VERSION 3.7.2)
project(scms_test)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(my_ros_test src/main.cpp)
target_link_libraries(my_ros_test ${catkin_LIBRARIES})

install(
  PROGRAMS
    scripts/disco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)