cmake_minimum_required(VERSION 3.7.2)
project(scms_test)

find_package(catkin)
catkin_package()

add_compile_options(-std=c++11)
add_executable(test src/main.cpp)
target_link_libraries(test ${catkin_LIBRARIES})

install(
  PROGRAMS
    scripts/disco.py
    src/main.cpp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)