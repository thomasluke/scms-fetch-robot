cmake_minimum_required(VERSION 3.7.2)
project(grasping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    genmsg
    message_generation

    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
    DIRECTORY srv
FILES
    pose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    geometry_msgs

    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

# include_directories(include ${catkin_INCLUDE_DIRS})


include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

# add_subdirectory(move_group_interface)

add_executable(grasping src/main.cpp src/grasp.cpp)
add_executable(grasping_service src/service.cpp src/grasp.cpp)
target_link_libraries(grasping ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(grasping_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS grasping DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS grasping_service DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# # install(
# #   PROGRAMS
# #     scripts/disco.py
# #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)