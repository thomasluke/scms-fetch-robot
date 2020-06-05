cmake_minimum_required(VERSION 3.0.2)
project(vision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package( catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    message_generation
    drink_menu
    tf
    tf2_eigen
    tf2_geometry_msgs
    resource_retriever
  )

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    AlvarMarkers
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_geometry_msgs  
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)
  
find_package(AlvarMarkers)
include_directories(${AlvarMarkers_INCLUDE_DIRS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vision src/vision.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES} ${AlvarMarker_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(vision ${catkin_EXPORTED_TARGETS})
install(TARGETS vision DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(vision_service src/service.cpp src/vision_c.cpp)
target_link_libraries(vision_service ${OpenCV_LIBRARIES} ${AlvarMarker_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(vision_service ${catkin_EXPORTED_TARGETS})
install(TARGETS vision_service DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
