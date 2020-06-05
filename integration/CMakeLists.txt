cmake_minimum_required(VERSION 3.5)

project(integration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED 
    COMPONENTS 
        roscpp
        std_msgs
        geometry_msgs
        genmsg
        message_generation
        grasping
)

## Declare a catkin package
catkin_package(
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS 
        roscpp 
)

include_directories(include 
    ${THIS_PACKAGE_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

add_executable(integration
    src/main.cpp
    src/integration.cpp
)

target_link_libraries(integration ${catkin_LIBRARIES})
add_dependencies(integration ${catkin_EXPORTED_TARGETS})

install(TARGETS integration DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})