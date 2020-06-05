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

# file(GLOB SRCS
#     "src/*.h"
#     "src/*.cpp"
#     "src/*.hpp"
# )

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    grasping
)

## Declare a catkin package
catkin_package(
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS 
        message_runtime 
        roscpp 
        std_msgs 
        geometry_msgs
        grasping
)

include_directories(include ${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(integration
    src/main.cpp
    src/integration.cpp
)

target_link_libraries(integration ${catkin_LIBRARIES})

install(TARGETS integration DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})