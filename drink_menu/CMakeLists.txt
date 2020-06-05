cmake_minimum_required(VERSION 3.5)

project(drink_menu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)


###
### QT
### https://cpc110.blogspot.com/2020/04/how-to-properly-add-qrc-resource-folder.html
###

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
#find all the qt UI stuff
file(GLOB UI
    "src/*.ui"
)
#make them into headers
qt5_wrap_ui (MB_UI_HDRS  ${UI})

#make them into headers
qt5_wrap_ui (MB_UI_HDRS  ${UI})

# Generate resources 
qt5_add_resources(RESOURCES_RCC ${RESOURCE})

include_directories(include ${catkin_INCLUDE_DIRS})


file(GLOB SRCS
    "src/*.h"
    "src/*.cpp"
    "src/*.hpp"
)

file(GLOB QT_SRCS
    "src/*.h"
    "src/*.cpp"
    "src/*.hpp"
)

add_message_files(
    DIRECTORY msg
FILES
    drink.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime roscpp std_msgs
)

add_executable(drink_menu
    ${MB_UI_HDRS} ${QT_SRCS} ${SRCS} ${RESOURCE}
)

target_link_libraries(drink_menu Qt5::Widgets Qt5::Core ${catkin_LIBRARIES} )
add_dependencies(drink_menu drink_menu_generate_messages_cpp)
