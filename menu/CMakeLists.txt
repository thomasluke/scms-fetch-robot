# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8)


# Change project name
project (drinks_menu) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(src)
add_subdirectory(libs)

# include_directories("$ENV{HOME}/libraries/pugixml/src")
# link_directories("$ENV{HOME}/libraries/pugixml/src")


add_executable(menu src/drinks.cpp)

#link the library (here we called it shapes, you do not need to specify the lib (library would be called libshapes)
target_link_libraries(menu pugixml)