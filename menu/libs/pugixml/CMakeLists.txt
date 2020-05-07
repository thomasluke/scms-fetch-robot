
# Create the library, replace <library_name> with the name of library 
# Replace <cpp_files> with the files that form part of library
add_library(pugixml STATIC pugixml.cpp)
# Replace <header_files> with the header files that form part of library and need to be distributed, seperated by ";" like "simulator.h;types.h"
set_target_properties(pugixml  PROPERTIES PUBLIC_HEADER "pugiconfig.hpp;pugixml.hpp")