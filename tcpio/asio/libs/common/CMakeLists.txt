cmake_minimum_required(VERSION 3.15)
#add_compile_options(-fPIC)
find_package(NlohmannJson REQUIRED)
add_library(common OBJECT
    src/util.cc
    src/can.cc
    src/timestamp.cc
    src/data_handler.cc)
target_include_directories(common PUBLIC include ${NLOHMANNJSON_INCLUDES})
target_include_directories(common PRIVATE include/common)
