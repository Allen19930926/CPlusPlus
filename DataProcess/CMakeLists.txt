cmake_minimum_required(VERSION 3.0)

PROJECT (DEMO)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions("-Wall -g")

add_subdirectory(business)

aux_source_directory(ft SRC_LIST)
aux_source_directory(ft/fusion_case FT_LIST)

include_directories(
    common
	business/include
    ThirdParty/gtest/include
)

link_directories(
    ThirdParty/gtest/lib
)

ADD_EXECUTABLE(demo ${SRC_LIST} ${FT_LIST})

target_link_libraries(demo gtest_main gtest -lpthread adasbusiness)


