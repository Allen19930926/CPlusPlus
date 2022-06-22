cmake_minimum_required(VERSION 3.0)

PROJECT (DEMO)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions("-Wall -g")

add_subdirectory(business)

SET(SRC_LIST ft/demo.cpp)

include_directories(
	business/include
    ThirdParty/
)

link_directories(
    ThirdParty/muduo/lib
)

ADD_EXECUTABLE(demo ${SRC_LIST})


target_link_libraries(demo libmuduo_base.a libmuduo_net.a -lpthread adasbusiness)


