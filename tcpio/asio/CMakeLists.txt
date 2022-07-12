cmake_minimum_required(VERSION 3.0)

PROJECT (DEMO)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
add_definitions("-Wall -g")

add_subdirectory(app)

SET(SRC_LIST ft/main.cpp)

include_directories(
    app/application/include
    app/infrastructure/common
	app/infrastructure/tcp/include
    app/domain/data_fusion/include
    app/domain/proxy/include
)

ADD_EXECUTABLE(demo ${SRC_LIST})


target_link_libraries(demo -lpthread adasbusiness)


