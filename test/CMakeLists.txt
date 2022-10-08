cmake_minimum_required(VERSION 3.0)

project(asio)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib")

add_definitions("-Wall -g")

SET(SRC_LIST main.cpp)

include_directories(
    common/
)

ADD_EXECUTABLE(asio  ${SRC_LIST})


target_link_libraries(asio -lpthread)


