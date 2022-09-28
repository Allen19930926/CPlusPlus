cmake_minimum_required(VERSION 3.0)

project(post-fusion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

OPTION(FT_TEST "complie control to remove hobot files" OFF)

add_definitions("-Wall -g")

add_subdirectory(app)

SET(SRC_LIST stand_alone/main.cpp)

include_directories(
    app/infrastructure/common
    app/infrastructure/tcp
    app/infrastructure/KM_algorithm
)

ADD_EXECUTABLE(post-fusion  ${SRC_LIST})

target_link_libraries(post-fusion -lpthread FUSION_BUSINESS)



