cmake_minimum_required(VERSION 3.0)

project(post-fusion)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/dependencies/cmake)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

OPTION(FT_TEST "complie control to remove hobot files" OFF)

add_definitions("-Wall -g")

add_subdirectory(app)

SET(SRC_LIST stand_alone/main.cpp)

include_directories(
    app/infrastructure/tcp
    app/infrastructure/common
    app/domain/infrastructure/kalman_filters
    app/domain/data_association
)

link_directories(
    ${GLOG_LIBRARIES}
)

ADD_EXECUTABLE(post-fusion  ${SRC_LIST})

target_link_libraries(post-fusion -lpthread fusion_bussiness glog)

