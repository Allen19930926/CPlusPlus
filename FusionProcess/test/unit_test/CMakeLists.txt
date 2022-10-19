cmake_minimum_required(VERSION 3.0)

PROJECT (FUSION_TEST)

OPTION(FT_TEST "complie control to remove hobot files" OFF)
message("FTTEST, " ${FT_TEST})

SET(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../..)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/dependencies/cmake)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(gtest REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions("-Wall -g --coverage")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -fno-exceptions")

add_subdirectory(${PROJECT_SOURCE_DIR}/app fusion)

SET(SRC_LIST main.cpp)
aux_source_directory(${PROJECT_SOURCE_DIR}/test/unit_test/domain FT_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/test/unit_test/infrastructure/kalman_filter FT_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/test/unit_test/infrastructure/sensor_object FT_FILES)

include_directories(
    ${PROJECT_SOURCE_DIR}/dependencies/gtest/include
    ${PROJECT_SOURCE_DIR}/app/domain/data_association
    ${PROJECT_SOURCE_DIR}/app/infrastructure/sensor_object
    ${PROJECT_SOURCE_DIR}/app/infrastructure/track_object
    ${PROJECT_SOURCE_DIR}/app/infrastructure/kalman_filter
)

link_directories(
    ${PROJECT_SOURCE_DIR}/dependencies/gtest/lib
    ${GLOG_LIBRARIES}
)

ADD_EXECUTABLE(ft ${SRC_LIST} ${FT_FILES})

target_link_libraries(ft gtest_main gtest -lpthread -lgcov fusion_bussiness glog)


