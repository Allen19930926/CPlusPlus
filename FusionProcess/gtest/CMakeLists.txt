cmake_minimum_required(VERSION 3.0)

PROJECT (FUSION_TEST)

OPTION(FT_TEST "complie control to remove hobot files" OFF)
message("FTTEST, " ${FT_TEST})


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/..)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
add_definitions("-Wall -g --coverage")

add_subdirectory(${PROJECT_SOURCE_DIR}/app fusion)

SET(SRC_LIST main.cpp)
aux_source_directory(${PROJECT_SOURCE_DIR}/gtest/func_test/source FT_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/gtest/unit_test/infrastructure FT_FILES)

message(${PROJECT_SOURCE_DIR}/ft/unit_test/infrastructure)

include_directories(
    ThirdParty/gtest/include
    ${PROJECT_SOURCE_DIR}/app/infrastructure/KM_algorithm
)

link_directories(
    ThirdParty/gtest/lib
)

ADD_EXECUTABLE(ft ${SRC_LIST} ${FT_FILES})

target_link_libraries(ft gtest_main gtest -lpthread -lgcov FUSION_BUSINESS)


