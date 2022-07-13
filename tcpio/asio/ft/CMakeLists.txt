cmake_minimum_required(VERSION 3.0)

PROJECT (FUNCTIONTEST)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions("-Wall -g")

SET(ADAS_DIR ${PROJECT_SOURCE_DIR}/..)

message("*****************************", ${ADAS_DIR}/app)

add_subdirectory(${ADAS_DIR}/app adasout)

SET(SRC_LIST main.cpp)
aux_source_directory(ft/fusion_case FT_LIST)

include_directories(
    ${ADAS_DIR}/app/infrastructure/common
	${ADAS_DIR}/app/domain/data_fusion/include
	${ADAS_DIR}/app/domain/proxy/include
	${ADAS_DIR}/app/application/include
    ${ADAS_DIR}/ThirdParty/gtest/include
)

link_directories(
    ${ADAS_DIR}/ThirdParty/gtest/lib
)

ADD_EXECUTABLE(fusion_ft ${SRC_LIST} ${FT_LIST})

target_link_libraries(fusion_ft gtest_main gtest -lpthread adasbusiness)


