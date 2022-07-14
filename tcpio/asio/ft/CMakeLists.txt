cmake_minimum_required(VERSION 3.0)

PROJECT (FUNCTIONTEST)

OPTION(ENABLE_GCOV "Enable gcov (debug, Linux builds only)" OFF)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fprofile-arcs -ftest-coverage")
add_definitions("-Wall -g --coverage")

SET(ADAS_DIR ${PROJECT_SOURCE_DIR}/..)

#message("*****************************", ${ADAS_DIR}/app)

add_subdirectory(${ADAS_DIR}/app adasout)

SET(SRC_LIST main.cpp fusion_case/fusion_test.cpp)

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
ADD_EXECUTABLE(ft ${SRC_LIST} )

target_link_libraries(ft gtest_main gtest -lpthread -lgcov adasbusiness)


