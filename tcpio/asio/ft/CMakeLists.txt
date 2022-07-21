cmake_minimum_required(VERSION 3.0)

PROJECT (FUNCTIONTEST)

OPTION(FT_TEST "complie control to remove hobot files" OFF)
message("FTTEST, " ${FT_TEST})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -std=c++11 -fprofile-arcs -ftest-coverage")
add_definitions("-Wall -g --coverage")

SET(ADAS_DIR ${PROJECT_SOURCE_DIR}/..)

message("*****************************", ${ADAS_DIR}/app)

add_subdirectory(${ADAS_DIR}/app adasout)

SET(SRC_LIST main.cpp)
aux_source_directory(${PROJECT_SOURCE_DIR}/ft_context/source FT_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/fusion_case FT_FILES)

include_directories(
    ${ADAS_DIR}/app/infrastructure/common
	${ADAS_DIR}/app/domain/data_fusion/include
	${ADAS_DIR}/app/domain/proxy/include
	${ADAS_DIR}/app/application/include
    ${ADAS_DIR}/ThirdParty/gtest/include
    ft_context/include
)

link_directories(
    ${ADAS_DIR}/ThirdParty/gtest/lib
)
ADD_EXECUTABLE(ft ${SRC_LIST} ${FT_FILES})

target_link_libraries(ft gtest_main gtest -lpthread -lgcov adasbusiness)


