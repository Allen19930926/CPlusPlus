cmake_minimum_required(VERSION 3.0)

PROJECT (adasbusiness)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(MUDUO_DIR ${PROJECT_SOURCE_DIR}/../ThirdParty)

#message("*****************************", ${MUDUO_DIR})

SET(SRC_LIST source/*.cpp)

include_directories(
	include
	${MUDUO_DIR}
)

link_directories(
    MUDUO_DIR/muduo/lib
)

aux_source_directory(${PROJECT_SOURCE_DIR}/source ADAS_FILES)

add_library(${PROJECT_NAME} STATIC ${ADAS_FILES})




