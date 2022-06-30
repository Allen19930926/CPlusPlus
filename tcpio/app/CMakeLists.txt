cmake_minimum_required(VERSION 3.0)

PROJECT (adasbusiness)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(MUDUO_DIR ${PROJECT_SOURCE_DIR}/../ThirdParty)

#message("*****************************", ${PROJECT_SOURCE_DIR}/tcp/source)

SET(SRC_LIST tcp/source/*.cpp v2x_data_fusion/source/*.cpp)

include_directories(
	common
	tcp/include
	v2x_data_fusion/include
	${MUDUO_DIR}
)

link_directories(
    MUDUO_DIR/muduo/lib
)

aux_source_directory(${PROJECT_SOURCE_DIR}/tcp/source ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/v2x_data_fusion/source ADAS_FILES)

add_library(${PROJECT_NAME} STATIC ${ADAS_FILES})




