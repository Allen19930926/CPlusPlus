cmake_minimum_required(VERSION 3.0)

PROJECT (adasdomain)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fprofile-arcs -ftest-coverage")

if (FT_TEST)

else ()
    add_subdirectory(communication-user)
endif()

include_directories(
	data_fusion/include
	proxy/include
	${PROJECT_SOURCE_DIR}/../application/include
	${PROJECT_SOURCE_DIR}/../infrastructure/common
	${PROJECT_SOURCE_DIR}/../infrastructure/tcp/include
)

aux_source_directory(${PROJECT_SOURCE_DIR}/data_fusion/source DOMAIN_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/proxy/source DOMAIN_FILES)

add_library(${PROJECT_NAME} STATIC ${DOMAIN_FILES})

target_link_libraries(${PROJECT_NAME}  -lpthread -lgcov )




