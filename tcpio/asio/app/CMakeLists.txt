cmake_minimum_required(VERSION 3.0)

PROJECT (adasbusiness)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fprofile-arcs -ftest-coverage")

OPTION(FT_TEST "complie control to remove hobot files" OFF)

if (FT_TEST)

else ()
    add_subdirectory(communication-user)
endif()

include_directories(
	application/include
	domain/data_fusion/include
	domain/proxy/include
	infrastructure/common
	infrastructure/tcp/include
    communication-user
)

aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/tcp/source ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/domain/data_fusion/source ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/domain/proxy/source ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/application/source ADAS_FILES)

add_library(${PROJECT_NAME} STATIC ${ADAS_FILES})


    target_link_libraries(${PROJECT_NAME}  -lpthread -lgcov )




