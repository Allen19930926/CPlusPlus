cmake_minimum_required(VERSION 3.0)

PROJECT (adasbusiness)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++14 -fprofile-arcs -ftest-coverage")

add_subdirectory(domain)

SET(SRC_LIST infrastructure/tcp/source/*.cpp domain/data_fusion/source/*.cpp domain/proxy/source/*.cpp)

IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
SET(SRC_LIST infrastructure/ipcm/source/aarch64/*.cpp)
ELSE()
SET(SRC_LIST infrastructure/ipcm/source/x64/*.cpp)
ENDIF()

include_directories(
	application/include
	domain/data_fusion/include
	domain/proxy/include
	infrastructure/common
	infrastructure/common/ipc
	infrastructure/tcp/include
	infrastructure/ipcm/include
)


aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/common ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/tcp/source ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/domain/data_fusion/source ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/domain/proxy/source ADAS_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/application/source ADAS_FILES)

IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/ipcm/source/aarch64 ADAS_FILES)
ELSE()
    aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/ipcm/source/x64 ADAS_FILES)
ENDIF()

add_library(${PROJECT_NAME} STATIC ${ADAS_FILES})
IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
target_link_libraries(${PROJECT_NAME} ipc-shm adasdomain)
ENDIF()
target_link_libraries(${PROJECT_NAME} -lpthread -lgcov adasdomain)




