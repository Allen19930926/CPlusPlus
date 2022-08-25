cmake_minimum_required(VERSION 3.0)

PROJECT (FUNCTIONTEST)

OPTION(FT_TEST "complie control to remove hobot files" OFF)
message("FTTEST, " ${FT_TEST})


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
add_definitions("-Wall -g --coverage")

SET(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/..)
SET(CMAKE_SOURCE_DIR   ${CMAKE_SOURCE_DIR}/..)
SET(CMAKE_CURRENT_SOURCE_DIR   ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(import_deps)
find_package(Eigen3 REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/libs libsout)
add_subdirectory(${PROJECT_SOURCE_DIR}/app adasout)

SET(SRC_LIST main.cpp)
aux_source_directory(${PROJECT_SOURCE_DIR}/ft/ft_context/source FT_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/ft/fusion_case FT_FILES)

include_directories(
    ${PROJECT_SOURCE_DIR}/app/infrastructure/common
    ${PROJECT_SOURCE_DIR}/app/infrastructure/common/ipc
    ${PROJECT_SOURCE_DIR}/app/infrastructure/ipcm/include
    ${PROJECT_SOURCE_DIR}/app/infrastructure/tcp/include
    ${PROJECT_SOURCE_DIR}/app/infrastructure/udp/include
	${PROJECT_SOURCE_DIR}/app/domain/data_fusion/include
	${PROJECT_SOURCE_DIR}/app/domain/proxy/include
	${PROJECT_SOURCE_DIR}/app/application/include
    ThirdParty/gtest/include
    ft_context/include
)

link_directories(
    ThirdParty/gtest/lib
    ${PROJECT_SOURCE_DIR}/dependencies/linux_default/glog/lib
)
ADD_EXECUTABLE(ft ${SRC_LIST} ${FT_FILES})

target_link_libraries(ft gtest_main gtest -lpthread -lgcov adasbusiness glog)


