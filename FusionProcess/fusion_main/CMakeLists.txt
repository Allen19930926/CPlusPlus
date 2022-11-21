cmake_minimum_required(VERSION 3.0)

project(fusion_main)

set(ROOT_DIR ${PROJECT_SOURCE_DIR}/..)
# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${ROOT_DIR}/out)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${ROOT_DIR}/out)

set(SRC_LIST main.cpp)

add_executable(${PROJECT_NAME} ${SRC_LIST})

if(CMAKE_HOST_WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE fusion_app glog)
elseif(CMAKE_HOST_UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE fusion_app glog -lpthread)
endif()