cmake_minimum_required(VERSION 3.0)

project(fusion_unit_test)

set(ROOT_DIR ${PROJECT_SOURCE_DIR}/..)
# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${ROOT_DIR}/out)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${ROOT_DIR}/out)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -fno-exceptions -Wno-unused-function")

include(${ROOT_DIR}/script/cmake/preprocess.cmake)

# 递归查找文件夹下的 *.cpp 文件保存到 TEST_SRC_FILES
file(GLOB_RECURSE TEST_SRC_FILES "*.cpp")
include_directories(mock)
# 将 HEDADER_FILES 和 TEST_SRC_FILES 保存到 AllFile 变量
set(AllFile ${TEST_HEADER_FILES} ${TEST_SRC_FILES})

# 对 AllFile 变量里面的所有文件分类(保留资源管理器的目录结构)
foreach(fileItem ${AllFile})       
    # Get the directory of the source file
    get_filename_component(PARENT_DIR "${fileItem}" DIRECTORY)
    # Remove common directory prefix to make the group
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}" "" GROUP "${PARENT_DIR}")
    # Make sure we are using windows slashes
    string(REPLACE "/" "\\" GROUP "${GROUP}")
    # Group into "Source Files" and "Header Files"
    set(GROUP "${GROUP}")
    source_group("${GROUP}" FILES "${fileItem}")
endforeach()

add_subdirectory(${ROOT_DIR}/app fusion)

add_executable(${PROJECT_NAME} ${AllFile})

if(CMAKE_HOST_WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE fusion gtest glog)
elseif(CMAKE_HOST_UNIX)
    target_link_libraries(${PROJECT_NAME} fusion_app gtest glog -lpthread -lgcov)
endif()

