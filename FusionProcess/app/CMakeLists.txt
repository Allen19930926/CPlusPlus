cmake_minimum_required(VERSION 3.0)

project(fusion_app)

# 设置编译生成的库输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../dependencies/lib/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../dependencies/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../dependencies/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../dependencies/lib)

# 添加工程源文件
aux_source_directory(application SRC_LIST)
aux_source_directory(domain/data_association SRC_LIST)
aux_source_directory(domain/track_prediction SRC_LIST)
aux_source_directory(domain/track_update SRC_LIST)
aux_source_directory(infrastructure/common SRC_LIST)
aux_source_directory(infrastructure/kalman_filter SRC_LIST)
aux_source_directory(infrastructure/mahalanobis SRC_LIST)
aux_source_directory(infrastructure/sensor_object SRC_LIST)
aux_source_directory(infrastructure/track_object SRC_LIST)
aux_source_directory(infrastructure/hungarian SRC_LIST)

if (NOT (FT_TEST EQUAL 2))
    include_directories(infrastructure/tcp)
    aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/tcp SRC_LIST)
endif()

# 递归查找文件夹下的所有 .h 文件并保存到 HEADER_FILES
file(GLOB_RECURSE HEADER_FILES "*.h")
# 将 HEDADER_FILES 和 SRC_LIST 保存到 AllFile 变量
set(AllFile ${HEADER_FILES} ${SRC_LIST})

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

# 添加工程所需的依赖库
add_library(${PROJECT_NAME} STATIC ${AllFile})

# 启用VS项目文件夹分类
if(CMAKE_HOST_WIN32)
    option(USE_SOLUTION_FOLDERS  ON)
endif()
