set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

string(REGEX REPLACE ".*/\(.*\)" "\\1" CURDIR ${CMAKE_CURRENT_SOURCE_DIR})

OPTION(FT_TEST "complie control to remove hobot files" OFF)

if(CMAKE_HOST_WIN32)
    message(STATUS "current plantform: window")
    add_definitions(-D GLOG_NO_ABBREVIATED_SEVERITIES)
elseif(CMAKE_HOST_UNIX)
    message(STATUS "current platnform: unix")
    add_definitions("-Wall -g")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -fno-exceptions")
endif()

message(STATUS "current FT_TEST state is: ${FT_TEST}")


# 设置全局可以引用的头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../app
    ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include
)

# 为跨平台不通用的库文件单独添加头文件目录
if(CMAKE_HOST_WIN32)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/cross_platform/window/include)
elseif(CMAKE_HOST_UNIX)
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
        include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/cross_platform/aarch64/include)
    else()
        include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/cross_platform/linux/include)
    endif()
endif()

    
if (NOT (FT_TEST EQUAL 2))
    include_directories(app/infrastructure/tcp)
endif()

# 设置全局可以添加的依赖库目录
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/lib)
# 为跨平台不通用的库文件单独添加库文件目录
if(CMAKE_HOST_WIN32)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/cross_platform/window/lib)
elseif(CMAKE_HOST_UNIX)
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
        link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/cross_platform/aarch64/lib)
    else()
        link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/cross_platform/linux/lib)
    endif()
endif()
