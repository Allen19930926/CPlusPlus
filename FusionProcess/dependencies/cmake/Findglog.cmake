cmake_minimum_required(VERSION 3.5)

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    set(GLOG_INCLUDES  ${PROJECT_SOURCE_DIR}/dependencies/linux_aarch64/glog/include)
    set(GLOG_LIBRARIES ${PROJECT_SOURCE_DIR}/dependencies/linux_aarch64/glog/lib)
else()
    set(GLOG_INCLUDES  ${PROJECT_SOURCE_DIR}/dependencies/linux_default/glog/include)
    set(GLOG_LIBRARIES ${PROJECT_SOURCE_DIR}/dependencies/linux_default/glog/lib)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(glog DEFAULT_MSG GLOG_INCLUDES GLOG_LIBRARIES)
