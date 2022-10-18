cmake_minimum_required(VERSION 3.5)

set(GTEST_INCLUDES ${PROJECT_SOURCE_DIR}/dependencies/gtest/include)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(gtest DEFAULT_MSG GTEST_INCLUDES)
