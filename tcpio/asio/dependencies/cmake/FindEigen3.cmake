cmake_minimum_required(VERSION 3.5)

set(EIGEN3_INCLUDES ${PROJECT_SOURCE_DIR}/dependencies/eigen3)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDES)
