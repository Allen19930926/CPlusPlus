cmake_minimum_required(VERSION 3.5)

set(NLOHMANNJSON_INCLUDES ${PROJECT_SOURCE_DIR}/dependencies/nlohmann)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NlohmannJson DEFAULT_MSG NLOHMANNJSON_INCLUDES)
