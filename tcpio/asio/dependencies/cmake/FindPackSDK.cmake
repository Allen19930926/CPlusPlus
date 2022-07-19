cmake_minimum_required(VERSION 3.5)

set(PACKSDK_LIBRARIES pack-sdk-0.1.40)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PackSDK DEFAULT_MSG PACKSDK_LIBRARIES)