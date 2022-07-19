cmake_minimum_required(VERSION 3.5)

if (MSVC)
    set(PROTOBUF_LIBRARY libprotobuf)
else()
    set(PROTOBUF_LIBRARY protobuf)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Protobuf DEFAULT_MSG PROTOBUF_LIBRARY)