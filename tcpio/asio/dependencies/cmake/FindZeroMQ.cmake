cmake_minimum_required(VERSION 3.5)

if (MSVC)
    set(ZEROMQ_LIBRARY libzmq)
else()
    set(ZEROMQ_LIBRARY zmq)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZeroMQ DEFAULT_MSG ZEROMQ_LIBRARY)