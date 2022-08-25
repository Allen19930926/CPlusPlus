cmake_minimum_required(VERSION 3.15)
add_subdirectory(common)
add_subdirectory(hobot-adas-sdk)

IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
add_subdirectory(ipc-shm)
ENDIF()
