# cmake version
cmake_minimum_required(VERSION 3.0)

PROJECT (hobot-communication)

include_directories(
	${PROJECT_SOURCE_DIR}/../../infrastructure/common
)

aux_source_directory(${PROJECT_SOURCE_DIR} HOBOT_COMMUNICATION_FILES)

add_library(${PROJECT_NAME} STATIC ${HOBOT_COMMUNICATION_FILES})
find_package(Threads)
target_link_dep_libraries(${PROJECT_NAME} PRIVATE ${COMMUNICATION_LIBRARIES} ${PACKSDK_LIBRARIES} hobot-auto-protocol ${PROTOBUF_LIBRARY} ${OPENCV_LIBRARIES} ${TURBOJPEG_LIBRARIES} hlog-static fmt)
target_link_libraries(${PROJECT_NAME} PRIVATE common ${CMAKE_DL_LIBS} Threads::Threads)

