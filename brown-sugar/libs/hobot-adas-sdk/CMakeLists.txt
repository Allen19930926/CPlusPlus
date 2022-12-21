# cmake version
cmake_minimum_required(VERSION 3.15)

add_library(hobot-adas-sdk SHARED
    # src/data_receiver.cc
    src/hobot-adas-sdk.cc)
find_package(Threads)
target_link_dep_libraries(hobot-adas-sdk PRIVATE  ${COMMUNICATION_LIBRARIES} hobot-auto-protocol ${ZEROMQ_LIBRARY} ${PROTOBUF_LIBRARY} ${PACKSDK_LIBRARIES} ${OPENCV_LIBRARYIES} hlog-static fmt)
target_link_libraries(hobot-adas-sdk PRIVATE ${CMAKE_DL_LIBS} Threads::Threads)
target_link_libraries(hobot-adas-sdk PUBLIC common)
target_include_directories(hobot-adas-sdk PUBLIC include)
target_include_directories(hobot-adas-sdk PRIVATE include/hobot-adas-sdk)
target_include_directories(hobot-adas-sdk PRIVATE src)
include(GenerateExportHeader)
generate_export_header(hobot-adas-sdk EXPORT_FILE_NAME hobot-adas-sdk/export.h)
target_include_directories(hobot-adas-sdk PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
install(TARGETS hobot-adas-sdk)
