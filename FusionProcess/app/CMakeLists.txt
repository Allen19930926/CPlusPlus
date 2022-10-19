cmake_minimum_required(VERSION 3.0)

PROJECT (fusion_bussiness)

include_directories(
    application/
    domain/data_association
    domain/track_prediction
    domain/track_update
    infrastructure/common
    infrastructure/sensor_object
    infrastructure/track_object
    infrastructure/kalman_filter
    ${GTEST_INCLUDES}
)


aux_source_directory(${PROJECT_SOURCE_DIR}/application FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/domain/data_association FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/domain/track_update FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/kalman_filter FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/sensor_object FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/track_object FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/kalman_filter FUSION_FILES)

if (NOT (FT_TEST EQUAL 2))
    include_directories(infrastructure/tcp)
    aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/tcp FUSION_FILES)
endif()

add_library(${PROJECT_NAME} STATIC ${FUSION_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDES} ${GLOG_INCLUDES})
target_link_libraries(${PROJECT_NAME} -lpthread -lgcov)




