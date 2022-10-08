cmake_minimum_required(VERSION 3.0)

PROJECT (fusion_bussiness)

include_directories(
    infrastructure/tcp
    infrastructure/common
    domain/data_association/KM_algorithm
)


aux_source_directory(${PROJECT_SOURCE_DIR}/domain/data_association/KM_algorithm FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/tcp FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/kalman_filter FUSION_FILES)
#aux_source_directory(${PROJECT_SOURCE_DIR}/domain/data_fusion/source FUSION_FILES)
#aux_source_directory(${PROJECT_SOURCE_DIR}/domain/proxy/source FUSION_FILES)
#aux_source_directory(${PROJECT_SOURCE_DIR}/application/source FUSION_FILES)


add_library(${PROJECT_NAME} STATIC ${FUSION_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDES} ${GLOG_INCLUDES})
target_link_libraries(${PROJECT_NAME} -lpthread -lgcov)




