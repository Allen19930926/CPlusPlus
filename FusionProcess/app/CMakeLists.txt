cmake_minimum_required(VERSION 3.0)

PROJECT (FUSION_BUSINESS)

include_directories(
    infrastructure/tcp
    infrastructure/common
    infrastructure/KM_algorithm
)


aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/KM_algorithm FUSION_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/infrastructure/tcp FUSION_FILES)
#aux_source_directory(${PROJECT_SOURCE_DIR}/domain/data_fusion/source FUSION_FILES)
#aux_source_directory(${PROJECT_SOURCE_DIR}/domain/proxy/source FUSION_FILES)
#aux_source_directory(${PROJECT_SOURCE_DIR}/application/source FUSION_FILES)


add_library(${PROJECT_NAME} STATIC ${FUSION_FILES})
target_link_libraries(${PROJECT_NAME} -lpthread -lgcov)




