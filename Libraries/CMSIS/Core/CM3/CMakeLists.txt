target_include_directories ( ${PROJECT_NAME} PUBLIC .    )
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core_cm3.c )
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/system_stm32f10x.c )


