target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hw_config.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/main.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/stm32f10x_it.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/stm32f1_delay.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/AT45DB321D.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Calibration.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/usb_desc.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/usb_endp.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/usb_istr.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/usb_prop.c)
target_sources( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/usb_pwr.c)


