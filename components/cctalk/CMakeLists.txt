idf_component_register(SRCS ./coin_hopper_device.cpp ./coin_acceptor_device.cpp ./cctalk_link_controller.cpp ./cctalk_device.cpp ./serial_worker.cpp INCLUDE_DIRS . REQUIRES freertos driver pthread esp_timer)



