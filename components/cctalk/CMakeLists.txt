
idf_component_register(
        SRCS
        "cctalk.cpp"
        "esp_idf_cctalk_uart.cpp"
        "events/cctalk_event_queue.cpp"
        "coin_acceptor/coin_acceptor.cpp"
        "hopper/hopper.cpp"
        INCLUDE_DIRS
        "."
        "coin_acceptor"
        "hopper"
        "events"
        REQUIRES
        driver
        freertos
)