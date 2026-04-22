
idf_component_register(
        SRCS
        "cctalk.cpp"
        "events/cctalk_event_queue.cpp"
        INCLUDE_DIRS
        "."
        "coin_acceptor"
        "bill_validator"
        "hopper"
        "events"
        REQUIRES
        driver
        freertos
)