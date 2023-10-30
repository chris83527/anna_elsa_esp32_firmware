idf_component_register(
    SRCS ds3231.cpp
    INCLUDE_DIRS .
    REQUIRES log
    PRIV_REQUIRES i2c_manager
)
