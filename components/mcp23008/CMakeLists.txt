idf_component_register(
    SRCS mcp23008.cpp
    INCLUDE_DIRS .
    REQUIRES driver log
    PRIV_REQUIRES i2c_manager
)
