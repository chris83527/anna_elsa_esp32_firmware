idf_component_register(
    SRCS mcp23x17.cpp
    INCLUDE_DIRS .
    REQUIRES driver i2c_manager log
)
