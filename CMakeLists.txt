
cmake_minimum_required(VERSION 3.16.0)

set(EXTRA_COMPONENT_DIRS components/led_strip components/protocol_common components/esp32-ds3231 components/i2cdev components/mcp23x17 components/cctalk)


include(/home/chris/esp/esp-idf/tools/cmake/project.cmake)

project(anna_elsa_esp32)
