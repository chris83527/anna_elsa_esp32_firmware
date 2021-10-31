
cmake_minimum_required(VERSION 3.16.0)


#set(EXTRA_COMPONENT_DIRS components/led_strip components/protocol_common /home/chris/esp/esp-idf-lib/components components/cctalk)
set(EXTRA_COMPONENT_DIRS "components/cctalk"
"/home/chris/esp/esp-idf/components/esp_hw_support"
"/home/chris/esp/esp-idf/components/spiffs"
"/home/chris/esp/esp-idf-lib/components/color"
"/home/chris/esp/esp-idf-lib/components/lib8tion"
"/home/chris/esp/esp-idf-lib/components/i2cdev"
"/home/chris/esp/esp-idf-lib/components/protocol_common"
"/home/chris/esp/esp-idf-lib/components/ds3231"
"/home/chris/esp/esp-idf-lib/components/mcp23008"
"/home/chris/esp/esp-idf-lib/components/mcp23x17"
"/home/chris/esp/esp-idf-lib/components/led_strip"
"/home/chris/esp/esp-adf/components")


include(/home/chris/esp/esp-idf/tools/cmake/project.cmake)

project(anna_elsa_esp32)
