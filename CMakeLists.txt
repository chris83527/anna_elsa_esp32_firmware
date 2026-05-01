cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0111 NEW)

set(EXTRA_COMPONENT_DIRS 
"components/i2cmanager"
"components/cctalk"
"components/ht16k33" 
"components/m20ly02z" 
"components/ssd1306"
"components/mcp23x17" 
"components/mcp23008" 
"components/ds3231" 
"components/esp_littlefs"
"components/ws2812b"
"/home/chris/esp/esp-idf/components/esp_hw_support"
"/home/chris/esp/esp-idf/components/nvs_flash"
"/home/chris/esp/esp-idf/components/esp_common"
"/home/chris/esp/esp-idf/components/spiffs"
"/home/chris/esp/esp-idf-lib/components/esp_idf_lib_helpers"
)
#"/home/chris/esp/esp-idf/components/esp_http_server"
include($ENV{IDF_PATH}/tools/cmake/project.cmake)


project(anna_elsa_esp32)
