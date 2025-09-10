cmake_minimum_required(VERSION 3.16)

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
"components/esp_ws28xx"
"/home/chris/esp/esp-idf/components/esp_hw_support"
"/home/chris/esp/esp-idf/components/nvs_flash"
"/home/chris/esp/esp-idf/components/esp_common"
"/home/chris/esp/esp-idf/components/spiffs"
"/home/chris/esp/esp-idf-lib/components/lib8tion"
"/home/chris/esp/esp-idf-lib/components/esp_idf_lib_helpers"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(anna_elsa_esp32)
