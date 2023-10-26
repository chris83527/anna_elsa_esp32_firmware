cmake_minimum_required(VERSION 3.16.0)

set(EXTRA_COMPONENT_DIRS 
"components/i2c_manager"
"components/cctalk"
"components/server" 
"components/anna_elsa_audio_board" 
"components/ht16k33" 
"components/m20ly02z" 
"components/ssd1306"
"components/joltwallet__littlefs" 
"/home/chris/esp/esp-idf-v4.4.6/components/esp_hw_support" 
"/home/chris/esp/esp-idf-v4.4.6/components/esp_https_server" 
"/home/chris/esp/esp-idf-v4.4.6/components/esp_wifi" 
"/home/chris/esp/esp-idf-v4.4.6/components/nvs_flash" 
"/home/chris/esp/esp-idf-v4.4.6/components/esp_common" 
"/home/chris/esp/esp-idf-v4.4.6/components/spiffs" 
"/home/chris/esp/esp-idf-lib/components/color" 
"/home/chris/esp/esp-idf-lib/components/lib8tion" 
"/home/chris/esp/esp-idf-lib/components/ds3231" 
"/home/chris/esp/esp-idf-lib/components/mcp23008" 
"/home/chris/esp/esp-idf-lib/components/mcp23x17" 
"/home/chris/esp/esp-idf-lib/components/lib8tion" 
"/home/chris/esp/esp-idf-lib/components/i2cdev" 
"/home/chris/esp/esp-idf-lib/components/esp_idf_lib_helpers" 
"/home/chris/esp/esp-idf-lib/components/led_strip" 
"/home/chris/esp/esp-adf/components")


include(/home/chris/esp/esp-idf-v4.4.6/tools/cmake/project.cmake)

project(anna_elsa_esp32)
