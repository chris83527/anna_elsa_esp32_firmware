cmake_minimum_required(VERSION 3.16.0)

set(EXTRA_COMPONENT_DIRS 
"components/i2c_manager"
"components/cctalk"
"components/anna_elsa_audio_board" 
"components/ht16k33" 
"components/m20ly02z" 
"components/ssd1306"
"components/mcp23x17" 
"components/mcp23008" 
"components/ds3231" 
"components/esp32-wifi-manager" 
"/home/chris/esp/esp-idf/components/esp_hw_support" 
"/home/chris/esp/esp-idf/components/nvs_flash" 
"/home/chris/esp/esp-idf/components/esp_common" 
"/home/chris/esp/esp-idf/components/spiffs" 
"/home/chris/esp/esp-idf-lib/components/color" 
"/home/chris/esp/esp-idf-lib/components/lib8tion" 
"/home/chris/esp/esp-idf-lib/components/led_strip" 
"/home/chris/esp/esp-idf-lib/components/esp_idf_lib_helpers" 
"/home/chris/esp/esp-adf/components")


include(/home/chris/esp/esp-idf/tools/cmake/project.cmake)

project(anna_elsa_esp32)
