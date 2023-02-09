
cmake_minimum_required(VERSION 3.16.0)


#set(EXTRA_COMPONENT_DIRS components/led_strip components/protocol_common /home/chris/esp/esp-idf-lib/components components/cctalk)
set(EXTRA_COMPONENT_DIRS 
"components/cctalk" 
"components/anna_elsa_audio_board" 
"components/ht16k33" 
"components/m20ly02z" 
"components/ssd1306"
"components/joltwallet__littlefs" 
"/home/chris/esp/esp-idf-v4.4/components/esp_hw_support" 
"/home/chris/esp/esp-idf-v4.4/components/esp_https_server" 
"/home/chris/esp/esp-idf-v4.4/components/esp_wifi" 
"/home/chris/esp/esp-idf-v4.4/components/nvs_flash" 
"/home/chris/esp/esp-idf-v4.4/components/esp_common" 
"/home/chris/esp/esp-idf-v4.4/components/spiffs" 
"/home/chris/esp/esp-idf-lib/components/color" 
"/home/chris/esp/esp-idf-lib/components/lib8tion" 
"/home/chris/esp/esp-idf-lib/components/i2cdev" 
"/home/chris/esp/esp-idf-lib/components/ds3231" 
"/home/chris/esp/esp-idf-lib/components/mcp23008" 
"/home/chris/esp/esp-idf-lib/components/mcp23x17" 
"/home/chris/esp/esp-idf-lib/components/lib8tion" 
"/home/chris/esp/esp-idf-lib/components/esp_idf_lib_helpers" 
"/home/chris/esp/esp-idf-lib/components/led_strip" 
"/home/chris/esp/esp-adf/components")


include(/home/chris/esp/esp-idf-v4.4/tools/cmake/project.cmake)

project(anna_elsa_esp32)
