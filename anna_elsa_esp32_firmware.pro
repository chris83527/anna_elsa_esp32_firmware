# Created by and for Qt Creator This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

#TARGET = anna_elsa_esp32_firmware

HEADERS = \
   $$PWD/build/bootloader/config/sdkconfig.h \
   $$PWD/build/config/sdkconfig.h \
   $$PWD/components/cctalk/include/cctalk.h \
   $$PWD/components/cctalk/include/cctalk_constants.h \
   $$PWD/components/cctalk/include/cctalk_response.h \
   $$PWD/components/esp_idf_lib_helpers/esp_idf_lib_helpers.h \
   $$PWD/components/ht16k33/include/ht16k33.h \
   $$PWD/components/m20ly02z/include/m20ly02z.h \
   $$PWD/components/mcp23x17/mcp23x17.h \
   $$PWD/components/protocol_common/include/addr_from_stdin.h \
   $$PWD/components/protocol_common/include/protocol_common.h \
   $$PWD/config/sdkconfig.h \
   $$PWD/main/config.h \
   $$PWD/main/spiffs.h \
   $$PWD/main/tableau.h \
   $$PWD/main/wave.h \
   $$PWD/main/webserver.h

SOURCES = \
   $$PWD/build/bootloader/CMakeFiles/3.16.3/CompilerIdC/CMakeCCompilerId.c \
   $$PWD/build/bootloader/CMakeFiles/3.16.3/CompilerIdCXX/CMakeCXXCompilerId.cpp \
   $$PWD/build/bootloader/project_elf_src_esp32.c \
   $$PWD/build/CMakeFiles/3.16.3/CompilerIdC/CMakeCCompilerId.c \
   $$PWD/build/CMakeFiles/3.16.3/CompilerIdCXX/CMakeCXXCompilerId.cpp \
   $$PWD/build/project_elf_src_esp32.c \
   $$PWD/components/cctalk/cctalk.c \
   $$PWD/components/cctalk/cctalk_response.c \
   $$PWD/components/ht16k33/ht16k33.c \
   $$PWD/components/m20ly02z/m20ly02z.c \
   $$PWD/components/mcp23x17/mcp23x17.c \
   $$PWD/components/protocol_common/addr_from_stdin.c \
   $$PWD/components/protocol_common/connect.c \
   $$PWD/components/protocol_common/stdin_out.c \
   $$PWD/main/main.c \
   $$PWD/main/spiffs.c \
   $$PWD/main/tableau.c \
   $$PWD/main/wave.c \
   $$PWD/main/webserver.c

INCLUDEPATH = \
    $$PWD/build/bootloader/config \
    $$PWD/build/config \
    $$PWD/components/cctalk/include \
    $$PWD/components/esp_idf_lib_helpers \
    $$PWD/components/ht16k33/include \
    $$PWD/components/m20ly02z/include \
    $$PWD/components/mcp23x17 \
    $$PWD/components/protocol_common/include \
    $$PWD/config \
    $$PWD/main

#DEFINES = 

