# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.
cmake_minimum_required(VERSION 3.16)

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.c*)

idf_component_register(SRCS ${app_sources})


# Create a SPIFFS image from the contents of the 'spiffs_image' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash'.

spiffs_create_partition_image(audio ../audio FLASH_IN_PROJECT)
littlefs_create_partition_image(httpd ../httpd)
