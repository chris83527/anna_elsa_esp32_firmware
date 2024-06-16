#!/bin/sh

export IDF_PATH=/home/chris/esp/esp-idf-v5.2.2

. /home/chris/esp/esp-idf-v5.2.2/export.sh

idf.py build

cd build;
python -m esptool --chip esp32 merge_bin --fill-flash-size 16MB -o flash_merged.bin  @flash_args
cd ..

qemu-system-xtensa -s -S -nographic -machine esp32 -drive file=build/flash_merged.bin,if=mtd,format=raw -serial tcp::5555,server,nowait
