#!/bin/sh

mkdir -p build/kernels

# build <title> <what> <comms> <mem> [<for>]
build() {
    echo
    echo "\033[32mCompiling $1... please wait!\033[0m"
    echo "\033[32mCleaning up first...\033[0m"
    make clean BUILDWHAT=$2 BUILDCOMMS=$3 BUILDMEM=$4 BUILDFOR=$5
    echo "\033[32mCompiling...\033[0m"
    make BUILDWHAT=$2 BUILDCOMMS=$3 BUILDMEM=$4 BUILDFOR=$5
    echo "\033[32mCompile $1 finished!\033[0m"
}

echo
echo "\033[1;36m***********************************\033[0m"
echo "\033[1;36m* Compiling kernel binaries\033[0m"
echo "\033[1;36m***********************************\033[0m"

# Flash kernels (lean, no EEPROM code). SH7055 kernels carry both the 0.35um and the 0.18um flash driver and pick
# the right one at run time (pl_flash_7055_auto.c), so there are no separate 180nm builds.
build "SH7055 K-Line kernel"              SH7055         KLINE  FLASH
build "SH7055 Denso CAN kernel"           SH7055         CAN    FLASH
build "SH7058 K-Line kernel"              SH7058         KLINE  FLASH
build "SH7058 Denso CAN kernel"           SH7058         CAN    FLASH
build "SH7058 Denso CAN-TP kernel"        SH7058         CAN_TP FLASH
build "SH7058 Diesel CAN-TP kernel"       SH7058D_EURO4  CAN_TP FLASH
build "SH7059 Diesel CAN-TP kernel"       SH7059D_EURO5  CAN_TP FLASH
build "SH7059 Diesel CAN kernel"          SH7059D_EURO5  CAN    FLASH
# Denso SH7055 TCU (0.35um): ISO15765 like the CAN-TP kernels, image at 0xFFFF9000 (the TCU bootloader's fixed 8KB window), .bss elsewhere
build "SH7055 Denso CAN-TP TCU kernel"     SH7055         CAN_TP FLASH TCU

# EEPROM kernels: the flash kernel plus the serial-EEPROM commands, built as ssmk_<comms>_<what>_eeprom.bin
build "SH7055 K-Line EEPROM kernel"       SH7055         KLINE  EEPROM
build "SH7058 K-Line EEPROM kernel"       SH7058         KLINE  EEPROM
build "SH7058 Denso CAN-TP EEPROM kernel" SH7058         CAN_TP EEPROM
