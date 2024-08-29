#!/bin/sh

mkdir -p build/kernels

echo
echo "\033[1;36m***********************************\033[0m"
echo "\033[1;36m* Compiling kernel binaries\033[0m"
echo "\033[1;36m***********************************\033[0m"
echo
echo "\033[32mCompiling SH7055 K-Line kernel... please wait!\033[0m"
echo "\033[32mCleaning up first...\033[0m"
make clean BUILDWHAT=SH7055 BUILDCOMMS=KLINE BUILDMEM=FLASH
echo "\033[32mCompiling...\033[0m"
make BUILDWHAT=SH7055 BUILDCOMMS=KLINE BUILDMEM=FLASH
echo "\033[32mCompile SH7055 K-Line kernel finished!\033[0m"

echo
echo "\033[32mCompiling SH7055 Denso CAN kernel... please wait!\033[0m"
echo "\033[32mCleaning up first...\033[0m"
make clean BUILDWHAT=SH7055 BUILDCOMMS=CAN BUILDMEM=FLASH
echo "\033[32mCompiling...\033[0m"
make BUILDWHAT=SH7055 BUILDCOMMS=CAN BUILDMEM=FLASH
echo "\033[32mCompile SH7055 Denso CAN kernel finished!\033[0m"

echo
echo "\033[32mCompiling SH7058 K-Line kernel... please wait!\033[0m"
echo "\033[32mCleaning up first...\033[0m"
make clean BUILDWHAT=SH7058 BUILDCOMMS=KLINE BUILDMEM=FLASH
echo "\033[32mCompiling...\033[0m"
make BUILDWHAT=SH7058 BUILDCOMMS=KLINE BUILDMEM=FLASH
echo "\033[32mCompile SH7058 K-Line kernel finished!\033[0m"

echo
echo "\033[32mCompiling SH7058 Denso CAN kernel... please wait!\033[0m"
echo "\033[32mCleaning up first...\033[0m"
make clean BUILDWHAT=SH7058 BUILDCOMMS=CAN BUILDMEM=FLASH
echo "\033[32mCompiling...\033[0m"
make BUILDWHAT=SH7058 BUILDCOMMS=CAN BUILDMEM=FLASH
echo "\033[32mCompile SH7058 Denso CAN kernel finished!\033[0m"

echo
echo "\033[32mCompiling SH7058 Denso CAN-TP kernel... please wait!\033[0m"
echo "\033[32mCleaning up first...\033[0m"
make clean BUILDWHAT=SH7058 BUILDCOMMS=CAN_TP BUILDMEM=FLASH
echo "\033[32mCompiling...\033[0m"
make BUILDWHAT=SH7058 BUILDCOMMS=CAN_TP BUILDMEM=FLASH
echo "\033[32mCompile SH7058 CAN-TP kernel finished!\033[0m"

echo
echo "\033[32mCompiling SH7058 Diesel CAN-TP kernel... please wait!\033[0m"
echo "\033[32mCleaning up first...\033[0m"
make clean BUILDWHAT=SH7058D_EURO4 BUILDCOMMS=CAN_TP BUILDMEM=FLASH
echo "\033[32mCompiling...\033[0m"
make BUILDWHAT=SH7058D_EURO4 BUILDCOMMS=CAN_TP BUILDMEM=FLASH
echo "\033[32mCompile SH7058 Diesel CAN-TP kernel finished!\033[0m"

echo
echo "\033[32mCompiling SH7059 Diesel CAN-TP kernel... please wait!\033[0m"
echo "\033[32mCleaning up first...\033[0m"
make clean BUILDWHAT=SH7059D_EURO5 BUILDCOMMS=CAN_TP BUILDMEM=FLASH
echo "\033[32mCompiling...\033[0m"
make BUILDWHAT=SH7059D_EURO5 BUILDCOMMS=CAN_TP BUILDMEM=FLASH
echo "\033[32mCompile SH7059 Diesel CAN-TP kernel finished!\033[0m"
