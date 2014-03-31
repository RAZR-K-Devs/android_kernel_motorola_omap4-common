#!/bin/bash
set -m

# built kernel & modules
echo "Building kernel and modules..."
echo " "
export PATH=/data/4.4/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-
make mapphone_OCE_defconfig
export LOCALVERSION="-JBX-3.0-Hybrid-Razr-4.4"
make
echo " "

echo "------------- "
echo "Done building"
echo "------------- "
echo " "
