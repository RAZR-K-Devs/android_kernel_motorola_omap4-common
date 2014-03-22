#!/bin/sh
export PATH=/data/4.4/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-
make mapphone_OCE_defconfig
echo "Obtaining your amount of physical CPU cores..."; sleep 2
		cores=$(nproc)
		echo "Your CPU has "$cores" cores"
		echo
		core=$(($cores * 2))
make -j$core
