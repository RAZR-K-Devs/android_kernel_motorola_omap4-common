#!/bin/bash
set -m

# Sync ?
cd /home/dtrail/android/4.4
while true; do
    read -p "Do you wish to sync repo? " yn
    case $yn in
        [Yy]* ) echo "Syncing repo..."; echo " "; repo sync; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo " "

# Exporting changelog to file
cd /home/dtrail/android/4.4
while true; do
    read -p "Do you wto clean build dirs? " yn
    case $yn in
        [Yy]* ) echo "Cleaning out kernel source directory..."; make mrproper; make ARCH=arm distclean; cd ~/android/android_kernel_motorola_omap4-common; make mrproper; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo " "

# We build the kernel and its modules first
# Launch execute script in background
# First get tags in shell
cd /home/dtrail/android/4.4
export USE_CCACHE=1
source build/envsetup.sh
export PATH=${PATH/\/path\/to\/jdk\/dir:/}
lunch cm_spyder-userdebug

# built kernel & modules
echo "Building kernel and modules..."
echo " "

# export PATH=/home/dtrail/android/4.4/prebuilt/linux-x86/toolchain/arm-unknown-linux-gnueabi-standard_4.7.2/bin:$PATH
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-

# export TARGET_KERNEL_CUSTOM_TOOLCHAIN=arm-unknown-linux-gnueabi-standard_4.7.2
export LOCALVERSION="-JBX-3.0-Hybrid-Atrix2-4.4"
export BOARD_HAS_SDCARD_INTERNAL=false
make -j8 TARGET_KERNEL_SOURCE=/home/dtrail/android/android_kernel_motorola_omap4-common/ TARGET_KERNEL_CONFIG=mapphone_OCEdison_defconfig $OUT/boot.img
echo " "

# Build libhealthd.omap4
while true; do
    read -p "Do you wish to include 10% battery meter? " yn
    case $yn in
        [Yy]* ) echo "Moving Ramdisk into built path..."; echo " "; cp /home/dtrail/android/4.4/out/target/product/spyder/ramdisk.img /home/dtrail/android/built/4.4/3.0/edison/rls/jbx/Applications/ramdisk/; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo " "

# We don't use the kernel but the modules
echo "Copying modules to package folder"
echo " "
cp -r /home/dtrail/android/4.4/out/target/product/spyder/system/lib/modules/* /home/dtrail/android/built/4.4/3.0/edison/rls/system/lib/modules/
cp /home/dtrail/android/4.4/out/target/product/spyder/kernel /home/dtrail/android/built/4.4/3.0/edison/rls/system/etc/kexec/

echo "------------- "
echo "Done building"
echo "------------- "
echo " "

# Copy and rename the zImage into nightly/nightly package folder
# Keep in mind that we assume that the modules were already built and are in place
# So we just copy and rename, then pack to zip including the date
echo "Packaging flashable Zip file..."
echo " "

cd /home/dtrail/android/built/4.4/3.0/edison/rls
zip -r "JBX-Kernel-3.0-Hybrid-Edison-4.4_$(date +"%Y-%m-%d").zip" *
mv "JBX-Kernel-3.0-Hybrid-Edison-4.4_$(date +"%Y-%m-%d").zip" /home/dtrail/android/out

# Exporting changelog to file
cd /home/dtrail/android/android_kernel_motorola_omap4-common
while true; do
    read -p "Do you wish to push the latest changelog? " yn
    case $yn in
        [Yy]* ) echo "Exporting changelog to file: '/built/Changelog-[date]'"; echo " "; git log --oneline --since="4 day ago" > /home/dtrail/android/android_kernel_motorola_omap4-common/changelog/Changelog_$(date +"%Y-%m-%d"); git log --oneline  > /home/dtrail/android/android_kernel_motorola_omap4-common/changelog/Full_History_Changelog; git add changelog/ .; git commit -m "Added todays changelog and updated full history"; git push origin JBX_30X; echo " "; echo "done"; break;;
        [Nn]* ) exit;;
        * ) echo "Please answer yes or no.";;
    esac
done
