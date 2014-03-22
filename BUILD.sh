#!/bin/bash
######################################
#	   KERNEL BUILDER	     #
######################################
#				     #
#(c)2014 dtrail <mnl.manz@gmail.com> #
######################################
# 
# Prepare compiler
#	
#
# Define variable
#
	# placeholder for check() and adjust() functions
	# After detection/user input the PATHs will be stored here
	declare cm_dir
	declare kern_dir
	declare toolchain
	declare built_dir
	declare out_dir	
	
	# Return values of PATH detection in check() function
	# In case the sources cannot be found in default place or on local hdd
	# they will return 'false' - which starts the user_check()
	declare kern_check=true
	declare cm_check=true
	declare arm_check=true
	declare builtdir_check=true
	declare outdir_check=true
	
	# CPU Cores
	declare core
	
function config_read {

	# Default PATHs
	echo "Reading config...." >&2
	source config.conf
	echo "Kernel Source folder	:  $kerndir" >&2
	echo "CM Source folder		:  $cmdir" >&2
	echo "Toolchain path		:  $arm_path" >&2
	echo "Built folder		:  $builtdir" >&2
	echo "Out folder		:  $outdir" >&2
}
echo " "
echo "Welcome to the JBX-KERNEL-BUILDER!"
echo " "
######################################
#	     Main Menu		     #
######################################
function main {
		# clear the screen here
		clear;
	echo "######################################"
	echo "#		    Main Menu		   #"
	echo "######################################"
	echo " "
	echo " "
	echo "[1] Check your environemnt"
	echo " "
	echo " "
	echo " "
	echo "[2]"
	echo "This option is useful to see quickly if the kernel gets compiled after commited any changes."
	echo "This is recommended to test new code, otherwise a full build takes more time and when it gets"
	echo "interrupted by any compiler error or something, it's hard to see the actual error message."
	echo "After the test build is successfully compiled, run a full build."
	echo " "
	echo "[3]"
	echo "This will build a full installer package (Kernel, modules, Aroma Installer)"
	echo "and place it in the specified out dir - Ready to flash!"
	echo " "
	echo "[4]"
	echo "You might want to clean the kernel source directoy and start over from scratch."
	echo "can be useful if you got too many errors while compiling, and fixed them one by one,"
	echo "in some cases the kernel will compile - but after cleaned out the source dir"
	echo "some new errors might appear. To sum it up: If you commited a lot of changes"
	echo "to the kernel, it's always better to clean out the build dirs"
	echo
	echo "[5]"
	echo "Do I really have to explain this option? :-D"
	echo "	BUT: You might want to enter your local PATHs in the config.conf file."
	echo "	The Builder checks it and checks also if these paths are true."
	echo " 	If any of them doesn't exist, the Builder lets you enter the correct PATHs,"
	echo "	but they WILL NOT be saved (I have problems to make this work - for now)"
	echo "  You can just exit, etner your PATHs and then restart the Builder."
	echo " "
	echo " "
	echo " "
		# Choose option
		while true; do
		    read -p "Select: [1] Check environment [2]Compile Test [3]Build full package [4]Clean build dirs [5]Exit : " 12345
		    case $12345 in
			echo
			[1]* ) check;;
		        [2]* ) testb;;
		        [3]* ) full;;
			[4]* ) make mrproper; make clean; main;;
			[5]* ) exit;;
		        * ) echo "Enter a number from 1-4 ";;
		    esac
		done
}

	
	
	
#### We start here

#######################
## Declare functions ##
#######################

##
## Clear screen: Usage 'clear;'
##############################
function clear {
	local CLEAR=''

	type -p clear >/dev/null && \
		CLEAR=$(exec clear)
	[[ -z $CLEAR ]] && type -p tput >/dev/null && \
		CLEAR=$(exec tput clear)
	[[ -z $CLEAR ]] && \
		CLEAR=$'\e[H\e[2J'

	echo -en "$CLEAR"

	eval "function clear { echo -en '$CLEAR'; }"
}

function check {

	# Read from config file
	config_read;
	# check if kernel source exist in default path
	if [ -d $kerndir ]; 
	  then 
		echo "Kernel source found!";
		sleep 1;
		kern_dir=$kerndir
	
        else
		echo "Kernel source NOT found!";
		sleep 1;
		kern_check=false
		user_check;
	fi


	# check if CM source exist in default path
	if [ -d $cmdir ];
	  then 
		echo "CM source found!";
		echo "Toolchain path set!";
		sleep 1;
		cm_dir=$cmdir
		toolchain=$arm_path/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin

        else
		echo "CM source NOT found!";
		sleep 1;
		cm_check=false
		user_check;
	fi

	# check if built folder exist in default path
	if [ -d $builtdir ];
	  then 
		echo "built folder found!";
		sleep 1;
		built_dir=$builtdir

        else
		echo "built folder NOT found!";
		sleep 1;
		builtdir_check=false
		user_check;
	fi

	# check if out folder exist in default path
	if [ -d $outdir ];
	  then 
		echo "toolchain found!";
		sleep 1;
		out_dir=$outdir

        else
		echo "toolchain NOT found!";
		sleep 1;
		outdir_check=false
		user_check;
	fi


	
	# Now show the result
	echo " "
	echo "##############"
	echo "KERNEL SOURCE: $kern_dir"
	echo "CM SOURCE    : $cm_dir"
	echo "Toolchain    : $toolchain"
	echo "##############"
	
	sleep 1
	store;
}

# Check for directories
function user_check {

	# Clear screen
	clear;

	echo "Hello, "$USER".  Please enter your EXACT paths to your local sources here, including slashes,etc."

	# Check return value (Kernel)
	if [ $kern_check=false ];

	  then
		echo -n "Enter Kernel source PATH and press [ENTER]: "
		read kerneldir

	# Check return value (CM)
	if [ $cm_check=false ];	

	  then
		echo -n "Enter CM source PATH and press [ENTER]: "
		read cmsource
	echo

	# Check return value (built)
	if [ $builtdir_check=false ];	

	  then
		echo -n "Enter built PATH and press [ENTER]: "
		read builtdirpath
	echo

	# Check return value (out)
	if [ $outdir_check=false ];	

	  then
		echo -n "Enter OUT PATH and press [ENTER]: "
		read outpath
		mkdir outpath
		echo "Out folder created at $outpath"
	echo

	echo " "
	echo "Your Kernel source is at  : $kerneldir"
	echo " "
	echo "Your CM source is at      : $cmsource"
	echo " "
	echo "Your toolchain is at 	: $cmsource/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin"
	echo " "
	echo "Your built folder is at 	: $builtdirpath"
	echo " "
	echo "Your OUT folder is at 	: $outpath"
	echo " "
	while true; do
	    read -p "Correct? Press [y], if not correct press [n] to retry : " yn
	    case $yn in
	        [Yy]* ) store;;
	        [Nn]* ) user_check;;
	        * ) echo "Enter 'y' or 'n' ";;
	    esac
	done
}



##
## Do we have all needed stuff in its places?
#############################################
store()
{
	kern_dir=$kerneldir
	cm_dir=$cmsource
	toolchain=$cmsource/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin
	built_dir=$builtdirpath
	out_dir=$outpath; 
	kern_check=true
	cm_check=true
	arm_check=true
	builtdir_check=true
	outdir_check=true
	sleep 1
	echo " "
	echo $kern_check
	echo $cm_check
	echo $arm_check
	echo $builtdir_check
	echo $outdir_check
	sleep 1
	adjust;
}

#
# Show stored paths in variables
#
adjust()
{
	echo " "
	echo "Well, everything should be in the right place:"
	echo " "
	echo $kern_dir
	echo $cm_dir
	echo $toolchain
	echo $built_dir
	echo $out_dir
	echo " "
	echo " "
	echo "Starting main menu..."
	sleep 3
	main;
}



#
# Core selection (building speed)
#
core_select()
{
		echo "Obtaining your amount of physical CPU cores..."; sleep 2
		cores=$(nproc)
		echo "Your CPU has "$cores" cores"
		echo
		core=$(($cores * 2))
}

##########################################
# preparation done! Continue with script #
##########################################

######################################
#	     TEST - BUILD	     #
######################################
function testb {
	# We need your actual CPU power
	core_select;
	# clear the screen here
	clear
	# Prepare the environment
	echo "Preparing environment..."
	echo
	echo "export ARCH=arm"
	export ARCH=arm
	sleep 1
	echo "export SUBARCH=arm"
	export SUBARCH=arm
	sleep 1
	echo "export PATH=$toolchain:$PATH"
	export PATH=$toolchain:$PATH
	sleep 1
	echo "export CROSS_COMPILE=arm-eabi-"
	export CROSS_COMPILE=arm-eabi-
	sleep 1
	echo
	echo "Now we are prepared! Choose your device you want to treat with JBX-Kernel."
	echo "NOTE: This will build ONLY the kernel! No modules, no ramdisk, no installer package!"
	echo " "
	# Select device
	while true; do
	    read -p "Select device: [1]Spyder [2]Edison [3]Targa [4]Abort : " 1234
	    case $1234 in
	        [1]* ) make mapphone_OCE_defconfig; make -j$core; echo "Test Kernel built!"; echo " "; sleep 1; errchk;;
	        [2]* ) make mapphone_OCEdison_defconfig; make -j$core; echo "Test Kernel built!"; echo " "; sleep 1; errchk;;
		[3]* ) make mapphone_OCTarga_defconfig; make -j$core; echo "Test Kernel built!"; echo " "; sleep 1; errchk;;
		[4]* ) break;;
	        * ) echo "Enter a number from 1-4 ";;
	    esac
	done
}


function errchk {

	echo
	echo "Congrats! The kernel has been built! --Or not???."
	echo "Usually we use the most recent amount of CPU cores x2 to compile."
	echo "This often leads to the compiler doesn't show the speficic error."
	echo "A simple 'make' without any flags will show what's actually wrong."
	echo " "
	echo "Do you want to send a simple 'make' command to see what's wrong? Press 'y'"
	echo "Otherwise, if the kernel has been built successfully, press 'n'"
	echo " "
	echo "NOTE: When pressing 'y' this meanu will appear again,"
	echo "	    so yoo have time to fix the error and try again and again -"
	echo "	    just until the compile succeeds. "
	echo "	    Keep in mind to clean the build dirs and repeat the test compile"
	echo " 	    - Just to be safe! ;-)"
	echo ""
	echo "	    To leave this menu, press 'n'"
	# Select device
	while true; do
	    read -p "Do you want to send a simple 'make' command to see what's wrong? : " yn
	    case $1234 in
	        [Yy]* ) echo "Build again with 'make' only..."; make; errchk;;
	        [Nn]* ) main;;
	        * ) echo "Enter 'y' or 'n' ";;
	    esac
	done
}
######################################
#	     FULL - BUILD	     #
######################################
function full {
	# clear the screen here
	clear
	# Select device
	while true; do
	    read -p "Select device: [1]Spyder [2]Edison [3]Targa [4]Abort : " 1234
	    case $1234 in
	        [1]* ) spyder; main;;
	        [2]* ) echo "Currently not available..."; main;;
		[3]* ) echo "Currently not available..."; main;;
		[4]* ) main;;
	        * ) echo "Enter a number from 1-4 ";;
	    esac
	done
}

##############################
#	    SPYDER	     #
##############################
function spyder {

	sleep 1
	clear;
	echo " "
	echo " "
	echo "JBX-Kernel: SPYDER "
	echo "-------------------"
	sleep 1
	echo " 	   PREPARE"
	echo " "
	# Sync ?
	cd /data/4.4
	while true; do
	    read -p "Do you wish to sync repo? " yn
	    case $yn in
	        [Yy]* ) echo "Syncing repo..."; echo " "; repo sync; break;;
	        [Nn]* ) break;;
	        * ) echo "Please answer yes or no.";;
	    esac
	done
	
	echo " "
	
	# Clean build dirs
	cd /data/4.4
	while true; do
	    read -p "Do you wto clean build dirs? " yn
	    case $yn in
        	[Yy]* ) echo "Cleaning out kernel source directory..."; make mrproper; make ARCH=arm distclean; cd $kern_dir; make mrproper; break;;
        	[Nn]* ) break;;
        	* ) echo "Please answer yes or no.";;
	    esac
	done

	echo " "
	clear;
	# We build the kernel and its modules first
	# Launch execute script in background
	# First get tags in shell
	echo " "
	echo "Entering Android source path..."
	sleep 1
	cd $cm_dir
	echo " "
	echo "exporting CCACHE=1..."
	sleep 1
	export USE_CCACHE=1
	echo " "
	echo "Call envsetup.sh..."
	source build/envsetup.sh
	# Make sure we use the right JDK
	echo " "
	echo "Exporting the right JDK version into our path..."
	export PATH=${PATH/\/path\/to\/jdk\/dir:/}
	echo " "
	echo "Call device setup: cm_spyder-userdebug..."
	lunch cm_spyder-userdebug
	echo " "
	# built kernel & modules
	echo "Building kernel and modules..."
	echo " "
	
	# exporting compiler flags and paths
	echo "export ARCH=arm"
	export ARCH=arm
	sleep 1
	echo "export SUBARCH=arm"
	export SUBARCH=arm
	sleep 1
	echo "export PATH=$toolchain:$PATH"
	export PATH=$toolchain:$PATH
	sleep 1
	echo "export CROSS_COMPILE=arm-eabi-"
	export CROSS_COMPILE=arm-eabi-
	sleep 1
	echo
	export LOCALVERSION="-JBX-3.0-Hybrid-Razr-4.4"
	echo "Start building....."
	make -j$core TARGET_KERNEL_SOURCE=$kern_dir TARGET_KERNEL_CONFIG=mapphone_OCE_defconfig $OUT/boot.img
	echo " "
	echo " "
	echo "DONE!"
	echo " "
	
	# Build libhealthd.omap4
	while true; do
	    read -p "Do you wish to include 10% battery meter? " yn
	    case $yn in
	        [Yy]* ) echo "Moving Ramdisk into built path..."; echo " "; cp $cm_dir/out/target/product/spyder/ramdisk.img $built_dir/4.4/3.0/rls/jbx/Applications/ramdisk/; break;;
        	[Nn]* ) break;;
        	* ) echo "Please answer yes or no.";;
	    esac
	done

	echo " "

	# We don't use the kernel but the modules
	echo "Copying modules to package folder"
	echo " "
	cp -r $cm_dir/out/target/product/spyder/system/lib/modules/* $built_dir/4.4/3.0/rls/system/lib/modules/
	cp $cm_dir/out/target/product/spyder/kernel $built_dir/4.4/3.0/rls/system/etc/kexec/
	
	echo "------------- "
	echo "Done building"
	echo "------------- "
	echo " "
	
	# Copy and rename the zImage into nightly/nightly package folder
	# Keep in mind that we assume that the modules were already built and are in place
	# So we just copy and rename, then pack to zip including the date
	echo "Packaging flashable Zip file..."
	echo " "
	
	cd /home/dtrail/android/built/4.4/3.0/rls
	zip -r "JBX-Kernel-3.0-Hybrid-Spyder-4.4_$(date +"%Y-%m-%d").zip" *
	mv "JBX-Kernel-3.0-Hybrid-Spyder-4.4_$(date +"%Y-%m-%d").zip" $out_dir
	
	# Exporting changelog to file
	cd $kern_dir
	while true; do
	    read -p "Do you wish to push the latest changelog? " yn
	    case $yn in
	        [Yy]* ) echo "Exporting changelog to file: '/built/Changelog-[date]'"; echo " "; git log --oneline --since="4 day ago" > $kern_dir/changelog/Changelog_$(date +"%Y-%m-%d"); git log --oneline  > $kern_dir/changelog/Full_History_Changelog; git add changelog/ .; git commit -m "Added todays changelog and updated full history"; git push origin JBX_30X; echo " "; echo "done"; break;;
        	[Nn]* ) exit;;
        	* ) echo "Please answer yes or no.";;
	    esac
	done
}

echo " ": main;
