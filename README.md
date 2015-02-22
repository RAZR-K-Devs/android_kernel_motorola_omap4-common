omap4_jbx_kernel
===============

JBX-Kernel for Motorola XT910/XT912 OMAP4 (D-WiZ JBX Rom Series - Battery Saving Technologies) [AOSP, JB]

This Kernel is based on the Motorola 3.0.8 Kernel for Kexec which was initiated by the STS-Dev-Team.
See this link for the original source: https://github.com/STS-Dev-Team/kernel_mapphone_kexec
See credits below

The latest Kernel is patched up to stable 3.0.31 release!




[JBX-Kernel-Hybrid]
-------------------
3.0.8 / 3.0.31

Some features of JBX-Kernel:


- Suspend Governor Control - (CURRENTLY DISABLED)
- Battery Friend toggle (a battery friendly mode)
- DPLL CASCADING
- Dynamic Hotplug: Second core will be turned off ONLY while screen is off - independent from selected governor.
- Optimized OPP Table for smooth CPU scaling
- Frequencies: 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1350
- Modifed Smartreflex driver (Custom Sensor for detecting n-Value).
- Smartreflex Tuning Interface: Set min calibrated voltage
- Smartreflex Tuning Interface: Set max calibrated voltage
- Overriden trim quirks (experimental port from OMAP4460, no DCC) to override factoy defaults and reach stable overclocking up to ~1,5ghz (depends on device, each silicon is different!)
- Overclocking using Live OC (mine runs stable at a maximum frequency of 1,498ghz!)
- hwmod, uart - cleanups from pre-kexec config to safe power
- hwmod, uart, IRQ - Configuration for power saving
- CPU: lower voltages for CORE and IVA. Give CORE the abbility to scale up to higher voltage if needed
- Added IVA_NITROSB
- fsync Control: Less IO Speed but safes you from loosing data if phone crashes
- Dynamic fsync control: FSYNC interval is dynamic depending on screen state (SCREEN OFF: synchronous, SCREEN ON: asynchronous)
- Dynamic page-writeback: Page writeback interval is dynamic depending on screen state.
- Frandom v2
- JRCU / Tiny RCU (currently JRCU in use)
- Raised voltage limits for mpu a bit
- Raised the temperature limits from 64c* to 74c* (degrees)
- optimized CRC32 algorithm (better code generation)
- Optimized LZO compression
- RW Readahead dynamically depending on storage device (automatic detection of the best value)
- zRAM support
- GPU has 4 scaling steps and OC to 384mhz (Base freq: 102 mhz --> 154 mhz, 307 mhz, 384 mhz)
- GPU: Added C4 states for Power Down mode
- GPU Governor switch [on3demand, activeidle, onoff, userspace)
- Multicore Power Saving Mode Control
- ARCH Dependant Power feature
- Temp Control (Modify temperature limit for CPU/MPU) - (CURRENTLY DISABLED)
- Gamma Control
- Front Buffer Delay Control (draw in x msecs on early suspend)
- Screen/Display: Modified OMAPDSS for sharpness and lightning colors
- OMAPDSS: Added variable clock rate and OPP - allows the screen to scale down power and voltage
- lowmemkiller: Heavy modified for R/W Speed and efficient performance
- ZCACHE, ZSMALLOC, XVMALLOC backported from 3.4, 3.7 and 3.10
- Custom Voltage Support
- IO-Schedulers: SIOPlus, Fifo, Row, VR, Noop, Deadline, CFQ
- CPU: More Governors
- Deep Idle
- ARM Topology
- Many improvements in overall OMAP PM
- SELinux permissive
- GREAT performance!
- battery life!
- Support for Ezekeel's "eXperience" App (Adds features: Music Control with Vol-Buttons, etc..)
- Support for Trickster Mod Kernel Control App (Download from Gplay)

-...more to come


WIP:

- Fix Touch Wake
- Fix BLX
- Fix battery (also in Roms, battd)



[HOW CAN I BUILD JBX-KERNEL ?]
-----------------------------

This tutorial explains how to build JBX-Kernel 3.0 Series (3.0.31)! You will need Linux!




PREPARE ROM SOURCE

1. Create work dir with FULL CM12 sources on your local hard drive.
2. After syncing depencies, prebuilts, etc, head into your CM11 source, go to "YOUR_CM_SOURCE_DIR/.repo/local_manifests/".
3. Edit the "roomservice.xml" and find the following line

	project name="Cyanogenmod/android_device_motorola_omap4-common" path="device/motorola/omap4-common" remote="github" 

4. Replace it with this line

	project name="RAZR-K-Devs/android_device_motorola_omap4-common" path="device/motorola/omap4-common" remote="github"




PREPARE KERNEL SOURCE

1. git clone this repo (JBX-Kernel) into a seperate folder.
2. Be sure you're in JBX_40X branch.
3. Edit build scripts for your local paths.
   (i.e. for Droid RAZR use 'build_quad.sh', for ATRIX 2 use 'build_edison_quad.sh', etc..).
4. Keep in mind that the build scripts assume you to have my 'built_rls' repo cloned on your local harddrive.
5. If you don't want to use it, exclude the section and install the kernel manually! Otherwise proceed NOW at 'OPTIONAL'.
6. Run the build script (or proceed firsth with OPTIONAL).

NOTE: CM12 doesn't use the vendor/cm/get-prebuilts script anymore! Just skip this step if you come to it by reading the CM building guides!


[OPTIONAL]
----------
If you want to use my AROMA Installer Setup continue here:

clone my repo "built_rls" and name its folder "built".
Edit the paths in build scripts accordingly to where you stored 'built_rls' repo.
Now, finally, run the build script.

You will find a ready package of JBX-Kernel in your 'out' dir (or wherever you defined it to be saved).
Just flash this zip file and enjoy!





For releases and support check the XDA-Thread: http://forum.xda-developers.com/showthread.php?t=2223517



*Credits to 

- Kholk & [mbm] for Kexec Initial Release
- Hashcode & dhacker29 (STS-Dev-Team) for fixing Kexec, Base Kernel release and all their great work!
- Ezekeel, Imoseyon - for source code of: Live OC, Custom Coltage, SLQB, GPU OC, Optimized CRC32 Algorithm, Touch Wake, Wheatley, Lazy, Fsync, Gamma Control, more...
- Bigeyes0x0 for Trickster Mod App and support
- Biotik / Bionx NX - Catalyst/JBX-CoDevelopment
- XDA-Developers - great community!

