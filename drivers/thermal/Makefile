#
# Makefile for sensor chip drivers.
#

obj-$(CONFIG_THERMAL)				+= thermal_sys.o
thermal_sys-y                   		+= thermal_core.o 

# interface to/from other layers providing sensors
thermal_sys-$(CONFIG_THERMAL_HWMON)             += thermal_hwmon.o
thermal_sys-$(CONFIG_THERMAL_OF)                += of-thermal.o 

# governors
thermal_sys-$(CONFIG_THERMAL_GOV_FAIR_SHARE)    += fair_share.o
