#
# Makefile for sensor chip drivers.
#

obj-$(CONFIG_THERMAL)				+= thermal_sys.o
thermal_sys-y                   		+= thermal_core.o 
obj-$(CONFIG_CPU_THERMAL)       		+= cpu_cooling.o

# interface to/from other layers providing sensors
thermal_sys-$(CONFIG_THERMAL_HWMON)             += thermal_hwmon.o
thermal_sys-$(CONFIG_THERMAL_OF)                += of-thermal.o 

# governors
obj-$(CONFIG_THERMAL_GOV_FAIR_SHARE)        += fair_share.o
obj-$(CONFIG_THERMAL_GOV_WISE)         += step_wise.o
obj-$(CONFIG_THERMAL_GOV_USER_SPACE)        += user_space.o
