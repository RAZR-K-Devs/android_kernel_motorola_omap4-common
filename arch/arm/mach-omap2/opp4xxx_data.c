/*
 * OMAP4 OPP table definitions.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *	Kevin Hilman
 *	Thara Gopinath
 * Copyright (C) 2010-2011 Nokia Corporation.
 *      Eduardo Valentin
 *      Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/opp.h>

#include <plat/cpu.h>
#include <plat/common.h>

#include "control.h"
#include "omap_opp_data.h"
#include "pm.h"

#ifdef CONFIG_CUSTOM_VOLTAGE      
#include <linux/custom_voltage.h>
#endif

#ifdef CONFIG_LIVE_OC
#include <linux/live_oc.h>
#endif


/*
 * Structures containing OMAP4430 voltage supported and various
 * voltage dependent data for each VDD. 
 * 
 * dtrail: We have default OPP1-5; extendet to steps of +100mhz = 12 OPPs
 * 	   Because we have limited eFuses, we must define one eFuse per each OPP in header file 
 *   	   and adjust voltages for OPPs
 *
 * 		Define voltage for and fuses for new OPPs
 */
#ifdef CONFIG_OMAP_OCFREQ_12
#define OMAP4430_VDD_MPU_OPP_UVA_UV		 990000			/* 100 */
#define OMAP4430_VDD_MPU_OPP50_UV		1005000			/* 200 */
#define OMAP4430_VDD_MPU_OPP100_UV		1020000			/* 300 */
#define OMAP4430_VDD_MPU_OPP150_UV		1075000			/* 400 */
#define OMAP4430_VDD_MPU_OPP200_UV		1125000			/* 500 */
#define OMAP4430_VDD_MPU_OPP250_UV		1195000			/* 600 */
#define OMAP4430_VDD_MPU_OPPTURBO_UV		1275000			/* 700 */
#define OMAP4430_VDD_MPU_OPPTURBOB_UV		1320000			/* 800 */
#define OMAP4430_VDD_MPU_OPPTURBOC_UV		1355000			/* 900 */
#define OMAP4430_VDD_MPU_OPPNITRO_UV		1385000			/* 1000 */
#define OMAP4430_VDD_MPU_OPPNITROS_UV		1390000			/* 1100 */
#define OMAP4430_VDD_MPU_OPPNITROSB_UV		1395000			/* 1200 */
#define OMAP4430_VDD_MPU_OPPNITROSBA_UV		1405000			/* 1300 */
#else
#define OMAP4430_VDD_MPU_OPP25_UV		 990000			/* 100 */
#define OMAP4430_VDD_MPU_OPP50_UV		1010000			/* 200 */
#define OMAP4430_VDD_MPU_OPP100_UV		1020000			/* 300 */
#define OMAP4430_VDD_MPU_OPPTURBO_UV		1080000			/* 400 */
#define OMAP4430_VDD_MPU_OPPTURBOB_UV		1195000			/* 600 */
#define OMAP4430_VDD_MPU_OPPNITRO_UV		1320000			/* 800 */
#define OMAP4430_VDD_MPU_OPPNITROSB_UV		1383000			/* 1000 */
#endif

/* dtrail: connecting OPPs to fuses and set smartreflex values */

#ifdef CONFIG_OMAP_SMARTREFLEX_CUSTOM_SENSOR
struct omap_volt_data omap443x_vdd_mpu_volt_data[] = {
#ifdef CONFIG_OMAP_OCFREQ_12
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP_UVA_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP150_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP200_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP250_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0, 0, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBOB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0, 0, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBOC_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROS_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROSB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROSBA_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0, 0),
};
#else
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP25_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP25, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBOB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBOB, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0, 0, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROSB_UV, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0, 0),
};
#endif
#else
struct omap_volt_data omap443x_vdd_mpu_volt_data[] = {
#ifdef CONFIG_OMAP_OCFREQ_12
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP_UVA_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP150_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP200_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP250_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBOB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBOC_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROS_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROSB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROSBA_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};
#else
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP25_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP25, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf4, 0x0c,  OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBOB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBOB, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROSB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};
#endif
#endif									
#ifdef CONFIG_BATTERY_FRIEND
#define OMAP4430_VDD_IVA_OPP50_UV		 950000
#define OMAP4430_VDD_IVA_OPP100_UV		1140000
#define OMAP4430_VDD_IVA_OPPTURBO_UV		1291000
#define OMAP4430_VDD_IVA_OPPNITRO_UV    	1375000
#define OMAP4430_VDD_IVA_OPPNITROSB_UV    	1376000
#else
#define OMAP4430_VDD_IVA_OPP50_UV		 910000
#define OMAP4430_VDD_IVA_OPP100_UV		1075000
#define OMAP4430_VDD_IVA_OPPTURBO_UV		1175000
#define OMAP4430_VDD_IVA_OPPNITRO_UV    	1275000
#define OMAP4430_VDD_IVA_OPPNITROSB_UV    	1376000
#endif

#ifdef CONFIG_OMAP_SMARTREFLEX_CUSTOM_SENSOR
struct omap_volt_data omap443x_vdd_iva_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0, 0, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPNITRO, 0, 0, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0, 0),
};
#else
struct omap_volt_data omap443x_vdd_iva_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPNITRO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};
#endif
/* We define two more GPU scaling steps and adjust voltages */
#define OMAP4430_VDD_CORE_OPP25_UV		 902000
#define OMAP4430_VDD_CORE_OPP50_UV	         962000
#define OMAP4430_VDD_CORE_OPP100_UV		1127000
#define OMAP4430_VDD_CORE_OPP100_OV_UV		1250000

#ifdef CONFIG_OMAP_SMARTREFLEX_CUSTOM_SENSOR
struct omap_volt_data omap443x_vdd_core_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP25_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0, 0, 0xf4, 0x0c, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP100_OV_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0, 0),
};
#else
struct omap_volt_data omap443x_vdd_core_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP25_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0xf4, 0x0c, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf4, 0x0c, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf9, 0x16, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP100_OV_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf9, 0x16, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};
#endif

/* Dependency of domains are as follows for OMAP4430 (OPP based):
 *
 *	MPU	IVA	CORE
 *	50	50	50+
 *	50	100+	100
 *	100+	50	100
 *	100+	100+	100
 */

/* OMAP 4430 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap443x_vdd_mpu_core_dep_data[] = {
#ifdef CONFIG_OMAP_OCFREQ_12
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP_UVA_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP50_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP150_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP200_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP250_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBOB_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBOC_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITRO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROS_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROSB_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROSBA_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
};
#else
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP25_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP50_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBOB_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITRO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROSB_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
};
#endif

struct omap_vdd_dep_info omap443x_vddmpu_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap443x_vdd_mpu_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap443x_vdd_mpu_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

/* OMAP 4430 MPU IVA VDD dependency table */
static struct omap_vdd_dep_volt omap443x_vdd_iva_core_dep_data[] = {
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPP50_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPPNITRO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPPNITROSB_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap443x_vddiva_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap443x_vdd_iva_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap443x_vdd_iva_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

static struct omap_opp_def __initdata omap443x_opp_def_list[] = {
#ifdef CONFIG_OMAP_OCFREQ_12
	/* MPU OPP1 - OPP25 */
 	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 100000000, OMAP4430_VDD_MPU_OPP_UVA_UV),
	/* MPU OPP1 - OPP50 */
 	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 200000000, OMAP4430_VDD_MPU_OPP50_UV),
	/* MPU OPP1 - OPP100 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 300000000, OMAP4430_VDD_MPU_OPP100_UV),
	/* MPU OPP1 - OPP150 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 400000000, OMAP4430_VDD_MPU_OPP150_UV),
	/* MPU OPP1 - OPP200 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 500000000, OMAP4430_VDD_MPU_OPP200_UV),
	/* MPU OPP1 - OPP250 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 600000000, OMAP4430_VDD_MPU_OPP250_UV),
	/* MPU OPP1 - OPP300 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 700000000, OMAP4430_VDD_MPU_OPPTURBO_UV),
	/* MPU OPP2 - OPP-TB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 800000000, OMAP4430_VDD_MPU_OPPTURBOB_UV),
	/* MPU OPP2 - OPP-TBB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 900000000, OMAP4430_VDD_MPU_OPPTURBOC_UV),
	/* MPU OPP3 - OPP-NT */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 1000000000, OMAP4430_VDD_MPU_OPPNITRO_UV),
	/* MPU OPP3 - OPP-NTS */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 1100000000, OMAP4430_VDD_MPU_OPPNITROS_UV),
	/* MPU OPP4 - OPP-NTSB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 1200000000, OMAP4430_VDD_MPU_OPPNITROSB_UV),
	/* MPU OPP4 - OPP-NTSB1 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 1300000000, OMAP4430_VDD_MPU_OPPNITROSBA_UV),
#else
	/* MPU OPP1 - OPP25 */
 	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 100000000, OMAP4430_VDD_MPU_OPP25_UV),
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 200000000, OMAP4430_VDD_MPU_OPP50_UV),
	/* MPU OPP1 - OPP100 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 300000000, OMAP4430_VDD_MPU_OPP100_UV),
	/* MPU OPP1 - OPP100B */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 400000000, OMAP4430_VDD_MPU_OPPTURBO_UV),
	/* MPU OPP2 - OPP-TB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 600000000, OMAP4430_VDD_MPU_OPPTURBOB_UV),
	/* MPU OPP3 - OPP-NT */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", false, 800000000, OMAP4430_VDD_MPU_OPPNITRO_UV),
	/* MPU OPP4 - OPP-NTSB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", false, 1000000000, OMAP4430_VDD_MPU_OPPNITROSB_UV),
#endif
	/* MPU OPP1 - OPP50  */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 100000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* L3 OPP2 - OPP100, OPP-Turbo, OPP-SB */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* L3 OPP3 - GIve core the abbillity to use higher voltage if needed - this can prevent crashes during OC */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4430_VDD_CORE_OPP100_OV_UV),
	/* IVA OPP1 - OPP50 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 133000000, OMAP4430_VDD_IVA_OPP50_UV),
	/* IVA OPP2 - OPP100 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 266100000, OMAP4430_VDD_IVA_OPP100_UV),
	/* IVA OPP3 - OPP-Turbo */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 332000000, OMAP4430_VDD_IVA_OPPTURBO_UV),
	/* IVA OPP4 - OPP-Nitro */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 430000000, OMAP4430_VDD_IVA_OPPNITRO_UV),
	/* IVA OPP4 - OPP-NitroSB */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 500000000, OMAP4430_VDD_IVA_OPPNITROSB_UV),
	/* SGX OPP1 - OPP50 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 100300000, OMAP4430_VDD_CORE_OPP25_UV),
	/* SGX OPP1 - OPP50 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 153600000, OMAP4430_VDD_CORE_OPP50_UV),
	/* SGX OPP2 - OPP100 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 307200000, OMAP4430_VDD_CORE_OPP100_UV),
	/* SGX OPP3 - OPPOV  dtrail: Added third GPU OPP and overclocked to factory default */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 384000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* SGX OPP4 - OPPOV  dtrail: Added fourth GPU OPP and overclocked to 416mhz */
//	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", false, 416000000, OMAP4430_VDD_CORE_OPP100_OV_UV),
	/* FDIF OPP1 - OPP25 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 32000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* FDIF OPP2 - OPP50 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 64000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* FDIF OPP3 - OPP100 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 128000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* DSP OPP1 - OPP50 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 232750000, OMAP4430_VDD_IVA_OPP50_UV),
	/* DSP OPP2 - OPP100 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 465500000, OMAP4430_VDD_IVA_OPP100_UV),
	/* DSP OPP3 - OPPTB */
#ifdef CONFIG_BATTERY_FRIEND
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "false", false, 496000000, OMAP4430_VDD_IVA_OPPTURBO_UV),
#else
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 496000000, OMAP4430_VDD_IVA_OPPTURBO_UV),
#endif
	/* HSI OPP1 - OPP50 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* HSI OPP2 - OPP100 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 192000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* ABE OPP1 - OPP50 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 98304000, OMAP4430_VDD_IVA_OPP50_UV),
	/* ABE OPP2 - OPP100 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 196608000, OMAP4430_VDD_IVA_OPP100_UV),
	/* DSS OPP1 - OPP50 */
	OPP_INITIALIZER("dss_dispc", "virt_lcd_pclk", "core", true, 91000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* DSS OPP2 - OPP100 */
	OPP_INITIALIZER("dss_dispc", "virt_lcd_pclk", "core", true, 170000000, OMAP4430_VDD_CORE_OPP100_UV),
};

#define OMAP4460_VDD_MPU_OPP50_UV    1025000
#define OMAP4460_VDD_MPU_OPP100_UV    1203000
#define OMAP4460_VDD_MPU_OPPTURBO_UV    1317000
#define OMAP4460_VDD_MPU_OPPNITRO_UV    1380000
#define OMAP4460_VDD_MPU_OPPNITROSB_UV    1390000

#ifdef CONFIG_OMAP_SMARTREFLEX_CUSTOM_SENSOR
struct omap_volt_data omap446x_vdd_mpu_volt_data[] = {
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPTURBO_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0, 0, 0xfa, 0x23, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITRO_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITROSB_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0, 0, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0, 0),
};
#else
struct omap_volt_data omap446x_vdd_mpu_volt_data[] = {
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPTURBO_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITRO_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITROSB_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};
#endif

#define OMAP4460_VDD_IVA_OPP50_UV     950000
#define OMAP4460_VDD_IVA_OPP100_UV    1140000
#define OMAP4460_VDD_IVA_OPPTURBO_UV    1291000
#define OMAP4460_VDD_IVA_OPPNITRO_UV    1375000

#ifdef CONFIG_OMAP_SMARTREFLEX_CUSTOM_SENSOR
struct omap_volt_data omap446x_vdd_iva_volt_data[] = {
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPTURBO_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0, 0, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPNITRO_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPPNITRO, 0, 0, 0xfa, 0x23, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0, 0),
};
#else
struct omap_volt_data omap446x_vdd_iva_volt_data[] = {
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPTURBO_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
  VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPNITRO_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPPNITRO, 0xfa, 0x23, OMAP_ABB_FAST_OPP),
  VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};
#endif

#define OMAP4460_VDD_CORE_OPP50_UV     962000
#define OMAP4460_VDD_CORE_OPP100_UV    1127000
#define OMAP4460_VDD_CORE_OPP100_OV_UV    1250000

#ifdef CONFIG_OMAP_SMARTREFLEX_CUSTOM_SENSOR
struct omap_volt_data omap446x_vdd_core_volt_data[] = {
  VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0, 0, 0xf4, 0x0c, OMAP_ABB_NONE),
  VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0, 0, 0xf9, 0x16, OMAP_ABB_NONE),
  VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_OV_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100OV, 0, 0, 0xf9, 0x16, OMAP_ABB_NONE),
  VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0, 0),
};
#else
struct omap_volt_data omap446x_vdd_core_volt_data[] = {
  VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0xf4, 0x0c, OMAP_ABB_NONE),
  VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf9, 0x16, OMAP_ABB_NONE),
  VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_OV_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100OV, 0xf9, 0x16, OMAP_ABB_NONE),
  VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};
#endif

/* OMAP 4460 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap446x_vdd_mpu_core_dep_data[] = {
  {.main_vdd_volt = OMAP4460_VDD_MPU_OPP50_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP50_UV},
  {.main_vdd_volt = OMAP4460_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
  {.main_vdd_volt = OMAP4460_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
  {.main_vdd_volt = OMAP4460_VDD_MPU_OPPNITRO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
  {.main_vdd_volt = OMAP4460_VDD_MPU_OPPNITROSB_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap446x_vddmpu_dep_info[] = {
  {
    .name  = "core",
    .dep_table = omap446x_vdd_mpu_core_dep_data,
    .nr_dep_entries = ARRAY_SIZE(omap446x_vdd_mpu_core_dep_data),
  },
  {.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

/* OMAP 4460 MPU IVA VDD dependency table */
static struct omap_vdd_dep_volt omap446x_vdd_iva_core_dep_data[] = {
  {.main_vdd_volt = OMAP4460_VDD_IVA_OPP50_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP50_UV},
  {.main_vdd_volt = OMAP4460_VDD_IVA_OPP100_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
  {.main_vdd_volt = OMAP4460_VDD_IVA_OPPTURBO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap446x_vddiva_dep_info[] = {
  {
    .name  = "core",
    .dep_table = omap446x_vdd_iva_core_dep_data,
    .nr_dep_entries = ARRAY_SIZE(omap446x_vdd_iva_core_dep_data),
  },
  {.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

static struct omap_opp_def __initdata omap446x_opp_def_list[] = {
  /* MPU OPP1 - OPP50 */
  OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 350000000, OMAP4460_VDD_MPU_OPP50_UV),
  /* MPU OPP2 - OPP100 */
  OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 700000000, OMAP4460_VDD_MPU_OPP100_UV),
  /* MPU OPP3 - OPP-Turbo */
  OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 920000000, OMAP4460_VDD_MPU_OPPTURBO_UV),
  /* MPU OPP4 - OPP-Nitro */
  OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1200000000, OMAP4460_VDD_MPU_OPPNITRO_UV),
  /* MPU OPP4 - OPP-Nitro SpeedBin */
  OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1500000000, OMAP4460_VDD_MPU_OPPNITROSB_UV),
  /* L3 OPP1 - OPP50 */
  OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 100000000, OMAP4460_VDD_CORE_OPP50_UV),
  /* L3 OPP2 - OPP100 */
  OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4460_VDD_CORE_OPP100_UV),
  /* IVA OPP1 - OPP50 */
  OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 133000000, OMAP4460_VDD_IVA_OPP50_UV),
  /* IVA OPP2 - OPP100 */
  OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 266100000, OMAP4460_VDD_IVA_OPP100_UV),
  /*
   * IVA OPP3 - OPP-Turbo + Disabled as the reference schematics
   * recommends Phoenix VCORE2 which can supply only 600mA - so the ones
   * above this OPP frequency, even though OMAP is capable, should be
   * enabled by board file which is sure of the chip power capability
   */
  OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 332000000, OMAP4460_VDD_IVA_OPPTURBO_UV),
  /* IVA OPP4 - OPP-Nitro */
  OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 430000000, OMAP4460_VDD_IVA_OPPNITRO_UV),
  /* IVA OPP5 - OPP-Nitro SpeedBin*/
  OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 500000000, OMAP4460_VDD_IVA_OPPNITRO_UV),

  /* SGX OPP1 - OPP50 */
  OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 153600000, OMAP4460_VDD_CORE_OPP50_UV),
  /* SGX OPP2 - OPP100 */
  OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 307200000, OMAP4460_VDD_CORE_OPP100_UV),
  /* SGX OPP3 - OPPOV */
  OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 384000000, OMAP4460_VDD_CORE_OPP100_OV_UV),
  /* FDIF OPP1 - OPP25 */
  OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 32000000, OMAP4460_VDD_CORE_OPP50_UV),
  /* FDIF OPP2 - OPP50 */
  OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 64000000, OMAP4460_VDD_CORE_OPP50_UV),
  /* FDIF OPP3 - OPP100 */
  OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 128000000, OMAP4460_VDD_CORE_OPP100_UV),
  /* DSP OPP1 - OPP50 */
  OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 232750000, OMAP4460_VDD_IVA_OPP50_UV),
  /* DSP OPP2 - OPP100 */
  OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 465500000, OMAP4460_VDD_IVA_OPP100_UV),
  /* DSP OPP3 - OPPTB */
  OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", false, 496000000, OMAP4460_VDD_IVA_OPPTURBO_UV),
  /* HSI OPP1 - OPP50 */
  OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4460_VDD_CORE_OPP50_UV),
  /* HSI OPP2 - OPP100 */
  OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 192000000, OMAP4460_VDD_CORE_OPP100_UV),
  /* ABE OPP1 - OPP50 */
  OPP_INITIALIZER("aess", "abe_clk", "iva", true, 98304000, OMAP4460_VDD_IVA_OPP50_UV),
  /* ABE OPP2 - OPP100 */
  OPP_INITIALIZER("aess", "abe_clk", "iva", true, 196608000, OMAP4460_VDD_IVA_OPP100_UV),
  /* DSS OPP1 - OPP50 */
  OPP_INITIALIZER("dss_dispc", "virt_lcd_pclk", "core", true, 93000000, OMAP4460_VDD_CORE_OPP50_UV),
  /* DSS OPP2 - OPP100 */
#ifdef CONFIG_PVR_GOVERNOR
  OPP_INITIALIZER("dss_dispc", "virt_lcd_pclk", "core", true, 170000000, OMAP4460_VDD_CORE_OPP75_UV),
#else
  OPP_INITIALIZER("dss_dispc", "virt_lcd_pclk", "core", true, 170000000, OMAP4460_VDD_CORE_OPP100_UV),
#endif 
}; 

/**
* omap4_mpu_opp_enable() - helper to enable the OPP
* @freq: frequency to enable
*/
static void __init omap4_opp_enable(const char *oh_name, unsigned long freq)
{
		struct device *dev;
		int r;

		dev = omap_hwmod_name_get_dev(oh_name);
		if (IS_ERR(dev)) {
			pr_err("%s: no %s device, did not enable f=%ld\n", __func__,
				oh_name, freq);
		return;
		}

		r = opp_enable(dev, freq);
		if (r < 0)
			dev_err(dev, "%s: opp_enable failed(%d) f=%ld\n", __func__,
				r, freq);
}


int __init omap4_opp_init(void)
{
	int r = -ENODEV;
	int trimmed = 1;

	if (!cpu_is_omap44xx())
		return r;

	if (cpu_is_omap443x())
		r = omap_init_opp_table(omap443x_opp_def_list,
				ARRAY_SIZE(omap443x_opp_def_list));
	else if (cpu_is_omap446x()) {
		r = omap_init_opp_table(omap446x_opp_def_list,
				ARRAY_SIZE(omap446x_opp_def_list));
		trimmed = omap_readl(0x4a002268) & ((1 << 18) | (1 << 19));
		if (!trimmed) 
			omap_writel(0x29, 0x4a002330);
	}


		if (r)
		goto out;

		/* Enable Nitro and NitroSB IVA OPPs */
		if (omap4_has_iva_430mhz())
			omap4_opp_enable("iva", 430000000);
		if (omap4_has_iva_500mhz())
			omap4_opp_enable("iva", 500000000);

		/* Enable Nitro and NitroSB MPU OPPs */
		if (omap4_has_mpu_1_2ghz())
			omap4_opp_enable("mpu", 1200000000);
			omap4_opp_enable("mpu", 1300000000);

		if (omap4_has_mpu_1_5ghz()&& trimmed)
			omap4_opp_enable("mpu", 1500000000); 

		/* Enable GPU OPPs */
			omap4_opp_enable("gpu", 153600000);
			omap4_opp_enable("gpu", 384000000);


#ifdef CONFIG_CUSTOM_VOLTAGE
	customvoltage_init();
#endif

#ifdef CONFIG_LIVE_OC
	liveoc_init();
#endif

out:
	return r;
}
device_initcall(omap4_opp_init);
