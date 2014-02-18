/*
 * Copyright (C) 2007-2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <asm/div64.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include "cpcap_charge_table.h"

#ifdef CONFIG_BLX
#include <linux/blx.h>
#endif

#define CPCAP_BATT_IRQ_BATTDET 0x01
#define CPCAP_BATT_IRQ_OV      0x02
#define CPCAP_BATT_IRQ_CC_CAL  0x04
#define CPCAP_BATT_IRQ_ADCDONE 0x08c
#define CPCAP_BATT_IRQ_MACRO   0x10
#define INDCHRG_RS_TIME        (15*60)	/* 15 mins */
#define INDCHRG_RS_CPCY		95	/* 95% */
#define INDCHRG_HOT_TEMP	600	/* 60 C */
#define INDCHRG_COLD_TEMP	-200	/* -20 C */

#define CPCAP_BATT_PRINT_STATUS (1U << 0)
#define CPCAP_BATT_PRINT_TRANSITION (1U << 1)

static int debug_mask;
static int timestamp;
static unsigned long cc_counter;
static unsigned char cc_counter_percentage;

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define pr_cpcap_batt(debug_level_mask, args...) \
	do { \
		if (debug_mask & CPCAP_BATT_PRINT_##debug_level_mask) { \
			pr_info(args); \
		} \
	} while (0)

#define USE_OWN_CALCULATE_METHOD
static long cpcap_batt_ioctl(struct file *file,
			    unsigned int cmd,
			    unsigned long arg);
static unsigned int cpcap_batt_poll(struct file *file, poll_table *wait);
static int cpcap_batt_open(struct inode *inode, struct file *file);
static ssize_t cpcap_batt_read(struct file *file, char *buf, size_t count,
			       loff_t *ppos);
static int cpcap_batt_probe(struct platform_device *pdev);
static int cpcap_batt_remove(struct platform_device *pdev);
static int cpcap_batt_resume(struct platform_device *pdev);
static int set_timestamp(const char *val, const struct kernel_param *kp);
static int set_cc_counter(const char *val, const struct kernel_param *kp);
static int set_cc_counter_percentage(const char *val,
				const struct kernel_param *kp);

struct cpcap_batt_ps {
	struct power_supply batt;
	struct power_supply ac;
	struct power_supply usb;
	struct cpcap_device *cpcap;
	struct cpcap_batt_data batt_state;
	struct cpcap_batt_ac_data ac_state;
	struct cpcap_batt_usb_data usb_state;
	struct cpcap_adc_request req;
	struct mutex lock;
	char irq_status;
	char data_pending;
	wait_queue_head_t wait;
	char async_req_pending;
	unsigned long last_run_time;
	bool no_update;
	unsigned long ind_chrg_dsbl_time;
};

static void cpcap_batt_ind_chrg_ctrl(struct cpcap_batt_ps *sply);

static const struct file_operations batt_fops = {
	.owner = THIS_MODULE,
	.open = cpcap_batt_open,
	.unlocked_ioctl = cpcap_batt_ioctl,
	.read = cpcap_batt_read,
	.poll = cpcap_batt_poll,
};

static struct miscdevice batt_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "cpcap_batt",
	.fops	= &batt_fops,
};

static enum power_supply_property cpcap_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT
};

static enum power_supply_property cpcap_batt_ac_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_MODEL_NAME
};

static enum power_supply_property cpcap_batt_usb_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME
};

static struct platform_driver cpcap_batt_driver = {
	.probe		= cpcap_batt_probe,
	.remove		= cpcap_batt_remove,
	.resume		= cpcap_batt_resume,
	.driver		= {
		.name	= "cpcap_battery",
		.owner	= THIS_MODULE,
	},
};

static struct cpcap_batt_ps *cpcap_batt_sply;
#ifdef USE_OWN_CALCULATE_METHOD
static int cpcap_batt_status(struct cpcap_batt_ps *sply);
static int cpcap_batt_counter(struct cpcap_batt_ps *sply);
static int cpcap_batt_value(struct cpcap_batt_ps *sply, int value);
#endif

static struct kernel_param_ops timestamp_param_ops = {
	.set = set_timestamp,
	.get = param_get_int,
};

module_param_cb(timestamp, &timestamp_param_ops, &timestamp, 0644);
MODULE_PARM_DESC(timestamp, "Epoch format timestamp value which indicates last"
	"time when cycle_count var was updated");

static struct kernel_param_ops cc_counter_param_ops = {
	.set = set_cc_counter,
	.get = param_get_ulong,
};

module_param_cb(cc_counter, &cc_counter_param_ops, &cc_counter, 0644);
MODULE_PARM_DESC(cc_counter, "Charge cycle counter value");

static struct kernel_param_ops cc_counter_percentage_param_ops = {
	.set = set_cc_counter_percentage,
	.get = param_get_ushort,
};

module_param_cb(cc_counter_percentage, &cc_counter_percentage_param_ops,
	&cc_counter_percentage, 0644);
MODULE_PARM_DESC(cc_counter_percentage, "Charge cycle counter percentage value");

void cpcap_batt_irq_hdlr(enum cpcap_irqs irq, void *data)
{
	struct cpcap_batt_ps *sply = data;
	struct cpcap_platform_data *pdata = sply->cpcap->spi->dev.platform_data;

	mutex_lock(&sply->lock);
	sply->data_pending = 1;

	switch (irq) {
	case CPCAP_IRQ_BATTDETB:
#ifdef USE_OWN_CALCULATE_METHOD
               // printk("CPCAP_IRQ_BATTDETB\n");
#endif
		sply->irq_status |= CPCAP_BATT_IRQ_BATTDET;
		cpcap_irq_unmask(sply->cpcap, irq);
		break;

	case CPCAP_IRQ_VBUSOV:
#ifdef USE_OWN_CALCULATE_METHOD
               // printk("CPCAP_IRQ_VBUSOV\n");
#endif
		sply->irq_status |=  CPCAP_BATT_IRQ_OV;
		cpcap_irq_unmask(sply->cpcap, irq);
		cpcap_batt_ind_chrg_ctrl(sply);
		break;

	case CPCAP_IRQ_CC_CAL:
#ifdef USE_OWN_CALCULATE_METHOD
                //printk("CPCAP_IRQ_CC_CAL");
#endif
		sply->irq_status |= CPCAP_BATT_IRQ_CC_CAL;
		cpcap_irq_unmask(sply->cpcap, irq);
		break;

	case CPCAP_IRQ_UC_PRIMACRO_7:
	case CPCAP_IRQ_UC_PRIMACRO_8:
	case CPCAP_IRQ_UC_PRIMACRO_9:
	case CPCAP_IRQ_UC_PRIMACRO_10:
	case CPCAP_IRQ_UC_PRIMACRO_11:
#ifdef USE_OWN_CALCULATE_METHOD
                //printk("CPCAP_IRQ_UC_PRIMACRO\n");
#endif
		sply->irq_status |= CPCAP_BATT_IRQ_MACRO;
		break;
	default:
		break;
	}

	mutex_unlock(&sply->lock);

	wake_up_interruptible(&sply->wait);
}

void cpcap_batt_adc_hdlr(struct cpcap_device *cpcap, void *data)
{
	struct cpcap_batt_ps *sply = data;
	mutex_lock(&sply->lock);

	sply->async_req_pending = 0;

	sply->data_pending = 1;

	sply->irq_status |= CPCAP_BATT_IRQ_ADCDONE;

	mutex_unlock(&sply->lock);

	wake_up_interruptible(&sply->wait);
}

static int cpcap_batt_open(struct inode *inode, struct file *file)
{
	file->private_data = cpcap_batt_sply;
	return 0;
}

static unsigned int cpcap_batt_poll(struct file *file, poll_table *wait)
{
	struct cpcap_batt_ps *sply = file->private_data;
	unsigned int ret = 0;

	poll_wait(file, &sply->wait, wait);

	if (sply->data_pending)
		ret = (POLLIN | POLLRDNORM);

	return ret;
}

static ssize_t cpcap_batt_read(struct file *file,
			       char *buf, size_t count, loff_t *ppos)
{
	struct cpcap_batt_ps *sply = file->private_data;
	int ret = -EFBIG;
	unsigned long long temp;

	if (count >= sizeof(char)) {
		mutex_lock(&sply->lock);
		if (!copy_to_user((void *)buf, (void *)&sply->irq_status,
				  sizeof(sply->irq_status)))
			ret = sizeof(sply->irq_status);
		else
			ret = -EFAULT;
		sply->data_pending = 0;
		temp = sched_clock();
		do_div(temp, NSEC_PER_SEC);
		sply->last_run_time = (unsigned long)temp;

		sply->irq_status = 0;
		mutex_unlock(&sply->lock);
	}

	return ret;
}

static long cpcap_batt_ioctl(struct file *file,
			    unsigned int cmd,
			    unsigned long arg)
{
	long ret = 0;
	int i;
	struct cpcap_batt_ps *sply = file->private_data;
	struct cpcap_adc_request *req_async = &sply->req;
	struct cpcap_adc_request req;
	struct cpcap_adc_us_request req_us;
	struct spi_device *spi = sply->cpcap->spi;
	struct cpcap_platform_data *data = spi->dev.platform_data;

	switch (cmd) {
	case CPCAP_IOCTL_BATT_DISPLAY_UPDATE:
#ifdef USE_OWN_CALCULATE_METHOD
		printk("Prevent battd to set value :)\n");
		return 0;
#endif
		if (sply->no_update)
			return 0;
		if (copy_from_user((void *)&sply->batt_state,
				   (void *)arg, sizeof(struct cpcap_batt_data)))
			return -EFAULT;
		power_supply_changed(&sply->batt);
		cpcap_batt_ind_chrg_ctrl(sply);
#ifdef USE_OWN_CALCULATE_METHOD
		printk("CPCAP_IOCTL_BATT_DISPLAY_UPDATE");
#endif
		if (data->batt_changed)
			data->batt_changed(&sply->batt, &sply->batt_state);

		timestamp = sply->batt_state.timestamp;
		cc_counter = sply->batt_state.charge_cycle_counter;
		cc_counter_percentage =
			sply->batt_state.charge_cycle_counter_percentage;

		break;

	case CPCAP_IOCTL_BATT_ATOD_ASYNC:
		mutex_lock(&sply->lock);
		if (!sply->async_req_pending) {
			if (copy_from_user((void *)&req_us, (void *)arg,
					   sizeof(struct cpcap_adc_us_request)
					   )) {
				mutex_unlock(&sply->lock);
				return -EFAULT;
			}

			req_async->format = req_us.format;
			req_async->timing = req_us.timing;
			req_async->type = req_us.type;
			req_async->callback = cpcap_batt_adc_hdlr;
			req_async->callback_param = sply;
#ifdef USE_OWN_CALCULATE_METHOD
                        printk("CPCAP_IOCTL_BATT_ATOD_ASYNC:\n format %d\n timing %d\n type %d\n",req_us.format , req_us.timing, req_us.type);
#endif
			ret = cpcap_adc_async_read(sply->cpcap, req_async);
			if (!ret)
				sply->async_req_pending = 1;
		} else {
			ret = -EAGAIN;
		}
		mutex_unlock(&sply->lock);

		break;

	case CPCAP_IOCTL_BATT_ATOD_SYNC:
		if (copy_from_user((void *)&req_us, (void *)arg,
				   sizeof(struct cpcap_adc_us_request)))
			return -EFAULT;

#ifdef USE_OWN_CALCULATE_METHOD
		//return 0; //Uncomment to disable battd completely
#endif


		req.format = req_us.format;
		req.timing = req_us.timing;
		req.type = req_us.type;

		ret = cpcap_adc_sync_read(sply->cpcap, &req);

		if (ret)
			return ret;

		req_us.status = req.status;
#ifdef USE_OWN_CALCULATE_METHOD
                //printk("CPCAP_IOCTL_BATT_ATOD_SYNC:\n format %d\n timing %d\n type %d\n status %d\n",req.format , req.timing, req.type, req.status);
#endif
		for (i = 0; i < CPCAP_ADC_BANK0_NUM; i++)
			req_us.result[i] = req.result[i];
#ifdef USE_OWN_CALCULATE_METHOD
/*
if (req.type == 0) {
	printk("Dump of CPCAP_ADC_BANK0_NUM:\n CPCAP_ADC_VBUS:%d\n CPCAP_ADC_AD3:%d\n CPCAP_ADC_BPLUS_AD4:%d\n CPCAP_ADC_CHG_ISENSE:%d\n CPCAP_ADC_BATTI_ADC:%d\n CPCAP_ADC_USB_ID:%d\n",
                           req_us.result[CPCAP_ADC_VBUS],req_us.result[CPCAP_ADC_AD3],req_us.result[CPCAP_ADC_BPLUS_AD4],req_us.result[CPCAP_ADC_CHG_ISENSE],
                               req_us.result[CPCAP_ADC_BATTI_ADC],req_us.result[CPCAP_ADC_USB_ID]);
}

if (req.type == 2) {
	printk("Dump of CPCAP_ADC_BANK1_NUM:\n CPCAP_ADC_AD8:%d\n CPCAP_ADC_AD9:%d\n CPCAP_ADC_LICELL:%d\n CPCAP_ADC_HV_BATTP:%d\n CPCAP_ADC_TSX1_AD12:%d\n CPCAP_ADC_TSX2_AD13:%d\n CPCAP_ADC_TSX2_AD13:%d\n, CPCAP_ADC_TSX2_AD14:%d\n",req_us.result[CPCAP_ADC_AD8],req_us.result[CPCAP_ADC_AD9],req_us.result[CPCAP_ADC_LICELL],req_us.result[CPCAP_ADC_HV_BATTP],
                               req_us.result[CPCAP_ADC_TSX1_AD12],req_us.result[CPCAP_ADC_TSX2_AD13],req_us.result[CPCAP_ADC_TSY1_AD14],req_us.result[CPCAP_ADC_TSY2_AD15]);
}
*/
               // printk("Result Voltage: %dmV\n",req_us.result[CPCAP_ADC_BATTP]);
#endif
		if (copy_to_user((void *)arg, (void *)&req_us,
				 sizeof(struct cpcap_adc_us_request)))
			return -EFAULT;
		break;

	case CPCAP_IOCTL_BATT_ATOD_READ:
		req_us.format = req_async->format;
		req_us.timing = req_async->timing;
		req_us.type = req_async->type;
		req_us.status = req_async->status;
#ifdef USE_OWN_CALCULATE_METHOD
             //   printk("CPCAP_IOCTL_BATT_ATOD_READ:\n format %d\n timing %d\n type %d\n status %d\n",req_us.format , req_us.timing, req_us.type, req_us.status);
#endif
		for (i = 0; i < CPCAP_ADC_BANK0_NUM; i++)
			req_us.result[i] = req_async->result[i];

		if (copy_to_user((void *)arg, (void *)&req_us,
				 sizeof(struct cpcap_adc_us_request)))
			return -EFAULT;
		break;

	default:
		return -ENOTTY;
		break;
	}

	return ret;
}

static void cpcap_batt_ind_chrg_ctrl(struct cpcap_batt_ps *sply)
{

	unsigned long long temp;
	unsigned short cpcap_reg;
	struct cpcap_platform_data *pdata = sply->cpcap->spi->dev.platform_data;

	pr_cpcap_batt(STATUS, "batt update: capacity=%d",
		      sply->batt_state.capacity);
	pr_cpcap_batt(STATUS, "batt update: capacity_one=%d",
		      sply->batt_state.batt_capacity_one);

	if (cpcap_regacc_read(sply->cpcap, CPCAP_REG_INTS1, &cpcap_reg))
		cpcap_reg = 0;

	temp = sched_clock();
	do_div(temp, NSEC_PER_SEC);
	pr_cpcap_batt(STATUS, "batt update: time=%lld", temp);

	if ((((sply->ac_state.model == CPCAP_BATT_AC_CABLE) ||
		(sply->ac_state.model == CPCAP_BATT_AC_SMARTDOCK)) &&
			(sply->ac_state.online)) || (sply->usb_state.online)) 
									{
		if (pdata->ind_chrg->force_charge_complete != NULL)
			pdata->ind_chrg->force_charge_complete(1);
		sply->ind_chrg_dsbl_time = 0;
		pr_cpcap_batt(TRANSITION, "cable insert, chrgcmpl set");

	} else if ((sply->batt_state.batt_temp >= INDCHRG_HOT_TEMP)
		   || (sply->batt_state.batt_temp <= INDCHRG_COLD_TEMP)) {
		if (pdata->ind_chrg->force_charge_terminate != NULL)
			pdata->ind_chrg->force_charge_terminate(1);
		pr_cpcap_batt(TRANSITION, "overtemperature chrgterm set");
		sply->ind_chrg_dsbl_time = (unsigned long)temp;

	} else if ((sply->ac_state.model == CPCAP_BATT_AC_IND) &&
		   (cpcap_reg & CPCAP_BIT_VBUSOV_S)) {
		if (pdata->ind_chrg->force_charge_terminate != NULL)
			pdata->ind_chrg->force_charge_terminate(1);
		pr_cpcap_batt(TRANSITION, "overvoltage interrupt chrgterm set");
		sply->ind_chrg_dsbl_time = (unsigned long)temp;

	} else if ((sply->batt_state.batt_capacity_one >= 100) &&
		   (sply->ac_state.model == CPCAP_BATT_AC_IND)) {
		if (pdata->ind_chrg->force_charge_complete != NULL)
			pdata->ind_chrg->force_charge_complete(1);
		pr_cpcap_batt(TRANSITION, "batt capacity full, chrgcmpl set");
		sply->ind_chrg_dsbl_time = (unsigned long)temp;
#ifdef CONFIG_BLX
	} else if ((get_charginglimit() != MAX_CHARGINGLIMIT && sply->batt_state.batt_capacity_one >= get_charginglimit()) && 
		   (sply->ac_state.model == CPCAP_BATT_AC_IND)) {
			pdata->ind_chrg->force_charge_complete(1);
		pr_cpcap_batt(TRANSITION, "BLX: capacity reached, chrgcmpl set");
		pr_info("BLX: capacity reached %d, chrgcmpl set\n", get_charginglimit());
		sply->ind_chrg_dsbl_time = (unsigned long)temp;
#endif

	} else if (((temp - sply->ind_chrg_dsbl_time) >= INDCHRG_RS_TIME) ||
		   (sply->batt_state.batt_capacity_one <= INDCHRG_RS_CPCY)) {
		if (pdata->ind_chrg->force_charge_complete != NULL)
			pdata->ind_chrg->force_charge_complete(0);
		if (pdata->ind_chrg->force_charge_terminate != NULL)
			pdata->ind_chrg->force_charge_terminate(0);
		pr_cpcap_batt(TRANSITION, "batt cap low/timer, chrgcmpl clear");
	}
}

static char *cpcap_batt_chrg_models[] = {
	"none", "cable", "inductive", "smartdock"
};

static int cpcap_batt_ac_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	int ret = 0;
	struct cpcap_batt_ps *sply = container_of(psy, struct cpcap_batt_ps,
						 ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sply->ac_state.online;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = cpcap_batt_chrg_models[sply->ac_state.model];
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static char *cpcap_batt_usb_models[] = {
	"none", "usb", "factory"
};

static int cpcap_batt_usb_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int ret = 0;
	struct cpcap_batt_ps *sply = container_of(psy, struct cpcap_batt_ps,
						 usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sply->usb_state.online;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = sply->usb_state.current_now;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = cpcap_batt_usb_models[sply->usb_state.model];
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int cpcap_batt_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int ret = 0;
	struct cpcap_batt_ps *sply = container_of(psy, struct cpcap_batt_ps,
						  batt);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
#ifdef USE_OWN_CALCULATE_METHOD
		val->intval = cpcap_batt_status(sply);
#else
		val->intval = sply->batt_state.status;
#endif
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = sply->batt_state.health;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sply->batt_state.present;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef USE_OWN_CALCULATE_METHOD
		val->intval = cpcap_batt_counter(sply);
#else
		val->intval = sply->batt_state.batt_capacity;
#endif
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef USE_OWN_CALCULATE_METHOD
		val->intval = cpcap_batt_value(sply, CPCAP_ADC_BATTP)*1000;
#else
		val->intval = sply->batt_state.batt_volt;
#endif
		break;

	case POWER_SUPPLY_PROP_TEMP:
#ifdef USE_OWN_CALCULATE_METHOD
		val->intval = (cpcap_batt_value(sply, CPCAP_ADC_AD3)-273)*10;
#else
		val->intval = sply->batt_state.batt_temp;
#endif
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = sply->batt_state.batt_full_capacity;
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
#ifdef USE_OWN_CALCULATE_METHOD
		val->intval = cpcap_batt_counter(sply);
#else
		val->intval = sply->batt_state.batt_capacity_one;
#endif
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = sply->batt_state.cycle_count;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifdef USE_OWN_CALCULATE_METHOD
static int cpcap_batt_status(struct cpcap_batt_ps *sply) {
        if (sply->usb_state.online == 1 || sply->ac_state.online == 1) {
	   return POWER_SUPPLY_STATUS_CHARGING;
        } else if (cpcap_batt_counter(sply) > 95) {
	   return POWER_SUPPLY_STATUS_FULL;
        } else {
           return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

static int cpcap_batt_value(struct cpcap_batt_ps *sply, int value) {
        int i;
	struct cpcap_adc_request req;
	struct cpcap_adc_us_request req_us;

	req.format = CPCAP_ADC_FORMAT_CONVERTED;
	req.timing = CPCAP_ADC_TIMING_IMM;
	req.type = CPCAP_ADC_TYPE_BANK_0;
 
	cpcap_adc_sync_read(sply->cpcap, &req);

	req_us.status = req.status;

	for (i = 0; i < CPCAP_ADC_BANK0_NUM; i++)
	    req_us.result[i] = req.result[i];

        return req_us.result[value];
}

static int cpcap_batt_counter(struct cpcap_batt_ps *sply) {
/*
        int percent, volt_batt, range, max, min;

	min  = 3500;
        max  = 4200;
        volt_batt = cpcap_batt_value(sply, CPCAP_ADC_BATTP);
	range = (max - min) / 100;
	percent = (volt_batt - min) / range;
	if (percent > 100) percent = 100;
	if (volt_batt > 4150) percent = 100;
	if (percent < 0)   percent = 0;

	return percent;
*/

	int i, volt_batt;
	u32 cap = 0;

	volt_batt = cpcap_batt_value(sply, CPCAP_ADC_BATTP);
	printk("%s: batt_vol=%d\n",__func__, volt_batt);

	for (i=0; i < ARRAY_SIZE(tbl); i++) {
		if (volt_batt <= 3500) {
			cap = 0;
			break;
		}
		if (volt_batt >= 4181) {
			cap = 100;
			break;
		}
		if (volt_batt >= tbl[i].volt_batt) {
			if (i == (ARRAY_SIZE(tbl)-1)) {
				cap = 99;
				break;
			}
			continue;
		}
		cap = tbl[i].capacity;
		break;
	}
	printk("%s: capacity=%d\n",__func__,cap);

	return cap;
}

#if 0
void delay_ms(__u32 t)
{
    __u32 timeout = t*HZ/1000;
    	
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(timeout);
}
#endif

#define MAX_LVLS 2 // TODO Change it.
static const unsigned short percent_map[MAX_LVLS][2] = {
/* mV     percent */
{3500,0}, /*0%*/
{3501,0}, /*0%*/
{3502,0}, /*0%*/
{3503,0}, /*0%*/
{3504,0}, /*0%*/
{3505,0}, /*0%*/
{3506,0}, /*0%*/
{3507,0}, /*0%*/
{3508,0}, /*0%*/
{3509,0}, /*0%*/
{3510,0}, /*0%*/
{3511,0}, /*0%*/
{3512,0}, /*0%*/
{3513,0}, /*0%*/
{3514,0}, /*0%*/
{3515,1}, /*1%*/
{3516,1}, /*1%*/
{3517,1}, /*1%*/
{3518,1}, /*1%*/
{3519,1}, /*1%*/
{3520,1}, /*1%*/
{3521,1}, /*1%*/
{3522,1}, /*1%*/
{3523,1}, /*1%*/
{3524,1}, /*1%*/
{3525,1}, /*1%*/
{3526,1}, /*1%*/
{3527,1}, /*1%*/
{3528,1}, /*1%*/
{3529,1}, /*1%*/
{3530,2}, /*2%*/
{3531,2}, /*2%*/
{3532,2}, /*2%*/
{3533,2}, /*2%*/
{3534,2}, /*2%*/
{3535,2}, /*2%*/
{3536,2}, /*2%*/
{3537,2}, /*2%*/
{3538,2}, /*2%*/
{3539,2}, /*2%*/
{3540,2}, /*2%*/
{3541,2}, /*2%*/
{3542,2}, /*2%*/
{3543,2}, /*2%*/
{3544,2}, /*2%*/
{3545,3}, /*3%*/
{3546,3}, /*3%*/
{3547,3}, /*3%*/
{3548,3}, /*3%*/
{3549,3}, /*3%*/
{3550,3}, /*3%*/
{3551,3}, /*3%*/
{3552,3}, /*3%*/
{3553,3}, /*3%*/
{3554,3}, /*3%*/
{3555,3}, /*3%*/
{3556,3}, /*3%*/
{3557,3}, /*3%*/
{3558,3}, /*3%*/
{3559,3}, /*3%*/
{3560,4}, /*4%*/
{3561,4}, /*4%*/
{3562,4}, /*4%*/
{3563,4}, /*4%*/
{3564,4}, /*4%*/
{3565,4}, /*4%*/
{3566,4}, /*4%*/
{3567,4}, /*4%*/
{3568,4}, /*4%*/
{3569,4}, /*4%*/
{3570,4}, /*4%*/
{3571,4}, /*4%*/
{3572,4}, /*4%*/
{3573,4}, /*4%*/
{3574,4}, /*4%*/
{3575,5}, /*5%*/
{3576,5}, /*5%*/
{3577,5}, /*5%*/
{3578,5}, /*5%*/
{3579,5}, /*5%*/
{3580,5}, /*5%*/
{3581,5}, /*5%*/
{3582,5}, /*5%*/
{3583,5}, /*5%*/
{3584,5}, /*5%*/
{3585,5}, /*5%*/
{3586,5}, /*5%*/
{3587,5}, /*5%*/
{3588,5}, /*5%*/
{3589,5}, /*5%*/
{3590,6}, /*6%*/
{3591,6}, /*6%*/
{3592,6}, /*6%*/
{3593,6}, /*6%*/
{3594,6}, /*6%*/
{3595,6}, /*6%*/
{3596,6}, /*6%*/
{3597,6}, /*6%*/
{3598,6}, /*6%*/
{3599,6}, /*6%*/
{3600,6}, /*6%*/
{3601,6}, /*6%*/
{3602,6}, /*6%*/
{3603,7}, /*7%*/
{3604,7}, /*7%*/
{3605,7}, /*7%*/
{3606,7}, /*7%*/
{3607,7}, /*7%*/
{3608,7}, /*7%*/
{3609,7}, /*7%*/
{3610,7}, /*7%*/
{3611,7}, /*7%*/
{3612,7}, /*7%*/
{3613,8}, /*8%*/
{3614,8}, /*8%*/
{3615,8}, /*8%*/
{3616,8}, /*8%*/
{3617,8}, /*8%*/
{3618,8}, /*8%*/
{3619,8}, /*8%*/
{3620,8}, /*8%*/
{3621,8}, /*8%*/
{3622,8}, /*8%*/
{3623,8}, /*8%*/
{3624,9}, /*9%*/
{3625,9}, /*9%*/
{3626,9}, /*9%*/
{3627,9}, /*9%*/
{3628,9}, /*9%*/
{3629,9}, /*9%*/
{3630,9}, /*9%*/
{3631,9}, /*9%*/
{3632,9}, /*9%*/
{3633,9}, /*9%*/
{3634,9}, /*9%*/
{3635,10}, /*10%*/
{3636,10}, /*10%*/
{3637,10}, /*10%*/
{3638,10}, /*10%*/
{3639,10}, /*10%*/
{3640,10}, /*10%*/
{3641,10}, /*10%*/
{3642,10}, /*10%*/
{3643,10}, /*10%*/
{3644,10}, /*10%*/
{3645,10}, /*10%*/
{3646,11}, /*11%*/
{3647,11}, /*11%*/
{3648,11}, /*11%*/
{3649,11}, /*11%*/
{3650,11}, /*11%*/
{3651,11}, /*11%*/
{3652,12}, /*12%*/
{3653,12}, /*12%*/
{3654,12}, /*12%*/
{3655,12}, /*12%*/
{3656,12}, /*12%*/
{3657,12}, /*12%*/
{3658,13}, /*13%*/
{3659,13}, /*13%*/
{3660,13}, /*13%*/
{3661,13}, /*13%*/
{3662,13}, /*13%*/
{3663,13}, /*13%*/
{3664,14}, /*14%*/
{3665,14}, /*14%*/
{3666,14}, /*14%*/
{3667,14}, /*14%*/
{3668,14}, /*14%*/
{3669,14}, /*14%*/
{3670,15}, /*15%*/
{3671,15}, /*15%*/
{3672,15}, /*15%*/
{3673,15}, /*15%*/
{3674,15}, /*15%*/
{3675,15}, /*15%*/
{3676,16}, /*16%*/
{3677,16}, /*16%*/
{3678,16}, /*16%*/
{3679,16}, /*16%*/
{3680,16}, /*16%*/
{3681,16}, /*16%*/
{3682,17}, /*17%*/
{3683,17}, /*17%*/
{3684,17}, /*17%*/
{3685,17}, /*17%*/
{3686,17}, /*17%*/
{3687,17}, /*17%*/
{3688,18}, /*18%*/
{3689,18}, /*18%*/
{3690,18}, /*18%*/
{3691,18}, /*18%*/
{3692,18}, /*18%*/
{3693,18}, /*18%*/
{3694,19}, /*19%*/
{3695,19}, /*19%*/
{3696,19}, /*19%*/
{3697,19}, /*19%*/
{3698,19}, /*19%*/
{3699,20}, /*20%*/
{3700,20}, /*20%*/
{3701,20}, /*20%*/
{3702,20}, /*20%*/
{3703,20}, /*20%*/
{3704,20}, /*20%*/
{3705,21}, /*21%*/
{3706,21}, /*21%*/
{3707,21}, /*21%*/
{3708,21}, /*21%*/
{3709,21}, /*21%*/
{3710,21}, /*21%*/
{3711,22}, /*22%*/
{3712,22}, /*22%*/
{3713,22}, /*22%*/
{3714,22}, /*22%*/
{3715,22}, /*22%*/
{3716,22}, /*22%*/
{3717,23}, /*23%*/
{3718,23}, /*23%*/
{3719,23}, /*23%*/
{3720,23}, /*23%*/
{3721,23}, /*23%*/
{3722,23}, /*23%*/
{3723,24}, /*24%*/
{3724,24}, /*24%*/
{3725,24}, /*24%*/
{3726,24}, /*24%*/
{3727,24}, /*24%*/
{3728,24}, /*24%*/
{3729,25}, /*25%*/
{3730,25}, /*25%*/
{3731,25}, /*25%*/
{3732,25}, /*25%*/
{3733,25}, /*25%*/
{3734,25}, /*25%*/
{3735,26}, /*26%*/
{3736,26}, /*26%*/
{3737,26}, /*26%*/
{3738,26}, /*26%*/
{3739,26}, /*26%*/
{3740,26}, /*26%*/
{3741,27}, /*27%*/
{3742,27}, /*27%*/
{3743,27}, /*27%*/
{3744,27}, /*27%*/
{3745,27}, /*27%*/
{3746,27}, /*27%*/
{3747,27}, /*27%*/
{3748,28}, /*28%*/
{3749,28}, /*28%*/
{3750,28}, /*28%*/
{3751,28}, /*28%*/
{3752,28}, /*28%*/
{3753,28}, /*28%*/
{3754,29}, /*29%*/
{3755,29}, /*29%*/
{3756,29}, /*29%*/
{3757,29}, /*29%*/
{3758,29}, /*29%*/
{3759,29}, /*29%*/
{3760,29}, /*29%*/
{3761,30}, /*30%*/
{3762,30}, /*30%*/
{3763,30}, /*30%*/
{3764,30}, /*30%*/
{3765,30}, /*30%*/
{3766,30}, /*30%*/
{3767,30}, /*30%*/
{3768,31}, /*31%*/
{3769,31}, /*31%*/
{3770,31}, /*31%*/
{3771,31}, /*31%*/
{3772,31}, /*31%*/
{3773,31}, /*31%*/
{3774,32}, /*32%*/
{3775,32}, /*32%*/
{3776,32}, /*32%*/
{3777,32}, /*32%*/
{3778,32}, /*32%*/
{3779,32}, /*32%*/
{3780,33}, /*33%*/
{3781,33}, /*33%*/
{3782,33}, /*33%*/
{3783,33}, /*33%*/
{3784,33}, /*33%*/
{3785,33}, /*33%*/
{3786,34}, /*34%*/
{3787,34}, /*34%*/
{3788,34}, /*34%*/
{3789,34}, /*34%*/
{3790,34}, /*34%*/
{3791,34}, /*34%*/
{3792,35}, /*35%*/
{3793,35}, /*35%*/
{3794,35}, /*35%*/
{3795,35}, /*35%*/
{3796,35}, /*35%*/
{3797,35}, /*35%*/
{3798,36}, /*36%*/
{3799,36}, /*36%*/
{3800,36}, /*36%*/
{3801,36}, /*36%*/
{3802,36}, /*36%*/
{3803,36}, /*36%*/
{3804,37}, /*37%*/
{3805,37}, /*37%*/
{3806,37}, /*37%*/
{3807,37}, /*37%*/
{3808,37}, /*37%*/
{3809,37}, /*37%*/
{3810,38}, /*38%*/
{3811,38}, /*38%*/
{3812,38}, /*38%*/
{3813,38}, /*38%*/
{3814,38}, /*38%*/
{3815,38}, /*38%*/
{3816,39}, /*39%*/
{3817,39}, /*39%*/
{3818,39}, /*39%*/
{3819,39}, /*39%*/
{3820,39}, /*39%*/
{3821,39}, /*39%*/
{3822,40}, /*40%*/
{3823,40}, /*40%*/
{3824,40}, /*40%*/
{3825,40}, /*40%*/
{3826,40}, /*40%*/
{3827,40}, /*40%*/
{3828,41}, /*41%*/
{3829,41}, /*41%*/
{3830,41}, /*41%*/
{3831,41}, /*41%*/
{3832,41}, /*41%*/
{3833,41}, /*41%*/
{3834,42}, /*42%*/
{3835,42}, /*42%*/
{3836,42}, /*42%*/
{3837,42}, /*42%*/
{3838,42}, /*42%*/
{3839,42}, /*42%*/
{3840,43}, /*43%*/
{3841,43}, /*43%*/
{3842,43}, /*43%*/
{3843,43}, /*43%*/
{3844,43}, /*43%*/
{3845,43}, /*43%*/
{3846,44}, /*44%*/
{3847,44}, /*44%*/
{3848,44}, /*44%*/
{3849,44}, /*44%*/
{3850,44}, /*44%*/
{3851,44}, /*44%*/
{3852,44}, /*44%*/
{3853,45}, /*45%*/
{3854,45}, /*45%*/
{3855,45}, /*45%*/
{3856,45}, /*45%*/
{3857,45}, /*45%*/
{3858,45}, /*45%*/
{3859,46}, /*46%*/
{3860,46}, /*46%*/
{3861,46}, /*46%*/
{3862,46}, /*46%*/
{3863,46}, /*46%*/
{3864,46}, /*46%*/
{3865,47}, /*47%*/
{3866,47}, /*47%*/
{3867,47}, /*47%*/
{3868,47}, /*47%*/
{3869,47}, /*47%*/
{3870,47}, /*47%*/
{3871,47}, /*47%*/
{3872,48}, /*48%*/
{3873,48}, /*48%*/
{3874,48}, /*48%*/
{3875,48}, /*48%*/
{3876,48}, /*48%*/
{3877,48}, /*48%*/
{3878,48}, /*48%*/
{3879,49}, /*49%*/
{3880,49}, /*49%*/
{3881,49}, /*49%*/
{3882,49}, /*49%*/
{3883,49}, /*49%*/
{3884,49}, /*49%*/
{3885,50}, /*50%*/
{3886,50}, /*50%*/
{3887,50}, /*50%*/
{3888,50}, /*50%*/
{3889,50}, /*50%*/
{3890,50}, /*50%*/
{3891,51}, /*51%*/
{3892,51}, /*51%*/
{3893,51}, /*51%*/
{3894,51}, /*51%*/
{3895,51}, /*51%*/
{3896,51}, /*51%*/
{3897,52}, /*52%*/
{3898,52}, /*52%*/
{3899,52}, /*52%*/
{3900,52}, /*52%*/
{3901,52}, /*52%*/
{3902,52}, /*52%*/
{3903,53}, /*53%*/
{3904,53}, /*53%*/
{3905,53}, /*53%*/
{3906,53}, /*53%*/
{3907,53}, /*53%*/
{3908,53}, /*53%*/
{3909,54}, /*54%*/
{3910,54}, /*54%*/
{3911,54}, /*54%*/
{3912,54}, /*54%*/
{3913,54}, /*54%*/
{3914,54}, /*54%*/
{3915,55}, /*55%*/
{3916,55}, /*55%*/
{3917,55}, /*55%*/
{3918,55}, /*55%*/
{3919,55}, /*55%*/
{3920,55}, /*55%*/
{3921,56}, /*56%*/
{3922,56}, /*56%*/
{3923,56}, /*56%*/
{3924,56}, /*56%*/
{3925,56}, /*56%*/
{3926,56}, /*56%*/
{3927,57}, /*57%*/
{3928,57}, /*57%*/
{3929,57}, /*57%*/
{3930,57}, /*57%*/
{3931,57}, /*57%*/
{3932,57}, /*57%*/
{3933,58}, /*58%*/
{3934,58}, /*58%*/
{3935,58}, /*58%*/
{3936,58}, /*58%*/
{3937,58}, /*58%*/
{3938,58}, /*58%*/
{3939,59}, /*59%*/
{3940,59}, /*59%*/
{3941,59}, /*59%*/
{3942,59}, /*59%*/
{3943,59}, /*59%*/
{3944,59}, /*59%*/
{3945,60}, /*60%*/
{3946,60}, /*60%*/
{3947,60}, /*60%*/
{3948,60}, /*60%*/
{3949,60}, /*60%*/
{3950,60}, /*60%*/
{3951,61}, /*61%*/
{3952,61}, /*61%*/
{3953,61}, /*61%*/
{3954,61}, /*61%*/
{3955,61}, /*61%*/
{3956,61}, /*61%*/
{3957,62}, /*62%*/
{3958,62}, /*62%*/
{3959,62}, /*62%*/
{3960,62}, /*62%*/
{3961,62}, /*62%*/
{3962,62}, /*62%*/
{3963,63}, /*63%*/
{3964,63}, /*63%*/
{3965,63}, /*63%*/
{3966,63}, /*63%*/
{3967,63}, /*63%*/
{3968,63}, /*63%*/
{3969,64}, /*64%*/
{3970,64}, /*64%*/
{3971,64}, /*64%*/
{3972,64}, /*64%*/
{3973,64}, /*64%*/
{3974,64}, /*64%*/
{3975,65}, /*65%*/
{3976,65}, /*65%*/
{3977,65}, /*65%*/
{3978,65}, /*65%*/
{3979,65}, /*65%*/
{3980,65}, /*65%*/
{3981,66}, /*66%*/
{3982,66}, /*66%*/
{3983,66}, /*66%*/
{3984,66}, /*66%*/
{3985,66}, /*66%*/
{3986,66}, /*66%*/
{3987,67}, /*67%*/
{3988,67}, /*67%*/
{3989,67}, /*67%*/
{3990,67}, /*67%*/
{3991,67}, /*67%*/
{3992,67}, /*67%*/
{3993,68}, /*68%*/
{3994,68}, /*68%*/
{3995,68}, /*68%*/
{3996,68}, /*68%*/
{3997,68}, /*68%*/
{3998,68}, /*68%*/
{3999,69}, /*69%*/
{4000,69}, /*69%*/
{4001,69}, /*69%*/
{4002,69}, /*69%*/
{4003,69}, /*69%*/
{4004,69}, /*69%*/
{4005,70}, /*70%*/
{4006,70}, /*70%*/
{4007,70}, /*70%*/
{4008,70}, /*70%*/
{4009,70}, /*70%*/
{4010,70}, /*70%*/
{4011,71}, /*71%*/
{4012,71}, /*71%*/
{4013,71}, /*71%*/
{4014,71}, /*71%*/
{4015,71}, /*71%*/
{4016,71}, /*71%*/
{4017,72}, /*72%*/
{4018,72}, /*72%*/
{4019,72}, /*72%*/
{4020,72}, /*72%*/
{4021,72}, /*72%*/
{4022,72}, /*72%*/
{4023,73}, /*73%*/
{4024,73}, /*73%*/
{4025,73}, /*73%*/
{4026,73}, /*73%*/
{4027,73}, /*73%*/
{4028,73}, /*73%*/
{4029,74}, /*74%*/
{4030,74}, /*74%*/
{4031,74}, /*74%*/
{4032,74}, /*74%*/
{4033,74}, /*74%*/
{4034,74}, /*74%*/
{4035,75}, /*75%*/
{4036,75}, /*75%*/
{4037,75}, /*75%*/
{4038,75}, /*75%*/
{4039,75}, /*75%*/
{4040,75}, /*75%*/
{4041,76}, /*76%*/
{4042,76}, /*76%*/
{4043,76}, /*76%*/
{4044,76}, /*76%*/
{4045,76}, /*76%*/
{4046,76}, /*76%*/
{4047,77}, /*77%*/
{4048,77}, /*77%*/
{4049,77}, /*77%*/
{4050,77}, /*77%*/
{4051,77}, /*77%*/
{4052,77}, /*77%*/
{4053,78}, /*78%*/
{4054,78}, /*78%*/
{4055,78}, /*78%*/
{4056,78}, /*78%*/
{4057,78}, /*78%*/
{4058,78}, /*78%*/
{4059,79}, /*79%*/
{4060,79}, /*79%*/
{4061,79}, /*79%*/
{4062,79}, /*79%*/
{4063,79}, /*79%*/
{4064,79}, /*79%*/
{4065,80}, /*80%*/
{4066,80}, /*80%*/
{4067,80}, /*80%*/
{4068,80}, /*80%*/
{4069,80}, /*80%*/
{4070,80}, /*80%*/
{4071,81}, /*81%*/
{4072,81}, /*81%*/
{4073,81}, /*81%*/
{4074,81}, /*81%*/
{4075,81}, /*81%*/
{4076,81}, /*81%*/
{4077,82}, /*82%*/
{4078,82}, /*82%*/
{4079,82}, /*82%*/
{4080,82}, /*82%*/
{4081,82}, /*82%*/
{4082,82}, /*82%*/
{4083,83}, /*83%*/
{4084,83}, /*83%*/
{4085,83}, /*83%*/
{4086,83}, /*83%*/
{4087,83}, /*83%*/
{4088,83}, /*83%*/
{4089,84}, /*84%*/
{4090,84}, /*84%*/
{4091,84}, /*84%*/
{4092,84}, /*84%*/
{4093,84}, /*84%*/
{4094,84}, /*84%*/
{4095,85}, /*85%*/
{4096,85}, /*85%*/
{4097,85}, /*85%*/
{4098,85}, /*85%*/
{4099,85}, /*85%*/
{4100,85}, /*85%*/
{4101,86}, /*86%*/
{4102,86}, /*86%*/
{4103,86}, /*86%*/
{4104,86}, /*86%*/
{4105,86}, /*86%*/
{4106,86}, /*86%*/
{4107,87}, /*87%*/
{4108,87}, /*87%*/
{4109,87}, /*87%*/
{4110,87}, /*87%*/
{4111,87}, /*87%*/
{4112,87}, /*87%*/
{4113,88}, /*88%*/
{4114,88}, /*88%*/
{4115,88}, /*88%*/
{4116,88}, /*88%*/
{4117,88}, /*88%*/
{4118,88}, /*88%*/
{4119,89}, /*89%*/
{4120,89}, /*89%*/
{4121,89}, /*89%*/
{4122,89}, /*89%*/
{4123,89}, /*89%*/
{4124,89}, /*89%*/
{4125,89}, /*89%*/
{4126,90}, /*90%*/
{4127,90}, /*90%*/
{4128,90}, /*90%*/
{4129,90}, /*90%*/
{4130,90}, /*90%*/
{4131,91}, /*91%*/
{4132,91}, /*91%*/
{4133,91}, /*91%*/
{4134,91}, /*91%*/
{4135,91}, /*91%*/
{4136,91}, /*91%*/
{4137,92}, /*92%*/
{4138,92}, /*92%*/
{4139,92}, /*92%*/
{4140,92}, /*92%*/
{4141,92}, /*92%*/
{4142,92}, /*92%*/
{4143,93}, /*93%*/
{4144,93}, /*93%*/
{4145,93}, /*93%*/
{4146,93}, /*93%*/
{4147,93}, /*93%*/
{4148,93}, /*93%*/
{4149,94}, /*94%*/
{4150,94}, /*94%*/
{4151,94}, /*94%*/
{4152,94}, /*94%*/
{4153,94}, /*94%*/
{4154,94}, /*94%*/
{4155,95}, /*95%*/
{4156,95}, /*95%*/
{4157,95}, /*95%*/
{4158,95}, /*95%*/
{4159,95}, /*95%*/
{4160,95}, /*95%*/
{4161,96}, /*96%*/
{4162,96}, /*96%*/
{4163,96}, /*96%*/
{4164,96}, /*96%*/
{4165,96}, /*96%*/
{4166,96}, /*96%*/
{4167,96}, /*96%*/
{4168,97}, /*97%*/
{4169,97}, /*97%*/
{4170,97}, /*97%*/
{4171,97}, /*97%*/
{4172,97}, /*97%*/
{4173,97}, /*97%*/
{4174,98}, /*98%*/
{4175,98}, /*98%*/
{4176,98}, /*98%*/
{4177,98}, /*98%*/
{4178,98}, /*98%*/
{4179,98}, /*98%*/
{4180,99}, /*99%*/
{4181,99}, /*99%*/
{4182,99}, /*99%*/
{4183,99}, /*99%*/
{4184,99}, /*99%*/
{4185,99}, /*99%*/
{4186,100}, /*100%*/
{4187,100}, /*100%*/
{4188,100}, /*100%*/
{4189,100}, /*100%*/
{4190,100}, /*100%*/
{4191,100}, /*100%*/
{4192,100}, /*100%*/
{4193,100}, /*100%*/
{4194,100}, /*100%*/
{4195,100}, /*100%*/
{4196,100}, /*100%*/
{4197,100}, /*100%*/
{4198,100}, /*100%*/
{4199,100}, /*100%*/
{4200,100}, /*100%*/
};
#if 0
static int cpcap_batt_monitor(void* arg) {


        int i, ret, percent, volt_batt, range, max, min;
	unsigned short value;
	struct cpcap_batt_ps *sply = cpcap_batt_sply;
	struct cpcap_adc_request req;
	struct cpcap_adc_us_request req_us;
        struct cpcap_adc_phase phase;
/*	cpcap_regacc_write(sply->cpcap, CPCAP_REG_CRM, CPCAP_BIT_CHRG_LED_EN, CPCAP_BIT_CHRG_LED_EN); //Enable charge led
	cpcap_regacc_write(sply->cpcap, CPCAP_REG_USBC2, CPCAP_BIT_USBXCVREN, CPCAP_BIT_USBXCVREN);
	cpcap_regacc_write(sply->cpcap, CPCAP_REG_CRM, CPCAP_BIT_RVRSMODE, CPCAP_BIT_RVRSMODE);
	cpcap_regacc_write(sply->cpcap, CPCAP_REG_CRM, CPCAP_BIT_VCHRG0, CPCAP_BIT_VCHRG0);
*/
   while (1) {  //TODO: Need split this big function

/*
//Before start battd
CPCAP_REG_CRM 784
CPCAP_REG_CCM 0
CPCAP_REG_USBC1 4608
CPCAP_REG_USBC2 49184
CPCAP_MACRO_7 0, 8 0, 9 0, 10 0, 11 0, 12 0
//After star battd
CPCAP_REG_CRM 849 = 0x351

CPCAP_REG_CCM 1006 = 0x3EE
CPCAP_MACRO_7 0, 8 0, 9 1, 10 0, 11 0, 12 1
*/
/*
	   printk("****Battery Phasing start ****\n");
	   phase.offset_batti = -1;
	   phase.slope_batti = 128;
	   phase.offset_chrgi = 0;
	   phase.slope_chrgi = 126;
	   phase.offset_battp = 14;
	   phase.slope_battp = 128;
	   phase.offset_bp = 0;
	   phase.slope_bp = 128;
	   phase.offset_battt = 3;
	   phase.slope_battt = 129;
	   phase.offset_chrgv = -4;

	   cpcap_adc_phase(sply->cpcap, &phase);
	   printk("****Battery Phasing end ****\n");
*/
//For start Macros 7 we need phasing.
//        if (!cpcap_uc_status(sply->cpcap, CPCAP_MACRO_7)){

           //sply->irq_status |= CPCAP_BATT_IRQ_MACRO;  This IRQ Called after start Marco 7 by cpcap_batt_irq_hdlr.
/*
           cpcap_uc_start(sply->cpcap, CPCAP_MACRO_7);
           cpcap_uc_start(sply->cpcap, CPCAP_MACRO_9);
           cpcap_uc_start(sply->cpcap, CPCAP_MACRO_12);
	   cpcap_regacc_write(sply->cpcap, CPCAP_REG_CRM, 0x351, 0x351);
	   cpcap_regacc_write(sply->cpcap, CPCAP_REG_CCM, 0x3EE, 0x3EE);
           cpcap_regacc_write(sply->cpcap, CPCAP_REG_CRM, CPCAP_BIT_CHRG_LED_EN, CPCAP_BIT_CHRG_LED_EN); //Enable charge led

           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CCC1, &value);
	   printk("CPCAP_REG_CCC1 %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CRM, &value);
	   printk("CPCAP_REG_CRM %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CCCC2, &value);
	   printk("CPCAP_REG_CCCC2 %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CCM, &value);
	   printk("CPCAP_REG_CCM %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CCA1, &value);
	   printk("CPCAP_REG_CCA1 %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CCA2, &value);
	   printk("CPCAP_REG_CCA2 %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CCO, &value);
	   printk("CPCAP_REG_CC0 %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_CCI, &value);
	   printk("CPCAP_REG_CCI %d \n",value);
           cpcap_regacc_read(sply->cpcap, CPCAP_REG_USBC1, &value);
	   printk("CPCAP_REG_USBC1 %d \n",value);

           cpcap_regacc_read(sply->cpcap, CPCAP_REG_USBC2, &value);
	   printk("CPCAP_REG_USBC2 %d \n",value);

           printk("CPCAP_MACRO_7 %d, 8 %d, 9 %d, 10 %d, 11 %d, 12 %d\n",
            cpcap_uc_status(sply->cpcap, CPCAP_MACRO_7), cpcap_uc_status(sply->cpcap,CPCAP_MACRO_8), cpcap_uc_status(sply->cpcap,CPCAP_MACRO_9),cpcap_uc_status(sply->cpcap,CPCAP_MACRO_10), cpcap_uc_status(sply->cpcap,CPCAP_MACRO_11),cpcap_uc_status(sply->cpcap,CPCAP_MACRO_12));

       // printk("ac_state.online: %d\n",sply->ac_state.online);
        //printk("usb_state.online: %d\n",sply->usb_state.online);
    	printk("Result Voltage: %dmV\n",sply->batt_state.batt_volt/1000);
    	printk("Result Temp: %d*C\n",sply->batt_state.batt_temp/10);
        printk("batt_state.status: %d\n",sply->batt_state.status);


//Getting values from cpcap.

      //  sply->batt_state.batt_volt = req_us.result[CPCAP_ADC_BATTP]*1000;
      //  sply->batt_state.batt_temp = (req_us.result[CPCAP_ADC_AD3]-273)*10;  //cpcap report temp in kelvins !!!not accurately!!!

        //printk("CPCAP_IOCTL_BATT_ATOD_SYNC:\n format %d\n timing %d\n type %d\n status %d\n",req.format , req.timing, req.type, req.status);

	//printk("Dump of CPCAP_ADC_BANK0_NUM:\n CPCAP_ADC_VBUS:%d\n CPCAP_ADC_AD3:%d\n CPCAP_ADC_BPLUS_AD4:%d\n CPCAP_ADC_CHG_ISENSE:%d\n CPCAP_ADC_BATTI_ADC:%d\n CPCAP_ADC_USB_ID:%d\n",
        //                   req_us.result[CPCAP_ADC_VBUS],req_us.result[CPCAP_ADC_AD3],req_us.result[CPCAP_ADC_BPLUS_AD4],req_us.result[CPCAP_ADC_CHG_ISENSE],
        //                       req_us.result[CPCAP_ADC_BATTI_ADC],req_us.result[CPCAP_ADC_USB_ID]);


//Calculate Percent like in bootmenu. TODO: Replace formula with TABLE.

	power_supply_changed(&sply->batt);

	printk("Result percent: %d\n",sply->batt_state.capacity);

        delay_ms(10000);
*/
  }

 return 0;
}
#endif
#endif

static int set_timestamp(const char *val, const struct kernel_param *kp)
{
	int return_val = param_set_int(val, kp);

	if (return_val)
		return return_val;
	else
		return 0;
}

static int set_cc_counter(const char *val, const struct kernel_param *kp)
{
	int return_val = param_set_ulong(val, kp);

	if (return_val)
		return return_val;
	else
		return 0;
}


static int set_cc_counter_percentage(const char *val,
			const struct kernel_param *kp)
{
	int return_val = param_set_ushort(val, kp);

	if (return_val)
		return return_val;
	else
		return 0;
}

static int cpcap_batt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct cpcap_batt_ps *sply;
	struct cpcap_platform_data *pdata;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		ret = -EINVAL;
		goto prb_exit;
	}

	sply = kzalloc(sizeof(struct cpcap_batt_ps), GFP_KERNEL);
	if (sply == NULL) {
		ret = -ENOMEM;
		goto prb_exit;
	}

	sply->cpcap = pdev->dev.platform_data;
	mutex_init(&sply->lock);
	init_waitqueue_head(&sply->wait);

	sply->batt_state.status	= POWER_SUPPLY_STATUS_DISCHARGING; //Set discharging by default.  //POWER_SUPPLY_STATUS_UNKNOWN;
	sply->batt_state.health	= POWER_SUPPLY_HEALTH_GOOD;
	sply->batt_state.present = 1;
	sply->batt_state.capacity = 100;	/* Percentage */
	sply->batt_state.batt_volt = 4200000;	/* uV */
	sply->batt_state.batt_temp = 230;	/* tenths of degrees Celsius */
	sply->batt_state.batt_full_capacity = 0;
	sply->batt_state.batt_capacity_one = 99;
	sply->batt_state.cycle_count = 100;	/* Percentage */

	sply->ac_state.online = 0;
	sply->ac_state.model = CPCAP_BATT_AC_NONE;

	sply->usb_state.online = 0;
	sply->usb_state.current_now = 0;
	sply->usb_state.model = CPCAP_BATT_USB_MODEL_NONE;

	sply->batt.properties = cpcap_batt_props;
	sply->batt.num_properties = ARRAY_SIZE(cpcap_batt_props);
	sply->batt.get_property = cpcap_batt_get_property;
	sply->batt.name = "battery";
	sply->batt.type = POWER_SUPPLY_TYPE_BATTERY;

	sply->ac.properties = cpcap_batt_ac_props;
	sply->ac.num_properties = ARRAY_SIZE(cpcap_batt_ac_props);
	sply->ac.get_property = cpcap_batt_ac_get_property;
	sply->ac.name = "ac";
	sply->ac.type = POWER_SUPPLY_TYPE_MAINS;

	sply->usb.properties = cpcap_batt_usb_props;
	sply->usb.num_properties = ARRAY_SIZE(cpcap_batt_usb_props);
	sply->usb.get_property = cpcap_batt_usb_get_property;
	sply->usb.name = "usb";
	sply->usb.type = POWER_SUPPLY_TYPE_USB;

	sply->no_update = false;

	pdata = sply->cpcap->spi->dev.platform_data;

	ret = power_supply_register(&pdev->dev, &sply->ac);
	if (ret)
		goto prb_exit;
	ret = power_supply_register(&pdev->dev, &sply->batt);
	if (ret)
		goto unregac_exit;
	ret = power_supply_register(&pdev->dev, &sply->usb);
	if (ret)
		goto unregbatt_exit;
	platform_set_drvdata(pdev, sply);
	sply->cpcap->battdata = sply;
	cpcap_batt_sply = sply;

	ret = misc_register(&batt_dev);
	if (ret)
		goto unregusb_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_VBUSOV,
				 cpcap_batt_irq_hdlr, sply);
	if (ret)
		goto unregmisc_exit;
	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_BATTDETB,
				 cpcap_batt_irq_hdlr, sply);
	if (ret)
		goto unregirq_exit;
	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_CC_CAL,
				 cpcap_batt_irq_hdlr, sply);
	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11);

	if (ret)
		goto unregirq_exit;

	goto prb_exit;


unregirq_exit:
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_VBUSOV);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_BATTDETB);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_CC_CAL);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11);
unregmisc_exit:
	misc_deregister(&batt_dev);
unregusb_exit:
	power_supply_unregister(&sply->usb);
unregbatt_exit:
	power_supply_unregister(&sply->batt);
unregac_exit:
	power_supply_unregister(&sply->ac);

prb_exit:
#ifdef USE_OWN_CALCULATE_METHOD
// batt_task = kthread_create(cpcap_batt_monitor, (void*)0, "cpcap_batt_monitor");
//wake_up_process(batt_task);
#endif
	return ret;
}

static int cpcap_batt_remove(struct platform_device *pdev)
{
	struct cpcap_batt_ps *sply = platform_get_drvdata(pdev);
	struct cpcap_platform_data *pdata = sply->cpcap->spi->dev.platform_data;

	power_supply_unregister(&sply->batt);
	power_supply_unregister(&sply->ac);
	power_supply_unregister(&sply->usb);
	misc_deregister(&batt_dev);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_VBUSOV);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_BATTDETB);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_CC_CAL);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11);
	sply->cpcap->battdata = NULL;
	kfree(sply);

	return 0;
}

static int cpcap_batt_resume(struct platform_device *pdev)
{
	struct cpcap_batt_ps *sply = platform_get_drvdata(pdev);
	struct cpcap_platform_data *pdata = sply->cpcap->spi->dev.platform_data;
	unsigned long cur_time;
	unsigned long long temp;

	temp = sched_clock();
	do_div(temp, NSEC_PER_SEC);
	cur_time = ((unsigned long)temp);
	if ((cur_time - sply->last_run_time) < 0)
		sply->last_run_time = 0;

	if ((cur_time - sply->last_run_time) > 50) {
		mutex_lock(&sply->lock);
		sply->data_pending = 1;
		sply->irq_status |= CPCAP_BATT_IRQ_MACRO;

		mutex_unlock(&sply->lock);

		wake_up_interruptible(&sply->wait);
	}

	cpcap_batt_ind_chrg_ctrl(sply);

	return 0;
}
void cpcap_batt_set_ac_prop(struct cpcap_device *cpcap,
	struct cpcap_batt_ac_data *ac)
{
	struct cpcap_batt_ps *sply = cpcap->battdata;
	struct spi_device *spi = cpcap->spi;
	struct cpcap_platform_data *data = spi->dev.platform_data;

	if (sply != NULL) {
		sply->ac_state.online = ac->online;
		sply->ac_state.model = ac->model;
		power_supply_changed(&sply->ac);

		if (data->ac_changed)
			data->ac_changed(&sply->ac, &sply->ac_state);
	}

	cpcap_batt_ind_chrg_ctrl(sply);
}
EXPORT_SYMBOL(cpcap_batt_set_ac_prop);

void cpcap_batt_set_usb_prop_online(struct cpcap_device *cpcap, int online,
				    enum cpcap_batt_usb_model model)
{
	struct cpcap_batt_ps *sply = cpcap->battdata;
	struct spi_device *spi = cpcap->spi;
	struct cpcap_platform_data *data = spi->dev.platform_data;

	if (sply != NULL) {

		sply->usb_state.online = online;
		sply->usb_state.model = model;
		power_supply_changed(&sply->usb);

		if (data->usb_changed)
			data->usb_changed(&sply->usb, &sply->usb_state);
	}

	cpcap_batt_ind_chrg_ctrl(sply);
}
EXPORT_SYMBOL(cpcap_batt_set_usb_prop_online);

void cpcap_batt_set_usb_prop_curr(struct cpcap_device *cpcap, unsigned int curr)
{
	struct cpcap_batt_ps *sply = cpcap->battdata;
	struct spi_device *spi = cpcap->spi;
	struct cpcap_platform_data *data = spi->dev.platform_data;

	if (sply != NULL) {

		sply->usb_state.current_now = curr;
		power_supply_changed(&sply->usb);

		if (data->usb_changed)
			data->usb_changed(&sply->usb, &sply->usb_state);
	}
}
EXPORT_SYMBOL(cpcap_batt_set_usb_prop_curr);

/*
 * Debugfs interface to test how system works with different values of
 * the battery properties. Once the propety value is set through the
 * debugfs, updtes from the drivers will be discarded.
 */
#ifdef CONFIG_DEBUG_FS

static int cpcap_batt_debug_set(void *prop, u64 val)
{
	int data = (int)val;
	enum power_supply_property psp = (enum power_supply_property)prop;
	struct cpcap_batt_ps *sply = cpcap_batt_sply;
	bool changed = true;
	sply->no_update = true;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		sply->batt_state.status = data;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		sply->batt_state.health = data;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		sply->batt_state.present = data;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		sply->batt_state.capacity = data;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		sply->batt_state.batt_volt = data;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		sply->batt_state.batt_temp = data;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		sply->batt_state.batt_full_capacity = data;
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		sply->batt_state.batt_capacity_one = data;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		sply->batt_state.cycle_count = data;
		break;

	default:
		changed = false;
		break;
	}

	if (changed)
		power_supply_changed(&sply->batt);

	return 0;
}

static int cpcap_batt_debug_get(void *prop, u64 *val)
{
	enum power_supply_property psp = (enum power_supply_property)prop;
	struct cpcap_batt_ps *sply = cpcap_batt_sply;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		*val = sply->batt_state.status;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		*val = sply->batt_state.health;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		*val = sply->batt_state.present;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		*val = sply->batt_state.capacity;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		*val = sply->batt_state.batt_volt;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		*val = sply->batt_state.batt_temp;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		*val = sply->batt_state.batt_full_capacity;
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		*val = sply->batt_state.batt_capacity_one;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		*val = sply->batt_state.cycle_count;
		break;

	default:
		break;
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpcap_battery_fops, cpcap_batt_debug_get,
			cpcap_batt_debug_set, "%llu\n");

static int __init cpcap_batt_debug_init(void)
{
	struct dentry *dent = debugfs_create_dir("battery", 0);
	int            ret  = 0;

	if (!IS_ERR(dent)) {
		debugfs_create_file("status", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_STATUS, &cpcap_battery_fops);
		debugfs_create_file("health", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_HEALTH, &cpcap_battery_fops);
		debugfs_create_file("present", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_PRESENT, &cpcap_battery_fops);
		debugfs_create_file("voltage", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_VOLTAGE_NOW, &cpcap_battery_fops);
		debugfs_create_file("capacity", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_CAPACITY, &cpcap_battery_fops);
		debugfs_create_file("temp", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_TEMP, &cpcap_battery_fops);
		debugfs_create_file("charge_full_design", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
		  &cpcap_battery_fops);
		debugfs_create_file("charge_counter", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_CHARGE_COUNTER,
		  &cpcap_battery_fops);
		debugfs_create_file("cycle_count", 0666, dent,
		  (void *)POWER_SUPPLY_PROP_CYCLE_COUNT,
		  &cpcap_battery_fops);
	} else {
		ret = PTR_ERR(dent);
	}

	return ret;
}

late_initcall(cpcap_batt_debug_init);

#endif /* CONFIG_DEBUG_FS */

static int __init cpcap_batt_init(void)
{
	return platform_driver_register(&cpcap_batt_driver);
}
subsys_initcall(cpcap_batt_init);

static void __exit cpcap_batt_exit(void)
{
	platform_driver_unregister(&cpcap_batt_driver);
}
module_exit(cpcap_batt_exit);

MODULE_ALIAS("platform:cpcap_batt");
MODULE_DESCRIPTION("CPCAP BATTERY driver");
MODULE_AUTHOR("Motorola, Quarx, dtrail");
MODULE_LICENSE("GPL");
