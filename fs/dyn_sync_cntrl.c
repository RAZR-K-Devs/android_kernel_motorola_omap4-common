/*
 * Author: Paul Reioux aka Faux123 <reioux@gmail.com>
 * Reworked and fixed: Ivan Cerrato aka pengus77 <ivan@cerrato.biz>
 *
 * Copyright 2012 Paul Reioux
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/earlysuspend.h>
#include <linux/reboot.h>
#include <linux/writeback.h>
#include <linux/workqueue.h>

#define DYN_FSYNC_VERSION 2
extern bool dyn_async;
extern void sync_filesystems(int wait);

struct delayed_work fsync_suspend_work;

bool dyn_fsync_force_off = false;
bool dyn_fsync_can_sync = false;
bool dyn_fsync_active = false;
bool dyn_fsync_was_active = false;
bool suspended = false;

static void fsync_suspend_work_fn(struct work_struct *work);

static void dyn_fsync_flush(bool force)
{
	if (force)
		dyn_fsync_can_sync = true;

	if (dyn_fsync_active && dyn_fsync_can_sync)
	{
		pr_info("%s: flushing all outstanding buffers\n", __FUNCTION__);
		wakeup_flusher_threads(0);
		sync_filesystems(0);
		sync_filesystems(1);
	}
}

static ssize_t dyn_fsync_active_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", (dyn_fsync_active ? 1 : 0));
}

void dyn_fsync_enabled(bool enabled)
{
	if (enabled) {
		dyn_fsync_can_sync = false;
		dyn_fsync_active = true;
	} else {
		dyn_fsync_flush(true);
		dyn_fsync_active = false;
	}
}
EXPORT_SYMBOL(dyn_fsync_enabled);

static ssize_t dyn_fsync_active_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;

	if (dyn_fsync_force_off)
		pr_info("%s: dynamic fsync locked - commands will be stored and replied ( when charge > 5%% )\n", __FUNCTION__);

	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data == 1 && !dyn_fsync_active) {
			if (dyn_fsync_force_off) {
				pr_info("%s: dynamic fsync will be enabled\n", __FUNCTION__);
				dyn_fsync_was_active = true;
			} else {
				pr_info("%s: dynamic fsync ENABLED - async fsync DISABLED\n", __FUNCTION__);
				dyn_fsync_enabled(true);
				dyn_async = false;
				if (suspended)
					schedule_delayed_work_on(0, &fsync_suspend_work, 1*HZ);
			}
		}
		else if (data == 0 && dyn_fsync_active) {
			if (dyn_fsync_force_off) {
				pr_info("%s: dynamic fsync will be disabled\n", __FUNCTION__);
				dyn_fsync_was_active = false;
			} else {
				pr_info("%s: dynamic fsync DISABLED - async fsync ENABLED\n", __FUNCTION__);
				dyn_fsync_enabled(false);
				dyn_async = true;
			}
		}
	} else
		pr_info("%s: unknown input!\n", __FUNCTION__);

	return count;
}

static ssize_t dyn_fsync_locked_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dyn_fsync_force_off ? 1 : 0);
}

static ssize_t dyn_fsync_future_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dyn_fsync_was_active ? 1 : 0);
}

static struct kobj_attribute dyn_fsync_active_attribute = 
	__ATTR(Dyn_fsync_active, 0666, dyn_fsync_active_show, dyn_fsync_active_store);

static struct kobj_attribute dyn_fsync_locked_attribute =
	__ATTR(dyn_fsync_locked, 0444 , dyn_fsync_locked_show, NULL);

static struct kobj_attribute dyn_fsync_future_attribute =
	__ATTR(dyn_fsync_future_state, 0444 , dyn_fsync_future_show, NULL);

static struct attribute *dyn_fsync_active_attrs[] =
	{
		&dyn_fsync_active_attribute.attr,
		&dyn_fsync_locked_attribute.attr,
		&dyn_fsync_future_attribute.attr,
		NULL,
	};

static struct attribute_group dyn_fsync_active_attr_group =
	{
		.attrs = dyn_fsync_active_attrs,
	};

static struct kobject *dyn_fsync_kobj;

static void fsync_suspend_work_fn(struct work_struct *work)
{
	dyn_fsync_flush(true);
}

static void dyn_fsync_suspend(struct early_suspend *h)
{
	if (dyn_fsync_active)
		schedule_delayed_work_on(0, &fsync_suspend_work, 3*HZ);

	suspended = true;
}

static void dyn_fsync_resume(struct early_suspend *h)
{
	if (dyn_fsync_active) {
		cancel_delayed_work(&fsync_suspend_work);
		dyn_fsync_can_sync = false;
	}

	suspended = false;
}

static struct early_suspend dyn_fsync_can_sync_handler = {
	.suspend = dyn_fsync_suspend,
	.resume = dyn_fsync_resume,
};

static int dyn_fsync_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
	if (dyn_fsync_active)
		dyn_fsync_flush(true);

	return NOTIFY_DONE;
}

static struct notifier_block dyn_fsync_reboot_handler = {
        .notifier_call = dyn_fsync_notify,
};

static int dyn_fsync_init(void)
{
	int sysfs_result;

	register_early_suspend(&dyn_fsync_can_sync_handler);
	register_reboot_notifier(&dyn_fsync_reboot_handler);

	dyn_fsync_kobj = kobject_create_and_add("dyn_fsync", kernel_kobj);
	if (!dyn_fsync_kobj) {
		pr_err("%s dyn_fsync kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(dyn_fsync_kobj, &dyn_fsync_active_attr_group);

	if (sysfs_result) {
		pr_info("%s dyn_fsync sysfs create failed!\n", __FUNCTION__);
		kobject_put(dyn_fsync_kobj);
	}

	INIT_DELAYED_WORK_DEFERRABLE(&fsync_suspend_work, fsync_suspend_work_fn);

	return sysfs_result;
}

static void dyn_fsync_exit(void)
{
	unregister_early_suspend(&dyn_fsync_can_sync_handler);
	unregister_reboot_notifier(&dyn_fsync_reboot_handler);

	if (dyn_fsync_kobj != NULL)
		kobject_put(dyn_fsync_kobj);
}

module_init(dyn_fsync_init);
module_exit(dyn_fsync_exit);

