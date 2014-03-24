/*
 * Author: Manuel Manz alias dtrail <mnl.manz@gmail.com>
 *
 * Copyright 2013 Manuel Manz
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
#include <linux/mutex.h>
#include <linux/dpll.h>

#define DPLL_VERSION_MAJOR 1
#define DPLL_VERSION_MINOR 1

static DEFINE_MUTEX(dpll_mutex);
//#if defined(CONFIG_MAPPHONE_EDISON) || defined(CONFIG_MAPPHONE_TARGA)
bool dpll_active __read_mostly = true;
//#else
//bool dpll_active __read_mostly = true;
//#endif

static ssize_t dpll_active_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", (dpll_active ? 1 : 0));
}

static ssize_t dpll_active_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data == 1) {
			pr_info("%s: DPLL_CASCADING enabled\n", __FUNCTION__);
			dpll_active = true;
		}
		else if (data == 0) {
			pr_info("%s: DPLL_CASCADING disabled\n", __FUNCTION__);
			dpll_active = false;
		}
		else
			pr_info("%s: bad value: %u\n", __FUNCTION__, data);
	} else
		pr_info("%s: unknown input!\n", __FUNCTION__);

	return count;
}


static ssize_t dpll_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u by dtrail\n",
		DPLL_VERSION_MAJOR,
		DPLL_VERSION_MINOR);
}

static struct kobj_attribute dpll_active_attribute = 
	__ATTR(dpll_active, 0755,
		dpll_active_show,
		dpll_active_store);

static struct kobj_attribute dpll_version_attribute = 
	__ATTR(dpll_version, 0444, dpll_version_show, NULL);

static struct attribute *dpll_active_attrs[] =
	{
		&dpll_active_attribute.attr,
		&dpll_version_attribute.attr,
		NULL,
	};

static struct attribute_group dpll_active_attr_group =
	{
		.attrs = dpll_active_attrs,
	};

static struct kobject *dpll_kobj;

static int dpll_init(void)
{
	int sysfs_result;


	dpll_kobj = kobject_create_and_add("dpll", kernel_kobj);
	if (!dpll_kobj) {
		pr_err("%s DPLL_CASCADING kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(dpll_kobj,
			&dpll_active_attr_group);

        if (sysfs_result) {
		pr_info("%s DPLL_CASCADING sysfs create failed!\n", __FUNCTION__);
		kobject_put(dpll_kobj);
	}
	return sysfs_result;
}

static void dpll_exit(void)
{
	if (dpll_kobj != NULL)
		kobject_put(dpll_kobj);
}

module_init(dpll_init);
module_exit(dpll_exit);
