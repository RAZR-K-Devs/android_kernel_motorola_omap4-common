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

#define HDMI_VERSION_MAJOR 1
#define HDMI_VERSION_MINOR 1

static DEFINE_MUTEX(hdmi_mutex);

bool hdmi_active = true;

static ssize_t hdmi_active_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", (hdmi_active ? 1 : 0));
}

static ssize_t hdmi_active_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data == 1) {
			pr_info("%s: HDMI enabled\n", __FUNCTION__);
			hdmi_active = true;
		}
		else if (data == 0) {
			pr_info("%s: HDMI disabled\n", __FUNCTION__);
			hdmi_active = false;
		}
		else
			pr_info("%s: bad value: %u\n", __FUNCTION__, data);
	} else
		pr_info("%s: unknown input!\n", __FUNCTION__);

	return count;
}


static ssize_t hdmi_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u by dtrail\n",
		HDMI_VERSION_MAJOR,
		HDMI_VERSION_MINOR);
}

static struct kobj_attribute hdmi_active_attribute = 
	__ATTR(hdmi_active, 0755,
		hdmi_active_show,
		hdmi_active_store);

static struct kobj_attribute hdmi_version_attribute = 
	__ATTR(hdmi_version, 0444, hdmi_version_show, NULL);

static struct attribute *hdmi_active_attrs[] =
	{
		&hdmi_active_attribute.attr,
		&hdmi_version_attribute.attr,
		NULL,
	};

static struct attribute_group hdmi_active_attr_group =
	{
		.attrs = hdmi_active_attrs,
	};

static struct kobject *hdmi_kobj;

static int hdmi_init(void)
{
	int sysfs_result;


	hdmi_kobj = kobject_create_and_add("hdmi", kernel_kobj);
	if (!hdmi_kobj) {
		pr_err("%s HDMI_TOGGLE kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(hdmi_kobj,
			&hdmi_active_attr_group);

        if (sysfs_result) {
		pr_info("%s HDMI_TOGGLE sysfs create failed!\n", __FUNCTION__);
		kobject_put(hdmi_kobj);
	}
	return sysfs_result;
}

static void hdmi_exit(void)
{
	if (hdmi_kobj != NULL)
		kobject_put(hdmi_kobj);
}

module_init(hdmi_init);
module_exit(hdmi_exit);
