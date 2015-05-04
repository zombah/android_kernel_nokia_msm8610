/*  *  linux/drivers/sbl/sbllog.c *  *
  *  Copyright (C) 2013 NOKIA Inc,
  *
  *
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  *
  *
  * Author: Yulin Ren (yulin.ren@nokia.com)
 */



#include <linux/kernel.h>

#include <linux/module.h>

#include <linux/init.h>

#include <linux/mm.h>

#include <linux/fs.h>

#include <linux/types.h>

#include <linux/delay.h>

#include <linux/moduleparam.h>

#include <linux/slab.h>

#include <linux/errno.h>

#include <linux/ioctl.h>

#include <linux/cdev.h>

#include <linux/device.h>
#include <linux/string.h>

#include <linux/list.h>

#include <linux/pci.h>

#include <asm/uaccess.h>

#include <asm/atomic.h>

#include <asm/unistd.h>


#include "sbllog.h"
static int sbl_major = SBL_LOG_MAJOR;

module_param(sbl_major, int, S_IRUGO);

static struct sbl_dev *sbl_devp;
static struct cdev cdev;

static struct class *sbldev_class;

int sbl_open(struct inode *inode, struct file *filp)
{
	struct sbl_dev *dev;

	dev = sbl_devp;

	filp->private_data = dev;
	return 0;
}

int sbl_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int sbldev_mmap(struct file*filp, struct vm_area_struct *vma)
{
	struct sbl_dev *dev = filp->private_data;

	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_RESERVED;


	if (remap_pfn_range(vma,vma->vm_start,((unsigned int)(dev->data))>>PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		pr_debug("sbldev_map error exit\n");
		return  -EAGAIN;
	}
	return 0;
}


static const struct file_operations sbl_fops =
{
	.owner = THIS_MODULE,
	.open = sbl_open,
	.release = sbl_release,
	.mmap = sbldev_mmap,
};




static __init int sbldev_init(void)
{
	int result= -1;
	int i;
	dev_t devno = MKDEV(sbl_major, 0);


	if (sbl_major)
		result = register_chrdev_region(devno, 1, "sbldev");
	else  {
		result = alloc_chrdev_region(&devno, 0, 1, "sbldev");
		sbl_major = MAJOR(devno);
	}


	if (result < 0)
		return result;

	/*cdev init*/
	cdev_init(&cdev, &sbl_fops);
	cdev.owner = THIS_MODULE;
	cdev.ops = &sbl_fops;

    result = -1;

	i = cdev_add(&cdev, MKDEV(sbl_major, 0), 1);
	if( i < 0) {
		pr_debug("sbldev cdev_add fail: %d\n", i);
		goto fail_malloc;
	}

	sbldev_class = class_create(THIS_MODULE, "sbldev_class");

	if(IS_ERR(sbldev_class)) {

		pr_debug("Err: failed in creating class.\n");
		goto fail_malloc;
	}

	/* register device in sysfs, and this will cause udevd to create corresponding device node */

	device_create(sbldev_class, NULL, MKDEV(sbl_major, 0), NULL, "sbllog%d", 0 );

	sbl_devp = (struct sbl_dev *)kmalloc(sizeof(struct sbl_dev), GFP_KERNEL);
	if (!sbl_devp) {
		pr_debug("sbldev kmalloc fail\n");
		goto fail_malloc;
	}
	memset(sbl_devp, 0, sizeof(struct sbl_dev));

	sbl_devp->size = 1024*1024;
	sbl_devp->data = (char *)0xde00000;

	return 0;

fail_malloc:
	unregister_chrdev(sbl_major, "sbldrv");
	return result;
}


static void sbldev_exit(void)
{
	cdev_del(&cdev);
	device_destroy(sbldev_class, MKDEV(sbl_major, 0));

	class_destroy(sbldev_class);

	unregister_chrdev_region(MKDEV(sbl_major, 0), 1);
	kfree(sbl_devp);
}

MODULE_AUTHOR("nokia");
MODULE_LICENSE("GPL");

module_init(sbldev_init);
module_exit(sbldev_exit);

