/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#if defined(CONFIG_ARA_S01)
#define FLASH_NAME "qcom,as3644S01"
#else
#define FLASH_NAME "qcom,as3644"
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver as3644_i2c_driver;

static struct msm_camera_i2c_reg_array as3644_init_array[] = {
	{0x03, 0x00},
	{0x02, 0xc4}
};

static struct msm_camera_i2c_reg_array as3644_off_array[] = {
	{0x03, 0x00},
};

static struct msm_camera_i2c_reg_array as3644_release_array[] = {
	{0x03, 0x00},
};

static struct msm_camera_i2c_reg_array as3644_low_array[] = {
	{0x02, 0xc4},
	{0x03, 0x24}
};

static struct msm_camera_i2c_reg_array as3644_high_array[] = {
	{0x02, 0xc4},
	{0x03, 0x3F}
};

static void __exit msm_flash_as3644_i2c_remove(void)
{
	i2c_del_driver(&as3644_i2c_driver);
	return;
}

static const struct of_device_id as3644_i2c_trigger_dt_match[] = {
	{.compatible = FLASH_NAME},
	{}
};

MODULE_DEVICE_TABLE(of, as3644_i2c_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id as3644_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_as3644_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("msm_flash_as3644_i2c_probe: id is NULL");
		id = as3644_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver as3644_i2c_driver = {
	.id_table = as3644_i2c_id,
	.probe  = msm_flash_as3644_i2c_probe,
	.remove = __exit_p(msm_flash_as3644_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = as3644_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_as3644_i2c_add_driver(void)
{
	CDBG("%s called\n", __func__);
	return i2c_add_driver(&as3644_i2c_driver);
}

static struct msm_camera_i2c_client as3644_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting as3644_init_setting = {
	.reg_setting = as3644_init_array,
	.size = ARRAY_SIZE(as3644_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting as3644_off_setting = {
	.reg_setting = as3644_off_array,
	.size = ARRAY_SIZE(as3644_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting as3644_release_setting = {
	.reg_setting = as3644_release_array,
	.size = ARRAY_SIZE(as3644_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting as3644_low_setting = {
	.reg_setting = as3644_low_array,
	.size = ARRAY_SIZE(as3644_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting as3644_high_setting = {
	.reg_setting = as3644_high_array,
	.size = ARRAY_SIZE(as3644_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t as3644_regs = {
	.init_setting = &as3644_init_setting,
	.off_setting = &as3644_off_setting,
	.low_setting = &as3644_low_setting,
	.high_setting = &as3644_high_setting,
	.release_setting = &as3644_release_setting,
};

static struct msm_flash_fn_t as3644_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &as3644_i2c_client,
	.reg_setting = &as3644_regs,
	.func_tbl = &as3644_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_as3644_i2c_add_driver);
module_exit(msm_flash_as3644_i2c_remove);
MODULE_DESCRIPTION("as3644");
MODULE_LICENSE("GPL v2");
