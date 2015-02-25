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
#include "msm_sensor.h"

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define CAM_DRV_I2C_REG_SMIA          (0x0003)
#define CAM_DRV_I2C_REG_SMIA_MODEL    (0x0000)
#define CAM_DRV_I2C_REG_SMIA_REVISION (0x0002)
#define CAM_DRV_I2C_REG_SMIA_VERSION  (0x0004)
#define CAM_DRV_I2C_REG_ET8EX           (0x04) /* A register in ET8EK8-AS */
#define CAM_DRV_I2C_REG_SMIApp_VERSION  (0x0011)    /*SMIA++ version information register*/
#define CAM_DRV_I2C_REG_SMIApp_MODULE_DATE_PHASE (0x0015)
#define CAMERA_SMIA_SENSOR_REVISION_NUMBER      (0x0018)

/* SMIApp sensor manufacturer ID's */
#define CAM_SMIApp_MANUFACTURER_RESERVED    0x00 /*  0 */
#define CAM_SMIApp_MANUFACTURER_ST          0x01 /*  1 */
#define CAM_SMIApp_MANUFACTURER_AGILENT     0x02 /*  2 */
#define CAM_SMIApp_MANUFACTURER_FOVEON      0x03 /*  3 */
#define CAM_SMIApp_MANUFACTURER_HYNIX       0x04 /*  4 */
#define CAM_SMIApp_MANUFACTURER_IC_MEDIA    0x05 /*  5 */
#define CAM_SMIApp_MANUFACTURER_MICRON      0x06 /*  6 */
#define CAM_SMIApp_MANUFACTURER_OMNIVISION  0x07 /*  7 */
#define CAM_SMIApp_MANUFACTURER_PANASONIC   0x08 /*  8 */
#define CAM_SMIApp_MANUFACTURER_SAMSUNG     0x09 /*  9 */
#define CAM_SMIApp_MANUFACTURER_SHARP       0x0A /* 10 */
#define CAM_SMIApp_MANUFACTURER_SONY        0x0B /* 11 */
#define CAM_SMIApp_MANUFACTURER_TOSHIBA     0x0C /* 12 */
#define CAM_SMIApp_MANUFACTURER_CYPRESS     0x0D /* 13 */
#define CAM_SMIApp_MANUFACTURER_BYD         0x0E /* 14 */
#define CAM_SMIApp_MANUFACTURER_FOXCONN     0x0F /* 15 */
#define CAM_SMIApp_MANUFACTURER_LITEON      0x11 /* 17 */

/* SMIApp sensor model ID's */

/* Saruman modules */
#define CAM_SMIApp_MODEL_ST_VW6854                 0x0356 /*   854 - Used in Saruman EDoF 3.2 MP camera module with ST silicon */
                                                          /*   854 Also used in MaloneHP ST EDoF 3.2MP SMIA65 camera */
#define CAM_SMIApp_MODEL_ST_VX6953                 0x03B9 /*   953 - Used in Capone EDoF 5MP camera module with ST silicon */
#define CAM_SMIApp_MODEL_FOXCONN_CMNK5EN0XX        0x03F8 /*   1016 - Used in Capone EDoF 5MP camera module with Foxonn silicon */
#define CAM_SMIApp_MODEL_SAMSUNG_S5K4E2GX          0x4E20 /*   Used in Capone EDoF 5MP camera module with Samsung silicon */     
#define CAM_SMIApp_MODEL_SHARP_CAPONE              0x4E20 /*   Used in Capone EDoF 5MP camera module with Sharp silicon */     
       
/* Gambino modules. */
#define CAM_SMIApp_MODEL_TOSHIBA_GAMBINO_8MP_SI_TBD 0x218E /* Toshiba Gambino EDoF 8MP camera module with TBD silicon. */
#define CAM_SMIApp_MODEL_FOXCONN_GAMBINO_8MP_SI_TBD 0x0088 /* Foxconn Gambino EDoF 8MP camera module with TBD silicon. */

/* Kestrel modules */
#define CAM_SMIApp_MODEL_LITEON_KESTREL            0x5B60 /*   Used in Kestrel 2MP camera module with Liteon silicon */     

#define CAM_SMIApp_MODEL_TOSHIBA_RAISU            0x563A /*   Used in RAISU 5MP camera module with TOSHIBA silicon */  

#define CAM_SMIApp_MODEL_ST_RAISU            0x07A3 /*   Used in RAISU 5MP camera module with ST silicon */  


struct smiapp_sensor_module_info {
    uint8_t manufacturer_id;
    uint16_t module_id;
    uint8_t revision;
    const char      *name;
};
const struct smiapp_sensor_module_info smiapp_sensor_module_table[] = {
    {CAM_SMIApp_MANUFACTURER_TOSHIBA,CAM_SMIApp_MODEL_TOSHIBA_RAISU, 3,"smiapp0c563a03"},
    {CAM_SMIApp_MANUFACTURER_ST,CAM_SMIApp_MODEL_ST_RAISU,2,"smiapp0107a302"},
};



#define SMIAPP_SENSOR_NAME "smiapp"
DEFINE_MSM_MUTEX(smiapp_mut);

static struct msm_sensor_ctrl_t smiapp_s_ctrl;

static struct msm_sensor_power_setting smiapp_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VDIG,
        .config_val = 0,
        .delay = 1,
    },
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 9600000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
};

static struct v4l2_subdev_info smiapp_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id smiapp_i2c_id[] = {
	{SMIAPP_SENSOR_NAME, (kernel_ulong_t)&smiapp_s_ctrl},
	{ }
};

static int32_t msm_smiapp_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &smiapp_s_ctrl);
}

static struct i2c_driver smiapp_i2c_driver = {
	.id_table = smiapp_i2c_id,
	.probe  = msm_smiapp_i2c_probe,
	.driver = {
		.name = SMIAPP_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client smiapp_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id smiapp_dt_match[] = {
	{.compatible = "qcom,smiapp", .data = &smiapp_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, smiapp_dt_match);

static struct platform_driver smiapp_platform_driver = {
	.driver = {
		.name = "qcom,smiapp",
		.owner = THIS_MODULE,
		.of_match_table = smiapp_dt_match,
	},
};

static int32_t smiapp_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(smiapp_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init smiapp_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&smiapp_platform_driver,
		smiapp_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&smiapp_i2c_driver);
}

static void __exit smiapp_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (smiapp_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&smiapp_s_ctrl);
		platform_driver_unregister(&smiapp_platform_driver);
	} else
		i2c_del_driver(&smiapp_i2c_driver);
	return;
}

int32_t smiapp_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    uint16_t chipid = 0;
    uint16_t manufacturer_id = 0;
    uint16_t revision = 0;
    uint16_t module_count = sizeof(smiapp_sensor_module_table)/sizeof(smiapp_sensor_module_table[0]);
    uint16_t i= 0;
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            CAM_DRV_I2C_REG_SMIA_MODEL,
            &chipid, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("%s: %s: read chipid failed\n", __func__,
            s_ctrl->sensordata->sensor_name);
        return rc;
    }
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
        s_ctrl->sensor_i2c_client,
        CAM_DRV_I2C_REG_SMIA,
        &manufacturer_id, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: %s: read manufacturer_id failed\n", __func__,
            s_ctrl->sensordata->sensor_name);
        return rc;
    }
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
    s_ctrl->sensor_i2c_client,
    CAM_DRV_I2C_REG_SMIA_REVISION,
    &revision, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: %s: read revision failed\n", __func__,
            s_ctrl->sensordata->sensor_name);
        return rc;
    }
    for(i= 0; i<module_count; i++)
    {
        
        CDBG("%s: read id:%02x,%04x,%02x, expected id %02x,%04x,%02x,\n", __func__,manufacturer_id, chipid,revision,
        smiapp_sensor_module_table[i].manufacturer_id,smiapp_sensor_module_table[i].module_id,smiapp_sensor_module_table[i].revision);
        
        if (chipid == smiapp_sensor_module_table[i].module_id
            && manufacturer_id == smiapp_sensor_module_table[i].manufacturer_id
            && revision == smiapp_sensor_module_table[i].revision
            ) {
            s_ctrl->sensordata->sensor_name = smiapp_sensor_module_table[i].name;
            CDBG("smiapp_match_id: %s,id match successed!\n",s_ctrl->sensordata->sensor_name);
            break;
        }
    }
    if (i == module_count) {
        pr_err("msm_sensor_match_id chip id doesnot match\n");
        return -ENODEV;
    }    
    return rc;
}



static struct msm_sensor_fn_t smiapp_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = smiapp_match_id,
};



static struct msm_sensor_ctrl_t smiapp_s_ctrl = {
	.sensor_i2c_client = &smiapp_sensor_i2c_client,
	.power_setting_array.power_setting = smiapp_power_setting,
	.power_setting_array.size = ARRAY_SIZE(smiapp_power_setting),
	.msm_sensor_mutex = &smiapp_mut,
	.sensor_v4l2_subdev_info = smiapp_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(smiapp_subdev_info),
	.func_tbl = &smiapp_sensor_func_tbl,
};

module_init(smiapp_init_module);
module_exit(smiapp_exit_module);
MODULE_DESCRIPTION("smiapp");
MODULE_LICENSE("GPL v2");
