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
#include "msm_cci.h"
#include "msm_camera_io_util.h"

#include <linux/delay.h>

//static const char * ov7695_orientation = NULL;

#define OV7695_OUTPUT_YUV 0
#define OV7695_OUTPUT_RAW 1

#define OV7695_OUTPUT_FORMAT OV7695_OUTPUT_YUV

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
#define OV7695_SENSOR_NAME "ov7695"
DEFINE_MSM_MUTEX(ov7695_mut);

static struct msm_sensor_ctrl_t ov7695_s_ctrl;

static struct msm_sensor_power_setting ov7695_power_setting[] = {
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
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},
};

static struct v4l2_subdev_info ov7695_subdev_info[] = {
	{
#if OV7695_OUTPUT_FORMAT == OV7695_OUTPUT_YUV
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
#endif
#if OV7695_OUTPUT_FORMAT == OV7695_OUTPUT_RAW
		.code   = V4L2_MBUS_FMT_SBGGR8_1X8,
#endif
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static int32_t msm_ov7695_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	CDBG("%s, E.", __func__);

    //Normal
	return msm_sensor_i2c_probe(client, id, &ov7695_s_ctrl);
}

static const struct i2c_device_id ov7695_i2c_id[] = {
	{OV7695_SENSOR_NAME, (kernel_ulong_t)&ov7695_s_ctrl},
	{ }
};

static struct i2c_driver ov7695_i2c_driver = {
	.id_table = ov7695_i2c_id,
	.probe  = msm_ov7695_i2c_probe,
	.driver = {
		.name = OV7695_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov7695_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};


static const struct of_device_id ov7695_dt_match[] = {
	{.compatible = "qcom,ov7695", .data = &ov7695_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov7695_dt_match);

static struct platform_driver ov7695_platform_driver = {
	.driver = {
		.name = "qcom,ov7695",
		.owner = THIS_MODULE,
		.of_match_table = ov7695_dt_match,
	},
};

static int32_t ov7695_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	CDBG("%s, E.", __func__);
	match = of_match_device(ov7695_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov7695_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov7695_platform_driver,
		ov7695_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov7695_i2c_driver);
}

static void __exit ov7695_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov7695_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov7695_s_ctrl);
		platform_driver_unregister(&ov7695_platform_driver);
	} else
		i2c_del_driver(&ov7695_i2c_driver);
	return;
}

#if OV7695_OUTPUT_FORMAT == OV7695_OUTPUT_YUV
static struct msm_camera_i2c_reg_conf ov7695_start_settings[] = {
	{0x0100, 0x01},
	//{0x3014, 0x00},
    /*
      ov said it work on their EVK board,
      but camera can't work on QC'platform,
      maybe because of LP state of MIPI is 11
      while ov is 00,turn on/off MIPI phy should
      enable stream on,means 0x0100 should be 0x1
    */
};

static struct msm_camera_i2c_reg_conf ov7695_stop_settings[] = {
	//{0x0100, 0x00},
	//{0x3014, 0x20},
};

static struct msm_camera_i2c_reg_conf ov7695_recommend_settings[] = {
    //30fps mipi 192mbps
	//{0x0103, 0x01},  //     ; software reset
	{0x3620, 0x2f},  //
	{0x3621, 0x47},  //
	{0x3623, 0x12},  //
	{0x3718, 0x88},  //
	{0x3703, 0x80},  //
	{0x3712, 0x40},  //
	{0x3706, 0x40},  //
	{0x3631, 0x44},  //
	{0x3632, 0x05},  //
	{0x3013, 0xd0},  //
	{0x3705, 0x1d},  //
	{0x3713, 0x0e},  //
	{0x3012, 0x0a},  //
	{0x3717, 0x18},  //
	{0x0309, 0x24},  // ; DACC clk div by 4
	{0x3820, 0x90},  //
	{0x4803, 0x08},  // ; HS prepare in UI
	//{0x0101, 0x02},  // ; mirror on, flip off
	{0x5100, 0x01},  // ; lenc
	{0x4500, 0x24},  //
	{0x520a, 0xf4},  // ; red gain from 0x400 to 0xfff
	{0x520b, 0xf4},  // ; green gain from 0x400 to 0xfff
	{0x520c, 0xf4},  // ; blue gain from 0x400 to 0xfff
	{0x3a18, 0x01},  // ; gain ceiling 0x100
	{0x3a19, 0x00},  // ; gain ceiling
	{0x3503, 0x03},  // ; AGC manual on, AEC manual on
	{0x3500, 0x00},  // ; exposure
	{0x3501, 0x21},  // ; exposure
	{0x3502, 0x00},  // ; exposure
	{0x350a, 0x00},  // ; gain
	{0x350b, 0x00},  // ; gain
	{0x4008, 0x02},  // ; bl start
	{0x4009, 0x09},  // ; bl end
	{0x3002, 0x09},  // ; FSIN output
	{0x3024, 0x00},  //
	{0x3503, 0x00},  // ; AGC auto on, AEC auto on
	//{0x0101, 0x01},  // ; mirror_on
	{0x5002, 0x4a},  // ; [7:6] Y source select, manual 50Hz
	{0x5910, 0x00},  // ; Y formula
	{0x3a0f, 0x58},  // ; AEC in H
	{0x3a10, 0x50},  // ; 38 ;AEC in L
	{0x3a1b, 0x5a},  // ; 40 ;AEC out H
	{0x3a1e, 0x4e},  // ; 36 ;AEC out L
	{0x3a11, 0xa0},  // ; 80 ;control zone H
	{0x3a1f, 0x28},  // ; 18 ; control zone L
	{0x3a18, 0x00},  // ; gain ceiling
	{0x3a19, 0xf8},  // ; gain ceiling, max gain 15.5x
	{0x3503, 0x00},  // ; aec/agc auto on
	{0x3a0d, 0x04},  // ; 60Hz max band step
	{0x5000, 0xff},  // ; lcd, gma, awb, awbg, bc, wc, lenc, isp
	{0x5001, 0x3f},  // ; avg, blc, sde, uv_avg, cmx, cip
	{0x5003, 0x80},  // ;
	// ;lenc         //
	{0x5100, 0x1 },  //;r_x
	{0x5101, 0x40},  //
	{0x5102, 0x1 },  //;r_y
	{0x5103, 0x10},  //
	{0x5104, 0x3f},  // ;r_a1
	{0x5105, 0x5 },  //
	{0x5106, 0xff},  //
	{0x5107, 0xf },  //
	{0x5108, 0x1 },  //;g_x
	{0x5109, 0x40},  //
	{0x510a, 0x0 },  //;g_y
	{0x510b, 0xf0},  //
	{0x510c, 0x45},  // ;g_a1
	{0x510d, 0x6 },  //
	{0x510e, 0xff},  //
	{0x510f, 0xf },  //
	{0x5110, 0x1 },  //;b_x
	{0x5111, 0x40},  //
	{0x5112, 0x0 },  //;b_y
	{0x5113, 0xe0},  //
	{0x5114, 0x18},  // ;b_a1
	{0x5115, 0x5 },  //
	{0x5116, 0xff},  //
	{0x5117, 0xf },  //
	// ;AWB          //
	{0x520a, 0x74},  // ; red gain from 0x400 to 0x7ff
	{0x520b, 0x64},  // ; green gain from 0x400 to 0x7ff
	{0x520c, 0xd4},  // ; blue gain from 0x400 to 0xdff
	// ; Gamma       //
	{0x5301, 0x05},  //
	{0x5302, 0x0c},  //
	{0x5303, 0x1c},  //
	{0x5304, 0x2a},  //
	{0x5305, 0x39},  //
	{0x5306, 0x45},  //
	{0x5307, 0x53},  //
	{0x5308, 0x5d},  //
	{0x5309, 0x68},  //
	{0x530a, 0x7f},  //
	{0x530b, 0x91},  //
	{0x530c, 0xa5},  //
	{0x530d, 0xc6},  //
	{0x530e, 0xde},  //
	{0x530f, 0xef},  //
	{0x5310, 0x16},  //
	// ;sharpen/den  //oise
	{0x5500, 0x08},  // ; sharp th1 8x
	{0x5501, 0x48},  // ; sharp th2 8x
	{0x5502, 0x18},  // ; sharp mt offset1
	{0x5503, 0x04},  // ; sharp mt offset2
	{0x5504, 0x08},  // ; dns th1 8x
	{0x5505, 0x48},  // ; dns th2 8x
	{0x5506, 0x02},  // ; dns offset1
	{0x5507, 0x16},  // ; dns offset2
	{0x5508, 0x2d},  // ; [6]:sharp_man [4]:dns_man
	{0x5509, 0x08},  // ; sharpth th1 8x
	{0x550a, 0x48},  // ; sharpth th2 8x
	{0x550b, 0x06},  // ; sharpth offset1
	{0x550c, 0x04},  // ; sharpth offset2
	{0x550d, 0x01},  // ; recursive_en
	// ; SDE, for s  //aturation 120% under D65
	{0x5800, 0x06},  // ; saturation on, contrast on
	{0x5803, 0x2e},  // ; 40 ; sat th2
	{0x5804, 0x20},  // ; 34 ; sat th1
	{0x580b, 0x02},  // ; Y offset man on
	// ; CMX QE      //
	{0x5600, 0x00},  // ; mtx 1.7, UV CbCr disable
	{0x5601, 0x2c},  // ; CMX1
	{0x5602, 0x5a},  // ; CMX2
	{0x5603, 0x06},  // ; CMX3
	{0x5604, 0x1c},  // ; CMX4
	{0x5605, 0x65},  // ; CMX5
	{0x5606, 0x81},  // ; CMX6
	{0x5607, 0x9f},  // ; CMX7
	{0x5608, 0x8a},  // ; CMX8
	{0x5609, 0x15},  // ; CMX9
	{0x560a, 0x01},  // ; Sign
	{0x560b, 0x9c},  // ; Sign
	{0x3811, 0x07},  // ; Tradeoff position to make YUV/RAW x VGA/QVGA x Mirror/Flip all work
	{0x3813, 0x06},  //
	{0x3630, 0x79},  // ; ADC7 when R3021=0xb1, ADC6(0x69) if R3021 = others
	//{0x0100 , 0x00},  //,; streaming
//IQ setting tony 2013_12_17
	{0x4002,0x00},  //ss
    {0x4003,0x10},  //
    {0x3503,0x00},  //ss
    {0x3a0f,0x58},  //
    {0x3a10,0x50},  //
    {0x3a1b,0x58},  //
    {0x3a1e,0x50},  //
    {0x3a11,0xa0},  //
    {0x3a1f,0x28},  //
    {0x5100,0x1 },  //ss
    {0x5101,0x30},  //
    {0x5102,0x0 },  //
    {0x5103,0xf0},  //
    {0x5104,0x5f},  //
    {0x5105,0x5 },  //
    {0x5106,0xff},  //
    {0x5107,0xf },  //
    {0x5108,0x1 },  //
    {0x5109,0x40},  //
    {0x510a,0x0 },  //
    {0x510b,0xe8},  //
    {0x510c,0x45},  //
    {0x510d,0x6 },  //
    {0x510e,0xff},  //
    {0x510f,0xf },  //
    {0x5110,0x1 },  //
    {0x5111,0x40},  //
    {0x5112,0x0 },  //
    {0x5113,0xe0},  //
    {0x5114,0x18},  //
    {0x5115,0x5 },  //
    {0x5116,0xff},  //
    {0x5117,0xf },  //
    {0x520a,0x74},  //
    {0x520b,0x64},  //
    {0x520c,0xd4},  //
    {0x5004,0x41},  //
    {0x5006,0x41},  //
    {0x5301,0x05},  //ss
    {0x5302,0x0c},  //
    {0x5303,0x1c},  //
    {0x5304,0x2a},  //
    {0x5305,0x39},  //
    {0x5306,0x45},  //
    {0x5307,0x53},  //
    {0x5308,0x5d},  //
    {0x5309,0x68},  //
    {0x530a,0x7f},  //
    {0x530b,0x91},  //
    {0x530c,0xA5},  //
    {0x530d,0xC6},  //
    {0x530e,0xDe},  //
    {0x530f,0xEf},  //
    {0x5310,0x16},  //
    {0x5601,0x2c},  //ss
    {0x5602,0x5a},  //
    {0x5603,0x06},  //
    {0x5604,0x1c},  //
    {0x5605,0x65},  //
    {0x5606,0x81},  //
    {0x5607,0x9f},  //
    {0x5608,0x8a},  //
    {0x5609,0x15},  //
    {0x560a,0x01},  //
    {0x560b,0x9c},  //
    {0x5508,0x2d},  //ss
    {0x5504,0x08},  //
    {0x5505,0x48},  //
    {0x5506,0x02},  //
    {0x5507,0x14},  //
    {0x5508,0x2d},  //ss
    {0x5500,0x08},  //
    {0x5501,0x48},  //
    {0x5502,0x1f},  //
    {0x5503,0x08},  //
    {0x5509,0x08},  //
    {0x550a,0x48},  //
    {0x550b,0x06},  //
    {0x550c,0x04},  //
    {0x5800,0x02},  //ss
    {0x580b,0x00},  //
    {0x5803,0x2e},  //
    {0x5804,0x20},  //
    {0x5809,0x08},  //
    {0x580a,0x80},  //
    {0x5002,0x4a},  //ss
    {0x3a05,0x30},  //
    {0x3a18,0x00},  //
    {0x3a19,0xe0},  //
    {0x0100 , 0x00},  // streaming
};

static struct msm_camera_i2c_reg_conf ov7695_VGA_settings[] = {
	{0x0340, 0x02},  //frame_length_lines
	{0x0341, 0x18},  //
	{0x0342, 0x02},  //line_length_pclk
	{0x0343, 0xEA},  //
	{0x034c, 0x02},  //
	{0x034d, 0x80},  //
	{0x034e, 0x01},  //
	{0x034f, 0xE0},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_saturation[7][6] = {
	{
		//Saturation - 3
		{0x5604, 0x15}, //
		{0x5605, 0x4b}, //
		{0x5606, 0x60}, //
		{0x5607, 0x76}, //
		{0x5608, 0x67}, //
		{0x5609, 0x0f}, //
    },
	{
		//Saturation - 2

		{0x5604, 0x17}, //
		{0x5605, 0x53}, //
		{0x5606, 0x6a}, //
		{0x5607, 0x83}, //
		{0x5608, 0x72}, //
		{0x5609, 0x11}, //
	},
	{
		//saturation -1
		{0x5604, 0x19}, //
		{0x5605, 0x5b}, //
		{0x5606, 0x74}, //
		{0x5607, 0x90}, //
		{0x5608, 0x7d}, //
		{0x5609, 0x13}, //
	},
	{
		// Saturation 0

		{0x5604, 0x1c}, //
		{0x5605, 0x65}, //
		{0x5606, 0x81}, //
		{0x5607, 0x9f}, //
		{0x5608, 0x8a}, //
		{0x5609, 0x15}, //
	},
	{
		//Saturation + 1

		{0x5604, 0x1e}, //
		{0x5605, 0x6f}, //
		{0x5606, 0x8d}, //
		{0x5607, 0xae}, //
		{0x5608, 0x97}, //
		{0x5609, 0x17}, //
	},
	{
		//aturation + 2
		{0x5604, 0x21}, //
		{0x5605, 0x7a}, //
		{0x5606, 0x9b}, //
		{0x5607, 0xbf}, //
		{0x5608, 0xa6}, //
		{0x5609, 0x19}, //
	},
	{
		//Saturation + 3
		//
		{0x5604, 0x25}, //
		{0x5605, 0x86}, //
		{0x5606, 0xab}, //
		{0x5607, 0xd2}, //
		{0x5608, 0xb7}, //
		{0x5609, 0x1b}, //
	},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_contrast[7][2] = {
	{
		//Contrast -3

		{0x5805, 0x14}, //
		{0x5806, 0x14}, //

	},
	{
		//Contrast -2

		{0x5805, 0x18}, //
		{0x5806, 0x18}, //
	},
	{
		//Contrast -1

		{0x5805, 0x1c}, //
		{0x5806, 0x1c}, //
	},
	{
		//  Contrast 0

		{0x5805, 0x00}, //
		{0x5806, 0x20}, //
	},
	{
		//Contrast +1

		{0x5805, 0x10}, //
		{0x5806, 0x24}, //
	},
	{
		//Contrast +2

		{0x5805, 0x18}, //
		{0x5806, 0x28}, //
	},
	{
		// Contrast +3

		{0x5805, 0x1c}, //
		{0x5806, 0x2c}, //
	},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_sharpness[7][2] = {
	{
		{0x5807, 0x20}, //
		{0x5808, 0x08}, //
	}, /* SHARPNESS LEVEL 0*/
	{
		{0x5807, 0x10}, //
		{0x5808, 0x08}, //
	}, /* SHARPNESS LEVEL 1*/
	{
		{0x5807, 0x00}, //
		{0x5808, 0x00}, //
	}, /* SHARPNESS LEVEL 2*/
	{
		{0x5807, 0x10}, //
		{0x5808, 0x00}, //
	}, /* SHARPNESS LEVEL 3*/
	{
		{0x5807, 0x20}, //
		{0x5808, 0x00}, //
	}, /* SHARPNESS LEVEL 4*/
	{
		{0x5807, 0x28}, //
		{0x5808, 0x00}, //
	}, /* SHARPNESS LEVEL 5*/
	{
		{0x5807, 0x30}, //
		{0x5808, 0x00}, //
	}, /* SHARPNESS LEVEL 6*/
};

static struct msm_camera_i2c_reg_conf ov7695_reg_iso[7][2] = {
	/* auto */
	{
		{0x3a18, 0x00},
		{0x3a19, 0xf8},

	},
	/* auto hjt */
	{
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
	},
	/* iso 100 */
	{
		{0x3a18, 0x00},
		{0x3a19, 0x3c},
	},
	/* iso 200 */
	{
		{0x3a18, 0x00},
		{0x3a19, 0x7c},
	},
	/* iso 400 */
	{
		{0x3a18, 0x00},
		{0x3a19, 0xc8},
	},
	/* iso 800 */
	{
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
	},
	/* iso 1600 */
	{
		{0x3a18, 0x01},
		{0x3a19, 0xf8},
	},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_exposure_compensation[5][6] = {
	/* -2 */
	{
		//EV -2

		{0x3a0f, 0x38}, //
		{0x3a10, 0x30}, //
		{0x3a11, 0x61}, //
		{0x3a1b, 0x38}, //
		{0x3a1e, 0x30}, //
		{0x3a1f, 0x10}, //
	},
	/* -1 */
	{
		//EV -1

		{0x3a0f, 0x48}, //
		{0x3a10, 0x40}, //
		{0x3a11, 0x80}, //
		{0x3a1b, 0x48}, //
		{0x3a1e, 0x40}, //
		{0x3a1f, 0x20}, //
	},
	/* 0 */
	{
		//EV 0

		{0x3a0f, 0x58}, //
		{0x3a10, 0x50}, //
		{0x3a11, 0x91}, //
		{0x3a1b, 0x58}, //
		{0x3a1e, 0x50}, //
		{0x3a1f, 0x20}, //
	},
	/* 1 */
	{
		//V +1

		{0x3a0f, 0x60}, //
		{0x3a10, 0x58}, //
		{0x3a11, 0xa0}, //
		{0x3a1b, 0x60}, //
		{0x3a1e, 0x58}, //
		{0x3a1f, 0x20}, //
	},
	/* 2 */
	{
		//V +2

		{0x3a0f, 0x70}, //
		{0x3a10, 0x60}, //
		{0x3a11, 0xa0}, //
		{0x3a1b, 0x70}, //
		{0x3a1e, 0x60}, //
		{0x3a1f, 0x20}, //
	},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_antibanding[][2] = {
	/* OFF */
	{

	},
	/* 50Hz */
	{

	},
	/* 60Hz */
	{

	},
	/* AUTO */
	{

	},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_effect_normal[] = {
	/* normal: */
	{0x5803, 0x2e}, // //saturation U
	{0x5804, 0x20}, // //saturation V

};

static struct msm_camera_i2c_reg_conf ov7695_reg_effect_black_white[] = {
	/* B&W: */
	{0x5803, 0x80}, // //saturation U
	{0x5804, 0x80}, // //saturation V
};

static struct msm_camera_i2c_reg_conf ov7695_reg_effect_negative[] = {
	/* Negative: */

};

static struct msm_camera_i2c_reg_conf ov7695_reg_effect_old_movie[] = {
	/* Sepia(antique): */
	{0x5803, 0x40}, // //saturation U
	{0x5804, 0xa0}, // //saturation V
};

static struct msm_camera_i2c_reg_conf ov7695_reg_effect_solarize[] = {

    //greenish
	{0x5803, 0x58}, // //saturation U
	{0x5804, 0x58}, // //saturation V
};

static struct msm_camera_i2c_reg_conf ov7695_reg_effect_redish[] = {

    //redish
	{0x5803, 0x80}, // //saturation U
	{0x5804, 0xc0}, // //saturation V
};

static struct msm_camera_i2c_reg_conf ov7695_reg_effect_blueish[] = {

    //blueish
	{0x5803, 0xa0}, // //saturation U
	{0x5804, 0x80}, // //saturation V
};
#if 0
static struct msm_camera_i2c_reg_conf ov7695_reg_scene_auto[] = {
	/* <SCENE_auto> */

};

static struct msm_camera_i2c_reg_conf ov7695_reg_scene_portrait[] = {
	/* <CAMTUNING_SCENE_PORTRAIT> */

};

static struct msm_camera_i2c_reg_conf ov7695_reg_scene_landscape[] = {
	/* <CAMTUNING_SCENE_LANDSCAPE> */

};

static struct msm_camera_i2c_reg_conf ov7695_reg_scene_night[] = {
	/* <SCENE_NIGHT> */

};
#endif
static struct msm_camera_i2c_reg_conf ov7695_reg_wb_auto[] = {
	/* Auto: */

};

static struct msm_camera_i2c_reg_conf ov7695_reg_wb_sunny[] = {
	/* Sunny: */
	{0x5204, 0x05},
	{0x5205, 0x7b},
	{0x5206, 0x04},
	{0x5207, 0x00},
	{0x5208, 0x05},
	{0x5209, 0x15},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_wb_cloudy[] = {
	/* Cloudy: */
	{0x5204, 0x06},
	{0x5205, 0x00},
	{0x5206, 0x04},
	{0x5207, 0x00},
	{0x5208, 0x04},
	{0x5209, 0x60},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_wb_office[] = {
	/* Office: */
	{0x5204, 0x05},
	{0x5205, 0xa0},
	{0x5206, 0x04},
	{0x5207, 0x00},
	{0x5208, 0x08},
	{0x5209, 0x4e},
};

static struct msm_camera_i2c_reg_conf ov7695_reg_wb_home[] = {
	/* Home: */
	{0x5204, 0x04},
	{0x5205, 0x00},
	{0x5206, 0x04},
	{0x5207, 0xdc},
	{0x5208, 0x0b},
	{0x5209, 0xb4},
};

static void ov7695_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}
}




static int32_t ov7695_mirror_flip(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	int16_t ori = 0x1;//2;
/*
	if (ov7695_orientation == NULL)
		return 0;

	if (strstr(ov7695_orientation, "Normal"))
		ori = 0x0;
	if (strstr(ov7695_orientation, "Mirror"))
		ori = 0x01;
	else if (strstr(ov7695_orientation, "Flip"))
		ori = 0x02;
	else if (strstr(ov7695_orientation, "MF"))
		ori = 0x03;
*/

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		i2c_write(s_ctrl->sensor_i2c_client,
				  0x0101,
				  ori,
				  MSM_CAMERA_I2C_BYTE_DATA);

	return rc;
}

static void ov7695_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	ov7695_i2c_write_table(s_ctrl, &ov7695_reg_saturation[value][0],
		ARRAY_SIZE(ov7695_reg_saturation[value]));
}

static void ov7695_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	ov7695_i2c_write_table(s_ctrl, &ov7695_reg_contrast[value][0],
		ARRAY_SIZE(ov7695_reg_contrast[value]));
}

static void ov7695_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	uint16_t reg3a00;
	CDBG("%s %d", __func__, value);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                                      s_ctrl->sensor_i2c_client,
                                                      0x5808,
                                                      &reg3a00, MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("%s %d", __func__, reg3a00);
	ov7695_i2c_write_table(s_ctrl, &ov7695_reg_sharpness[val][0],
		ARRAY_SIZE(ov7695_reg_sharpness[val]));
}


static void ov7695_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	ov7695_i2c_write_table(s_ctrl, &ov7695_reg_iso[value][0],
		ARRAY_SIZE(ov7695_reg_iso[value]));
}

static void ov7695_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	CDBG("%s %d", __func__, value);
	ov7695_i2c_write_table(s_ctrl, &ov7695_reg_exposure_compensation[val][0],
		ARRAY_SIZE(ov7695_reg_exposure_compensation[val]));
}

static void ov7695_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	uint16_t reg5800;
	CDBG("%s %d", __func__, value);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                                      s_ctrl->sensor_i2c_client,
                                                      0x5800,
                                                      &reg5800, MSM_CAMERA_I2C_BYTE_DATA);

	if (value == MSM_CAMERA_EFFECT_MODE_OFF)
		reg5800 &= (~0x58);
	else if (value == MSM_CAMERA_EFFECT_MODE_NEGATIVE)
		reg5800 = (reg5800 & (~0x18)) | 0x40;
	else
		reg5800 = (reg5800 | 0x18) & (~0x40);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                                      s_ctrl->sensor_i2c_client, 0x5800,
                                                      reg5800,
                                                      MSM_CAMERA_I2C_BYTE_DATA);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_normal[0],
			ARRAY_SIZE(ov7695_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_black_white[0],
			ARRAY_SIZE(ov7695_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_negative[0],
			ARRAY_SIZE(ov7695_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_old_movie[0],
			ARRAY_SIZE(ov7695_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_solarize[0],
			ARRAY_SIZE(ov7695_reg_effect_solarize));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_POSTERIZE: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_redish[0],
			ARRAY_SIZE(ov7695_reg_effect_redish));
		break;
	}
    case MSM_CAMERA_EFFECT_MODE_AQUA: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_blueish[0],
			ARRAY_SIZE(ov7695_reg_effect_blueish));
		break;
	}
	default:
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_effect_normal[0],
			ARRAY_SIZE(ov7695_reg_effect_normal));
	}

}

static void ov7695_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	uint16_t reg5002;
	uint16_t reg3a00;

	CDBG("%s %d", __func__, value);
	ov7695_i2c_write_table(s_ctrl, &ov7695_reg_antibanding[value][0],
		ARRAY_SIZE(ov7695_reg_antibanding[value]));
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                                      s_ctrl->sensor_i2c_client,
                                                      0x3a00,
                                                      &reg3a00, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                                      s_ctrl->sensor_i2c_client,
                                                      0x5002,
                                                      &reg5002, MSM_CAMERA_I2C_BYTE_DATA);
	switch (value) {
	case 0: //off
		reg3a00 &= 0xdf;
		break;
	case 1: //60hz
		reg5002 &= 0xfd;
		reg3a00 |= 0x20;
		break;
	case 2: //50hz
		reg5002 |= 0x02;
		reg3a00 |= 0x20;
		break;
	case 3: //auto
		reg5002 |= 0x02;
		reg3a00 |= 0x20;
		break;
	default:
		break;
	}

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                                      s_ctrl->sensor_i2c_client, 0x3a00,
                                                      reg3a00,
                                                      MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                                      s_ctrl->sensor_i2c_client, 0x5002,
                                                      reg5002,
                                                      MSM_CAMERA_I2C_BYTE_DATA);

}
/*
static void ov7695_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	return;

	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_scene_auto[0],
			ARRAY_SIZE(ov7695_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_scene_night[0],
			ARRAY_SIZE(ov7695_reg_scene_night));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_scene_landscape[0],
			ARRAY_SIZE(ov7695_reg_scene_landscape));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_scene_portrait[0],
			ARRAY_SIZE(ov7695_reg_scene_portrait));
		break;
	}
	default:
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_scene_auto[0],
			ARRAY_SIZE(ov7695_reg_scene_auto));
	}
}
*/
static void ov7695_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{

	uint16_t temp = 0;


	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                                      s_ctrl->sensor_i2c_client,
                                                      0x5200,
                                                      &temp, MSM_CAMERA_I2C_BYTE_DATA);

	if (value == MSM_CAMERA_WB_MODE_AUTO) {
		temp &= 0xdf;
	} else {
		temp |= 0x20;
	}
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                                      s_ctrl->sensor_i2c_client, 0x5200,
                                                      temp,
                                                      MSM_CAMERA_I2C_BYTE_DATA);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_wb_auto[0],
			ARRAY_SIZE(ov7695_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_wb_home[0],
			ARRAY_SIZE(ov7695_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_wb_sunny[0],
			ARRAY_SIZE(ov7695_reg_wb_sunny));
		break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_wb_office[0],
			ARRAY_SIZE(ov7695_reg_wb_office));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_wb_cloudy[0],
			ARRAY_SIZE(ov7695_reg_wb_cloudy));
		break;
	}
	default:
		ov7695_i2c_write_table(s_ctrl, &ov7695_reg_wb_auto[0],
		ARRAY_SIZE(ov7695_reg_wb_auto));
	}
}

int32_t ov7695_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;

	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		/* Write Recommend settings */
		pr_err("%s, sensor write init setting!!", __func__);
#if 1
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x0103,
			0x01, MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
#endif
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			ov7695_recommend_settings,
			ARRAY_SIZE(ov7695_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);

		ov7695_mirror_flip(s_ctrl);
		break;
	case CFG_SET_RESOLUTION:
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
							   ov7695_VGA_settings,
							   ARRAY_SIZE(ov7695_VGA_settings),
							   MSM_CAMERA_I2C_BYTE_DATA);
		break;
	case CFG_SET_STOP_STREAM:
		//msleep(1000);
		pr_err("%s, sensor stop stream!!", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			ov7695_stop_settings,
			ARRAY_SIZE(ov7695_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
        //msleep(1000);
		break;
	case CFG_SET_START_STREAM:
		pr_err("%s, sensor start stream!!", __func__);
        //msleep(1000);//ov said that the cause of the picture of capture look yellow
					 //is that the setting not reset fully when streaming off and on
					 //but actually it no effect with the delay from 10ms to 1s

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			ov7695_start_settings,
			ARRAY_SIZE(ov7695_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		//msleep(1000);
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		kfree(stop_setting->reg_setting);

		break;
	}
	case CFG_SET_SATURATION: {
		int32_t sat_lev;
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Saturation Value is %d", __func__, sat_lev);
		ov7695_set_saturation(s_ctrl, sat_lev);
		break;
	}
	case CFG_SET_CONTRAST: {
		int32_t con_lev;
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Contrast Value is %d", __func__, con_lev);
		ov7695_set_contrast(s_ctrl, con_lev);
		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Sharpness Value is %d", __func__, shp_lev);
		ov7695_set_sharpness(s_ctrl, shp_lev);
		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: ISO Value is %d", __func__, iso_lev);
		ov7695_set_iso(s_ctrl, iso_lev);
		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		ov7695_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Effect mode is %d", __func__, effect_mode);
		ov7695_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: anti-banding mode is %d", __func__,
			antibanding_mode);
		ov7695_set_antibanding(s_ctrl, antibanding_mode);
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: best shot mode is %d", __func__, bs_mode);
		//ov7695_set_scene_mode(s_ctrl, bs_mode);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: white balance is %d", __func__, wb_mode);
		ov7695_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int32_t ov7695_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	//return 0;

	CDBG("%s, E. calling i2c_read:, i2c_addr:0x%x, id_reg_addr:0x%x\n",
		__func__,
		s_ctrl->sensordata->slave_info->sensor_slave_addr,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x300a,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: 0x%x expected id 0x76:\n", __func__, chipid);
	if (chipid != 0x76) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x300b,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: 0x%x expected id 0x95:\n", __func__, chipid);
	if (chipid != 0x95) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	return rc;
}


static struct msm_sensor_fn_t ov7695_sensor_func_tbl = {
	.sensor_config = ov7695_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = ov7695_match_id,
};
#endif

static struct msm_sensor_ctrl_t ov7695_s_ctrl = {
	.sensor_i2c_client = &ov7695_sensor_i2c_client,
	.power_setting_array.power_setting = ov7695_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov7695_power_setting),
	.msm_sensor_mutex = &ov7695_mut,
	.sensor_v4l2_subdev_info = ov7695_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov7695_subdev_info),
#if OV7695_OUTPUT_FORMAT == OV7695_OUTPUT_YUV
	.func_tbl = &ov7695_sensor_func_tbl,
#endif
};
module_init(ov7695_init_module);
module_exit(ov7695_exit_module);
MODULE_DESCRIPTION("ov7695");
MODULE_LICENSE("GPL v2");
