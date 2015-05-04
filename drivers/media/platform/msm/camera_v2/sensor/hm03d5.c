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

#define HM03d5_OUTPUT_YUV 0
#define HM03d5_OUTPUT_RAW 1

#define HM03d5_OUTPUT_FORMAT HM03d5_OUTPUT_YUV

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
#define HM03d5_SENSOR_NAME "hm03d5"
DEFINE_MSM_MUTEX(hm03d5_mut);

static struct msm_sensor_ctrl_t hm03d5_s_ctrl;

static struct msm_sensor_power_setting hm03d5_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 100,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 10,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 10,
	},
};

static struct v4l2_subdev_info hm03d5_subdev_info[] = {
	{
#if HM03d5_OUTPUT_FORMAT == HM03d5_OUTPUT_YUV
		.code   = V4L2_MBUS_FMT_UYVY8_2X8,//V4L2_MBUS_FMT_YUYV8_2X8,//V4L2_MBUS_FMT_VYUY8_2X8,
#endif
#if HM03d5_OUTPUT_FORMAT == HM03d5_OUTPUT_RAW
		.code   = V4L2_MBUS_FMT_SBGGR8_1X8,
#endif
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static int32_t msm_hm03d5_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	CDBG("%s, E.", __func__);

    //Normal
	return msm_sensor_i2c_probe(client, id, &hm03d5_s_ctrl);
}

static const struct i2c_device_id hm03d5_i2c_id[] = {
	{HM03d5_SENSOR_NAME, (kernel_ulong_t)&hm03d5_s_ctrl},
	{ }
};

static struct i2c_driver hm03d5_i2c_driver = {
	.id_table = hm03d5_i2c_id,
	.probe  = msm_hm03d5_i2c_probe,
	.driver = {
		.name = HM03d5_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hm03d5_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};


static const struct of_device_id hm03d5_dt_match[] = {
	{.compatible = "qcom,hm03d5", .data = &hm03d5_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hm03d5_dt_match);

static struct platform_driver hm03d5_platform_driver = {
	.driver = {
		.name = "qcom,hm03d5",
		.owner = THIS_MODULE,
		.of_match_table = hm03d5_dt_match,
	},
};

static int32_t hm03d5_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	CDBG("%s, E.", __func__);
	match = of_match_device(hm03d5_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hm03d5_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hm03d5_platform_driver,
		hm03d5_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hm03d5_i2c_driver);
}

static void __exit hm03d5_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hm03d5_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hm03d5_s_ctrl);
		platform_driver_unregister(&hm03d5_platform_driver);
	} else
		i2c_del_driver(&hm03d5_i2c_driver);
	return;
}

#if HM03d5_OUTPUT_FORMAT == HM03d5_OUTPUT_YUV
static struct msm_camera_i2c_reg_conf hm03d5_start_settings[] = {
		{0x0000,0x01},
		{0x0100,0x01},
		{0x0101,0x01},
		{0x0005,0x01},	//Turn on rolling shutter
};

static struct msm_camera_i2c_reg_conf hm03d5_stop_settings[] = {
		{0x0005,0x00},	//Turn off rolling shutter
};

static struct msm_camera_i2c_reg_conf hm03d5_recommend_settings[] = {
		{0x0025,0x80},
		{0x0024,0x40},
		{0x0027,0x10},
		{0x0040,0x0F},
		{0x0053,0x0F},
		{0x0044,0x06},
		{0x0046,0xD8},
		{0x0055,0x41},
		{0x004A,0x04},
		{0x0026,0x07},
		{0x002a,0x1f},
		{0x008e,0x80},
		{0x0028,0x00},
		{0x0088,0xCD},
		{0x0090,0x00},
		{0x0091,0x10},
		{0x0092,0x11},
		{0x0093,0x12},
		{0x0094,0x13},
		{0x0095,0x17},
		{0x0086,0x82},
		{0x0080,0x80},
		{0x011F,0x54},
		{0x0121,0x80},
		{0x0122,0x6B},
		{0x0123,0xA5},
		{0x0124,0xD2},
		{0x0125,0xDF},
		{0x0126,0x61},
		{0x0140,0x14},
		{0x0141,0x0A},
		{0x0142,0x14},
		{0x0143,0x0A},
		{0x0144,0x08},
		{0x0145,0x00},
		{0x0146,0xF3},
		{0x0148,0x50},
		{0x0149,0x0C},
		{0x014A,0x60},
		{0x014B,0x28},
		{0x014C,0x20},
		{0x014D,0x2E},
		{0x014E,0x05},
		{0x014F,0x05},
		{0x0150,0x0D},
		{0x0155,0x00},
		{0x0156,0x0A},
		{0x0157,0x0A},
		{0x0158,0x0A},
		{0x0159,0x0A},
		{0x015A,0x0C},
		{0x015B,0x0C},
		{0x0160,0x14},
		{0x0161,0x14},
		{0x0162,0x28},
		{0x0163,0x28},
		{0x0164,0x08},
		{0x0165,0x0A},
		{0x0166,0x08},
		{0x0167,0x0A},
		{0x0168,0x04},
		{0x0169,0x06},
		{0x016A,0x04},
		{0x016B,0x06},
		{0x01B0,0x33},
		{0x01B1,0x10},
		{0x01B2,0x10},
		{0x01B3,0x0C},
		{0x01B4,0x0A},
		{0x01D8,0x0C},
		{0x01DE,0x08},
		{0x01E4,0x08},
		{0x01E5,0x0C},
		{0x0220,0x00},
		{0x0221,0xE0},
		{0x0222,0x00},
		{0x0223,0x80},
		{0x0224,0x80},
		{0x0225,0x00},
		{0x0226,0x80},
		{0x022A,0xA8},
		{0x022B,0x00},
		{0x022C,0x80},
		{0x022D,0x11},
		{0x022E,0x10},
		{0x022F,0x11},
		{0x0230,0x10},
		{0x0233,0x11},
		{0x0234,0x10},
		{0x0235,0x4E},
		{0x0236,0x01},
		{0x0237,0x46},
		{0x0238,0x01},
		{0x023B,0x46},
		{0x023C,0x01},
		{0x023D,0xF8},
		{0x023E,0x00},
		{0x023F,0xF8},
		{0x0240,0x00},
		{0x0243,0xF8},
		{0x0244,0x00},
		{0x0251,0x0A},
		{0x0252,0x00},
		{0x0280,0x0B},
		{0x0282,0x15},
		{0x0284,0x2A},
		{0x0286,0x4C},
		{0x0288,0x5A},
		{0x028A,0x67},
		{0x028C,0x73},
		{0x028E,0x7D},
		{0x0290,0x86},
		{0x0292,0x8E},
		{0x0294,0x9A},
		{0x0296,0xA6},
		{0x0298,0xBB},
		{0x029A,0xCF},
		{0x029C,0xE2},
		{0x029E,0x26},
		{0x02A0,0x04},
		{0x02C0,0xB1},
		{0x02C1,0x01},
		{0x02C2,0x7D},
		{0x02C3,0x07},
		{0x02C4,0xD2},
		{0x02C5,0x07},
		{0x02C6,0xC4},
		{0x02C7,0x07},
		{0x02C8,0x79},
		{0x02C9,0x01},
		{0x02CA,0xC4},
		{0x02CB,0x07},
		{0x02CC,0xF7},
		{0x02CD,0x07},
		{0x02CE,0x3B},
		{0x02CF,0x07},
		{0x02D0,0xCF},
		{0x02D1,0x01},
		{0x0302,0x00},
		{0x0303,0x00},
		{0x0304,0x00},
		{0x02E0,0x04},
		{0x02F0,0x5E},
		{0x02F1,0x07},
		{0x02F2,0xA0},
		{0x02F3,0x00},
		{0x02F4,0x02},
		{0x02F5,0x00},
		{0x02F6,0xC4},
		{0x02F7,0x07},
		{0x02F8,0x11},
		{0x02F9,0x00},
		{0x02FA,0x2A},
		{0x02FB,0x00},
		{0x02FC,0xA1},
		{0x02FD,0x07},
		{0x02FE,0xB8},
		{0x02FF,0x07},
		{0x0300,0xA7},
		{0x0301,0x00},
		{0x0305,0x00},
		{0x0306,0x00},
		{0x0307,0x7A},
		{0x0328,0x00},
		{0x0329,0x04},
		{0x032D,0x66},
		{0x032E,0x01},
		{0x032F,0x00},
		{0x0330,0x01},
		{0x0331,0x66},
		{0x0332,0x01},
		{0x0333,0x00},
		{0x0334,0x00},
		{0x0335,0x00},
		{0x033E,0x00},
		{0x033F,0x00},
		{0x0340,0x38},
		{0x0341,0x84},
		{0x0342,0x31},
		{0x0343,0x44},
		{0x0344,0x6C},
		{0x0345,0x40},
		{0x0346,0x78},
		{0x0347,0x5C},
		{0x0348,0x58},
		{0x0349,0x55},
		{0x034A,0x5E},
		{0x034B,0x69},
		{0x034C,0x4A},
		{0x0350,0x80},
		{0x0351,0x90},
		{0x0352,0x08},
		{0x0353,0x18},
		{0x0354,0x73},
		{0x0355,0x45},
		{0x0356,0x78},
		{0x0357,0xD0},
		{0x0358,0x05},
		{0x035A,0x05},
		{0x035B,0xA0},
		{0x0381,0x52},
		{0x0382,0x3A},
		{0x0383,0x20},
		{0x038A,0x80},
		{0x038B,0x0C},
		{0x038C,0xC1},
		{0x038E,0x46},
		{0x038F,0x05},
		{0x0390,0xF4},
		{0x0391,0x10},
		{0x0393,0x80},
		{0x0395,0x12},
		{0x0398,0x01},
		{0x0399,0xF0},
		{0x039A,0x03},
		{0x039B,0x00},
		{0x039C,0x04},
		{0x039D,0x00},
		{0x039E,0x06},
		{0x039F,0x00},
		{0x03A0,0x09},
		{0x03A1,0x50},
		{0x03A6,0x10},
		{0x03A7,0x10},
		{0x03A8,0x36},
		{0x03A9,0x40},
		{0x03AE,0x26},
		{0x03AF,0x22},
		{0x03B0,0x0A},
		{0x03B1,0x08},
		{0x03B3,0x00},
		{0x03B5,0x08},
		{0x03B7,0xA0},
		{0x03B9,0xD0},
		{0x03BB,0xFF},
		{0x03BC,0xFF},
		{0x03BE,0x04},
		{0x03BF,0x1D},
		{0x03C0,0x2E},
		{0x03C3,0x0F},
		{0x03D0,0xE0},
		{0x0420,0x82},
		{0x0421,0x00},
		{0x0422,0x00},
		{0x0423,0x84},
		{0x0430,0x00},
		{0x0431,0x68},
		{0x0432,0x28},
		{0x0433,0x30},
		{0x0434,0x00},
		{0x0435,0x40},
		{0x0436,0x00},
		{0x0450,0xFF},
		{0x0451,0xFF},
		{0x0452,0xC0},
		{0x0453,0x70},
		{0x0454,0x00},
		{0x045A,0x00},
		{0x045B,0x10},
		{0x045C,0x00},
		{0x045D,0xA0},
		{0x0465,0x02},
		{0x0466,0x34},
		{0x047A,0x00},
		{0x047B,0x00},
		{0x0480,0x50},
		{0x0481,0x06},
		{0x04B0,0x4C},
		{0x04B6,0x38},
		{0x04B9,0x0C},
		{0x04B3,0x08},
		{0x04B1,0x8C},
		{0x04B4,0x20},
		{0x0540,0x00},
		{0x0541,0x7F},
		{0x0542,0x00},
		{0x0543,0x98},
		{0x0580,0x01},
		{0x0581,0x04},
		{0x0590,0x20},
		{0x0591,0x30},
		{0x0594,0x02},
		{0x0595,0x08},
		{0x05A0,0x04},
		{0x05A1,0x10},
		{0x05A2,0xA0},
		{0x05A3,0xA0},
		{0x05A5,0x20},
		{0x05A6,0x20},
		{0x05B0,0x50},
		{0x05B1,0x02},
		{0x05B2,0x50},
		{0x05B3,0x04},
		{0x05D0,0x10},
		{0x05D1,0x06},
		{0x05E4,0x04},
		{0x05E5,0x00},
		{0x05E6,0x83},
		{0x05E7,0x02},
		{0x05E8,0x04},
		{0x05E9,0x00},
		{0x05EA,0xE3},
		{0x05EB,0x01},
		{0x0b20,0x9E},
		{0x007C,0x02},
		{0x007D,0x01},
};

static struct msm_camera_i2c_reg_conf hm03d5_VGA_settings[] = {
		{0x0027,0x10},//CbYCrY
		{0x008e,0x80},//[7:6] mclk div, [5:4] iq
		{0x0124,0xd2},
		{0x0125,0xdf},//Scaler OFF
/*
		{0x000d,0x00},
		{0x000e,0x00},
*/
		{0x05e4,0x04},//Windowing for 640x480
		{0x05e5,0x00},
		{0x05e6,0x83},
		{0x05e7,0x02},
		{0x05e8,0x04},
		{0x05e9,0x00},
		{0x05ea,0xe3},
		{0x05eb,0x01},

        //MIPI TX related settings
		{0x0b02,0x01},//tlpx_width
		{0x0b03,0x02},//hs_zero_width
		{0x0b04,0x00},//hs_trail_width
		{0x0b05,0x05},//clk_zero_width
		{0x0b06,0x02},//clk_trail_width
		{0x0b07,0x2e},//mark1_width

		{0x0b0e,0x07},//clk_front_proch
		{0x0b0f,0x00},//clk_back_proch
		{0x0b11,0x7f},//MIPI1 - LP strength control for clock lane
		{0x0b12,0x7f},//MIPI2 - LP strength control for data lane
		{0x0b33,0x00},//dbg_sel [7]:dbg_insert_state, [6] dbg_fpga
		{0x0b22,0x02},//hs_exit enable
		{0x0b32,0x00},//
		{0x0b35,0x00},//te_width_lh
		{0x0b36,0x00},//te_width_hl
		{0x0b37,0x00},//clk_width_lh
		{0x0b38,0x00},//clk_width_hl
		{0x0b39,0x00},//hs_exit at least 100ns
		{0x0b3a,0x00},//data_exit at least 100ns
		{0x0b40,0x01},//
		{0x0b34,0x07},//rd_buf_latency
		{0x0b3f,0x09},//[6] ths_prepare_suppress, [5] tclk_prepare_suppress, [4:0] mipi_request_delay
		{0x0b42,0x00},

		{0x0b20,0xbe},//0xbe},//0x9e},
		{0x007c,0x06},//pre-hsync 0x2
		{0x007d,0x01},//pre-vsync
};

static struct msm_camera_i2c_reg_conf hm03d5_QVGA_settings[] = {
		{0x0027,0x10},//CbYCrY
		{0x008e,0x90},//[7:6] mclk div, [5:4] iq
		{0x0124,0xd2},
		{0x0125,0xff},//Scaler ON
/*
		{0x000d,0x01},
		{0x000e,0x81},
*/
		{0x05e5,0x00},
		{0x05e4,0x02},
		{0x05e7,0x01},
		{0x05e6,0x41},
		{0x05e9,0x00},
		{0x05e8,0x04},
		{0x05eb,0x00},
		{0x05ea,0xf3},

        //MIPI TX related settings
		{0x0b03,0x00},//hs_zero_width
		{0x0b04,0x03},//hs_trail_width
		{0x0b0e,0x01},//clk_front_proch
		{0x0b0f,0x01},//clk_back_proch
		{0x0b11,0x7f},//MIPI1 - LP strength control for clock lane
		{0x0b12,0x7f},//MIPI2 - LP strength control for data lane
		{0x0b3f,0x02},//[6] ths_prepare_suppress, [5] tclk_prepare_suppress, [4:0] mipi_request_delay
		{0x0b42,0x01},

		{0x0b20,0x9e},
		{0x007c,0x03},//pre-hsync
		{0x007d,0x01},//pre-vsync
};

static struct msm_camera_i2c_reg_conf hm03d5_QQVGA_settings[] = {
		{0x0027,0x10},//CbYCrY
		{0x008e,0xe0},//[7:6] mclk div, [5:4] iq
		{0x0124,0xf2},
		{0x0125,0xff},//Scaler ON
/*
		{0x000d,0x01},
		{0x000e,0x11},
*/
		{0x05e5,0x00},
		{0x05e4,0x02},
		{0x05e7,0x01},
		{0x05e6,0x41},
		{0x05e9,0x00},
		{0x05e8,0x04},
		{0x05eb,0x00},
		{0x05ea,0xf3},

        //MIPI TX related settings
		{0x0b03,0x00},//hs_zero_width
		{0x0b04,0x03},//hs_trail_width
		{0x0b0e,0x01},//clk_front_proch
		{0x0b0f,0x01},//clk_back_proch
		{0x0b11,0x7f},//MIPI1 - LP strength control for clock lane
		{0x0b12,0x7f},//MIPI2 - LP strength control for data lane
		{0x0b3f,0x02},//[6] ths_prepare_suppress, [5] tclk_prepare_suppress, [4:0] mipi_request_delay
		{0x0b42,0x01},

		{0x0b20,0x9e},
		{0x007c,0x20},//pre-hsync
		{0x007d,0x01},//pre-vsync
};

static struct msm_camera_i2c_reg_conf hm03d5_CIF_settings[] = {
		{0x0027,0x10},//CbYCrY
		{0x008e,0x80},//[7:6] mclk div, [5:4] iq
		{0x0124,0xd2},
		{0x0125,0xdf},//Scaler ON
/*
		{0x000d,0x00},
		{0x000e,0x00},
*/
		{0x05e5,0x00},
		{0x05e4,0x04},
		{0x05e7,0x01},
		{0x05e6,0x63},
		{0x05e9,0x00},
		{0x05e8,0x05},
		{0x05eb,0x01},
		{0x05ea,0x24},

        //MIPI TX related settings
		{0x0b03,0x00},//hs_zero_width
		{0x0b04,0x03},//hs_trail_width
		{0x0b0e,0x01},//clk_front_proch
		{0x0b0f,0x01},//clk_back_proch
		{0x0b11,0x7f},//MIPI1 - LP strength control for clock lane
		{0x0b12,0x7f},//MIPI2 - LP strength control for data lane
		{0x0b3f,0x02},//[6] ths_prepare_suppress, [5] tclk_prepare_suppress, [4:0] mipi_request_delay
		{0x0b42,0x01},

		{0x0b20,0x9e},
		{0x007c,0x20},//pre-hsync
		{0x007d,0x01},//pre-vsync
};

static struct msm_camera_i2c_reg_conf hm03d5_QCIF_settings[] = {
		{0x0027,0x10},//CbYCrY
		{0x008e,0x90},//[7:6] mclk div, [5:4] iq
		{0x0124,0xd2},
		{0x0125,0xff},//Scaler ON
/*
		{0x000d,0x01},
		{0x000e,0x11},
*/
		{0x05e5,0x00},
		{0x05e4,0x04},
		{0x05e7,0x00},
		{0x05e6,0xB3},
		{0x05e9,0x00},
		{0x05e8,0x05},
		{0x05eb,0x00},
		{0x05ea,0x94},

        //MIPI TX related settings
		{0x0b03,0x00},//hs_zero_width
		{0x0b04,0x03},//hs_trail_width
		{0x0b0e,0x01},//clk_front_proch
		{0x0b0f,0x01},//clk_back_proch
		{0x0b11,0x7f},//MIPI1 - LP strength control for clock lane
		{0x0b12,0x7f},//MIPI2 - LP strength control for data lane
		{0x0b3f,0x02},//[6] ths_prepare_suppress, [5] tclk_prepare_suppress, [4:0] mipi_request_delay
		{0x0b42,0x01},

		{0x0b20,0x9e},
		{0x007c,0x03},//pre-hsync
		{0x007d,0x01},//pre-vsync
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_ae_awb_command_update[] = {
		{0x0000, 0xFF},	//AE CMU
		{0x0100, 0xFF},	//AE CMU
		{0x0101, 0xFF},	//AE CMU
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_saturation[11][1] = {
    {
        //	-5 Level:
	    {0x0480,0x0A},
    },
	{
        //	-4 Level:
	    {0x0480,0x14},
	},
    {
        //	-3 Level:
	    {0x0480,0x1D},
    },
	{
        //	-2 Level:
	    {0x0480,0x28},
	},
	{
        //	-1 Level:
	    {0x0480,0x32},
	},
	{
        //	0 Level:
	    {0x0480,0x40},
	},
	{
        //	+1 Level:
	    {0x0480,0x50},
	},
	{
        //	+2 Level:
	    {0x0480,0x60},
	},
	{
        //	+3 Level:
	    {0x0480,0x70},
    },
	{
        //	+4 Level:
	    {0x0480,0x80},
	},
	{
        //	+5 Level:
	    {0x0480,0x90},
    },
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_contrast[11][1] = {
    {
        //	-5 Level:
	    {0x04B0,0x0A},
    },
	{
        //	-4 Level:
	    {0x04B0,0x0D},
	},
    {
        //	-3 Level:
	    {0x04B0,0x10},
    },
	{
        //	-2 Level:
	    {0x04B0,0x20},
	},
	{
        //	-1 Level:
	    {0x04B0,0x30},
	},
	{
        //	0 Level:
	    {0x04B0,0x40},
	},
	{
        //	+1 Level:
	    {0x04B0,0x50},
	},
	{
        //	+2 Level:
	    {0x04B0,0x60},
	},
	{
        //	+3 Level:
	    {0x04B0,0x70},
    },
	{
        //	+4 Level:
	    {0x04B0,0x80},
	},
	{
        //	+5 Level:
	    {0x04B0,0x90},
    },
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_sharpness[7][2] = {
	{
	    {0x5B0,0x08},
	    {0x5B2,0x08},

	}, /* SHARPNESS LEVEL 0*/
	{
	    {0x5B0,0x10},
	    {0x5B2,0x10},

	}, /* SHARPNESS LEVEL 1*/
	{
	    {0x5B0,0x20},
	    {0x5B2,0x20},

	}, /* SHARPNESS LEVEL 2*/
	{
	    {0x5B0,0x30},
	    {0x5B2,0x30},

	}, /* SHARPNESS LEVEL 3*/
	{
	    {0x5B0,0x40},
	    {0x5B2,0x40},

	}, /* SHARPNESS LEVEL 4*/
	{
	    {0x5B0,0x50},
	    {0x5B2,0x50},

	}, /* SHARPNESS LEVEL 5*/
	{
	    {0x5B0,0x60},
	    {0x5B2,0x60},

	}, /* SHARPNESS LEVEL 6*/
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_iso[7][2] = {
     /* auto */
	{
		{0x0392, 0x03},
		{0x0393, 0x80},
	},
	/* auto hjt */
	{
		{0x0392, 0x03},
		{0x0393, 0x80},
	},
	/* iso 100 */
	{
		{0x0392,0x03},
		{0x0393,0x50},
	},
	/* iso 200 */
	{
		{0x0392,0x03},
		{0x0393,0x60},
	},
	/* iso 400 */
	{
		{0x0392,0x02},
		{0x0393,0x70},
	},
	/* iso 800 */
	{
		{0x0392,0x03},
		{0x0393,0x80},
	},
    /* iso 1600 */
	{
		{0x0392,0x03},
		{0x0393,0xF0},
	},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_exposure_compensation[9][4] = {
	/* -4 */
	{// AE_EV_COMP_n13:
		{0x04C0,0xC0},
		{0x038E,0x28},	// 20-28-30
		{0x0381,0x30},
		{0x0382,0x20},
	},
	/* -3 */
	{// AE_EV_COMP_n10:
		{0x04C0,0xB0},
		{0x038E,0x30},	// 28-30-38
		{0x0381,0x38},
		{0x0382,0x28},
	},
	/* -2 */
	{// AE_EV_COMP_n07:
		{0x04C0,0xA0},
		{0x038E,0x38},  // 30-38-40
		{0x0381,0x40},
		{0x0382,0x30},
	},
	/* -1 */
	{// AE_EV_COMP_n03:
		{0x04C0,0x90},
		{0x038E,0x40},	// 38-40-48
		{0x0381,0x48},
		{0x0382,0x38},
	},
	/* 0 */
	{// AE_EV_COMP_00:
		{0x04C0,0x00},
		{0x038E,0x48},	// 40-48-50
		{0x0381,0x50},
		{0x0382,0x40},
	},
	/* 1 */
	{// AE_EV_COMP_03:
		{0x04C0,0x10},
		{0x038E,0x50},	// 48-50-58
		{0x0381,0x58},
		{0x0382,0x48},
	},
	/* 2 */
	{// AE_EV_COMP_07:
		{0x04C0,0x20},
		{0x038E,0x58},	// 50-58-60
		{0x0381,0x60},
		{0x0382,0x50},
	},
	/* 3 */
	{// AE_EV_COMP_10:
		{0x04C0,0x30},
		{0x038E,0x60},	// 58-60-68
		{0x0381,0x68},
		{0x0382,0x58},
	},
	/* 4 */
	{// AE_EV_COMP_13:
		{0x04C0,0x40},
		{0x038E,0x68},	// 60-68-70
		{0x0381,0x70},
		{0x0382,0x60},
	},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_update[] = {
	{0x0005, 0x01},
	{0x0000, 0x01},
	{0x0100, 0xFF},
	{0x0101, 0xFF},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_disable_settings[] = {
	{0x0005, 0x00},
	{0x0000, 0x01},
	{0x0100, 0xFF},
	{0x0101, 0xFF},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_normal[] = {
	/* normal: */
	{0x0488, 0x10}, //[0]:Image scense, [1]:Image Xor
	{0x0486, 0x00}, //Hue, sin
	{0x0487, 0xFF}, //Hue, cos
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_black_white[] = {
	/* B&W: */
	{0x0486, 0x80}, //Hue, sin
	{0x0487, 0x80}, //Hue, cos
	{0x0488, 0x11}, //[0]:Image scense, [1]:Image Xor
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_negative[] = {
	/* Negative: */
	{0x0488, 0x12}, //[0]:Image scense, [1]:Image Xor
	{0x0486, 0x00}, //Hue, sin
	{0x0487, 0xFF}, //Hue, cos
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_old_movie[] = {
	/* Sepia(antique): */
	{0x0486, 0x40}, //Hue, sin
    {0x0487, 0xA0}, //Hue, cos
	{0x0488, 0x11}, //[0]:Image scense, [1]:Image Xor
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_solarize[] = {
    //greenish
	{0x0486, 0x60}, //Hue, sin
    {0x0487, 0x60}, //Hue, cos
	{0x0488, 0x11}, //[0]:Image scense, [1]:Image Xor
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_redish[] = {
    //redish
	{0x0486, 0xB0}, //Hue, sin
    {0x0487, 0x80}, //Hue, cos
	{0x0488, 0x11}, //[0]:Image scense, [1]:Image Xor
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_effect_blueish[] = {
    //blueish
	{0x0486, 0xB0}, //Hue, sin
    {0x0487, 0x80}, //Hue, cos
	{0x0488, 0x11}, //[0]:Image scense, [1]:Image Xor
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_scene_auto[] = {
	/* <SCENE_auto> */
	{0x0390, 0xF8},        //15-30fps
	{0x038F, 0x03},
	{0x0015, 0x00},
	{0x038F, 0x03},
	{0x0390, 0xF8},
	{0x0392, 0x03},       //Gain
	{0x038E, 0x4C},       //AE Target
	{0x0381, 0x58},
	{0x0382, 0x40},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_scene_portrait[] = {
	/* <CAMTUNING_SCENE_PORTRAIT> */
	{0x0390, 0xF4},        //10fps
	{0x038F, 0x05},
	{0x0015, 0x00},
	{0x038F, 0x05},
	{0x0390, 0xF4},
	{0x0392, 0x03},        //Gain
	{0x038E, 0x5C},        //AE Target
	{0x0381, 0x68},
	{0x0382, 0x50},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_scene_landscape[] = {
	/* <CAMTUNING_SCENE_LANDSCAPE> */
	{0x0390, 0xFA},        //20fps
	{0x038F, 0x02},
	{0x0015, 0x00},
	{0x038F, 0x02},
	{0x0390, 0x20},
	{0x0392, 0x03},        //Gain
	{0x038E, 0x54},        //AE Target
	{0x0381, 0x60},
	{0x0382, 0x48},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_scene_night[] = {
	/* <SCENE_NIGHT> */
	{0x0390, 0xE8},        //5fps
	{0x038F, 0x0B},
	{0x0015, 0x00},
	{0x038F, 0x0B},
	{0x0390, 0xE8},
	{0x0392, 0x04},        //Gain
    {0x038E, 0x5C},        //AE Target
	{0x0381, 0x68},
	{0x0382, 0x50},
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_wb_auto[] = {
	/* Auto: */
	{0x0380, 0xff},   // select Auto WB
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_wb_sunny[] = {
	/* Sunny: */
//	{0x0380, 0xFD},  	//Disable AWB
	{0x032D, 0x60},
	{0x032E, 0x01}, 	//Red
	{0x032F, 0x00},
	{0x0330, 0x01},		//Green
	{0x0331, 0x20},
	{0x0332, 0x01},		//Blue
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_wb_cloudy[] = {
	/* Cloudy: */
	{0x032D, 0x70},
	{0x032E, 0x01}, 	//Red
	{0x032F, 0x00},
	{0x0330, 0x01},		//Green
	{0x0331, 0x08},
	{0x0332, 0x01},		//Blue
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_wb_office[] = {
	/* Office: */
	{0x032D, 0x00},
	{0x032E, 0x01}, 	//Red
	{0x032F, 0x14},
	{0x0330, 0x01},		//Green
	{0x0331, 0xD6},
	{0x0332, 0x01},		//Blue
};

static struct msm_camera_i2c_reg_conf hm03d5_reg_wb_home[] = {
	/* Home: */
	{0x032D, 0x10},
	{0x032E, 0x01}, 	//Red
	{0x032F, 0x00},
	{0x0330, 0x01},		//Green
	{0x0331, 0xA0},
	{0x0332, 0x01},		//Blue
};

static void hm03d5_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
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


static int32_t hm03d5_mirror_flip(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	int16_t ori = 0x3;//2;//2;
/*
	switch (image_mirror)
	{
	case IMAGE_NORMAL:
		HM03D5_write_cmos_sensor(0x0006, 0x00);     //0x02
		HM03D5_write_cmos_sensor(0x0000, 0x01);

		break;
	case IMAGE_H_MIRROR:
		HM03D5_write_cmos_sensor(0x0006, 0x02);
		HM03D5_write_cmos_sensor(0x0000, 0x01);

		break;
	case IMAGE_V_MIRROR:
		HM03D5_write_cmos_sensor(0x0006, 0x01);
		HM03D5_write_cmos_sensor(0x0000, 0x01);

		break;
	case IMAGE_HV_MIRROR:
		HM03D5_write_cmos_sensor(0x0006, 0x03);    //0x01
		HM03D5_write_cmos_sensor(0x0000, 0x01);

		break;
*/
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		i2c_write(s_ctrl->sensor_i2c_client,
				  0x0006,
				  ori,
				  MSM_CAMERA_I2C_BYTE_DATA);

	return rc;
}

static void hm03d5_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);

	hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_saturation[value][0],
		ARRAY_SIZE(hm03d5_reg_saturation[value]));
}

static void hm03d5_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_contrast[value][0],
		ARRAY_SIZE(hm03d5_reg_contrast[value]));
}

static void hm03d5_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	CDBG("%s %d", __func__, value);
	hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_sharpness[val][0],
		ARRAY_SIZE(hm03d5_reg_sharpness[val]));
}

static void hm03d5_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	long rc = 0;
	CDBG("%s %d", __func__, value);

	hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_iso[value][0],
		ARRAY_SIZE(hm03d5_reg_iso[value]));

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_reg_ae_awb_command_update,
			ARRAY_SIZE(hm03d5_reg_ae_awb_command_update),
			MSM_CAMERA_I2C_BYTE_DATA);
}

static void hm03d5_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	long rc = 0;
	int val = (value + 12) / 6;
	CDBG("%s %d %d", __func__, value,val);

	hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_exposure_compensation[val][0],
		ARRAY_SIZE(hm03d5_reg_exposure_compensation[val]));

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_reg_ae_awb_command_update,
			ARRAY_SIZE(hm03d5_reg_ae_awb_command_update),
			MSM_CAMERA_I2C_BYTE_DATA);
}

static void hm03d5_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	long rc = 0;
    unsigned short temp = 0;

	CDBG("%s %d %ld", __func__, value,rc);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x0120,
			&temp, MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("%s reg[0x0120]=%d ", __func__, temp);
    temp = temp | ((1<<2));//|(1<<0));
	CDBG("%s temp=%d ", __func__, temp);

    if((value == MSM_CAMERA_EFFECT_MODE_OFF)||(value == MSM_CAMERA_EFFECT_MODE_NEGATIVE))
    {
        temp = temp | ((1<<4)|(1<<5));
    }
    else
    {
        temp = temp | ((1<<4));
    }
	CDBG("%s temp=%d ", __func__, temp);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_reg_effect_disable_settings,
			ARRAY_SIZE(hm03d5_reg_effect_disable_settings),
			MSM_CAMERA_I2C_BYTE_DATA);

	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_normal[0],
			ARRAY_SIZE(hm03d5_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_black_white[0],
			ARRAY_SIZE(hm03d5_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_negative[0],
			ARRAY_SIZE(hm03d5_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_old_movie[0],
			ARRAY_SIZE(hm03d5_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_solarize[0],
			ARRAY_SIZE(hm03d5_reg_effect_solarize));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_POSTERIZE: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_redish[0],
			ARRAY_SIZE(hm03d5_reg_effect_redish));
		break;
	}
    case MSM_CAMERA_EFFECT_MODE_AQUA: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_blueish[0],
			ARRAY_SIZE(hm03d5_reg_effect_blueish));
		break;
	}
	default:
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_effect_normal[0],
			ARRAY_SIZE(hm03d5_reg_effect_normal));
	}

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write(s_ctrl->sensor_i2c_client,
				  0x0120,
				  temp,
				  MSM_CAMERA_I2C_BYTE_DATA);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_reg_effect_update,
			ARRAY_SIZE(hm03d5_reg_effect_update),
			MSM_CAMERA_I2C_BYTE_DATA);
}

static void hm03d5_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
    long rc = 0;
    unsigned short temp = 0;
	CDBG("%s %d ", __func__, value);

    if(value == 1)//50hz
    {
        temp = temp & (~0x1);
    }
    else if(value == 2)
    {
        temp = temp | (0x1);
    }
	CDBG("%s temp=%d ", __func__, temp);

    if((value == 1)||(value == 2))
    {
        //0:off 1:50 hz 2:60 hz 3 Auto
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x0120,
			&temp, MSM_CAMERA_I2C_BYTE_DATA);
	    CDBG("%s reg[0x0120]=%d ", __func__, temp);

	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write(s_ctrl->sensor_i2c_client,
				  0x0120,
				  temp,
				  MSM_CAMERA_I2C_BYTE_DATA);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_reg_ae_awb_command_update,
			ARRAY_SIZE(hm03d5_reg_ae_awb_command_update),
			MSM_CAMERA_I2C_BYTE_DATA);
    }
}

static void hm03d5_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
    long rc = 0;
    CDBG("%s %d", __func__, value);
	//return;

	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_scene_auto[0],
			ARRAY_SIZE(hm03d5_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_scene_night[0],
			ARRAY_SIZE(hm03d5_reg_scene_night));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_scene_landscape[0],
			ARRAY_SIZE(hm03d5_reg_scene_landscape));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_scene_portrait[0],
			ARRAY_SIZE(hm03d5_reg_scene_portrait));
		break;
	}
	default:
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_scene_auto[0],
			ARRAY_SIZE(hm03d5_reg_scene_auto));
	}

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_reg_effect_update,
			ARRAY_SIZE(hm03d5_reg_effect_update),
			MSM_CAMERA_I2C_BYTE_DATA);
}

static void hm03d5_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
    long rc = 0;
    unsigned short temp = 0;

	//CDBG("%s %d %ld", __func__, value,rc);
	CDBG("%s %d ", __func__, value);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x0380,
			&temp, MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("%s reg[0x0380]=%d ", __func__, temp);
    temp = temp | (1<<0);

    if(value != MSM_CAMERA_WB_MODE_AUTO)
    {
        temp = temp & (~0x2);
    }
    else
    {
        temp = temp | (0x2);
    }

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write(s_ctrl->sensor_i2c_client,
				  0x0380,
				  temp,
				  MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("%s reg[0x0380]=%d ", __func__, temp);

	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_wb_auto[0],
			ARRAY_SIZE(hm03d5_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_wb_sunny[0],
			ARRAY_SIZE(hm03d5_reg_wb_sunny));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_wb_home[0],
			ARRAY_SIZE(hm03d5_reg_wb_home));
		break;
	}

	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_wb_office[0],
			ARRAY_SIZE(hm03d5_reg_wb_office));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_wb_cloudy[0],
			ARRAY_SIZE(hm03d5_reg_wb_cloudy));
		break;
	}
	default:
		hm03d5_i2c_write_table(s_ctrl, &hm03d5_reg_wb_auto[0],
		ARRAY_SIZE(hm03d5_reg_wb_auto));
	}

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_reg_ae_awb_command_update,
			ARRAY_SIZE(hm03d5_reg_ae_awb_command_update),
			MSM_CAMERA_I2C_BYTE_DATA);
}

int32_t hm03d5_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
    unsigned short temp=0;

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
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write(s_ctrl->sensor_i2c_client,
				  0x0022,
				  0x01,
				  MSM_CAMERA_I2C_BYTE_DATA);
		mdelay(50);
		/* Write Recommend settings */
		pr_err("%s, sensor write init setting!!", __func__);

        hm03d5_mirror_flip(s_ctrl);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_recommend_settings,
			ARRAY_SIZE(hm03d5_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);

		mdelay(400);

		break;
	case CFG_SET_RESOLUTION:{
		int val = 0;
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s %d, sensor size", __func__,val);

        if (val == 0)
		    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
							   hm03d5_VGA_settings,
							   ARRAY_SIZE(hm03d5_VGA_settings),
							   MSM_CAMERA_I2C_BYTE_DATA);
		else if (val == 1)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
							   hm03d5_QVGA_settings,
							   ARRAY_SIZE(hm03d5_QVGA_settings),
							   MSM_CAMERA_I2C_BYTE_DATA);
		else if (val == 2)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
							   hm03d5_QQVGA_settings,
							   ARRAY_SIZE(hm03d5_QQVGA_settings),
							   MSM_CAMERA_I2C_BYTE_DATA);
		else if (val == 3)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
							   hm03d5_CIF_settings,
							   ARRAY_SIZE(hm03d5_CIF_settings),
							   MSM_CAMERA_I2C_BYTE_DATA);
		else if (val == 4)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
							   hm03d5_QCIF_settings,
							   ARRAY_SIZE(hm03d5_QCIF_settings),
							   MSM_CAMERA_I2C_BYTE_DATA);
        break;
    }
	case CFG_SET_STOP_STREAM:
		pr_err("%s, sensor stop stream!!", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_stop_settings,
			ARRAY_SIZE(hm03d5_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);

		break;
	case CFG_SET_START_STREAM:
		pr_err("%s, sensor start stream!!", __func__);

    while(1)
    {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			hm03d5_start_settings,
			ARRAY_SIZE(hm03d5_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
	    mdelay(50);	 //20121025

	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x0005,
			&temp, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d 0x0005 read %d\n", __func__, __LINE__,
			temp);
        if(temp==1)
            break;
    }
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
		hm03d5_set_saturation(s_ctrl, sat_lev);

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
		hm03d5_set_contrast(s_ctrl, con_lev);

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
		hm03d5_set_sharpness(s_ctrl, shp_lev);

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
		hm03d5_set_iso(s_ctrl, iso_lev);

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
		hm03d5_set_exposure_compensation(s_ctrl, ec_lev);

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
		hm03d5_set_effect(s_ctrl, effect_mode);

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
		hm03d5_set_antibanding(s_ctrl, antibanding_mode);

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
		hm03d5_set_scene_mode(s_ctrl, bs_mode);

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
		hm03d5_set_white_balance_mode(s_ctrl, wb_mode);

		break;
	}

	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int32_t hm03d5_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	uint16_t temp_chipid = 0;

	CDBG("%s, E. calling i2c_read:, i2c_addr:0x%x, id_reg_addr:0x%x\n",
		__func__,
		s_ctrl->sensordata->slave_info->sensor_slave_addr,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x0001,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

    temp_chipid = 0x03;
	CDBG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,temp_chipid);

	if (chipid != temp_chipid) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x0002,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

    temp_chipid = 0xd5;
	CDBG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,temp_chipid);

	if (chipid != temp_chipid) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	return rc;
}

static struct msm_sensor_fn_t hm03d5_sensor_func_tbl = {
	.sensor_config = hm03d5_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = hm03d5_match_id,
};
#endif

static struct msm_sensor_ctrl_t hm03d5_s_ctrl = {
	.sensor_i2c_client = &hm03d5_sensor_i2c_client,
	.power_setting_array.power_setting = hm03d5_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hm03d5_power_setting),
	.msm_sensor_mutex = &hm03d5_mut,
	.sensor_v4l2_subdev_info = hm03d5_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hm03d5_subdev_info),
#if HM03d5_OUTPUT_FORMAT == HM03d5_OUTPUT_YUV
	.func_tbl = &hm03d5_sensor_func_tbl,
#endif
};
module_init(hm03d5_init_module);
module_exit(hm03d5_exit_module);
MODULE_DESCRIPTION("hm03d5");
MODULE_LICENSE("GPL v2");
