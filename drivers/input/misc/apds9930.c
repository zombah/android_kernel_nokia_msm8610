/*
*  apds9930.c - Linux kernel modules for ambient light + proximity sensor
*
*  Copyright (C) 2013 Zifeng Qiu <zifeng.qiu@nokia.com>
*  Copyright (C) 2013 NOKIA
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/io.h>

#define KMSGERR(...)    dev_err(&apds9930_i2c_client->dev, __VA_ARGS__)
#define KMSGINF(...)    dev_info(&apds9930_i2c_client->dev, __VA_ARGS__)

#define APDS9930_DRV_NAME   "apds9930_prox_als"
#define DRIVER_VERSION      "1.1.0"

#define REL_LIGHT           REL_MISC  // added to support LIGHT - light sensor

#define APDS9930_ALS_THRESHOLD_HSYTERESIS   20  /* 20 = 20% */

#define APDS9930_NV_BIN_FILE_NAME           "/persist/apds9930.bin"

#define APDS9930_DEVICE_ID                  0x39

/* Change History
*
* 1.0.0  Fundamental Functions of APDS-993x
*
*/

#define APDS_IOCTL_PS_ENABLE        1
#define APDS_IOCTL_PS_GET_ENABLE    2
#define APDS_IOCTL_PS_POLL_DELAY    3
#define APDS_IOCTL_ALS_ENABLE       4
#define APDS_IOCTL_ALS_GET_ENABLE   5
#define APDS_IOCTL_ALS_POLL_DELAY   6
#define APDS_IOCTL_PS_GET_PDATA     7 // pdata
#define APDS_IOCTL_ALS_GET_CH0DATA  8 // ch0data
#define APDS_IOCTL_ALS_GET_CH1DATA  9 // ch1data

#define APDS_DISABLE_PS             0
#define APDS_ENABLE_PS_NO_INT       2
#define APDS_ENABLE_PS_WITH_INT     1

#define APDS_DISABLE_ALS            0
#define APDS_ENABLE_ALS_NO_INT      1
#define APDS_ENABLE_ALS_WITH_INT    2

#define APDS_ALS_POLL_SLOW          0 // 1 Hz (1s)
#define APDS_ALS_POLL_MEDIUM        1 // 10 Hz (100ms)
#define APDS_ALS_POLL_FAST          2 // 20 Hz (50ms)

/*
* Defines
*/

#define APDS9930_ENABLE_REG     0x00
#define APDS9930_ATIME_REG      0x01
#define APDS9930_PTIME_REG      0x02
#define APDS9930_WTIME_REG      0x03
#define APDS9930_AILTL_REG      0x04
#define APDS9930_AILTH_REG      0x05
#define APDS9930_AIHTL_REG      0x06
#define APDS9930_AIHTH_REG      0x07
#define APDS9930_PILTL_REG      0x08
#define APDS9930_PILTH_REG      0x09
#define APDS9930_PIHTL_REG      0x0A
#define APDS9930_PIHTH_REG      0x0B
#define APDS9930_PERS_REG       0x0C
#define APDS9930_CONFIG_REG     0x0D
#define APDS9930_PPCOUNT_REG    0x0E
#define APDS9930_CONTROL_REG    0x0F
#define APDS9930_REV_REG        0x11
#define APDS9930_ID_REG         0x12
#define APDS9930_STATUS_REG     0x13
#define APDS9930_CH0DATAL_REG   0x14
#define APDS9930_CH0DATAH_REG   0x15
#define APDS9930_CH1DATAL_REG   0x16
#define APDS9930_CH1DATAH_REG   0x17
#define APDS9930_PDATAL_REG     0x18
#define APDS9930_PDATAH_REG     0x19
#define APDS9930_POFFSET_REG    0x1E

#define CMD_BYTE                0x80
#define CMD_WORD                0xA0
#define CMD_SPECIAL             0xE0

#define CMD_CLR_PS_INT          0xE5
#define CMD_CLR_ALS_INT         0xE6
#define CMD_CLR_PS_ALS_INT      0xE7


/* Register Value define : ATIME */
#define APDS9930_100MS_ADC_TIME 0xDB  /* 100.64ms integration time */
#define APDS9930_50MS_ADC_TIME  0xED  /* 51.68ms integration time */
#define APDS9930_27MS_ADC_TIME  0xF6  /* 27.2ms integration time */

/* Register Value define : PRXCNFG */
#define APDS9930_ALS_REDUCE 0x04  /* ALSREDUCE - ALS Gain reduced by 4x */

/* Register Value define : PERS */
#define APDS9930_PPERS_0  0x00  /* Every proximity ADC cycle */
#define APDS9930_PPERS_1  0x10  /* 1 consecutive proximity value out of range */
#define APDS9930_PPERS_2  0x20  /* 2 consecutive proximity value out of range */
#define APDS9930_PPERS_3  0x30  /* 3 consecutive proximity value out of range */
#define APDS9930_PPERS_4  0x40  /* 4 consecutive proximity value out of range */
#define APDS9930_PPERS_5  0x50  /* 5 consecutive proximity value out of range */
#define APDS9930_PPERS_6  0x60  /* 6 consecutive proximity value out of range */
#define APDS9930_PPERS_7  0x70  /* 7 consecutive proximity value out of range */
#define APDS9930_PPERS_8  0x80  /* 8 consecutive proximity value out of range */
#define APDS9930_PPERS_9  0x90  /* 9 consecutive proximity value out of range */
#define APDS9930_PPERS_10 0xA0  /* 10 consecutive proximity value out of range */
#define APDS9930_PPERS_11 0xB0  /* 11 consecutive proximity value out of range */
#define APDS9930_PPERS_12 0xC0  /* 12 consecutive proximity value out of range */
#define APDS9930_PPERS_13 0xD0  /* 13 consecutive proximity value out of range */
#define APDS9930_PPERS_14 0xE0  /* 14 consecutive proximity value out of range */
#define APDS9930_PPERS_15 0xF0  /* 15 consecutive proximity value out of range */

#define APDS9930_APERS_0  0x00  /* Every ADC cycle */
#define APDS9930_APERS_1  0x01  /* 1 consecutive proximity value out of range */
#define APDS9930_APERS_2  0x02  /* 2 consecutive proximity value out of range */
#define APDS9930_APERS_3  0x03  /* 3 consecutive proximity value out of range */
#define APDS9930_APERS_5  0x04  /* 5 consecutive proximity value out of range */
#define APDS9930_APERS_10 0x05  /* 10 consecutive proximity value out of range */
#define APDS9930_APERS_15 0x06  /* 15 consecutive proximity value out of range */
#define APDS9930_APERS_20 0x07  /* 20 consecutive proximity value out of range */
#define APDS9930_APERS_25 0x08  /* 25 consecutive proximity value out of range */
#define APDS9930_APERS_30 0x09  /* 30 consecutive proximity value out of range */
#define APDS9930_APERS_35 0x0A  /* 35 consecutive proximity value out of range */
#define APDS9930_APERS_40 0x0B  /* 40 consecutive proximity value out of range */
#define APDS9930_APERS_45 0x0C  /* 45 consecutive proximity value out of range */
#define APDS9930_APERS_50 0x0D  /* 50 consecutive proximity value out of range */
#define APDS9930_APERS_55 0x0E  /* 55 consecutive proximity value out of range */
#define APDS9930_APERS_60 0x0F  /* 60 consecutive proximity value out of range */

/* Register Value define : CONTROL */
#define APDS9930_AGAIN_1X 0x00  /* 1X ALS GAIN */
#define APDS9930_AGAIN_8X 0x01  /* 8X ALS GAIN */
#define APDS9930_AGAIN_16X  0x02  /* 16X ALS GAIN */
#define APDS9930_AGAIN_120X 0x03  /* 120X ALS GAIN */

#define APDS9930_PRX_IR_DIOD  0x20  /* Proximity uses CH1 diode */

#define APDS9930_PGAIN_1X 0x00  /* PS GAIN 1X */
#define APDS9930_PGAIN_2X 0x04  /* PS GAIN 2X */
#define APDS9930_PGAIN_4X 0x08  /* PS GAIN 4X */
#define APDS9930_PGAIN_8X 0x0C  /* PS GAIN 8X */

#define APDS9930_PDRVIE_100MA 0x00  /* PS 100mA LED drive */
#define APDS9930_PDRVIE_50MA  0x40  /* PS 50mA LED drive */
#define APDS9930_PDRVIE_25MA  0x80  /* PS 25mA LED drive */
#define APDS9930_PDRVIE_12_5MA  0xC0  /* PS 12.5mA LED drive */

#define APDS9930_I2C_RETRY      10

#define APDS9930_PROX_AVERAGE_RAW_DATA_COUNT    10

typedef enum
{
    APDS9930_ALS_RES_27MS = 0,    /* 27.2ms integration time */
    APDS9930_ALS_RES_51MS = 1,    /* 51.68ms integration time */
    APDS9930_ALS_RES_100MS = 2     /* 100.64ms integration time */
} apds9930_als_res_e;

typedef enum
{
    APDS9930_ALS_GAIN_1X    = 0,    /* 1x AGAIN */
    APDS9930_ALS_GAIN_8X    = 1,    /* 8x AGAIN */
    APDS9930_ALS_GAIN_16X   = 2,    /* 16x AGAIN */
    APDS9930_ALS_GAIN_120X  = 3     /* 120x AGAIN */
} apds9930_als_gain_e;

/*
* Structs
*/

/* Pay attention to this structure, it must be the same as Selftest HAL */
struct PROX_SELFTEST_RESULT {
    bool i2c_status;
    bool interrupt_pin_support;
    bool interrupt_pin_status;
};

struct apds9930_nvitem_data {
    unsigned char sensor_device_id;
    unsigned char filler[1];
    unsigned char nv_ps_calibration_done;
    unsigned char nv_als_calibration_done;
    /* PS */
    unsigned short int nv_ps_threshold;
    unsigned short int nv_ps_offset_value;
    /* ALS */
    unsigned int nv_als_ga;
};

struct apds9930_data {
    struct i2c_client *client;
    struct mutex operation_lock;
    struct delayed_work dwork;    /* for PS & ALS interrupt */
    struct delayed_work ps_dwork; /* for PS polling */
    struct delayed_work als_dwork;  /* for ALS polling */
    struct input_dev *input_dev_als;
    struct input_dev *input_dev_ps;

    int irq;
    atomic_t suspended;
    unsigned int enable_suspended_value;  /* suspend_resume usage */

    unsigned int enable;
    unsigned int atime;
    unsigned int ptime;
    unsigned int wtime;
    unsigned int ailt;
    unsigned int aiht;
    unsigned int pilt;
    unsigned int piht;
    unsigned int pers;
    unsigned int config;
    unsigned int ppcount;
    unsigned int control;

    /* control flag from HAL */
    unsigned int enable_ps_sensor;
    unsigned int enable_als_sensor;

    /* PS parameters */
    unsigned int ps_threshold;
    unsigned int ps_hysteresis_threshold;   /* always lower than ps_threshold */
    unsigned int ps_poll_delay; /* needed for proximity sensor polling : micro-sencond (us) */
    unsigned int ps_pulse_number;   /* pulse number */
    unsigned int ps_pdrive_ma;  /* drive current */
    unsigned int ps_pgain;  /* proximity gain */
    unsigned int ps_poffset;    /* proximity offset */
    unsigned int ps_detection;    /* 0 = near-to-far; 1 = far-to-near */
    unsigned int ps_data;     /* to store PS data */

    /* ALS parameters */
    unsigned int als_threshold_l; /* low threshold */
    unsigned int als_threshold_h; /* high threshold */
    unsigned int als_data;    /* to store ALS data */
    int als_prev_lux;   /* to store previous lux value */

    unsigned int als_gain;    /* needed for Lux calculation */
    unsigned int als_poll_delay;  /* needed for light sensor polling : micro-second (us) */
    unsigned int als_atime_index; /* storage for als integratiion time */
    unsigned int als_again_index; /* storage for als GAIN */
    unsigned int als_reduce;  /* flag indicate ALS 6x reduction */
    bool als_use_irq_in_idle;   /* whether use interrupt mode in idle */

    unsigned int als_threshold_hsytersis;
    unsigned int als_ga;
    unsigned int als_coe_b;
    unsigned int als_coe_c;
    unsigned int als_coe_d;
    unsigned int als_df;

    /* For selftest */
    atomic_t ps_selftest_ongoing;
    atomic_t ps_selftest_int;
    wait_queue_head_t ps_selftest_wq;
};

/*
* Global data
*/
static struct i2c_client *apds9930_i2c_client; /* global i2c_client to support ioctl */
static struct workqueue_struct *apds_workqueue;

static unsigned char apds9930_als_atime_tb[] = { 0xF6, 0xED, 0xDB };
static unsigned short apds9930_als_integration_tb[] = {2720, 5168, 10064};
static unsigned short apds9930_als_res_tb[] = { 10240, 19456, 37888 };
static unsigned char apds9930_als_again_tb[] = { 1, 8, 16, 120 };
static unsigned char apds9930_als_again_bit_tb[] = { 0x00, 0x01, 0x02, 0x03 };

/*
* Management functions
*/

static void apds9930_wait_for_device_resume(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);

    do {
        if (atomic_read(&data->suspended) == 0)
            break;
        else
        {
            KMSGINF("%s, suspended state is %d\n", __func__, atomic_read(&data->suspended));
            msleep(10);
        }
    } while(1);
}

static s32 apds9930_i2c_smbus_write_byte(struct i2c_client *client, u8 value)
{
    u8 index;
    s32 ret;

    for (index = 0; index < APDS9930_I2C_RETRY; index++)
    {
        ret = i2c_smbus_write_byte(client, value);
        if ( ret >= 0)
            break;
        else {
            KMSGINF("%s: i2c_smbus_write_byte, value = 0x%x, ret = %d, index = %u\n", __func__, value, ret, index);
            msleep(10);
        }
    }
    return ret;
}

static s32 apds9930_i2c_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value)
{
    u8 index;
    s32 ret;

    for (index = 0; index < APDS9930_I2C_RETRY; index++)
    {
        ret = i2c_smbus_write_byte_data(client, command, value);
        if ( ret >= 0)
            break;
        else {
            KMSGINF("%s: i2c_smbus_write_byte_data, command = 0x%x, value = 0x%x, ret = %d, index = %u\n", __func__, command, value, ret, index);
            msleep(10);
        }
    }
    return ret;
}

static s32 apds9930_i2c_smbus_write_word_data(struct i2c_client *client, u8 command, u16 value)
{
    u8 index;
    s32 ret;

    for (index = 0; index < APDS9930_I2C_RETRY; index++)
    {
        ret = i2c_smbus_write_word_data(client, command, value);
        if ( ret >= 0)
            break;
        else {
            KMSGINF("%s: i2c_smbus_write_word_data, command = 0x%x, value = 0x%x, ret = %d, index = %u\n", __func__, command, value, ret, index);
            msleep(10);
        }
    }
    return ret;
}

static s32 apds9930_i2c_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
    u8 index;
    s32 ret;

    for (index = 0; index < APDS9930_I2C_RETRY; index++)
    {
        ret = i2c_smbus_read_byte_data(client, command);
        if ( ret >= 0)
            break;
        else {
            KMSGINF("%s: i2c_smbus_read_byte_data, command = 0x%x, ret = %d, index = %u\n", __func__, command, ret, index);
            msleep(10);
        }
    }
    return ret;
}

static s32 apds9930_i2c_smbus_read_word_data(struct i2c_client *client, u8 command)
{
    u8 index;
    s32 ret;

    for (index = 0; index < APDS9930_I2C_RETRY; index++)
    {
        ret = i2c_smbus_read_word_data(client, command);
        if ( ret >= 0)
            break;
        else {
            KMSGINF("%s: i2c_smbus_read_word_data, command = 0x%x, ret = %d, index = %u\n", __func__, command, ret, index);
            msleep(10);
        }
    }
    return ret;
}

static int apds9930_set_command(struct i2c_client *client, int command)
{
    int ret;
    int clearInt;

    if (command == 0)
        clearInt = CMD_CLR_PS_INT;
    else if (command == 1)
        clearInt = CMD_CLR_ALS_INT;
    else
        clearInt = CMD_CLR_PS_ALS_INT;

    ret = apds9930_i2c_smbus_write_byte(client, clearInt);

    return ret;
}

static int apds9930_set_enable(struct i2c_client *client, int enable)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_ENABLE_REG, enable);

    data->enable = enable;

    return ret;
}

static int apds9930_set_atime(struct i2c_client *client, int atime)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_ATIME_REG, atime);

    data->atime = atime;

    return ret;
}

static int apds9930_set_ptime(struct i2c_client *client, int ptime)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_PTIME_REG, ptime);

    data->ptime = ptime;

    return ret;
}

static int apds9930_set_wtime(struct i2c_client *client, int wtime)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_WTIME_REG, wtime);

    data->wtime = wtime;

    return ret;
}

static int apds9930_set_ailt(struct i2c_client *client, int threshold)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_AILTL_REG, threshold);

    data->ailt = threshold;

    return ret;
}

static int apds9930_set_aiht(struct i2c_client *client, int threshold)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_AIHTL_REG, threshold);

    data->aiht = threshold;

    return ret;
}

static int apds9930_set_pilt(struct i2c_client *client, int threshold)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PILTL_REG, threshold);

    data->pilt = threshold;

    return ret;
}

static int apds9930_set_piht(struct i2c_client *client, int threshold)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PIHTL_REG, threshold);

    data->piht = threshold;

    return ret;
}

static int apds9930_set_pers(struct i2c_client *client, int pers)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_PERS_REG, pers);

    data->pers = pers;

    return ret;
}

static int apds9930_set_config(struct i2c_client *client, int config)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONFIG_REG, config);

    data->config = config;

    return ret;
}

static int apds9930_set_ppcount(struct i2c_client *client, int ppcount)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_PPCOUNT_REG, ppcount);

    data->ppcount = ppcount;

    return ret;
}

static int apds9930_set_control(struct i2c_client *client, int control)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;

    ret = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONTROL_REG, control);

    data->control = control;

    return ret;
}

static int LuxCalculation(struct i2c_client *client, int ch0data, int ch1data)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int luxValue=0;

    int IAC1=0;
    int IAC2=0;
    int IAC=0;

    IAC1 = (ch0data - (data->als_coe_b*ch1data)/100);      // re-adjust COE_B to avoid 2 decimal point
    IAC2 = ((data->als_coe_c*ch0data)/100 - (data->als_coe_d*ch1data)/100);   // re-adjust COE_C and COE_D to void 2 decimal point

    if (IAC1 > IAC2)
        IAC = IAC1;
    else if (IAC1 <= IAC2)
        IAC = IAC2;
    else
        IAC = 0;

    if (IAC1<0 && IAC2<0) {
        IAC = 0;  // cdata and irdata saturated
        return -1;  // don't report first, change gain may help
    }

    if (data->als_reduce) {
        luxValue = ((IAC*data->als_ga*data->als_df)/100)*4/((apds9930_als_integration_tb[data->als_atime_index]/100)*apds9930_als_again_tb[data->als_again_index]);
    }
    else {
        luxValue = ((IAC*data->als_ga*data->als_df)/100)/((apds9930_als_integration_tb[data->als_atime_index]/100)*apds9930_als_again_tb[data->als_again_index]);
    }

    return luxValue;
}

static void apds9930_change_ps_threshold(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);

    data->ps_data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_PDATAL_REG);
    KMSGINF("%s: proximity_data = %d\n", __func__, data->ps_data);

    if ( (data->ps_data > data->pilt) && (data->ps_data >= data->piht) ) {
        /* far-to-near detected */
        data->ps_detection = 1;

        input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);/* FAR-to-NEAR detection */
        input_sync(data->input_dev_ps);

        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PILTL_REG, data->ps_hysteresis_threshold);
        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PIHTL_REG, 1024);

        data->pilt = data->ps_hysteresis_threshold;
        data->piht = 1024;

        KMSGINF("far-to-near detected\n");
    }
    else if ( (data->ps_data <= data->pilt) && (data->ps_data < data->piht) ) {
        /* near-to-far detected */
        data->ps_detection = 0;

        input_report_abs(data->input_dev_ps, ABS_DISTANCE, 10);/* NEAR-to-FAR detection */
        input_sync(data->input_dev_ps);

        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PILTL_REG, 0);
        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PIHTL_REG, data->ps_threshold);

        data->pilt = 0;
        data->piht = data->ps_threshold;

        KMSGINF("near-to-far detected\n");
    }
    else if ( (data->pilt == 1024) && (data->piht == 0) ) { // force interrupt
        /* special case */
        /* near-to-far detected */
        data->ps_detection = 0;

        input_report_abs(data->input_dev_ps, ABS_DISTANCE, 10);/* NEAR-to-FAR detection */
        input_sync(data->input_dev_ps);

        apds9930_set_pilt(client, 0);   // init threshold for proximity
        apds9930_set_piht(client, data->ps_threshold);

        KMSGINF("near-to-far detected\n");
    }
}

static void apds9930_change_als_threshold(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ch0data, ch1data;
    int luxValue=0;
    int err;
    unsigned char change_again=0;
    unsigned char control_data=0;
    unsigned char lux_is_valid=1;

    ch0data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);
    ch1data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH1DATAL_REG);

    luxValue = LuxCalculation(client, ch0data, ch1data);

    if (luxValue >= 0) {
        luxValue = luxValue<30000 ? luxValue : 30000;
        data->als_prev_lux = luxValue;
    }
    else {
        lux_is_valid = 0; // don't report, the lux is invalid value
        luxValue = data->als_prev_lux;
        if (data->als_reduce) {
            lux_is_valid = 1;
            luxValue = 30000; // report anyway since this is the lowest gain
        }
    }

    KMSGINF("lux=%d ch0data=%d ch1data=%d again=%d als_reduce=%d\n", luxValue, ch0data, ch1data, apds9930_als_again_tb[data->als_again_index], data->als_reduce);

    // check PS under sunlight
    if ( (data->ps_detection == 1) && (ch0data > (75*(1024*(256-apds9930_als_atime_tb[data->als_atime_index])))/100)) // PS was previously in far-to-near condition
    {
        // need to inform input event as there will be no interrupt from the PS
        input_report_abs(data->input_dev_ps, ABS_DISTANCE, 10);/* NEAR-to-FAR detection */
        input_sync(data->input_dev_ps);

        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PILTL_REG, 0);
        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PIHTL_REG, data->ps_threshold);

        data->pilt = 0;
        data->piht = data->ps_threshold;

        data->ps_detection = 0; /* near-to-far detected */

        KMSGINF("apds_993x_proximity_handler = FAR\n");
    }

    if (lux_is_valid) {
        input_report_rel(data->input_dev_als, REL_LIGHT, luxValue); // report the lux level
        input_sync(data->input_dev_als);
    }

    data->als_data = ch0data;

    data->als_threshold_l = (data->als_data * (100-APDS9930_ALS_THRESHOLD_HSYTERESIS) ) /100;
    data->als_threshold_h = (data->als_data * (100+APDS9930_ALS_THRESHOLD_HSYTERESIS) ) /100;

    if (data->als_threshold_h >= apds9930_als_res_tb[data->als_atime_index]) {
        data->als_threshold_h = apds9930_als_res_tb[data->als_atime_index];
    }

    if (data->als_data >= (apds9930_als_res_tb[data->als_atime_index]*90)/100) {
        // lower AGAIN if possible
        if (data->als_again_index != APDS9930_ALS_GAIN_1X) {
            data->als_again_index--;
            change_again = 1;
        }
        else {
            err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONFIG_REG, APDS9930_ALS_REDUCE);
            if (err >= 0) {
                data->als_reduce = 1;
            }
        }
    }
    else if (data->als_data <= (apds9930_als_res_tb[data->als_atime_index]*10)/100) {
        // increase AGAIN if possible
        if (data->als_reduce) {
            err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONFIG_REG, 0);
            if (err >= 0) {
                data->als_reduce = 0;
            }
        }
        else if (data->als_again_index != APDS9930_ALS_GAIN_120X) {
            data->als_again_index++;
            change_again = 1;
        }
    }

    if (change_again) {
        control_data = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_CONTROL_REG);
        control_data = control_data & 0xFC;
        control_data = control_data | apds9930_als_again_bit_tb[data->als_again_index];
        apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONTROL_REG, control_data);
    }

    apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_AILTL_REG, data->als_threshold_l);
    apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_AIHTL_REG, data->als_threshold_h);

}

static void apds9930_reschedule_work(struct apds9930_data *data,
    unsigned long delay)
{
    if (unlikely(atomic_read(&data->ps_selftest_ongoing) == 1))
    {
        atomic_set(&data->ps_selftest_int, 1);
        wake_up_interruptible(&data->ps_selftest_wq);
        return;
    }
    /*
    * If work is already scheduled then subsequent schedules will not
    * change the scheduled time that's why we have to cancel it first.
    */
    __cancel_delayed_work(&data->dwork);
    queue_delayed_work(apds_workqueue, &data->dwork, delay);
}

/* ALS polling routine */
static void apds9930_als_polling_work_handler(struct work_struct *work)
{
    struct apds9930_data *data = container_of(work, struct apds9930_data, als_dwork.work);
    struct i2c_client *client=data->client;
    int ch0data, ch1data, pdata;
    int luxValue=0;
    int err;
    unsigned char change_again=0;
    unsigned char control_data=0;
    unsigned char lux_is_valid=1;

    KMSGINF("%s enter\n", __func__);

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    if (data->enable_als_sensor != APDS_ENABLE_ALS_NO_INT)
    {
        KMSGINF("%s: enable_als_sensor = %d, return directly\n", __func__, data->enable_als_sensor);
        mutex_unlock(&data->operation_lock);
        return;
    }

    ch0data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);
    ch1data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH1DATAL_REG);
    pdata = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_PDATAL_REG);
    KMSGINF("ch0data=%d ch1data=%d pdata=%d\n", ch0data, ch1data, pdata);

    luxValue = LuxCalculation(client, ch0data, ch1data);
    KMSGINF("luxValue=%d\n", luxValue);

    if (luxValue >= 0) {
        luxValue = luxValue<30000 ? luxValue : 30000;
        data->als_prev_lux = luxValue;
    }
    else {
        lux_is_valid = 0; // don't report, this is invalid lux value
        luxValue = data->als_prev_lux;
        if (data->als_reduce) {
            KMSGINF("als_reduce=%d\n", data->als_reduce);
            lux_is_valid = 1;
            luxValue = 30000; // report anyway since this is the lowest gain
        }
    }

    KMSGINF("lux_is_valid=%d lux=%d ch0data=%d ch1data=%d pdata=%d delay=%d again=%d als_reduce=%d)\n", lux_is_valid, luxValue, ch0data, ch1data, pdata, data->als_poll_delay, apds9930_als_again_tb[data->als_again_index], data->als_reduce);

    // check PS under sunlight
    if ( (data->ps_detection == 1) && (ch0data > (75*(1024*(256-data->atime)))/100) )  // PS was previously in far-to-near condition
    {
        // need to inform input event as there will be no interrupt from the PS
        input_report_abs(data->input_dev_ps, ABS_DISTANCE, 10);/* NEAR-to-FAR detection */
        input_sync(data->input_dev_ps);

        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PILTL_REG, 0);
        apds9930_i2c_smbus_write_word_data(client, CMD_WORD|APDS9930_PIHTL_REG, data->ps_threshold);

        data->pilt = 0;
        data->piht = data->ps_threshold;

        data->ps_detection = 0; /* near-to-far detected */

        KMSGINF("apds_993x_proximity_handler = FAR\n");
    }

    if (lux_is_valid) {
        input_report_rel(data->input_dev_als, REL_LIGHT, luxValue); // report the lux level
        input_sync(data->input_dev_als);
        KMSGINF("%s: report the lux level\n", __func__);
    }

    data->als_data = ch0data;

    if (data->als_data >= (apds9930_als_res_tb[data->als_atime_index]*90)/100 ) {
        // lower AGAIN if possible
        if (data->als_again_index != APDS9930_ALS_GAIN_1X) {
            data->als_again_index--;
            change_again = 1;
        }
        else {
            err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONFIG_REG, APDS9930_ALS_REDUCE);
            if (err >= 0) {
                data->als_reduce = 1;
            }
        }
    }
    else if (data->als_data <= (apds9930_als_res_tb[data->als_atime_index]*10)/100) {
        // increase AGAIN if possible
        if (data->als_reduce) {
            err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONFIG_REG, 0);
            if (err >= 0) {
                data->als_reduce = 0;
            }
        }
        else if (data->als_again_index != APDS9930_ALS_GAIN_120X) {
            data->als_again_index++;
            change_again = 1;
        }
    }

    if (change_again) {
        control_data = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_CONTROL_REG);
        control_data = control_data & 0xFC;
        control_data = control_data | apds9930_als_again_bit_tb[data->als_again_index];
        apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONTROL_REG, control_data);
    }

    mutex_unlock(&data->operation_lock);

    queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay)); // restart timer
}

/* PS polling routine */
static void apds9930_ps_polling_work_handler(struct work_struct *work)
{
    struct apds9930_data *data = container_of(work, struct apds9930_data, ps_dwork.work);
    struct i2c_client *client=data->client;
    int ch0data;

    KMSGINF("%s enter\n", __func__);

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    if (data->enable_ps_sensor != APDS_ENABLE_PS_NO_INT)
    {
        KMSGINF("%s: enable_ps_sensor = %d, return directly\n", __func__, data->enable_ps_sensor);
        mutex_unlock(&data->operation_lock);
        return;
    }

    ch0data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);
    KMSGINF("%s: ch0data = %d, als_res_tb = %d, ps_detection = %d\n", __func__, ch0data, apds9930_als_res_tb[data->als_atime_index], data->ps_detection);

    if (ch0data < (75*(apds9930_als_res_tb[data->als_atime_index]))/100)
        apds9930_change_ps_threshold(client);
    else {
        if (data->ps_detection == 1) {
            apds9930_change_ps_threshold(client);
        }
        else {
            KMSGINF("Triggered by background ambient noise\n");
        }
    }

    mutex_unlock(&data->operation_lock);

    queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay)); // restart timer
}

/* PS interrupt routine */
static void apds9930_ps_als_int_work_handler(struct work_struct *work)
{
    struct apds9930_data *data = container_of(work, struct apds9930_data, dwork.work);
    struct i2c_client *client=data->client;
    int status;
    int ch0data;
    int enable;

    KMSGINF("%s enter\n", __func__);

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    status = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_STATUS_REG);
    enable = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_ENABLE_REG);

    apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_ENABLE_REG, 1); /* disable 993x's ADC first */

    KMSGINF("%s: status = %x\n", __func__, status);

    if ((status & enable & 0x30) == 0x30) {
        /* both PS and ALS are interrupted */
        apds9930_change_als_threshold(client);

        KMSGINF("%s: Both PS and ALS are interrupted\n", __func__);

        ch0data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);
        if (ch0data < (75*(1024*(256-data->atime)))/100)
            apds9930_change_ps_threshold(client);
        else {
            if (data->ps_detection == 1) {
                apds9930_change_ps_threshold(client);
            }
            else {
                KMSGINF("Triggered by background ambient noise\n");
            }
        }

        apds9930_set_command(client, 2);  /* 2 = CMD_CLR_PS_ALS_INT */
    }
    else if ((status & enable & 0x20) == 0x20) {
        /* only PS is interrupted */

        KMSGINF("%s: Only PS is interrupted\n", __func__);
        /* check if this is triggered by background ambient noise */
        ch0data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);
        if (ch0data < (75*(apds9930_als_res_tb[data->als_atime_index]))/100)
            apds9930_change_ps_threshold(client);
        else {
            if (data->ps_detection == 1) {
                apds9930_change_ps_threshold(client);
            }
            else {
                KMSGINF("Triggered by background ambient noise\n");
            }
        }


        apds9930_set_command(client, 0);  /* 0 = CMD_CLR_PS_INT */
    }
    else if ((status & enable & 0x10) == 0x10) {
        /* only ALS is interrupted */
        KMSGINF("%s: Only ALS is interrupted\n", __func__);

        apds9930_change_als_threshold(client);

        apds9930_set_command(client, 1);  /* 1 = CMD_CLR_ALS_INT */
    }

    apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_ENABLE_REG, data->enable);

    mutex_unlock(&data->operation_lock);

    enable_irq(client->irq);
}

/* assume this is ISR */
static irqreturn_t apds9930_interrupt(int vec, void *info)
{
    struct i2c_client *client=(struct i2c_client *)info;
    struct apds9930_data *data = i2c_get_clientdata(client);

    disable_irq_nosync(client->irq);

    KMSGINF("==> apds9930_interrupt\n");
    apds9930_reschedule_work(data, 0);

    return IRQ_HANDLED;
}

/*
* IOCTL support
*/

static int apds9930_read_ps_als_nvitem(struct apds9930_nvitem_data *apds9930_nvitem)
{
    int ret = 0;
    mm_segment_t old_fs;
    loff_t sz_file;
    unsigned long magic;

    struct inode *inode;
    struct file *file;

    file = filp_open( APDS9930_NV_BIN_FILE_NAME, O_RDONLY, 0 );
    if (IS_ERR(file)){
        KMSGERR("%s: open file error...\n", __func__);
        ret = -1;
        goto exit;
    }
    KMSGINF("%s: open file success\n", __func__);
    inode=file->f_dentry->d_inode;
    magic=inode->i_sb->s_magic;
    sz_file=inode->i_size;
    KMSGINF("%s,nvitem file size = %d\n", __func__, (int)sz_file);

    if (sz_file < sizeof(*apds9930_nvitem)) {
        /* skip when file is invaild*/
        KMSGERR("%s: file size error...\n", __func__);
        ret = -1;
        goto exit_close_file;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if (file->f_op->llseek(file, 0, SEEK_SET) != 0) {
        KMSGERR("%s: lseek failure...\n", __func__);
        ret = -1;
        goto exit_restore_fs;
    }

    ret = file->f_op->read(file, (char *)apds9930_nvitem, sizeof(*apds9930_nvitem), &file->f_pos);
    if (ret <= 0){
        goto exit_restore_fs;
    }

    if (apds9930_nvitem->sensor_device_id != APDS9930_DEVICE_ID) {
        KMSGERR("%s: Error nvitem file...\n", __func__);
        ret = -1;
        goto exit_restore_fs;
    }

exit_restore_fs:
    /* resume user space */
    set_fs(old_fs);
exit_close_file:
    /* close file */
    filp_close(file,NULL);
exit:
    return ret;
}

static void apds9930_read_ps_nvitem(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    struct apds9930_nvitem_data *apds9930_nvitem = (struct apds9930_nvitem_data *)kzalloc(sizeof(*apds9930_nvitem)+1, GFP_KERNEL);
    int ret = 0;

    if (apds9930_nvitem != NULL)
        ret = apds9930_read_ps_als_nvitem(apds9930_nvitem);
    else
    {
        KMSGERR("%s: Memory allocation fails\n", __func__);
    }
    if (ret > 0)
    {
        if ( apds9930_nvitem->nv_ps_calibration_done == 1 )
        {
            unsigned int diff = data->ps_threshold - data->ps_hysteresis_threshold;
            data->ps_threshold = apds9930_nvitem->nv_ps_threshold;
            if ( diff < data->ps_threshold )
                data->ps_hysteresis_threshold = data->ps_threshold - diff;
            else
                data->ps_hysteresis_threshold = 0;
            KMSGINF("%s: Update the ps_threshold: %d, ps_hysteresis_threshold: %d\n", __func__, data->ps_threshold, data->ps_hysteresis_threshold);
        }
    }
    if (apds9930_nvitem != NULL)
    {
        kfree(apds9930_nvitem);
    }
}

static void apds9930_read_als_nvitem(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    struct apds9930_nvitem_data *apds9930_nvitem = (struct apds9930_nvitem_data *)kzalloc(sizeof(*apds9930_nvitem)+1, GFP_KERNEL);
    int ret = 0;

    if (apds9930_nvitem != NULL)
        ret = apds9930_read_ps_als_nvitem(apds9930_nvitem);
    else
    {
        KMSGERR("%s: Memory allocation fails\n", __func__);
    }
    if (ret > 0)
    {
        if ( apds9930_nvitem->nv_als_calibration_done == 1 )
        {
            data->als_ga = apds9930_nvitem->nv_als_ga;
            KMSGINF("%s: Update the als_ga: %d\n", __func__, data->als_ga);
        }
    }
    if (apds9930_nvitem != NULL)
    {
        kfree(apds9930_nvitem);
    }
}

static int apds9930_enable_als_sensor(struct i2c_client *client, int val)
{
    struct apds9930_data *data = i2c_get_clientdata(client);

    KMSGINF("%s: enable als sensor ( %d )\n", __func__, val);

    if ((val != APDS_DISABLE_ALS) && (val != APDS_ENABLE_ALS_WITH_INT) && (val != APDS_ENABLE_ALS_NO_INT)) {
        KMSGERR("%s: enable als sensor=%d\n", __func__, val);
        return -1;
    }

    if ((val == APDS_ENABLE_ALS_WITH_INT) || (val == APDS_ENABLE_ALS_NO_INT)) {
        // turn on light  sensor
        if (data->enable_als_sensor==APDS_DISABLE_ALS) {

            data->enable_als_sensor = val;

            apds9930_set_enable(client,0); /* Power Off */

            apds9930_read_als_nvitem(client);

            if (data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT) {
                KMSGINF("%s: Start to enable ALS without interrupt\n", __func__);
                if (data->enable_ps_sensor == APDS_ENABLE_PS_WITH_INT) {
                    KMSGINF("%s: enable PS with interrupt and ALS without interrupt\n", __func__);
                    apds9930_set_enable(client, 0x2F);   /* Enable PS with interrupt */
                }
                else if(data->enable_ps_sensor == APDS_ENABLE_PS_NO_INT) {
                    KMSGINF("%s: enable PS without interrupt and ALS without interrupt\n", __func__);
                    apds9930_set_enable(client, 0x0F);   /* Enable PS and ALS without interrupt */
                }
                else {
                    KMSGINF("%s: enable ALS without interrupt\n", __func__);
                    apds9930_set_enable(client, 0x0B);   /* no interrupt*/
                }

                /*
                * If work is already scheduled then subsequent schedules will not
                * change the scheduled time that's why we have to cancel it first.
                */
                __cancel_delayed_work(&data->als_dwork);
                KMSGINF("%s: Start the apds9930_als_polling_work_handler, the als_poll_delay is %d\n", __func__, data->als_poll_delay);
                queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));

            }
            else {  // als with int
                KMSGINF("%s: Start to enable ALS with interrupt\n", __func__);
                apds9930_set_ailt( client, 0xFFFF); // force first ALS interrupt in order to get environment reading
                apds9930_set_aiht( client, 0);

                if (data->enable_ps_sensor == APDS_ENABLE_PS_WITH_INT) {
                    KMSGINF("%s: enable PS with interrupt and ALS with interrupt\n", __func__);
                    apds9930_set_enable(client, 0x3F);   /* Enable both ALS and PS with interrupt */
                }
                else if (data->enable_ps_sensor == APDS_ENABLE_PS_NO_INT) {
                    KMSGINF("%s: enable PS without interrupt and ALS with interrupt\n", __func__);
                    apds9930_set_enable(client, 0x1F);   /* Enable both ALS and PS with interrupt */
                }
                else {
                    KMSGINF("%s: enable ALS with interrupt\n", __func__);
                    apds9930_set_enable(client, 0x1B);   /* only enable light sensor with interrupt*/
                }

                /*
                * If work is already scheduled then subsequent schedules will not
                * change the scheduled time that's why we have to cancel it first.
                */
                __cancel_delayed_work(&data->als_dwork);
            }
        }
    }
    else {
        KMSGINF("%s: Start to disable ALS\n", __func__);
        //turn off light sensor
        // what if the p sensor is active?
        data->enable_als_sensor = APDS_DISABLE_ALS;

        if (data->enable_ps_sensor == APDS_ENABLE_PS_WITH_INT) {
            apds9930_set_enable(client,0); /* Power Off */

            apds9930_set_pilt(client, 0);
            apds9930_set_piht(client, data->ps_threshold);

            KMSGINF("%s: enable PS with interrupt, disable ALS\n", __func__);
            apds9930_set_enable(client, 0x2D);   /* only enable prox sensor with interrupt */
        }
        else if (data->enable_ps_sensor == APDS_ENABLE_PS_NO_INT) {

            KMSGINF("%s: enable PS without interrupt, disable ALS\n", __func__);
            apds9930_set_enable(client, 0x0D);   /* only enable prox sensor without interrupt */
            /*
            * If work is already scheduled then subsequent schedules will not
            * change the scheduled time that's why we have to cancel it first.
            */
            __cancel_delayed_work(&data->ps_dwork);
            queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));
        }
        else {
            KMSGINF("%s: disable PS and ALS\n", __func__);
            apds9930_set_enable(client, 0);
            /*
            * If work is already scheduled then subsequent schedules will not
            * change the scheduled time that's why we have to cancel it first.
            */
            __cancel_delayed_work(&data->ps_dwork);
        }

        /*
        * If work is already scheduled then subsequent schedules will not
        * change the scheduled time that's why we have to cancel it first.
        */
        __cancel_delayed_work(&data->als_dwork);

    }

    return 0;
}

static int apds9930_set_als_poll_delay(struct i2c_client *client, unsigned int val)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ret;
    int atime_index=0;

    KMSGINF("%s : %d\n", __func__, val);

    if ((val != APDS_ALS_POLL_SLOW) && (val != APDS_ALS_POLL_MEDIUM) && (val != APDS_ALS_POLL_FAST)) {
        KMSGERR("%s:invalid value=%d\n", __func__, val);
        return -1;
    }

    if (val == APDS_ALS_POLL_FAST) {
        data->als_poll_delay = 50;    // 50ms
        atime_index = APDS9930_ALS_RES_27MS;
    }
    else if (val == APDS_ALS_POLL_MEDIUM) {
        data->als_poll_delay = 100;   // 100ms
        atime_index = APDS9930_ALS_RES_51MS;
    }
    else {  // APDS_ALS_POLL_SLOW
        data->als_poll_delay = 1000;    // 1000ms
        atime_index = APDS9930_ALS_RES_100MS;
    }

    ret = apds9930_set_atime(client, apds9930_als_atime_tb[atime_index]);
    if (ret >= 0) {
        data->als_atime_index = atime_index;
        KMSGINF("poll delay %d, atime_index %d\n", data->als_poll_delay, data->als_atime_index);
    }
    else
        return -1;

    /*
    * If work is already scheduled then subsequent schedules will not
    * change the scheduled time that's why we have to cancel it first.
    */
    __cancel_delayed_work(&data->als_dwork);
    queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));

    return 0;
}

static int apds9930_enable_ps_sensor(struct i2c_client *client, int val)
{
    struct apds9930_data *data = i2c_get_clientdata(client);

    KMSGINF("%s: enable ps sensor ( %d )\n", __func__, val);

    if ((val != APDS_DISABLE_PS) && (val != APDS_ENABLE_PS_NO_INT) && (val != APDS_ENABLE_PS_WITH_INT)) {
        KMSGERR("%s:invalid value=%d\n", __func__, val);
        return -1;
    }

    if((val == APDS_ENABLE_PS_WITH_INT) || (val == APDS_ENABLE_PS_NO_INT)) {
        //turn on p sensor
        if (data->enable_ps_sensor == APDS_DISABLE_PS) {

            data->enable_ps_sensor = val;

            apds9930_set_enable(client,0); /* Power Off */

            apds9930_read_ps_nvitem(client);

            if (data->enable_ps_sensor == APDS_ENABLE_PS_NO_INT) {
                KMSGINF("%s: Start to enable PS without interrupt\n", __func__);

                if (data->enable_als_sensor == APDS_DISABLE_ALS) {
                    KMSGINF("%s: Start to enable PS without interrupt and disable ALS\n", __func__);
                    apds9930_set_enable(client, 0x0D);   /* only enable PS without interrupt */
                }
                else if (data->enable_als_sensor == APDS_ENABLE_ALS_WITH_INT) {
                    KMSGINF("%s: Start to enable PS without interrupt and ALS with interrupt\n", __func__);
                    apds9930_set_enable(client, 0x1F);   /* enable ALS with interrupt and PS without interrupt */
                }
                else { // APDS_ENABLE_ALS_NO_INT
                    KMSGINF("%s: Start to enable PS without interrupt and ALS without interrupt\n", __func__);
                    apds9930_set_enable(client, 0x0F);   /* enable PS and ALS without interrupt */
                }
                /*
                * If work is already scheduled then subsequent schedules will not
                * change the scheduled time that's why we have to cancel it first.
                */
                __cancel_delayed_work(&data->ps_dwork);
                KMSGINF("%s: Start the apds9930_ps_polling_work_handler, the ps_poll_delay is %d\n", __func__, data->ps_poll_delay);
                queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));
            }
            else {  // ps with int
                KMSGINF("%s: Start to enable PS with interrupt\n", __func__);

                // force first interrupt to inform HAL
                apds9930_set_pilt(client, 1024);
                apds9930_set_piht(client, 0);

                if (data->enable_als_sensor == APDS_DISABLE_ALS) {
                    KMSGINF("%s: Start to enable PS with interrupt and disable ALS\n", __func__);
                    apds9930_set_enable(client, 0x2D);   /* only enable PS interrupt */
                }
                else if (data->enable_als_sensor == APDS_ENABLE_ALS_WITH_INT) {
                    KMSGINF("%s: Start to enable PS with interrupt and ALS with interrupt\n", __func__);
                    apds9930_set_enable(client, 0x3F);   /* enable ALS and PS interrupt */
                }
                else { // APDS_ENABLE_ALS_NO_INT
                    KMSGINF("%s: Start to enable PS with interrupt and ALS without interrupt\n", __func__);
                    apds9930_set_enable(client, 0x2F);   /* enable PS interrupt */
                }
                /*
                * If work is already scheduled then subsequent schedules will not
                * change the scheduled time that's why we have to cancel it first.
                */
                __cancel_delayed_work(&data->ps_dwork);
            }
        }
    }
    else {
        //turn off p sensor - kk 25 Apr 2011 we can't turn off the entire sensor, the light sensor may be needed by HAL
        data->enable_ps_sensor = APDS_DISABLE_PS;
        KMSGINF("%s: Start to disable PS\n", __func__);
        if (data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT) {
            KMSGINF("%s: Start to disable PS and enable ALS without interrupt\n", __func__);
            apds9930_set_enable(client, 0x0B);   /* no ALS interrupt */

            /*
            * If work is already scheduled then subsequent schedules will not
            * change the scheduled time that's why we have to cancel it first.
            */
            __cancel_delayed_work(&data->als_dwork);
            queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay)); // 100ms
        }
        else if (data->enable_als_sensor == APDS_ENABLE_ALS_WITH_INT) {
            KMSGINF("%s: Start to disable PS and enable ALS with interrupt\n", __func__);
            // reconfigute light sensor setting
            apds9930_set_enable(client,0); /* Power Off */
            apds9930_set_ailt( client, 0xFFFF); // Force ALS interrupt
            apds9930_set_aiht( client, 0);

            apds9930_set_enable(client, 0x13);   /* enable ALS interrupt */
        }
        else {  // APDS_DISABLE_ALS
            KMSGINF("%s: Start to disable PS and ALS\n", __func__);
            apds9930_set_enable(client, 0);

            /*
            * If work is already scheduled then subsequent schedules will not
            * change the scheduled time that's why we have to cancel it first.
            */
            __cancel_delayed_work(&data->als_dwork);
        }
        /*
        * If work is already scheduled then subsequent schedules will not
        * change the scheduled time that's why we have to cancel it first.
        */
        __cancel_delayed_work(&data->ps_dwork);
    }

    return 0;
}

static int apds9930_ps_open(struct inode *inode, struct file *file)
{
    KMSGINF("apds9930_ps_open\n");
    return 0;
}

static int apds9930_ps_release(struct inode *inode, struct file *file)
{
    KMSGINF("apds9930_ps_release\n");
    return 0;
}

static long apds9930_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct apds9930_data *data;
    struct i2c_client *client;
    int enable;
    int ret = -1;

    if (arg == 0) return -1;

    if(apds9930_i2c_client == NULL) {
        KMSGERR("apds9930_ps_ioctl error: i2c driver not installed\n");
        return -EFAULT;
    }

    client = apds9930_i2c_client;
    data = i2c_get_clientdata(apds9930_i2c_client);

    switch (cmd) {
    case APDS_IOCTL_PS_ENABLE:

        if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
            KMSGERR("apds9930_ps_ioctl: copy_from_user failed\n");
            return -EFAULT;
        }

        ret = apds9930_enable_ps_sensor(client, enable);
        if(ret < 0) {
            return ret;
        }
        break;

    case APDS_IOCTL_PS_GET_ENABLE:
        if (copy_to_user((void __user *)arg, &data->enable_ps_sensor, sizeof(data->enable_ps_sensor))) {
            KMSGERR("apds9930_ps_ioctl: copy_to_user failed\n");
            return -EFAULT;
        }
        break;

    case APDS_IOCTL_PS_GET_PDATA:

        data->ps_data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_PDATAL_REG);

        if (copy_to_user((void __user *)arg, &data->ps_data, sizeof(data->ps_data))) {
            KMSGERR("apds9930_ps_ioctl: copy_to_user failed\n");
            return -EFAULT;
        }
        break;

    default:
        break;
    }


    return 0;
}

static int apds9930_als_open(struct inode *inode, struct file *file)
{
    KMSGINF("apds9930_als_open\n");
    return 0;
}

static int apds9930_als_release(struct inode *inode, struct file *file)
{
    KMSGINF("apds9930_als_release\n");
    return 0;
}

static long apds9930_als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct apds9930_data *data;
    struct i2c_client *client;
    int enable;
    int ret = -1;
    unsigned int delay;

    if (arg == 0) return -1;

    if(apds9930_i2c_client == NULL){
        KMSGERR("apds9930_als_ioctl error: i2c driver not installed\n");
        return -EFAULT;
    }

    client = apds9930_i2c_client;
    data = i2c_get_clientdata(apds9930_i2c_client);

    switch (cmd) {

    case APDS_IOCTL_ALS_ENABLE:

        if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
            KMSGERR("apds9930_als_ioctl: copy_from_user failed\n");
            return -EFAULT;
        }

        ret = apds9930_enable_als_sensor(client, enable);
        if(ret < 0){
            return ret;
        }
        break;

    case APDS_IOCTL_ALS_POLL_DELAY:

        if (data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT) {
            if (copy_from_user(&delay, (void __user *)arg, sizeof(delay))) {
                KMSGERR("apds9930_als_ioctl: copy_to_user failed\n");
                return -EFAULT;
            }

            ret = apds9930_set_als_poll_delay (client, delay);
            if(ret < 0){
                return ret;
            }
        }
        else {
            KMSGERR("apds9930_als_ioctl: als is not in polling mode!\n");
            return -EFAULT;
        }
        break;

    case APDS_IOCTL_ALS_GET_ENABLE:
        if (copy_to_user((void __user *)arg, &data->enable_als_sensor, sizeof(data->enable_als_sensor))) {
            KMSGERR("apds9930_als_ioctl: copy_to_user failed\n");
            return -EFAULT;
        }
        break;

    case APDS_IOCTL_ALS_GET_CH0DATA:

        data->als_data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);

        if (copy_to_user((void __user *)arg, &data->als_data, sizeof(data->als_data))) {
            KMSGERR("apds9930_ps_ioctl: copy_to_user failed\n");
            return -EFAULT;
        }
        break;

    case APDS_IOCTL_ALS_GET_CH1DATA:

        data->als_data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH1DATAL_REG);

        if (copy_to_user((void __user *)arg, &data->als_data, sizeof(data->als_data))) {
            KMSGERR("apds9930_ps_ioctl: copy_to_user failed\n");
            return -EFAULT;
        }
        break;

    default:
        break;
    }

    return 0;
}

/*
* SysFS support
*/

static ssize_t apds9930_show_ch0data(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ch0data;

    apds9930_wait_for_device_resume(client);
    mutex_lock(&data->operation_lock);
    ch0data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);
    mutex_unlock(&data->operation_lock);

    return sprintf(buf, "%d\n", ch0data);
}

static ssize_t apds9930_show_ch1data(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    int ch1data;

    apds9930_wait_for_device_resume(client);
    mutex_lock(&data->operation_lock);
    ch1data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH1DATAL_REG);
    mutex_unlock(&data->operation_lock);

    return sprintf(buf, "%d\n", ch1data);
}

static ssize_t apds9930_show_pdata(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    int pdata, pdata_average = 0;
    int i = 0;
    apds9930_wait_for_device_resume(client);
    mutex_lock(&data->operation_lock);
    for (i = 0; i < APDS9930_PROX_AVERAGE_RAW_DATA_COUNT; i++)
    {
        pdata = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_PDATAL_REG);
        KMSGINF("%s: ps senosr raw data is ( %d )\n", __func__, pdata);
        if (pdata >= 0)
        {
            pdata_average += pdata;
            msleep(50);
        }
        else
        {
            mutex_unlock(&data->operation_lock);
            return -1;
        }
    }
    pdata = pdata_average/APDS9930_PROX_AVERAGE_RAW_DATA_COUNT;
    mutex_unlock(&data->operation_lock);

    return sprintf(buf, "%d\n", pdata);
}

static ssize_t apds9930_show_proximity_enable(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds9930_store_proximity_enable(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);

    KMSGINF("%s: enable ps senosr ( %ld )\n", __func__, val);

    if ((val != APDS_DISABLE_PS) && (val != APDS_ENABLE_PS_NO_INT) && (val != APDS_ENABLE_PS_WITH_INT)) {
        KMSGERR("**%s:store invalid value=%ld\n", __func__, val);
        return count;
    }

    apds9930_wait_for_device_resume(client);
    mutex_lock(&data->operation_lock);
    apds9930_enable_ps_sensor(client, val);
    mutex_unlock(&data->operation_lock);

    return count;
}

static ssize_t apds9930_show_light_enable(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds9930_store_light_enable(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);

    KMSGINF("%s: enable als sensor ( %ld )\n", __func__, val);

    if ((val != APDS_DISABLE_ALS) && (val != APDS_ENABLE_ALS_WITH_INT) && (val != APDS_ENABLE_ALS_NO_INT))
    {
        KMSGERR("**%s: store invalid valeu=%ld\n", __func__, val);
        return count;
    }

    apds9930_wait_for_device_resume(client);
    mutex_lock(&data->operation_lock);
    apds9930_enable_als_sensor(client, val);
    mutex_unlock(&data->operation_lock);

    return count;
}

static ssize_t apds9930_show_light_poll_delay(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->als_poll_delay);
}

static ssize_t apds9930_store_light_poll_delay(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    unsigned int delay;

    KMSGINF("%s: set als sensor poll delay ( %ld )\n", __func__, val);

    if (data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT)
    {
        if (val >= 1000000000) { // 1000 ms
            delay = APDS_ALS_POLL_SLOW;
        }
        else if (val >= 200000000) { // 200 ms
            delay = APDS_ALS_POLL_MEDIUM;
        }
        else { // 50 ms
            delay = APDS_ALS_POLL_FAST;
        }
        apds9930_wait_for_device_resume(client);
        mutex_lock(&data->operation_lock);
        apds9930_set_als_poll_delay(client, delay);
        mutex_unlock(&data->operation_lock);
    }
    else
    {
        KMSGERR("Is not in NO_INT state\n");
    }

    return count;
}

/* For debug purpose */
static int apds9930_strtok(const char *buf, size_t count, char **token, const int token_nr)
{
    char *buf2 = (char *)kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
    char **token2 = token;
    unsigned int num_ptr = 0, num_nr = 0, num_neg = 0;
    int i = 0, start = 0, end = (int)count;

    strcpy(buf2, buf);

    /* We need to breakup the string into separate chunks in order for kstrtoint
     * or strict_strtol to parse them without returning an error. Stop when the end of
     * the string is reached or when enough value is read from the string */
    while((start < end) && (i < token_nr)) {
        /* We found a negative sign */
        if(*(buf2 + start) == '-') {
            /* Previous char(s) are numeric, so we store their value first before proceed */
            if(num_nr > 0) {
                /* If there is a pending negative sign, we adjust the variables to account for it */
                if(num_neg) {
                    num_ptr--;
                    num_nr++;
                }
                *token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
                strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
                *(*token2+num_nr) = '\n';
                i++;
                token2++;
                /* Reset */
                num_ptr = num_nr = 0;
            }
            /* This indicates that there is a pending negative sign in the string */
            num_neg = 1;
        }
        /* We found a numeric */
        else if((*(buf2 + start) >= '0') && (*(buf2 + start) <= '9')) {
            /* If the previous char(s) are not numeric, set num_ptr to current char */
            if(num_nr < 1)
                num_ptr = start;
            num_nr++;
        }
        /* We found an unwanted character */
        else {
            /* Previous char(s) are numeric, so we store their value first before proceed */
            if(num_nr > 0) {
                if(num_neg) {
                    num_ptr--;
                    num_nr++;
                }
                *token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
                strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
                *(*token2+num_nr) = '\n';
                i++;
                token2++;
            }
            /* Reset all the variables to start afresh */
            num_ptr = num_nr = num_neg = 0;
        }
        start++;
    }

    kfree(buf2);

    return (i == token_nr) ? token_nr : -1;
}

static ssize_t apds9930_read_registers(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    char *buf_write = buf;
    int i = 0, num = 0;

    KMSGINF("%s: enter\n", __func__);

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    for(i = 0; i < 0x10; i++)
    {
        buf_write = buf + num;
        num += sprintf(buf_write, "0x%x - 0x%x\n", i, apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|i));
    }
    for(i = 0x12; i < 0x1A; i++)
    {
        buf_write = buf + num;
        num += sprintf(buf_write, "0x%x - 0x%x\n", i, apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|i));
    }
    buf_write = buf + num;
    num += sprintf(buf_write, "0x1E - 0x%x\n", apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|0x1E));

    mutex_unlock(&data->operation_lock);

    return num;
}

static ssize_t apds9930_write_registers(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    const int buf_count = 2; /* How many valid data that we expect to get from the string */
    char **buf2;
    int buf_output[buf_count];
    int err = 0, i = 0;

    KMSGINF("%s: enter\n", __func__);

    buf2 = (char **)kzalloc(buf_count * sizeof(char *), GFP_KERNEL);

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    if (apds9930_strtok(buf, count, buf2, buf_count) < 0) {
        KMSGERR("%s: Some Error happens\n", __func__);
    }
    else {
        /* Convert string to integers  */
        for(i = 0 ; i < buf_count ; i++) {
            err = kstrtoint((const char *)*(buf2+i), 10, (int *)&buf_output[i]);
            if(err < 0) {
                KMSGERR("%s: kstrtoint returned err = %d." \
                        "No register data will be updated.\n", __func__ , err);
                goto Exit;
            }
        }
        if ( (buf_output[0] <= 0x0F ) || (buf_output[0] == 0x1E) )
        {
            KMSGINF("%s: Will write 0x%x to register 0x%x\n", __func__, buf_output[1], buf_output[0]);
            err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|buf_output[0], buf_output[1]);
        }
        else if ( buf_output[0] == 0x13 )
        {
            if ( buf_output[1] & 0x20 )
            {
                KMSGINF("%s: Clear PS Interrupt\n", __func__);
                apds9930_set_command(client, 0);
            }
            if ( buf_output[1] & 0x10 )
            {
                KMSGINF("%s: Clear ALS Interrupt\n", __func__);
                apds9930_set_command(client, 1);
            }
        }
    }

Exit:
    mutex_unlock(&data->operation_lock);

    for(i = 0 ; i < buf_count ; i++)
        kfree(*(buf2+i));

    kfree(buf2);

    return (err < 0) ? err : count;
}
/* For debug purpose */

/* Proximity self test function */
static ssize_t apds9930_selftest_run_and_get_data(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);

    struct PROX_SELFTEST_RESULT* apds9930_prox_selftest_result = (struct PROX_SELFTEST_RESULT*)buf;
    unsigned int enable_ps_sensor_backup = data->enable_ps_sensor;
    unsigned int enable_als_sensor_backup = data->enable_als_sensor;
    int id;

    KMSGINF("%s: enter\n", __func__);

    memset(apds9930_prox_selftest_result, 0, sizeof(*apds9930_prox_selftest_result));

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    id = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_ID_REG);
    if (id == APDS9930_DEVICE_ID) {
        apds9930_prox_selftest_result->i2c_status = true;
    }
    else {
        apds9930_prox_selftest_result->i2c_status = false;
    }

    apds9930_prox_selftest_result->interrupt_pin_support = true;
    apds9930_prox_selftest_result->interrupt_pin_status = false;

    if ( (apds9930_prox_selftest_result->i2c_status == true) &&
         (apds9930_prox_selftest_result->interrupt_pin_support == true) )
    {
        apds9930_enable_ps_sensor(client, APDS_DISABLE_PS);
        apds9930_enable_als_sensor(client, APDS_DISABLE_ALS);

        atomic_set(&data->ps_selftest_int, 0);
        atomic_set(&data->ps_selftest_ongoing, 1);
        apds9930_enable_ps_sensor(client, APDS_ENABLE_PS_WITH_INT);
        wait_event_interruptible_timeout(data->ps_selftest_wq, atomic_read(&data->ps_selftest_int),
                                         msecs_to_jiffies(500));
        if (atomic_read(&data->ps_selftest_int) == 1) {
            int status = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_STATUS_REG);
            int enable = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_ENABLE_REG);

            apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_ENABLE_REG, 1); /* disable 993x's ADC first */
            if ((status & enable & 0x20) == 0x20) {
                apds9930_set_command(client, 0);  /* 0 = CMD_CLR_PS_INT */
                apds9930_prox_selftest_result->interrupt_pin_status = true;
            }
            enable_irq(client->irq);
        }
        atomic_set(&data->ps_selftest_ongoing, 0);
        apds9930_enable_ps_sensor(client, APDS_DISABLE_PS);
        apds9930_enable_ps_sensor(client, enable_ps_sensor_backup);
        apds9930_enable_als_sensor(client, enable_als_sensor_backup);
    }

    KMSGINF("%s: exit, i2c_status = %d, interrupt_pin_support = %d, interrupt_pin_status = %d\n", __func__,\
            apds9930_prox_selftest_result->i2c_status, apds9930_prox_selftest_result->interrupt_pin_support, apds9930_prox_selftest_result->interrupt_pin_status);

    mutex_unlock(&data->operation_lock);

    return sizeof(*apds9930_prox_selftest_result);
}
/* Proximity self test function */

static ssize_t apds9930_show_proximity_threshold(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "far_to_near: %d\nnear_to_far: %d\n", data->ps_threshold, data->ps_hysteresis_threshold);
}

static ssize_t apds9930_store_proximity_far_to_near_threshold(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    unsigned int param;
    struct file *file;
    mm_segment_t old_fs;
    loff_t sz_file;
    int ret = count, file_ret;
    struct inode *inode;
    struct apds9930_nvitem_data *apds9930_nvitem;
    char *pReadBuff = NULL;
    unsigned long magic;
    unsigned char file_exist = 1;

    /* Read threshold */
    sscanf(buf, "%u", &param);

    KMSGINF("%s: far_to_near_threshold is: %u\n", __func__, param);

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    file = filp_open( APDS9930_NV_BIN_FILE_NAME, O_RDWR, 0 );
    if (IS_ERR(file))
    {
        KMSGERR("%s: open file error, file not exist\n", __func__);
        file = filp_open( APDS9930_NV_BIN_FILE_NAME, O_RDWR|O_CREAT, 0666 );
        if (IS_ERR(file))
        {
            KMSGERR("%s: Create file error...\n", __func__);
            ret = -1;
            goto exit;
        }
        file_exist = 0;
    }

    inode = file->f_dentry->d_inode;
    magic = inode->i_sb->s_magic;
    sz_file = inode->i_size;

    pReadBuff = kzalloc(sizeof(*apds9930_nvitem) + 1,GFP_KERNEL);

    if (!pReadBuff)
    {
        KMSGERR("%s: Mem Alloc Fail...\n", __func__);
        ret = -ENOMEM;
        goto exit_close_file;
    }
    apds9930_nvitem = (struct apds9930_nvitem_data*)pReadBuff;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if ( file_exist )
    {
        if (file->f_op->llseek(file, 0, SEEK_SET) != 0) {
            KMSGERR("%s: lseek failure...\n", __func__);
            ret = -1;
            goto exit_restore_fs;
        }

        file_ret = file->f_op->read(file, (char *)apds9930_nvitem, sizeof(*apds9930_nvitem), &file->f_pos);
        if (file_ret <= 0){
            KMSGERR("%s: read failure...\n", __func__);
            ret = -1;
            goto exit_restore_fs;
        }

        if (file->f_op->llseek(file, 0, SEEK_SET) != 0) {
            KMSGERR("%s: lseek failure...\n", __func__);
            ret = -1;
            goto exit_restore_fs;
        }
    }

    apds9930_nvitem->sensor_device_id = APDS9930_DEVICE_ID;
    apds9930_nvitem->nv_ps_calibration_done = 1;
    apds9930_nvitem->nv_ps_threshold = param;

    file_ret = file->f_op->write(file, pReadBuff, sizeof(*apds9930_nvitem), &file->f_pos);
    if(!file_ret)
    {
        KMSGERR("%s: write file error...\n", __func__);
        ret = -1;
        goto exit_restore_fs;
    }

exit_restore_fs:
    /*resume user space*/
    set_fs(old_fs);
    kfree(pReadBuff);
exit_close_file:
    filp_close(file,NULL);
exit:
    mutex_unlock(&data->operation_lock);
    return ret;
}

static ssize_t apds9930_show_distance_status(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->ps_detection);
}

static int apds9930_get_light_luminance(struct i2c_client *client, int *luminance)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int loop_count = 5;
    int luxValue = 0;
    unsigned char lux_is_valid = 1;

    KMSGINF("%s: enter\n", __func__);

    while (loop_count --) {
        int ch0data, ch1data;
        int err;
        unsigned char change_again=0;
        unsigned char control_data=0;

        luxValue = 0;
        lux_is_valid = 1;
        ch0data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH0DATAL_REG);
        ch1data = apds9930_i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CH1DATAL_REG);
        KMSGINF("ch0data=%d ch1data=%d\n", ch0data, ch1data);

        luxValue = LuxCalculation(client, ch0data, ch1data);
        KMSGINF("luxValue=%d\n", luxValue);

        if (luxValue >= 0) {
            luxValue = luxValue<30000 ? luxValue : 30000;
            data->als_prev_lux = luxValue;
        }
        else {
            lux_is_valid = 0; // don't report, this is invalid lux value
            luxValue = data->als_prev_lux;
            if (data->als_reduce) {
                KMSGINF("als_reduce=%d\n", data->als_reduce);
                lux_is_valid = 1;
                luxValue = 30000; // report anyway since this is the lowest gain
            }
        }

        KMSGINF("lux_is_valid=%d lux=%d ch0data=%d ch1data=%d delay=%d again=%d als_reduce=%d)\n", lux_is_valid, luxValue, ch0data, ch1data, data->als_poll_delay, apds9930_als_again_tb[data->als_again_index], data->als_reduce);

        if (lux_is_valid) {
            // Complete this Calculation
            break;
        }

        data->als_data = ch0data;

        if (data->als_data >= (apds9930_als_res_tb[data->als_atime_index]*90)/100 ) {
            // lower AGAIN if possible
            if (data->als_again_index != APDS9930_ALS_GAIN_1X) {
                data->als_again_index--;
                change_again = 1;
            }
            else {
                err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONFIG_REG, APDS9930_ALS_REDUCE);
                if (err >= 0) {
                    data->als_reduce = 1;
                }
            }
        }
        else if (data->als_data <= (apds9930_als_res_tb[data->als_atime_index]*10)/100) {
            // increase AGAIN if possible
            if (data->als_reduce) {
                err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONFIG_REG, 0);
                if (err >= 0) {
                    data->als_reduce = 0;
                }
            }
            else if (data->als_again_index != APDS9930_ALS_GAIN_120X) {
                data->als_again_index++;
                change_again = 1;
            }
        }

        if (change_again) {
            control_data = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_CONTROL_REG);
            control_data = control_data & 0xFC;
            control_data = control_data | apds9930_als_again_bit_tb[data->als_again_index];
            apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_CONTROL_REG, control_data);
        }

        msleep(100); //Wait 100ms
    }

    if (lux_is_valid)
    {
        *luminance = luxValue;
        return lux_is_valid;
    }
    else
    {
        *luminance = 0;
        return -1;
    }
}

static ssize_t apds9930_show_light_luminance(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    int luminance = 0;

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    if (apds9930_get_light_luminance(client, &luminance) > 0)
    {
        mutex_unlock(&data->operation_lock);
        return sprintf(buf, "%d\n", luminance);
    }
    else
    {
        mutex_unlock(&data->operation_lock);
        KMSGERR("%s: Could not get light luminance\n", __func__);
        return -1;
    }
}

static ssize_t apds9930_store_light_calibration_data(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9930_data *data = i2c_get_clientdata(client);
    int luminance = 0;
    unsigned int param;
    struct file *file;
    mm_segment_t old_fs;
    loff_t sz_file;
    int ret = count, file_ret;
    struct inode *inode;
    struct apds9930_nvitem_data *apds9930_nvitem;
    char *pReadBuff = NULL;
    unsigned long magic;
    unsigned char file_exist = 1;

    apds9930_wait_for_device_resume(client);

    mutex_lock(&data->operation_lock);

    if (apds9930_get_light_luminance(client, &luminance) <= 0)
    {
        KMSGERR("%s: Could not get light luminance\n", __func__);
        ret = -1;
        goto exit;
    }

    if (luminance <= 0)
    {
        KMSGERR("%s: Invalid light luminance\n", __func__);
        ret = -1;
        goto exit;
    }

    /* Read calibration data */
    sscanf(buf, "%u", &param);

    KMSGINF("%s: light calibration data is: %u\n", __func__, param);

    file = filp_open( APDS9930_NV_BIN_FILE_NAME, O_RDWR, 0 );
    if (IS_ERR(file))
    {
        KMSGERR("%s: open file error, file not exist\n", __func__);
        file = filp_open( APDS9930_NV_BIN_FILE_NAME, O_RDWR|O_CREAT, 0666 );
        if (IS_ERR(file))
        {
            KMSGERR("%s: Create file error...\n", __func__);
            ret = -1;
            goto exit;
        }
        file_exist = 0;
    }

    inode = file->f_dentry->d_inode;
    magic = inode->i_sb->s_magic;
    sz_file = inode->i_size;

    pReadBuff = kzalloc(sizeof(*apds9930_nvitem) + 1,GFP_KERNEL);

    if (!pReadBuff)
    {
        KMSGERR("%s: Mem Alloc Fail...\n", __func__);
        ret = -ENOMEM;
        goto exit_close_file;
    }
    apds9930_nvitem = (struct apds9930_nvitem_data*)pReadBuff;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if ( file_exist )
    {
        if (file->f_op->llseek(file, 0, SEEK_SET) != 0) {
            KMSGERR("%s: lseek failure...\n", __func__);
            ret = -1;
            goto exit_restore_fs;
        }

        file_ret = file->f_op->read(file, (char *)apds9930_nvitem, sizeof(*apds9930_nvitem), &file->f_pos);
        if (file_ret <= 0){
            KMSGERR("%s: read failure...\n", __func__);
            ret = -1;
            goto exit_restore_fs;
        }

        if (file->f_op->llseek(file, 0, SEEK_SET) != 0) {
            KMSGERR("%s: lseek failure...\n", __func__);
            ret = -1;
            goto exit_restore_fs;
        }
    }

    apds9930_nvitem->sensor_device_id = APDS9930_DEVICE_ID;
    apds9930_nvitem->nv_als_calibration_done = 1;
    apds9930_nvitem->nv_als_ga = data->als_ga * param / luminance;

    file_ret = file->f_op->write(file, pReadBuff, sizeof(*apds9930_nvitem), &file->f_pos);
    if(!file_ret)
    {
        KMSGERR("%s: write file error...\n", __func__);
        ret = -1;
        goto exit_restore_fs;
    }

exit_restore_fs:
    /*resume user space*/
    set_fs(old_fs);
    kfree(pReadBuff);
exit_close_file:
    filp_close(file,NULL);
exit:
    mutex_unlock(&data->operation_lock);
    return ret;
}

static DEVICE_ATTR(ch0data, S_IRUGO,
    apds9930_show_ch0data, NULL);

static DEVICE_ATTR(ch1data, S_IRUGO,
    apds9930_show_ch1data, NULL);

static DEVICE_ATTR(proximity_raw_data, S_IRUGO,
    apds9930_show_pdata, NULL);

static DEVICE_ATTR(proximity_enable, S_IWUGO | S_IRUGO,
    apds9930_show_proximity_enable, apds9930_store_proximity_enable);

static DEVICE_ATTR(light_enable, S_IWUGO | S_IRUGO,
    apds9930_show_light_enable, apds9930_store_light_enable);

static DEVICE_ATTR(light_poll_delay, S_IWUGO | S_IRUGO,
    apds9930_show_light_poll_delay, apds9930_store_light_poll_delay);

static DEVICE_ATTR(debug_rw_regs, S_IWUGO | S_IRUGO,
    apds9930_read_registers, apds9930_write_registers);

static DEVICE_ATTR(proximity_selftest, S_IRUGO,
    apds9930_selftest_run_and_get_data, NULL);

static DEVICE_ATTR(proximity_crosstalk_data, S_IWUGO | S_IRUGO,
    apds9930_show_proximity_threshold, apds9930_store_proximity_far_to_near_threshold);

static DEVICE_ATTR(proximity_distance_status, S_IRUGO,
    apds9930_show_distance_status, NULL);

static DEVICE_ATTR(light_luminance_data, S_IRUGO,
    apds9930_show_light_luminance, NULL);

static DEVICE_ATTR(light_calibration_data, S_IWUGO,
    NULL, apds9930_store_light_calibration_data);

static struct attribute *apds9930_attributes[] = {
    &dev_attr_ch0data.attr,
    &dev_attr_ch1data.attr,
    &dev_attr_proximity_raw_data.attr,
    &dev_attr_proximity_enable.attr,
    &dev_attr_light_enable.attr,
    &dev_attr_light_poll_delay.attr,
    &dev_attr_debug_rw_regs.attr,
    &dev_attr_proximity_selftest.attr,
    &dev_attr_proximity_crosstalk_data.attr,
    &dev_attr_proximity_distance_status.attr,
    &dev_attr_light_luminance_data.attr,
    &dev_attr_light_calibration_data.attr,
    NULL
};

static const struct attribute_group apds9930_attr_group = {
    .attrs = apds9930_attributes,
};

static struct file_operations apds9930_ps_fops = {
    .owner = THIS_MODULE,
    .open = apds9930_ps_open,
    .release = apds9930_ps_release,
    .unlocked_ioctl = apds9930_ps_ioctl,
};

static struct miscdevice apds9930_ps_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "apds_ps_dev",
    .fops = &apds9930_ps_fops,
};

static struct file_operations apds9930_als_fops = {
    .owner = THIS_MODULE,
    .open = apds9930_als_open,
    .release = apds9930_als_release,
    .unlocked_ioctl = apds9930_als_ioctl,
};

static struct miscdevice apds9930_als_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "apds_als_dev",
    .fops = &apds9930_als_fops,
};

/*
* Initialization function
*/

static int apds9930_init_client(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);
    int err;
    int id;

    err = apds9930_set_enable(client, 0);

    if (err < 0)
        return err;

    id = apds9930_i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9930_ID_REG);
    if (id == APDS9930_DEVICE_ID) {
        KMSGINF("APDS-9930\n");
    }
    else {
        KMSGERR("Not APDS-9930\n");
        return -EIO;
    }

    err = apds9930_set_atime(client, apds9930_als_atime_tb[data->als_atime_index]); // 100.64ms ALS integration time
    if (err < 0) return err;

    err = apds9930_set_ptime(client, 0xFF); // 2.72ms Prox integration time
    if (err < 0) return err;

    err = apds9930_set_wtime(client, 0xFF); // 2.72ms Wait time
    if (err < 0) return err;

    err = apds9930_set_ppcount(client, data->ps_pulse_number);   /* pulse number */
    if (err < 0) return err;

    err = apds9930_set_config(client, 0);   // no long wait
    if (err < 0) return err;

    err = apds9930_i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9930_POFFSET_REG, data->ps_poffset); /* proximity offset */
    if (err < 0) return err;

    err = apds9930_set_control(client, data->ps_pdrive_ma|APDS9930_PRX_IR_DIOD|data->ps_pgain|apds9930_als_again_bit_tb[data->als_again_index]);
    if (err < 0) return err;

    err = apds9930_set_pilt(client, 0);   // init threshold for proximity
    if (err < 0) return err;

    err = apds9930_set_piht(client, data->ps_threshold);
    if (err < 0) return err;

    err = apds9930_set_ailt(client, 0xFFFF);  // force first ALS interrupt to get the environment reading
    if (err < 0) return err;

    err = apds9930_set_aiht(client, 0);
    if (err < 0) return err;

    err = apds9930_set_pers(client, APDS9930_PPERS_1|APDS9930_APERS_1); // 1 consecutive Interrupt persistence
    if (err < 0) return err;

    // sensor is in disabled mode but all the configurations are preset

    return 0;
}

static int __devinit apds9930_parse_dt_and_init(struct device *dev, struct apds9930_data* data)
{
    int ret = 0;
    u32 val;

    ret = of_property_read_u32(dev->of_node, "apds9930,ps_detection_threshold", &val);
    if (!ret)
    {
        KMSGINF("The ps_detection_threshold is %d\n", val);
        data->ps_threshold = val;
    }
    else
    {
        KMSGERR("The ps_detection_threshold is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,ps_hsythresis_threshold", &val);
    if (!ret)
    {
        KMSGINF("The ps_hsythresis_threshold is %d\n", val);
        data->ps_hysteresis_threshold = val;
    }
    else
    {
        KMSGERR("The ps_hsythresis_threshold is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,ps_pulse_number", &val);
    if (!ret)
    {
        KMSGINF("The ps_pulse_number is %d\n", val);
        data->ps_pulse_number = val;
    }
    else
    {
        KMSGERR("The ps_pulse_number is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,ps_pdrive_ma", &val);
    if (!ret)
    {
        if (val == APDS9930_PDRVIE_100MA)
            KMSGINF("The ps_pdrive_ma is 100MA\n");
        else if (val == APDS9930_PDRVIE_50MA)
            KMSGINF("The ps_pdrive_ma is 50MA\n");
        else if (val == APDS9930_PDRVIE_25MA)
            KMSGINF("The ps_pdrive_ma is 25MA\n");
        else if (val == APDS9930_PDRVIE_12_5MA)
            KMSGINF("The ps_pdrive_ma is 12.5MA\n");
        else
        {
            KMSGERR("The ps_pdrive_ma setting is wrong, ps_pdrive_ma = 0x%x\n", val);
            ret = -ENOSYS;
            goto exit;
        }
        data->ps_pdrive_ma = val;
    }
    else
    {
        KMSGERR("The ps_pdrive_ma is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,ps_pgain", &val);
    if (!ret)
    {
        if (val == APDS9930_PGAIN_1X)
            KMSGINF("The ps_pgain is 1X\n");
        else if (val == APDS9930_PGAIN_2X)
            KMSGINF("The ps_pgain is 2X\n");
        else if (val == APDS9930_PGAIN_4X)
            KMSGINF("The ps_pgain is 4X\n");
        else if (val == APDS9930_PGAIN_8X)
            KMSGINF("The ps_pgain is 8X\n");
        else
        {
            KMSGERR("The ps_pgain setting is wrong, ps_pgain = 0x%x\n", val);
            ret = -ENOSYS;
            goto exit;
        }
        data->ps_pgain = val;
    }
    else
    {
        KMSGERR("The ps_pgain is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,ps_poffset", &val);
    if (!ret)
    {
        KMSGINF("The ps_poffset is %d\n", val);
        data->ps_poffset = val;
    }
    else
    {
        KMSGERR("The ps_poffset is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,als_use_irq_in_idle", &val);
    if (!ret)
    {
        KMSGINF("The als_use_irq_in_idle is %d\n", val);
        data->als_use_irq_in_idle = val;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,als_threshold_hsytersis", &val);
    if (!ret)
    {
        KMSGINF("The als_threshold_hsytersis is %d\n", val);
        data->als_threshold_hsytersis = val;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,als_ga", &val);
    if (!ret)
    {
        KMSGINF("The als_ga is %d\n", val);
        data->als_ga = val;
    }
    else
    {
        KMSGERR("The als_ga is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,als_coe_b", &val);
    if (!ret)
    {
        KMSGINF("The als_coe_b is %d\n", val);
        data->als_coe_b = val;
    }
    else
    {
        KMSGERR("The als_coe_b is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,als_coe_c", &val);
    if (!ret)
    {
        KMSGINF("The als_coe_c is %d\n", val);
        data->als_coe_c = val;
    }
    else
    {
        KMSGERR("The als_coe_c is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,als_coe_d", &val);
    if (!ret)
    {
        KMSGINF("The als_coe_d is %d\n", val);
        data->als_coe_d = val;
    }
    else
    {
        KMSGERR("The als_coe_d is not set\n");
        goto exit;
    }

    ret = of_property_read_u32(dev->of_node, "apds9930,als_df", &val);
    if (!ret)
    {
        KMSGINF("The als_df is %d\n", val);
        data->als_df = val;
    }
    else
    {
        KMSGERR("The als_df is not set\n");
        goto exit;
    }

    data->enable = 0; /* default mode is standard */
    data->ps_detection = 0; /* default to no detection */
    data->enable_als_sensor = 0;  // default to 0
    data->enable_ps_sensor = 0; // default to 0
    data->als_poll_delay = 100; // default to 100ms
    data->ps_poll_delay = 100; // default to 100ms
    data->als_atime_index = APDS9930_ALS_RES_100MS; // 100ms ATIME
    data->als_again_index = APDS9930_ALS_GAIN_8X; // 8x AGAIN
    data->als_reduce = 0; // no ALS 6x reduction
    data->als_prev_lux = 0;
    atomic_set(&data->suspended, 0);
    data->enable_suspended_value = 0; /* suspend_resume usage */

    atomic_set(&data->ps_selftest_ongoing, 0);
    atomic_set(&data->ps_selftest_int, 0);

exit:
    return ret;
}

/*
* I2C init/probing/exit functions
*/

static struct i2c_driver apds9930_driver;
static int __devinit apds9930_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct apds9930_data *data;
    int err = 0;

    apds9930_i2c_client = client;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
        KMSGERR("Client is not i2c capable. Abort.\n");
        err = -EIO;
        goto exit;
    }

    data = kzalloc(sizeof(struct apds9930_data), GFP_KERNEL);
    if (!data) {
        KMSGERR("Failed to allocate memory.\n");
        err = -ENOMEM;
        goto exit;
    }

    data->client = client;

    i2c_set_clientdata(client, data);

    /* Parse the parameters and initialize */
    err = apds9930_parse_dt_and_init(&client->dev, data);
    if (err)
        goto exit_kfree;

    /* Initialize the APDS993X chip */
    err = apds9930_init_client(client);
    if (err)
        goto exit_kfree;

    mutex_init(&data->operation_lock);

    INIT_DELAYED_WORK(&data->dwork, apds9930_ps_als_int_work_handler);

    INIT_DELAYED_WORK(&data->ps_dwork, apds9930_ps_polling_work_handler);

    INIT_DELAYED_WORK(&data->als_dwork, apds9930_als_polling_work_handler);

    init_waitqueue_head(&data->ps_selftest_wq);

    /* Register to Input Device */
    data->input_dev_als = input_allocate_device();
    if (!data->input_dev_als) {
        err = -ENOMEM;
        KMSGERR("Failed to allocate input device als\n");
        goto exit_kfree;
    }

    data->input_dev_ps = input_allocate_device();
    if (!data->input_dev_ps) {
        err = -ENOMEM;
        KMSGERR("Failed to allocate input device ps\n");
        goto exit_free_dev_als;
    }

    input_set_capability(data->input_dev_als, EV_REL, REL_LIGHT);

    input_set_capability(data->input_dev_ps, EV_ABS, ABS_DISTANCE);
    input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 10, 0, 0);

    data->input_dev_als->name = "light";
    data->input_dev_als->id.bustype = BUS_I2C;
    data->input_dev_als->dev.parent = &data->client->dev;

    data->input_dev_ps->name = "proximity";
    data->input_dev_ps->id.bustype = BUS_I2C;
    data->input_dev_ps->dev.parent = &data->client->dev;

    err = input_register_device(data->input_dev_als);
    if (err) {
        err = -ENOMEM;
        KMSGERR("Unable to register input device als: %s\n",
            data->input_dev_als->name);
        goto exit_free_dev_ps;
    }

    err = input_register_device(data->input_dev_ps);
    if (err) {
        err = -ENOMEM;
        KMSGERR("Unable to register input device ps: %s\n",
            data->input_dev_ps->name);
        goto exit_unregister_dev_als;
    }

    /* Register sysfs hooks */
    err = sysfs_create_group(&client->dev.kobj, &apds9930_attr_group);
    if (err)
        goto exit_unregister_dev_ps;

    /* Register for sensor ioctl */
    err = misc_register(&apds9930_ps_device);
    if (err) {
        KMSGERR("Unalbe to register ps ioctl: %d", err);
        goto exit_remove_sysfs_group;
    }

    err = misc_register(&apds9930_als_device);
    if (err) {
        KMSGERR("Unalbe to register als ioctl: %d", err);
        goto exit_unregister_ps_ioctl;
    }

    {
        int irq_no = of_get_named_gpio(client->dev.of_node, "apds9930,irq-gpio", 0);
        if (!gpio_is_valid(irq_no)) {
            KMSGERR("Interrupt GPIO is not specified\n");
            goto exit_unregister_als_ioctl;
        }
        err = gpio_request(irq_no, "apds9930_irq");
        if (err) {
            KMSGERR("Request gpio failed, err = %d\n", err);
            goto exit_unregister_als_ioctl;
        }
        err = gpio_tlmm_config(GPIO_CFG(
            irq_no, 0,
            GPIO_CFG_INPUT,
            GPIO_CFG_PULL_UP,
            GPIO_CFG_2MA),
            GPIO_CFG_ENABLE);
        if (err) {
            KMSGERR("Unable to config tlmm = %d\n", err);
            gpio_free(irq_no);
            goto exit_unregister_als_ioctl;
        }
        err = gpio_direction_input(irq_no);
        if (err) {
            KMSGERR("Set direction for irq failed, err = %d\n", err);
            gpio_free(irq_no);
            goto exit_unregister_als_ioctl;
        }
        client->irq = gpio_to_irq(irq_no);
        KMSGINF("IRQ Number is %d\n", client->irq);
        if (request_irq(client->irq, apds9930_interrupt, IRQF_TRIGGER_LOW,
            APDS9930_DRV_NAME, (void *)client)) {
                gpio_free(irq_no);
                KMSGERR("%s Could not allocate APDS9930_INT !\n", __func__);
                goto exit_unregister_als_ioctl;
        }
    }
    // KMSGINF("IRQ Number is %d\n", client->irq);
    // if (request_irq(client->irq, apds9930_interrupt, IRQF_TRIGGER_LOW,
    //     APDS9930_DRV_NAME, (void *)client)) {
    //         KMSGERR("%s Could not allocate APDS9930_INT !\n", __func__);

    //         goto exit_unregister_als_ioctl;
    // }

    irq_set_irq_wake(client->irq, 1);

    KMSGINF("%s interrupt is hooked\n", __func__);

    KMSGINF("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);

    return 0;

exit_unregister_als_ioctl:
    misc_deregister(&apds9930_als_device);
exit_unregister_ps_ioctl:
    misc_deregister(&apds9930_ps_device);
exit_remove_sysfs_group:
    sysfs_remove_group(&client->dev.kobj, &apds9930_attr_group);
exit_unregister_dev_ps:
    input_unregister_device(data->input_dev_ps);
exit_unregister_dev_als:
    input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
    input_free_device(data->input_dev_ps);
exit_free_dev_als:
    input_free_device(data->input_dev_als);
exit_kfree:
    kfree(data);
exit:
    return err;
}

static int __devexit apds9930_remove(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);

    /* Power down the device */
    apds9930_set_enable(client, 0);

    misc_deregister(&apds9930_als_device);
    misc_deregister(&apds9930_ps_device);

    sysfs_remove_group(&client->dev.kobj, &apds9930_attr_group);

    input_unregister_device(data->input_dev_ps);
    input_unregister_device(data->input_dev_als);

    free_irq(client->irq, client);

    kfree(data);

    return 0;
}

#ifdef CONFIG_PM

static int apds9930_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct apds9930_data *data = i2c_get_clientdata(client);

    KMSGINF("%s\n", __func__);
    atomic_set(&data->suspended, 1);
    return 0;
}

static int apds9930_resume(struct i2c_client *client)
{
    struct apds9930_data *data = i2c_get_clientdata(client);

    KMSGINF("%s\n", __func__);
    atomic_set(&data->suspended, 0);
    return 0;
}

#else

#define apds9930_suspend  NULL
#define apds9930_resume   NULL

#endif

static const struct i2c_device_id apds9930_id[] = {
    { APDS9930_DRV_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, apds9930_id);

static const struct of_device_id apds9930_prox_als_of_match[] = {
    { .compatible = "avago,apds9930", },
    { },
};

static struct i2c_driver apds9930_driver = {
    .driver = {
        .name = APDS9930_DRV_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = apds9930_prox_als_of_match,
    },
    .suspend = apds9930_suspend,
    .resume = apds9930_resume,
    .probe  = apds9930_probe,
    .remove = __devexit_p(apds9930_remove),
    .id_table = apds9930_id,
};

static int __init apds9930_init(void)
{
    apds_workqueue = create_workqueue("proximity_als");

    if (!apds_workqueue)
        return -ENOMEM;

    return i2c_add_driver(&apds9930_driver);
}

static void __exit apds9930_exit(void)
{
    if (apds_workqueue)
        destroy_workqueue(apds_workqueue);

    apds_workqueue = NULL;

    i2c_del_driver(&apds9930_driver);
}

MODULE_AUTHOR("Zifeng Qiu <zifeng.qiu@nokia.com>");
MODULE_DESCRIPTION("APDS993X ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9930_init);
module_exit(apds9930_exit);

