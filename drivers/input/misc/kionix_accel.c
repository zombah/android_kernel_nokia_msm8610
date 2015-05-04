/* drivers/input/misc/kionix_accel.c - Kionix accelerometer driver
 *
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kionix_accel.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/of_device.h>

/* Debug Message Flags */
#define KIONIX_KMSG_ERR    1    /* Print kernel debug message for error */
#define KIONIX_KMSG_INF    1    /* Print kernel debug message for info */

#if KIONIX_KMSG_ERR
#define KMSGERR(format, ...)    dev_err(format, ## __VA_ARGS__)
#else
#define KMSGERR(format, ...)
#endif

#if KIONIX_KMSG_INF
#define KMSGINF(format, ...)    dev_info(format, ## __VA_ARGS__)
#else
#define KMSGINF(format, ...)
#endif

#define DOUBLE_TAP_INTERRUPT   REL_MISC

/******************************************************************************
 * Accelerometer WHO_AM_I return value
 *****************************************************************************/
#define KIONIX_ACCEL_WHO_AM_I_KX022             0x14

/******************************************************************************
 * Registers for All Accelerometer Group
 *****************************************************************************/
#define ACCEL_WHO_AM_I        0x0F

/*****************************************************************************/
/* Registers for Accelerometer Group 7 */
/*****************************************************************************/
/* Registers */
#define ACCEL_XOUT_L          0x06
#define ACCEL_INT_STATUS1     0x12
#define ACCEL_INT_STATUS2     0x13
#define ACCEL_INT_REL         0x17
#define ACCEL_CTRL_REG1       0x18
#define ACCEL_CTRL_REG3       0x1A
#define ACCEL_INT_CTRL1       0x1C
#define ACCEL_DATA_CTRL       0x1B    /* ODCNTL */
#define ACCEL_INT_CTRL3       0x1E
#define ACCEL_INT_CTRL4       0x1F
#define ACCEL_TDTRC           0x24    /* Enable single or double tap */
#define ACCEL_TDTC            0x25    /* The duration between two taps */
#define ACCEL_TTH             0x26    /* The highest threshold of tap */
#define ACCEL_TTL             0x27    /* The lowest threshold of tap */
#define ACCEL_FTD             0x28    /* The duration between two thresholds for one tap */
#define ACCEL_STD             0x29    /* THe whole duration include two taps */
#define ACCEL_TLT             0x2A
#define ACCEL_TWS             0x2B
#define ACCEL_SELFTEST_REG    0x60    /* SELF_TEST REG */
/* INT_STATUS_REG1 */
#define ACCEL_INT_TAP_Z_POSITIVE   0x01
#define ACCEL_INT_TAP_Z_NEGATIVE   0x02
#define ACCEL_INT_TAP_Z       (ACCEL_INT_TAP_Z_POSITIVE | ACCEL_INT_TAP_Z_NEGATIVE)
/* INT_STATUS_REG2 */
#define ACCEL_INT_NO_TAP      0x00
#define ACCEL_INT_SINGLE_TAP  0x04
#define ACCEL_INT_DOUBLE_TAP  0x08
/* CTRL_REG1 */
#define ACCEL_PC1_OFF         0x7F
#define ACCEL_PC1_ON          (1 << 7)
#define ACCEL_DRDYE           (1 << 5)
#define ACCEL_TDTE            (1 << 2)
#define ACCEL_G_8G            (2 << 3)
#define ACCEL_G_4G            (1 << 3)
#define ACCEL_G_2G            (0 << 3)
#define ACCEL_G_MASK          (3 << 3)
#define ACCEL_RES_8BIT        (0 << 6)
#define ACCEL_RES_16BIT       (1 << 6)
#define ACCEL_RES_MASK        (1 << 6)
/* CTRL_REG3 */
#define ACCEL_ODR_DDTAP_50    0x00
#define ACCEL_ODR_DDTAP_100   0x08
#define ACCEL_ODR_DDTAP_200   0x10
#define ACCEL_ODR_DDTAP_400   0x18
#define ACCEL_ODR_DDTAP_12_5  0x20
#define ACCEL_ODR_DDTAP_25    0x28
#define ACCEL_ODR_DDTAP_800   0x30
#define ACCEL_ODR_DDTAP_1600  0x38
/* INT_CTRL1 */
#define ACCEL_IEA             (1 << 4)
#define ACCEL_IEN             (1 << 5)
/* INT_CTRL3 */
#define ACCEL_Z_POSITIVE      0x01
#define ACCEL_Z_NEGATIVE      0x02
#define ACCEL_Z_TAP_ENABLE    (ACCEL_Z_POSITIVE | ACCEL_Z_NEGATIVE)
/* INT_CTRL4 */
#define ACCEL_INT1_MAP_TDTI   0x04
/* TDTRC */
#define ACCEL_TDTRC_STRE      0x01
#define ACCEL_TDTRC_DTRE      0x02
/* DATA_CTRL_REG */
#define ACCEL_ODR0_781        0x08
#define ACCEL_ODR1_563        0x09
#define ACCEL_ODR3_125        0x0A
#define ACCEL_ODR6_25         0x0B
#define ACCEL_ODR12_5         0x00
#define ACCEL_ODR25           0x01
#define ACCEL_ODR50           0x02
#define ACCEL_ODR100          0x03
#define ACCEL_ODR200          0x04
#define ACCEL_ODR400          0x05
#define ACCEL_ODR800          0x06
#define ACCEL_ODR1600         0x07

/* Double tap tuning */
/*
|
|....................................................!............   TTH
|        |            !            |                 !
|       | |           !           | |                !
|       | |           !           | |                !
|      |   |          !          |   |               !
|      |   |          !          |   |               !
|.....|.....|.........!.........|.....|..............!............   TTL
|    |:     :|        !        |:     :|             !
|   | :     : |       !       | :     : |            !
|__|__:_____:__|______!______|__:_____:__|___________!______
   !  :STD1 :  !      !      !  :STD2 :  !           !               STD = STD1+STD2
   !           !      !      !           !           !               FTD_HIGH > STD1(2) > FTD_LOW
   !----TLT----!      !      !----TLT----!           !               TLT > STD1(2)
   !-------TDTC-------!                              !               TDTC > TLT
   !----------------------TWS------------------------!               TWS >= TDTC * 2
*/
#define DOUBLE_TAP_TUNING_TDTC           57      /* 57 / 1600 = 35ms */
#define DOUBLE_TAP_TUNING_TTH            0x7F
#define DOUBLE_TAP_TUNING_TTL            0x18    /* 4G-8bit, unit is 31.25mg, 750/31.25=0x18 */
#define DOUBLE_TAP_TUNING_FTD            0x60    /* FTD_HIGH = 7.5ms, FTD_LOW = 0 */ /* 0x50-6.25ms */
#define DOUBLE_TAP_TUNING_STD            24      /* 24 / 1600 = 15ms */
#define DOUBLE_TAP_TUNING_TLT            32      /* 32 / 1600 = 20ms */ /* 56-35ms */
#define DOUBLE_TAP_TUNING_TWS            (DOUBLE_TAP_TUNING_TLT+1)      /* 33 / 1600 = 21ms */ /* 96-60ms */
/*****************************************************************************/

/* Input Event Constants */
#define ACCEL_G_MAX           8096
#define ACCEL_FUZZ            0
#define ACCEL_FLAT            0
/* I2C Retry Constants */
#define KIONIX_I2C_RETRY_COUNT      10   /* Number of times to retry i2c */
#define KIONIX_I2C_RETRY_TIMEOUT    10    /* Timeout between retry (miliseconds) */

/* Earlysuspend Contants */
#define KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT    5000    /* Timeout (miliseconds) */

/* Below are for selftest */
/* Need 50ms for stable when power on */
#define KIONIX_ACCEL_POWER_ON_TIME_MS               50

#define KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS      10
#define KIONIX_ACCEL_ACTIVE_TO_STANDBY_TIME_MS      10

/* From Datasheet, need to wait 100ms for selftest data ready */
#define KIONIX_ACCEL_SELFTEST_START_TIME_MS         100
#define KIONIX_ACCEL_SELFTEST_STOP_TIME_MS          100
 /* Used time between sample readings, we will 1.6KHZ */
#define KIONIX_ACCEL_SAMPLE_READY_TIME_MS           2
/* Selftest samples */
#define KIONIX_ACCEL_SELFTEST_SAMPLES               10
/* Selftest threshold for self-motivation */
#define KIONIX_ACCEL_SELFTEST_X_HIGH_POSITIVE       10650
#define KIONIX_ACCEL_SELFTEST_X_LOW_POSITIVE        5734
#define KIONIX_ACCEL_SELFTEST_Y_HIGH_POSITIVE       10650
#define KIONIX_ACCEL_SELFTEST_Y_LOW_POSITIVE        5734
#define KIONIX_ACCEL_SELFTEST_Z_HIGH_POSITIVE       10650
#define KIONIX_ACCEL_SELFTEST_Z_LOW_POSITIVE        5734
#define KIONIX_ACCEL_SELFTEST_X_HIGH_NEGATIVE       -7372
#define KIONIX_ACCEL_SELFTEST_X_LOW_NEGATIVE        -13108
#define KIONIX_ACCEL_SELFTEST_Y_HIGH_NEGATIVE       -5734
#define KIONIX_ACCEL_SELFTEST_Y_LOW_NEGATIVE        -10650
#define KIONIX_ACCEL_SELFTEST_Z_HIGH_NEGATIVE       -5734
#define KIONIX_ACCEL_SELFTEST_Z_LOW_NEGATIVE        -10650
/* Upper are for selftest */

#define MASK_16_10_BITS   0xFFC0

/* Pay attention to this structure, it must be the same as Selftest HAL */
struct ACCEL_SELFTEST_RESULT {
    bool i2c_status;
    bool interrupt_pin_support;
    bool interrupt_pin_status;
    bool x_axis_pass;
    bool y_axis_pass;
    bool z_axis_pass;
    int  x_axis_result;
    int  y_axis_result;
    int  z_axis_result;
};

struct kionix_accel_axis_data {
    int x_axis;
    int y_axis;
    int z_axis;
};
/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate (ODR).
 */
static const struct {
    unsigned int cutoff;
    u8 mask;
} kionix_accel_odr_table[] = {
    { 2,    ACCEL_ODR1600 },
    { 3,    ACCEL_ODR800  },
    { 5,    ACCEL_ODR400  },
    { 10,   ACCEL_ODR200  },
    { 20,   ACCEL_ODR100  },
    { 40,   ACCEL_ODR50   },
    { 80,   ACCEL_ODR25   },
    { 160,  ACCEL_ODR12_5 },
    { 320,  ACCEL_ODR6_25 },
    { 640,  ACCEL_ODR3_125},
    { 1280, ACCEL_ODR1_563},
    { 0,    ACCEL_ODR0_781},
};

enum {
    accel_ctrl_reg1 = 0,
    accel_data_ctrl,
    accel_int_ctrl,
    accel_int_ctrl4,
    accel_regs_count,
};

enum {
    TAP_NO_DIRECTION = 0,
    TAP_POSITIVE,
    TAP_NEGATIVE,
};

enum {
    ACCEL_I2C_RESUME = 0,
    ACCEL_I2C_SUSPEND,
};

struct kionix_accel_driver {
    struct i2c_client *client;
    struct kionix_accel_platform_data accel_pdata;
    struct input_dev *input_dev;
    struct input_dev *input_dev_ddtap;
    struct delayed_work accel_work;
    struct workqueue_struct *accel_workqueue;

    int accel_data[3];
    u8 axis_map_x;
    u8 axis_map_y;
    u8 axis_map_z;
    bool negate_x;
    bool negate_y;
    bool negate_z;
    u8 shift;

    unsigned int poll_interval;
    unsigned int poll_delay;
    u8 *accel_registers;

    atomic_t accel_enabled;          /* Indicate whether there are any clients open the accelerometer */
    atomic_t accel_enabled_int;      /* Indicate whether there are any clients open the accelerometer int */
    atomic_t accel_input_event;      /* Indicate whether there are any clients are waiting input event */
    rwlock_t rwlock_accel_data;

    bool accel_ddtap;
    int irq1;
    struct work_struct irq1_work;
    struct workqueue_struct *irq1_work_queue;
    int irq2;
    struct work_struct irq2_work;
    struct workqueue_struct *irq2_work_queue;

    atomic_t accel_selftest_int;
    atomic_t accel_selftest_ongoing;
    wait_queue_head_t accel_selftest_wq;

    struct proc_dir_entry *proc_dir, *proc_entry;

    unsigned int test;
    struct delayed_work accel_test;
    struct workqueue_struct *accel_testqueue;

    unsigned long tap_pre_tip, tap_tip, tap_init_tip;
    unsigned int tap_time_ms;
    u8 tap_pre_direction, tap_direction;

    atomic_t suspend;
};

static void kionix_accel_i2c_resume(struct kionix_accel_driver *acceld)
{
    do {
        if (atomic_read(&acceld->suspend) == ACCEL_I2C_RESUME)
            break;
        else {  /* ACCEL_i2C_SUSPEND */
            KMSGINF(&acceld->client->dev, "Accel, %s, suspended state is %d\n", __func__, atomic_read(&acceld->suspend));
            msleep(10);
        }
    } while(1);
}

static s32 kionix_accel_i2c_write_byte_data(struct i2c_client *client, u8 command, u8 value)
{
    u8 index;
    s32 ret;
    for (index = 0; index < KIONIX_I2C_RETRY_COUNT; index++) {
        ret = i2c_smbus_write_byte_data(client, command, value);
        if (ret >= 0) {
            break;
        }
        else {
            KMSGERR(&client->dev, "Accel, i2c_smbus_write_byte_data, command = 0x%x, value = 0x%x, ret = %d, index = %u\n", command, value, ret, index);
            msleep(KIONIX_I2C_RETRY_TIMEOUT);
        }
    }
    return ret;
}

static s32 kionix_accel_i2c_read_byte_data(struct i2c_client *client, u8 command)
{
    u8 index;
    s32 ret;
    for (index = 0; index < KIONIX_I2C_RETRY_COUNT; index++) {
        ret = i2c_smbus_read_byte_data(client, command);
        if (ret >= 0) {
            break;
        }
        else {
            KMSGERR(&client->dev, "Accel, i2c_smbus_read_byte_data, command = 0x%x, ret = %d, index = %u\n", command, ret, index);
            msleep(KIONIX_I2C_RETRY_TIMEOUT);
        }
    }
    return ret;
}

static int kionix_acc_parse_dt(struct device *dev, struct kionix_accel_platform_data *acceld)
{
    int ret = 0;
    u32 val;

    ret = of_property_read_u32(dev->of_node, "kionix_acc,min_interval", &val);
    if (!ret)
        acceld->min_interval = val;

    ret = of_property_read_u32(dev->of_node, "kionix_acc,poll_interval", &val);
    if (!ret)
        acceld->poll_interval = val;

    ret = of_property_read_u32(dev->of_node, "kionix_acc,accel_direction", &val);
    if (!ret)
        acceld->accel_direction = (u8)val;

    ret = of_property_read_u32(dev->of_node, "kionix_acc,accel_irq_use_ddtap", &val);
    if (!ret)
        acceld->accel_irq_use_ddtap = (bool)val;

    ret = of_property_read_u32(dev->of_node, "kionix_acc,accel_res", &val);
    if (!ret)
        acceld->accel_res = (u8)val;

    ret = of_property_read_u32(dev->of_node, "kionix_acc,accel_g_range", &val);
    if (!ret)
        acceld->accel_g_range = (u8)val;

    /* accel_tap_z_direction: 0, single direction, 1, two direction */
    ret = of_property_read_u32(dev->of_node, "kionix_acc,accel_tap_z_direction", &val);
    if (!ret)
        acceld->accel_tap_z_direction = (u8)val;

    /* accel_dtap_z_direction: 0, double tap with same direction, 1, double tap with any two direction */
    ret = of_property_read_u32(dev->of_node, "kionix_acc,accel_dtap_z_direction", &val);
    if (!ret)
        acceld->accel_dtap_z_direction = (u8)val;

    return ret;
}

static int kionix_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len)
{
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = client->flags,
            .len = 1,
            .buf = &addr,
        },
        {
            .addr = client->addr,
            .flags = client->flags | I2C_M_RD,
            .len = len,
            .buf = data,
        },
    };

    return i2c_transfer(client->adapter, msgs, 2);
}

static int kionix_accel_power_on_init(struct kionix_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    /* ensure that PC1 is cleared before updating control registers */
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, 0);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, can not enter standby mode\n", __func__);
        return err;
    }
    msleep(KIONIX_ACCEL_ACTIVE_TO_STANDBY_TIME_MS);

    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_DATA_CTRL, acceld->accel_registers[accel_data_ctrl]);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config work ODR fail.\n", __func__);
        return err;
    }

    if (acceld->accel_ddtap == 1) {
        err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_INT_CTRL1, acceld->accel_registers[accel_int_ctrl]);
        if (err < 0) {
            KMSGERR(&acceld->client->dev, "Accel, %s, config basic int init fail.\n", __func__);
            return err;
        }
    }

    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, (acceld->accel_registers[accel_ctrl_reg1] | ACCEL_PC1_ON));
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, can not enter normal mode\n", __func__);
        return err;
    }
    msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);

    return err;
}

static void kionix_accel_report_accel_data(struct kionix_accel_driver *acceld)
{
    struct { union {
        s16 accel_data_s16[3];
        s8  accel_data_s8[6];
    }; } accel_data;
    s16 x, y, z;
    int err;
    /* Only read the output registers if enabled */
    if(atomic_read(&acceld->accel_enabled) > 0) {
        err = kionix_i2c_read(acceld->client, ACCEL_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
        if (err < 0) {
            KMSGERR(&acceld->client->dev, "%s: Accel, faild to read work data output\n", __func__);
        } else {
            write_lock(&acceld->rwlock_accel_data);
            x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) & MASK_16_10_BITS;
            y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) & MASK_16_10_BITS;
            z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) & MASK_16_10_BITS;
            x = x >> acceld->shift; y = y >> acceld->shift; z = z >> acceld->shift;
            acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x);
            acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y);
            acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z);

            if(atomic_read(&acceld->accel_input_event) > 0) {
                input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
                input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
                input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
                input_sync(acceld->input_dev);
            }

            write_unlock(&acceld->rwlock_accel_data);
        }
    }
}

static int kionix_accel_int_init(struct kionix_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    /* ensure that PC1 is cleared before updating control registers */
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, 0);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, can not enter standby mode\n", __func__);
        return err;
    }
    msleep(KIONIX_ACCEL_ACTIVE_TO_STANDBY_TIME_MS);

    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG3, ACCEL_ODR_DDTAP_1600);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config ODR 1600Hz fail\n", __func__);
        return err;
    }

    if (acceld->accel_pdata.accel_tap_z_direction == 0) {
        /* negate_z: 0, the sensor layout is positive, the direction of sensor is same as phone. */
        /*                1, the sensor layout is reverse with layout. */
        /* If the sensor layout is positive, the double tap direction is negative, or else it is positive. */
        if (acceld->negate_z == 0) {
            err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_INT_CTRL3, ACCEL_Z_NEGATIVE);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, config Z axis tap positive fail\n", __func__);
                return err;
            }
        }
        else {
            err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_INT_CTRL3, ACCEL_Z_POSITIVE);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, config Z axis tap negative fail\n", __func__);
                return err;
            }
        }
    }
    else {
        err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_INT_CTRL3, ACCEL_Z_TAP_ENABLE);
        if (err < 0) {
            KMSGERR(&acceld->client->dev, "Accel, %s, config Z axis tap two direction fail\n", __func__);
            return err;
        }
    }

    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_INT_CTRL4, ACCEL_INT1_MAP_TDTI);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config tap map INT1 fail\n", __func__);
        return err;
    }
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_TDTRC, ACCEL_TDTRC_STRE);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config single tap enable fail\n", __func__);
        return err;
    }
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_TTH, DOUBLE_TAP_TUNING_TTH);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config high threshold fail\n", __func__);
        return err;
    }
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_TTL, DOUBLE_TAP_TUNING_TTL);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config low threshold fail\n", __func__);
        return err;
    }
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_FTD, DOUBLE_TAP_TUNING_FTD);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config FTD fail\n", __func__);
        return err;
    }
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_TLT, DOUBLE_TAP_TUNING_TLT);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config TLT fail\n", __func__);
        return err;
    }
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_TWS, DOUBLE_TAP_TUNING_TWS);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config TWS fail\n", __func__);
        return err;
    }

    return err;
}


static void kionix_accel_update_g_range(struct kionix_accel_driver *acceld)
{
    acceld->accel_registers[accel_ctrl_reg1] &= ~ACCEL_G_MASK;
    KMSGINF(&acceld->client->dev, "Accel, %s, range = %u\n", __func__, acceld->accel_pdata.accel_g_range);

    switch (acceld->accel_pdata.accel_g_range) {
        case KIONIX_ACCEL_G_8G:
        case KIONIX_ACCEL_G_6G:
            acceld->shift = 2;
            acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_G_8G;
            break;
        case KIONIX_ACCEL_G_4G:
            acceld->shift = 3;
            acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_G_4G;
            break;
        case KIONIX_ACCEL_G_2G:
        default:
            acceld->shift = 4;
            acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_G_2G;
            break;
    }

    return;
}

static int kionix_accel_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
    int err = 0, i;
    u8 odr;
    KMSGINF(&acceld->client->dev, "Accel, %s, update odr %d\n", __func__, poll_interval);

    /* Use the lowest ODR that can support the requested poll interval */
    for (i = 0; i < ARRAY_SIZE(kionix_accel_odr_table); i++) {
        odr = kionix_accel_odr_table[i].mask;
        if (poll_interval < kionix_accel_odr_table[i].cutoff)
            break;
    }

    /* Do not need to update DATA_CTRL_REG register if the ODR is not changed */
    if(acceld->accel_registers[accel_data_ctrl] == odr)
        return 0;
    else
        acceld->accel_registers[accel_data_ctrl] = odr;

    /* Do not need to update DATA_CTRL_REG register if the sensor is not currently turn on */
    if(atomic_read(&acceld->accel_enabled) > 0) {
        err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, 0);
        if (err < 0) {
            KMSGERR(&acceld->client->dev, "Accel, %s, can not enter standby mode\n", __func__);
            return err;
        }

        err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_DATA_CTRL, acceld->accel_registers[accel_data_ctrl]);
        if (err < 0) {
            KMSGERR(&acceld->client->dev, "Accel, %s, Update ODR fail\n", __func__);
            return err;
        }

        err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, (acceld->accel_registers[accel_ctrl_reg1] | ACCEL_PC1_ON));
        if (err < 0) {
            KMSGERR(&acceld->client->dev, "Accel, %s, can not enter normal mode\n", __func__);
            return err;
        }
        msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);
    }
    return err;
}

static int kionix_accel_power_on(struct kionix_accel_driver *acceld)
{
    if (acceld->accel_pdata.power_on) {
        acceld->accel_pdata.power_on(&acceld->input_dev->dev);
        acceld->accel_pdata.power_on(&acceld->input_dev_ddtap->dev);
    }

    return 0;
}

static void kionix_accel_power_off(struct kionix_accel_driver *acceld)
{
    if (acceld->accel_pdata.power_off) {
        acceld->accel_pdata.power_off(&acceld->input_dev->dev);
        acceld->accel_pdata.power_off(&acceld->input_dev_ddtap->dev);
    }
}

static irqreturn_t kionix_accel_isr(int irq, void *dev)
{
    struct kionix_accel_driver *acceld = dev;

    disable_irq_nosync(irq);
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);
    if (unlikely(atomic_read(&acceld->accel_selftest_ongoing) == 1))
    {
        atomic_set(&acceld->accel_selftest_int, 1);
        wake_up_interruptible(&acceld->accel_selftest_wq);
        return IRQ_HANDLED;
    }
    acceld->tap_pre_tip = acceld->tap_tip;
    acceld->tap_tip = jiffies;
    queue_work(acceld->irq1_work_queue, &acceld->irq1_work);

    return IRQ_HANDLED;
}

static void kionix_accel_irq1_work_func(struct work_struct *work)
{

    struct kionix_accel_driver *acceld = container_of(work, struct kionix_accel_driver, irq1_work);
    /* TODO  add interrupt service procedure. ie:kionix_acc_get_int1_source(acceld); */
    int buf;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    kionix_accel_i2c_resume(acceld);

    if(atomic_read(&acceld->accel_enabled_int) > 0) {
        buf = kionix_accel_i2c_read_byte_data(acceld->client, ACCEL_INT_STATUS2);
        KMSGINF(&acceld->client->dev, "Accel, Interrupt status 2 buf %d\n", buf);
        if((buf & 0x0C) == ACCEL_INT_SINGLE_TAP) {
            /* only tap in the top of z axis is enable */
            if (acceld->accel_pdata.accel_tap_z_direction == 0) {
                /* sensor is same direction with phone layout */
                if (acceld->negate_z == 0) {
                    buf = kionix_accel_i2c_read_byte_data(acceld->client, ACCEL_INT_STATUS1);
                    KMSGINF(&acceld->client->dev, "Accel, z layout is 0, Interrupt status 1 buf %d\n", buf);
                    if((buf & 0x03) == ACCEL_INT_TAP_Z_NEGATIVE) {
                        acceld->tap_pre_direction = acceld->tap_direction;
                        acceld->tap_direction = TAP_NEGATIVE;
                    }
                    else {
                        acceld->tap_pre_direction = acceld->tap_direction;
                        acceld->tap_direction = TAP_NO_DIRECTION;
                    }
                }
                /* sensor is reverse direction with phone layout */
                else {
                    buf = kionix_accel_i2c_read_byte_data(acceld->client, ACCEL_INT_STATUS1);
                    KMSGINF(&acceld->client->dev, "Accel, z layout is 1, Interrupt status 1 buf %d\n", buf);
                    if((buf & 0x03) == ACCEL_INT_TAP_Z_POSITIVE) {
                        acceld->tap_pre_direction = acceld->tap_direction;
                        acceld->tap_direction = TAP_POSITIVE;
                    }
                    else {
                        acceld->tap_pre_direction = acceld->tap_direction;
                        acceld->tap_direction = TAP_NO_DIRECTION;
                    }
                }
            }
            /* tap in the top or bottom of z axis are enable */
            else {
                /* double tap must same diretion */
                if (acceld->accel_pdata.accel_dtap_z_direction == 0) {
                    buf = kionix_accel_i2c_read_byte_data(acceld->client, ACCEL_INT_STATUS1);
                    KMSGINF(&acceld->client->dev, "Accel, do not care z layout, double tap both 0 or 1, Interrupt status 1 buf %d\n", buf);
                    if((buf & 0x03) == ACCEL_INT_TAP_Z_NEGATIVE) {
                        acceld->tap_pre_direction = acceld->tap_direction;
                        acceld->tap_direction = TAP_NEGATIVE;
                    }
                    else if((buf & 0x03) == ACCEL_INT_TAP_Z_POSITIVE) {
                        acceld->tap_pre_direction = acceld->tap_direction;
                        acceld->tap_direction = TAP_POSITIVE;
                    }
                    else {
                        acceld->tap_pre_direction = acceld->tap_direction;
                        acceld->tap_direction = TAP_NO_DIRECTION;
                    }
                }
                /* double tap come from any two direction of z axis */
                else {
                    KMSGINF(&acceld->client->dev, "Accel, do not care z layout, double tap any direction\n");
                    acceld->tap_pre_direction = TAP_POSITIVE;
                    acceld->tap_direction = TAP_POSITIVE;
                }
            }
            if((acceld->tap_direction == acceld->tap_pre_direction) && (acceld->tap_direction != TAP_NO_DIRECTION)) {
                acceld->tap_time_ms = jiffies_to_msecs(acceld->tap_tip - acceld->tap_pre_tip);
                KMSGINF(&acceld->client->dev, "Accel, double tap time %dms\n", acceld->tap_time_ms);
                if((acceld->tap_time_ms < 150) || (acceld->tap_time_ms > 300))
                    goto exit;

                acceld->tap_tip = acceld->tap_init_tip;
                input_report_rel(acceld->input_dev_ddtap, DOUBLE_TAP_INTERRUPT, 1);
                input_sync(acceld->input_dev_ddtap);
                KMSGINF(&acceld->client->dev, "Accel, double tap int response and report\n");
            }
        }
    }
exit:    /* exit: */
    buf = kionix_accel_i2c_read_byte_data(acceld->client, ACCEL_INT_REL);
    if (buf < 0)
        KMSGERR(&acceld->client->dev, "Accel, %s, clear interrupt error\n", __func__);
    enable_irq(acceld->client->irq);
}

static void kionix_accel_work(struct work_struct *work)
{
    struct kionix_accel_driver *acceld = container_of((struct delayed_work *)work,    struct kionix_accel_driver, accel_work);
    queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, acceld->poll_delay);
    kionix_accel_i2c_resume(acceld);
    kionix_accel_report_accel_data(acceld);
}

static void kionix_accel_update_direction(struct kionix_accel_driver *acceld)
{
    unsigned int direction = acceld->accel_pdata.accel_direction;

    write_lock(&acceld->rwlock_accel_data);
    acceld->axis_map_x = ((direction-1)%2);
    acceld->axis_map_y =  (direction%2);
    acceld->axis_map_z =  2;
    acceld->negate_z = ((direction-1)/4);
    acceld->negate_x =  ((direction/2)%2);
    acceld->negate_y = (((direction+1)/4)%2);

    write_unlock(&acceld->rwlock_accel_data);
    return;
}

/* current consumption */
/* normal mode(phone is normal): 130ua*/
/* double tap mode(phone is glance or sleep): 130ua */
/* low power mode(phone is glance or sleep, but tap function is off): 2ua */

static int kionix_accel_enable(struct kionix_accel_driver *acceld)
{
    int err = -1;

    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    if(atomic_read(&acceld->accel_enabled_int) == 0) {
        err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, (acceld->accel_registers[accel_ctrl_reg1] | ACCEL_PC1_ON));
        if (err < 0) {
            KMSGERR(&acceld->client->dev, "Accel, %s, can not enter normal mode\n", __func__);
            return err;
        }
        msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);
    }
    queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);
    atomic_inc(&acceld->accel_enabled);
    KMSGINF(&acceld->client->dev, "Accel, sensor enter polling mode\n");

    return err;
}

static int kionix_accel_disable(struct kionix_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    if(atomic_read(&acceld->accel_enabled) > 0){
        if(atomic_dec_and_test(&acceld->accel_enabled)) {
            cancel_delayed_work(&acceld->accel_work);
            KMSGINF(&acceld->client->dev, "Accel, sensor exit polling mode\n");
        }
        if(atomic_read(&acceld->accel_enabled_int) == 0) {
           err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, 0);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, can not enter standby mode\n", __func__);
                return err;
            }
            KMSGINF(&acceld->client->dev, "Accel, sensor no polling and isr mode\n");
        }
    }
    return err;
}

static int kionix_accel_enable_int(struct kionix_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, 0);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, can not enter standby mode\n", __func__);
        return err;
    }

    acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_TDTE;
    err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, (acceld->accel_registers[accel_ctrl_reg1] | ACCEL_PC1_ON));
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, can not enter normal mode\n", __func__);
        return err;
    }
    msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);

    atomic_inc(&acceld->accel_enabled_int);
    KMSGINF(&acceld->client->dev, "Accel, sensor enter isr mode\n");
    enable_irq(acceld->client->irq);

    return err;
}

static int kionix_accel_disable_int(struct kionix_accel_driver *acceld)
{
    u8 backup;
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    if(atomic_read(&acceld->accel_enabled_int) > 0){
        if(atomic_dec_and_test(&acceld->accel_enabled_int)) {
            disable_irq_nosync(acceld->client->irq);
            backup = kionix_accel_i2c_read_byte_data(acceld->client, ACCEL_CTRL_REG1);

            err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, 0);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, can not enter standby mode\n", __func__);
                return err;
            }

            acceld->accel_registers[accel_ctrl_reg1] &= ~ACCEL_TDTE;
            backup &= ~ACCEL_TDTE;
            err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, backup);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, can not diable tap interrupt\n", __func__);
                return err;
            }
            msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);
            KMSGINF(&acceld->client->dev, "Accel, sensor exit isr mode\n");
        }
        if(atomic_read(&acceld->accel_enabled) == 0) {
            err = kionix_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, 0);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, can not enter standby mode\n", __func__);
                return err;
            }
            KMSGINF(&acceld->client->dev, "Accel, sensor no polling and isr mode\n");
        }
    }
    return err;
}

static int kionix_accel_input_open(struct input_dev *input)
{
    struct kionix_accel_driver *acceld = input_get_drvdata(input);
    atomic_inc(&acceld->accel_input_event);
    return 0;
}

static void kionix_accel_input_close(struct input_dev *dev)
{
    struct kionix_accel_driver *acceld = input_get_drvdata(dev);
    atomic_dec(&acceld->accel_input_event);
}

static int kionix_accel_input_ddtap_open(struct input_dev *input)
{
    return 0;
}

static void kionix_accel_input_ddtap_close(struct input_dev *dev)
{
    ;
}

static int __devinit kionix_accel_setup_input_device(struct kionix_accel_driver *acceld)
{
    struct input_dev *input_dev, *input_dev_ddtap;
    int err;

    input_dev = input_allocate_device();
    if (!input_dev) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to allocate input device for poll.\n", __func__);
        return -ENOMEM;
    }
    acceld->input_dev = input_dev;
    input_dev->open = kionix_accel_input_open;
    input_dev->close = kionix_accel_input_close;
    input_set_drvdata(input_dev, acceld);
    __set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_X, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
    input_set_abs_params(input_dev, ABS_Y, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
    input_set_abs_params(input_dev, ABS_Z, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
    input_dev->name = "accel";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &acceld->client->dev;
    err = input_register_device(acceld->input_dev);
    if (err) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to register input device. returned err = %d\n", __func__, err);
        input_free_device(acceld->input_dev);
        return err;
    }

    if(acceld->accel_ddtap == 1) {
        input_dev_ddtap = input_allocate_device();
        if (!input_dev_ddtap) {
            KMSGERR(&acceld->client->dev, "Accel, %s, failed to allocate input device for ddtap.\n", __func__);
            return -ENOMEM;
        }
        acceld->input_dev_ddtap = input_dev_ddtap;
        input_dev_ddtap->open = kionix_accel_input_ddtap_open;
        input_dev_ddtap->close = kionix_accel_input_ddtap_close;
        input_set_drvdata(input_dev_ddtap, acceld);
        __set_bit(REL_WHEEL, input_dev_ddtap->relbit);
        input_set_capability(input_dev_ddtap, EV_REL, DOUBLE_TAP_INTERRUPT);
        input_dev_ddtap->name = "ddtap";
        input_dev_ddtap->id.bustype = BUS_I2C;
        input_dev_ddtap->dev.parent = &acceld->client->dev;
        err = input_register_device(acceld->input_dev_ddtap);
        if (err) {
            KMSGERR(&acceld->client->dev, "Accel, %s, failed to register ddtap device. returned err = %d\n", __func__, err);
            input_free_device(acceld->input_dev_ddtap);
            return err;
        }
    }
    return 0;
}

static int kionix_accel_selftest_get_data( struct kionix_accel_driver* acceld, struct kionix_accel_axis_data* data )
{
    int kionix_accel_sample_count = 0;
    int err = 0, loop = 0;
    struct { union {
        s16 accel_data_s16[3];
        s8  accel_data_s8[6];
    }; } accel_data;
    s16 x, y, z;

    while( kionix_accel_sample_count < KIONIX_ACCEL_SELFTEST_SAMPLES )
    {
        kionix_accel_sample_count++;
        loop = KIONIX_I2C_RETRY_COUNT;
        while(loop) {
            err = kionix_i2c_read(acceld->client, ACCEL_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
            if(err < 0) {
                loop--;
                mdelay(KIONIX_I2C_RETRY_TIMEOUT);
            } else
                loop = 0;
        }
        if (err < 0) {
            goto Exit;
        } else {
            x = ((s16) le16_to_cpu(accel_data.accel_data_s16[0]));
            y = ((s16) le16_to_cpu(accel_data.accel_data_s16[1]));
            z = ((s16) le16_to_cpu(accel_data.accel_data_s16[2]));
        }
        data->x_axis += x;
        data->y_axis += y;
        data->z_axis += z;
        mdelay(KIONIX_ACCEL_SAMPLE_READY_TIME_MS);
    }
Exit:
    return err;
}

#define I2C_WRITE_ERROR_CHECK(reg, data)    do { \
                                                int loop = KIONIX_I2C_RETRY_COUNT, err; \
                                                while(loop) { \
                                                    err = i2c_smbus_write_byte_data(acceld->client, (reg), (data)); \
                                                    if (err < 0) { \
                                                        loop--; mdelay(KIONIX_I2C_RETRY_TIMEOUT); \
                                                    } else loop = 0; \
                                                } \
                                                if (err < 0) goto i2c_error_exit; \
                                            } while(0)
#define I2C_READ_ERROR_CHECK(reg, data)     do { \
                                                int loop = KIONIX_I2C_RETRY_COUNT, err; \
                                                while(loop) { \
                                                    err = i2c_smbus_read_byte_data(acceld->client, (reg)); \
                                                    if (err < 0) { \
                                                        loop--; mdelay(KIONIX_I2C_RETRY_TIMEOUT); \
                                                    } else loop = 0; \
                                                } \
                                                if (err < 0) goto i2c_error_exit; \
                                                else data = (u8)err; \
                                            } while(0)

static ssize_t kionix_accel_selftest_run_and_get_data(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev* input_dev = acceld->input_dev;

    struct ACCEL_SELFTEST_RESULT* kionix_accel_selftest_result = (struct ACCEL_SELFTEST_RESULT*)buf;
    struct kionix_accel_axis_data kionix_accel_selftest_data = { 0 };
    struct kionix_accel_axis_data kionix_accel_normal_data = { 0 };
    u8* kionix_accel_registers_backup = kzalloc(sizeof(u8)*accel_regs_count, GFP_KERNEL);
    u8 temp_reg;

    KMSGINF(&acceld->client->dev, "kionix_accel_selftest_run_and_get_data -> Enter\n");
    if (kionix_accel_registers_backup == NULL) {
        KMSGERR(&client->dev, "failed to allocate memory for accel_registers. Abort.\n");
        goto Exit_1;
    }
    memset(kionix_accel_selftest_result, 0, sizeof(*kionix_accel_selftest_result));

    kionix_accel_i2c_resume(acceld);

    mutex_lock(&input_dev->mutex);

    if (atomic_read(&acceld->accel_enabled_int) > 0) {
        /* IRQ is already enabled */
        /* Disable IRQ first */
        disable_irq(acceld->client->irq);
    }
    I2C_READ_ERROR_CHECK(ACCEL_INT_REL, temp_reg); //Clear interrupt first
    I2C_READ_ERROR_CHECK(ACCEL_CTRL_REG1, kionix_accel_registers_backup[accel_ctrl_reg1]);
    I2C_READ_ERROR_CHECK(ACCEL_DATA_CTRL, kionix_accel_registers_backup[accel_data_ctrl]);
    I2C_READ_ERROR_CHECK(ACCEL_INT_CTRL1, kionix_accel_registers_backup[accel_int_ctrl]);
    I2C_READ_ERROR_CHECK(ACCEL_INT_CTRL4, kionix_accel_registers_backup[accel_int_ctrl4]);

    /* Init KIONIX022 */
    I2C_WRITE_ERROR_CHECK(ACCEL_CTRL_REG1, 0);
    msleep(KIONIX_ACCEL_ACTIVE_TO_STANDBY_TIME_MS);
    I2C_WRITE_ERROR_CHECK(ACCEL_INT_CTRL1, 0x02); /* Positive selftest */
    I2C_WRITE_ERROR_CHECK(ACCEL_DATA_CTRL, ACCEL_ODR1600);
    I2C_WRITE_ERROR_CHECK(ACCEL_INT_CTRL4, 0);
    I2C_WRITE_ERROR_CHECK(ACCEL_CTRL_REG1, ACCEL_PC1_ON|ACCEL_G_2G|ACCEL_RES_16BIT);
    msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);

    /* Read Normal Data */
    KMSGINF(&acceld->client->dev, "kionix_accel, read normal data\n");
    if ( kionix_accel_selftest_get_data( acceld, &kionix_accel_normal_data) < 0 )
    {
        goto i2c_error_exit;
    }
    /* Start SelfTest */
    I2C_WRITE_ERROR_CHECK(ACCEL_SELFTEST_REG, 0xCA);
    msleep(KIONIX_ACCEL_SELFTEST_START_TIME_MS);
    KMSGINF(&acceld->client->dev, "kionix_accel, selftest start\n");

    /* Read SelfTest Data */
    KMSGINF(&acceld->client->dev, "kionix_accel, read selftest data\n");
    if ( kionix_accel_selftest_get_data( acceld, &kionix_accel_selftest_data) < 0 )
    {
        goto i2c_error_exit;
    }

    /* Stop SelfTest */
    I2C_WRITE_ERROR_CHECK(ACCEL_SELFTEST_REG, 0);
    msleep(KIONIX_ACCEL_SELFTEST_STOP_TIME_MS);
    KMSGINF(&acceld->client->dev, "kionix_accel, selftest stop\n");

    /* Compare */
    {
        int selftest_x_high, selftest_x_low, selftest_y_high, selftest_y_low, selftest_z_high, selftest_z_low;

        selftest_x_high = KIONIX_ACCEL_SELFTEST_X_HIGH_POSITIVE;
        selftest_x_low  = KIONIX_ACCEL_SELFTEST_X_LOW_POSITIVE;
        selftest_y_high = KIONIX_ACCEL_SELFTEST_Y_HIGH_POSITIVE;
        selftest_y_low  = KIONIX_ACCEL_SELFTEST_Y_LOW_POSITIVE;
        selftest_z_high = KIONIX_ACCEL_SELFTEST_Z_HIGH_POSITIVE;
        selftest_z_low  = KIONIX_ACCEL_SELFTEST_Z_LOW_POSITIVE;

        /* Average the normal data samples */
        kionix_accel_normal_data.x_axis /= KIONIX_ACCEL_SELFTEST_SAMPLES;
        kionix_accel_normal_data.y_axis /= KIONIX_ACCEL_SELFTEST_SAMPLES;
        kionix_accel_normal_data.z_axis /= KIONIX_ACCEL_SELFTEST_SAMPLES;
        /* Average the selftest data samples */
        kionix_accel_selftest_data.x_axis /= KIONIX_ACCEL_SELFTEST_SAMPLES;
        kionix_accel_selftest_data.y_axis /= KIONIX_ACCEL_SELFTEST_SAMPLES;
        kionix_accel_selftest_data.z_axis /= KIONIX_ACCEL_SELFTEST_SAMPLES;
        /* Check output change range */
        kionix_accel_selftest_result->x_axis_result = kionix_accel_selftest_data.x_axis - kionix_accel_normal_data.x_axis;
        KMSGINF(&acceld->client->dev, "kionix_accel, selftest, normal_x_axis: %d, selftest_x_axis: %d, output_change: %d\n", kionix_accel_normal_data.x_axis, kionix_accel_selftest_data.x_axis, kionix_accel_selftest_result->x_axis_result);
        if ( ( kionix_accel_selftest_result->x_axis_result > selftest_x_high ) ||
             ( kionix_accel_selftest_result->x_axis_result < selftest_x_low ) )
        {
            kionix_accel_selftest_result->x_axis_pass = false;
        }
        else
        {
            kionix_accel_selftest_result->x_axis_pass = true;
        }
        kionix_accel_selftest_result->y_axis_result = kionix_accel_selftest_data.y_axis - kionix_accel_normal_data.y_axis;
        KMSGINF(&acceld->client->dev, "kionix_accel, selftest, normal_y_axis: %d, selftest_y_axis: %d, output_change: %d\n", kionix_accel_normal_data.y_axis, kionix_accel_selftest_data.y_axis, kionix_accel_selftest_result->y_axis_result);
        if ( ( kionix_accel_selftest_result->y_axis_result > selftest_y_high ) ||
             ( kionix_accel_selftest_result->y_axis_result < selftest_y_low ) )
        {
            kionix_accel_selftest_result->y_axis_pass = false;
        }
        else
        {
            kionix_accel_selftest_result->y_axis_pass = true;
        }
        kionix_accel_selftest_result->z_axis_result = kionix_accel_selftest_data.z_axis - kionix_accel_normal_data.z_axis;
        KMSGINF(&acceld->client->dev, "kionix_accel, selftest, normal_z_axis: %d, selftest_z_axis: %d, output_change: %d\n", kionix_accel_normal_data.z_axis, kionix_accel_selftest_data.z_axis, kionix_accel_selftest_result->z_axis_result);
        if ( ( kionix_accel_selftest_result->z_axis_result > selftest_z_high ) ||
             ( kionix_accel_selftest_result->z_axis_result < selftest_z_low ) )
        {
            kionix_accel_selftest_result->z_axis_pass = false;
        }
        else
        {
            kionix_accel_selftest_result->z_axis_pass = true;
        }
    }

    /* Test Interrupt */
    KMSGINF(&acceld->client->dev, "kionix_accel, selftest, test interrupt\n");
    kionix_accel_selftest_result->interrupt_pin_support = true;
    kionix_accel_selftest_result->interrupt_pin_status = false;
    if (kionix_accel_selftest_result->interrupt_pin_support == true)
    {
        I2C_READ_ERROR_CHECK(ACCEL_INT_REL, temp_reg); /*Clear interrupt first*/
        enable_irq(acceld->client->irq); /*Enable interrupt*/
        atomic_set(&acceld->accel_selftest_int, 0);
        atomic_set(&acceld->accel_selftest_ongoing, 1);
        I2C_WRITE_ERROR_CHECK(ACCEL_CTRL_REG1, 0);
        msleep(KIONIX_ACCEL_ACTIVE_TO_STANDBY_TIME_MS);
        I2C_WRITE_ERROR_CHECK(ACCEL_INT_CTRL1, 0x30); /* Enable physical interrupt */
        I2C_WRITE_ERROR_CHECK(ACCEL_DATA_CTRL, ACCEL_ODR200); /* Interrupt will come after about 5ms */
        I2C_WRITE_ERROR_CHECK(ACCEL_INT_CTRL4, 0x10); /* DRDY report on pin1 */
        I2C_WRITE_ERROR_CHECK(ACCEL_CTRL_REG1, ACCEL_PC1_ON|ACCEL_G_2G|ACCEL_RES_16BIT|ACCEL_DRDYE); /* Enable DRDY */
        msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);
        wait_event_interruptible_timeout(acceld->accel_selftest_wq, atomic_read(&acceld->accel_selftest_int),
                                         msecs_to_jiffies(100));
        if (atomic_read(&acceld->accel_selftest_int) == 1) {
            I2C_READ_ERROR_CHECK(ACCEL_INT_STATUS2, temp_reg); /*Read interrupt source */
            if (temp_reg & 0x10) {
                kionix_accel_selftest_result->interrupt_pin_status = true;
                KMSGINF(&acceld->client->dev, "kionix_accel, selftest, get DRDY interrupt\n");
            }
            I2C_READ_ERROR_CHECK(ACCEL_INT_REL, temp_reg); /*Clear interrupt */
        }
        atomic_set(&acceld->accel_selftest_ongoing, 0);
    }

    /* Write back the restore registers */
    I2C_WRITE_ERROR_CHECK(ACCEL_CTRL_REG1, 0);
    msleep(KIONIX_ACCEL_ACTIVE_TO_STANDBY_TIME_MS);
    I2C_WRITE_ERROR_CHECK(ACCEL_DATA_CTRL, kionix_accel_registers_backup[accel_data_ctrl]);
    I2C_WRITE_ERROR_CHECK(ACCEL_INT_CTRL1, kionix_accel_registers_backup[accel_int_ctrl]);
    I2C_WRITE_ERROR_CHECK(ACCEL_INT_CTRL4, kionix_accel_registers_backup[accel_int_ctrl4]);
    I2C_WRITE_ERROR_CHECK(ACCEL_CTRL_REG1, kionix_accel_registers_backup[accel_ctrl_reg1]);
    msleep(KIONIX_ACCEL_STANDBY_TO_ACTIVE_TIME_MS);
    if (atomic_read(&acceld->accel_enabled_int) > 0) {
        /* IRQ was enabled previously */
        /* Re-enable IRQ */
        enable_irq(acceld->client->irq);
    }

    /* I2C status is true */
    kionix_accel_selftest_result->i2c_status = true;
    if (kionix_accel_registers_backup != NULL) {
        kfree(kionix_accel_registers_backup);
    }

    mutex_unlock(&input_dev->mutex);
    KMSGINF(&acceld->client->dev, "kionix_accel_selftest_run_and_get_data -> Success Exit\n");
    return sizeof(struct ACCEL_SELFTEST_RESULT);

Exit_1:
    kionix_accel_selftest_result->interrupt_pin_support = true;
    kionix_accel_selftest_result->interrupt_pin_status = false;
    kionix_accel_selftest_result->x_axis_pass = false;
    kionix_accel_selftest_result->y_axis_pass = false;
    kionix_accel_selftest_result->z_axis_pass = false;
i2c_error_exit:
    kionix_accel_selftest_result->i2c_status = false;
    if (kionix_accel_registers_backup != NULL) {
        kfree(kionix_accel_registers_backup);
    }

    mutex_unlock(&input_dev->mutex);
    KMSGINF(&acceld->client->dev, "kionix_accel_selftest_run_and_get_data -> Error Exit\n");
    return sizeof(struct ACCEL_SELFTEST_RESULT);
}

static void kionix_accel_test(struct work_struct *test)
{
    struct kionix_accel_driver *acceld = container_of((struct delayed_work *)test, struct kionix_accel_driver, accel_test);
    s16 accel_data_s16[3];
    int err;
    kionix_accel_i2c_resume(acceld);
    err = kionix_i2c_read(acceld->client, ACCEL_XOUT_L, (u8 *)accel_data_s16, 6);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "TEST, %s, faild to read test data output\n", __func__);
    } else {
        KMSGINF(&acceld->client->dev, "TEST: RAW DATA1, x, 0x%.4x, y, 0x%.4x, z, 0x%.4x\n",
                le16_to_cpu(accel_data_s16[0]), le16_to_cpu(accel_data_s16[1]), le16_to_cpu(accel_data_s16[2]));
        KMSGINF(&acceld->client->dev, "TEST: RAW DATA2, x, 0x%.4x, y, 0x%.4x, z, 0x%.4x\n",
                (le16_to_cpu(accel_data_s16[0]) & MASK_16_10_BITS),
                (le16_to_cpu(accel_data_s16[1]) & MASK_16_10_BITS),
                (le16_to_cpu(accel_data_s16[2]) & MASK_16_10_BITS));
        KMSGINF(&acceld->client->dev, "TEST: RAW DATA3, x, %d, y, %d, z %d\n",
                ((s16)(le16_to_cpu(accel_data_s16[0]) & MASK_16_10_BITS)) >> 6,
                ((s16)(le16_to_cpu(accel_data_s16[1]) & MASK_16_10_BITS)) >> 6,
                ((s16)(le16_to_cpu(accel_data_s16[2]) & MASK_16_10_BITS)) >> 6);
    }
    queue_delayed_work(acceld->accel_testqueue, &acceld->accel_test, msecs_to_jiffies(500));
}


/* Returns the enable state of device */
static ssize_t kionix_accel_get_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", atomic_read(&acceld->accel_enabled) > 0 ? 1 : 0);
}

static ssize_t kionix_accel_get_enable_int(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", atomic_read(&acceld->accel_enabled_int) > 0 ? 1 : 0);
}

/* Allow users to enable/disable the device */
static ssize_t kionix_accel_set_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long enable;
    int err = 0;

    /* Lock the device to prevent races with open/close (and itself) */
    mutex_lock(&input_dev->mutex);
    err = strict_strtoul(buf, 10, &enable);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }

    kionix_accel_i2c_resume(acceld);

    if(enable)
        err = kionix_accel_enable(acceld);
    else
        err = kionix_accel_disable(acceld);

exit:
    mutex_unlock(&input_dev->mutex);

    return (err < 0) ? err : count;
}

static ssize_t kionix_accel_set_enable_int(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev_ddtap = acceld->input_dev_ddtap;
    unsigned long enable;
    int err = 0;

    /* Lock the device to prevent races with open/close (and itself) */
    mutex_lock(&input_dev_ddtap->mutex);
    err = strict_strtoul(buf, 10, &enable);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }

    kionix_accel_i2c_resume(acceld);

    if(enable)
        err = kionix_accel_enable_int(acceld);
    else
        err = kionix_accel_disable_int(acceld);

exit:
    mutex_unlock(&input_dev_ddtap->mutex);

    return (err < 0) ? err : count;
}


/* Returns currently selected poll interval (in ms) */
static ssize_t kionix_accel_get_delay(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", acceld->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kionix_accel_set_delay(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long interval;
    int err = 0;

    /* Lock the device to prevent races with open/close (and itself) */
    mutex_lock(&input_dev->mutex);
    err = strict_strtoul(buf, 10, &interval);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }
    if (interval == 0) {
        err = -EINVAL;
        KMSGERR(&acceld->client->dev, "Accel, %s, invalid interval = %d\n", __func__, (unsigned int) interval);
        goto exit;
    }

    /* Set current interval to the greater of the minimum interval or the requested interval */
    acceld->poll_interval = max((unsigned int)interval, acceld->accel_pdata.min_interval);
    acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);
    kionix_accel_i2c_resume(acceld);
    err = kionix_accel_update_odr(acceld, acceld->poll_interval);
    if(err < 0) {
        KMSGERR(&acceld->client->dev, "%s: Accel, failed to update ODR.\n", __func__);
    }

exit:
    mutex_unlock(&input_dev->mutex);

    return (err < 0) ? err : count;
}

/* Returns the direction of device */
static ssize_t kionix_accel_get_direct(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", acceld->accel_pdata.accel_direction);
}

/* Allow users to change the direction the device */
static ssize_t kionix_accel_set_direct(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long direction;
    int err = 0;

    /* Lock the device to prevent races with open/close (and itself) */
    mutex_lock(&input_dev->mutex);
    err = strict_strtoul(buf, 10, &direction);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }
    if(direction < 1 || direction > 8) {
        err = -EINVAL;
        KMSGERR(&acceld->client->dev, "Accel, %s, invalid direction = %d\n", __func__, (unsigned int) direction);
        goto exit;
    } else {
        acceld->accel_pdata.accel_direction = (u8) direction;
        kionix_accel_update_direction(acceld);
    }

exit:
    mutex_unlock(&input_dev->mutex);

    return (err < 0) ? err : count;
}

/* Returns the data output of device */
static ssize_t kionix_accel_get_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    int x, y, z;

    read_lock(&acceld->rwlock_accel_data);

    x = acceld->accel_data[acceld->axis_map_x];
    y = acceld->accel_data[acceld->axis_map_y];
    z = acceld->accel_data[acceld->axis_map_z];

    read_unlock(&acceld->rwlock_accel_data);

    return sprintf(buf, "%d %d %d\n", x, y, z);
}

static ssize_t kionix_accel_get_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    u8 retval;
    char *bufloop;
    int buflen;

    bufloop = buf;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x06);
    buflen = sprintf(bufloop, "XOUTL..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x07);
    buflen = sprintf(bufloop, "XOUTH..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x08);
    buflen = sprintf(bufloop, "YOUTL..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x09);
    buflen = sprintf(bufloop, "YOUTH..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x0A);
    buflen = sprintf(bufloop, "ZOUTL..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x0B);
    buflen = sprintf(bufloop, "ZOUTH..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x0C);
    buflen = sprintf(bufloop, "COTR...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x0F);
    buflen = sprintf(bufloop, "WHO....... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x12);
    buflen = sprintf(bufloop, "INS1...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x13);
    buflen = sprintf(bufloop, "INS2...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x14);
    buflen = sprintf(bufloop, "INS3...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x15);
    buflen = sprintf(bufloop, "STAT...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x17);
    buflen = sprintf(bufloop, "INL....... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x18);
    buflen = sprintf(bufloop, "CNTL1..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x19);
    buflen = sprintf(bufloop, "CNTL2..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x1A);
    buflen = sprintf(bufloop, "CNTL3..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x1B);
    buflen = sprintf(bufloop, "ODCNTL.... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x1C);
    buflen = sprintf(bufloop, "INC1...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x1D);
    buflen = sprintf(bufloop, "INC2...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x1E);
    buflen = sprintf(bufloop, "INC3...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x1F);
    buflen = sprintf(bufloop, "INC4...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x20);
    buflen = sprintf(bufloop, "INC5...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x21);
    buflen = sprintf(bufloop, "INC6...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x24);
    buflen = sprintf(bufloop, "TDTRC..... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x25);
    buflen = sprintf(bufloop, "TDTC...... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x26);
    buflen = sprintf(bufloop, "TTH....... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x27);
    buflen = sprintf(bufloop, "TTL....... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x28);
    buflen = sprintf(bufloop, "FTD....... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x29);
    buflen = sprintf(bufloop, "STD....... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x2A);
    buflen = sprintf(bufloop, "TLT....... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x2B);
    buflen = sprintf(bufloop, "TWS....... 0x%.2x\n", retval); bufloop+=buflen;

    return bufloop-buf;
}

static ssize_t kionix_accel_set_test(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long test;
    int err = 0;

    /* Lock the device to prevent races with open/close (and itself) */
    mutex_lock(&input_dev->mutex);
    err = strict_strtoul(buf, 10, &test);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }
    if((test != 1) && (test != 0)) {
        err = -EINVAL;
        KMSGERR(&acceld->client->dev, "Accel, %s, invalid test = %d\n", __func__, (unsigned int) test);
        goto exit;
    }
    
    if(test == 1) {
        if(acceld->test == 0) {
            acceld->accel_testqueue = create_workqueue("Kionix Accel Testqueue");
            if (!acceld->accel_testqueue) {
                KMSGERR(&client->dev, "Accel, %s, failed to create test queue\n", __func__);
                err = -EPERM;
                goto exit;
            }
            INIT_DELAYED_WORK(&acceld->accel_test, kionix_accel_test);
            queue_delayed_work(acceld->accel_testqueue, &acceld->accel_test, 0);
        }
        acceld->test = 1;
    } else {
        if (acceld->test == 1) {
            cancel_delayed_work_sync(&acceld->accel_test);
            destroy_workqueue(acceld->accel_testqueue);
        }
        acceld->test = 0;
    }
exit:
    mutex_unlock(&input_dev->mutex);
    return (err < 0) ? err : count;
}

static ssize_t kionix_accel_get_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", acceld->test);
}

static DEVICE_ATTR(enable_poll,  S_IRUGO|S_IWUGO, kionix_accel_get_enable, kionix_accel_set_enable);
static DEVICE_ATTR(enable_int,  S_IRUGO|S_IWUGO, kionix_accel_get_enable_int, kionix_accel_set_enable_int);
static DEVICE_ATTR(delay,  S_IRUGO|S_IWUGO, kionix_accel_get_delay, kionix_accel_set_delay);
static DEVICE_ATTR(direct,  S_IRUGO|S_IWUGO, kionix_accel_get_direct, kionix_accel_set_direct);
static DEVICE_ATTR(data, S_IRUGO, kionix_accel_get_data, NULL);
static DEVICE_ATTR(selftest_data, S_IRUGO, kionix_accel_selftest_run_and_get_data, NULL);
static DEVICE_ATTR(test, S_IRUGO|S_IWUGO, kionix_accel_get_test, kionix_accel_set_test);
static DEVICE_ATTR(getreg,  S_IRUGO, kionix_accel_get_reg, NULL);

static struct attribute *kionix_accel_attributes[] = {
    &dev_attr_enable_poll.attr,
    &dev_attr_enable_int.attr,
    &dev_attr_delay.attr,
    &dev_attr_direct.attr,
    &dev_attr_data.attr,
    &dev_attr_selftest_data.attr,
    &dev_attr_test.attr,
    &dev_attr_getreg.attr,
    NULL
};

static struct attribute_group kionix_accel_attribute_group = {
    .attrs = kionix_accel_attributes
};

static int __devinit kionix_hw_init(struct kionix_accel_driver *acceld)
{
    int retval = kionix_accel_i2c_read_byte_data(acceld->client, ACCEL_WHO_AM_I);

    if(retval != KIONIX_ACCEL_WHO_AM_I_KX022) {
        KMSGERR(&acceld->client->dev, "Accel, %s, accelerometer is not KX022.\n", __func__);
        return -ENODEV;
    }
    KMSGINF(&acceld->client->dev, "Accel, %s, accelerometer is KX022.\n", __func__);

    switch(acceld->accel_pdata.accel_res) {
        case KIONIX_ACCEL_RES_6BIT:
        case KIONIX_ACCEL_RES_8BIT:
            acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_RES_8BIT;
            break;
        case KIONIX_ACCEL_RES_12BIT:
        case KIONIX_ACCEL_RES_16BIT:
        default:
            acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_RES_16BIT;
            break;
    }
    kionix_accel_update_g_range(acceld);
    acceld->poll_interval = acceld->accel_pdata.poll_interval;
    acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);
    retval = kionix_accel_update_odr(acceld, acceld->poll_interval);
    if(retval < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, hardware init, config ODR fail.\n", __func__);
        return retval;
    }

    kionix_accel_update_direction(acceld);

    return retval;
}

static int __devinit kionix_accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct kionix_accel_platform_data *accel_pdata;
    struct kionix_accel_driver *acceld;
    int err;

    /* 1. check i2c client */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
        KMSGERR(&client->dev, "Accel, %s, client is not i2c capable. Abort.\n", __func__);
        return -ENXIO;
    }
    KMSGINF(&client->dev, "Accel, %s, client is i2c capable. OK.\n", __func__);

    /* 2. check config node, then ready memory to get platform struct data */
    if (client->dev.of_node) {
        accel_pdata = devm_kzalloc(&client->dev, sizeof(struct kionix_accel_platform_data), GFP_KERNEL);
        if (!accel_pdata) {
            KMSGERR(&client->dev, "Accel, %s, failed to allocate memory for platform data. Abort.\n", __func__);
            return -ENOMEM;
        }
    } else {
        KMSGERR(&client->dev, "Accel, %s, failed to get config node info. Abort.\n", __func__);
        return -EINVAL;
    }
    KMSGINF(&client->dev, "Accel, %s, allocate memory for platform data. OK.\n", __func__);

    /* 3. ready driver struct memory */
    acceld = kzalloc(sizeof(*acceld), GFP_KERNEL);
    if (acceld == NULL) {
        KMSGERR(&client->dev, "Accel, %s, failed to allocate memory for module data. Abort.\n", __func__);
        return -ENOMEM;
    }
    KMSGINF(&client->dev, "Accel, %s, allocate memory for module data. OK.\n", __func__);

    /* 4. get platform data form config node file */
    err =  kionix_acc_parse_dt(&client->dev, accel_pdata);
    if (err) {
        KMSGERR(&client->dev, "Accel, %s, failed to parse device tree\n", __func__);
        goto err_free_mem;
    }
    KMSGINF(&client->dev, "Accel, %s, parse device tree. OK.\n", __func__);

    /* 5. connect private device info into client. */
    acceld->client = client;
    acceld->accel_pdata = *accel_pdata;
    i2c_set_clientdata(client, acceld);

    /* 6. setup input device */
    if(acceld->accel_pdata.accel_irq_use_ddtap && client->irq) {
        acceld->accel_ddtap = 1;
    }
    else
        acceld->accel_ddtap = 0;

    err = kionix_accel_setup_input_device(acceld);
    if (err) {
        KMSGERR(&client->dev, "Accel, %s, failed to setup input device. Abort\n", __func__);
        goto err_free_mem;
    }

    /* 7. ready some init interface for spare action */
    err = kionix_accel_power_on(acceld);
    if (err < 0)
        goto err_free_device_register;

    if (accel_pdata->init) {
        err = accel_pdata->init(&(acceld->input_dev->dev));
        if (err < 0)
            goto err_accel_pdata_power_off;
        err = accel_pdata->init(&(acceld->input_dev_ddtap->dev));
        if (err < 0)
            goto err_accel_pdata_power_off;
    }

    /* 8. Hw init and verify */
    atomic_set(&acceld->suspend, 0);

    acceld->accel_registers = kzalloc(sizeof(u8)*accel_regs_count, GFP_KERNEL);
    if (acceld->accel_registers == NULL) {
        KMSGERR(&client->dev, "Accel, %s, failed to allocate memory for accel_registers. Abort.\n", __func__);
        goto err_accel_pdata_exit;
    }

    rwlock_init(&acceld->rwlock_accel_data);
    err = kionix_hw_init(acceld);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to init hardware. Abort.\n", __func__);
        goto err_free_accel_registers;
    }
    KMSGINF(&client->dev, "Accel, %s, Device is correct. OK.\n", __func__);

    /* 9. if necessary, do INT init */
    if(acceld->accel_ddtap == 1) {
        acceld->accel_registers[accel_int_ctrl] |= ACCEL_IEN | ACCEL_IEA;
        acceld->accel_ddtap = 1;
        err = kionix_accel_int_init(acceld);
        if (err) {
            KMSGERR(&acceld->client->dev, "Accel, %s, failed to init INT. Abort.\n", __func__);
            goto err_free_accel_registers;
        }
    }
    KMSGINF(&client->dev, "Accel, %s, Int init success. OK.\n", __func__);

    atomic_set(&acceld->accel_enabled, 0);
    atomic_set(&acceld->accel_enabled_int, 0);
    atomic_set(&acceld->accel_input_event, 0);
    atomic_set(&acceld->accel_selftest_int, 0);
    atomic_set(&acceld->accel_selftest_ongoing, 0);
    init_waitqueue_head(&acceld->accel_selftest_wq);

    acceld->proc_dir = proc_mkdir("sensors", NULL);
    if (acceld->proc_dir == NULL) {
        KMSGERR(&client->dev, "Accel, %s, failed to create /proc/sensors. Abort.\n", __func__);
        goto err_free_accel_registers;
    }
    else {
        acceld->proc_entry = create_proc_entry( "accelinfo", 0644, acceld->proc_dir);
        if (acceld->proc_entry == NULL) {
            KMSGERR(&client->dev, "Accel, %s, failed to create /proc/cpu/accelinfo. Abort.\n", __func__);
            goto err_free_proc_direction;
        }
    }

    /* 10. init work queue and irq, irq work queue */
    acceld->accel_workqueue = create_workqueue("Kionix Accel Workqueue");
    if (!acceld->accel_workqueue) {
        KMSGERR(&client->dev, "Accel, %s, failed to create work queue. Abort.\n", __func__);
        goto err_free_proc_file;
    }
    INIT_DELAYED_WORK(&acceld->accel_work, kionix_accel_work);

    if (acceld->accel_ddtap) {
        INIT_WORK(&acceld->irq1_work, kionix_accel_irq1_work_func);
        acceld->irq1_work_queue = create_singlethread_workqueue("Kionix Accel Intqueue");
        if (!acceld->irq1_work_queue) {
            KMSGERR(&client->dev, "Accel, %s, failed to create int work queue. Abort.\n", __func__);
            goto err_free_workqueue;
        }
        err = request_irq(client->irq, kionix_accel_isr, IRQF_TRIGGER_HIGH, "kionix_acc_irq1", acceld); /* IRQF_ONESHOT */
        if (err < 0) {
            KMSGERR(&client->dev, "Accel, %s, failed to request irq returned err = %d. Abort.\n", __func__, err);
            goto err_free_irq_workqueue;
        }
        disable_irq_nosync(client->irq);
        enable_irq_wake(client->irq);    /* enable wake up phone function when is in the sleep */
    }
    KMSGINF(&client->dev, "Accel, %s, Init work queue and IRQ init, IRQ work queue success. OK.\n", __func__);

    /* 11. create file system for device */
    err = sysfs_create_group(&client->dev.kobj, &kionix_accel_attribute_group);
    if (err) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to create sysfs returned err = %d. Abort.\n", __func__, err);
        goto err_free_irq;
    }

    KMSGINF(&client->dev, "Accel, %s, Crete file system. OK.\n", __func__);

    err = kionix_accel_power_on_init(acceld);
    if (err) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to power on. Abort.\n", __func__);
        goto err_free_group;
    }
    KMSGINF(&client->dev, "Accel, %s, KIONIX sensor Probe Done. OK.\n", __func__);

    acceld->test = 0;
    acceld->tap_tip = jiffies;
    acceld->tap_init_tip = acceld->tap_tip;
    acceld->tap_direction = TAP_NO_DIRECTION;

    return 0;

err_free_group:
    sysfs_remove_group(&client->dev.kobj, &kionix_accel_attribute_group);
err_free_irq:
    if (acceld->accel_ddtap) {
        disable_irq_wake(client->irq);
        free_irq(client->irq, acceld);
    }
err_free_irq_workqueue:
    if (acceld->accel_ddtap)
        destroy_workqueue(acceld->irq1_work_queue);
err_free_workqueue:
    destroy_workqueue(acceld->accel_workqueue);
err_free_proc_file:
    remove_proc_entry("accelinfo", acceld->proc_dir);
err_free_proc_direction:
    remove_proc_entry("sensors", NULL);
err_free_accel_registers:
    kfree(acceld->accel_registers);
err_accel_pdata_exit:
    if (accel_pdata->exit)
        accel_pdata->exit();
err_accel_pdata_power_off:
    kionix_accel_power_off(acceld);
err_free_device_register:
    if (acceld->accel_ddtap)
        input_unregister_device(acceld->input_dev_ddtap);
    input_unregister_device(acceld->input_dev);
err_free_mem:
    kfree(acceld);
    return err;
}

static int kionix_accel_remove(struct i2c_client *client)
{
    struct kionix_accel_driver *acceld;
    acceld = i2c_get_clientdata(client);

    sysfs_remove_group(&client->dev.kobj, &kionix_accel_attribute_group);
    if (acceld->accel_ddtap) {
        disable_irq_wake(client->irq);    /* cancel wake up phone function when is in the sleep */
        free_irq(client->irq, acceld);
        destroy_workqueue(acceld->irq1_work_queue);
    }
    destroy_workqueue(acceld->accel_workqueue);

    remove_proc_entry("accelinfo", acceld->proc_dir);
    remove_proc_entry("sensors", NULL);
    kfree(acceld->accel_registers);
    if (acceld->accel_pdata.exit)
        acceld->accel_pdata.exit();
    kionix_accel_power_off(acceld);
    if (acceld->accel_ddtap)
        input_unregister_device(acceld->input_dev_ddtap);
    input_unregister_device(acceld->input_dev);
    kfree(acceld);

    return 0;
}

#ifdef CONFIG_PM
static int kionix_accel_resume(struct i2c_client *client)
{
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    KMSGINF(&client->dev, "%s\n", __func__);
    atomic_set(&acceld->suspend, ACCEL_I2C_RESUME);
    return 0;
}

static int kionix_accel_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
    KMSGINF(&client->dev, "%s\n", __func__);
    atomic_set(&acceld->suspend, ACCEL_I2C_SUSPEND);
    return 0;
}
#else
#define kionix_acc_suspend    NULL
#define kionix_acc_resume    NULL
#endif /* CONFIG_PM */


static const struct i2c_device_id kionix_accel_id[] = {
    { KIONIX_ACCEL_NAME, 0 },
    { },
};

MODULE_DEVICE_TABLE(i2c, kionix_accel_id);

static const struct of_device_id kionix_acc_of_match[] = {
    { .compatible = "kionix,kionix_acc", },
    { },
};

static struct i2c_driver kionix_accel_driver = {
    .driver = {
        .name    = KIONIX_ACCEL_NAME,
        .owner   = THIS_MODULE,
        .of_match_table = kionix_acc_of_match,
    },
    .probe       = kionix_accel_probe,
    .remove      = __devexit_p(kionix_accel_remove),
    .suspend     = kionix_accel_suspend,
    .resume      = kionix_accel_resume,
    .id_table    = kionix_accel_id,
};

static int __init kionix_accel_init(void)
{
    return i2c_add_driver(&kionix_accel_driver);
}
module_init(kionix_accel_init);

static void __exit kionix_accel_exit(void)
{
    i2c_del_driver(&kionix_accel_driver);
}
module_exit(kionix_accel_exit);

MODULE_DESCRIPTION("Kionix accelerometer driver");
MODULE_AUTHOR("Kuching Tan <kuchingtan@kionix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.3.0");

