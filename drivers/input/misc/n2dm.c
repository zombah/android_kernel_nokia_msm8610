/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name          : n2dm.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *                    : Matteo Dameno (matteo.dameno@st.com)
 *                    : Denis Ciocca (denis.ciocca@st.com)
 *                    : Both authors are willing to be considered the contact
 *                    : and update points for the driver.
 * Version            : V.1.0.12
 * Date               : 2013/Nov/12
 * Description        : N2DM accelerometer driver sensor
 *
 *******************************************************************************/

#include        <linux/kernel.h>
#include        <linux/delay.h>
#include        <linux/i2c.h>
#include        <linux/input.h>
#include        <linux/interrupt.h>
#include        <linux/workqueue.h>
#include        <linux/module.h>
#include        <linux/slab.h>
#include        <linux/input/n2dm.h>
#include        <linux/version.h>
#include        <linux/proc_fs.h>
#include        <linux/of_device.h>
#include        <linux/sched.h>

/* Debug Message Flags */
#define N2DM_KMSG_ERR    1    /* Print kernel debug message for error */
#define N2DM_KMSG_INF    1    /* Print kernel debug message for info */

#if N2DM_KMSG_ERR
#define KMSGERR(format, ...)    dev_err(format, ## __VA_ARGS__)
#else
#define KMSGERR(format, ...)
#endif

#if N2DM_KMSG_INF
#define KMSGINF(format, ...)    dev_info(format, ## __VA_ARGS__)
#else
#define KMSGINF(format, ...)
#endif

#define DOUBLE_TAP_INTERRUPT   REL_MISC

/******************************************************************************
 * Accelerometer WHO_AM_I return value
 *****************************************************************************/
#define ST_ACCEL_WHO_AM_I_N2DM             0x33

/*****************************************************************************/
/* Registers for Accelerometer */
/*****************************************************************************/
#define ACCEL_WHO_AM_I        (0x0F)
#define ACCEL_CTRL_REG1       (0x20)   /* config ODR, Low power, XYZ enable */
#define ACCEL_CTRL_REG2       (0x21)   /* config High-pass filter enable for CLICK function */
#define ACCEL_CTRL_REG3       (0x22)   /* config CLICK interrupt enable on INT1 pin */
#define ACCEL_CTRL_REG4       (0x23)   /* config Block data update, range, selftest */
#define ACCEL_CTRL_REG5       (0x24)   /* config INT latch */
#define ACCEL_CTRL_REG6       (0x25)   /* config INT2 */
#define ACCEL_XOUT_L          (0x28)

#define ACCEL_CLICK_CFG       (0x38)   /* config INT double tap or single tap enable on XYZ */
#define ACCEL_CLICK_SRC       (0x39)   /* status of click int */
#define ACCEL_CLICK_THS       (0x3A)   /* The threshold of tap */
#define ACCEL_TIME_LIMIT      (0x3B)
#define ACCEL_TIME_LATENCY    (0x3C)
#define ACCEL_TIME_WINDOW     (0x3D)

#define ALL_ZEROES            (0x00)

/* CTRL_REG1 */
#define ACCEL_ACC_PM_OFF          (0x00)
#define ACCEL_ACC_ENABLE_ALL_AXES (0x07)

#define ACCEL_ODR1            (0x10)
#define ACCEL_ODR10           (0x20)
#define ACCEL_ODR25           (0x30)
#define ACCEL_ODR50           (0x40)
#define ACCEL_ODR100          (0x50)
#define ACCEL_ODR200          (0x60)
#define ACCEL_ODR400          (0x70)
#define ACCEL_ODR1344         (0x90)
#define ACCEL_ODR1620_LPW     (0x88)
#define ACCEL_ODR5376_LPW     (0x98)
#define ACCEL_POWER_DOWN      (0x00)
/* CTRL_REG2 */
#define ACCEL_HPCLCK          (0x04)
/* CTRL_REG3 */
#define ACCEL_INT1_CLICK      (0x80)
#define ACCEL_INT1_DRDY       (0x10)
/* CTRL_REG4 */
#define ACCEL_BDU_ENABLE      (0x80)
#define ACCEL_G_2G            (0x00)
#define ACCEL_G_4G            (0x10)
#define ACCEL_G_8G            (0x20)
#define ACCEL_G_16G           (0x30)
#define ACCEL_ST0             (0x02)
#define ACCEL_ST1             (0x04)
/* CTRL_REG5 */
#define ACCEL_LATCH_INT1_NO   (0x00)
#define ACCEL_LATCH_INT1      (0x08)
#define ACCEL_LATCH_INT2_NO   (0x00)
#define ACCEL_LATCH_INT2      (0x02)
/* CTRL_REG6 */
#define ACCEL_INT2_CLICK      (0x80)
/* CLICK_CONFIG */
#define ACCEL_DOUBLE_Z_ENABLE (0x20)
#define ACCEL_SINGLE_Z_ENABLE (0x10)
#define ACCEL_DOUBLE_Y_ENABLE (0x08)
#define ACCEL_SINGLE_Y_ENABLE (0x04)
#define ACCEL_DOUBLE_X_ENABLE (0x02)
#define ACCEL_SINGLE_X_ENABLE (0x01)
/* CLICK_SRC */
#define ACCEL_INT_CLICK       (0x40)
#define ACCEL_DOUBLE_CLICK    (0x20)
#define ACCEL_SINGLE_CLICK    (0x10)
#define ACCEL_POSITIVE        (0x00)
#define ACCEL_NEGATIVE        (0x08)
#define ACCEL_INT_Z_CLICK     (0x04)
#define ACCEL_INT_Y_CLICK     (0x02)
#define ACCEL_INT_X_CLICK     (0x01)

/* TUNING_PARAM */
#define CLICK_THRESHOLD       (50)     /* (50 * 2G / 128) = 780mg */
#define CLICK_TIME_LIMIT      (13)     /* (13 * 1000 / 1344) = 9.67ms */
#define CLICK_TIME_LATENCY    (50)     /* (50 * 1000 / 1344) = 37ms */
#define CLICK_TIME_WINDOW     (250)    /* (250 * 1000 / 1344) = 186ms */

/* Input Event Constants */
#define ACCEL_G_MAX    16000
#define ACCEL_FUZZ     0
#define ACCEL_FLAT     0

#define N2DM_I2C_RETRY_COUNT   10  /* Number of times to retry i2c */
#define N2DM_I2C_RETRY_TIMEOUT 10  /* Timeout between retry (miliseconds) */

#define I2C_AUTO_INCREMENT (0x80)

struct {
    unsigned int cutoff_ms;
    unsigned int mask;
} n2dm_acc_odr_table[] = {
    {    1, ACCEL_ODR1344 },
    {    3, ACCEL_ODR400  },
    {    5, ACCEL_ODR200  },
    {   10, ACCEL_ODR100  },
    {   20, ACCEL_ODR50   },
    {   40, ACCEL_ODR25   },
    {  100, ACCEL_ODR10   },
    { 1000, ACCEL_ODR1    },
};

struct n2dm_accel_driver {
    struct i2c_client *client;
    struct n2dm_accel_platform_data accel_pdata;
    struct input_dev *input_dev;
    struct input_dev *input_dev_ddtap;
    struct delayed_work accel_work;
    struct workqueue_struct *accel_workqueue;

    int on_before_suspend;

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

    atomic_t accel_enabled;
    atomic_t accel_enabled_int;
    atomic_t accel_input_event;
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

enum {
    TAP_NO_DIRECTION = 0,
    TAP_POSITIVE,
    TAP_NEGATIVE,
};

enum {
    ACCEL_I2C_RESUME = 0,
    ACCEL_I2C_SUSPEND,
};

/* Below are for selftest */
/* Used time between sample readings, we will 1.344KHZ */
#define N2DM_ACCEL_SAMPLE_READY_TIME_MS           2
/* Selftest samples */
#define N2DM_ACCEL_SELFTEST_SAMPLES               10
/* Selftest threshold for self-motivation */
#define N2DM_SELFTEST_X_HIGH_POSITIVE  360
#define N2DM_SELFTEST_X_LOW_POSITIVE   17
#define N2DM_SELFTEST_Y_HIGH_POSITIVE  360
#define N2DM_SELFTEST_Y_LOW_POSITIVE   17
#define N2DM_SELFTEST_Z_HIGH_POSITIVE  360
#define N2DM_SELFTEST_Z_LOW_POSITIVE   17
#define N2DM_SELFTEST_X_HIGH_NEGATIVE  -17
#define N2DM_SELFTEST_X_LOW_NEGATIVE   -360
#define N2DM_SELFTEST_Y_HIGH_NEGATIVE  -17
#define N2DM_SELFTEST_Y_LOW_NEGATIVE   -360
#define N2DM_SELFTEST_Z_HIGH_NEGATIVE  -17
#define N2DM_SELFTEST_Z_LOW_NEGATIVE   -360

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

struct accel_axis_data {
    int x_axis;
    int y_axis;
    int z_axis;
};

static int n2dm_accel_parse_dt(struct device *dev, struct n2dm_accel_platform_data *acceld)
{
    int ret = 0;
    u32 val;

    ret = of_property_read_u32(dev->of_node, "n2dm_acc,min_interval", &val);
    if (!ret)
        acceld->min_interval = val;

    ret = of_property_read_u32(dev->of_node, "n2dm_acc,poll_interval", &val);
    if (!ret)
        acceld->poll_interval = val;

    ret = of_property_read_u32(dev->of_node, "n2dm_acc,accel_direction", &val);
    if (!ret)
        acceld->accel_direction = (u8)val;

    ret = of_property_read_u32(dev->of_node, "n2dm_acc,accel_irq_use_ddtap", &val);
    if (!ret)
        acceld->accel_irq_use_ddtap = (bool)val;

    ret = of_property_read_u32(dev->of_node, "n2dm_acc,accel_g_range", &val);
    if (!ret)
        acceld->accel_range = (u8)val;

    ret = of_property_read_u32(dev->of_node, "n2dm_acc,accel_tap_z_direction", &val);
    if (!ret)
        acceld->accel_tap_z_direction = (u8)val;

    ret = of_property_read_u32(dev->of_node, "n2dm_acc,accel_dtap_z_direction", &val);
    if (!ret)
        acceld->accel_dtap_z_direction = (u8)val;

    return ret;
}

static void n2dm_accel_i2c_resume(struct n2dm_accel_driver *acceld)
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

static s32 n2dm_accel_i2c_write_byte_data(struct i2c_client *client, u8 command, u8 value)
{
    u8 index;
    s32 ret;
    for (index = 0; index < N2DM_I2C_RETRY_COUNT; index++) {
        ret = i2c_smbus_write_byte_data(client, command, value);
        if (ret >= 0) {
            break;
        }
        else {
            KMSGERR(&client->dev, "Accel, i2c_smbus_write_byte_data, command = 0x%x, value = 0x%x, ret = %d, index = %u\n", command, value, ret, index);
            msleep(N2DM_I2C_RETRY_TIMEOUT);
        }
    }
    return ret;
}

static s32 n2dm_accel_i2c_read_byte_data(struct i2c_client *client, u8 command)
{
    u8 index;
    s32 ret;
    for (index = 0; index < N2DM_I2C_RETRY_COUNT; index++) {
        ret = i2c_smbus_read_byte_data(client, command);
        if (ret >= 0) {
            break;
        }
        else {
            KMSGERR(&client->dev, "Accel, i2c_smbus_read_byte_data, command = 0x%x, ret = %d, index = %u\n", command, ret, index);
            msleep(N2DM_I2C_RETRY_TIMEOUT);
        }
    }
    return ret;
}

static int n2dm_accel_i2c_read(struct n2dm_accel_driver *stat, u8 *buf, int len)
{
    int ret;
    u8 reg = buf[0];
    u8 cmd = reg;

    if (len > 1)
        cmd = (I2C_AUTO_INCREMENT | reg);

    ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
    if (ret != sizeof(cmd))
        return ret;

    return i2c_master_recv(stat->client, buf, len);
}

static void n2dm_accel_report_accel_data(struct n2dm_accel_driver *acceld)
{
    s16 accel_data_s16[3];
    int err;
    s16 x, y, z;

    if(atomic_read(&acceld->accel_enabled) > 0) {
        accel_data_s16[0] = (ACCEL_XOUT_L | I2C_AUTO_INCREMENT);
        err = n2dm_accel_i2c_read(acceld, (u8 *)accel_data_s16, 6);

        if (err < 0) {
            KMSGERR(&acceld->client->dev, "Accel, %s: read data output error = %d\n", __func__, err);
        } else {
            write_lock(&acceld->rwlock_accel_data);
            x = ((s16) le16_to_cpu(accel_data_s16[acceld->axis_map_x])) & MASK_16_10_BITS;
            y = ((s16) le16_to_cpu(accel_data_s16[acceld->axis_map_y])) & MASK_16_10_BITS;
            z = ((s16) le16_to_cpu(accel_data_s16[acceld->axis_map_z])) & MASK_16_10_BITS;
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

static int n2dm_accel_int_init(struct n2dm_accel_driver *acceld)
{
    int err;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG2, ACCEL_HPCLCK);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config hight pass filter fail.\n", __func__);
        return err;
    }

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG3, ACCEL_INT1_CLICK);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config tap map INT1 fail.\n", __func__);
        return err;
    }

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CLICK_THS, CLICK_THRESHOLD);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config low threshold fail.\n", __func__);
        return err;
    }

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_TIME_LIMIT, CLICK_TIME_LIMIT);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config time limit fail.\n", __func__);
        return err;
    }

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_TIME_LATENCY, CLICK_TIME_LATENCY);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, config time latency fail.\n", __func__);
        return err;
    }

    err = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CLICK_SRC);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, clear int fail.\n", __func__);
        return err;
    }

    return 0;
}

static int n2dm_accel_update_g_range(struct n2dm_accel_driver *acceld)
{
    int err;
    u8 updated_val, mask = N2DM_ACC_FS_MASK;
    KMSGINF(&acceld->client->dev, "Accel, %s, range = %u\n", __func__, acceld->accel_pdata.accel_range);

    switch (acceld->accel_pdata.accel_range) {
        case N2DM_ACC_G_2G:
            acceld->shift = 4;
            break;
        case N2DM_ACC_G_4G:
            acceld->shift = 3;
            break;
        case N2DM_ACC_G_8G:
            acceld->shift = 2;
            break;
        case N2DM_ACC_G_16G:
            acceld->shift = 1;
            break;
        default:
            KMSGERR(&acceld->client->dev, "Accel, %s, invalid fs range requested: %u\n", __func__, acceld->accel_pdata.accel_range);
            return -1;
    }

    n2dm_accel_i2c_resume(acceld);

    /* Updates configuration register 4, which contains fs range setting */
    err = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CTRL_REG4);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, read old g range fail.\n", __func__);
        return err;
    }
    updated_val = ((mask & acceld->accel_pdata.accel_range) | ((~mask) & err));
    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG4, updated_val);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, update new g range fail.\n", __func__);
        return err;
    }

    return 0;
}

static int n2dm_accel_update_odr(struct n2dm_accel_driver *acceld, int poll_interval)
{
    int err, i;
    u8 odr;
    KMSGINF(&acceld->client->dev, "Accel, %s, update odr %d\n", __func__, poll_interval);

    /* Following, looks for the longest possible odr interval scrolling the
     * odr_table vector from the end (shortest interval) backward (longest
     * interval), to support the poll_interval requested by the system.
     * It must be the longest interval lower then the poll interval.*/

    /* If device is currently enabled, we need to write new configuration out to it */
    if (atomic_read(&acceld->accel_enabled_int) > 0) {
        odr = ACCEL_ODR1344 | ACCEL_ACC_ENABLE_ALL_AXES;
        KMSGINF(&acceld->client->dev, "Accel, isr mode %d\n", odr);
    } else if (atomic_read(&acceld->accel_enabled) > 0) {
        for (i = ARRAY_SIZE(n2dm_acc_odr_table) - 1; i >= 0; i--) {
            if ((n2dm_acc_odr_table[i].cutoff_ms <= poll_interval) || (i == 0))
                break;
        }
        odr = n2dm_acc_odr_table[i].mask | ACCEL_ACC_ENABLE_ALL_AXES;
        KMSGINF(&acceld->client->dev, "Accel, polling mode %d\n", odr);
    } else {
        odr = ACCEL_POWER_DOWN;
        KMSGINF(&acceld->client->dev, "Accel, idle mode %d\n", odr);
    }

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, odr);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, update new odr fail.\n", __func__);
        return err;
    }

    return 0;
}

static void n2dm_accel_power_off(struct n2dm_accel_driver *acceld)
{
    if (acceld->accel_pdata.power_off) {
        acceld->accel_pdata.power_off(&acceld->input_dev->dev);
        acceld->accel_pdata.power_off(&acceld->input_dev_ddtap->dev);
    }
}

static int n2dm_accel_power_on(struct n2dm_accel_driver *acceld)
{
    if (acceld->accel_pdata.power_on) {
        acceld->accel_pdata.power_on(&acceld->input_dev->dev);
        acceld->accel_pdata.power_on(&acceld->input_dev_ddtap->dev);
    }

    return 0;
}

static irqreturn_t n2dm_accel_isr(int irq, void *dev)
{
    struct n2dm_accel_driver *acceld = dev;
    disable_irq_nosync(irq);
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);
    acceld->tap_pre_tip = acceld->tap_tip;
    acceld->tap_tip = jiffies;

    queue_work(acceld->irq1_work_queue, &acceld->irq1_work);
    return IRQ_HANDLED;
}

static void n2dm_accel_irq1_work_func(struct work_struct *work)
{
    struct n2dm_accel_driver *acceld = container_of(work, struct n2dm_accel_driver, irq1_work);
    int buf;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    n2dm_accel_i2c_resume(acceld);

    /* read int status and clear int at the same time. */
    if(atomic_read(&acceld->accel_enabled_int) > 0) {
        /* only tap in the top of z axis is enable */
        if(acceld->accel_pdata.accel_tap_z_direction == 0) {
            /* sensor is same direction with phone layout */
            if(acceld->negate_z == 0) {
                buf = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CLICK_SRC);
                KMSGINF(&acceld->client->dev, "Accel, z layout is 0, status = %d\n", buf);
                if(buf == (ACCEL_INT_CLICK | ACCEL_SINGLE_CLICK | ACCEL_NEGATIVE | ACCEL_INT_Z_CLICK)) {
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
                buf = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CLICK_SRC);
                KMSGINF(&acceld->client->dev, "Accel, z layout is 1, status  = %d\n", buf);
                if(buf == (ACCEL_INT_CLICK | ACCEL_SINGLE_CLICK | ACCEL_POSITIVE| ACCEL_INT_Z_CLICK)) {
                    acceld->tap_pre_direction = acceld->tap_direction;
                    acceld->tap_direction = TAP_POSITIVE;
                }
                else {
                    acceld->tap_pre_direction = acceld->tap_direction;
                    acceld->tap_direction = TAP_NO_DIRECTION;
                }
            }
        }
        /* tap in the top or bottom of z axis are enalbe */
        else {
            /* double tap must same direction */
            if(acceld->accel_pdata.accel_dtap_z_direction == 0) {
                buf = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CLICK_SRC);
                KMSGINF(&acceld->client->dev, "Accel, do not care z layout, double tap both 0 or 1, status = %d\n", buf);
                if(buf == (ACCEL_INT_CLICK | ACCEL_SINGLE_CLICK | ACCEL_POSITIVE | ACCEL_INT_Z_CLICK)) {
                    acceld->tap_pre_direction = acceld->tap_direction;
                    acceld->tap_direction = TAP_POSITIVE;
                }
                else if (buf == (ACCEL_INT_CLICK | ACCEL_SINGLE_CLICK | ACCEL_NEGATIVE | ACCEL_INT_Z_CLICK)) {
                    acceld->tap_pre_direction = acceld->tap_direction;
                    acceld->tap_direction = TAP_NEGATIVE;
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
            KMSGINF(&acceld->client->dev, "Accel, double tap time %ums\n", acceld->tap_time_ms);
            if((acceld->tap_time_ms < 150) || (acceld->tap_time_ms > 300))
                goto exit;

            acceld->tap_tip = acceld->tap_init_tip;
            input_report_rel(acceld->input_dev_ddtap, DOUBLE_TAP_INTERRUPT, 1);
            input_sync(acceld->input_dev_ddtap);
            KMSGINF(&acceld->client->dev, "Accel, double tap int response and report\n");
        }
    }
    if (unlikely(atomic_read(&acceld->accel_selftest_ongoing) == 1)) {
        atomic_set(&acceld->accel_selftest_int, 1);
        atomic_set(&acceld->accel_selftest_ongoing, 0);
        wake_up_interruptible(&acceld->accel_selftest_wq);
    }
exit:
    enable_irq(acceld->client->irq);
}

static void n2dm_accel_work(struct work_struct *work)
{
    struct n2dm_accel_driver *acceld = container_of((struct delayed_work *)work, struct n2dm_accel_driver, accel_work);

    queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, acceld->poll_delay);
    n2dm_accel_i2c_resume(acceld);
    n2dm_accel_report_accel_data(acceld);
}

static void n2dm_accel_update_direction(struct n2dm_accel_driver *acceld)
{
    unsigned int direction = acceld->accel_pdata.accel_direction;
    write_lock(&acceld->rwlock_accel_data);
    acceld->axis_map_x = ((direction-1)%2);
    acceld->axis_map_y = (direction%2);
    acceld->axis_map_z = 2;
    acceld->negate_z   = ((direction-1)/4);
    acceld->negate_x   = ((direction/2)%2);
    acceld->negate_y   = (((direction+1)/4)%2);
    write_unlock(&acceld->rwlock_accel_data);
    return;
}

/* current consumption */
/* normal mode(phone is nomal): 6uA, ODR is 25Hz */
/* double tap mode(phone is glance or sleep): 185ua ODR is 1344Hz */
/* low power mode(phone is glance or sleep, but tap function is off): 0.5ua ODR is 0 */

static int n2dm_accel_enable(struct n2dm_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    atomic_inc(&acceld->accel_enabled);
    err = n2dm_accel_update_odr(acceld, acceld->poll_interval);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, update odr fail.\n", __func__);
        return err;
    }
    queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);
    KMSGINF(&acceld->client->dev, "Accel, sensor enter polling mode\n");
    return err;
}

static int n2dm_accel_disable(struct n2dm_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    if(atomic_read(&acceld->accel_enabled) > 0){
        if(atomic_dec_and_test(&acceld->accel_enabled)) {
            cancel_delayed_work(&acceld->accel_work);
            err = n2dm_accel_update_odr(acceld, acceld->poll_interval);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, update odr fail.\n", __func__);
                return err;
            }
            KMSGINF(&acceld->client->dev, "Accel, sensor exit polling mode\n");
        }
    }
    return err;
}

static int n2dm_accel_enable_int(struct n2dm_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    atomic_inc(&acceld->accel_enabled_int);
    err = n2dm_accel_update_odr(acceld, acceld->poll_interval);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, update odr fail.\n", __func__);
        return err;
    }

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CLICK_CFG, ACCEL_SINGLE_Z_ENABLE);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, enable z single int fail.\n", __func__);
        return err;
    }
    KMSGINF(&acceld->client->dev, "Accel, sensor enter isr mode\n");
    enable_irq(acceld->client->irq);
    return err;
}

static int n2dm_accel_disable_int(struct n2dm_accel_driver *acceld)
{
    int err = -1;
    KMSGINF(&acceld->client->dev, "Accel, %s\n", __func__);

    if(atomic_read(&acceld->accel_enabled_int) > 0){
        if(atomic_dec_and_test(&acceld->accel_enabled_int)) {
            disable_irq_nosync(acceld->client->irq);
            err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CLICK_CFG, 0);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, disable int fail.\n", __func__);
                return err;
            }
            err = n2dm_accel_update_odr(acceld, acceld->poll_interval);
            if (err < 0) {
                KMSGERR(&acceld->client->dev, "Accel, %s, update odr fail.\n", __func__);
                return err;
            }
            KMSGINF(&acceld->client->dev, "Accel, sensor exit isr mode\n");
        }
    }
    return err;
}

static int n2dm_accel_input_open(struct input_dev *input)
{
    struct n2dm_accel_driver *acceld = input_get_drvdata(input);
    atomic_inc(&acceld->accel_input_event);
    return 0;
}

static void n2dm_accel_input_close(struct input_dev *dev)
{
    struct n2dm_accel_driver *acceld = input_get_drvdata(dev);
    atomic_dec(&acceld->accel_input_event);
}

static int n2dm_accel_input_ddtap_open(struct input_dev *input)
{
    return 0;
}

static void n2dm_accel_input_ddtap_close(struct input_dev *dev)
{
    ;
}

static int n2dm_accel_setup_input_device(struct n2dm_accel_driver *acceld)
{
    struct input_dev *input_dev, *input_dev_ddtap;
    int err;

    input_dev = input_allocate_device();
    acceld->input_dev = input_allocate_device();
    if (!input_dev) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to allocate input device for poll.\n", __func__);
        return -ENOMEM;
    }
    acceld->input_dev = input_dev;
    input_dev->open = n2dm_accel_input_open;
    input_dev->close = n2dm_accel_input_close;
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
        input_dev_ddtap->open = n2dm_accel_input_ddtap_open;
        input_dev_ddtap->close = n2dm_accel_input_ddtap_close;
        input_set_drvdata(input_dev_ddtap, acceld);
        __set_bit(REL_MISC, input_dev_ddtap->relbit);
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

static int n2dm_accel_hw_init(struct n2dm_accel_driver *acceld)
{
    int err = -1;
    int retval = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_WHO_AM_I);

    if(retval != ST_ACCEL_WHO_AM_I_N2DM) {
        KMSGERR(&acceld->client->dev, "Accel, %s, accelerometer is not N2DM.\n", __func__);
        return -ENODEV;
    }
    KMSGINF(&acceld->client->dev, "Accel, %s, accelerometer is N2DM.\n", __func__);

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, ALL_ZEROES | ACCEL_ACC_ENABLE_ALL_AXES);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, hardware init, config CTRL REG1 fail.\n", __func__);
        return err;
    }
    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG4, ALL_ZEROES | ACCEL_BDU_ENABLE);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, hardware init, config CTRL REG4 fail.\n", __func__);
        return err;
    }

    err = n2dm_accel_update_g_range(acceld);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, hardware init, config range fail.\n", __func__);
        return err;
    }
    acceld->poll_interval = acceld->accel_pdata.poll_interval;
    acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);
    err = n2dm_accel_update_odr(acceld, acceld->poll_interval);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, hardware init, config ODR fail.\n", __func__);
        return err;
    }
    n2dm_accel_update_direction(acceld);

    return err;
}

static void n2dm_accel_test(struct work_struct *test)
{
    struct n2dm_accel_driver *acceld = container_of((struct delayed_work *)test, struct n2dm_accel_driver, accel_test);
    s16 accel_data_s16[3];
    int err;
    accel_data_s16[0] = (ACCEL_XOUT_L | I2C_AUTO_INCREMENT);
    n2dm_accel_i2c_resume(acceld);
    err = n2dm_accel_i2c_read(acceld, (u8 *)accel_data_s16, 6);

    if (err < 0) {
        KMSGERR(&acceld->client->dev, "TEST, %s: read data output error = %d\n", __func__, err);
    } else {
        KMSGINF(&acceld->client->dev, "TEST: RAW DATA1, x, 0x%.4x, y, 0x%.4x, z, 0x%.4x\n",
                le16_to_cpu(accel_data_s16[0]), le16_to_cpu(accel_data_s16[1]), le16_to_cpu(accel_data_s16[2]));
        KMSGINF(&acceld->client->dev, "TEST: RAW DATA2, x, 0x%.4x, y, 0x%.4x, z, 0x%.4x\n",
                (le16_to_cpu(accel_data_s16[0]) & MASK_16_10_BITS),
                (le16_to_cpu(accel_data_s16[1]) & MASK_16_10_BITS),
                (le16_to_cpu(accel_data_s16[2]) & MASK_16_10_BITS));
        KMSGINF(&acceld->client->dev, "TEST: RAW DATA3, x, %d, y, %d, z, %d\n",
                ((s16)(le16_to_cpu(accel_data_s16[0]) & MASK_16_10_BITS)) >> 6,
                ((s16)(le16_to_cpu(accel_data_s16[1]) & MASK_16_10_BITS)) >> 6,
                ((s16)(le16_to_cpu(accel_data_s16[2]) & MASK_16_10_BITS)) >> 6);
    }
    queue_delayed_work(acceld->accel_testqueue, &acceld->accel_test, msecs_to_jiffies(500));
}


static ssize_t attr_get_polling_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", acceld->poll_interval);
}

static ssize_t attr_set_polling_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long interval;
    int err = 0;

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

    interval = max((unsigned int)interval, acceld->accel_pdata.min_interval);

    acceld->accel_pdata.poll_interval = interval;
    acceld->poll_interval = interval;
    acceld->poll_delay = msecs_to_jiffies(interval);
    n2dm_accel_i2c_resume(acceld);
    err = n2dm_accel_update_odr(acceld, interval);
    if(err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to update ODR.\n", __func__);
    }
exit:
    mutex_unlock(&input_dev->mutex);
    return (err < 0) ? err : size;
}

static ssize_t attr_get_direction(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", acceld->accel_pdata.accel_direction);
}

static ssize_t attr_set_direction(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long direction;
    int err = 0;

    mutex_lock(&input_dev->mutex);
    err = strict_strtoul(buf, 10, &direction);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }
    if (direction < 1 || direction > 8) {
        err = -EINVAL;
        KMSGERR(&acceld->client->dev, "Accel, %s, invalid direction = %d\n", __func__, (unsigned int) direction);
        goto exit;
    } else {
        acceld->accel_pdata.accel_direction = (u8)direction;
        n2dm_accel_update_direction(acceld);
    }
exit:
    mutex_unlock(&input_dev->mutex);
    return (err < 0) ? err: size;
}

static ssize_t attr_get_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    return sprintf(buf, "%d\n", atomic_read(&acceld->accel_enabled) > 0 ? 1 : 0);
}

static ssize_t attr_set_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long enable;
    int err = 0;

    mutex_lock(&input_dev->mutex);
    err = strict_strtoul(buf, 10, &enable);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }

    n2dm_accel_i2c_resume(acceld);

    if (enable)
        err = n2dm_accel_enable(acceld);
    else
        err = n2dm_accel_disable(acceld);
exit:
    mutex_unlock(&input_dev->mutex);
    return (err < 0) ? err : size;
}

static ssize_t attr_get_enable_int(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    return sprintf(buf, "%d\n", atomic_read(&acceld->accel_enabled_int) > 0 ? 1 : 0);
}

static ssize_t attr_set_enable_int(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev_ddtap = acceld->input_dev_ddtap;
    unsigned long enable;
    int err = 0;

    mutex_lock(&input_dev_ddtap->mutex);
    err = strict_strtoul(buf, 10, &enable);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }

    n2dm_accel_i2c_resume(acceld);

    if (enable)
        err = n2dm_accel_enable_int(acceld);
    else
        err = n2dm_accel_disable_int(acceld);
exit:
    mutex_unlock(&input_dev_ddtap->mutex);
    return (err < 0) ? err : size;
}

static ssize_t attr_get_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    u8 retval;
    char *bufloop;
    int buflen;

    bufloop = buf;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x0f);
    buflen = sprintf(bufloop, "WHO_AM_I............. 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x20);
    buflen = sprintf(bufloop, "CTRL1................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x21);
    buflen = sprintf(bufloop, "CTRL2................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x22);
    buflen = sprintf(bufloop, "CTRL3................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x23);
    buflen = sprintf(bufloop, "CTRL4................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x28);
    buflen = sprintf(bufloop, "XOUTL................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x29);
    buflen = sprintf(bufloop, "XOUTH................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x2a);
    buflen = sprintf(bufloop, "YOUTL................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x2b);
    buflen = sprintf(bufloop, "YOUTH................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x2c);
    buflen = sprintf(bufloop, "ZOUTL................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x2d);
    buflen = sprintf(bufloop, "ZOUTH................ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x38);
    buflen = sprintf(bufloop, "CLICKCFG............. 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x39);
    buflen = sprintf(bufloop, "CLICKSRC............. 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x3A);
    buflen = sprintf(bufloop, "CLICKTHS............. 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x3B);
    buflen = sprintf(bufloop, "TIMELIMIT............ 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x3C);
    buflen = sprintf(bufloop, "TIMELATENCY.......... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x3D);
    buflen = sprintf(bufloop, "TIMEWINDOW........... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x3E);
    buflen = sprintf(bufloop, "ACTTHS............... 0x%.2x\n", retval); bufloop+=buflen;
    retval = i2c_smbus_read_byte_data(acceld->client, 0x3F);
    buflen = sprintf(bufloop, "ACTDUS............... 0x%.2x\n", retval); bufloop+=buflen;

    return bufloop-buf;
}

static ssize_t attr_set_test(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev *input_dev = acceld->input_dev;
    unsigned long test;
    int err = 0;

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
            acceld->accel_testqueue = create_workqueue("N2DM Accel Testqueue");
            if (!acceld->accel_testqueue) {
                KMSGERR(&client->dev, "%s: Accel, failed to create test queue\n", __func__);
                err = -EPERM;
                goto exit;
            }
            INIT_DELAYED_WORK(&acceld->accel_test, n2dm_accel_test);
            queue_delayed_work(acceld->accel_testqueue, &acceld->accel_test, 0);
        }
        acceld->test = 1;
    } else {
        if(acceld->test == 1) {
            cancel_delayed_work_sync(&acceld->accel_test);
            destroy_workqueue(acceld->accel_testqueue);
        }
        acceld->test = 0;
    }
exit:
    mutex_unlock(&input_dev->mutex);
    return (err < 0) ? err : size;
}

static ssize_t attr_get_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", acceld->test);
}


static int n2dm_accel_selftest_get_data( struct n2dm_accel_driver* acceld, struct accel_axis_data* data )
{
    int sample_count = 0;
    int err = 0;
    /* x,y,z hardware data */
    s16 accel_data_s16[3];
    s16 x, y, z;

    while( sample_count < N2DM_ACCEL_SELFTEST_SAMPLES )
    {
        sample_count++;
        accel_data_s16[0] = (ACCEL_XOUT_L | I2C_AUTO_INCREMENT);
        err = n2dm_accel_i2c_read(acceld, (u8 *)accel_data_s16, 6);
        if(err < 0) {
            KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
        } else {
            write_lock(&acceld->rwlock_accel_data);
            x = ((s16) le16_to_cpu(accel_data_s16[acceld->axis_map_x])) & MASK_16_10_BITS;
            y = ((s16) le16_to_cpu(accel_data_s16[acceld->axis_map_y])) & MASK_16_10_BITS;
            z = ((s16) le16_to_cpu(accel_data_s16[acceld->axis_map_z])) & MASK_16_10_BITS;
            x = x >> 6; y = y >> 6; z = z >> 6;

            data->x_axis += x;
            data->y_axis += y;
            data->z_axis += z;
            write_unlock(&acceld->rwlock_accel_data);
            mdelay(N2DM_ACCEL_SAMPLE_READY_TIME_MS);
        }
    }
    return 0;
}

static ssize_t n2dm_accel_selftest_run_and_get_data(struct device *dev,
                struct device_attribute *attr, char *buf)
{

    struct i2c_client *client = to_i2c_client(dev);
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    struct input_dev* input_dev = acceld->input_dev;

    struct ACCEL_SELFTEST_RESULT* selftest_result = (struct ACCEL_SELFTEST_RESULT*)buf;
    struct accel_axis_data selftest_data = { 0 };
    struct accel_axis_data normal_data = { 0 };
    int err, reg_ctrl1, reg_ctrl3, reg_ctrl4;
    int selftest_x_high, selftest_x_low, selftest_y_high, selftest_y_low, selftest_z_high, selftest_z_low;
    u8 acc_data[6];

    KMSGINF(&acceld->client->dev, "selftest: n2dm_accel_selftest_run_and_get_data -> Enter\n");

    memset(selftest_result, 0, sizeof(*selftest_result));

    n2dm_accel_i2c_resume(acceld);

    mutex_lock(&input_dev->mutex);
    if (atomic_read(&acceld->accel_enabled_int) > 0) {
        /* IRQ is already enabled, Disable IRQ first */
        disable_irq_nosync(acceld->client->irq);
    }

    /* Init n2dm selftest */
    reg_ctrl1 = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CTRL_REG1);
    if (reg_ctrl1 < 0)
        goto Exit_1;

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, (ACCEL_ACC_ENABLE_ALL_AXES | ACCEL_ODR1344));
    if (err < 0)
        goto Exit_1;

    /* Start SelfTest 0 */
    /* Read Normal Data */
    KMSGINF(&acceld->client->dev, "selftest ST0 read normal data\n");

    if ( n2dm_accel_selftest_get_data( acceld, &normal_data) < 0 )
        goto Exit_1;

    reg_ctrl4 = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CTRL_REG4);
    if (reg_ctrl4 < 0)
        goto Exit_1;

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG4, (reg_ctrl4 | ACCEL_ST0));
    if (err < 0)
        goto Exit_1;

    KMSGINF(&acceld->client->dev, "selftest ST0 read selftest data\n");

    /* Read SelfTest 0 Data */
    if ( n2dm_accel_selftest_get_data( acceld, &selftest_data) < 0 )
        goto Exit_1;

    /* Compare and get result. */
    selftest_x_high = N2DM_SELFTEST_X_HIGH_NEGATIVE;
    selftest_x_low  = N2DM_SELFTEST_X_LOW_NEGATIVE;
    selftest_y_high = N2DM_SELFTEST_Y_HIGH_NEGATIVE;
    selftest_y_low  = N2DM_SELFTEST_Y_LOW_NEGATIVE;
    selftest_z_high = N2DM_SELFTEST_Z_HIGH_NEGATIVE;
    selftest_z_low  = N2DM_SELFTEST_Z_LOW_NEGATIVE;

    /* Average the normal data samples */
    normal_data.x_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    normal_data.y_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    normal_data.z_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    /* Average the selftest data samples */
    selftest_data.x_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    selftest_data.y_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    selftest_data.z_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    /* Check output change range */
    selftest_result->x_axis_result = selftest_data.x_axis - normal_data.x_axis;
    KMSGINF(&acceld->client->dev, "selftest ST0 normal_x_axis: %d, selftest_x_axis: %d, output_change: %d\n",
            normal_data.x_axis, selftest_data.x_axis, selftest_result->x_axis_result);

    if ((selftest_result->x_axis_result > selftest_x_high) || (selftest_result->x_axis_result < selftest_x_low)) {
        selftest_result->x_axis_pass = false;
    }
    else {
        selftest_result->x_axis_pass = true;
    }

    selftest_result->y_axis_result = selftest_data.y_axis - normal_data.y_axis;
    KMSGINF(&acceld->client->dev, "selftest ST0 normal_y_axis: %d, selftest_y_axis: %d, output_change: %d\n",
            normal_data.y_axis, selftest_data.y_axis, selftest_result->y_axis_result);
    if ((selftest_result->y_axis_result > selftest_y_high) || (selftest_result->y_axis_result < selftest_y_low)) {
        selftest_result->y_axis_pass = false;
    }
    else {
        selftest_result->y_axis_pass = true;
    }

    selftest_result->z_axis_result = selftest_data.z_axis - normal_data.z_axis;
    KMSGINF(&acceld->client->dev, "selftest ST0 normal_z_axis: %d, selftest_z_axis: %d, output_change: %d\n",
            normal_data.z_axis, selftest_data.z_axis, selftest_result->z_axis_result);
    if ((selftest_result->z_axis_result > selftest_z_high) || (selftest_result->z_axis_result < selftest_z_low)) {
        selftest_result->z_axis_pass = false;
    }
    else {
        selftest_result->z_axis_pass = true;
    }

    /* Start SelfTest 1 */
    /* Read Normal Data */
    KMSGINF(&acceld->client->dev, "selftest ST1 read normal data\n");
    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG4, reg_ctrl4);
    if (err < 0)
        goto Exit_1;

    if ( n2dm_accel_selftest_get_data( acceld, &normal_data) < 0 )
        goto Exit_1;

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG4, (reg_ctrl4 | ACCEL_ST1));
    if (err < 0)
        goto Exit_1;

    KMSGINF(&acceld->client->dev, "selftest ST1 read selftest data\n");

    /* Read SelfTest 1 Data */
    if ( n2dm_accel_selftest_get_data( acceld, &selftest_data) < 0 )
        goto Exit_1;

    /* Compare and get result. */
    selftest_x_high = N2DM_SELFTEST_X_HIGH_POSITIVE;
    selftest_x_low  = N2DM_SELFTEST_X_LOW_POSITIVE;
    selftest_y_high = N2DM_SELFTEST_Y_HIGH_POSITIVE;
    selftest_y_low  = N2DM_SELFTEST_Y_LOW_POSITIVE;
    selftest_z_high = N2DM_SELFTEST_Z_HIGH_POSITIVE;
    selftest_z_low  = N2DM_SELFTEST_Z_LOW_POSITIVE;

    /* Average the normal data samples */
    normal_data.x_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    normal_data.y_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    normal_data.z_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    /* Average the selftest data samples */
    selftest_data.x_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    selftest_data.y_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    selftest_data.z_axis /= N2DM_ACCEL_SELFTEST_SAMPLES;
    /* Check output change range */
    selftest_result->x_axis_result = selftest_data.x_axis - normal_data.x_axis;
    KMSGINF(&acceld->client->dev, "selftest ST1 normal_x_axis: %d, selftest_x_axis: %d, output_change: %d\n",
            normal_data.x_axis, selftest_data.x_axis, selftest_result->x_axis_result);

    if ((selftest_result->x_axis_result > selftest_x_high) || (selftest_result->x_axis_result < selftest_x_low)) {
        selftest_result->x_axis_pass = false;
    }
    else {
        selftest_result->x_axis_pass = true;
    }

    selftest_result->y_axis_result = selftest_data.y_axis - normal_data.y_axis;
    KMSGINF(&acceld->client->dev, "selftest ST1 normal_y_axis: %d, selftest_y_axis: %d, output_change: %d\n",
            normal_data.y_axis, selftest_data.y_axis, selftest_result->y_axis_result);
    if ((selftest_result->y_axis_result > selftest_y_high) || (selftest_result->y_axis_result < selftest_y_low)) {
        selftest_result->y_axis_pass = false;
    }
    else {
        selftest_result->y_axis_pass = true;
    }

    selftest_result->z_axis_result = selftest_data.z_axis - normal_data.z_axis;
    KMSGINF(&acceld->client->dev, "selftest ST1 normal_z_axis: %d, selftest_z_axis: %d, output_change: %d\n",
            normal_data.z_axis, selftest_data.z_axis, selftest_result->z_axis_result);
    if ((selftest_result->z_axis_result > selftest_z_high) || (selftest_result->z_axis_result < selftest_z_low)) {
        selftest_result->z_axis_pass = false;
    }
    else {
        selftest_result->z_axis_pass = true;
    }

    /* Stop SelfTest */
    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG4, reg_ctrl4);
    if (err < 0)
        goto Exit_1;

    /* Test Interrupt */
    selftest_result->interrupt_pin_support = true;
    selftest_result->interrupt_pin_status = false;
    if (selftest_result->interrupt_pin_support == true) {
        atomic_set(&acceld->accel_selftest_int, 0);
        atomic_set(&acceld->accel_selftest_ongoing, 1);
        init_waitqueue_head(&acceld->accel_selftest_wq);

        reg_ctrl3 = n2dm_accel_i2c_read_byte_data(acceld->client, ACCEL_CTRL_REG3);
        if (reg_ctrl3 < 0)
            goto Exit_1;

        err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG3, ACCEL_INT1_DRDY);
        enable_irq(acceld->client->irq);
        if (err < 0)
            goto Exit_1;

        wait_event_interruptible_timeout(acceld->accel_selftest_wq, atomic_read(&acceld->accel_selftest_int), msecs_to_jiffies(100));
        if (atomic_read(&acceld->accel_selftest_int) == 1) {
            selftest_result->interrupt_pin_status = true;
            KMSGINF(&acceld->client->dev, "selftest  get DRDY interrupt\n");
            /* read data to clear INT */
            acc_data[0] = (ACCEL_XOUT_L | I2C_AUTO_INCREMENT);
            n2dm_accel_i2c_read(acceld, acc_data, 6);
        }

        disable_irq_nosync(acceld->client->irq);
        err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG3, reg_ctrl3);
        if (err < 0)
            goto Exit_1;
    }

    err = n2dm_accel_i2c_write_byte_data(acceld->client, ACCEL_CTRL_REG1, reg_ctrl1);
    if (err < 0)
        goto Exit_1;
    if (atomic_read(&acceld->accel_enabled_int) > 0) {
        /* IRQ was enabled previously, Re-enable IRQ */
        enable_irq(acceld->client->irq);
    }

    /* I2C status is true */
    selftest_result->i2c_status = true;
    mutex_unlock(&input_dev->mutex);
    KMSGINF(&acceld->client->dev, "selftest n2dm_accel_selftest_run_and_get_data -> Success Exit\n");

    return sizeof(struct ACCEL_SELFTEST_RESULT);

Exit_1:
    selftest_result->interrupt_pin_support = true;
    selftest_result->interrupt_pin_status = false;
    selftest_result->x_axis_pass = false;
    selftest_result->y_axis_pass = false;
    selftest_result->z_axis_pass = false;
    selftest_result->i2c_status = false;
    mutex_unlock(&input_dev->mutex);
    KMSGINF(&acceld->client->dev, "selftest n2dm_accel_selftest_run_and_get_data -> Error Exit\n");
    return sizeof(struct ACCEL_SELFTEST_RESULT);
}

static DEVICE_ATTR(enable_poll, S_IRUGO|S_IWUGO, attr_get_enable, attr_set_enable);
static DEVICE_ATTR(enable_int,  S_IRUGO|S_IWUGO, attr_get_enable_int, attr_set_enable_int);
static DEVICE_ATTR(delay,       S_IRUGO|S_IWUGO, attr_get_polling_rate, attr_set_polling_rate);
static DEVICE_ATTR(direction,   S_IRUGO|S_IWUGO, attr_get_direction, attr_set_direction);
static DEVICE_ATTR(selftest_data, S_IRUGO, n2dm_accel_selftest_run_and_get_data, NULL);
static DEVICE_ATTR(test, S_IRUGO|S_IWUGO, attr_get_test, attr_set_test);
static DEVICE_ATTR(getreg,  S_IRUGO, attr_get_reg, NULL);

static struct attribute *n2dm_accel_attributes[] = {
    &dev_attr_enable_poll.attr,
    &dev_attr_enable_int.attr,
    &dev_attr_delay.attr,
    &dev_attr_direction.attr,
    &dev_attr_selftest_data.attr,
    &dev_attr_test.attr,
    &dev_attr_getreg.attr,
    NULL
};

static struct attribute_group n2dm_accel_attribute_group = {
    .attrs = n2dm_accel_attributes
};

static int __devinit n2dm_accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct n2dm_accel_driver *acceld;
    struct n2dm_accel_platform_data *accel_pdata;
    int err;

    /* 1. check i2c client */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
        KMSGERR(&client->dev, "Accel, %s, client is not i2c capable. Abort.\n", __func__);
        return -ENXIO;
    }
    KMSGINF(&client->dev, "Accel, %s, client is i2c capable. OK.\n", __func__);

    /* 2. check config node, then ready memory to get platform struct data */
    if (client->dev.of_node) {
        accel_pdata = devm_kzalloc(&client->dev, sizeof(struct n2dm_accel_platform_data), GFP_KERNEL);
        if (!accel_pdata) {
            KMSGERR(&client->dev, "Accel, %s, failed to allocate memory for platform data. Abort.\n", __func__);
            return -ENOMEM;
        }
    }
    else {
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

    /* 4. get platform data from config node file */
    err = n2dm_accel_parse_dt(&client->dev, accel_pdata);
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

    err = n2dm_accel_setup_input_device(acceld);
    if (err) {
        KMSGERR(&client->dev, "Accel, %s, failed to setup input device. Abort\n", __func__);
        goto err_free_mem;
    }

    /* 7. ready some init interface for spare action */
    err = n2dm_accel_power_on(acceld);
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

    rwlock_init(&acceld->rwlock_accel_data);
    err = n2dm_accel_hw_init(acceld);
    if (err < 0) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to init hardware. Abort.\n", __func__);
        goto err_accel_pdata_exit;
    }
    KMSGINF(&client->dev, "Accel, %s, Device is correct. OK.\n", __func__);

    /* 9. if necessary, do INT init */
    if(acceld->accel_ddtap == 1) {
        err = n2dm_accel_int_init(acceld);
        if (err) {
            KMSGERR(&acceld->client->dev, "Accel, %s, failed to init INT. Abort.\n", __func__);
            goto err_accel_pdata_exit;
        }
    }
    KMSGINF(&client->dev, "Accel, %s, Int init success. OK.\n", __func__);

    atomic_set(&acceld->accel_enabled, 0);
    atomic_set(&acceld->accel_enabled_int, 0);
    atomic_set(&acceld->accel_input_event, 0);

    acceld->proc_dir = proc_mkdir("sensors", NULL);
    if (acceld->proc_dir == NULL) {
        KMSGERR(&client->dev, "Accel, %s, failed to create /proc/sensors. Abort.\n", __func__);
        goto err_accel_pdata_exit;
    }
    else {
        acceld->proc_entry = create_proc_entry( "accelinfo", 0644, acceld->proc_dir);
        if (acceld->proc_entry == NULL) {
            KMSGERR(&client->dev, "Accel, %s, failed to create /proc/cpu/accelinfo. Abort.\n", __func__);
            goto err_free_proc_direction;
        }
    }

    /* 10. init work queue and irq, irq work queue */
    acceld->accel_workqueue = create_workqueue("N2DM Accel Workqueue");
    if (!acceld->accel_workqueue) {
        KMSGERR(&client->dev, "Accel, %s, failed to create work queue. Abort.\n", __func__);
        goto err_free_proc_file;
    }
    INIT_DELAYED_WORK(&acceld->accel_work, n2dm_accel_work);

    if (acceld->accel_ddtap) {
        INIT_WORK(&acceld->irq1_work, n2dm_accel_irq1_work_func);
        acceld->irq1_work_queue = create_singlethread_workqueue("N2DM Accel Intqueue");
        if (!acceld->irq1_work_queue) {
            KMSGERR(&client->dev, "Accel, %s, failed to create int work queue. Abort.\n", __func__);
            goto err_free_workqueue;
        }

        err = request_irq(client->irq, n2dm_accel_isr, IRQF_TRIGGER_RISING, "n2dm_acc_irq1", acceld); /* IRQF_ONESHOT */
        if (err < 0) {
            KMSGERR(&client->dev, "Accel, %s, failed to request irq returned err = %d. Abort.\n", __func__, err);
            goto err_free_irq_workqueue;
        }
        disable_irq_nosync(client->irq);
        enable_irq_wake(client->irq);    /* enable wake up phone function when is in the sleep */
    }
    KMSGINF(&client->dev, "Accel, %s, Init work queue and IRQ init, IRQ work queue success. OK.\n", __func__);

    /* 11. create file system for device */
    err = sysfs_create_group(&client->dev.kobj, &n2dm_accel_attribute_group);
    if (err) {
        KMSGERR(&acceld->client->dev, "Accel, %s, failed to create sysfs returned err = %d. Abort.\n", __func__, err);
        goto err_free_irq;
    }
    KMSGINF(&client->dev, "Accel, %s, Crete file system. OK.\n", __func__);

    acceld->test = 0;
    acceld->tap_tip = jiffies;
    acceld->tap_init_tip = acceld->tap_tip;
    acceld->tap_direction = TAP_NO_DIRECTION;
    KMSGINF(&client->dev, "Accel, %s, N2DM sensor Probe Done. OK.\n", __func__);
    return 0;

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
err_accel_pdata_exit:
    if (accel_pdata->exit)
        accel_pdata->exit();
err_accel_pdata_power_off:
    n2dm_accel_power_off(acceld);
err_free_device_register:
    if (acceld->accel_ddtap)
        input_unregister_device(acceld->input_dev_ddtap);
    input_unregister_device(acceld->input_dev);
err_free_mem:
    kfree(acceld);
    return err;

}

static int n2dm_accel_remove(struct i2c_client *client)
{
    struct n2dm_accel_driver *acceld;
    acceld = i2c_get_clientdata(client);

    sysfs_remove_group(&client->dev.kobj, &n2dm_accel_attribute_group);
    if (acceld->accel_ddtap) {
        disable_irq_wake(client->irq);    /* cancel wake up phone function when is in the sleep */
        free_irq(client->irq, acceld);
        destroy_workqueue(acceld->irq1_work_queue);
    }
    destroy_workqueue(acceld->accel_workqueue);
    remove_proc_entry("accelinfo", acceld->proc_dir);
    remove_proc_entry("sensors", NULL);
    if (acceld->accel_pdata.exit)
        acceld->accel_pdata.exit();
    n2dm_accel_power_off(acceld);
    if (acceld->accel_ddtap)
        input_unregister_device(acceld->input_dev_ddtap);
    input_unregister_device(acceld->input_dev);
    kfree(acceld);

    return 0;
}


#ifdef CONFIG_PM
static int n2dm_accel_resume(struct i2c_client *client)
{
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    KMSGINF(&client->dev, "%s\n", __func__);
    atomic_set(&acceld->suspend, ACCEL_I2C_RESUME);
    return 0;
}

static int n2dm_accel_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct n2dm_accel_driver *acceld = i2c_get_clientdata(client);
    KMSGINF(&client->dev, "%s\n", __func__);
    atomic_set(&acceld->suspend, ACCEL_I2C_SUSPEND);
    return 0;
}
#else
#define n2dm_acc_suspend    NULL
#define n2dm_acc_resume    NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id n2dm_acc_id[]
        = { { N2DM_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, n2dm_acc_id);

static const struct of_device_id n2dm_acc_match[] = {
    { .compatible = "st,n2dm_acc", },
    { },
};

static struct i2c_driver n2dm_acc_driver = {
    .driver = {
            .owner = THIS_MODULE,
            .name = N2DM_ACC_DEV_NAME,
            .of_match_table = n2dm_acc_match,
          },
    .probe = n2dm_accel_probe,
    .remove = __devexit_p(n2dm_accel_remove),
    .suspend = n2dm_accel_suspend,
    .resume = n2dm_accel_resume,
    .id_table = n2dm_acc_id,
};

static int __init n2dm_accel_init(void)
{
    pr_info("%s accelerometer driver: init\n", N2DM_ACC_DEV_NAME);
    return i2c_add_driver(&n2dm_acc_driver);
}

static void __exit n2dm_accel_exit(void)
{
    pr_info("%s accelerometer driver exit\n", N2DM_ACC_DEV_NAME);
    i2c_del_driver(&n2dm_acc_driver);
    return;
}

module_init(n2dm_accel_init);
module_exit(n2dm_accel_exit);

MODULE_DESCRIPTION("n2dm accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
