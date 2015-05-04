/******************** (C) COPYRIGHT 2013 Nokia *********************************
 *
 * File Name          : n2dm.h
 * Version            : V.1.0.1
 * Date               : 2013/Nov/21
 * Description        : N2DM accelerometer driver sensor
 *
 *******************************************************************************/

#ifndef __N2DM_H__
#define __N2DM_H__


#define N2DM_ACC_DEV_NAME  "n2dm_accel"

#define N2DM_ACC_MIN_POLL_PERIOD_MS 1


#ifdef __KERNEL__

#define N2DM_ACC_DEFAULT_INT1_GPIO  (-EINVAL)
#define N2DM_ACC_DEFAULT_INT2_GPIO  (-EINVAL)

/* Accelerometer Sensor Full Scale */
#define N2DM_ACC_FS_MASK      (0x30)

struct n2dm_accel_platform_data {
    unsigned int poll_interval;
    unsigned int min_interval;

    #define N2DM_ACC_G_2G     (0x00)
    #define N2DM_ACC_G_4G     (0x10)
    #define N2DM_ACC_G_8G     (0x20)
    #define N2DM_ACC_G_16G    (0x30)
    u8 accel_range;

    u8 accel_direction;
    bool accel_irq_use_ddtap;

    u8 accel_tap_z_direction;
    u8 accel_dtap_z_direction;

    int (*init)(struct device*);
    void (*exit)(void);
    int (*power_on)(struct device*);
    int (*power_off)(struct device*);
};

#endif /* __KERNEL__ */

#endif /* __N2DM_H__ */




