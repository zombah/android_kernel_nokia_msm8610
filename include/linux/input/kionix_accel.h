/******************** (C) COPYRIGHT 2013 Nokia *********************************
 *
 * File Name          : kionix_accel.h
 * Version            : V.1.0.2
 * Date               : 2013/Nov/21
 * Description        : KIONIX accelerometer driver sensor
 *
 *******************************************************************************/

#ifndef __KIONIX_ACCEL_H__
#define __KIONIX_ACCEL_H__

#define KIONIX_ACCEL_I2C_ADDR        0x1e
#define KIONIX_ACCEL_NAME            "kionix_accel"
#define KIONIX_ACCEL_IRQ             "kionix-irq"



struct kionix_accel_platform_data {
    /* Although the accelerometer can perform at high ODR,
     * there is a need to keep the maximum ODR to a lower
     * value due to power consumption or other concern.
     * Use this variable to set the minimum allowable
     * interval for data to be reported from the
     * accelerometer. Unit is measured in milli-
     * seconds. Recommended value is 5ms. */
    unsigned int min_interval;
    /* Use this variable to set the default interval for
     * data to be reported from the accelerometer. This
     * value will be used during driver setup process,
     * but can be changed by the system during runtime via
     * sysfs control. Recommended value is 200ms.*/
    unsigned int poll_interval;

    /* This variable controls the corresponding direction
     * of the accelerometer that is mounted on the board
     * of the device. Refer to the porting guide for
     * details. Valid value is 1 to 8. */
    u8 accel_direction;

    /* Use this variable to choose whether or not to use
     * Double interrupt interrupt mode.
     * Valid value is 0 or 1.*/
    bool accel_irq_use_ddtap;

    /* Use this variable to control the number of
     * effective bits of the accelerometer output.
     * Use the macro definition to select the desired
     * number of effective bits. */
    #define KIONIX_ACCEL_RES_12BIT    0
    #define KIONIX_ACCEL_RES_8BIT     1
    #define KIONIX_ACCEL_RES_6BIT     2
    #define KIONIX_ACCEL_RES_16BIT    3    //KX023
    u8 accel_res;

    /* Use this variable to control the G range of
     * the accelerometer output. Use the macro definition
     * to select the desired G range.*/
    #define KIONIX_ACCEL_G_2G        0
    #define KIONIX_ACCEL_G_4G        1
    #define KIONIX_ACCEL_G_6G        2
    #define KIONIX_ACCEL_G_8G        3
    u8 accel_g_range;

    u8 accel_tap_z_direction;
    u8 accel_dtap_z_direction;
    /* Optional callback functions that can be implemented
     * on per product basis. If these callbacks are defined,
     * they will be called by the driver. */
    int (*init)(struct device*);
    void (*exit)(void);
    int (*power_on)(struct device*);
    int (*power_off)(struct device*);
};
#endif  /* __KIONIX_ACCEL_H__ */
