/*  Date: 2012/6/13 10:00:00
 *  Revision: 1.4
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file BMA2X2.c
   brief This file contains all function implementations for the BMA2X2 in linux

*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define BMA2X2_ENABLE_INT1


#define SENSOR_NAME             "bma250_accel"
#define ABSMIN                -512
#define ABSMAX                512
#define SLOPE_THRESHOLD_VALUE         32
#define SLOPE_DURATION_VALUE         1
#define INTERRUPT_LATCH_MODE         13
#define INTERRUPT_ENABLE         1
#define INTERRUPT_DISABLE         0
#define MAP_SLOPE_INTERRUPT         2
#define SLOPE_X_INDEX             5
#define SLOPE_Y_INDEX             6
#define SLOPE_Z_INDEX             7
#define BMA2X2_MAX_DELAY        200
#define BMA2X2_RANGE_SET        3  /* +/- 2G */
#define BMA2X2_BW_SET            12 /* 125HZ  */

#define LOW_G_INTERRUPT                REL_Z
#define HIGH_G_INTERRUPT             REL_HWHEEL
#define SLOP_INTERRUPT                 REL_DIAL
#define DOUBLE_TAP_INTERRUPT             REL_WHEEL
#define SINGLE_TAP_INTERRUPT             REL_MISC
#define ORIENT_INTERRUPT             ABS_PRESSURE
#define FLAT_INTERRUPT                 ABS_DISTANCE


#define HIGH_G_INTERRUPT_X_HAPPENED            1
#define HIGH_G_INTERRUPT_Y_HAPPENED             2
#define HIGH_G_INTERRUPT_Z_HAPPENED             3
#define HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED         4
#define HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED        5
#define HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED         6
#define SLOPE_INTERRUPT_X_HAPPENED             7
#define SLOPE_INTERRUPT_Y_HAPPENED             8
#define SLOPE_INTERRUPT_Z_HAPPENED             9
#define SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED         10
#define SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED         11
#define SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED         12
#define DOUBLE_TAP_INTERRUPT_HAPPENED             13
#define SINGLE_TAP_INTERRUPT_HAPPENED             14
#define UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED         15
#define UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED         16
#define UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED     17
#define UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED    18
#define DOWNWARD_PORTRAIT_UP_INTERRUPT_HAPPENED     19
#define DOWNWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED     20
#define DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED     21
#define DOWNWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED     22
#define FLAT_INTERRUPT_TURE_HAPPENED            23
#define FLAT_INTERRUPT_FALSE_HAPPENED            24
#define LOW_G_INTERRUPT_HAPPENED            25

#define BMA2X2_EEP_OFFSET                       0x16
#define BMA2X2_IMAGE_BASE                       0x38
#define BMA2X2_IMAGE_LEN                        22

#define BMA2X2_CHIP_ID_REG                      0x00
#define BMA2X2_VERSION_REG                      0x01
#define BMA2X2_X_AXIS_LSB_REG                   0x02
#define BMA2X2_X_AXIS_MSB_REG                   0x03
#define BMA2X2_Y_AXIS_LSB_REG                   0x04
#define BMA2X2_Y_AXIS_MSB_REG                   0x05
#define BMA2X2_Z_AXIS_LSB_REG                   0x06
#define BMA2X2_Z_AXIS_MSB_REG                   0x07
#define BMA2X2_TEMP_RD_REG                      0x08
#define BMA2X2_STATUS1_REG                      0x09
#define BMA2X2_STATUS2_REG                      0x0A
#define BMA2X2_STATUS_TAP_SLOPE_REG             0x0B
#define BMA2X2_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA2X2_STATUS_FIFO_REG                  0x0E
#define BMA2X2_RANGE_SEL_REG                    0x0F
#define BMA2X2_BW_SEL_REG                       0x10
#define BMA2X2_MODE_CTRL_REG                    0x11
#define BMA2X2_LOW_NOISE_CTRL_REG               0x12
#define BMA2X2_DATA_CTRL_REG                    0x13
#define BMA2X2_RESET_REG                        0x14
#define BMA2X2_INT_ENABLE1_REG                  0x16
#define BMA2X2_INT_ENABLE2_REG                  0x17
#define BMA2X2_INT_SLO_NO_MOT_REG               0x18
#define BMA2X2_INT1_PAD_SEL_REG                 0x19
#define BMA2X2_INT_DATA_SEL_REG                 0x1A
#define BMA2X2_INT2_PAD_SEL_REG                 0x1B
#define BMA2X2_INT_SRC_REG                      0x1E
#define BMA2X2_INT_SET_REG                      0x20
#define BMA2X2_INT_CTRL_REG                     0x21
#define BMA2X2_LOW_DURN_REG                     0x22
#define BMA2X2_LOW_THRES_REG                    0x23
#define BMA2X2_LOW_HIGH_HYST_REG                0x24
#define BMA2X2_HIGH_DURN_REG                    0x25
#define BMA2X2_HIGH_THRES_REG                   0x26
#define BMA2X2_SLOPE_DURN_REG                   0x27
#define BMA2X2_SLOPE_THRES_REG                  0x28
#define BMA2X2_SLO_NO_MOT_THRES_REG             0x29
#define BMA2X2_TAP_PARAM_REG                    0x2A
#define BMA2X2_TAP_THRES_REG                    0x2B
#define BMA2X2_ORIENT_PARAM_REG                 0x2C
#define BMA2X2_THETA_BLOCK_REG                  0x2D
#define BMA2X2_THETA_FLAT_REG                   0x2E
#define BMA2X2_FLAT_HOLD_TIME_REG               0x2F
#define BMA2X2_FIFO_WML_TRIG                    0x30
#define BMA2X2_SELF_TEST_REG                    0x32
#define BMA2X2_EEPROM_CTRL_REG                  0x33
#define BMA2X2_SERIAL_CTRL_REG                  0x34
#define BMA2X2_EXTMODE_CTRL_REG                 0x35
#define BMA2X2_OFFSET_CTRL_REG                  0x36
#define BMA2X2_OFFSET_PARAMS_REG                0x37
#define BMA2X2_OFFSET_X_AXIS_REG                0x38
#define BMA2X2_OFFSET_Y_AXIS_REG                0x39
#define BMA2X2_OFFSET_Z_AXIS_REG                0x3A
#define BMA2X2_GP0_REG                          0x3B
#define BMA2X2_GP1_REG                          0x3C
#define BMA2X2_FIFO_MODE_REG                    0x3E
#define BMA2X2_FIFO_DATA_OUTPUT_REG             0x3F




#define BMA2X2_CHIP_ID__POS             0
#define BMA2X2_CHIP_ID__MSK             0xFF
#define BMA2X2_CHIP_ID__LEN             8
#define BMA2X2_CHIP_ID__REG             BMA2X2_CHIP_ID_REG

#define BMA2X2_VERSION__POS          0
#define BMA2X2_VERSION__LEN          8
#define BMA2X2_VERSION__MSK          0xFF
#define BMA2X2_VERSION__REG          BMA2X2_VERSION_REG



#define BMA2X2_NEW_DATA_X__POS          0
#define BMA2X2_NEW_DATA_X__LEN          1
#define BMA2X2_NEW_DATA_X__MSK          0x01
#define BMA2X2_NEW_DATA_X__REG          BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X14_LSB__POS           2
#define BMA2X2_ACC_X14_LSB__LEN           6
#define BMA2X2_ACC_X14_LSB__MSK           0xFC
#define BMA2X2_ACC_X14_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X12_LSB__POS           4
#define BMA2X2_ACC_X12_LSB__LEN           4
#define BMA2X2_ACC_X12_LSB__MSK           0xF0
#define BMA2X2_ACC_X12_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X10_LSB__POS           6
#define BMA2X2_ACC_X10_LSB__LEN           2
#define BMA2X2_ACC_X10_LSB__MSK           0xC0
#define BMA2X2_ACC_X10_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X8_LSB__POS           0
#define BMA2X2_ACC_X8_LSB__LEN           0
#define BMA2X2_ACC_X8_LSB__MSK           0x00
#define BMA2X2_ACC_X8_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X_MSB__POS           0
#define BMA2X2_ACC_X_MSB__LEN           8
#define BMA2X2_ACC_X_MSB__MSK           0xFF
#define BMA2X2_ACC_X_MSB__REG           BMA2X2_X_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Y__POS          0
#define BMA2X2_NEW_DATA_Y__LEN          1
#define BMA2X2_NEW_DATA_Y__MSK          0x01
#define BMA2X2_NEW_DATA_Y__REG          BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y14_LSB__POS           2
#define BMA2X2_ACC_Y14_LSB__LEN           6
#define BMA2X2_ACC_Y14_LSB__MSK           0xFC
#define BMA2X2_ACC_Y14_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y12_LSB__POS           4
#define BMA2X2_ACC_Y12_LSB__LEN           4
#define BMA2X2_ACC_Y12_LSB__MSK           0xF0
#define BMA2X2_ACC_Y12_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y10_LSB__POS           6
#define BMA2X2_ACC_Y10_LSB__LEN           2
#define BMA2X2_ACC_Y10_LSB__MSK           0xC0
#define BMA2X2_ACC_Y10_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y8_LSB__POS           0
#define BMA2X2_ACC_Y8_LSB__LEN           0
#define BMA2X2_ACC_Y8_LSB__MSK           0x00
#define BMA2X2_ACC_Y8_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y_MSB__POS           0
#define BMA2X2_ACC_Y_MSB__LEN           8
#define BMA2X2_ACC_Y_MSB__MSK           0xFF
#define BMA2X2_ACC_Y_MSB__REG           BMA2X2_Y_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Z__POS          0
#define BMA2X2_NEW_DATA_Z__LEN          1
#define BMA2X2_NEW_DATA_Z__MSK          0x01
#define BMA2X2_NEW_DATA_Z__REG          BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z14_LSB__POS           2
#define BMA2X2_ACC_Z14_LSB__LEN           6
#define BMA2X2_ACC_Z14_LSB__MSK           0xFC
#define BMA2X2_ACC_Z14_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z12_LSB__POS           4
#define BMA2X2_ACC_Z12_LSB__LEN           4
#define BMA2X2_ACC_Z12_LSB__MSK           0xF0
#define BMA2X2_ACC_Z12_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z10_LSB__POS           6
#define BMA2X2_ACC_Z10_LSB__LEN           2
#define BMA2X2_ACC_Z10_LSB__MSK           0xC0
#define BMA2X2_ACC_Z10_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z8_LSB__POS           0
#define BMA2X2_ACC_Z8_LSB__LEN           0
#define BMA2X2_ACC_Z8_LSB__MSK           0x00
#define BMA2X2_ACC_Z8_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z_MSB__POS           0
#define BMA2X2_ACC_Z_MSB__LEN           8
#define BMA2X2_ACC_Z_MSB__MSK           0xFF
#define BMA2X2_ACC_Z_MSB__REG           BMA2X2_Z_AXIS_MSB_REG

#define BMA2X2_TEMPERATURE__POS         0
#define BMA2X2_TEMPERATURE__LEN         8
#define BMA2X2_TEMPERATURE__MSK         0xFF
#define BMA2X2_TEMPERATURE__REG         BMA2X2_TEMP_RD_REG

#define BMA2X2_LOWG_INT_S__POS          0
#define BMA2X2_LOWG_INT_S__LEN          1
#define BMA2X2_LOWG_INT_S__MSK          0x01
#define BMA2X2_LOWG_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_HIGHG_INT_S__POS          1
#define BMA2X2_HIGHG_INT_S__LEN          1
#define BMA2X2_HIGHG_INT_S__MSK          0x02
#define BMA2X2_HIGHG_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_SLOPE_INT_S__POS          2
#define BMA2X2_SLOPE_INT_S__LEN          1
#define BMA2X2_SLOPE_INT_S__MSK          0x04
#define BMA2X2_SLOPE_INT_S__REG          BMA2X2_STATUS1_REG


#define BMA2X2_SLO_NO_MOT_INT_S__POS          3
#define BMA2X2_SLO_NO_MOT_INT_S__LEN          1
#define BMA2X2_SLO_NO_MOT_INT_S__MSK          0x08
#define BMA2X2_SLO_NO_MOT_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_DOUBLE_TAP_INT_S__POS     4
#define BMA2X2_DOUBLE_TAP_INT_S__LEN     1
#define BMA2X2_DOUBLE_TAP_INT_S__MSK     0x10
#define BMA2X2_DOUBLE_TAP_INT_S__REG     BMA2X2_STATUS1_REG

#define BMA2X2_SINGLE_TAP_INT_S__POS     5
#define BMA2X2_SINGLE_TAP_INT_S__LEN     1
#define BMA2X2_SINGLE_TAP_INT_S__MSK     0x20
#define BMA2X2_SINGLE_TAP_INT_S__REG     BMA2X2_STATUS1_REG

#define BMA2X2_ORIENT_INT_S__POS         6
#define BMA2X2_ORIENT_INT_S__LEN         1
#define BMA2X2_ORIENT_INT_S__MSK         0x40
#define BMA2X2_ORIENT_INT_S__REG         BMA2X2_STATUS1_REG

#define BMA2X2_FLAT_INT_S__POS           7
#define BMA2X2_FLAT_INT_S__LEN           1
#define BMA2X2_FLAT_INT_S__MSK           0x80
#define BMA2X2_FLAT_INT_S__REG           BMA2X2_STATUS1_REG

#define BMA2X2_FIFO_FULL_INT_S__POS           5
#define BMA2X2_FIFO_FULL_INT_S__LEN           1
#define BMA2X2_FIFO_FULL_INT_S__MSK           0x20
#define BMA2X2_FIFO_FULL_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_FIFO_WM_INT_S__POS           6
#define BMA2X2_FIFO_WM_INT_S__LEN           1
#define BMA2X2_FIFO_WM_INT_S__MSK           0x40
#define BMA2X2_FIFO_WM_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_DATA_INT_S__POS           7
#define BMA2X2_DATA_INT_S__LEN           1
#define BMA2X2_DATA_INT_S__MSK           0x80
#define BMA2X2_DATA_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_SLOPE_FIRST_X__POS        0
#define BMA2X2_SLOPE_FIRST_X__LEN        1
#define BMA2X2_SLOPE_FIRST_X__MSK        0x01
#define BMA2X2_SLOPE_FIRST_X__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_FIRST_Y__POS        1
#define BMA2X2_SLOPE_FIRST_Y__LEN        1
#define BMA2X2_SLOPE_FIRST_Y__MSK        0x02
#define BMA2X2_SLOPE_FIRST_Y__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_FIRST_Z__POS        2
#define BMA2X2_SLOPE_FIRST_Z__LEN        1
#define BMA2X2_SLOPE_FIRST_Z__MSK        0x04
#define BMA2X2_SLOPE_FIRST_Z__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_SIGN_S__POS         3
#define BMA2X2_SLOPE_SIGN_S__LEN         1
#define BMA2X2_SLOPE_SIGN_S__MSK         0x08
#define BMA2X2_SLOPE_SIGN_S__REG         BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_X__POS        4
#define BMA2X2_TAP_FIRST_X__LEN        1
#define BMA2X2_TAP_FIRST_X__MSK        0x10
#define BMA2X2_TAP_FIRST_X__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_Y__POS        5
#define BMA2X2_TAP_FIRST_Y__LEN        1
#define BMA2X2_TAP_FIRST_Y__MSK        0x20
#define BMA2X2_TAP_FIRST_Y__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_Z__POS        6
#define BMA2X2_TAP_FIRST_Z__LEN        1
#define BMA2X2_TAP_FIRST_Z__MSK        0x40
#define BMA2X2_TAP_FIRST_Z__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_SIGN_S__POS         7
#define BMA2X2_TAP_SIGN_S__LEN         1
#define BMA2X2_TAP_SIGN_S__MSK         0x80
#define BMA2X2_TAP_SIGN_S__REG         BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_HIGHG_FIRST_X__POS        0
#define BMA2X2_HIGHG_FIRST_X__LEN        1
#define BMA2X2_HIGHG_FIRST_X__MSK        0x01
#define BMA2X2_HIGHG_FIRST_X__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_FIRST_Y__POS        1
#define BMA2X2_HIGHG_FIRST_Y__LEN        1
#define BMA2X2_HIGHG_FIRST_Y__MSK        0x02
#define BMA2X2_HIGHG_FIRST_Y__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_FIRST_Z__POS        2
#define BMA2X2_HIGHG_FIRST_Z__LEN        1
#define BMA2X2_HIGHG_FIRST_Z__MSK        0x04
#define BMA2X2_HIGHG_FIRST_Z__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_SIGN_S__POS         3
#define BMA2X2_HIGHG_SIGN_S__LEN         1
#define BMA2X2_HIGHG_SIGN_S__MSK         0x08
#define BMA2X2_HIGHG_SIGN_S__REG         BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_ORIENT_S__POS             4
#define BMA2X2_ORIENT_S__LEN             3
#define BMA2X2_ORIENT_S__MSK             0x70
#define BMA2X2_ORIENT_S__REG             BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_FLAT_S__POS               7
#define BMA2X2_FLAT_S__LEN               1
#define BMA2X2_FLAT_S__MSK               0x80
#define BMA2X2_FLAT_S__REG               BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_FIFO_FRAME_COUNTER_S__POS             0
#define BMA2X2_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA2X2_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA2X2_FIFO_FRAME_COUNTER_S__REG             BMA2X2_STATUS_FIFO_REG

#define BMA2X2_FIFO_OVERRUN_S__POS             7
#define BMA2X2_FIFO_OVERRUN_S__LEN             1
#define BMA2X2_FIFO_OVERRUN_S__MSK             0x80
#define BMA2X2_FIFO_OVERRUN_S__REG             BMA2X2_STATUS_FIFO_REG

#define BMA2X2_RANGE_SEL__POS             0
#define BMA2X2_RANGE_SEL__LEN             4
#define BMA2X2_RANGE_SEL__MSK             0x0F
#define BMA2X2_RANGE_SEL__REG             BMA2X2_RANGE_SEL_REG

#define BMA2X2_BANDWIDTH__POS             0
#define BMA2X2_BANDWIDTH__LEN             5
#define BMA2X2_BANDWIDTH__MSK             0x1F
#define BMA2X2_BANDWIDTH__REG             BMA2X2_BW_SEL_REG

#define BMA2X2_SLEEP_DUR__POS             1
#define BMA2X2_SLEEP_DUR__LEN             4
#define BMA2X2_SLEEP_DUR__MSK             0x1E
#define BMA2X2_SLEEP_DUR__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_MODE_CTRL__POS             5
#define BMA2X2_MODE_CTRL__LEN             3
#define BMA2X2_MODE_CTRL__MSK             0xE0
#define BMA2X2_MODE_CTRL__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_DEEP_SUSPEND__POS          5
#define BMA2X2_DEEP_SUSPEND__LEN          1
#define BMA2X2_DEEP_SUSPEND__MSK          0x20
#define BMA2X2_DEEP_SUSPEND__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_LOW_POWER__POS          6
#define BMA2X2_EN_LOW_POWER__LEN          1
#define BMA2X2_EN_LOW_POWER__MSK          0x40
#define BMA2X2_EN_LOW_POWER__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_SUSPEND__POS            7
#define BMA2X2_EN_SUSPEND__LEN            1
#define BMA2X2_EN_SUSPEND__MSK            0x80
#define BMA2X2_EN_SUSPEND__REG            BMA2X2_MODE_CTRL_REG

#define BMA2X2_SLEEP_TIMER__POS          5
#define BMA2X2_SLEEP_TIMER__LEN          1
#define BMA2X2_SLEEP_TIMER__MSK          0x20
#define BMA2X2_SLEEP_TIMER__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_LOW_POWER_MODE__POS          6
#define BMA2X2_LOW_POWER_MODE__LEN          1
#define BMA2X2_LOW_POWER_MODE__MSK          0x40
#define BMA2X2_LOW_POWER_MODE__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_EN_LOW_NOISE__POS          7
#define BMA2X2_EN_LOW_NOISE__LEN          1
#define BMA2X2_EN_LOW_NOISE__MSK          0x80
#define BMA2X2_EN_LOW_NOISE__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_DIS_SHADOW_PROC__POS       6
#define BMA2X2_DIS_SHADOW_PROC__LEN       1
#define BMA2X2_DIS_SHADOW_PROC__MSK       0x40
#define BMA2X2_DIS_SHADOW_PROC__REG       BMA2X2_DATA_CTRL_REG

#define BMA2X2_EN_DATA_HIGH_BW__POS         7
#define BMA2X2_EN_DATA_HIGH_BW__LEN         1
#define BMA2X2_EN_DATA_HIGH_BW__MSK         0x80
#define BMA2X2_EN_DATA_HIGH_BW__REG         BMA2X2_DATA_CTRL_REG

#define BMA2X2_EN_SOFT_RESET__POS         0
#define BMA2X2_EN_SOFT_RESET__LEN         8
#define BMA2X2_EN_SOFT_RESET__MSK         0xFF
#define BMA2X2_EN_SOFT_RESET__REG         BMA2X2_RESET_REG

#define BMA2X2_EN_SOFT_RESET_VALUE        0xB6

#define BMA2X2_EN_SLOPE_X_INT__POS         0
#define BMA2X2_EN_SLOPE_X_INT__LEN         1
#define BMA2X2_EN_SLOPE_X_INT__MSK         0x01
#define BMA2X2_EN_SLOPE_X_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SLOPE_Y_INT__POS         1
#define BMA2X2_EN_SLOPE_Y_INT__LEN         1
#define BMA2X2_EN_SLOPE_Y_INT__MSK         0x02
#define BMA2X2_EN_SLOPE_Y_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SLOPE_Z_INT__POS         2
#define BMA2X2_EN_SLOPE_Z_INT__LEN         1
#define BMA2X2_EN_SLOPE_Z_INT__MSK         0x04
#define BMA2X2_EN_SLOPE_Z_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_DOUBLE_TAP_INT__POS      4
#define BMA2X2_EN_DOUBLE_TAP_INT__LEN      1
#define BMA2X2_EN_DOUBLE_TAP_INT__MSK      0x10
#define BMA2X2_EN_DOUBLE_TAP_INT__REG      BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SINGLE_TAP_INT__POS      5
#define BMA2X2_EN_SINGLE_TAP_INT__LEN      1
#define BMA2X2_EN_SINGLE_TAP_INT__MSK      0x20
#define BMA2X2_EN_SINGLE_TAP_INT__REG      BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_ORIENT_INT__POS          6
#define BMA2X2_EN_ORIENT_INT__LEN          1
#define BMA2X2_EN_ORIENT_INT__MSK          0x40
#define BMA2X2_EN_ORIENT_INT__REG          BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_FLAT_INT__POS            7
#define BMA2X2_EN_FLAT_INT__LEN            1
#define BMA2X2_EN_FLAT_INT__MSK            0x80
#define BMA2X2_EN_FLAT_INT__REG            BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_HIGHG_X_INT__POS         0
#define BMA2X2_EN_HIGHG_X_INT__LEN         1
#define BMA2X2_EN_HIGHG_X_INT__MSK         0x01
#define BMA2X2_EN_HIGHG_X_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_HIGHG_Y_INT__POS         1
#define BMA2X2_EN_HIGHG_Y_INT__LEN         1
#define BMA2X2_EN_HIGHG_Y_INT__MSK         0x02
#define BMA2X2_EN_HIGHG_Y_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_HIGHG_Z_INT__POS         2
#define BMA2X2_EN_HIGHG_Z_INT__LEN         1
#define BMA2X2_EN_HIGHG_Z_INT__MSK         0x04
#define BMA2X2_EN_HIGHG_Z_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_LOWG_INT__POS            3
#define BMA2X2_EN_LOWG_INT__LEN            1
#define BMA2X2_EN_LOWG_INT__MSK            0x08
#define BMA2X2_EN_LOWG_INT__REG            BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_NEW_DATA_INT__POS        4
#define BMA2X2_EN_NEW_DATA_INT__LEN        1
#define BMA2X2_EN_NEW_DATA_INT__MSK        0x10
#define BMA2X2_EN_NEW_DATA_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_FFULL_EN_INT__POS        5
#define BMA2X2_INT_FFULL_EN_INT__LEN        1
#define BMA2X2_INT_FFULL_EN_INT__MSK        0x20
#define BMA2X2_INT_FFULL_EN_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_FWM_EN_INT__POS        6
#define BMA2X2_INT_FWM_EN_INT__LEN        1
#define BMA2X2_INT_FWM_EN_INT__MSK        0x40
#define BMA2X2_INT_FWM_EN_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__POS        0
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__MSK        0x01
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__POS        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__MSK        0x02
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__POS        2
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__MSK        0x04
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__POS        3
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__MSK        0x08
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_EN_INT1_PAD_LOWG__POS        0
#define BMA2X2_EN_INT1_PAD_LOWG__LEN        1
#define BMA2X2_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA2X2_EN_INT1_PAD_LOWG__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_HIGHG__POS       1
#define BMA2X2_EN_INT1_PAD_HIGHG__LEN       1
#define BMA2X2_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA2X2_EN_INT1_PAD_HIGHG__REG       BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SLOPE__POS       2
#define BMA2X2_EN_INT1_PAD_SLOPE__LEN       1
#define BMA2X2_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA2X2_EN_INT1_PAD_SLOPE__REG       BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__POS        3
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__LEN        1
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_DB_TAP__POS      4
#define BMA2X2_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA2X2_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA2X2_EN_INT1_PAD_DB_TAP__REG      BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA2X2_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA2X2_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA2X2_EN_INT1_PAD_SNG_TAP__REG     BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_ORIENT__POS      6
#define BMA2X2_EN_INT1_PAD_ORIENT__LEN      1
#define BMA2X2_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA2X2_EN_INT1_PAD_ORIENT__REG      BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_FLAT__POS        7
#define BMA2X2_EN_INT1_PAD_FLAT__LEN        1
#define BMA2X2_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA2X2_EN_INT1_PAD_FLAT__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_LOWG__POS        0
#define BMA2X2_EN_INT2_PAD_LOWG__LEN        1
#define BMA2X2_EN_INT2_PAD_LOWG__MSK        0x01
#define BMA2X2_EN_INT2_PAD_LOWG__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_HIGHG__POS       1
#define BMA2X2_EN_INT2_PAD_HIGHG__LEN       1
#define BMA2X2_EN_INT2_PAD_HIGHG__MSK       0x02
#define BMA2X2_EN_INT2_PAD_HIGHG__REG       BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SLOPE__POS       2
#define BMA2X2_EN_INT2_PAD_SLOPE__LEN       1
#define BMA2X2_EN_INT2_PAD_SLOPE__MSK       0x04
#define BMA2X2_EN_INT2_PAD_SLOPE__REG       BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__POS        3
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__LEN        1
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_DB_TAP__POS      4
#define BMA2X2_EN_INT2_PAD_DB_TAP__LEN      1
#define BMA2X2_EN_INT2_PAD_DB_TAP__MSK      0x10
#define BMA2X2_EN_INT2_PAD_DB_TAP__REG      BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SNG_TAP__POS     5
#define BMA2X2_EN_INT2_PAD_SNG_TAP__LEN     1
#define BMA2X2_EN_INT2_PAD_SNG_TAP__MSK     0x20
#define BMA2X2_EN_INT2_PAD_SNG_TAP__REG     BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_ORIENT__POS      6
#define BMA2X2_EN_INT2_PAD_ORIENT__LEN      1
#define BMA2X2_EN_INT2_PAD_ORIENT__MSK      0x40
#define BMA2X2_EN_INT2_PAD_ORIENT__REG      BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_FLAT__POS        7
#define BMA2X2_EN_INT2_PAD_FLAT__LEN        1
#define BMA2X2_EN_INT2_PAD_FLAT__MSK        0x80
#define BMA2X2_EN_INT2_PAD_FLAT__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_NEWDATA__POS     0
#define BMA2X2_EN_INT1_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT1_PAD_NEWDATA__MSK     0x01
#define BMA2X2_EN_INT1_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT1_PAD_FWM__POS     1
#define BMA2X2_EN_INT1_PAD_FWM__LEN     1
#define BMA2X2_EN_INT1_PAD_FWM__MSK     0x02
#define BMA2X2_EN_INT1_PAD_FWM__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT1_PAD_FFULL__POS     2
#define BMA2X2_EN_INT1_PAD_FFULL__LEN     1
#define BMA2X2_EN_INT1_PAD_FFULL__MSK     0x04
#define BMA2X2_EN_INT1_PAD_FFULL__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_FFULL__POS     5
#define BMA2X2_EN_INT2_PAD_FFULL__LEN     1
#define BMA2X2_EN_INT2_PAD_FFULL__MSK     0x20
#define BMA2X2_EN_INT2_PAD_FFULL__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_FWM__POS     6
#define BMA2X2_EN_INT2_PAD_FWM__LEN     1
#define BMA2X2_EN_INT2_PAD_FWM__MSK     0x40
#define BMA2X2_EN_INT2_PAD_FWM__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_NEWDATA__POS     7
#define BMA2X2_EN_INT2_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT2_PAD_NEWDATA__MSK     0x80
#define BMA2X2_EN_INT2_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_UNFILT_INT_SRC_LOWG__POS        0
#define BMA2X2_UNFILT_INT_SRC_LOWG__LEN        1
#define BMA2X2_UNFILT_INT_SRC_LOWG__MSK        0x01
#define BMA2X2_UNFILT_INT_SRC_LOWG__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_HIGHG__POS       1
#define BMA2X2_UNFILT_INT_SRC_HIGHG__LEN       1
#define BMA2X2_UNFILT_INT_SRC_HIGHG__MSK       0x02
#define BMA2X2_UNFILT_INT_SRC_HIGHG__REG       BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_SLOPE__POS       2
#define BMA2X2_UNFILT_INT_SRC_SLOPE__LEN       1
#define BMA2X2_UNFILT_INT_SRC_SLOPE__MSK       0x04
#define BMA2X2_UNFILT_INT_SRC_SLOPE__REG       BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__POS        3
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__LEN        1
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__MSK        0x08
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_TAP__POS         4
#define BMA2X2_UNFILT_INT_SRC_TAP__LEN         1
#define BMA2X2_UNFILT_INT_SRC_TAP__MSK         0x10
#define BMA2X2_UNFILT_INT_SRC_TAP__REG         BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_DATA__POS        5
#define BMA2X2_UNFILT_INT_SRC_DATA__LEN        1
#define BMA2X2_UNFILT_INT_SRC_DATA__MSK        0x20
#define BMA2X2_UNFILT_INT_SRC_DATA__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__POS       0
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__MSK       0x01
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__REG       BMA2X2_INT_SET_REG

#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__POS       2
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__MSK       0x04
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__REG       BMA2X2_INT_SET_REG

#define BMA2X2_INT1_PAD_OUTPUT_TYPE__POS        1
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__LEN        1
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__MSK        0x02
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__REG        BMA2X2_INT_SET_REG

#define BMA2X2_INT2_PAD_OUTPUT_TYPE__POS        3
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__LEN        1
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__MSK        0x08
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__REG        BMA2X2_INT_SET_REG

#define BMA2X2_INT_MODE_SEL__POS                0
#define BMA2X2_INT_MODE_SEL__LEN                4
#define BMA2X2_INT_MODE_SEL__MSK                0x0F
#define BMA2X2_INT_MODE_SEL__REG                BMA2X2_INT_CTRL_REG

#define BMA2X2_RESET_INT__POS           7
#define BMA2X2_RESET_INT__LEN           1
#define BMA2X2_RESET_INT__MSK           0x80
#define BMA2X2_RESET_INT__REG           BMA2X2_INT_CTRL_REG

#define BMA2X2_LOWG_DUR__POS                    0
#define BMA2X2_LOWG_DUR__LEN                    8
#define BMA2X2_LOWG_DUR__MSK                    0xFF
#define BMA2X2_LOWG_DUR__REG                    BMA2X2_LOW_DURN_REG

#define BMA2X2_LOWG_THRES__POS                  0
#define BMA2X2_LOWG_THRES__LEN                  8
#define BMA2X2_LOWG_THRES__MSK                  0xFF
#define BMA2X2_LOWG_THRES__REG                  BMA2X2_LOW_THRES_REG

#define BMA2X2_LOWG_HYST__POS                   0
#define BMA2X2_LOWG_HYST__LEN                   2
#define BMA2X2_LOWG_HYST__MSK                   0x03
#define BMA2X2_LOWG_HYST__REG                   BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_LOWG_INT_MODE__POS               2
#define BMA2X2_LOWG_INT_MODE__LEN               1
#define BMA2X2_LOWG_INT_MODE__MSK               0x04
#define BMA2X2_LOWG_INT_MODE__REG               BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_HIGHG_DUR__POS                    0
#define BMA2X2_HIGHG_DUR__LEN                    8
#define BMA2X2_HIGHG_DUR__MSK                    0xFF
#define BMA2X2_HIGHG_DUR__REG                    BMA2X2_HIGH_DURN_REG

#define BMA2X2_HIGHG_THRES__POS                  0
#define BMA2X2_HIGHG_THRES__LEN                  8
#define BMA2X2_HIGHG_THRES__MSK                  0xFF
#define BMA2X2_HIGHG_THRES__REG                  BMA2X2_HIGH_THRES_REG

#define BMA2X2_HIGHG_HYST__POS                  6
#define BMA2X2_HIGHG_HYST__LEN                  2
#define BMA2X2_HIGHG_HYST__MSK                  0xC0
#define BMA2X2_HIGHG_HYST__REG                  BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_SLOPE_DUR__POS                    0
#define BMA2X2_SLOPE_DUR__LEN                    2
#define BMA2X2_SLOPE_DUR__MSK                    0x03
#define BMA2X2_SLOPE_DUR__REG                    BMA2X2_SLOPE_DURN_REG

#define BMA2X2_SLO_NO_MOT_DUR__POS                    2
#define BMA2X2_SLO_NO_MOT_DUR__LEN                    6
#define BMA2X2_SLO_NO_MOT_DUR__MSK                    0xFC
#define BMA2X2_SLO_NO_MOT_DUR__REG                    BMA2X2_SLOPE_DURN_REG

#define BMA2X2_SLOPE_THRES__POS                  0
#define BMA2X2_SLOPE_THRES__LEN                  8
#define BMA2X2_SLOPE_THRES__MSK                  0xFF
#define BMA2X2_SLOPE_THRES__REG                  BMA2X2_SLOPE_THRES_REG

#define BMA2X2_SLO_NO_MOT_THRES__POS                  0
#define BMA2X2_SLO_NO_MOT_THRES__LEN                  8
#define BMA2X2_SLO_NO_MOT_THRES__MSK                  0xFF
#define BMA2X2_SLO_NO_MOT_THRES__REG          BMA2X2_SLO_NO_MOT_THRES_REG

#define BMA2X2_TAP_DUR__POS                    0
#define BMA2X2_TAP_DUR__LEN                    3
#define BMA2X2_TAP_DUR__MSK                    0x07
#define BMA2X2_TAP_DUR__REG                    BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_SHOCK_DURN__POS             6
#define BMA2X2_TAP_SHOCK_DURN__LEN             1
#define BMA2X2_TAP_SHOCK_DURN__MSK             0x40
#define BMA2X2_TAP_SHOCK_DURN__REG             BMA2X2_TAP_PARAM_REG

#define BMA2X2_ADV_TAP_INT__POS                5
#define BMA2X2_ADV_TAP_INT__LEN                1
#define BMA2X2_ADV_TAP_INT__MSK                0x20
#define BMA2X2_ADV_TAP_INT__REG                BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_QUIET_DURN__POS             7
#define BMA2X2_TAP_QUIET_DURN__LEN             1
#define BMA2X2_TAP_QUIET_DURN__MSK             0x80
#define BMA2X2_TAP_QUIET_DURN__REG             BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_THRES__POS                  0
#define BMA2X2_TAP_THRES__LEN                  5
#define BMA2X2_TAP_THRES__MSK                  0x1F
#define BMA2X2_TAP_THRES__REG                  BMA2X2_TAP_THRES_REG

#define BMA2X2_TAP_SAMPLES__POS                6
#define BMA2X2_TAP_SAMPLES__LEN                2
#define BMA2X2_TAP_SAMPLES__MSK                0xC0
#define BMA2X2_TAP_SAMPLES__REG                BMA2X2_TAP_THRES_REG

#define BMA2X2_ORIENT_MODE__POS                  0
#define BMA2X2_ORIENT_MODE__LEN                  2
#define BMA2X2_ORIENT_MODE__MSK                  0x03
#define BMA2X2_ORIENT_MODE__REG                  BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_BLOCK__POS                 2
#define BMA2X2_ORIENT_BLOCK__LEN                 2
#define BMA2X2_ORIENT_BLOCK__MSK                 0x0C
#define BMA2X2_ORIENT_BLOCK__REG                 BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_HYST__POS                  4
#define BMA2X2_ORIENT_HYST__LEN                  3
#define BMA2X2_ORIENT_HYST__MSK                  0x70
#define BMA2X2_ORIENT_HYST__REG                  BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_AXIS__POS                  7
#define BMA2X2_ORIENT_AXIS__LEN                  1
#define BMA2X2_ORIENT_AXIS__MSK                  0x80
#define BMA2X2_ORIENT_AXIS__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_ORIENT_UD_EN__POS                  6
#define BMA2X2_ORIENT_UD_EN__LEN                  1
#define BMA2X2_ORIENT_UD_EN__MSK                  0x40
#define BMA2X2_ORIENT_UD_EN__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_THETA_BLOCK__POS                  0
#define BMA2X2_THETA_BLOCK__LEN                  6
#define BMA2X2_THETA_BLOCK__MSK                  0x3F
#define BMA2X2_THETA_BLOCK__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_THETA_FLAT__POS                  0
#define BMA2X2_THETA_FLAT__LEN                  6
#define BMA2X2_THETA_FLAT__MSK                  0x3F
#define BMA2X2_THETA_FLAT__REG                  BMA2X2_THETA_FLAT_REG

#define BMA2X2_FLAT_HOLD_TIME__POS              4
#define BMA2X2_FLAT_HOLD_TIME__LEN              2
#define BMA2X2_FLAT_HOLD_TIME__MSK              0x30
#define BMA2X2_FLAT_HOLD_TIME__REG              BMA2X2_FLAT_HOLD_TIME_REG

#define BMA2X2_FLAT_HYS__POS                   0
#define BMA2X2_FLAT_HYS__LEN                   3
#define BMA2X2_FLAT_HYS__MSK                   0x07
#define BMA2X2_FLAT_HYS__REG                   BMA2X2_FLAT_HOLD_TIME_REG

#define BMA2X2_FIFO_WML_TRIG_RETAIN__POS                   0
#define BMA2X2_FIFO_WML_TRIG_RETAIN__LEN                   6
#define BMA2X2_FIFO_WML_TRIG_RETAIN__MSK                   0x3F
#define BMA2X2_FIFO_WML_TRIG_RETAIN__REG                   BMA2X2_FIFO_WML_TRIG

#define BMA2X2_EN_SELF_TEST__POS                0
#define BMA2X2_EN_SELF_TEST__LEN                2
#define BMA2X2_EN_SELF_TEST__MSK                0x03
#define BMA2X2_EN_SELF_TEST__REG                BMA2X2_SELF_TEST_REG

#define BMA2X2_NEG_SELF_TEST__POS               2
#define BMA2X2_NEG_SELF_TEST__LEN               1
#define BMA2X2_NEG_SELF_TEST__MSK               0x04
#define BMA2X2_NEG_SELF_TEST__REG               BMA2X2_SELF_TEST_REG

#define BMA2X2_SELF_TEST_AMP__POS               4
#define BMA2X2_SELF_TEST_AMP__LEN               3
#define BMA2X2_SELF_TEST_AMP__MSK               0x70
#define BMA2X2_SELF_TEST_AMP__REG               BMA2X2_SELF_TEST_REG

#define BMA2X2_UNLOCK_EE_PROG_MODE__POS     0
#define BMA2X2_UNLOCK_EE_PROG_MODE__LEN     1
#define BMA2X2_UNLOCK_EE_PROG_MODE__MSK     0x01
#define BMA2X2_UNLOCK_EE_PROG_MODE__REG     BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_START_EE_PROG_TRIG__POS      1
#define BMA2X2_START_EE_PROG_TRIG__LEN      1
#define BMA2X2_START_EE_PROG_TRIG__MSK      0x02
#define BMA2X2_START_EE_PROG_TRIG__REG      BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EE_PROG_READY__POS          2
#define BMA2X2_EE_PROG_READY__LEN          1
#define BMA2X2_EE_PROG_READY__MSK          0x04
#define BMA2X2_EE_PROG_READY__REG          BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_UPDATE_IMAGE__POS                3
#define BMA2X2_UPDATE_IMAGE__LEN                1
#define BMA2X2_UPDATE_IMAGE__MSK                0x08
#define BMA2X2_UPDATE_IMAGE__REG                BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EE_REMAIN__POS                4
#define BMA2X2_EE_REMAIN__LEN                4
#define BMA2X2_EE_REMAIN__MSK                0xF0
#define BMA2X2_EE_REMAIN__REG                BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EN_SPI_MODE_3__POS              0
#define BMA2X2_EN_SPI_MODE_3__LEN              1
#define BMA2X2_EN_SPI_MODE_3__MSK              0x01
#define BMA2X2_EN_SPI_MODE_3__REG              BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_I2C_WATCHDOG_PERIOD__POS        1
#define BMA2X2_I2C_WATCHDOG_PERIOD__LEN        1
#define BMA2X2_I2C_WATCHDOG_PERIOD__MSK        0x02
#define BMA2X2_I2C_WATCHDOG_PERIOD__REG        BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_EN_I2C_WATCHDOG__POS            2
#define BMA2X2_EN_I2C_WATCHDOG__LEN            1
#define BMA2X2_EN_I2C_WATCHDOG__MSK            0x04
#define BMA2X2_EN_I2C_WATCHDOG__REG            BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_EXT_MODE__POS              7
#define BMA2X2_EXT_MODE__LEN              1
#define BMA2X2_EXT_MODE__MSK              0x80
#define BMA2X2_EXT_MODE__REG              BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_ALLOW_UPPER__POS        6
#define BMA2X2_ALLOW_UPPER__LEN        1
#define BMA2X2_ALLOW_UPPER__MSK        0x40
#define BMA2X2_ALLOW_UPPER__REG        BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_MAP_2_LOWER__POS            5
#define BMA2X2_MAP_2_LOWER__LEN            1
#define BMA2X2_MAP_2_LOWER__MSK            0x20
#define BMA2X2_MAP_2_LOWER__REG            BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_MAGIC_NUMBER__POS            0
#define BMA2X2_MAGIC_NUMBER__LEN            5
#define BMA2X2_MAGIC_NUMBER__MSK            0x1F
#define BMA2X2_MAGIC_NUMBER__REG            BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_UNLOCK_EE_WRITE_TRIM__POS        4
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__LEN        4
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__MSK        0xF0
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__REG        BMA2X2_CTRL_UNLOCK_REG

#define BMA2X2_EN_SLOW_COMP_X__POS              0
#define BMA2X2_EN_SLOW_COMP_X__LEN              1
#define BMA2X2_EN_SLOW_COMP_X__MSK              0x01
#define BMA2X2_EN_SLOW_COMP_X__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_EN_SLOW_COMP_Y__POS              1
#define BMA2X2_EN_SLOW_COMP_Y__LEN              1
#define BMA2X2_EN_SLOW_COMP_Y__MSK              0x02
#define BMA2X2_EN_SLOW_COMP_Y__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_EN_SLOW_COMP_Z__POS              2
#define BMA2X2_EN_SLOW_COMP_Z__LEN              1
#define BMA2X2_EN_SLOW_COMP_Z__MSK              0x04
#define BMA2X2_EN_SLOW_COMP_Z__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_FAST_CAL_RDY_S__POS             4
#define BMA2X2_FAST_CAL_RDY_S__LEN             1
#define BMA2X2_FAST_CAL_RDY_S__MSK             0x10
#define BMA2X2_FAST_CAL_RDY_S__REG             BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_CAL_TRIGGER__POS                5
#define BMA2X2_CAL_TRIGGER__LEN                2
#define BMA2X2_CAL_TRIGGER__MSK                0x60
#define BMA2X2_CAL_TRIGGER__REG                BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_RESET_OFFSET_REGS__POS           7
#define BMA2X2_RESET_OFFSET_REGS__LEN           1
#define BMA2X2_RESET_OFFSET_REGS__MSK           0x80
#define BMA2X2_RESET_OFFSET_REGS__REG           BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_COMP_CUTOFF__POS                 0
#define BMA2X2_COMP_CUTOFF__LEN                 1
#define BMA2X2_COMP_CUTOFF__MSK                 0x01
#define BMA2X2_COMP_CUTOFF__REG                 BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_X__POS        1
#define BMA2X2_COMP_TARGET_OFFSET_X__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA2X2_COMP_TARGET_OFFSET_X__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Y__POS        3
#define BMA2X2_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA2X2_COMP_TARGET_OFFSET_Y__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Z__POS        5
#define BMA2X2_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA2X2_COMP_TARGET_OFFSET_Z__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_FIFO_DATA_SELECT__POS                 0
#define BMA2X2_FIFO_DATA_SELECT__LEN                 2
#define BMA2X2_FIFO_DATA_SELECT__MSK                 0x03
#define BMA2X2_FIFO_DATA_SELECT__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_TRIGGER_SOURCE__POS                 2
#define BMA2X2_FIFO_TRIGGER_SOURCE__LEN                 2
#define BMA2X2_FIFO_TRIGGER_SOURCE__MSK                 0x0C
#define BMA2X2_FIFO_TRIGGER_SOURCE__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_TRIGGER_ACTION__POS                 4
#define BMA2X2_FIFO_TRIGGER_ACTION__LEN                 2
#define BMA2X2_FIFO_TRIGGER_ACTION__MSK                 0x30
#define BMA2X2_FIFO_TRIGGER_ACTION__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_MODE__POS                 6
#define BMA2X2_FIFO_MODE__LEN                 2
#define BMA2X2_FIFO_MODE__MSK                 0xC0
#define BMA2X2_FIFO_MODE__REG                 BMA2X2_FIFO_MODE_REG


#define BMA2X2_STATUS1                             0
#define BMA2X2_STATUS2                             1
#define BMA2X2_STATUS3                             2
#define BMA2X2_STATUS4                             3
#define BMA2X2_STATUS5                             4


#define BMA2X2_RANGE_2G                 3
#define BMA2X2_RANGE_4G                 5
#define BMA2X2_RANGE_8G                 8
#define BMA2X2_RANGE_16G                12


#define BMA2X2_BW_7_81HZ        0x08
#define BMA2X2_BW_15_63HZ       0x09
#define BMA2X2_BW_31_25HZ       0x0A
#define BMA2X2_BW_62_50HZ       0x0B
#define BMA2X2_BW_125HZ         0x0C
#define BMA2X2_BW_250HZ         0x0D
#define BMA2X2_BW_500HZ         0x0E
#define BMA2X2_BW_1000HZ        0x0F

#define BMA2X2_SLEEP_DUR_0_5MS        0x05
#define BMA2X2_SLEEP_DUR_1MS          0x06
#define BMA2X2_SLEEP_DUR_2MS          0x07
#define BMA2X2_SLEEP_DUR_4MS          0x08
#define BMA2X2_SLEEP_DUR_6MS          0x09
#define BMA2X2_SLEEP_DUR_10MS         0x0A
#define BMA2X2_SLEEP_DUR_25MS         0x0B
#define BMA2X2_SLEEP_DUR_50MS         0x0C
#define BMA2X2_SLEEP_DUR_100MS        0x0D
#define BMA2X2_SLEEP_DUR_500MS        0x0E
#define BMA2X2_SLEEP_DUR_1S           0x0F

#define BMA2X2_LATCH_DUR_NON_LATCH    0x00
#define BMA2X2_LATCH_DUR_250MS        0x01
#define BMA2X2_LATCH_DUR_500MS        0x02
#define BMA2X2_LATCH_DUR_1S           0x03
#define BMA2X2_LATCH_DUR_2S           0x04
#define BMA2X2_LATCH_DUR_4S           0x05
#define BMA2X2_LATCH_DUR_8S           0x06
#define BMA2X2_LATCH_DUR_LATCH        0x07
#define BMA2X2_LATCH_DUR_NON_LATCH1   0x08
#define BMA2X2_LATCH_DUR_250US        0x09
#define BMA2X2_LATCH_DUR_500US        0x0A
#define BMA2X2_LATCH_DUR_1MS          0x0B
#define BMA2X2_LATCH_DUR_12_5MS       0x0C
#define BMA2X2_LATCH_DUR_25MS         0x0D
#define BMA2X2_LATCH_DUR_50MS         0x0E
#define BMA2X2_LATCH_DUR_LATCH1       0x0F

#define BMA2X2_MODE_NORMAL             0
#define BMA2X2_MODE_LOWPOWER1          1
#define BMA2X2_MODE_SUSPEND            2
#define BMA2X2_MODE_DEEP_SUSPEND       3
#define BMA2X2_MODE_LOWPOWER2          4
#define BMA2X2_MODE_STANDBY            5

#define BMA2X2_X_AXIS           0
#define BMA2X2_Y_AXIS           1
#define BMA2X2_Z_AXIS           2

#define BMA2X2_Low_G_Interrupt       0
#define BMA2X2_High_G_X_Interrupt    1
#define BMA2X2_High_G_Y_Interrupt    2
#define BMA2X2_High_G_Z_Interrupt    3
#define BMA2X2_DATA_EN               4
#define BMA2X2_Slope_X_Interrupt     5
#define BMA2X2_Slope_Y_Interrupt     6
#define BMA2X2_Slope_Z_Interrupt     7
#define BMA2X2_Single_Tap_Interrupt  8
#define BMA2X2_Double_Tap_Interrupt  9
#define BMA2X2_Orient_Interrupt      10
#define BMA2X2_Flat_Interrupt        11
#define BMA2X2_FFULL_INTERRUPT       12
#define BMA2X2_FWM_INTERRUPT         13

#define BMA2X2_INT1_LOWG         0
#define BMA2X2_INT2_LOWG         1
#define BMA2X2_INT1_HIGHG        0
#define BMA2X2_INT2_HIGHG        1
#define BMA2X2_INT1_SLOPE        0
#define BMA2X2_INT2_SLOPE        1
#define BMA2X2_INT1_SLO_NO_MOT   0
#define BMA2X2_INT2_SLO_NO_MOT   1
#define BMA2X2_INT1_DTAP         0
#define BMA2X2_INT2_DTAP         1
#define BMA2X2_INT1_STAP         0
#define BMA2X2_INT2_STAP         1
#define BMA2X2_INT1_ORIENT       0
#define BMA2X2_INT2_ORIENT       1
#define BMA2X2_INT1_FLAT         0
#define BMA2X2_INT2_FLAT         1
#define BMA2X2_INT1_NDATA        0
#define BMA2X2_INT2_NDATA        1
#define BMA2X2_INT1_FWM          0
#define BMA2X2_INT2_FWM          1
#define BMA2X2_INT1_FFULL        0
#define BMA2X2_INT2_FFULL        1

#define BMA2X2_SRC_LOWG         0
#define BMA2X2_SRC_HIGHG        1
#define BMA2X2_SRC_SLOPE        2
#define BMA2X2_SRC_SLO_NO_MOT   3
#define BMA2X2_SRC_TAP          4
#define BMA2X2_SRC_DATA         5

#define BMA2X2_INT1_OUTPUT      0
#define BMA2X2_INT2_OUTPUT      1
#define BMA2X2_INT1_LEVEL       0
#define BMA2X2_INT2_LEVEL       1

#define BMA2X2_LOW_DURATION            0
#define BMA2X2_HIGH_DURATION           1
#define BMA2X2_SLOPE_DURATION          2
#define BMA2X2_SLO_NO_MOT_DURATION     3

#define BMA2X2_LOW_THRESHOLD            0
#define BMA2X2_HIGH_THRESHOLD           1
#define BMA2X2_SLOPE_THRESHOLD          2
#define BMA2X2_SLO_NO_MOT_THRESHOLD     3


#define BMA2X2_LOWG_HYST                0
#define BMA2X2_HIGHG_HYST               1

#define BMA2X2_ORIENT_THETA             0
#define BMA2X2_FLAT_THETA               1

#define BMA2X2_I2C_SELECT               0
#define BMA2X2_I2C_EN                   1

#define BMA2X2_SLOW_COMP_X              0
#define BMA2X2_SLOW_COMP_Y              1
#define BMA2X2_SLOW_COMP_Z              2

#define BMA2X2_CUT_OFF                  0
#define BMA2X2_OFFSET_TRIGGER_X         1
#define BMA2X2_OFFSET_TRIGGER_Y         2
#define BMA2X2_OFFSET_TRIGGER_Z         3

#define BMA2X2_GP0                      0
#define BMA2X2_GP1                      1

#define BMA2X2_SLO_NO_MOT_EN_X          0
#define BMA2X2_SLO_NO_MOT_EN_Y          1
#define BMA2X2_SLO_NO_MOT_EN_Z          2
#define BMA2X2_SLO_NO_MOT_EN_SEL        3

#define BMA2X2_WAKE_UP_DUR_20MS         0
#define BMA2X2_WAKE_UP_DUR_80MS         1
#define BMA2X2_WAKE_UP_DUR_320MS                2
#define BMA2X2_WAKE_UP_DUR_2560MS               3

#define BMA2X2_SELF_TEST0_ON            1
#define BMA2X2_SELF_TEST1_ON            2

#define BMA2X2_EE_W_OFF                 0
#define BMA2X2_EE_W_ON                  1

#define BMA2X2_DDTAP_INT_ON             1
#define BMA2X2_DDTAP_INT_OFF            0

#define BMA2X2_NEWDATA_INT_ON           1
#define BMA2X2_NEWDATA_INT_OFF          0

#define BMA2X2_LOW_TH_IN_G(gthres, range)           ((256 * gthres) / range)


#define BMA2X2_HIGH_TH_IN_G(gthres, range)          ((256 * gthres) / range)


#define BMA2X2_LOW_HY_IN_G(ghyst, range)            ((32 * ghyst) / range)


#define BMA2X2_HIGH_HY_IN_G(ghyst, range)           ((32 * ghyst) / range)


#define BMA2X2_SLOPE_TH_IN_G(gthres, range)    ((128 * gthres) / range)


#define BMA2X2_GET_BITSLICE(regvar, bitname)\
    ((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA2X2_SET_BITSLICE(regvar, bitname, val)\
    ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define BMA_KMSG_ERR
#define BMA_KMSG_INF

#ifdef BMA_KMSG_ERR
#define KMSGERR(format, ...)    dev_err(format, ## __VA_ARGS__)
#else
#define KMSGERR(format, ...)
#endif

#ifdef BMA_KMSG_INF
#define KMSGINF(format, ...)    dev_info(format, ## __VA_ARGS__)
#else
#define KMSGINF(format, ...)
#endif

#define BMA255_CHIP_ID 0xFA
#define BMA250E_CHIP_ID 0xF9
#define BMA222E_CHIP_ID 0xF8
#define BMA280_CHIP_ID 0xFB

#define BMA255_TYPE 0
#define BMA250E_TYPE 1
#define BMA222E_TYPE 2
#define BMA280_TYPE 3

#define MAX_FIFO_F_LEVEL 32
#define MAX_FIFO_F_BYTES 6
#define BMA_MAX_RETRY_I2C_XFER (100)
#define NEWDATA_INT_TIME_MS 8
#define DOUBLE_TAP_Z_NAG_MASK 0xc0
#define DOUBLE_TAP_Z_POS_MASK 0x40

#define DOUBLE_TAP_INT_TRI   0x10
#define SINGLE_TAP_INT_TRI  0x20

#define BMA2X2_SAMPLE_READY_TIME_MS  100
#define BMA2X2_SELFTEST_SAMPLE_COUNT 10
#define DIFFERENCE(a, b)  (a>b ? (a-b): (b-a))
#define BMA2X2_ST_X_LOW_LIMIT            51    /* 800 / HAL_SENSOR_ACCEL_8G_EACH_LSB_BMA250 */
#define BMA2X2_ST_Y_LOW_LIMIT            51    /* 800 / HAL_SENSOR_ACCEL_8G_EACH_LSB_BMA250 */
#define BMA2X2_ST_Z_LOW_LIMIT            26    /* 400 / HAL_SENSOR_ACCEL_8G_EACH_LSB_BMA250 */
#define TRUE    1
#define FALSE   0
#define MASK_16_10_BITS   0xFFC0

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
enum
{
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
};
enum
{
    POSITIVE = 0,
    NEGATIVE
};

enum {
  PAD_SEL_INT1_LOWG = 0,
  PAD_SEL_INT1_GIGHG,
  PAD_SEL_INT1_SLOPE,
  PAD_SEL_INT1_DBTAP,
  PAD_SEL_INT1_SNGTAP,
  PAD_SEL_INT1_ORIENT,
  PAD_SEL_INT1_FLAT,
  PAD_SEL_INT1_DATAREADY,
  PAD_SEL_INT1_ALL,
};

enum {
    ACCEL_I2C_RESUME = 0,
    ACCEL_I2C_SUSPEND,
};

static s32 selftest_data_positive = 0;
static s32 selftest_data_negative = 0;
static struct bma2x2_data  *bma2x2_accel;

struct bma2x2acc{
    s16 x,
        y,
        z;
} ;

struct bma2x2_data {
    struct i2c_client *bma2x2_client;
    atomic_t delay;
    atomic_t enable;
    atomic_t enable_ddtap;
    unsigned int chip_id;
    unsigned char mode;
    signed char sensor_type;
    struct input_dev *input;
    struct input_dev *ddtap;
    struct bma2x2acc value;
    struct mutex value_mutex;
    struct mutex enable_mutex;
    struct mutex enable_ddtap_mutex;
    struct mutex mode_mutex;
    struct delayed_work work;
    struct work_struct irq_work;
    u8 axis_map_x;
    u8 axis_map_y;
    u8 axis_map_z;
    bool negate_x;
    bool negate_y;
    bool negate_z;
    int IRQ;
    struct bma2x2_acc_platform_data *pdata;
    atomic_t selftest_int;
    atomic_t selftest_ongoing;
    wait_queue_head_t selftest_wq;
    bool test;
    struct delayed_work test_work;
    struct workqueue_struct *test_queue;
    atomic_t suspend;
};
struct bma2x2_acc_platform_data {
    unsigned int poll_interval;
    unsigned int min_interval;
    u8 accel_range;
    u8 accel_direction;
    int gpio_int1;
    int gpio_int2;
};

static int bma2x2_set_bandwidth(struct i2c_client *client, unsigned char BW);
static int bma2x2_set_mode(struct i2c_client *client, unsigned char Mode);

static void bma2x2_i2c_resume(struct i2c_client *client)
{
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    do {
        if (atomic_read(&bma2x2->suspend) == ACCEL_I2C_RESUME)
            break;
        else {  /* ACCEL_i2C_SUSPEND */
            KMSGINF(&client->dev, "Accel, %s, suspended state is %d\n", __func__, atomic_read(&bma2x2->suspend));
            msleep(10);
        }
    } while(1);
}

static int bma2x2_smbus_read_byte(struct i2c_client *client,
        unsigned char reg_addr, unsigned char *data)
{
    s32 dummy;
    u8 index;
    for(index = 0; index<10; index++)
    {
       dummy = i2c_smbus_read_byte_data(client, reg_addr);
       if((dummy > 0) || (dummy == 0))
          break;
       else{
           KMSGERR(&client->dev, "Accel, bma2x2_smbus_read_byte, reg_addr = 0x%x, dummy = %d, index = %u\n", reg_addr,dummy,index);
           msleep(10);
       }
    }
    *data = dummy & 0x000000ff;
    return dummy;
}

static int bma2x2_smbus_write_byte(struct i2c_client *client,
        unsigned char reg_addr, unsigned char *data)
{
    s32 dummy;
    u8 index;
    for(index = 0; index<10; index++)
    {
      dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
      if((dummy > 0) || (dummy == 0))
        break;
      else{
        KMSGERR(&client->dev, "Accel, bma2x2_smbus_write_byte, reg_addr = 0x%x, data = 0x%x, dummy = %d, index = %u\n", reg_addr,*data,dummy,index);
        msleep(10);
      }
    }
    return dummy;
}

static int bma2x2_smbus_read_byte_block(struct i2c_client *client,
        unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy;
    u8 index;
    for(index = 0; index<10; index++)
    {
        dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
        if((dummy > 0) || (dummy == 0))
            break;
        else{
          KMSGERR(&client->dev, "Accel, bma2x2_smbus_read_byte_block, reg_addr = 0x%x, data = 0x%x, dummy = %d, index = %u\n, len = 0x%x", reg_addr,*data,dummy,index,len);
          msleep(10);
        }
    }
    return dummy;
}

#ifdef BMA2X2_ENABLE_INT1
static int bma2x2_set_int1_pad_sel(struct i2c_client *client, unsigned char
        int1sel)
{
    int comres = 0;
    unsigned char data;
    unsigned char state;
    state = 0x01;

    switch (int1sel) {
    case PAD_SEL_INT1_LOWG:
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_EN_INT1_PAD_LOWG__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_LOWG,
                state);
        comres = bma2x2_smbus_write_byte(client,
                BMA2X2_EN_INT1_PAD_LOWG__REG, &data);
        break;
    case PAD_SEL_INT1_GIGHG:
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_EN_INT1_PAD_HIGHG__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_HIGHG,
                state);
        comres = bma2x2_smbus_write_byte(client,
                BMA2X2_EN_INT1_PAD_HIGHG__REG, &data);
        break;
    case PAD_SEL_INT1_SLOPE:
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_EN_INT1_PAD_SLOPE__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_SLOPE,
                state);
        comres = bma2x2_smbus_write_byte(client,
                BMA2X2_EN_INT1_PAD_SLOPE__REG, &data);
        break;
    case PAD_SEL_INT1_DBTAP:
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_EN_INT1_PAD_DB_TAP__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_DB_TAP,
                state);
        comres = bma2x2_smbus_write_byte(client,
                BMA2X2_EN_INT1_PAD_DB_TAP__REG, &data);
        break;
    case PAD_SEL_INT1_SNGTAP:
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_EN_INT1_PAD_SNG_TAP__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_SNG_TAP,
                state);
        comres = bma2x2_smbus_write_byte(client,
                BMA2X2_EN_INT1_PAD_SNG_TAP__REG, &data);
        break;
    case PAD_SEL_INT1_ORIENT:
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_EN_INT1_PAD_ORIENT__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_ORIENT,
                state);
        comres = bma2x2_smbus_write_byte(client,
                BMA2X2_EN_INT1_PAD_ORIENT__REG, &data);
        break;
    case PAD_SEL_INT1_FLAT:
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_EN_INT1_PAD_FLAT__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_FLAT,
                state);
        comres = bma2x2_smbus_write_byte(client,
                BMA2X2_EN_INT1_PAD_FLAT__REG, &data);
        break;
    case PAD_SEL_INT1_DATAREADY:
        comres = bma2x2_smbus_read_byte(client,BMA2X2_EN_INT1_PAD_NEWDATA__REG, &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_NEWDATA,
               state);
        comres = bma2x2_smbus_write_byte(client,
        BMA2X2_EN_INT1_PAD_NEWDATA__REG, &data);
        break;
    default:
        break;
    }
    return comres;
}
#endif /* BMA2X2_ENABLE_INT1 */
static int bma2x2_set_int_enable(struct device *dev, unsigned char value)
{
    int comres = 0;
    unsigned char data1;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    int pre_int_enable = atomic_read(&bma2x2->enable_ddtap);
    int pre_enable = atomic_read(&bma2x2->enable);
    KMSGINF(&client->dev, "Accel,bma2x2_set_int_enable start\n");
    KMSGINF(&client->dev, "Accel,bma2x2_set_int_enable pre_int_enable:%d,pre_enable:%d\n",pre_int_enable,pre_enable);
    bma2x2_i2c_resume(bma2x2->bma2x2_client);
    mutex_lock(&bma2x2->enable_ddtap_mutex);
    if(value){
        if(pre_int_enable == 0){
            if (pre_enable == 0) {
                bma2x2_set_mode(bma2x2->bma2x2_client,
                    BMA2X2_MODE_NORMAL);
            }
            enable_irq(client->irq);
            bma2x2_set_bandwidth(client, BMA2X2_BW_1000HZ);
            comres = bma2x2_smbus_read_byte(client, BMA2X2_INT_ENABLE1_REG, &data1);
            if(comres < 0)
            {
                mutex_unlock(&bma2x2->enable_ddtap_mutex);
                return comres;
            }
            data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_DOUBLE_TAP_INT,BMA2X2_DDTAP_INT_ON);
            comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE1_REG,&data1);
            atomic_set(&bma2x2->enable_ddtap, 1);
            enable_irq_wake(client->irq);
        }

    }else {
        if(pre_int_enable == 1){
            if (pre_enable == 0) {
                bma2x2_set_mode(bma2x2->bma2x2_client,
                    BMA2X2_MODE_SUSPEND);
            }
            disable_irq_nosync(client->irq);
            bma2x2_set_bandwidth(client, BMA2X2_BW_SET);
            comres = bma2x2_smbus_read_byte(client, BMA2X2_INT_ENABLE1_REG, &data1);
            if(comres < 0)
            {
                mutex_unlock(&bma2x2->enable_ddtap_mutex);
                return comres;
            }
            data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_DOUBLE_TAP_INT,BMA2X2_DDTAP_INT_OFF);
            comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE1_REG,&data1);
            atomic_set(&bma2x2->enable_ddtap, 0);
            disable_irq_wake(client->irq);
            }
    }
    mutex_unlock(&bma2x2->enable_ddtap_mutex);
    return comres;
}

#if defined(BMA2X2_ENABLE_INT1)
static int bma2x2_get_interruptstatus1(struct i2c_client *client, unsigned char
        *intstatus)
{
    int comres = 0;
    unsigned char data;

    comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS1_REG, &data);
    *intstatus = data;

    return comres;
}
static int bma2x2_get_interruptstatus2(struct i2c_client *client, unsigned char
        *intstatus)
{
    int comres = 0;
    unsigned char data;

    comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS_TAP_SLOPE_REG, &data);
    *intstatus = data;

    return comres;
}
#endif /* defined(BMA2X2_ENABLE_INT1)*/
static int bma2x2_set_Int_Mode(struct i2c_client *client, unsigned char Mode)
{
    int comres = 0;
    unsigned char data;


    comres = bma2x2_smbus_read_byte(client,
            BMA2X2_INT_MODE_SEL__REG, &data);
    if(comres < 0)
        return comres;
    data = BMA2X2_SET_BITSLICE(data, BMA2X2_INT_MODE_SEL, Mode);
    comres = bma2x2_smbus_write_byte(client,
            BMA2X2_INT_MODE_SEL__REG, &data);
    return comres;
}

static int bma2x2_get_Int_Mode(struct i2c_client *client, unsigned char *Mode)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client,
            BMA2X2_INT_MODE_SEL__REG, &data);
    if(comres < 0)
        return comres;
    data  = BMA2X2_GET_BITSLICE(data, BMA2X2_INT_MODE_SEL);
    *Mode = data;
    return comres;
}
static int bma2x2_set_tap_duration(struct i2c_client *client, unsigned char
        duration)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_DUR__REG, &data);
    if(comres < 0)
        return comres;
    data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_DUR, duration);
    comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_DUR__REG, &data);
    return comres;
}

static int bma2x2_get_tap_duration(struct i2c_client *client, unsigned char
        *status)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_PARAM_REG, &data);
    data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_DUR);
    *status = data;
    return comres;
}

static int bma2x2_set_tap_shock(struct i2c_client *client, unsigned char setval)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_SHOCK_DURN__REG,
            &data);
    if(comres < 0)
       return comres;
    data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_SHOCK_DURN, setval);
    comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_SHOCK_DURN__REG,
            &data);
    return comres;
}

static int bma2x2_get_tap_shock(struct i2c_client *client, unsigned char
        *status)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_PARAM_REG, &data);
    data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_SHOCK_DURN);
    *status = data;
    return comres;
}

static int bma2x2_set_tap_quiet(struct i2c_client *client, unsigned char
        duration)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_QUIET_DURN__REG,
            &data);
    if(comres < 0)
        return comres;
    data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_QUIET_DURN, duration);
    comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_QUIET_DURN__REG,
            &data);
    return comres;
}

static int bma2x2_get_tap_quiet(struct i2c_client *client, unsigned char
        *status)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_PARAM_REG, &data);
    data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_QUIET_DURN);
    *status = data;
    return comres;
}

static int bma2x2_set_tap_threshold(struct i2c_client *client, unsigned char
        threshold)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES__REG, &data);
    if(comres < 0)
        return comres;
    data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_THRES, threshold);
    comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_THRES__REG, &data);
    return comres;
}

static int bma2x2_get_tap_threshold(struct i2c_client *client, unsigned char
        *status)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES_REG, &data);
    data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_THRES);
    *status = data;
    return comres;
}

static int bma2x2_set_tap_samp(struct i2c_client *client, unsigned char samp)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_SAMPLES__REG, &data);
    if(comres < 0)
       return comres;
    data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_SAMPLES, samp);
    comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_SAMPLES__REG,
            &data);
    return comres;
}

static int bma2x2_get_tap_samp(struct i2c_client *client, unsigned char *status)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES_REG, &data);
    data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_SAMPLES);
    *status = data;
    return comres;
}
static int bma2x2_set_mode(struct i2c_client *client, unsigned char Mode)
{
    int comres = 0;
    unsigned char data1, data2;

    if (Mode < 6) {
        comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG,
                &data1);
        if(comres < 0)
            return comres;
        comres = bma2x2_smbus_read_byte(client,
                BMA2X2_LOW_NOISE_CTRL_REG,
                &data2);
        if(comres < 0)
            return comres;
        switch (Mode) {
        case BMA2X2_MODE_NORMAL:
                data1  = BMA2X2_SET_BITSLICE(data1,
                        BMA2X2_MODE_CTRL, 0);
                data2  = BMA2X2_SET_BITSLICE(data2,
                        BMA2X2_LOW_POWER_MODE, 0);
                bma2x2_smbus_write_byte(client,
                        BMA2X2_MODE_CTRL_REG, &data1);
                mdelay(1);
                bma2x2_smbus_write_byte(client,
                    BMA2X2_LOW_NOISE_CTRL_REG, &data2);
                break;
        case BMA2X2_MODE_LOWPOWER1:
                data1  = BMA2X2_SET_BITSLICE(data1,
                        BMA2X2_MODE_CTRL, 2);
                data2  = BMA2X2_SET_BITSLICE(data2,
                        BMA2X2_LOW_POWER_MODE, 0);
                bma2x2_smbus_write_byte(client,
                        BMA2X2_MODE_CTRL_REG, &data1);
                mdelay(1);
                bma2x2_smbus_write_byte(client,
                    BMA2X2_LOW_NOISE_CTRL_REG, &data2);
                break;
        case BMA2X2_MODE_SUSPEND:
                data1  = BMA2X2_SET_BITSLICE(data1,
                        BMA2X2_MODE_CTRL, 4);
                data2  = BMA2X2_SET_BITSLICE(data2,
                        BMA2X2_LOW_POWER_MODE, 0);
                bma2x2_smbus_write_byte(client,
                    BMA2X2_LOW_NOISE_CTRL_REG, &data2);
                mdelay(1);
                bma2x2_smbus_write_byte(client,
                    BMA2X2_MODE_CTRL_REG, &data1);
                break;
        case BMA2X2_MODE_DEEP_SUSPEND:
                data1  = BMA2X2_SET_BITSLICE(data1,
                            BMA2X2_MODE_CTRL, 1);
                data2  = BMA2X2_SET_BITSLICE(data2,
                        BMA2X2_LOW_POWER_MODE, 1);
                bma2x2_smbus_write_byte(client,
                        BMA2X2_MODE_CTRL_REG, &data1);
                mdelay(1);
                bma2x2_smbus_write_byte(client,
                    BMA2X2_LOW_NOISE_CTRL_REG, &data2);
                break;
        case BMA2X2_MODE_LOWPOWER2:
                data1  = BMA2X2_SET_BITSLICE(data1,
                        BMA2X2_MODE_CTRL, 2);
                data2  = BMA2X2_SET_BITSLICE(data2,
                        BMA2X2_LOW_POWER_MODE, 1);
                bma2x2_smbus_write_byte(client,
                        BMA2X2_MODE_CTRL_REG, &data1);
                mdelay(1);
                bma2x2_smbus_write_byte(client,
                    BMA2X2_LOW_NOISE_CTRL_REG, &data2);
                break;
        case BMA2X2_MODE_STANDBY:
                data1  = BMA2X2_SET_BITSLICE(data1,
                        BMA2X2_MODE_CTRL, 4);
                data2  = BMA2X2_SET_BITSLICE(data2,
                        BMA2X2_LOW_POWER_MODE, 1);
                bma2x2_smbus_write_byte(client,
                    BMA2X2_LOW_NOISE_CTRL_REG, &data2);
                mdelay(1);
                bma2x2_smbus_write_byte(client,
                        BMA2X2_MODE_CTRL_REG, &data1);
        break;
        }
    } else {
        comres = -1 ;
    }
    return comres;
}


static int bma2x2_get_mode(struct i2c_client *client, unsigned char *Mode)
{
    int comres = 0;
    unsigned char data1, data2;

    comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG, &data1);
    comres = bma2x2_smbus_read_byte(client, BMA2X2_LOW_NOISE_CTRL_REG,
            &data2);
    data1  = (data1 & 0xE0) >> 5;
    data2  = (data2 & 0x40) >> 6;
    if ((data1 == 0x00) && (data2 == 0x00)) {
        *Mode  = BMA2X2_MODE_NORMAL;
    } else {
        if ((data1 == 0x02) && (data2 == 0x00)) {
            *Mode  = BMA2X2_MODE_LOWPOWER1;
        } else {
            if ((data1 == 0x04 || data1 == 0x06) &&
                        (data2 == 0x00)) {
                *Mode  = BMA2X2_MODE_SUSPEND;
            } else {
                if (((data1 & 0x01) == 0x01)) {
                    *Mode  = BMA2X2_MODE_DEEP_SUSPEND;
                } else {
                    if ((data1 == 0x02) &&
                            (data2 == 0x01)) {
                        *Mode  = BMA2X2_MODE_LOWPOWER2;
                    } else {
                        if ((data1 == 0x04) && (data2 ==
                                    0x01)) {
                            *Mode  =
                            BMA2X2_MODE_STANDBY;
                        } else {
                            *Mode =
                        BMA2X2_MODE_DEEP_SUSPEND;
                        }
                    }
                }
            }
        }
    }
    return comres;
}

static int bma2x2_set_range(struct i2c_client *client, unsigned char Range)
{
    int comres = 0 ;
    unsigned char data1;
    if ((Range == 3) || (Range == 5) || (Range == 8) || (Range == 12)) {
        comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL_REG,
                &data1);
        switch (Range) {
        case BMA2X2_RANGE_2G:
            data1  = BMA2X2_SET_BITSLICE(data1,
                    BMA2X2_RANGE_SEL, 3);
            break;
        case BMA2X2_RANGE_4G:
            data1  = BMA2X2_SET_BITSLICE(data1,
                    BMA2X2_RANGE_SEL, 5);
            break;
        case BMA2X2_RANGE_8G:
            data1  = BMA2X2_SET_BITSLICE(data1,
                    BMA2X2_RANGE_SEL, 8);
            break;
        case BMA2X2_RANGE_16G:
            data1  = BMA2X2_SET_BITSLICE(data1,
                    BMA2X2_RANGE_SEL, 12);
            break;
        default:
            break;
        }
        comres += bma2x2_smbus_write_byte(client, BMA2X2_RANGE_SEL_REG,
                &data1);
    } else {
        comres = -1 ;
    }
    return comres;
}

static int bma2x2_get_range(struct i2c_client *client, unsigned char *Range)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL__REG, &data);
    data = BMA2X2_GET_BITSLICE(data, BMA2X2_RANGE_SEL);
    *Range = data;
    return comres;
}

static int bma2x2_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
    int comres = 0;
    unsigned char data;
    int Bandwidth = 0;

    if (BW > 7 && BW < 16) {
        switch (BW) {
        case BMA2X2_BW_7_81HZ:
            Bandwidth = BMA2X2_BW_7_81HZ;

            /*  7.81 Hz      64000 uS   */
            break;
        case BMA2X2_BW_15_63HZ:
            Bandwidth = BMA2X2_BW_15_63HZ;

            /*  15.63 Hz     32000 uS   */
            break;
        case BMA2X2_BW_31_25HZ:
            Bandwidth = BMA2X2_BW_31_25HZ;

            /*  31.25 Hz     16000 uS   */
            break;
        case BMA2X2_BW_62_50HZ:
            Bandwidth = BMA2X2_BW_62_50HZ;

            /*  62.50 Hz     8000 uS   */
            break;
        case BMA2X2_BW_125HZ:
            Bandwidth = BMA2X2_BW_125HZ;

            /*  125 Hz       4000 uS   */
            break;
        case BMA2X2_BW_250HZ:
            Bandwidth = BMA2X2_BW_250HZ;

            /*  250 Hz       2000 uS   */
            break;
        case BMA2X2_BW_500HZ:
            Bandwidth = BMA2X2_BW_500HZ;

            /*  500 Hz       1000 uS   */
            break;
        case BMA2X2_BW_1000HZ:
            Bandwidth = BMA2X2_BW_1000HZ;

            /*  1000 Hz      500 uS   */
            break;
        default:
            break;
        }
        comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG,
                &data);
        if(comres < 0)
            return comres;
        data = BMA2X2_SET_BITSLICE(data, BMA2X2_BANDWIDTH, Bandwidth);
        comres += bma2x2_smbus_write_byte(client, BMA2X2_BANDWIDTH__REG,
                &data);
    } else {
        comres = -1 ;
    }
    return comres;
}

static int bma2x2_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
    int comres = 0;
    unsigned char data;
    comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG, &data);
    data = BMA2X2_GET_BITSLICE(data, BMA2X2_BANDWIDTH);
    *BW = data ;
    return comres;
}
static int bma2x2_write_reg(struct i2c_client *client, unsigned char addr,
        unsigned char *data)
{
    int comres = 0 ;
    comres = bma2x2_smbus_write_byte(client, addr, data);
    return comres;
}

static ssize_t bma2x2_enable_int_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    return sprintf(buf, "%d\n", atomic_read(&bma2x2->enable_ddtap));
}

static ssize_t bma2x2_enable_int_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    bma2x2_set_int_enable(dev,data);
    return count;
}


static ssize_t bma2x2_int_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    if (bma2x2_get_Int_Mode(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");

    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_int_mode_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_Int_Mode(bma2x2->bma2x2_client, (unsigned char)data) < 0)
        return -EINVAL;
    return count;
}
static ssize_t bma2x2_tap_threshold_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    if (bma2x2_get_tap_threshold(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");
    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_tap_threshold_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_tap_threshold(bma2x2->bma2x2_client, (unsigned char)data)
            < 0)
        return -EINVAL;
    return count;
}
static ssize_t bma2x2_tap_duration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    if (bma2x2_get_tap_duration(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");
    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_tap_duration_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_tap_duration(bma2x2->bma2x2_client, (unsigned char)data)
            < 0)
        return -EINVAL;
    return count;
}
static ssize_t bma2x2_tap_quiet_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    if (bma2x2_get_tap_quiet(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");
    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_tap_quiet_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_tap_quiet(bma2x2->bma2x2_client, (unsigned char)data) < 0)
        return -EINVAL;
    return count;
}

static ssize_t bma2x2_tap_shock_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    if (bma2x2_get_tap_shock(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");
    return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_tap_shock_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_tap_shock(bma2x2->bma2x2_client, (unsigned char)data) < 0)
        return -EINVAL;
    return count;
}

static ssize_t bma2x2_tap_samp_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    if (bma2x2_get_tap_samp(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");
    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_tap_samp_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_tap_samp(bma2x2->bma2x2_client, (unsigned char)data) < 0)
        return -EINVAL;
    return count;
}
static int bma2x2_read_accel_xyz(struct i2c_client *client,
        signed char sensor_type, struct bma2x2acc *acc)
{
    int comres = 0;
    unsigned char data[6];
    s16 temp[3];
    comres = bma2x2_smbus_read_byte_block(client,
                BMA2X2_ACC_X12_LSB__REG, data, 6);
    temp[0] = ((s16) ((data[1] << 8) | data[0])) >> 4;
    temp[1] = ((s16) ((data[3] << 8) | data[2])) >> 4;
    temp[2] = ((s16) ((data[5] << 8) | data[4])) >> 4;
    acc->x = ((bma2x2_accel->negate_x) ? (-temp[bma2x2_accel->axis_map_x]) : (temp[bma2x2_accel->axis_map_x]));
    acc->y = ((bma2x2_accel->negate_y) ? (-temp[bma2x2_accel->axis_map_y]) : (temp[bma2x2_accel->axis_map_y]));
    acc->z = ((bma2x2_accel->negate_z) ? (-temp[bma2x2_accel->axis_map_z]) : (temp[bma2x2_accel->axis_map_z]));
    return comres;
}

static void bma2x2_work_func(struct work_struct *work)
{
    struct bma2x2_data *bma2x2 = container_of((struct delayed_work *)work,
            struct bma2x2_data, work);
    struct input_dev *input_dev = bma2x2->input;
    static struct bma2x2acc acc;
    unsigned long delay = msecs_to_jiffies(atomic_read(&bma2x2->delay));
    bma2x2_i2c_resume(bma2x2->bma2x2_client);
    mutex_lock(&input_dev->mutex);
    bma2x2_read_accel_xyz(bma2x2->bma2x2_client, bma2x2->sensor_type,&acc);
    mutex_unlock(&input_dev->mutex);
    input_report_abs(bma2x2->input, ABS_X, acc.x);
    input_report_abs(bma2x2->input, ABS_Y, acc.y);
    input_report_abs(bma2x2->input, ABS_Z, acc.z);
    input_sync(bma2x2->input);
    schedule_delayed_work(&bma2x2->work, delay);
}
static int bma2x2_read_accel_rawdata(struct i2c_client *client,
        signed char sensor_type, struct bma2x2acc *acc)
{
    int comres = 0;
    unsigned char data[6];
    KMSGINF(&client->dev, "Accel,bma2x2_read_accel_rawdata start\n");
    comres = bma2x2_smbus_read_byte_block(client,
                BMA2X2_ACC_X12_LSB__REG, data, 6);
    acc->x = ((s16) ((data[1] << 8) | data[0])) >> 4;
    acc->y = ((s16) ((data[3] << 8) | data[2])) >> 4;
    acc->z = ((s16) ((data[5] << 8) | data[4])) >> 4;
    KMSGINF(&client->dev, "Accel,bma2x2_read_accel_rawdata rawdata[0]: %d\n",acc->x);
    KMSGINF(&client->dev, "Accel,bma2x2_read_accel_rawdata rawdata[1]: %d\n",acc->y);
    KMSGINF(&client->dev, "Accel,bma2x2_read_accel_rawdata rawdata[2]: %d\n",acc->z);
    return comres;
}
static void bma2x2_test_work_func(struct work_struct *test_work)
{
    struct bma2x2_data *bma2x2 = container_of((struct delayed_work *)test_work,
            struct bma2x2_data, test_work);
    int err;
    struct i2c_client *client = bma2x2->bma2x2_client;
    struct input_dev *input_dev = bma2x2->input;
    static struct bma2x2acc acc;
    KMSGINF(&client->dev, "Accel,bma2x2_test_work_func start\n");
    bma2x2_i2c_resume(bma2x2->bma2x2_client);
    mutex_lock(&input_dev->mutex);
    err = bma2x2_read_accel_rawdata(bma2x2->bma2x2_client, bma2x2->sensor_type, &acc);
    mutex_unlock(&input_dev->mutex);
    if (err < 0) {
        KMSGERR(&client->dev, "Accel,%s: bma2x2_test_work_func read data output error = %d\n", __func__, err);
    } else {
        KMSGINF(&client->dev, "TEST: Accel,bma2x2 RAW DATA, x, %d, y, %d, z, %d\n",
                ((s16)(le16_to_cpu(acc.x) & MASK_16_10_BITS)) >> 6,
                ((s16)(le16_to_cpu(acc.y) & MASK_16_10_BITS)) >> 6,
                ((s16)(le16_to_cpu(acc.z) & MASK_16_10_BITS)) >> 6);
    }
    queue_delayed_work(bma2x2->test_queue, &bma2x2->test_work, msecs_to_jiffies(500));
}

static ssize_t bma2x2_register_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int address, value;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    sscanf(buf, "%d%d", &address, &value);
    if (bma2x2_write_reg(bma2x2->bma2x2_client, (unsigned char)address,
                (unsigned char *)&value) < 0)
        return -EINVAL;
    return count;
}
static ssize_t bma2x2_register_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{

    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    size_t count = 0;
    u8 reg[0x40];
    int i;

    for (i = 0; i < 0x40; i++) {
        bma2x2_smbus_read_byte(bma2x2->bma2x2_client, i, reg+i);

        count += sprintf(&buf[count], "0x%x: %d\n", i, reg[i]);
    }
    return count;
}

static ssize_t bma2x2_range_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    if (bma2x2_get_range(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");

    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_range_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_range(bma2x2->bma2x2_client, (unsigned char) data) < 0)
        return -EINVAL;
    return count;
}

static ssize_t bma2x2_bandwidth_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    if (bma2x2_get_bandwidth(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");
    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_bandwidth_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2->sensor_type == BMA280_TYPE)
        if ((unsigned char) data > 14)
            return -EINVAL;
    if (bma2x2_set_bandwidth(bma2x2->bma2x2_client,
                (unsigned char) data) < 0)
        return -EINVAL;

    return count;
}

static ssize_t bma2x2_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char data;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    if (bma2x2_get_mode(bma2x2->bma2x2_client, &data) < 0)
        return sprintf(buf, "Read error\n");
    return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_mode_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (bma2x2_set_mode(bma2x2->bma2x2_client, (unsigned char) data) < 0)
        return -EINVAL;
    return count;
}
static ssize_t bma2x2_value_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct bma2x2_data *bma2x2 = input_get_drvdata(input);
    struct bma2x2acc acc_value;

    bma2x2_read_accel_xyz(bma2x2->bma2x2_client, bma2x2->sensor_type,
                                &acc_value);
    return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
            acc_value.z);
}

static ssize_t bma2x2_delay_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    return sprintf(buf, "%d\n", atomic_read(&bma2x2->delay));
}

static ssize_t bma2x2_chip_id_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    return sprintf(buf, "%d\n", bma2x2->chip_id);
}
static ssize_t bma2x2_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if (data > BMA2X2_MAX_DELAY)
        data = BMA2X2_MAX_DELAY;
    atomic_set(&bma2x2->delay, (unsigned int) data);
    return count;
}


static ssize_t bma2x2_enable_poll_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    return sprintf(buf, "%d\n", atomic_read(&bma2x2->enable));
}

static void bma2x2_set_enable(struct device *dev, int enable)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    int pre_enable = atomic_read(&bma2x2->enable);
    int pre_int_enable = atomic_read(&bma2x2->enable_ddtap);
    KMSGINF(&client->dev, "Accel,bma2x2_set_enable start\n");
    KMSGINF(&client->dev, "Accel,bma2x2_set_enable, pre_enable:%d,pre_int_enable:%d\n",pre_enable,pre_int_enable);
    bma2x2_i2c_resume(bma2x2->bma2x2_client);
    mutex_lock(&bma2x2->enable_mutex);
    if (enable) {
        if (pre_enable == 0 && pre_int_enable == 0) {
            bma2x2_set_mode(bma2x2->bma2x2_client,
                    BMA2X2_MODE_NORMAL);
        }
        if (pre_enable == 0) {
            schedule_delayed_work(&bma2x2->work,
                msecs_to_jiffies(atomic_read(&bma2x2->delay)));
            atomic_set(&bma2x2->enable, 1);
        }

    } else {
        if (pre_enable == 1 && pre_int_enable == 0) {
            bma2x2_set_mode(bma2x2->bma2x2_client,
                    BMA2X2_MODE_SUSPEND);
        }
        if (pre_enable == 1) {
            cancel_delayed_work_sync(&bma2x2->work);
            atomic_set(&bma2x2->enable, 0);
        }
    }
    mutex_unlock(&bma2x2->enable_mutex);

}

static ssize_t bma2x2_enable_poll_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    int error;

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;
    if ((data == 0) || (data == 1))
        bma2x2_set_enable(dev, data);
    return count;
}
static void bma2x2_validate_output_data(struct i2c_client *client,struct ACCEL_SELFTEST_RESULT* result,u8 axis)
{
    s32 selftest_output_delta = 0;
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, axis:%d\n",axis);
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, selftest_data_positive1:%d\n",selftest_data_positive);
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, selftest_data_negative1:%d\n",selftest_data_negative);
    selftest_data_positive /= BMA2X2_SELFTEST_SAMPLE_COUNT;
    selftest_data_negative /= BMA2X2_SELFTEST_SAMPLE_COUNT;
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, selftest_data_positive2:%d\n",selftest_data_positive);
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, selftest_data_negative2:%d\n",selftest_data_negative);
    selftest_output_delta = DIFFERENCE(selftest_data_positive, selftest_data_negative);
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, selftest_output_delta:%d\n",selftest_output_delta);
    switch (axis) {
        case X_AXIS:
            result->x_axis_pass = (selftest_output_delta < BMA2X2_ST_X_LOW_LIMIT) ? FALSE : TRUE;
            result->x_axis_result = selftest_output_delta;
            break;
        case Y_AXIS:
            result->y_axis_pass = (selftest_output_delta < BMA2X2_ST_Y_LOW_LIMIT) ? FALSE : TRUE;
            result->y_axis_result = selftest_output_delta;
            break;
        case Z_AXIS:
            result->z_axis_pass = (selftest_output_delta < BMA2X2_ST_Z_LOW_LIMIT) ? FALSE : TRUE;
            result->z_axis_result = selftest_output_delta;
            break;
        default:
            break;
      }
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, result->x_axis:%d\n",result->x_axis_pass);
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, result->y_axis:%d\n",result->y_axis_pass);
    KMSGINF(&client->dev,"Accel,bma2x2_validate_output_data, result->z_axis:%d\n",result->z_axis_pass);
}
static int bma2x2_selftest_read_output(struct i2c_client *client,u8 axis, u8 sign)
{
    u8 i = 0;
    unsigned char data[2];
    s16 tmp_value;
    int comres = 0;
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_read_output, axis:%d,sign:%d\n",axis,sign);
    for(i = 0; i < BMA2X2_SELFTEST_SAMPLE_COUNT; i++) {
    switch (axis) {
        case X_AXIS:
            comres = bma2x2_smbus_read_byte_block(client,BMA2X2_ACC_X12_LSB__REG,data,2);
            break;
        case Y_AXIS:
            comres = bma2x2_smbus_read_byte_block(client,BMA2X2_ACC_Y12_LSB__REG,data,2);
            break;
        case Z_AXIS:
            comres = bma2x2_smbus_read_byte_block(client,BMA2X2_ACC_Z12_LSB__REG,data,2);
            break;
        default:
            KMSGINF(&client->dev,"Accel,bma2x2_selftest_read_output ERROR\n");
            break;
    }
    if(comres < 0)
       return comres;
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_read_output, data[0]:%d,data[1]:%d\n",data[0],data[1]);
    tmp_value = data[1];
    tmp_value = (tmp_value << 2) | (data[0] >> 6);
    tmp_value = tmp_value << 6;
    tmp_value = tmp_value >> 6;
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_read_output, tmp_value:%d\n",tmp_value);

    /* Save the selftest data for later use */
    if(sign == POSITIVE) {
        selftest_data_positive += tmp_value;
      }
    else {
        selftest_data_negative += tmp_value;
      }
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_read_output, selftest_data_positive:%d,selftest_data_negative:%d\n",selftest_data_positive,selftest_data_negative);
    if(i < BMA2X2_SELFTEST_SAMPLE_COUNT - 1) {
        /* Make a delay to make sure that next sample is ready */
        mdelay(20);
      }
    }
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_read_output, axis:%d,sign:%d,selftest_data_positive++:%d\n",axis,sign,selftest_data_positive);
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_read_output, axis:%d,sign:%d,selftest_data_negative++:%d\n",axis,sign,selftest_data_negative);
    return comres;
}
static ssize_t bma2x2_selftest_run_and_get_data(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    unsigned char data;
    unsigned char temp[3];
    int comres = -1;
    struct ACCEL_SELFTEST_RESULT* selftest_result = (struct ACCEL_SELFTEST_RESULT*)buf;
    memset(selftest_result, 0, sizeof(*selftest_result));
/*INT test*/
    KMSGINF(dev,"Accel,bma2x2_selftest_run_and_get_data, INT test start\n");
    bma2x2_i2c_resume(bma2x2->bma2x2_client);
    bma2x2_set_int_enable(dev,0);
    bma2x2_set_enable(dev,0);
    selftest_result->interrupt_pin_support = true;
    selftest_result->interrupt_pin_status = false;
    if(selftest_result->interrupt_pin_support == true)
    {
        atomic_set(&bma2x2->selftest_int, 0);
        atomic_set(&bma2x2->selftest_ongoing, 1);
        comres = bma2x2_smbus_read_byte_block(client,BMA2X2_STATUS1_REG,temp,3);/*clear the status reg*/
        if(comres < 0)
            goto exit_error;
        KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, clear the status\n");
        bma2x2_set_bandwidth(client, BMA2X2_BW_1000HZ);
        bma2x2_set_mode(client,BMA2X2_MODE_NORMAL);
        bma2x2_set_Int_Mode(client, BMA2X2_LATCH_DUR_LATCH1);/*latch interrupt 250ms*/
        KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, set bindwidth\n");
        bma2x2_set_int1_pad_sel(client, PAD_SEL_INT1_DATAREADY);/*mapping int line*/
        KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, mapping int line\n");
        data = 0;
        comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE1_REG,&data); /*disable all int*/
        if(comres < 0)
          goto exit_error;
        comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE2_REG,&data); /*disable all int*/
        if(comres < 0)
          goto exit_error;
        data = 0x10;
        comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE2_REG,&data);
        if(comres < 0)
          goto exit_error;
        KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, enable int data2:%d\n",data);
        enable_irq(bma2x2->IRQ);
        msleep(NEWDATA_INT_TIME_MS);
        wait_event_interruptible_timeout(bma2x2->selftest_wq, atomic_read(&bma2x2->selftest_int),
                                             msecs_to_jiffies(100));
        if (atomic_read(&bma2x2->selftest_int) == 1) {
            comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS2_REG, &data); /*Read interrupt type and check if it is the data ready int*/
        if(comres < 0)
            goto exit_error;
        KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, 0a:%d\n",data);
        selftest_result->interrupt_pin_status = TRUE;
        KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, get DRDY interrupt\n");
        }
        KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest_result->interrupt_pin_status:%d\n", selftest_result->interrupt_pin_status);
        atomic_set(&bma2x2->selftest_ongoing, 0);
        data = 0;
        comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE2_REG,&data);
        if(comres < 0)
          goto exit_error;
    }
/*selftest*/
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest start\n");
    data = 0x00;
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest disable selftest\n");
    bma2x2_set_bandwidth(client, BMA2X2_BW_125HZ);
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest set bandwidth 125Hz\n");
    bma2x2_set_range(client, BMA2X2_RANGE_8G);
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest set range 2G\n");
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
  /*x axis test*/
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest x axis positive start\n");
    selftest_data_positive = 0;
    selftest_data_negative = 0;
    data = 0x15; /* x, positive */
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
    comres = bma2x2_selftest_read_output(client,X_AXIS,POSITIVE);
    if(comres < 0)
        goto exit_error;
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest x axis negative start\n");
    data = 0x11;/*x, negative*/
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
    comres = bma2x2_selftest_read_output(client,X_AXIS,NEGATIVE);
    if(comres < 0)
        goto exit_error;
    bma2x2_validate_output_data(client,selftest_result,X_AXIS);
    /* y axis test*/
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest y axis positive start\n");
    selftest_data_positive = 0;
    selftest_data_negative = 0;
    data = 0x16; /* y,positive*/
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
    comres = bma2x2_selftest_read_output(client,Y_AXIS,POSITIVE);
    if(comres < 0)
        goto exit_error;
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest y axis negative start\n");
    data = 0x12; /*y,negative*/
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
    comres = bma2x2_selftest_read_output(client,Y_AXIS,NEGATIVE);
    if(comres < 0)
        goto exit_error;
    bma2x2_validate_output_data(client,selftest_result,Y_AXIS);
    /*Z axis test*/
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest z axis positive start\n");
    selftest_data_positive = 0;
    selftest_data_negative = 0;
    data = 0x17; /* z,positive*/
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
    comres = bma2x2_selftest_read_output(client,Z_AXIS,POSITIVE);
    if(comres < 0)
        goto exit_error;
    KMSGINF(&client->dev,"Accel,bma2x2_selftest_run_and_get_data, selftest z axis negative start\n");
    data = 0x13; /*z,negative*/
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
    comres = bma2x2_selftest_read_output(client,Z_AXIS,NEGATIVE);
    if(comres < 0)
        goto exit_error;
    bma2x2_validate_output_data(client,selftest_result,Z_AXIS);
    selftest_result->i2c_status = TRUE;
      /*disable selftest and SW reset*/
    data = 0x00;
    comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,&data);
    if(comres < 0)
        goto exit_error;
    mdelay(BMA2X2_SAMPLE_READY_TIME_MS);
  return sizeof(struct ACCEL_SELFTEST_RESULT);
exit_error:
  selftest_result->interrupt_pin_support = TRUE;
  selftest_result->interrupt_pin_status = FALSE;
  selftest_result->x_axis_pass = FALSE;
  selftest_result->y_axis_pass = FALSE;
  selftest_result->z_axis_pass = FALSE;
  selftest_result->i2c_status = FALSE;
  KMSGINF(dev, "Accel,bma2x2_selftest_run_and_get_data -> Error Exit\n");
  return sizeof(struct ACCEL_SELFTEST_RESULT);
}
static ssize_t bma2x2_get_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", bma2x2->test);
}
static ssize_t bma2x2_set_test(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
    struct input_dev *input_dev = bma2x2->input;
    unsigned long test;
    int err = 0;
    KMSGINF(&client->dev, "Accel,bma2x2_set_test start\n");

    mutex_lock(&input_dev->mutex);
    err = strict_strtoul(buf, 10, &test);
    if (err < 0) {
        KMSGERR(&client->dev, "Accel,%s: Bosch, bma2x2_set_test strict_strtoul returned err = %d\n", __func__, err);
        goto exit;
    }
    if((test != 1) && (test != 0)) {
        err = -EINVAL;
        KMSGERR(&client->dev, "Accel,%s: Bosch, bma2x2_set_test invalid test = %d\n", __func__, (unsigned int) test);
        goto exit;
    }

    if(test == 1) {
        if(bma2x2->test == FALSE) {
            bma2x2->test_queue = create_workqueue("bma2x2_set_test Bma Accel Testqueue");
            if (!bma2x2->test_queue) {
                KMSGERR(&client->dev, "Accel,%s: Bma_Accel, failed to create test queue\n", __func__);
                err = -EPERM;
                goto exit;
            }
            INIT_DELAYED_WORK(&bma2x2->test_work, bma2x2_test_work_func);
            queue_delayed_work(bma2x2->test_queue, &bma2x2->test_work, 0);
        }
        bma2x2->test = TRUE;
    } else {
        if(bma2x2->test == TRUE) {
            cancel_delayed_work_sync(&bma2x2->test_work);
            destroy_workqueue(bma2x2->test_queue);
        }
        bma2x2->test = FALSE;
    }
exit:
    mutex_unlock(&input_dev->mutex);
    return (err < 0) ? err : size;
}

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_range_show, bma2x2_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_bandwidth_show, bma2x2_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_mode_show, bma2x2_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
        bma2x2_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_delay_show, bma2x2_delay_store);
static DEVICE_ATTR(enable_poll, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_enable_poll_show, bma2x2_enable_poll_store);
static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_register_show, bma2x2_register_store);
static DEVICE_ATTR(chip_id, S_IRUGO,
        bma2x2_chip_id_show, NULL);
static DEVICE_ATTR(enable_int, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_enable_int_show, bma2x2_enable_int_store);
static DEVICE_ATTR(int_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_int_mode_show, bma2x2_int_mode_store);
static DEVICE_ATTR(tap_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_tap_duration_show, bma2x2_tap_duration_store);
static DEVICE_ATTR(tap_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_tap_threshold_show, bma2x2_tap_threshold_store);
static DEVICE_ATTR(tap_quiet, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_tap_quiet_show, bma2x2_tap_quiet_store);
static DEVICE_ATTR(tap_shock, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_tap_shock_show, bma2x2_tap_shock_store);
static DEVICE_ATTR(tap_samp, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_tap_samp_show, bma2x2_tap_samp_store);
static DEVICE_ATTR(selftest_data, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_selftest_run_and_get_data, NULL);
static DEVICE_ATTR(set_test, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
        bma2x2_get_test, bma2x2_set_test);


static struct attribute *bma2x2_attributes[] = {
    &dev_attr_range.attr,
    &dev_attr_bandwidth.attr,
    &dev_attr_mode.attr,
    &dev_attr_value.attr,
    &dev_attr_delay.attr,
    &dev_attr_enable_poll.attr,
    &dev_attr_reg.attr,
    &dev_attr_chip_id.attr,
    &dev_attr_enable_int.attr,
    &dev_attr_int_mode.attr,
    &dev_attr_tap_threshold.attr,
    &dev_attr_tap_duration.attr,
    &dev_attr_tap_quiet.attr,
    &dev_attr_tap_shock.attr,
    &dev_attr_tap_samp.attr,
    &dev_attr_selftest_data.attr,
    &dev_attr_set_test.attr,
    NULL
};

static struct attribute_group bma2x2_attribute_group = {
    .attrs = bma2x2_attributes
};
#if defined(BMA2X2_ENABLE_INT1)
static void bma2x2_irq_work_func(struct work_struct *work)
{
    struct bma2x2_data *bma2x2 = container_of((struct work_struct *)work,
            struct bma2x2_data, irq_work);

    unsigned char status = 0;
    unsigned char status2 = 0;
    bma2x2_i2c_resume(bma2x2->bma2x2_client);
    bma2x2_get_interruptstatus1(bma2x2->bma2x2_client, &status);
    KMSGINF(&bma2x2->bma2x2_client->dev, "Accel,bma2x2_irq_work_func interrupt status:%d\n",status);
    switch (status) {
    case DOUBLE_TAP_INT_TRI:
        bma2x2_get_interruptstatus2(bma2x2->bma2x2_client, &status2);
        KMSGINF(&bma2x2->bma2x2_client->dev, "Accel,bma2x2_irq_work_func interrupt status2:%d\n",status2);
        status2 = status2 & DOUBLE_TAP_Z_NAG_MASK;
        if(bma2x2->axis_map_z == 2){
             /* report the z+ and z-double tap */
             if(status2 == DOUBLE_TAP_Z_POS_MASK || status2 == DOUBLE_TAP_Z_NAG_MASK){
                input_report_rel(bma2x2->ddtap, DOUBLE_TAP_INTERRUPT,1);
                input_sync(bma2x2->ddtap);
             }
        }
        break;
    case SINGLE_TAP_INT_TRI:
        KMSGINF(&bma2x2->bma2x2_client->dev, "Accel,bma2x2_irq_work_func single tap interrupt happened\n");
        input_report_rel(bma2x2->input, SINGLE_TAP_INTERRUPT,
                    SINGLE_TAP_INTERRUPT_HAPPENED);
        break;
    default:
        break;
    }
  enable_irq(bma2x2->IRQ);
}

static irqreturn_t bma2x2_irq_handler(int irq, void *handle)
{
    struct bma2x2_data *data = handle;
    disable_irq_nosync(irq);
    KMSGINF(&data->bma2x2_client->dev,"Accel,bma2x2_irq_handler,start\n");
    if (data == NULL)
        return IRQ_HANDLED;
    if (data->bma2x2_client == NULL)
        return IRQ_HANDLED;
    if (unlikely(atomic_read(&data->selftest_ongoing) == 1))
    {
        KMSGINF(&data->bma2x2_client->dev,"Accel,bma2x2_irq_handler,selftest_ongoing\n");
        atomic_set(&data->selftest_int, 1);
        wake_up_interruptible(&data->selftest_wq);
        return IRQ_HANDLED;
    }
    schedule_work(&data->irq_work);
    return IRQ_HANDLED;
}
#endif /* defined(BMA2X2_ENABLE_INT1) */

static int bma2x2_acc_update_direction(struct bma2x2_data *data, u8 direction)
{
    data->axis_map_x = ((direction-1)%2);
    data->axis_map_y = (direction%2);
    data->axis_map_z = 2;
    data->negate_z   = ((direction-1)/4);
    data->negate_x   = ((direction/2)%2);
    data->negate_y   = (((direction+1)/4)%2);
    return 0;
}

static int bma2x2_acc_parse_dt(struct device *dev, struct bma2x2_acc_platform_data *acceld)
{
    int ret = 0;
    u32 val;

    ret = of_property_read_u32(dev->of_node, "bma250_acc,min_interval", &val);
    if (!ret)
        acceld->min_interval = val;

    ret = of_property_read_u32(dev->of_node, "bma250_acc,poll_interval", &val);
    if (!ret)
        acceld->poll_interval = val;

    ret = of_property_read_u32(dev->of_node, "bma250_acc,accel_direction", &val);
    if (!ret)
        acceld->accel_direction = (u8)val;

    ret = of_property_read_u32(dev->of_node, "bma250_acc,accel_g_range", &val);
    if (!ret)
        acceld->accel_range = (u8)val;

    return ret;
}


static int bma2x2_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int err = 0;
    int tempvalue;
    unsigned char tmp_chip_id;
    struct bma2x2_data *data;
    struct input_dev *dev,*input_dev_ddtap;
    struct bma2x2_acc_platform_data *accel_pdata;

    KMSGINF(&client->dev, "Accel,bma2x2_probe start\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        KMSGERR(&client->dev, "Accel,bma2x2_probe i2c_check_functionality error\n");
        err = -ENOMEM;
        goto exit;
    }
    data = kzalloc(sizeof(struct bma2x2_data), GFP_KERNEL);
    if (!data) {
        err = -ENOMEM;
        goto exit;
    }
    bma2x2_accel = data;
    accel_pdata = kzalloc(sizeof(struct bma2x2_acc_platform_data), GFP_KERNEL);
    if (!accel_pdata) {
        err = -ENOMEM;
        goto kfree_exit;
    }
    data->pdata = accel_pdata;
    /* read chip id */
    tempvalue = i2c_smbus_read_word_data(client, BMA2X2_CHIP_ID_REG);
    tmp_chip_id = tempvalue&0x00ff;

    switch (tmp_chip_id) {
    case BMA255_CHIP_ID:
        data->sensor_type = BMA255_TYPE;
        break;
    case BMA250E_CHIP_ID:
        data->sensor_type = BMA250E_TYPE;
        break;
    case BMA222E_CHIP_ID:
        data->sensor_type = BMA222E_TYPE;
        break;
    case BMA280_CHIP_ID:
        data->sensor_type = BMA280_TYPE;
        break;
    default:
        data->sensor_type = -1;
    }
    if (data->sensor_type != -1) {
        data->chip_id = tmp_chip_id;
        KMSGERR(&client->dev, "Accel,%s: Bosch Sensortec Device detected! sensor_type = %d. Abort.\n", __func__, data->sensor_type);
    } else{
        KMSGERR(&client->dev, "Accel,%s: Bosch Sensortec Device not detected! tempvalue = %d.\n", __func__, tempvalue);
        err = -ENODEV;
        goto kfree_exit1;
    }
    data->bma2x2_client = client;
    i2c_set_clientdata(client, data);
    mutex_init(&data->value_mutex);
    mutex_init(&data->mode_mutex);
    mutex_init(&data->enable_mutex);
    mutex_init(&data->enable_ddtap_mutex);
    atomic_set(&data->selftest_int, 0);
    atomic_set(&data->selftest_ongoing, 0);
    atomic_set(&data->suspend, ACCEL_I2C_RESUME);
    init_waitqueue_head(&data->selftest_wq);
    err = bma2x2_acc_parse_dt(&client->dev, accel_pdata);
    if (err) {
        KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 failed to parse device tree\n");
        goto kfree_exit1;
    }

    err = bma2x2_acc_update_direction(data, accel_pdata->accel_direction);
    if (err < 0) {
        KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 failed to update_accel_direction\n");
        goto  kfree_exit1;
    }
    bma2x2_set_bandwidth(client, BMA2X2_BW_SET);
    bma2x2_set_range(client, accel_pdata->accel_range);
    KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 set bandwidth and range done\n");

#if defined(BMA2X2_ENABLE_INT1)
    bma2x2_set_Int_Mode(client, BMA2X2_LATCH_DUR_250MS);/*latch interrupt 250ms*/
#endif

#ifdef BMA2X2_ENABLE_INT1
    /* maps interrupt to INT1 pin */
    bma2x2_set_int1_pad_sel(client, PAD_SEL_INT1_DBTAP);
#endif
    bma2x2_set_mode(client,BMA2X2_MODE_SUSPEND);
#if defined(BMA2X2_ENABLE_INT1)
    INIT_WORK(&data->irq_work, bma2x2_irq_work_func);
    data->IRQ = client->irq;
    err = request_irq(data->IRQ, bma2x2_irq_handler, IRQF_TRIGGER_RISING,
            "bma2x2", data);
    if (err){
        KMSGERR(&client->dev, "Accel,bma2x2_probe could not request irq\n");
        goto  kfree_exit1;
        }
    disable_irq_nosync(data->IRQ);
    enable_irq_wake(data->IRQ);
#endif
    INIT_DELAYED_WORK(&data->work, bma2x2_work_func);
    atomic_set(&data->delay, BMA2X2_MAX_DELAY);
    atomic_set(&data->enable, 0);
    atomic_set(&data->enable_ddtap, 0);

/*set input device for polling sensor coordinates*/
    dev = input_allocate_device();
    if (!dev)
        {
        KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 input allocate device error\n");
        goto  kfree_irq;
        }
    dev->name = "accel";
    dev->id.bustype = BUS_I2C;
    dev->dev.parent = &data->bma2x2_client->dev;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);

    input_set_drvdata(dev, data);

    err = input_register_device(dev);
    if (err < 0) {
        KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 register device error\n");
        goto error_input;
    }

    data->input = dev;

/*set the virtual double tap device*/
    input_dev_ddtap = input_allocate_device();
    if (!input_dev_ddtap)
    {
        KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 input allocate ddtap device error\n");
        goto error_virtual_input;
    }
    data->ddtap = input_dev_ddtap;
    input_dev_ddtap->name = "ddtap";
    input_dev_ddtap->id.bustype = BUS_I2C;
    input_dev_ddtap->dev.parent = &data->bma2x2_client->dev;
    __set_bit(REL_MISC, input_dev_ddtap->relbit);
    input_set_capability(input_dev_ddtap, EV_REL, DOUBLE_TAP_INTERRUPT);

    input_set_drvdata(input_dev_ddtap, data);
    err = input_register_device(input_dev_ddtap);
        if (err < 0) {
            KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 register ddtap device error\n");
            goto error_virtual_device;
        }
/*create file system*/
    err = sysfs_create_group(&client->dev.kobj,
        &bma2x2_attribute_group);
    if (err < 0)
        goto error_sysfs;
    KMSGERR(&client->dev, "Accel,bma2x2_probe BMA250 probe Done\n");
    return 0;
error_sysfs:
    input_unregister_device(data->ddtap);
error_virtual_device:
    input_free_device(input_dev_ddtap);
error_virtual_input:
    input_unregister_device(data->input);
error_input:
    input_free_device(dev);
kfree_irq:
  free_irq(client->irq, data);
kfree_exit1:
    kfree(accel_pdata);
kfree_exit:
    kfree(data);
exit:
    return err;
}

static int __devexit bma2x2_remove(struct i2c_client *client)
{
    struct bma2x2_data *data = i2c_get_clientdata(client);

    bma2x2_set_enable(&client->dev, 0);
    bma2x2_set_int_enable(&client->dev,0);
    disable_irq_wake(client->irq);
    free_irq(client->irq, data);
    sysfs_remove_group(&data->input->dev.kobj, &bma2x2_attribute_group);
    input_unregister_device(data->input);
    input_unregister_device(data->ddtap);
    kfree(data->pdata);
    kfree(data);
    return 0;
}
#ifndef CONFIG_NORMAL_MODE
static int bma2x2_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct bma2x2_data *data = i2c_get_clientdata(client);
    KMSGINF(&client->dev, "Accel: %s\n", __func__);
    atomic_set(&data->suspend, ACCEL_I2C_SUSPEND);
    return 0;
}
static int bma2x2_resume(struct i2c_client *client)
{
    struct bma2x2_data *data = i2c_get_clientdata(client);
    KMSGINF(&client->dev, "Accel,%s\n", __func__);
    atomic_set(&data->suspend, ACCEL_I2C_RESUME);
    return 0;
}
#else
#define bma2x2_suspend        NULL
#define bma2x2_resume        NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id bma2x2_id[] = {
    { SENSOR_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, bma2x2_id);

static const struct of_device_id bma250_acc_match[] = {
    { .compatible = "bma,bma250_acc", },
    { },
};

static struct i2c_driver bma2x2_driver = {
    .driver = {
        .owner    = THIS_MODULE,
        .name    = SENSOR_NAME,
        .of_match_table = bma250_acc_match,
    },
    .suspend    = bma2x2_suspend,
    .resume        = bma2x2_resume,
    .id_table    = bma2x2_id,
    .probe        = bma2x2_probe,
    .remove        = __devexit_p(bma2x2_remove),

};

static int __init BMA2X2_init(void)
{
    pr_info("Accel,%s accelerometer driver: init\n",
                            SENSOR_NAME);

    return i2c_add_driver(&bma2x2_driver);
}

static void __exit BMA2X2_exit(void)
{
    pr_info("Accel,%s accelerometer driver: exit\n",
                                SENSOR_NAME);

    i2c_del_driver(&bma2x2_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA2X2 accelerometer sensor driver");
MODULE_LICENSE("GPL");

module_init(BMA2X2_init);
module_exit(BMA2X2_exit);

