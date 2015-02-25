/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include "../../staging/android/timed_output.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>

#define LRA_CYCLE_US            4878
#define LRA_HALF_CYCLE_US       (LRA_CYCLE_US/2)
#define GPIO_STATE_HIGH         0x01
#define GPIO_STATE_LOW          0x00

struct lra_vib_device {
    struct device *dev;
};

static int vibrator_amp_gpio = -1;
static int vibrator_control_gpio = -1;
static struct hrtimer vibrator_timer;
static struct work_struct work_vibrator_deinit;
static unsigned long loop_count = 0;
static bool turn_on = false;


static void lra_deinit(void)
{
    pr_debug("Enter lra_deinit\n");

    loop_count = 0;
    turn_on = false;

    /* disable GPIOs and amplifier */
    gpio_set_value(vibrator_amp_gpio, GPIO_STATE_LOW);
    gpio_set_value(vibrator_control_gpio, GPIO_STATE_LOW);

    gpio_free(vibrator_amp_gpio);
    gpio_free(vibrator_control_gpio);

    pr_debug("Exit lra_deinit\n");
}

static void delayed_lra_deinit(struct work_struct *work)
{
    pr_debug("Enter delayed_lra_deinit\n");

    lra_deinit();

    pr_debug("Exit delayed_lra_deinit\n");
}

static void lra_init(void)
{
    int ret;

    pr_debug("Enter lra_init\n");

    ret = gpio_request(vibrator_amp_gpio, "vibrator_amp_gpio");
    pr_debug("ret_1: %d\n", ret);

    ret = gpio_direction_output(vibrator_amp_gpio, 0);
    pr_debug("ret_2: %d\n", ret);

    ret = gpio_request(vibrator_control_gpio, "vibrator_control_gpio");
    pr_debug("ret_3: %d\n", ret);

    ret = gpio_direction_output(vibrator_control_gpio, 0);
    pr_debug("ret_4: %d\n", ret);

    pr_debug("Exit lra_init\n");
}

static enum hrtimer_restart vibrator_on_off(struct hrtimer *timer)
{
    int ret_val = HRTIMER_NORESTART;

    if (loop_count > 0 ) {
        loop_count--;
        if (turn_on) {
            gpio_set_value(vibrator_control_gpio, GPIO_STATE_HIGH);
            turn_on = false;
        } else {
            gpio_set_value(vibrator_control_gpio, GPIO_STATE_LOW);
            turn_on = true;
        }
        hrtimer_forward(&vibrator_timer, ktime_get(), ktime_set(0, LRA_HALF_CYCLE_US * 1000));
        ret_val = HRTIMER_RESTART;
    } else {
        schedule_work(&work_vibrator_deinit);
        ret_val = HRTIMER_NORESTART;
    }
    return ret_val;
}

void lra_one_shot(uint16_t duration_ms)
{
    pr_debug("Enter lra_one_shot\n");

    if (work_pending(&work_vibrator_deinit)) {
        cancel_work_sync(&work_vibrator_deinit);
    }
    hrtimer_cancel(&vibrator_timer);
    loop_count = (duration_ms * 1000)/ LRA_HALF_CYCLE_US;

    pr_debug("loop_count: %lu\n", loop_count);

    turn_on = true;
    gpio_set_value(vibrator_amp_gpio, GPIO_STATE_HIGH);
    hrtimer_start(&vibrator_timer, ktime_set(0, LRA_HALF_CYCLE_US * 1000), HRTIMER_MODE_REL);

    pr_debug("Exit lra_one_shot\n");
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
    int ret_val = 0;

    pr_debug("Enter vibrator_get_time\n");

    if (loop_count > 0) {
        ret_val = (loop_count * LRA_HALF_CYCLE_US)/ 1000; /* ms */
    }

    pr_debug("Enter vibrator_get_time, ret_val: %d\n", ret_val);

    return ret_val;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
    pr_debug("Enter vibrator_enable\n");
    pr_debug("timeout: %d\n", value);

    lra_init();
    lra_one_shot(value);

    pr_debug("Exit vibrator_enable\n");
}

static struct timed_output_dev timed_vibrator = {
    .name = "vibrator",
    .get_time = vibrator_get_time,
    .enable = vibrator_enable,
};

static int __init lra_vib_probe(struct platform_device *pdev)
{
    struct lra_vib_device *lra_vibra;

    pr_debug("Enter lra_vib_probe\n");

    if (!pdev) {
        pr_err("pdev is null\n");
        return -ENOMEM;
    }
    if (!pdev->dev.of_node) {
        pr_err("No platform supplied from device tree\n");
        return -ENODATA;
    }
    lra_vibra = kzalloc(sizeof(struct lra_vib_device), GFP_KERNEL);
    if (!lra_vibra) {
        pr_err("Could not allocate lra_vib_device\n");
        return -ENOMEM;
    }
    lra_vibra->dev = &pdev->dev;
    platform_set_drvdata(pdev, lra_vibra);
    hrtimer_init(&vibrator_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    vibrator_timer.function = vibrator_on_off;
    INIT_WORK(&work_vibrator_deinit, delayed_lra_deinit);
    vibrator_amp_gpio = of_get_named_gpio(pdev->dev.of_node, "qcom,vibrator-amp-gpio", 0);

    pr_debug("vibrator_amp_gpio: %d\n", vibrator_amp_gpio);

    vibrator_control_gpio = of_get_named_gpio(pdev->dev.of_node, "qcom,vibrator-control-gpio", 0);

    pr_debug("vibrator_control_gpio: %d\n", vibrator_control_gpio);

    timed_output_dev_register(&timed_vibrator);

    pr_debug("Exit lra_vib_probe\n");

    return 0;
}

static int __devexit lra_vib_remove(struct platform_device *pdev)
{
    struct lra_vib_device *lra_vib;

    pr_debug("Enter lra_vib_remove\n");

    lra_vib = platform_get_drvdata(pdev);
    if (lra_vib == NULL) {
        pr_err("pdev is null\n");
        return -EINVAL;
    }
    if (work_pending(&work_vibrator_deinit)) {
        cancel_work_sync(&work_vibrator_deinit);
    }
    hrtimer_cancel(&vibrator_timer);
    lra_deinit();
    platform_set_drvdata(pdev, NULL);
    timed_output_dev_unregister(&timed_vibrator);
    kfree(lra_vib);

    pr_debug("Exit lra_vib_remove\n");

    return 0;
}

static const struct of_device_id lra_vib_match[] = {
    {.compatible = "qcom,lravibra"},
    {}
};

static struct platform_driver lra_vib_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "lra_vibra",
        .of_match_table = lra_vib_match,
    },
    .remove = __devexit_p(lra_vib_remove),
};

static int __init lra_vib_init(void)
{
    pr_debug("lra_vib_init\n");

    return platform_driver_probe(&lra_vib_driver, lra_vib_probe);
}
module_init(lra_vib_init);

static void __exit lra_vib_exit(void)
{
    pr_debug("lra_vib_exit\n");

    platform_driver_unregister(&lra_vib_driver);
}
module_exit(lra_vib_exit);

MODULE_DESCRIPTION("LRA vibrator driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jaakko Rautiainen <jaakko.rautiainen@nokia.com>");
