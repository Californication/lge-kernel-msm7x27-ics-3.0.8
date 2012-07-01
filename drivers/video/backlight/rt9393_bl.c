/* swift-backlight
 *
 * Copyright (C) 2009-2011 LGE Inc.
 * Author : MoonCheol Kang <knight0708@lge.com>
 *               2012 Henning Heinold <heinold@ refactored based on atmel_pwm_bl driver
 *
 * This software is licensed under the term of the GNU General Public
 * License version 2, as published by the Free Sofrware Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANABILITY or FITNESS FOR A PARTICLULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/rt9393_bl.h>
#include <linux/device.h>
#if defined(CONFIG_RT9393_SYS_DEBUG)
#include <linux/fs.h>
#endif

#if defined(CONFIG_RT9393_MSG_DEBUG)
enum {
	BL_DEBUG_DATA = 1U << 6,
	BL_DEBUG_FUNC = 1U << 7, /* function debug*/
};

static unsigned int lge_bl_debug_mask;

module_param_named(debug_mask, lge_bl_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define KNIGHT_DBG(mask, fmt, args...) \
	do {\
		if ((mask) & lge_bl_debug_mask) \
				pr_info("[MC] [%-18s:%5d] " \
					fmt, __func__, __LINE__, ## args);\
		} while (0)
#else
#define KNIGHT_DBG(mask, fmt, args...)
#endif

#define MAX_BRIGHTNESS		32
#define VMAX_BRIGHTNESS		16
#define MIN_BRIGHTNESS		0

wait_queue_head_t backlight_wait_q;
atomic_t backlight_event_handle;
atomic_t lcd_event_handled;

enum power_status {
	POWER_DOWN = 0,
	POWER_ON
} power_state;

static unsigned short current_intensity = 0;
/* static unsigned short power_state = 0; */
static DEFINE_SPINLOCK(rt9393_lock);
struct timer_list timerblbl;

static struct rt9393_platform_data *pdata;

struct bl9393_driver_data {
        struct backlight_device *bldev;
        struct platform_device  *pdev;
        int gpio;
};

static int rt9393_power_down(void)
{
	gpio_set_value(pdata->gpio, 0);
	mdelay(4); /* 3ms < tSHDN */
	power_state = POWER_DOWN;
	current_intensity = 0;
	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	atomic_set(&backlight_event_handle, 0);

	return 0;
}

static int rt9393_power_on(void)
{
	gpio_set_value(pdata->gpio, 1);
	udelay(40); /* 30us < tIH.INIT */
	power_state = POWER_ON;
	current_intensity = MAX_BRIGHTNESS;
	KNIGHT_DBG(lge_bl_debug_mask, "\n");

	return 0;
}

static void bl_timer(unsigned long arg)
{
	if(atomic_read(&backlight_event_handle) == 0) {
		atomic_set(&lcd_event_handled, 1);
		wake_up(&backlight_wait_q);
		mod_timer(&timerblbl, jiffies + msecs_to_jiffies(20));
	}
}

static int rt9393_send_intensity(struct backlight_device *bldev)
{
	int i, circular;
	/* struct rt9393_driver_data *drvdata = bl_get_data(bldev); */
	int next_intensity = bldev->props.brightness;

	KNIGHT_DBG(lge_bl_debug_mask, "set value = %d, current value = %d \n",
			next_intensity, current_intensity);

	if ((atomic_read(&backlight_event_handle) == 0) && (next_intensity != 0)) {
		mod_timer(&timerblbl, jiffies + msecs_to_jiffies(600));
		wait_event_interruptible_timeout(backlight_wait_q,
				atomic_read(&lcd_event_handled), HZ/2);
		atomic_set(&backlight_event_handle, 1);
		msleep(50);
	}

	spin_lock(&rt9393_lock);
	if (next_intensity == current_intensity) {
		KNIGHT_DBG(lge_bl_debug_mask, "next == current, %d \n", next_intensity);
		spin_unlock(&rt9393_lock);
		return 0;
	}

	if (next_intensity > VMAX_BRIGHTNESS) {
		next_intensity = VMAX_BRIGHTNESS;
		KNIGHT_DBG(lge_bl_debug_mask, "the value is over virtual maximum(%d)\n",
				VMAX_BRIGHTNESS);
	}
	else if (next_intensity < MIN_BRIGHTNESS) {
		next_intensity = MIN_BRIGHTNESS;
		KNIGHT_DBG(lge_bl_debug_mask, "the value is under minimum(%d),\
				we will turn off the BL\n", MIN_BRIGHTNESS);
	}

	/* calculate the circular... */
	if (next_intensity < current_intensity) {
		circular = current_intensity - next_intensity;
		KNIGHT_DBG(lge_bl_debug_mask, "circular = %d\n", circular);
	} else {
		circular = 32 - (next_intensity - current_intensity);
		KNIGHT_DBG(lge_bl_debug_mask, "circular = %d\n", circular);
	}

	/* real control RT9393 */
	if (next_intensity == 0) { /* shutdown */
		rt9393_power_down();
	} else {
		if (power_state == POWER_DOWN) {
			rt9393_power_on();
			circular = 32 - next_intensity;
		}
		for (i=0; i < circular; i++) {
			gpio_set_value(pdata->gpio, 0);
			udelay(2); /* 0.5 us < tIL < 500us */
			gpio_set_value(pdata->gpio, 1);
			udelay(2); /* 0.5us < tIH  */
		}
	}
	current_intensity = next_intensity;
	spin_unlock(&rt9393_lock);

	if (next_intensity == 0 && pdata->led_off)
		pdata->led_off();

	return 0;
}

static int rt9393_get_intensity(struct backlight_device *bldev)
{
	return current_intensity;
}

int rt9393_set_intensity(struct backlight_device *bldev)
{
	rt9393_send_intensity(bldev);
	return 0;
}
EXPORT_SYMBOL(rt9393_set_intensity);

static const struct backlight_ops rt9393_ops = {
	.get_brightness = rt9393_get_intensity,
	.update_status = rt9393_set_intensity,
};

#if defined(CONFIG_RT9393_SYS_DEBUG)
ssize_t brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", current_intensity);
}

ssize_t brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;
	struct bl9393_driver_data *drvdata = dev_get_drvdata(dev);
	struct backlight_device *bldev = drvdata->bldev;

	sscanf(buf, "%d\n", &value);
	KNIGHT_DBG(lge_bl_debug_mask, "set value from sysfile system %d\n", value);
	bd->props.brightness = value;
	rt9393_send_intensity(bldev);

	return count;
}

ssize_t circular_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct bl9393_driver_data *drvdata = dev_get_drvdata(dev);
	gpio_set_value(drvdata->gpio, 0);
	udelay(2); /* 0.5 us < tIL < 500us */
	gpio_set_value(drvdata->gpio, 1);
	udelay(2); /* 0.5us < tIH */
	current_intensity = current_intensity - 1;
	if (current_intensity == 0)
		current_intensity = VMAX_BRIGHTNESS;

	return count;
}

ssize_t gpio_no_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", GPIO_BL_EN);
}

DEVICE_ATTR(bri, 0666, brightness_show, brightness_store);
DEVICE_ATTR(cir, 0664, NULL, circular_store);
DEVICE_ATTR(gpio, 0664, gpio_no_show, NULL);

#endif /* CONFIG_RT9393_SYS_DEBUG */

static int rt9393_probe(struct platform_device *pdev)
{
	int err;
	struct backlight_properties props;
        struct backlight_device *bldev;
	struct bl9393_driver_data *drvdata;

        pr_debug("PROBING BACKLIGHT\n");

        pdata = pdev->dev.platform_data;
        if (!pdata) {
                return -ENODEV;
        }

	drvdata = kzalloc(sizeof(struct bl9393_driver_data), GFP_KERNEL);
        if (!drvdata)
                return -ENOMEM;

        drvdata->pdev = pdev;
	drvdata->gpio = pdata->gpio;

	err = gpio_request(drvdata->gpio, 0);

	if (err < 0 ) {
		dev_err(&pdev->dev, "Cannot get the gpio pin : %d\n", drvdata->gpio);
		kfree(drvdata);
		return err;
	}

	memset(&props, 0, sizeof(struct backlight_properties));

	bldev = backlight_device_register("rt9393", &pdev->dev, NULL, &rt9393_ops,&props);
	if (IS_ERR(bldev)) {
		dev_err(&pdev->dev, "failed to register backlight device\n");
		kfree(drvdata);
		return PTR_ERR(bldev);
	}

	drvdata->bldev = bldev;

	platform_set_drvdata(pdev, drvdata);

	if (system_state == SYSTEM_BOOTING) {
		current_intensity = 16;
		power_state = POWER_ON;
	} else {
		rt9393_power_down();
	}

	init_waitqueue_head(&backlight_wait_q);
	atomic_set(&backlight_event_handle, 1);
	atomic_set(&lcd_event_handled, 0);

	bldev->props.max_brightness = VMAX_BRIGHTNESS;
	bldev->props.power = 0; /* FB_BLANK_UNBLANK; */
	bldev->props.brightness = MAX_BRIGHTNESS/2;
	/* power_state = POWER_DOWN; */
	rt9393_set_intensity(bldev);

	setup_timer(&timerblbl, bl_timer, (unsigned long)pdev);

#if defined(CONFIG_RT9393_SYS_DEBUG)
	err = device_create_file(&bldev->dev, &dev_attr_bri);
	err = device_create_file(&bldev->dev, &dev_attr_cir);
	err = device_create_file(&bldev->dev, &dev_attr_gpio);
#endif
	return 0;

}

static int rt9393_remove(struct platform_device *pdev)
{
	struct bl9393_driver_data *drvdata = platform_get_drvdata(pdev);

	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	backlight_device_unregister(drvdata->bldev);
	platform_set_drvdata(pdev, NULL);
        kfree(drvdata);

	return 0;
}

static int rt9393_suspend(struct platform_device *pdev, pm_message_t state)
{
	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	return 0;
}

static int rt9393_resume(struct platform_device *pdev)
{
	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	return 0;
}

static struct platform_driver this_driver = {
	.probe 		= rt9393_probe,
	.remove		= rt9393_remove,
	.suspend	= rt9393_suspend,
	.resume		= rt9393_resume,
	.driver		= {
		.name	= "rt9393-bl",
	},
};

static int __init rt9393_init(void)
{
	int ret;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		pr_err("rt9393: platform driver cannot register\n");
		return ret;
	}
	return 0;
}
module_init(rt9393_init);

static void __exit rt9393_exit(void)
{
	platform_driver_unregister(&this_driver);
}
module_exit(rt9393_exit);

MODULE_DESCRIPTION("RT9393 driver for LCD Backlight");
MODULE_LICENSE("GPL");

