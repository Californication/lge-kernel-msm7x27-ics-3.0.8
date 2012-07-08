/* arch/arm/mach-msm/lge/board-swift-bt.c
 * Copyright (C) 2009 LGE, Inc.
 * miroslav_mm (c) myroslavmm@gmail.com
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/types.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>
#include "board-swift.h"
#include <linux/delay.h>
#include <linux/rfkill.h>

#define GPIO_BT_RESET_N     96
#define GPIO_BT_REG_ON      21
#define GPIO_WL_RESET_N     CONFIG_BCM4325_GPIO_WL_RESET

#ifdef CONFIG_BT
static unsigned bt_config_power_on[] = {
	GPIO_CFG(BT_WAKE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* WAKE */
	GPIO_CFG(BT_RFR, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* RFR */
	GPIO_CFG(BT_CTS, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CTS */
	GPIO_CFG(BT_RX, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* Rx */
	GPIO_CFG(BT_TX, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* Tx */
	GPIO_CFG(BT_PCM_DOUT, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DOUT */
	GPIO_CFG(BT_PCM_DIN, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DIN */
	GPIO_CFG(BT_PCM_SYNC, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_SYNC */
	GPIO_CFG(BT_PCM_CLK, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_CLK */
	GPIO_CFG(BT_HOST_WAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* HOST_WAKE */
	GPIO_CFG(BT_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* RESET_N */
};
static unsigned bt_config_power_off[] = {
	GPIO_CFG(BT_WAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* WAKE */
	GPIO_CFG(BT_RFR, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* RFR */
	GPIO_CFG(BT_CTS, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CTS */
	GPIO_CFG(BT_RX, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* Rx */
	GPIO_CFG(BT_TX, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* Tx */
	GPIO_CFG(BT_PCM_DOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_DOUT */
	GPIO_CFG(BT_PCM_DIN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_DIN */
	GPIO_CFG(BT_PCM_SYNC, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_SYNC */
	GPIO_CFG(BT_PCM_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_CLK */
	GPIO_CFG(BT_HOST_WAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* HOST_WAKE */
	GPIO_CFG(BT_RESET_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* RESET_N */	
};

static int swift_bluetooth_toggle_radio(void *data, bool blocked)
{
	int ret;
	int (*power_control)(int enable);

	power_control = ((struct bluetooth_platform_data *)data)->bluetooth_power;
	ret = (*power_control)((blocked ==  0) ? 1 : 0);
	return ret;
}

static int swift_bluetooth_power(int on)
{
	int pin, rc;

	printk(KERN_DEBUG "%s\n", __func__);

	if (on) {
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}
		/*Regulator On*/
		if(!gpio_get_value(GPIO_BT_REG_ON))
			gpio_set_value(GPIO_BT_REG_ON, 1);
		msleep (200);		
		/*Reset Off*/
		gpio_set_value(GPIO_BT_RESET_N, 0); 
		msleep(15);/*BCM4325 Requirement*/		
		/*Reset On*/
		gpio_set_value(GPIO_BT_RESET_N, 1);
		/*for safety*/
		msleep(200);
	} else {
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}
		gpio_set_value(GPIO_BT_RESET_N, 0);
		if(!gpio_get_value(GPIO_WL_RESET_N))
			gpio_set_value(GPIO_BT_REG_ON, 0);	
	}
	return 0;
}

static struct bluetooth_platform_data swift_bluetooth_data = {
	.bluetooth_power = swift_bluetooth_power,
	.bluetooth_toggle_radio = swift_bluetooth_toggle_radio,
};

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
	.dev = {
		.platform_data = &swift_bluetooth_data,
	},		
};


static void __init bt_power_init(void)
{
       swift_bluetooth_power(1);
       msleep (100);
       swift_bluetooth_power(0);
}
#else
#define bt_power_init(x) do {} while (0)
#endif

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= BT_HOST_WAKE,
		.end	= BT_HOST_WAKE,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= BT_WAKE,
		.end	= BT_WAKE,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(BT_HOST_WAKE),
		.end	= MSM_GPIO_TO_INT(BT_HOST_WAKE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct bluesleep_platform_data swift_bluesleep_data = {
	.bluetooth_port_num = 0,
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
	.dev = {
		.platform_data = &swift_bluesleep_data,
	},	
};

void __init lge_add_btpower_devices(void)
{
	bt_power_init();
#ifdef CONFIG_BT
	platform_device_register(&msm_bt_power_device);
#endif
	platform_device_register(&msm_bluesleep_device);
}
