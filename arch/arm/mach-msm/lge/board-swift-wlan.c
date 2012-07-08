/* arch/arm/mach-msm/lge/board-swift-mmc.c
 * Copyright (C) 2010 LGE Corporation.
 * Author: SungEun Kim <cleaneye.kim@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/mach/mmc.h>
#include <asm/mach-types.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>

#include "board-swift.h"

static void sdcc_gpio_init(void) {
	/* SDC2 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
#endif
}

static unsigned sdcc_cfg_data[][6] = {
	/* SDC2 configs */
	{
	GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	},
};

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable) {

	int i, rc;

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);

	for (i = 0; i < ARRAY_SIZE(sdcc_cfg_data[dev_id - 1]); i++) {
		rc = gpio_tlmm_config(sdcc_cfg_data[dev_id - 1][i],
			enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, sdcc_cfg_data[dev_id - 1][i], rc);
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd) {

	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			rc = vreg_disable(vreg_mmc);
			if (rc)
				pr_err("%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		rc = vreg_set_level(vreg_mmc, VREG_SD_LEVEL);
		if (!rc)
			rc = vreg_enable(vreg_mmc);
		if (rc)
			pr_err("%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);

	return 0;
}

#if defined(CONFIG_LGE_BCM432X_PATCH)
static unsigned int bcm432x_sdcc_wlan_slot_status(struct device *dev) {

	pr_err("%s: %d %d\n", __func__, CONFIG_BCM4325_GPIO_WL_RESET, gpio_get_value(CONFIG_BCM4325_GPIO_WL_RESET));
	return gpio_get_value(CONFIG_BCM4325_GPIO_WL_RESET);
}

static struct mmc_platform_data bcm432x_sdcc_wlan_data = {
	.ocr_mask   	= MMC_VDD_30_31,
	.translate_vdd	= msm_sdcc_setup_power,
	.status     	= bcm432x_sdcc_wlan_slot_status,
	.status_irq	= MSM_GPIO_TO_INT(CONFIG_BCM4325_GPIO_WL_RESET),
	.irq_flags      = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

static void __init msm7x2x_init_wlan(void) {

	vreg_mmc = vreg_get(NULL, "wlan");
	if (IS_ERR(vreg_mmc)) {
		pr_err("%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_mmc));
		return;
	}

	sdcc_gpio_init();

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
#if defined(CONFIG_LGE_BCM432X_PATCH)
	/* GPIO config */
	gpio_tlmm_config(GPIO_CFG(CONFIG_BCM4325_GPIO_WL_REGON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(CONFIG_BCM4325_GPIO_WL_REGON, 0);

	gpio_tlmm_config(GPIO_CFG(CONFIG_BCM4325_GPIO_WL_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(CONFIG_BCM4325_GPIO_WL_RESET, 0);

	gpio_tlmm_config(GPIO_CFG(CONFIG_BCM4325_GPIO_WL_HOSTWAKEUP, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	/* Register platform device */
	msm_add_sdcc(2, &bcm432x_sdcc_wlan_data);

	/* Enable RESET IRQ for wlan card detect */
	enable_irq(gpio_to_irq(CONFIG_BCM4325_GPIO_WL_RESET));
#endif
#endif
}

void __init lge_add_wlan_devices(void)
{
	msm7x2x_init_wlan();
}
