/* linux/arch/arm/mach-msm/swift/board-swift-panel.c
 *
 * Copyright (C) 2009-2011 LGE Inc.
 * Author : MoonCheol Kang <knight0708@lge.com>
 *               2012 Henning Heinold <heinold@inf.fu-berlin.de> 
 * This software is licensed under the term of GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/rt9393_bl.h>

#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/board.h>
#include <mach/board_lge.h>

#include "devices.h"
#include "board-swift.h"

#define MSM_FB_LCDC_VREG_OP(name, op) \
do { \
        vreg = vreg_get(0, name); \
        if (vreg_##op(vreg)) \
                pr_err("%s: %s vreg operation failed \n", \
                        (vreg_##op == vreg_enable) ? "vreg_enable" \
                                : "vreg_disable", name); \
} while (0)

static char *msm_fb_vreg[] = {
        "gp1",
        "gp2",
};

static int mddi_power_save_on;

static int msm_fb_mddi_power_save(int on)
{
        struct vreg *vreg;
        int flag_on = !!on;

        if (mddi_power_save_on == flag_on)
                return 0;

        mddi_power_save_on = flag_on;

        if (on) {
                MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], enable);
                MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], enable);
        } else{
                MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], disable);
                MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], disable);
        }

        return 0;
}

static struct mddi_platform_data mddi_pdata = {
        .mddi_power_save = msm_fb_mddi_power_save,
};

static void __init msm_fb_add_devices(void)
{
        msm_fb_register_device("mdp", 0);
        msm_fb_register_device("pmdh", &mddi_pdata);
}

static struct rt9393_platform_data rt9393_pdata = {
	.gpio = GPIO_LCD_BL_EN,
};

static struct platform_device rt9393_platform_device = {
	.name	= "rt9393-bl",
	.id	= 0,
	.dev	= {
		.platform_data = &rt9393_pdata,
	}
};

extern int rt9393_set_intensity(struct backlight_device *);
static struct msm_panel_samsung_pdata mddi_samsung_panel_data = {
        .gpio = GPIO_LCD_RESET_N, /* lcd reset_n */
        .pmic_backlight = &rt9393_set_intensity, /* special pmic_backlight function */
};

static struct platform_device mddi_samsung_panel_device = {
        .name   = "mddi_samsung",
        .id     = 0,
        .dev    = {
                .platform_data = &mddi_samsung_panel_data,
        }
};

/* common functions */
void __init lge_add_lcd_devices(void)
{
	platform_device_register(&rt9393_platform_device);
        platform_device_register(&mddi_samsung_panel_device);
        msm_fb_add_devices();
}

