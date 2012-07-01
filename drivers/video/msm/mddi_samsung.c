/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <asm/gpio.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>
#include <linux/delay.h>

#include "msm_fb.h"
#include "mddihosti.h"

#define DEBUG_LCD 0      

#define TM_GET_DID(id)  ((id) & 0xff)
#define TM_GET_PID(id)  (((id) & 0xff00)>>8)
#define LCD_CONTROL_BLOCK_BASE  0x110000
#define INTFLG          LCD_CONTROL_BLOCK_BASE|(0x18)
#define INTMSK          LCD_CONTROL_BLOCK_BASE|(0x1c)
#define VPOS            LCD_CONTROL_BLOCK_BASE|(0xc0)

typedef enum {
        SS_POWER_OFF,
        SS_POWER_ON,
        SS_DISPLAY_ON,
        SS_DISPLAY_OFF
} mddi_samsung_state_t;

static uint32 mddi_samsung_curr_vpos;
static boolean mddi_samsung_monitor_refresh_value = FALSE;
static boolean mddi_samsung_report_refresh_measurements = FALSE;

/* These value is taken from EVE source */
/* Dot clock (10MHz) / pixels per row (320) = rows_per_second
 * Rows Per second, this number arrived upon empirically
 * after observing the timing of Vsync pulses
 */
static uint32 mddi_samsung_rows_per_second = 31250;
static uint32 mddi_samsung_rows_per_refresh = 480;
/* rows_per_refresh / rows_per_second */
static uint32 mddi_samsung_usecs_per_refresh = 15360;
extern boolean mddi_vsync_detect_enabled;

static msm_fb_vsync_handler_type mddi_samsung_vsync_handler;
static void *mddi_samsung_vsync_handler_arg;
static uint16 mddi_samsung_vsync_attempts;

static mddi_samsung_state_t samsung_power_state = SS_POWER_OFF;

static struct msm_panel_samsung_pdata *mddi_samsung_pdata;

static int mddi_samsung_lcd_on(struct platform_device *pdev);
static int mddi_samsung_lcd_off(struct platform_device *pdev);

static void mddi_samsung_lcd_panel_poweron(void);
static void mddi_samsung_lcd_panel_poweroff(void);

struct display_table {
        unsigned reg;
        unsigned char count;
        unsigned int val_list[5];
};

/* START table for innotek panel from EVE source */
#define REGFLAG_DELAY           0xFFFE

static struct display_table mddi_samsung_power_on_table[] = {
	/* Power on Sequence */
	{0x11, 1, {0x00000000}},
	{REGFLAG_DELAY, 120,{}},
	{0xF3, 3, {0x00000101, 0x7F2C4407, 0x00000000}},
	{0xF4, 2, {0x5527230D, 0x00000011}},
	{0xF5, 2, {0xF0080010, 0x00001F05}},
	/* Normal Mode */
	{0x13, 1, {0x00000000}},
	/* Column address */
	{0x2A, 1, {0x3F010000}},
	/* Page address */
	{0x2B, 1, {0xDF010000}},
	/* Memory data address Control */
	{0x36, 1, {0x00000048}},
	/* Interface pixel format */
	{0x3A, 1, {0x00000005}},
	/* Display Control Set */
	{0xF2, 3, {0x03031414, 0x03080803, 0x00151500}},
	/* Tearing Effect Lion ON */
	{0x35, 1, {0x00000000}},
	/* CABC Control */
	{0x51, 1, {0x000000FF}},
	{0x53, 1, {0x0000002C}},
	{0x55, 1, {0x00000003}},
	{0x5E, 1, {0x00000000}},
	{0xCA, 1, {0x003F8080}},
	{0xCB, 1, {0x00000003}},
	{0xCC, 2, {0x008F0120, 0x000000EF}},
	{0xCD, 1, {0x00009704}},
	/* Positive Gamma RED */
	{0xF7, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Negative Gamma RED */
	{0xF8, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Positive Gamma GREEN */
	{0xF9, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Negative Gamma GREEN */
	{0xFA, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Positive Gamma BLUE */
	{0xFB, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Negative Gamma BLUE */
	{0xFC, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Gate Control Register */
	{0xFD, 1, {0x00003B11}},
	/* GRAM Write */
	{0x2C, 1, {0x00000000}},
	/* Display On */
	{0x29, 1, {0x00000000}},
};

static struct display_table mddi_samsung_power_off_table[] = {
	{0x28, 1, {0x00000000}},
	{0x10, 1, {0x00000000}},
	{REGFLAG_DELAY, 120,{}},
	{0xE0, 1, {0x00000002}},
};

static struct display_table mddi_samsung_position_table[] = {
    /* Column Address */
    {0x2A, 1, {0x3F010000}},
    /* Page Address */
    {0x2B, 1, {0xDF010000}},
};

static void display_table(struct display_table *table, unsigned int count)
{
	unsigned int i;
	for(i = 0; i < count; i++) {
        unsigned reg;
        reg = table[i].reg;
		
        switch (reg) {
		case REGFLAG_DELAY :
			mdelay(table[i].count - 2);
			break;
		default:
			mddi_host_register_cmds_write32(reg, table[i].count, table[i].val_list,
					0, 0, 0);
#if DEBUG_LCD
		       pr_debug("reg : %x, val : 0x%X.\n",
					reg, table[i].val_list[0]);
#endif
               break;
       	}
    }
}
/* END table for innotek panel from EVE source */

/* vsync is not used on the gt540 */
static void mddi_samsung_vsync_set_handler(msm_fb_vsync_handler_type handler,	/*ISR to be excuted */
					void *arg)
{
	boolean error = FALSE;
	unsigned long flags;

	pr_debug("%s : handler = %x\n", __func__, (unsigned int)handler);

	/* Disable interrupts */
	spin_lock_irqsave(&mddi_host_spin_lock, flags);
	/* INTLOCK(); */

	if (mddi_samsung_vsync_handler != NULL) {
		error = TRUE;
	} else {
		/* Register the handler for this particular GROUP interrupt source */
		mddi_samsung_vsync_handler = handler;
		mddi_samsung_vsync_handler_arg = arg;
	}
	
	/* Restore interrupts */
	spin_unlock_irqrestore(&mddi_host_spin_lock, flags);
	/* MDDI_INTFREE(); */
	if (error) {
		MDDI_MSG_ERR("MDDI: Previous Vsync handler never called\n");
	} else {
		/* Enable the vsync wakeup */
		mddi_queue_register_write(INTMSK, 0x0000, FALSE, 0);

		mddi_samsung_vsync_attempts = 1;
		mddi_vsync_detect_enabled = TRUE;
	}
}

static void mddi_samsung_vsync_detected(boolean detected)
{
	/* static timetick_type start_time = 0; */
	static struct timeval start_time;
	static boolean first_time = TRUE;
	/* unit32 mdp_cnt_val = 0; */
	/* timetick_type elapsed_us; */
	struct timeval now;
	uint32 elapsed_us;
	uint32 num_vsyncs;
#if DEBUG_LCD
	pr_debug("%s : detected = %d\n", __func__, detected);
#endif
	if ((detected) || (mddi_samsung_vsync_attempts > 5)) {
		if ((detected) || (mddi_samsung_monitor_refresh_value)) {
			/* if (start_time != 0) */
			if (!first_time) {
				jiffies_to_timeval(jiffies, &now);
				elapsed_us =
					(now.tv_sec - start_time.tv_sec) * 1000000 +
					now.tv_usec - start_time.tv_usec;
				/*
				 * LCD is configured for a refresh every usecs,
				 *  so to determine the number of vsyncs that
				 *  have occurred since the last measurement
				 *  add half that to the time difference and
				 *  divide by the refresh rate.
				 */
				num_vsyncs = (elapsed_us +
						(mddi_samsung_usecs_per_refresh >>
						 1))/
					mddi_samsung_usecs_per_refresh;
				/*
				 * LCD is configured for * hsyncs (rows) per
				 * refresh cycle. Calculate new rows_per_second
				 * value based upon these new measuerments.
				 * MDP can update with this new value.
				 */
				mddi_samsung_rows_per_second =
					(mddi_samsung_rows_per_refresh * 1000 *
					 num_vsyncs) / (elapsed_us / 1000);
			}
			/* start_time = timetick_get();*/
			first_time = FALSE;
			jiffies_to_timeval(jiffies, &start_time);
			if (mddi_samsung_report_refresh_measurements) {
				(void)mddi_queue_register_read_int(VPOS,
									&mddi_samsung_curr_vpos);
				/* mdp_cnt_val = MDP_LINE_COUNT; */
			}
		}
		/* if detected = TRUE, client initiated wakeup was detected */
		if (mddi_samsung_vsync_handler != NULL) {
			(*mddi_samsung_vsync_handler)
				(mddi_samsung_vsync_handler_arg);
			mddi_samsung_vsync_handler = NULL;
		}
		mddi_vsync_detect_enabled = FALSE;
		mddi_samsung_vsync_attempts = 0;
		/* need to disable the interrupt wakeup */
		if (!mddi_queue_register_write_int(INTMSK, 0x0001))
			MDDI_MSG_ERR("Vsync interrupt disable failed!\n");
		if (!detected) {
			/* give up after 5 failed attempts but show error */
			MDDI_MSG_NOTICE("Vsync detection failed!\n");
		} else if ((mddi_samsung_monitor_refresh_value) &&
				(mddi_samsung_report_refresh_measurements)) {
			MDDI_MSG_NOTICE("  Last Line Counter=%d!\n",
					mddi_samsung_curr_vpos);
			/* MDDI_MSG_NOTICE("  MDP Line Counter=%d!\n",mdp_cnt_val); */
			MDDI_MSG_NOTICE("  Lines Per Second=%d!\n",
					mddi_samsung_rows_per_second);
		}
		/* clear the interrupt */
		if (!mddi_queue_register_write_int(INTFLG, 0x0001))
			MDDI_MSG_ERR("Vsync interrupt clear failed!\n");
	} else {
		/* if detected = FALSE, we woke up from hibernation, but did not
		 * detect client initiated wakeup.
		 */
		mddi_samsung_vsync_attempts++;
	}
}

#ifdef CONFIG_FB_BACKLIGHT
static void mddi_samsung_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	struct msm_panel_samsung_pdata *pdata = mddi_samsung_pdata;
	struct backlight_device *bld;
	bld->props.brightness = mfd->bl_level;

	if (pdata && data->pmic_backlight)
		pmic_backlight(bld);
}
#endif


static void mddi_samsung_lcd_panel_poweron(void)
{
	struct msm_panel_samsung_pdata *pdata = mddi_samsung_pdata;
	if(pdata && pdata->gpio) {
#if DEBUG_LCD
	        pr_debug("%s : Reset pin = %d\n",__func__, pdata->gpio);
#endif
		gpio_set_value(pdata->gpio, 1);
		mdelay(9);
		gpio_set_value(pdata->gpio, 0);
		mdelay(9);
		gpio_set_value(pdata->gpio, 1);
	}
}

static void mddi_samsung_lcd_panel_poweroff(void) {
	struct msm_panel_samsung_pdata *pdata = mddi_samsung_pdata;
        if(pdata && pdata->gpio) {
		gpio_set_value(pdata->gpio, 0);
	}
}

int mddi_samsung_position(void)
{
	display_table(mddi_samsung_position_table, ARRAY_SIZE(mddi_samsung_position_table));
	return 0;
}
EXPORT_SYMBOL(mddi_samsung_position);

static int mddi_samsung_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(pdev);

        if (!mfd)
                return -ENODEV;
        if (mfd->key != MFD_KEY)
                return -EINVAL;

	mddi_samsung_lcd_panel_poweron();
	display_table(mddi_samsung_power_on_table, ARRAY_SIZE(mddi_samsung_power_on_table));
        samsung_power_state = SS_POWER_ON;

	return 0;
}

static int mddi_samsung_lcd_off(struct platform_device *pdev)
{
#if DEBUG_LCD
        pr_debug("%s : state = %d\n", __func__, ss_panel);
#endif
	display_table(mddi_samsung_power_off_table, ARRAY_SIZE(mddi_samsung_power_off_table));
        mddi_samsung_lcd_panel_poweroff();
        samsung_power_state = SS_POWER_OFF;

	return 0;
}

static int __devinit mddi_samsung_probe(struct platform_device *pdev)
{
        int err;

        if (pdev->id == 0) {
                pr_debug("%s : pdev->id = 0\n", __func__);
                mddi_samsung_pdata = pdev->dev.platform_data;
                return 0;
        }

	if (!mddi_samsung_pdata || (mddi_samsung_pdata && !mddi_samsung_pdata->gpio)) {
		pr_err("No platformdata or lcd_reset gpio set");
		return ENODEV;
	}

        err = gpio_request(mddi_samsung_pdata->gpio, 0);
        if (err < 0 ) {
                pr_err("Cannot get the gpio pin : %d\n", mddi_samsung_pdata->gpio);
                return err;
        }

	msm_fb_add_device(pdev);

        return 0;
}

static struct  platform_driver this_driver = {
  	.probe	= mddi_samsung_probe,
	.driver	= {
		.name	= "mddi_samsung",
	},
};

static struct msm_fb_panel_data samsung_panel_data = {
	.on	= mddi_samsung_lcd_on,
	.off	= mddi_samsung_lcd_off,
};
/*
static int ch_used[3];
*/

/*int mddi_samsung_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel) */
int mddi_samsung_device_register(struct msm_panel_info *pinfo)
{
	int ret;
	struct platform_device *pdev = NULL;
	pr_debug("%s\n", __func__);
/*
	if ((channel > 2) || ch_used[channel])
		return -ENODEV;*/

/*	if ((channel != INNOTEK) && */
/*	if(mddi_samsung_pdata && mddi_samsung_pdata->panel_num &&
		 (mddi_samsung_pdata->panel_num() < 2))
			return -ENODEV;

	ch_used[channel] = TRUE;*/
	
/*	pdev = platform_device_alloc("mddi_samsung", (panel << 8)|channel); */
	pdev = platform_device_alloc("mddi_samsung", 256);

	if (!pdev)
		return -ENOMEM;

/*	if (channel == INNOTEK) {*/
#ifdef CONFIG_FB_BACKLIGHT
		samsung_panel_data.set_backlight =
			&mddi_samsung_lcd_set_backlight;
#endif
		if (pinfo->lcd.vsync_enable) {
			samsung_panel_data.set_vsync_notifier =
				mddi_samsung_vsync_set_handler;
			mddi_lcd.vsync_detected =
				mddi_samsung_vsync_detected;
		}
	/*
	} else {
		samsung_panel_data.set_backlight = NULL;
		samsung_panel_data.set_vsync_notifier = NULL;
	}*/

	samsung_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &samsung_panel_data,
			sizeof(samsung_panel_data));
	if (ret) {
		pr_err("%s: adding platform_device_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: adding platform_device failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mddi_samsung_lcd_init(void)
{
	return platform_driver_register(&this_driver);
}

module_init(mddi_samsung_lcd_init);
MODULE_DESCRIPTION("Mddi Driver SAMSUNG(MSM)");
