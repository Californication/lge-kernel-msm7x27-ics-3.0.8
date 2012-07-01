/* drivers/video/msm/src/panel/mddi/mddi_innotek.h
 *
 * Copyright (C) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Copyright (C) 2009-2011 LGE Inc.
 *
 * All source code in this file is licensed under the following license
 * except where indicated.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include "msm_fb.h"
#include "mddihost.h"

/* for future use if we have more devices with the samsung driveric */
/*
static u32 mddi_samsung_innotek_panel_detect(void)
{
	return machine_is_msm7x27_swift();
}
*/

static int __init mddi_samsung_innotek_hvga_init(void)
{
	int ret;
	struct msm_panel_info pinfo;
	/*u32 panel;*/

	pr_info("Swift innoteck MDDI Init\n");

	/* panel = mddi_samsung_innotek_panel_detect(); */

	pinfo.xres = 320;
	pinfo.yres = 480;
	MSM_FB_SINGLE_MODE_PANEL(&pinfo);
	pinfo.type = MDDI_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 16;
	pinfo.lcd.vsync_enable = FALSE;
	/* vsync is not used 
	 * (mddi_innotek_rows_per_second * 100) /
	 * mddi_innotek_rows_per_refresh;
         */
	pinfo.lcd.refx100 = (31250 * 100) / 480;
	pinfo.lcd.v_back_porch = 14;
	pinfo.lcd.v_front_porch = 6;
	pinfo.lcd.v_pulse_width = 4;
	pinfo.lcd.hw_vsync_mode = FALSE;
	pinfo.lcd.vsync_notifier_period = (1 * HZ);

	pinfo.bl_max = 32;
	pinfo.bl_min = 1;

	pinfo.clk_rate = 122880000;
	pinfo.clk_min =  120000000;
	pinfo.clk_max =  130000000;
	pinfo.fb_num = 2;

	ret = mddi_samsung_device_register(&pinfo);
	if (ret) {
		pr_err("%s: failed to register device!\n", __func__);
		return ret;
	}
	return ret;
}

module_init(mddi_samsung_innotek_hvga_init)
