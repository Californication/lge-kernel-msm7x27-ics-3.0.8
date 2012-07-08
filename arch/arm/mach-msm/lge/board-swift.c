/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010 LGE. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/power_supply.h>
#include <mach/msm_memtypes.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/pmic.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>
#include <mach/socinfo.h>
#include "board-msm7627-regulator.h"
#include "devices.h"
#include "clock.h"
#include "acpuclock.h"
#include "msm-keypad-devices.h"
#include <mach/pm.h>
#ifdef CONFIG_ARCH_MSM7X27
#include <linux/msm_kgsl.h>
#endif
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/board_lge.h>
#include "board-swift.h"

//void __init swift_init_timed_vibrator(void);

extern struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR];

/* board-specific usb data definitions */
/* QCT originals are in device_lge.c, not here */
#ifdef CONFIG_USB_G_ANDROID
/* LGE_CHANGE
 * Currently, LG Android host driver has 2 USB bind orders as following;
 * - Android Platform : MDM + DIAG + GPS + UMS + ADB
 * - Android Net	  : DIAG + NDIS + MDM + GPS + UMS
 *
 * To bind LG AndroidNet, we add ACM function named "acm2".
 * Please refer to drivers/usb/gadget/f_acm.c.
 * 2011-01-12, hyunhui.park@lge.com
 */

/* The binding list for LGE Android USB */
char *usb_functions_lge_all[] = {
#ifdef CONFIG_USB_ANDROID_MTP
	"mtp",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	"ecm",
	"acm2",
#endif
#ifdef CONFIG_USB_F_SERIAL
	"nmea",
#endif
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
	"usb_cdrom_storage",
	"charge_only",
#endif
	"usb_mass_storage",
	"adb",
};


/* LG Android Platform */
char *usb_functions_lge_android_plat[] = {
	"acm", "diag", "nmea", "usb_mass_storage",
};

char *usb_functions_lge_android_plat_adb[] = {
	"acm", "diag", "nmea", "usb_mass_storage", "adb",
};

#ifdef CONFIG_USB_ANDROID_CDC_ECM
/* LG AndroidNet */
char *usb_functions_lge_android_net[] = {
	"diag", "ecm", "acm2", "nmea", "usb_mass_storage",
};

char *usb_functions_lge_android_net_adb[] = {
	"diag", "ecm", "acm2", "nmea", "usb_mass_storage", "adb",
};
#endif

#ifdef CONFIG_USB_ANDROID_RNDIS
/* LG AndroidNet RNDIS ver */
char *usb_functions_lge_android_rndis[] = {
	"rndis",
};

char *usb_functions_lge_android_rndis_adb[] = {
	"rndis", "adb",
};
#endif

#ifdef CONFIG_USB_ANDROID_MTP
/* LG AndroidNet MTP (in future use) */
char *usb_functions_lge_android_mtp[] = {
	"mtp",
};

char *usb_functions_lge_android_mtp_adb[] = {
	"mtp", "adb",
};
#endif

/* LG Manufacturing mode */
char *usb_functions_lge_manufacturing[] = {
	"acm", "diag",
};

/* Mass storage only mode */
char *usb_functions_lge_mass_storage_only[] = {
	"usb_mass_storage",
};

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
/* CDROM storage only mode(Autorun default mode) */
char *usb_functions_lge_cdrom_storage_only[] = {
	"usb_cdrom_storage",
};

char *usb_functions_lge_cdrom_storage_adb[] = {
	"usb_cdrom_storage", "adb",
};

char *usb_functions_lge_charge_only[] = {
	"charge_only",
};
#endif

/* QCT original's composition array is existed in device_lge.c */
struct android_usb_product usb_products[] = {
	{
		.product_id = 0x618E,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_plat),
		.functions = usb_functions_lge_android_plat,
	},
	{
		.product_id = 0x618E,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_plat_adb),
		.functions = usb_functions_lge_android_plat_adb,
	},
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		.product_id = 0x61A2,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_net),
		.functions = usb_functions_lge_android_net,
	},
	{
		.product_id = 0x61A1,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_net_adb),
		.functions = usb_functions_lge_android_net_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id = 0x61DA,
		.num_functions  = ARRAY_SIZE(usb_functions_lge_android_rndis),
		.functions  = usb_functions_lge_android_rndis,
	},
	{
		.product_id = 0x61D9,
		.num_functions  = ARRAY_SIZE(usb_functions_lge_android_rndis_adb),
		.functions  = usb_functions_lge_android_rndis_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_MTP
	/* FIXME: These pids are experimental.
	 * Don't use them in official version.
	 */
	{
		.product_id = 0x61C7,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_mtp),
		.functions = usb_functions_lge_android_mtp,
	},
	{
		.product_id = 0x61F9,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_mtp_adb),
		.functions = usb_functions_lge_android_mtp_adb,
	},
#endif
	{
		.product_id = 0x6000,
		.num_functions  = ARRAY_SIZE(usb_functions_lge_manufacturing),
		.functions  = usb_functions_lge_manufacturing,
	},
	{
		.product_id = 0x61C5,
		.num_functions = ARRAY_SIZE(usb_functions_lge_mass_storage_only),
		.functions = usb_functions_lge_mass_storage_only,
	},
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
	{
		/* FIXME: This pid is just for test */
		.product_id = 0x91C8,
		.num_functions = ARRAY_SIZE(usb_functions_lge_cdrom_storage_only),
		.functions = usb_functions_lge_cdrom_storage_only,
	},
	{
		.product_id = 0x61A6,
		.num_functions = ARRAY_SIZE(usb_functions_lge_cdrom_storage_adb),
		.functions = usb_functions_lge_cdrom_storage_adb,
	},
	{
		/* Charge only doesn't have no specific pid */
		.product_id = 0xFFFF,
		.num_functions = ARRAY_SIZE(usb_functions_lge_charge_only),
		.functions = usb_functions_lge_charge_only,
	},
#endif
};

struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns      = 1,
	.vendor     = "LGE",
	.product    = "Android Platform",
	.release    = 0x0100,
};

struct platform_device usb_mass_storage_device = {
	.name   = "usb_mass_storage",
	.id 	= -1,
	.dev    = {
		.platform_data = &mass_storage_pdata,
	},
};

struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID   	= 0x1004,
	.vendorDescr    = "LG Electronics Inc.",
};

struct platform_device rndis_device = {
	.name   = "rndis",
	.id 	= -1,
	.dev    = {
		.platform_data = &rndis_pdata,
	},
};


#ifdef CONFIG_USB_ANDROID_CDC_ECM
/* LGE_CHANGE
 * To bind LG AndroidNet, add platform data for CDC ECM.
 * 2011-01-12, hyunhui.park@lge.com
 */
struct usb_ether_platform_data ecm_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID   	= 0x1004,
	.vendorDescr    = "LG Electronics Inc.",
};

struct platform_device ecm_device = {
	.name   = "ecm",
	.id 	= -1,
	.dev    = {
		.platform_data = &ecm_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID_ACM
/* LGE_CHANGE
 * To bind LG AndroidNet, add platform data for CDC ACM.
 * 2011-01-12, hyunhui.park@lge.com
 */
struct acm_platform_data acm_pdata = {
	.num_inst	    = 1,
};

struct platform_device acm_device = {
	.name   = "acm",
	.id 	= -1,
	.dev    = {
		.platform_data = &acm_pdata,
	},
};
#endif

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
/* LGE_CHANGE
 * Add platform data and device for cdrom storage function.
 * It will be used in Autorun feature.
 * 2011-03-02, hyunhui.park@lge.com
 */
struct usb_cdrom_storage_platform_data cdrom_storage_pdata = {
	.nluns      = 1,
	.vendor     = "LGE",
	.product    = "Android Platform",
	.release    = 0x0100,
};

struct platform_device usb_cdrom_storage_device = {
	.name   = "usb_cdrom_storage",
	.id = -1,
	.dev    = {
		.platform_data = &cdrom_storage_pdata,
	},
};
#endif

struct android_usb_platform_data android_usb_pdata = {
	.vendor_id  = 0x1004,
	.product_id = 0x618E,
	.version    = 0x0100,
	.product_name       = "LGE USB Device",
	.manufacturer_name  = "LG Electronics Inc.",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_lge_all),
	.functions = usb_functions_lge_all,
	.serial_number = "LG_ANDROID_GT540_ICS_",
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);


#endif /* CONFIG_USB_G_ANDROID */

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_uart_dm1,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_device_tssc,
	&msm_kgsl_3d0,
};

extern struct sys_timer msm_timer;

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

/* decrease FB pmem size because thunderg uses hvga
 * qualcomm's original value depends on wvga resolution
 * 2010-04-18, cleaneye.kim@lge.com
 */
//unsigned pmem_fb_size = 	0x96000;
//unsigned pmem_adsp_size =	0xAE4000;

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7627_proccomm_regulator_data
	}
};

static void __init msm7627_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}


static void __init msm7x2x_init(void)
{
	if (socinfo_init() < 0)
		BUG();

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1);
#endif

	msm7627_init_regulators();
	msm_clock_init(&msm7x27_clock_init_data);
	acpuclk_init(&acpuclk_7x27_soc_data);

	msm_add_pmem_devices();
	msm_add_fb_device();
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (lge_get_uart_mode())
		platform_device_register(&msm_device_uart3);
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_ARCH_MSM7X27
	msm_add_kgsl_device();
#endif
	msm_add_usb_devices();

#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	msm_device_i2c_init();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	msm_pm_set_platform_data(msm7x27_pm_data, ARRAY_SIZE(msm7x27_pm_data));

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	lge_add_ramconsole_devices();
	lge_add_ers_devices();
	lge_add_panic_handler_devices();
#endif
	lge_add_camera_devices();
	lge_add_lcd_devices();
	lge_add_btpower_devices();
	lge_add_mmc_devices();
	lge_add_wlan_devices();
	lge_add_input_devices();
	lge_add_misc_devices();
	lge_add_pm_devices();

	/* gpio i2c devices should be registered at latest point */
	lge_add_gpio_i2c_devices();
//	swift_init_timed_vibrator();
}

static void __init msm7x27_init_early(void)
{
	msm_msm7x2x_allocate_memory_regions();
}

static void __init msm7x2x_map_io(void)
{
	msm_map_common_io();

#ifdef CONFIG_CACHE_L2X0
	/* 7x27 has 256KB L2 cache:
		64Kb/Way and 4-Way Associativity;
		R/W latency: 3 cycles;
		evmon/parity/share disabled. */
	l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

MACHINE_START(MSM7X27_SWIFT, "MSM7x27 SWIFT")
#ifdef CONFIG_MSM_DEBUG_UART
  .phys_io        = MSM_DEBUG_UART_PHYS,
  .io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
  .boot_params	  = PLAT_PHYS_OFFSET + 0x100,
  .map_io         = msm7x2x_map_io,
  .reserve	  = msm7x27_reserve,
  .init_irq       = msm7x2x_init_irq,
  .init_machine   = msm7x2x_init,
  .timer          = &msm_timer,
  .init_early     = msm7x27_init_early,
  .handle_irq     = vic_handle_irq,
MACHINE_END
