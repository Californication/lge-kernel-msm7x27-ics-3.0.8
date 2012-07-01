#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/android_pmem.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <asm/setup.h>
#endif

#ifdef CONFIG_LGE_POWER_ON_STATUS_PATCH
#include <mach/msm_smsm.h>
#include <asm/processor.h>
#endif

#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/board_lge.h>

// LGE_CHANGE_S : Diag access lock for Tracfone 2012-04-14
#ifdef LGE_TRACFONE
extern int user_diag_enable;

static char diag_lock_state = 0;
static int diag_set_usb_lock_state(const char *val, struct kernel_param *kp);
static int diag_get_usb_lock_state(char *buffer, struct kernel_param *kp);
module_param_call(diag_lock_state, diag_set_usb_lock_state, diag_get_usb_lock_state,
						&diag_lock_state, 0660);
MODULE_PARM_DESC(diag_lock_state, "Diag lock state");

static int diag_set_usb_lock_state(const char *val, struct kernel_param *kp)
{
	if (!val)
		return 1;

	if (!strncmp(val, "1", 1))
	       user_diag_enable = 0;
	else if (!strncmp(val, "0", 1))
	       user_diag_enable = 1;
	return 0;
}
static int diag_get_usb_lock_state(char *buffer, struct kernel_param *kp)
{
	if (!buffer)
		return 1;

	if (user_diag_enable)
		memcpy(buffer, "0", 1);
	else
		memcpy(buffer, "1", 1);

	return 1;
}

static char diag_lock_key[20] = {0x0, };
static int diag_lock_key_status = 0;
static int set_usb_lock_key(const char *val, struct kernel_param *kp);
static int get_usb_lock_key(char *buffer, struct kernel_param *kp);
module_param_call(diag_lock_key, set_usb_lock_key, get_usb_lock_key,
						&diag_lock_key, 0660);
MODULE_PARM_DESC(diag_lock_key, "Diag lock key");

static int set_usb_lock_key(const char *val, struct kernel_param *kp)
{
	extern int lg_usb_lock_code_is_valid(const char *key, int size);

	if (!val)
		return 1;

	if (strlen(val)!=16)
		return 1;

	if(lg_usb_lock_code_is_valid(val, 16))
		diag_lock_key_status = 1;
	else
		diag_lock_key_status = 0;

	return 0;
}
static int get_usb_lock_key(char *buffer, struct kernel_param *kp)
{
	if (!buffer)
		return 1;

	if (diag_lock_key_status)
		strcpy(buffer, "correct");
	else
		strcpy(buffer, "incorrect");

	diag_lock_key_status = 0;
	return strlen(buffer);
}

int get_usb_lock(void)
{
	extern int lg_get_usb_lock_state(void);

	if (lg_get_usb_lock_state()==1)
		user_diag_enable = 0;
	else
		user_diag_enable = 1;

	return user_diag_enable;
}

void set_usb_lock(int lock)
{
	extern int lg_set_usb_lock_state(int lock);

	lg_set_usb_lock_state(lock);

	if (lock)
		user_diag_enable = 0;
	else
		user_diag_enable = 1;
}
#endif
// LGE_CHANGE_E : Diag access lock for Tracfone 2012-04-14

/* LGE_CHANGE_S
	Read power on status from modem, and update boot reason.
	Ref:- Documentation\arm\msm\boot.txt
*/
#ifdef CONFIG_LGE_POWER_ON_STATUS_PATCH
void __init lge_board_pwr_on_status(void)
{
	unsigned smem_size;
	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	/* filter pwr on status byte */
	boot_reason &= 0xFF;
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
}
#endif /* CONFIG_LGE_POWER_ON_STATUS_PATCH*/

/* setting board revision information */
int lge_bd_rev;

static int __init board_revno_setup(char *rev_info)
{
	char *rev_str[] = { "evb", "rev_a", "rev_b", "rev_c", "rev_d", "rev_e",
		"rev_f", "rev_g", "rev_10", "rev_11", "rev_12" };
	int i;

	lge_bd_rev = LGE_REV_TOT_NUM;

	for (i = 0; i < LGE_REV_TOT_NUM; i++)
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = i;
			break;
		}

	printk(KERN_INFO"BOARD: LGE %s\n", rev_str[lge_bd_rev]);
	return 1;
}

__setup("lge.rev=", board_revno_setup);

/* setting whether uart console is enabled or disabled */
static int uart_console_mode = 0;

int __init lge_get_uart_mode(void)
{
	return uart_console_mode;
}

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 5)) {
		printk(KERN_INFO "UART CONSOLE : enable\n");
		uart_console_mode = 1;
	} else
		printk(KERN_INFO "UART CONSOLE : disable\n");

	return 1;
}

__setup("uart_console=", lge_uart_mode);


static char frst_mode[6];

static int __init lge_frst_mode(char *cmdline)
{
	//strncpy(cmdline, frst_mode, 5);
	strncpy(frst_mode, cmdline, 5);
	frst_mode[5]='\0';
	printk(KERN_INFO "FRST MODE : %s",frst_mode);

	return 1;
}
__setup("lge.frst=", lge_frst_mode);

char* get_frst_mode(void)
{
	return frst_mode;
}
EXPORT_SYMBOL(get_frst_mode);

/* pmem devices */
static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

__WEAK unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

__WEAK unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

__WEAK unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}

early_param("fb_size", fb_size_setup);

/* frame buffer devices */
static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_msm7x27a_surf() || machine_is_msm7x27a_u0()) {	
		if (!strncmp(name, "lcdc_toshiba_fwvga_pt", 21))
			ret = 0;
	} else {
		ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

void __init msm_add_fb_device(void)
{
	platform_device_register(&msm_fb_device);
}

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device *pmem_devices[] __initdata = {
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
};

void __init msm_add_pmem_devices(void)
{
	platform_add_devices(pmem_devices, ARRAY_SIZE(pmem_devices));
}

__WEAK unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

__WEAK unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
}

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
#ifdef __SPLINT__
#else
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
#endif
};

static void __init size_pmem_devices(void)
{
/*LGE_CHANGE_S :
 * newly added code in kernel3.0.8
 */
 	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		pmem_mdp_size = MSM7x25A_MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM7x25A_MSM_PMEM_ADSP_SIZE;
	} else {
		pmem_mdp_size = MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
	}

#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource[] = {
	{
		.name = "ram_console",
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

void __init lge_add_ramconsole_devices(void)
{
	struct resource *res = ram_console_resource;
	struct membank *bank = &meminfo.bank[0];
	res->start = MSM7X27_EBI1_CS0_BASE + bank->size;
	res->end = res->start + LGE_RAM_CONSOLE_SIZE - 1;
	printk(KERN_INFO "RAM CONSOLE START ADDR : 0x%x\n", res->start);
	printk(KERN_INFO "RAM CONSOLE END ADDR   : 0x%x\n", res->end);

	platform_device_register(&ram_console_device);
}
#endif

/* lge gpio i2c device */
#define MAX_GPIO_I2C_DEV_NUM		10
#define LOWEST_GPIO_I2C_BUS_NUM		2

static gpio_i2c_init_func_t *i2c_init_func[MAX_GPIO_I2C_DEV_NUM] __initdata;
static int i2c_dev_num __initdata;

void __init lge_add_gpio_i2c_device(gpio_i2c_init_func_t *init_func)
{
	i2c_init_func[i2c_dev_num] = init_func;
	i2c_dev_num++;
}

void __init lge_add_gpio_i2c_devices(void)
{
	int index;
	gpio_i2c_init_func_t *init_func_ptr;

	for (index = 0; index < i2c_dev_num; index++) {
		init_func_ptr = i2c_init_func[index];
		(*init_func_ptr)(LOWEST_GPIO_I2C_BUS_NUM + index);
	}
}

int __init lge_init_gpio_i2c_pin(struct i2c_gpio_platform_data *i2c_adap_pdata,
		struct gpio_i2c_pin gpio_i2c_pin,
		struct i2c_board_info *i2c_board_info_data)
{
	i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
	i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;

	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(gpio_i2c_pin.sda_pin, 1);
	gpio_set_value(gpio_i2c_pin.scl_pin, 1);

	if (gpio_i2c_pin.reset_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		gpio_set_value(gpio_i2c_pin.reset_pin, 1);
	}

	if (gpio_i2c_pin.irq_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}

	return 0;
}

int lge_init_gpio_i2c_pin_pullup(struct i2c_gpio_platform_data *i2c_adap_pdata,
		struct gpio_i2c_pin gpio_i2c_pin,
		struct i2c_board_info *i2c_board_info_data)
{
	i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
	i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;

	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(gpio_i2c_pin.sda_pin, 1);
	gpio_set_value(gpio_i2c_pin.scl_pin, 1);

	if (gpio_i2c_pin.reset_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		gpio_set_value(gpio_i2c_pin.reset_pin, 1);
	}

	if (gpio_i2c_pin.irq_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}

	return 0;
}
#if defined(CONFIG_ANDROID_RAM_CONSOLE) && defined(CONFIG_LGE_HANDLE_PANIC)
static struct resource crash_log_resource[] = {
	{
		.name = "crash_log",
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device panic_handler_device = {
	.name = "panic-handler",
	.num_resources = ARRAY_SIZE(crash_log_resource),
	.resource = crash_log_resource,
	.dev    = {
		.platform_data = NULL,
	}
};

void __init lge_add_panic_handler_devices(void)
{
	struct resource *res = crash_log_resource;
	struct membank *bank = &meminfo.bank[0];

	res->start = bank->start + bank->size + LGE_RAM_CONSOLE_SIZE;
	res->end = res->start + LGE_CRASH_LOG_SIZE - 1;

	printk(KERN_INFO "CRASH LOG START ADDR : %d\n", res->start);
	printk(KERN_INFO "CRASH LOG END ADDR   : %d\n", res->end);

	platform_device_register(&panic_handler_device);
}
#endif
/* lge common functions to add devices */
__WEAK void __init lge_add_input_devices(void)
{
}

__WEAK void __init lge_add_misc_devices(void)
{
}

__WEAK void __init lge_add_mmc_devices(void)
{
}

__WEAK void __init lge_add_sound_devices(void)
{
}

__WEAK void __init lge_add_lcd_devices(void)
{
}

__WEAK void __init lge_add_camera_devices(void)
{
}

__WEAK void __init lge_add_pm_devices(void)
{
}

__WEAK void __init lge_add_usb_devices(void)
{
}

__WEAK void __init lge_add_connectivity_devices(void)
{
}
__WEAK void __init lge_add_nfc_devices(void)
{
}
/*LGE_CHANGE_E : NFC*/
