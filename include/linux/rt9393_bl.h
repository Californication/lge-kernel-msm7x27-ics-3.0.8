/* include/linux/rt9393_bl.h
 *
 * Copyright (C) 2009-2011 LGE Inc.
 *               2012 Henning Heinold
 * Author : MoonCheol Kang <knight0708@lge.com>
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

#ifndef __INCLUDE_RT9393_BL_H
#define __INCLUDE_RT9393_BL_H

#include <linux/backlight.h>

struct rt9393_platform_data {
        int gpio;
	void (*led_off)(void);
};

#endif /* __INCLUDE_RT9393_BL_H */
