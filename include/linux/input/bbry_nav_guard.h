/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_INPUT_BBRY_NAV_GUARD_H
#define _LINUX_INPUT_BBRY_NAV_GUARD_H

#include <linux/types.h>

#ifdef CONFIG_BBRY_KEYBOARD_NAV_GUARD
bool bbry_nav_guard_active(void);
#else
static inline bool bbry_nav_guard_active(void)
{
	return false;
}
#endif

#endif
