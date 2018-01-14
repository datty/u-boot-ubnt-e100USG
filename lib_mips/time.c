/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include "cvmx.h"


static inline void mips_compare_set(u32 v)
{
	asm volatile ("mtc0 %0, $11" : : "r" (v));
}

static inline void mips_count_set(u32 v)
{
	asm volatile ("mtc0 %0, $9" : : "r" (v));
}


static inline uint64_t mips_count_get(void)
{

#if CONFIG_OCTEON
uint64_t octeon_get_cycles(void);
        return(octeon_get_cycles());
#else
	u32 count;
	asm volatile ("mfc0 %0, $9" : "=r" (count) :);
	return count;
#endif
}

/*
 * timer without interrupts
 */

int timer_init(void)
{
	mips_compare_set(0);
	mips_count_set(0);

	return 0;
}

void reset_timer(void)
{
	mips_count_set(0);
}

/*  This needs to count in units of CFG_HZ, _not_ raw cpu frequency */
uint64_t get_timer(uint64_t base)
{
        DECLARE_GLOBAL_DATA_PTR;
	return ((mips_count_get()/(gd->cpu_clock_mhz * CFG_HZ)) - base);
}

void set_timer(ulong t)
{
	mips_count_set(t);
}

void udelay (unsigned long usec)
{
        DECLARE_GLOBAL_DATA_PTR;
	uint64_t tmo;

	tmo = (uint64_t)usec * (gd->cpu_clock_mhz);
        cvmx_wait(tmo);
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On MIPS it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return mips_count_get();
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On MIPS it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return CFG_HZ;
}
