/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, <wd@denx.de>
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

#if !defined (__U_BOOT__)
#include <stdio.h>
#include <stdint.h>
#else
#include <common.h>
#include <command.h>
#endif
#include <asm/inca-ip.h>
#include <asm/mipsregs.h>

#ifdef CONFIG_OCTEON
#include "octeon_boot.h"
#endif

#if defined(__U_BOOT__)
int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
#if defined(CONFIG_INCA_IP)
	*INCA_IP_WDT_RST_REQ = 0x3f;
#elif defined(CONFIG_PURPLE) || defined(CONFIG_TB0229)
	void (*f)(void) = (void *) 0xbfc00000;

	f();
#elif defined(CONFIG_OCTEON)

        CVMX_SYNC;
        udelay(10000);
#ifdef OCTEON_CF_RESET_GPIO
        octeon_gpio_cfg_output(OCTEON_CF_RESET_GPIO);
        octeon_gpio_clr(OCTEON_CF_RESET_GPIO);
        CVMX_SYNC;
        udelay(1);
        octeon_gpio_set(OCTEON_CF_RESET_GPIO);
        CVMX_SYNC;
        udelay(5000);
#endif

    cvmx_write_csr(CVMX_CIU_SOFT_RST,1ull);
#endif
	fprintf(stderr, "*** reset failed ***\n");
	return 0;
}

void flush_cache (unsigned long start_addr, unsigned long size)
{
    CVMX_SYNC;
    CVMX_ICACHE_INVALIDATE;
    CVMX_DCACHE_INVALIDATE;
}
#endif

void write_one_tlb( int index, uint32_t pagemask, uint32_t hi, uint32_t low0, uint32_t low1 ){
	write_32bit_cp0_register(CP0_ENTRYLO0, low0);
	write_32bit_cp0_register(CP0_PAGEMASK, pagemask);
	write_32bit_cp0_register(CP0_ENTRYLO1, low1);
	write_32bit_cp0_register(CP0_ENTRYHI, hi);
	write_32bit_cp0_register(CP0_INDEX, index);
	tlb_write_indexed();
}

void write_one_tlb64(uint32_t index, uint32_t pagemask, uint64_t hi, uint64_t low0, uint64_t low1)
{
	write_64bit_cp0_register(CP0_ENTRYLO0, low0);
	write_32bit_cp0_register(CP0_PAGEMASK, pagemask);
	write_64bit_cp0_register(CP0_ENTRYLO1, low1);
	write_64bit_cp0_register(CP0_ENTRYHI, hi);
	write_32bit_cp0_register(CP0_INDEX, index);
	tlb_write_indexed();
}

int get_num_tlb_entries(void)
{
    int val;
    int num_entries = (read_32bit_cp0_register2(CP0_CONFIG, 1) >> 25) & 0x3f;

    val = read_32bit_cp0_register2(CP0_CONFIG, 3);
    if (val & (1<<31))
    {
        val = read_32bit_cp0_register2(CP0_CONFIG, 4) & 0xff;
        num_entries |= val << 6;
    }
    return(num_entries + 1);
}
