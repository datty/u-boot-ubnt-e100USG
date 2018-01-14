/*
 * (C) Copyright 2002-2003
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

#ifndef	__ASM_GBL_DATA_H
#define __ASM_GBL_DATA_H

#if !defined(__U_BOOT__)
#define k0 $26
#include "asm/u-boot.h"
#else
#include <asm/regdef.h>
#endif

#include <octeon_eeprom_types.h>
/*
 * The following data structure is placed in some memory wich is
 * available very early after boot (like DPRAM on MPC8xx/MPC82xx, or
 * some locked parts of the data cache) to allow for a minimum set of
 * global variables during system initialization (until we have set
 * up the memory controller so that we can use RAM).
 *
 * Keep it *SMALL* and remember to set CFG_GBL_DATA_SIZE > sizeof(gd_t)
 */
#define SERIAL_LEN  20
#define GD_TMP_STR_SIZE  30
typedef	struct	global_data {
    bd_t		*bd;
    unsigned long	flags;
    unsigned long	baudrate;
    unsigned long	have_console;	/* serial_init() was called */
    uint64_t    	ram_size;	/* RAM size */
    uint64_t    	reloc_off;	/* Relocation Offset */
    unsigned long	env_addr;	/* Address  of Environment struct */
    unsigned long	env_valid;	/* Checksum of Environment valid? */
    unsigned long       cpu_clock_mhz;  /* CPU clock speed in Mhz */
    unsigned long       ddr_clock_mhz;  /* DDR clock (not data rate!) in Mhz */
    unsigned long       ddr_ref_hertz;   /* DDR Ref clock Hertz */
    int                 mcu_rev_maj;
    int                 mcu_rev_min;
    int                 console_uart;

    /* EEPROM data structures as read from EEPROM or populated by other
    ** means on boards without an EEPROM */
    octeon_eeprom_board_desc_t board_desc;
    octeon_eeprom_clock_desc_t clock_desc;
    octeon_eeprom_mac_addr_t   mac_desc;

    void		**jt;		/* jump table */
    char                *err_msg;   /* pointer to error message to save until console is up */
    char                tmp_str[GD_TMP_STR_SIZE];  /* Temporary string used in ddr init code before DRAM is up */
    unsigned long	uboot_flash_address;	/* Address of normal bootloader in flash */
    unsigned long 	uboot_flash_size;	/* Size of normal bootloader */
    uint64_t    	dfm_ram_size;	/* DFM RAM size */
} gd_t;


/*
 * Global Data Flags
 */
#define	GD_FLG_RELOC	0x00001		/* Code was relocated to RAM     */
#define	GD_FLG_DEVINIT	0x00002		/* Devices have been initialized */
#define	GD_FLG_SILENT	0x00004		/* Silent mode			 */
#define GD_FLG_CLOCK_DESC_MISSING   0x0008
#define GD_FLG_BOARD_DESC_MISSING   0x0010
#define GD_FLG_DDR_VERBOSE          0x0020
#define GD_FLG_DDR0_CLK_INITIALIZED  0x0040
#define GD_FLG_DDR1_CLK_INITIALIZED  0x0080
#define GD_FLG_DDR2_CLK_INITIALIZED  0x0100
#define GD_FLG_RAM_RESIDENT          0x0200  /* RAM boot detected */
#define GD_FLG_FAILSAFE_MODE         0x0400  /* Use failsafe mode */
#define GD_FLG_DDR_TRACE_INIT        0x0800
#define GD_FLG_DFM_CLK_INITIALIZED   0x1000
#define GD_FLG_DFM_VERBOSE           0x2000
#define GD_FLG_DFM_TRACE_INIT        0x4000
#define GD_FLG_MEMORY_PRESERVED      0x8000

#define DECLARE_GLOBAL_DATA_PTR     register volatile gd_t *gd asm ("k0")

#endif /* __ASM_GBL_DATA_H */
