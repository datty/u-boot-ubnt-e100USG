/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 * Copyright 2004,2005 Cavium Networks
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

/*
 * This file contains the configuration parameters for
 * an example generic NAND stage2 u-boot configuration.
 */

#ifndef __CONFIG_H
#define __CONFIG_H
#include "octeon_common.h"


#define CONFIG_MIPS32		1  /* MIPS32 CPU core	*/
#define CONFIG_OCTEON		1
/* CONFIG_OCTEON_EBH5200 set by Makefile in include/config.h */ 

#if !defined(__U_BOOT_HOST__) && !defined(__BYTE_ORDER)
/* Set endian macros for simple exec includes, but not for host utilities */
#define __BYTE_ORDER __BIG_ENDIAN
#endif

/********************************************************************/
/* Here are the parameters that may need to be changed to customize
 * this configuration to a specific board.
 */


/* Default DDR clock if tuple doesn't exist in EEPROM */
#define DEFAULT_DDR2_CLOCK_FREQ_MHZ  266
#define DEFAULT_DDR3_CLOCK_FREQ_MHZ  533

/* Define the board type that will be used by stage 2 */
#define NAND_STAGE2_BOARD_TYPE      CVMX_BOARD_TYPE_EBB6300

/* Use the ebh5200 DRAM configuration, since the ebt5200 board
** has a matching configuration */
#define CONFIG_OCTEON_EBT5200   1 /* Define this to get DRAM config info compiled in */
#define CONFIG_OCTEON_EBB6300   1 /* Define this to get DRAM config info compiled in */
#define BOARD_MCU_TWSI_ADDR     0x60   /* Need to get the MCU address for pass 1.0 PLL MCU workaround */


/* Change the NAND MTD partitioning here.  The JFFS2 filesystem should be kept as small as reasonably possible
** to minimize boot time.  This environment variable is suitabel for passing directly to the Linux kernel.
** The 'mtdids' environment variable is used in conjunction with the 'mtdparts' variable to define the
** MTD partitions for u-boot.
*/
#define MTDPARTS    "mtdparts=mtdparts=octeon_nand0:4m(nand-reserved),8m(jffs2)\0"

/* End of parameters that typically need to be changed */
/********************************************************************/


/* CN52XX requires a fixed 50 MHZ CPU and DDR reference clocks, so these
** should not need to be changed */
#define DEFAULT_CPU_REF_FREQUENCY_MHZ		 50
#define DEFAULT_DDR_REF_FREQUENCY_MHZ		 50

/* Define the mtdids environment variable here.  This is used to map u-boot NAND device names
** to Linux NAND devide names.  This can contain multiple mappings, of the form:
** <u-boot_name>=<linux_name>,<u-boot_name>=<linux_name>,...
*/
#define MTDIDS      "mtdids=nand0=octeon_nand0\0"
/********************************************************************/

/* Used to control conditional compilation for shared code between simple
** exec and u-boot */
#define CONFIG_OCTEON_U_BOOT


/* let the eth address be writeable */
#define CONFIG_ENV_OVERWRITE 1


/* Set the TWSI clock to a conservative 100KHz. */
#define BOARD_TWSI_BUS_FREQ 	(100000)



#define CONFIG_OCTEON_PCI_HOST 0
/* Set bootdelay to 0 for immediate boot */
#define CONFIG_BOOTDELAY	0	/* autoboot after X seconds	*/

#define CONFIG_BAUDRATE			115200
#define CONFIG_DOWNLOAD_BAUDRATE	115200

/* valid baudrates */
#define CFG_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200, 230400, 460800 }

#define	CONFIG_TIMESTAMP		/* Print image info with timestamp */
#undef	CONFIG_BOOTARGS




/* Define this to enable built-in octeon ethernet support */
#define OCTEON_RGMII_ENET
#define OCTEON_MGMT_ENET

/* Enable Octeon built-in networking if RGMII support is enabled */
#if defined(OCTEON_RGMII_ENET)
#define OCTEON_INTERNAL_ENET
#endif


/*-----------------------------------------------------------------------
 * U-boot Commands Configuration
 */
/* Srecord loading seems to be busted - checking for ctrl-c eats bytes */
#define CONFIG_COMMANDS		((CONFIG_CMD_DFL | CFG_CMD_ELF | CFG_CMD_OCTEON | CFG_CMD_LOADB \
                                  | CFG_CMD_ENV | CFG_CMD_RUN \
                                  |  CFG_CMD_NAND | CFG_CMD_OCTEON_BOOTLOADER_UPDATE\
                                  | CFG_CMD_ASKENV | PCI_CONFIG_COMMANDS | CFG_CMD_NAND | CFG_CMD_JFFS2 | CFG_CMD_OCTEON_NAND)\
                                  & ~(CFG_CMD_FPGA | CFG_CMD_BDI \
                                   | CFG_CMD_BEDBUG | CFG_CMD_BOOTD | CFG_CMD_LOADS | CFG_CMD_NFS | CFG_CMD_FLASH \
                                   | CFG_CMD_FLASH | CFG_CMD_NET | CFG_CMD_DHCP | CFG_CMD_PING | CFG_CMD_MII))

#include <cmd_confdefs.h>



#define CFG_NO_FLASH

/*-----------------------------------------------------------------------
 * Networking Configuration
 */
#define CONFIG_MII

#if (CONFIG_COMMANDS & CFG_CMD_NET)
#define CONFIG_NET_MULTI

#define CONFIG_BOOTP_DEFAULT		(CONFIG_BOOTP_SUBNETMASK | \
					CONFIG_BOOTP_GATEWAY	 | \
					CONFIG_BOOTP_HOSTNAME	 | \
					CONFIG_BOOTP_BOOTPATH)

#define CONFIG_BOOTP_MASK		CONFIG_BOOTP_DEFAULT
#endif /* CONFIG_COMMANDS & CFG_CMD_NET */


#define CONFIG_NET_RETRY_COUNT 5

#define CONFIG_AUTO_COMPLETE 1
#define CFG_CONSOLE_INFO_QUIET 1
#define CFG_DEVICE_NULLDEV
#define CFG_CONSOLE_IS_IN_ENV


#define CFG_64BIT_VSPRINTF  1
#define CFG_64BIT_STRTOUL   1
/*
 * Miscellaneous configurable options
 */

#define	CFG_CBSIZE		256		/* Console I/O Buffer Size   */
#define	CFG_PBSIZE (CFG_CBSIZE + MAX_PROMPT_LEN + 16)  /* Print Buffer Size */
#define	CFG_MAXARGS		64		/* max number of command args*/

#define CFG_MALLOC_LEN		4*1024*1024   /* Needs to be large for JFFS2 */

#define CFG_BOOTPARAMS_LEN	16*1024

#define CFG_HZ			1000ull

#define CFG_SDRAM_BASE		0x80000000     /* Cached addr */


#define CFG_MEMTEST_START	(CFG_SDRAM_BASE + 0x100000)
#define CFG_MEMTEST_END		(CFG_SDRAM_BASE + 0xffffff)



#define	CONFIG_EXTRA_ENV_SETTINGS					\
        "autoload=n\0"					\
        MTDPARTS \
        MTDIDS \
        ""

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */

/* The following #defines are needed to get flash environment right */
#define	CFG_MONITOR_BASE	TEXT_BASE
#define	CFG_MONITOR_LEN		(192 << 10)

#define CFG_INIT_SP_OFFSET	0x400000



/* timeout values are in ticks */
#define CFG_FLASH_ERASE_TOUT	(2 * CFG_HZ) /* Timeout for Flash Erase */
#define CFG_FLASH_WRITE_TOUT	(2 * CFG_HZ) /* Timeout for Flash Write */

/* Address and size of Primary Environment Sector	*/
#define CFG_ENV_SIZE		(8*1024)
#define CFG_ENV_ADDR		(CFG_FLASH_BASE + CFG_FLASH_SIZE - CFG_ENV_SIZE)

#define CFG_ENV_IS_IN_OCTEON_NAND 1

#define CONFIG_MEMSIZE_IN_BYTES

#define CFG_MAX_NAND_DEVICE 8
#define CONFIG_SYS_NAND_BASE 0   /* Not used for Octeon, but must be set */
#define NAND_MAX_CHIPS CFG_MAX_NAND_DEVICE

/*-----------------------------------------------------------------------
 * Cache Configuration
 */
#define CFG_DCACHE_SIZE		(16 * 1024)
#define CFG_ICACHE_SIZE		(32 * 1024)
#define CFG_CACHELINE_SIZE	128

#endif	/* __CONFIG_H */
