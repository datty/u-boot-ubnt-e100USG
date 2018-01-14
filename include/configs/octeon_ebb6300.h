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
 * Octeon EBB6300 board.
 */

#ifndef __CONFIG_H
#define __CONFIG_H
#include "octeon_common.h"

#define CONFIG_MIPS32		1  /* MIPS32 CPU core	*/
#define CONFIG_OCTEON		1
/* CONFIG_OCTEON_EBB6300 set by Makefile in include/config.h */ 

#if !defined(__U_BOOT_HOST__) && !defined(__BYTE_ORDER)
/* Set endian macros for simple exec includes, but not for host utilities */
#define __BYTE_ORDER __BIG_ENDIAN
#endif

/* Defaults to use if bootloader cannot autodetect settings */
#define DEFAULT_ECLK_FREQ_MHZ			400  /* Used if multiplier read fails, and for DRAM refresh rates*/
#define DEFAULT_CPU_REF_FREQUENCY_MHZ		 50  /* Used if cpu reference clock reading fails. */

/* Default DDR clock if tuple doesn't exist in EEPROM */
#define EBB6300_DEF_DRAM_FREQ  533

#if 0
#define CONFIG_OCTEON_LLM_WORKAROUND
#define CONFIG_OCTEON_PCIX_WORKAROUND
#endif

/* Used to control conditional compilation for shared code between simple
** exec and u-boot */
#define CONFIG_OCTEON_U_BOOT


/* EBB6300 uses configuration eeprom */
#define CONFIG_OCTEON_EEPROM_TUPLES     1

/* let the eth address be writeable */
#define CONFIG_ENV_OVERWRITE 1


/* Configuration to cause u-boot to skip DDR init, and
** run completely from L2.  Memory size is limited to allow this. 
** NOTE: This config is for bringup/debug purposes, and is likely to be
** removed in the near future. */
#define CONFIG_L2_ONLY    0

#if CONFIG_L2_ONLY
#define CONFIG_NO_CLEAR_DDR 1
#endif

/* Addresses for various things on boot bus.  These addresses
** are used to configure the boot bus mappings. */
#define OCTEON_CHAR_LED_CHIP_SEL    4
#define OCTEON_CHAR_LED_BASE_ADDR   0x1d020000
#define OCTEON_PAL_CHIP_SEL         7
#define OCTEON_PAL_BASE_ADDR        0x1d030000
/* Define OCTEON_CF_16_BIT_BUS if board uses 16 bit CF interface */
#define OCTEON_CF_16_BIT_BUS


/* Octeon true IDE mode compact flash support.  True IDE mode is
** always 16 bit. */
#define OCTEON_CF_TRUE_IDE_CS0_CHIP_SEL     5
#define OCTEON_CF_TRUE_IDE_CS0_ADDR         0x1d040000
#define OCTEON_CF_TRUE_IDE_CS1_CHIP_SEL     6
#define OCTEON_CF_TRUE_IDE_CS1_ADDR         0x1d050000
#define OCTEON_CF_RESET_GPIO                5   /* Reset is GPIO 5 */


/* Set the TWSI clock to a conservative 100KHz. */
#define TWSI_BUS_FREQ 	(100000)

/* Remap the Octeon TWSI Slave Address. Default is 0x77. */
/* #define TWSI_BUS_SLAVE_ADDRESS 0x65 */

/* TWSI address for the ispPAC-POWR1220AT8 Voltage Controller chip */
#define BOARD_ISP_TWSI_ADDR 0x08

#define BOARD_MCU_AVAILABLE	1
#define BOARD_MCU_TWSI_ADDR           0x60 
#define BOARD_EEPROM_TWSI_ADDR        OCTEON_EBB6300_BOARD_EEPROM_TWSI_ADDR


#define CFG_64BIT_VSPRINTF  1
#define CFG_64BIT_STRTOUL   1



/* Set bootdelay to 0 for immediate boot */
#define CONFIG_BOOTDELAY	0	/* autoboot after X seconds	*/

#define CONFIG_BAUDRATE			115200
#define CONFIG_DOWNLOAD_BAUDRATE	115200

/* valid baudrates */
#define CFG_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200, 230400, 460800 }

#define	CONFIG_TIMESTAMP		/* Print image info with timestamp */
#undef	CONFIG_BOOTARGS


/*
** Define CONFIG_OCTEON_PCI_HOST = 1 to map the pci devices on the
** bus.  Define CONFIG_OCTEON_PCI_HOST = 0 for target mode when the
** host system performs the pci bus mapping instead.  Note that pci
** commands are enabled to allow access to configuration space for
** both modes.
*/
#ifndef CONFIG_OCTEON_PCI_HOST
#define CONFIG_OCTEON_PCI_HOST	0
#endif

/*
** Define CONFIG_PCI only if the system is known to provide a PCI
** clock to Octeon.  A bus error exception will occur if the inactive
** Octeon PCI core is accessed.  U-boot is not currently configured to
** recover when a exception occurs.
*/
#if CONFIG_OCTEON_PCI_HOST 
#define CONFIG_PCI
#endif
/*-----------------------------------------------------------------------
 * PCI Configuration
 */
#if defined(CONFIG_PCI)

#define PCI_CONFIG_COMMANDS (CFG_CMD_PCI)


#if (CONFIG_OCTEON_PCI_HOST)
#define CONFIG_PCI_PNP
#endif /* CONFIG_OCTEON_PCI_HOST */
#else  /* CONFIG_PCI */
#define PCI_CONFIG_COMMANDS (0)
#endif /* CONFIG_PCI */

/*
** The EBB6300 does not use the internal arbiter in Octeon.
** Enable this for boards that do.
*/
/* #define USE_OCTEON_INTERNAL_ARBITER */

/* Define this to enable built-in octeon ethernet support */
#define OCTEON_RGMII_ENET
#define OCTEON_MGMT_ENET

/* Enable Octeon built-in networking if RGMII support is enabled */
#if defined(OCTEON_RGMII_ENET)
#define OCTEON_INTERNAL_ENET
#endif

/* Disable networking for failsafe builds.  Failsafe builds
** have size constraint of 192K that must be satisfied. */
#ifdef CONFIG_OCTEON_FAILSAFE
#undef OCTEON_RGMII_ENET
#undef OCTEON_SPI4000_ENET
#undef OCTEON_INTERNAL_ENET
#undef OCTEON_MGMT_ENET
#define FAILSAFE_EXCLUDE_COMMANDS   OCTEON_FAILSAFE_EXCLUDE_COMMANDS
#define FAILSAFE_INCLUDE_ENVIRONMENT "ddr_verbose=yes\0"
#else
#define FAILSAFE_EXCLUDE_COMMANDS   (0)
#define FAILSAFE_INCLUDE_ENVIRONMENT
#define CFG_PCI_CONSOLE

#endif  /* CONFIG_OCTEON_FAILSAFE */


/* NAND is pretty large, and pushes the u-boot size beyond the 512k that is addressable
** through the MCU on evaluation boards.  This makes it easy to disable NAND support */
#ifndef CONFIG_OCTEON_NAND_COMMANDS
#define CONFIG_OCTEON_NAND_COMMANDS (CFG_CMD_NAND | CFG_CMD_JFFS2 | CFG_CMD_OCTEON_NAND)
#endif

/*-----------------------------------------------------------------------
 * U-boot Commands Configuration
 */
/* Srecord loading seems to be busted - checking for ctrl-c eats bytes */
#define CONFIG_COMMANDS		((CONFIG_CMD_DFL | CFG_CMD_ELF | CFG_CMD_OCTEON | CFG_CMD_LOADB | CFG_CMD_FLASH \
                                  | CFG_CMD_ENV | CFG_CMD_FLASH | CFG_CMD_IDE | CFG_CMD_FAT | CFG_CMD_EXT2 | CFG_CMD_RUN \
                                  | CFG_CMD_EEPROM | CFG_CMD_I2C | CFG_CMD_NET | CFG_CMD_DHCP | CFG_CMD_PING | CFG_CMD_MII \
                                  | CFG_CMD_ASKENV | PCI_CONFIG_COMMANDS | CFG_CMD_OCTEON_BOOTLOADER_UPDATE \
                                  | CONFIG_OCTEON_NAND_COMMANDS)\
                                  & ~(FAILSAFE_EXCLUDE_COMMANDS | CFG_CMD_FPGA | CFG_CMD_BDI \
                                   | CFG_CMD_BEDBUG | CFG_CMD_BOOTD | CFG_CMD_LOADS | CFG_CMD_NFS))

#include <cmd_confdefs.h>


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

#define CONFIG_DOS_PARTITION 1

#define CFG_IDE_MAXBUS 1
#define CFG_IDE_MAXDEVICE 2

#ifdef CFG_CMD_EEPROM
#define CFG_I2C_EEPROM_ADDR_LEN 2
#define CFG_I2C_EEPROM_ADDR     BOARD_EEPROM_TWSI_ADDR
#endif

/* Base address of Common memory for Compact flash */
#define CFG_ATA_BASE_ADDR  OCTEON_CF_COMMON_BASE_ADDR

/* Offset from base at which data is repeated so that it can be
** read as a block */
#define CFG_ATA_DATA_OFFSET     0x400

/* Not sure what this does, probably offset from base
** of the command registers */
#define CFG_ATA_REG_OFFSET      0

/*
 * Miscellaneous configurable options
 */
#define	CFG_LONGHELP			/* undef to save memory      */
#define	CFG_CBSIZE		256		/* Console I/O Buffer Size   */
#define	CFG_PBSIZE (CFG_CBSIZE+sizeof(MAX_PROMPT_LEN)+16)  /* Print Buffer Size */
#define	CFG_MAXARGS		64		/* max number of command args*/

#define CFG_MALLOC_LEN		64*1024

#define CFG_BOOTPARAMS_LEN	16*1024

#define CFG_HZ			1000ull

#define CFG_SDRAM_BASE		0x80000000     /* Cached addr */


#define CFG_MEMTEST_START	(CFG_SDRAM_BASE + 0x100000)
#define CFG_MEMTEST_END		(CFG_SDRAM_BASE + 0xffffff)


#if CONFIG_L2_ONLY
#define L2_ONLY_ENV "octeon_reserved_mem_load_size=0xc0000\0""octeon_reserved_mem_linux_size=0\0"
#else
#define L2_ONLY_ENV
#endif

#define	CONFIG_EXTRA_ENV_SETTINGS					\
        "burn_app=erase $(flash_unused_addr) +$(filesize);cp.b $(fileaddr) $(flash_unused_addr) $(filesize)\0"				\
        "bf=bootoct $(flash_unused_addr) forceboot numcores=$(numcores)\0"				\
        "nuke_env=protect off $(env_addr) +$(env_size); erase $(env_addr) +$(env_size)\0"				\
        "linux_cf=fatload ide 0 $(loadaddr) vmlinux.64;bootoctlinux $(loadaddr)\0"				\
        "ls=fatls ide 0\0"				\
        "autoload=n\0"					\
        L2_ONLY_ENV		\
        FAILSAFE_INCLUDE_ENVIRONMENT		\
        ""

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CFG_FLASH_SIZE	        (8*1024*1024)	/* Flash size (bytes) */
#define CFG_MAX_FLASH_BANKS	1	/* max number of memory banks */
#define CFG_MAX_FLASH_SECT	(512)	/* max number of sectors on one chip */

#define CFG_FLASH_BASE		(0x1fc00000 -  CFG_FLASH_SIZE) /* Remapped base of flash */


/* The following #defines are needed to get flash environment right */
#define	CFG_MONITOR_BASE	TEXT_BASE
#define	CFG_MONITOR_LEN		(192 << 10)

#define CFG_INIT_SP_OFFSET	0x400000


#define CFG_FLASH_CFI   1
#define CFG_FLASH_CFI_DRIVER   1

/* timeout values are in ticks */
#define CFG_FLASH_ERASE_TOUT	(2 * CFG_HZ) /* Timeout for Flash Erase */
#define CFG_FLASH_WRITE_TOUT	(2 * CFG_HZ) /* Timeout for Flash Write */

#if CONFIG_OCTEON_FAILSAFE || CONFIG_RAM_RESIDENT
#define	CFG_ENV_IS_NOWHERE	1
#else
#define	CFG_ENV_IS_IN_FLASH	1
#endif

/* Address and size of Primary Environment Sector	*/
#define CFG_ENV_SIZE		(8*1024)
#define CFG_ENV_ADDR		(CFG_FLASH_BASE + CFG_FLASH_SIZE - CFG_ENV_SIZE)



#define CONFIG_NR_DRAM_BANKS	2

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

/* Include shared board configuration, consisting mainly of DRAM details. */
#include "octeon_ebb6300_shared.h"


#define OCTEON_CONFIG_DFM (1)

/* MT41J128M8JP-15E:F  16 Meg x 8 x 8 banks  */
#define DFM_CFG_NUM_BANKS          8
#define DFM_CFG_DRAM_WIDTH         8
#define DFM_CFG_ROW_BITS          14
#define DFM_CFG_COL_BITS          10
#define DFM_CFG_NUM_RANKS          2
#define DFM_CFG_TCKMIN	        1500
#define DFM_CFG_TAAMIN	       13125
#define DFM_CFG_TWR	       15000 /* ps, Write Recovery Time (tWR)                     */
#define DFM_CFG_TRCD	       13500 /* ps, Minimum RAS to CAS delay (tRCD)               */
#define DFM_CFG_TRRD	        6000 /* ps, Minimum Row Active to Row Active delay (tRRD) */
#define DFM_CFG_TRP	       13500 /* ps, Minimum Row Precharge Delay (tRP)             */
#define DFM_CFG_TRAS	       36000 /* ps, Minimum Active to Precharge (tRAS)            */
#define DFM_CFG_TRC	       49500 /* ps, Minimum Active to Active/Refresh Delay (tRC)  */
#define DFM_CFG_TRFC	      110000 /* ps, Minimum Refresh Recovery Delay (tRFC)         */
#define DFM_CFG_TWTR	        7500 /* ps, Internal write to read command delay (tWTR)   */
#define DFM_CFG_TRTP	        7500 /* ps, Min Internal Rd to Precharge Cmd Delay (tRTP) */
#define DFM_CFG_TFAW	       30000 /* ps, Minimum Four Activate Window Delay (tFAW)     */
#define DFM_CFG_ADDR_MIRROR        1
#define DFM_EARLY_DQX		   0 /* Set for the rare case when ADD/CMD signals are
                                        routed shorter than the shortest DQS/DQ. */
#define DFM_CFG_CAS_LATENCIES ((1<<(11-4)) | (1<<(10-4)) | (1<<(9-4)) | (1<<(8-4)) | (1<<(7-4)) | (1<<(6-4)) | (1<<(5-4)))

#define OCTEON_EBB6300_CN63XX_DFM_ODT_CONFIGURATION \
    /* RANKS   reserved  WODT_MASK                LMCX_MODEREG_PARAMS1            RODT_CTL    RODT_MASK    reserved */ \
    /* =====   ======== ============== ========================================== ========= ============== ======== */ \
    /*   1 */ {   0,    0x00000001ULL, OCTEON_EBB6300_MODEREG_PARAMS1_1RANK_1SLOT,    5,     0x00000000ULL,  0  },     \
    /*   2 */ {   0,    0x00000101ULL, OCTEON_EBB6300_MODEREG_PARAMS1_2RANK_1SLOT,    5,     0x00000000ULL,  0  }

#define OCTEON_CN63XX_DFM_ODT_CONFIGURATION OCTEON_EBB6300_CN63XX_DFM_ODT_CONFIGURATION

#endif	/* __CONFIG_H */
