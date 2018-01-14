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
 * This file contains configuration parameters for Octeon UBNT E100 board.
 */

#ifndef __CONFIG_H
#define __CONFIG_H
#include "octeon_common.h"
#include "octeon_ubnt_e100_shared.h"

#define CONFIG_MIPS32		1  /* MIPS32 CPU core	*/
#define CONFIG_OCTEON		1



#if !defined(__U_BOOT_HOST__) && !defined(__BYTE_ORDER)
/* Set endian macros for simple exec includes, but not for host utilities */
#define __BYTE_ORDER __BIG_ENDIAN
#endif

/* Defaults to use if bootloader cannot autodetect settings */
#define DEFAULT_ECLK_FREQ_MHZ			400  /* Used if multiplier read fails, and for DRAM refresh rates*/



/* Used to control conditional compilation for shared code between simple
** exec and u-boot */
#define CONFIG_OCTEON_U_BOOT


#define CONFIG_OCTEON_EEPROM_TUPLES     0

/* let the eth address be writeable */
#define CONFIG_ENV_OVERWRITE 1

#if 0
#define OCTEON_GPIO_FAILSAFE_BIT 0
#else
#define CFG_NO_GPIO_FAILSAFE
#endif

#define CFG_64BIT_VSPRINTF  1
#define CFG_64BIT_STRTOUL   1


/* Set bootdelay to 0 for immediate boot */
#define CONFIG_BOOTDELAY	0	/* autoboot after X seconds	*/
#define CONFIG_ZERO_BOOTDELAY_CHECK

#define UBNT_EEPROM_OFFSET_KB 1024
#define UBNT_EEPROM_OFFSET _ubnt_eeprom_offset(UBNT_EEPROM_OFFSET_KB)
#define _ubnt_eeprom_offset(kb) __ubnt_eeprom_offset(kb)
#define __ubnt_eeprom_offset(kb) #kb "k"

/* autoboot to USB */
#define _FLASH_PARTS \
    "phys_mapped_flash:512k(boot0),512k(boot1)," \
    "64k@" UBNT_EEPROM_OFFSET "(eeprom)"

#define CONFIG_BOOTCOMMAND     \
    "fatload usb 0 $loadaddr vmlinux.64;" \
    "bootoctlinux $loadaddr coremask=0x3 root=/dev/sda2 rootdelay=15 rw " \
    "rootsqimg=squashfs.img rootsqwdir=w " \
    "mtdparts=" _FLASH_PARTS

#define CONFIG_FACTORY_RESET
#define CONFIG_FACTORY_RESET_GPIO	11
#define CONFIG_FACTORY_RESET_TIME	3
#define CONFIG_FACTORY_RESET_BOOTCMD	CONFIG_BOOTCOMMAND " resetsqimg"

#ifndef __ASSEMBLY__
extern void board_set_led_blink(void);
extern void board_set_led_on(void);
extern void board_set_led_off(void);
extern void board_set_led_normal(void);
#endif

#define CONFIG_BAUDRATE		115200
#define CONFIG_DOWNLOAD_BAUDRATE		115200

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
** Enable internal Octeon arbiter.
*/
#define USE_OCTEON_INTERNAL_ARBITER

/* Define this to enable built-in octeon ethernet support */
#define OCTEON_RGMII_ENET

/* Enable Octeon built-in networking if either SPI or RGMII support is enabled */
#if defined(OCTEON_RGMII_ENET) || defined(OCTEON_SPI4000_ENET)
#define OCTEON_INTERNAL_ENET
#endif



#if 1  /* Disable these defines to disable USB support in u-boot */
#define USB_COMMAND CFG_CMD_USB
#define CONFIG_USB_STORAGE
#define CONFIG_USB_OCTEON
#else
#define USB_COMMAND 0
#endif

/*-----------------------------------------------------------------------
 * U-boot Commands Configuration
 */
/* Srecord loading seems to be busted - checking for ctrl-c eats bytes */
#define CONFIG_COMMANDS ((CONFIG_CMD_DFL \
                          | CFG_CMD_ELF \
                          | CFG_CMD_OCTEON \
                          | CFG_CMD_LOADB \
                          | CFG_CMD_FLASH \
                          | CFG_CMD_ENV \
                          | CFG_CMD_FAT \
                          | CFG_CMD_EXT2 \
                          | CFG_CMD_RUN \
                          | CFG_CMD_NET \
                          | CFG_CMD_DHCP \
                          | CFG_CMD_PING \
                          | CFG_CMD_MII \
                          | USB_COMMAND \
                          | CFG_CMD_ASKENV \
                          | PCI_CONFIG_COMMANDS \
                          | CFG_CMD_GUNZIP \
                          | CFG_CMD_OCTEON_BOOTLOADER_UPDATE) \
                         & ~(CFG_CMD_FPGA \
                             | CFG_CMD_BDI \
                             | CFG_CMD_BEDBUG \
                             | CFG_CMD_BOOTD \
                             | CFG_CMD_LOADS \
                             | CFG_CMD_NFS))

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

#define CONFIG_DOS_PARTITION 1

#define CONFIG_LBA48	1


/*
 * Miscellaneous configurable options
 */
#define	CFG_LONGHELP			/* undef to save memory      */
#define	CFG_CBSIZE		256		/* Console I/O Buffer Size   */
#define	CFG_PBSIZE (CFG_CBSIZE + MAX_PROMPT_LEN + 16)  /* Print Buffer Size */
#define	CFG_MAXARGS		64		/* max number of command args*/

#define CFG_MALLOC_LEN		128*1024

#define CFG_BOOTPARAMS_LEN	16*1024

#define CFG_HZ			1000ull

#define CFG_SDRAM_BASE		0x80000000     /* Cached addr */


#define CFG_MEMTEST_START	(CFG_SDRAM_BASE + 0x100000)
#define CFG_MEMTEST_END		(CFG_SDRAM_BASE + 0xffffff)



#define CONFIG_EXTRA_ENV_SETTINGS \
        "nuke_env=protect off $(env_addr) +$(env_size);" \
                    "erase $(env_addr) +$(env_size)\0" \
        "autoload=n\0" \
        ""

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#ifndef CFG_FLASH_SIZE_MB
#define CFG_FLASH_SIZE_MB 8
#endif
#define CFG_FLASH_SIZE	(CFG_FLASH_SIZE_MB * 1024 * 1024)
#define CFG_MAX_FLASH_BANKS	1	/* max number of memory banks */
#ifndef CFG_MAX_FLASH_SECT
#define CFG_MAX_FLASH_SECT	(135)	/* max number of sectors on one chip */
#endif

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

#define	CFG_ENV_IS_IN_FLASH	1

/* Address and size of Primary Environment Sector	*/
#define CFG_ENV_SIZE		(8*1024)
#define CFG_ENV_ADDR		(CFG_FLASH_BASE + CFG_FLASH_SIZE - CFG_ENV_SIZE)

#define CFG_UBNT_EEPROM_ADDR (CFG_FLASH_BASE + (UBNT_EEPROM_OFFSET_KB * 1024))

#define CFG_UBNT_GD_SIZE    CFG_ENV_SIZE
#define CFG_UBNT_GD_MAC_DESC_OFF        0
#define CFG_UBNT_GD_BOARD_DESC_OFF      (CFG_UBNT_GD_MAC_DESC_OFF \
                                         + sizeof(octeon_eeprom_mac_addr_t))
#if CONFIG_OCTEON_UBNT_E120
#define CFG_UBNT_EEPROM_SIZE (4 * 1024)
#define CFG_UBNT_GD_ADDR    (CFG_UBNT_EEPROM_ADDR + CFG_UBNT_EEPROM_SIZE)
#else
#define CFG_UBNT_EEPROM_SIZE (64 * 1024)
#define CFG_UBNT_GD_ADDR    (CFG_ENV_ADDR - CFG_UBNT_GD_SIZE)
#endif

#define CFG_PRINT_MPR

#define CONFIG_NR_DRAM_BANKS	2

#define CONFIG_MEMSIZE_IN_BYTES

#define CFG_ALT_MEMTEST


/*-----------------------------------------------------------------------
 * Cache Configuration
 */
#define CFG_DCACHE_SIZE		( 8 * 1024)
#define CFG_ICACHE_SIZE		(32 * 1024)
#define CFG_CACHELINE_SIZE	128

#endif	/* __CONFIG_H */
