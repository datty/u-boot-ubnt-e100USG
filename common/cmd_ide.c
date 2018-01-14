/*
 * (C) Copyright 2000-2002
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
 *
 */

/*
 * IDE support
 */
#include <common.h>
#include <config.h>
#include <watchdog.h>
#include <command.h>
#include <image.h>
#include <asm/byteorder.h>
#include "octeon_boot.h"
#include "octeon_hal.h"
#include "octeon_pci.h"
#include <cvmx-compactflash.h>

#if defined(CONFIG_IDE_8xx_DIRECT) || defined(CONFIG_IDE_PCMCIA)
# include <pcmcia.h>
#endif
#ifdef CONFIG_8xx
# include <mpc8xx.h>
#endif
#ifdef CONFIG_MPC5xxx
#include <mpc5xxx.h>
#endif
#include <ide.h>
#include <ata.h>
#ifdef CONFIG_STATUS_LED
# include <status_led.h>
#endif
#ifndef __PPC__
#include <asm/io.h>
#ifdef __MIPS__
/* Macros depend on this variable */
const unsigned long mips_io_port_base = 0;   
#endif
#endif

#ifdef CONFIG_SHOW_BOOT_PROGRESS
# include <status_led.h>
# define SHOW_BOOT_PROGRESS(arg)	show_boot_progress(arg)
#else
# define SHOW_BOOT_PROGRESS(arg)
#endif

#ifdef __PPC__
# define EIEIO		__asm__ volatile ("eieio")
#else
# define EIEIO		/* nothing */
#endif

#undef	IDE_DEBUG


#ifdef	IDE_DEBUG
#define	PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define PRINTF(fmt,args...)
#endif

#if (CONFIG_COMMANDS & CFG_CMD_IDE)

#ifdef CONFIG_IDE_8xx_DIRECT
/* Timings for IDE Interface
 *
 * SETUP / LENGTH / HOLD - cycles valid for 50 MHz clk
 * 70	   165	    30	   PIO-Mode 0, [ns]
 *  4	     9	     2		       [Cycles]
 * 50	   125	    20	   PIO-Mode 1, [ns]
 *  3	     7	     2		       [Cycles]
 * 30	   100	    15	   PIO-Mode 2, [ns]
 *  2	     6	     1		       [Cycles]
 * 30	    80	    10	   PIO-Mode 3, [ns]
 *  2	     5	     1		       [Cycles]
 * 25	    70	    10	   PIO-Mode 4, [ns]
 *  2	     4	     1		       [Cycles]
 */

const static pio_config_t pio_config_ns [IDE_MAX_PIO_MODE+1] =
{
    /*	Setup  Length  Hold  */
	{ 70,	165,	30 },		/* PIO-Mode 0, [ns]	*/
	{ 50,	125,	20 },		/* PIO-Mode 1, [ns]	*/
	{ 30,	101,	15 },		/* PIO-Mode 2, [ns]	*/
	{ 30,	 80,	10 },		/* PIO-Mode 3, [ns]	*/
	{ 25,	 70,	10 },		/* PIO-Mode 4, [ns]	*/
};

static pio_config_t pio_config_clk [IDE_MAX_PIO_MODE+1];

#ifndef	CFG_PIO_MODE
#define	CFG_PIO_MODE	0		/* use a relaxed default */
#endif
static int pio_mode = CFG_PIO_MODE;

/* Make clock cycles and always round up */

#define PCMCIA_MK_CLKS( t, T ) (( (t) * (T) + 999U ) / 1000U )

#endif /* CONFIG_IDE_8xx_DIRECT */


/* ------------------------------------------------------------------------- */

/* Current I/O Device	*/
static int curr_device = -1;

/* Current offset for IDE0 / IDE1 bus access	*/
ulong ide_bus_offset[CFG_IDE_MAXBUS] = {
#if defined(CFG_ATA_IDE0_OFFSET)
	CFG_ATA_IDE0_OFFSET,
#endif
#if defined(CFG_ATA_IDE1_OFFSET) && (CFG_IDE_MAXBUS > 1)
	CFG_ATA_IDE1_OFFSET,
#endif
};

extern int sil24_init_one(void);
extern void sata_ident (block_dev_desc_t *dev_desc);
extern ulong sata_read (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer);
extern ulong sata_write (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer);
extern void sata_reset (void);

#define	ATA_CURR_BASE(dev)	(CFG_ATA_BASE_ADDR+ide_bus_offset[IDE_BUS(dev)])

#ifndef CONFIG_AMIGAONEG3SE
static int	    ide_bus_ok[CFG_IDE_MAXBUS];
#else
static int	    ide_bus_ok[CFG_IDE_MAXBUS] = {0,};
#endif

static int ide_dev_desc_register_offset;  /* Used to track how many devices have been registered */
static  block_dev_desc_t ide_dev_desc[CFG_IDE_MAXDEVICE];
/* ------------------------------------------------------------------------- */

#ifdef CONFIG_IDE_LED
#if !defined(CONFIG_KUP4K) &&  !defined(CONFIG_KUP4X) &&!defined(CONFIG_BMS2003)
static void  ide_led   (uchar led, uchar status);
#else
extern void  ide_led   (uchar led, uchar status);
#endif
#else
#ifndef CONFIG_AMIGAONEG3SE
#define ide_led(a,b)	/* dummy */
#else
extern void ide_led(uchar led, uchar status);
#define LED_IDE1  1
#define LED_IDE2  2
#define CONFIG_IDE_LED 1
#define DEVICE_LED(x) 1
#endif
#endif

#ifdef CONFIG_IDE_RESET
static void  ide_reset (void);
#else
#define ide_reset()	/* dummy */
#endif

static void  ide_ident (block_dev_desc_t *dev_desc);
static uchar ide_wait  (int dev, ulong t);
static uchar ide_wait_drq  (int dev, ulong t);

#define IDE_TIME_OUT	5000	/* 5 sec timeout */

#define ATAPI_TIME_OUT	7000	/* 7 sec timeout (5 sec seems to work...) */

#define IDE_SPIN_UP_TIME_OUT 5000 /* 5 sec spin-up timeout */

static void __inline__ ide_outb(int dev, int port, unsigned char val);
static unsigned char __inline__ ide_inb(int dev, int port);
static void input_data(int dev, ulong *sect_buf, int words);
static void output_data(int dev, ulong *sect_buf, int words);
static void ident_cpy (unsigned char *dest, unsigned char *src, unsigned int len);


#ifdef CONFIG_ATAPI
static void	atapi_inquiry(block_dev_desc_t *dev_desc);
ulong atapi_read (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer);
#endif


#ifdef CONFIG_IDE_8xx_DIRECT
static void set_pcmcia_timing (int pmode);
#endif

/* ------------------------------------------------------------------------- */

int do_ide (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int rcode = 0;

    switch (argc) {
    case 0:
    case 1:
	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
    case 2:
	if (strncmp(argv[1],"res",3) == 0) {
		puts ("\nReset IDE"
#ifdef CONFIG_IDE_8xx_DIRECT
			" on PCMCIA " PCMCIA_SLOT_MSG
#endif
			": ");

		ide_init ();
		return 0;
	} else if (strncmp(argv[1],"inf",3) == 0) {
		int i;

		putc ('\n');

		for (i=0; i<CFG_IDE_MAXDEVICE; ++i) {
			if (ide_dev_desc[i].type==DEV_TYPE_UNKNOWN)
				continue; /* list only known devices */
			printf ("IDE device %d: ", i);
			dev_print(&ide_dev_desc[i]);
		}
		return 0;

	} else if (strncmp(argv[1],"dev",3) == 0) {
		if ((curr_device < 0) || (curr_device >= CFG_IDE_MAXDEVICE)) {
			puts ("\nno IDE devices available\n");
			return 1;
		}
		printf ("\nIDE device %d: ", curr_device);
		dev_print(&ide_dev_desc[curr_device]);
		return 0;
	} else if (strncmp(argv[1],"part",4) == 0) {
		int dev, ok;

		for (ok=0, dev=0; dev<CFG_IDE_MAXDEVICE; ++dev) {
			if (ide_dev_desc[dev].part_type!=PART_TYPE_UNKNOWN) {
				++ok;
				if (dev)
					putc ('\n');
				print_part(&ide_dev_desc[dev]);
			}
		}
		if (!ok) {
			puts ("\nno IDE devices available\n");
			rcode ++;
		}
		return rcode;
	}
	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
    case 3:
	if (strncmp(argv[1],"dev",3) == 0) {
		int dev = (int)simple_strtoul(argv[2], NULL, 10);

		printf ("\nIDE device %d: ", dev);
		if (dev >= CFG_IDE_MAXDEVICE) {
			puts ("unknown device\n");
			return 1;
		}
		dev_print(&ide_dev_desc[dev]);
		/*ide_print (dev);*/

		if (ide_dev_desc[dev].type == DEV_TYPE_UNKNOWN) {
			return 1;
		}

		curr_device = dev;

		puts ("... is now current device\n");

		return 0;
	} else if (strncmp(argv[1],"part",4) == 0) {
		int dev = (int)simple_strtoul(argv[2], NULL, 10);

		if (ide_dev_desc[dev].part_type!=PART_TYPE_UNKNOWN) {
				print_part(&ide_dev_desc[dev]);
		} else {
			printf ("\nIDE device %d not available\n", dev);
			rcode = 1;
		}
		return rcode;
#if 0
	} else if (strncmp(argv[1],"pio",4) == 0) {
		int mode = (int)simple_strtoul(argv[2], NULL, 10);

		if ((mode >= 0) && (mode <= IDE_MAX_PIO_MODE)) {
			puts ("\nSetting ");
			pio_mode = mode;
			ide_init ();
		} else {
			printf ("\nInvalid PIO mode %d (0 ... %d only)\n",
				mode, IDE_MAX_PIO_MODE);
		}
		return;
#endif
	}

	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
    default:
	/* at least 4 args */

	if (strcmp(argv[1],"read") == 0) {
		ulong addr = simple_strtoul(argv[2], NULL, 16);
		ulong cnt  = simple_strtoul(argv[4], NULL, 16);
		ulong n;
#ifdef CFG_64BIT_STRTOUL
		lbaint_t blk  = simple_strtoull(argv[3], NULL, 16);

		printf ("\nIDE read: device %d block # %qd, count %ld ... ",
			curr_device, blk, cnt);
#else
		lbaint_t blk  = simple_strtoul(argv[3], NULL, 16);

		printf ("\nIDE read: device %d block # %ld, count %ld ... ",
			curr_device, blk, cnt);
#endif

		n = ide_dev_desc[curr_device].block_read (curr_device,
							  blk, cnt,
							  (ulong *)addr);
		/* flush cache after read */
		flush_cache (addr, cnt*ide_dev_desc[curr_device].blksz);

		printf ("%ld blocks read: %s\n",
			n, (n==cnt) ? "OK" : "ERROR");
		if (n==cnt) {
			return 0;
		} else {
			return 1;
		}
	} else if (strcmp(argv[1],"write") == 0) {
		ulong addr = simple_strtoul(argv[2], NULL, 16);
		ulong cnt  = simple_strtoul(argv[4], NULL, 16);
		ulong n;
#ifdef CFG_64BIT_STRTOUL
		lbaint_t blk  = simple_strtoull(argv[3], NULL, 16);

		printf ("\nIDE write: device %d block # %qd, count %ld ... ",
			curr_device, blk, cnt);
#else
		lbaint_t blk  = simple_strtoul(argv[3], NULL, 16);

		printf ("\nIDE write: device %d block # %ld, count %ld ... ",
			curr_device, blk, cnt);
#endif

		n = ide_write (curr_device, blk, cnt, (ulong *)addr);

		printf ("%ld blocks written: %s\n",
			n, (n==cnt) ? "OK" : "ERROR");
		if (n==cnt) {
			return 0;
		} else {
			return 1;
		}
	} else {
		printf ("Usage:\n%s\n", cmdtp->usage);
		rcode = 1;
	}

	return rcode;
    }
}

#ifdef CONFIG_CMD_DISKBOOT
int do_diskboot (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char *boot_device = NULL;
	char *ep;
	int dev, part = 0;
	ulong cnt;
	ulong addr;
	disk_partition_t info;
	image_header_t *hdr;
	int rcode = 0;

	switch (argc) {
	case 1:
		addr = load_addr;
		boot_device = getenv ("bootdevice");
		break;
	case 2:
		addr = simple_strtoul(argv[1], NULL, 16);
		boot_device = getenv ("bootdevice");
		break;
	case 3:
		addr = simple_strtoul(argv[1], NULL, 16);
		boot_device = argv[2];
		break;
	default:
		printf ("Usage:\n%s\n", cmdtp->usage);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	if (!boot_device) {
		puts ("\n** No boot device **\n");
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	dev = simple_strtoul(boot_device, &ep, 16);

	if (ide_dev_desc[dev].type==DEV_TYPE_UNKNOWN) {
		printf ("\n** Device %d not available\n", dev);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	if (*ep) {
		if (*ep != ':') {
			puts ("\n** Invalid boot device, use `dev[:part]' **\n");
			SHOW_BOOT_PROGRESS (-1);
			return 1;
		}
		part = simple_strtoul(++ep, NULL, 16);
	}
	if (get_partition_info (&ide_dev_desc[dev], part, &info)) {
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}
	if ((strncmp(info.type, BOOT_PART_TYPE, sizeof(info.type)) != 0) &&
	    (strncmp(info.type, BOOT_PART_COMP, sizeof(info.type)) != 0)) {
		printf ("\n** Invalid partition type \"%.32s\""
			" (expect \"" BOOT_PART_TYPE "\")\n",
			info.type);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	printf ("\nLoading from IDE device %d, partition %d: "
		"Name: %.32s  Type: %.32s\n",
		dev, part, info.name, info.type);

	PRINTF ("First Block: %ld,  # of blocks: %ld, Block Size: %ld\n",
		info.start, info.size, info.blksz);

	if (ide_dev_desc[dev].block_read (dev, info.start, 1, (ulong *)addr) != 1) {
		printf ("** Read error on %d:%d\n", dev, part);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	hdr = (image_header_t *)addr;

	if (ntohl(hdr->ih_magic) == IH_MAGIC) {

		print_image_hdr (hdr);

		cnt = (ntohl(hdr->ih_size) + sizeof(image_header_t));
		cnt += info.blksz - 1;
		cnt /= info.blksz;
		cnt -= 1;
	} else {
		printf("\n** Bad Magic Number **\n");
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	if (ide_dev_desc[dev].block_read (dev, info.start+1, cnt,
		      (ulong *)(addr+info.blksz)) != cnt) {
		printf ("** Read error on %d:%d\n", dev, part);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}


	/* Loading ok, update default load address */

	load_addr = addr;

#ifdef CONFIG_CMD_BOOTM
	/* Check if we should attempt an auto-start */
	if (((ep = getenv("autostart")) != NULL) && (strcmp(ep,"yes") == 0)) {
		char *local_args[2];
		extern int do_bootm (cmd_tbl_t *, int, int, char *[]);

		local_args[0] = argv[0];
		local_args[1] = NULL;

		printf ("Automatic boot of image at addr 0x%08lX ...\n", addr);

		do_bootm (cmdtp, 0, 1, local_args);
		rcode = 1;
	}
#endif
	return rcode;
}
U_BOOT_CMD(
	diskboot,	3,	1,	do_diskboot,
	"diskboot- boot from IDE device\n",
	"loadAddr dev:part\n"
);

#endif

/* ------------------------------------------------------------------------- */

/* Registers an ide (or ide-like: CF, SATA) device, returns the dev number assigned or -1 on error.
*  This needs to be called for all possibled devices on a bus at the time the bus is detected. */
int ide_register_device(int if_type, uint32_t quirks, uint8_t dev_on_bus, uint64_t data_addr, uint64_t reg_addr, uint64_t alt_addr, uint8_t dma_channel)
{

    if (ide_dev_desc_register_offset >= CFG_IDE_MAXDEVICE)
    {
        printf("ERROR: Unable to register IDE device, IDE descriptor list full!\n");
        return -1;
    }
    ide_dev_desc[ide_dev_desc_register_offset].if_type = if_type;
    ide_dev_desc[ide_dev_desc_register_offset].base_addr0 = data_addr;
    ide_dev_desc[ide_dev_desc_register_offset].base_addr1 = reg_addr;
    ide_dev_desc[ide_dev_desc_register_offset].base_addr2 = alt_addr;
    ide_dev_desc[ide_dev_desc_register_offset].quirks = quirks;
    ide_dev_desc[ide_dev_desc_register_offset].dev_on_bus = dev_on_bus;
    ide_dev_desc[ide_dev_desc_register_offset].dma_channel = dma_channel;

    PRINTF("Registering ide dev %d, data_base: 0x%llx, reg: 0x%llx\n",
           ide_dev_desc_register_offset, data_addr, reg_addr);

    ide_dev_desc[ide_dev_desc_register_offset].type=DEV_TYPE_UNKNOWN;
    ide_dev_desc[ide_dev_desc_register_offset].dev=ide_dev_desc_register_offset;
    ide_dev_desc[ide_dev_desc_register_offset].part_type=PART_TYPE_UNKNOWN;
    ide_dev_desc[ide_dev_desc_register_offset].blksz=0;
    ide_dev_desc[ide_dev_desc_register_offset].lba=0;
    ide_dev_desc[ide_dev_desc_register_offset].block_read=ide_read;


    return (ide_dev_desc_register_offset++);

}
block_dev_desc_t * ide_get_dev(int dev)
{
	return ((block_dev_desc_t *)&ide_dev_desc[dev]);
}


#define FLASH_RoundUP(_Dividend, _Divisor) (((_Dividend)+(_Divisor))/(_Divisor))



void ide_init (void)
{

#ifdef CONFIG_IDE_8xx_DIRECT
	DECLARE_GLOBAL_DATA_PTR;
	volatile immap_t *immr = (immap_t *)CFG_IMMR;
	volatile pcmconf8xx_t *pcmp = &(immr->im_pcmcia);
#endif
	unsigned char c;
	int i, bus;
#ifdef CONFIG_AMIGAONEG3SE
	unsigned int max_bus_scan;
	unsigned int ata_reset_time;
	char *s;
#endif
#ifdef CONFIG_IDE_8xx_PCCARD
	extern int pcmcia_on (void);
	extern int ide_devices_found; /* Initialized in check_ide_device() */
#endif	/* CONFIG_IDE_8xx_PCCARD */

#ifdef CONFIG_IDE_PREINIT
	extern int ide_preinit (void);
	WATCHDOG_RESET();

	if (ide_preinit ()) {
		puts ("ide_preinit failed\n");
		return;
	}
#endif	/* CONFIG_IDE_PREINIT */


#ifdef OCTEON_CF_RESET_GPIO
        /* Reset the CF card, as this seems to be necessary in some cases
        ** when using DMA */
        octeon_gpio_cfg_output(OCTEON_CF_RESET_GPIO);
        octeon_gpio_clr(OCTEON_CF_RESET_GPIO);
        CVMX_SYNC;
        udelay(20);
        octeon_gpio_set(OCTEON_CF_RESET_GPIO);
        CVMX_SYNC;
        udelay(100);
#endif

#ifdef CONFIG_IDE_8xx_PCCARD
	extern int pcmcia_on (void);
	extern int ide_devices_found; /* Initialized in check_ide_device() */

	WATCHDOG_RESET();

	ide_devices_found = 0;
	/* initialize the PCMCIA IDE adapter card */
	pcmcia_on();
	if (!ide_devices_found)
		return;
	udelay (1000000);	/* 1 s */
#endif	/* CONFIG_IDE_8xx_PCCARD */

	WATCHDOG_RESET();

#ifdef CONFIG_IDE_8xx_DIRECT
	/* Initialize PIO timing tables */
	for (i=0; i <= IDE_MAX_PIO_MODE; ++i) {
	    pio_config_clk[i].t_setup  = PCMCIA_MK_CLKS(pio_config_ns[i].t_setup,
							    gd->bus_clk);
	    pio_config_clk[i].t_length = PCMCIA_MK_CLKS(pio_config_ns[i].t_length,
							    gd->bus_clk);
	    pio_config_clk[i].t_hold   = PCMCIA_MK_CLKS(pio_config_ns[i].t_hold,
							    gd->bus_clk);
	    PRINTF ("PIO Mode %d: setup=%2d ns/%d clk"
		    "  len=%3d ns/%d clk"
		    "  hold=%2d ns/%d clk\n",
		    i,
		    pio_config_ns[i].t_setup,  pio_config_clk[i].t_setup,
		    pio_config_ns[i].t_length, pio_config_clk[i].t_length,
		    pio_config_ns[i].t_hold,   pio_config_clk[i].t_hold);
	}
#endif /* CONFIG_IDE_8xx_DIRECT */

	/* Reset the IDE just to be sure.
	 * Light LED's to show
	 */
	ide_led ((LED_IDE1 | LED_IDE2), 1);		/* LED's on	*/
	ide_reset (); /* ATAPI Drives seems to need a proper IDE Reset */

#ifdef CONFIG_IDE_8xx_DIRECT
	/* PCMCIA / IDE initialization for common mem space */
	pcmp->pcmc_pgcrb = 0;

	/* start in PIO mode 0 - most relaxed timings */
	pio_mode = 0;
	set_pcmcia_timing (pio_mode);
#endif /* CONFIG_IDE_8xx_DIRECT */





#if CONFIG_OCTEON_NAC38 && defined(CONFIG_PCI)
void via_ide_init(void);
        via_ide_init();
#endif
#if CFG_SIL_SATA_3124
        sil24_init_one();
#endif

	/*
	 * Wait for IDE to get ready.
	 * According to spec, this can take up to 31 seconds!
	 */
#ifndef CONFIG_AMIGAONEG3SE
	for (bus=0; bus<CFG_IDE_MAXBUS; ++bus) {
		int dev = bus * (CFG_IDE_MAXDEVICE / CFG_IDE_MAXBUS);
#else
	s = getenv("ide_maxbus");
	if (s)
	    max_bus_scan = simple_strtol(s, NULL, 10);
	else
	    max_bus_scan = CFG_IDE_MAXBUS;

	for (bus=0; bus<max_bus_scan; ++bus) {
		int dev = bus * (CFG_IDE_MAXDEVICE / max_bus_scan);
#endif

                /* IDE devices are now registered ahead of time, so skip unknown ones here */
                if (ide_dev_desc[bus*2].if_type == IF_TYPE_UNKNOWN)
                    continue;


#ifdef CONFIG_IDE_8xx_PCCARD
		/* Skip non-ide devices from probing */
		if ((ide_devices_found & (1 << bus)) == 0) {
			ide_led ((LED_IDE1 | LED_IDE2), 0); /* LED's off */
			continue;
		}
#endif

                /* NOTE: Assumes 2 drives per bus.... */
                if (ide_dev_desc[bus*2].if_type == IF_TYPE_IDE)
                {
                    if (ide_dev_desc[bus*2].quirks & BLOCK_QUIRK_IDE_CF)
                        printf(" Bus %d (CF Card): ", bus);
                    else
                        printf(" Bus %d (IDE)    : ", bus);
                }
                else if (ide_dev_desc[bus*2].if_type == IF_TYPE_SATA)
                {
                    if (ide_dev_desc[bus*2].quirks & BLOCK_QUIRK_IDE_4PORT)
                    {
                        if (!ide_bus_ok[bus])
                        {
                           ide_bus_ok[bus]  =1;
                           ide_bus_ok[bus+1]=1;
                           printf(" Bus %d (SATA)   : OK\n", bus);
                        }
                        continue;
                    }
                    else
                    {
                        ide_bus_ok[bus]  =1;
                        printf(" Bus %d (SATA)   : OK\n", bus);
                        continue;
                    }


                }

		ide_bus_ok[bus] = 0;

		/* Select device
		 */
		udelay (100000);		/* 100 ms */
		ide_outb (dev, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(dev));
		udelay (100000);		/* 100 ms */
#ifdef CONFIG_AMIGAONEG3SE
		ata_reset_time = ATA_RESET_TIME;
		s = getenv("ide_reset_timeout");
		if (s) ata_reset_time = 2*simple_strtol(s, NULL, 10);
#endif
		i = 0;
		do {
			udelay (10000);		/* 10 ms */

			c = ide_inb (dev, ATA_STATUS);
			i++;
#ifdef CONFIG_AMIGAONEG3SE
			if (i > (ata_reset_time * 100)) {
#else
			if (i > (ATA_RESET_TIME * 100)) {
#endif
				puts ("** Timeout **\n");
				ide_led ((LED_IDE1 | LED_IDE2), 0); /* LED's off */
#ifdef CONFIG_AMIGAONEG3SE
				/* If this is the second bus, the first one was OK */
				if (bus != 0) {
				    ide_bus_ok[bus] = 0;
				    goto skip_bus;
				}
#endif
// ***East :Check ATAPI command
				c=0xff;
				break;
//				return;
			}
			if ((i >= 100) && ((i%100)==0)) {
				putc ('.');
			}
		if ( c==0xff )
		   break;
		} while (c & ATA_STAT_BUSY);

		if (c & (ATA_STAT_BUSY | ATA_STAT_FAULT)) {
			puts ("not available  ");
			PRINTF ("Status = 0x%02X ", c);
#ifndef CONFIG_ATAPI /* ATAPI Devices do not set DRDY */
		} else  if ((c & ATA_STAT_READY) == 0) {
			puts ("not available  ");
			PRINTF ("Status = 0x%02X ", c);
#endif
		} else {
			puts ("OK ");
			ide_bus_ok[bus] = 1;
		}
		puts("\n");
		WATCHDOG_RESET();
	}

#ifdef CONFIG_AMIGAONEG3SE
      skip_bus:
#endif
	putc ('\n');

	ide_led ((LED_IDE1 | LED_IDE2), 0);	/* LED's off	*/

	curr_device = -1;
	for (i=0; i<CFG_IDE_MAXDEVICE; ++i) {
#ifdef CONFIG_IDE_LED
		int led = (IDE_BUS(i) == 0) ? LED_IDE1 : LED_IDE2;
#endif
                if (!ide_bus_ok[i])
                    continue;

                if (ide_dev_desc[i].if_type == IF_TYPE_UNKNOWN)
                {
                    /* Only initialize if not initialized during controller scan */
                    ide_dev_desc[i].type=DEV_TYPE_UNKNOWN;
                    ide_dev_desc[i].if_type=IF_TYPE_IDE;
                    ide_dev_desc[i].dev=i;
                    ide_dev_desc[i].part_type=PART_TYPE_UNKNOWN;
                    ide_dev_desc[i].blksz=0;
                    ide_dev_desc[i].lba=0;
                    ide_dev_desc[i].block_read=ide_read;
       		    if (!ide_bus_ok[IDE_BUS(i)])
    			continue;
                }
                /* We don't support multiple CF cards, and the 16 bit boot bus read/modify/write
                ** hack breaks the primary/secondary selection, so ignore all non-primary CF cards */
                if ((ide_dev_desc[i].quirks & BLOCK_QUIRK_IDE_CF) && ide_dev_desc[i].dev_on_bus == 1)
                    continue;

		ide_led (led, 1);		/* LED on	*/
		ide_ident(&ide_dev_desc[i]);
		ide_led (led, 0);		/* LED off	*/
		dev_print(&ide_dev_desc[i]);
/*		ide_print (i); */
		if ((ide_dev_desc[i].lba > 0) && (ide_dev_desc[i].blksz > 0)) {
			init_part (&ide_dev_desc[i]);			/* initialize partition type */
			if (curr_device < 0)
				curr_device = i;
		}
	}
	WATCHDOG_RESET();
}

/* ------------------------------------------------------------------------- */


#ifdef CONFIG_IDE_8xx_DIRECT

static void
set_pcmcia_timing (int pmode)
{
	volatile immap_t *immr = (immap_t *)CFG_IMMR;
	volatile pcmconf8xx_t *pcmp = &(immr->im_pcmcia);
	ulong timings;

	PRINTF ("Set timing for PIO Mode %d\n", pmode);

	timings = PCMCIA_SHT(pio_config_clk[pmode].t_hold)
		| PCMCIA_SST(pio_config_clk[pmode].t_setup)
		| PCMCIA_SL (pio_config_clk[pmode].t_length)
		;

	/* IDE 0
	 */
	pcmp->pcmc_pbr0 = CFG_PCMCIA_PBR0;
	pcmp->pcmc_por0 = CFG_PCMCIA_POR0
#if (CFG_PCMCIA_POR0 != 0)
			| timings
#endif
			;
	PRINTF ("PBR0: %08x  POR0: %08x\n", pcmp->pcmc_pbr0, pcmp->pcmc_por0);

	pcmp->pcmc_pbr1 = CFG_PCMCIA_PBR1;
	pcmp->pcmc_por1 = CFG_PCMCIA_POR1
#if (CFG_PCMCIA_POR1 != 0)
			| timings
#endif
			;
	PRINTF ("PBR1: %08x  POR1: %08x\n", pcmp->pcmc_pbr1, pcmp->pcmc_por1);

	pcmp->pcmc_pbr2 = CFG_PCMCIA_PBR2;
	pcmp->pcmc_por2 = CFG_PCMCIA_POR2
#if (CFG_PCMCIA_POR2 != 0)
			| timings
#endif
			;
	PRINTF ("PBR2: %08x  POR2: %08x\n", pcmp->pcmc_pbr2, pcmp->pcmc_por2);

	pcmp->pcmc_pbr3 = CFG_PCMCIA_PBR3;
	pcmp->pcmc_por3 = CFG_PCMCIA_POR3
#if (CFG_PCMCIA_POR3 != 0)
			| timings
#endif
			;
	PRINTF ("PBR3: %08x  POR3: %08x\n", pcmp->pcmc_pbr3, pcmp->pcmc_por3);

	/* IDE 1
	 */
	pcmp->pcmc_pbr4 = CFG_PCMCIA_PBR4;
	pcmp->pcmc_por4 = CFG_PCMCIA_POR4
#if (CFG_PCMCIA_POR4 != 0)
			| timings
#endif
			;
	PRINTF ("PBR4: %08x  POR4: %08x\n", pcmp->pcmc_pbr4, pcmp->pcmc_por4);

	pcmp->pcmc_pbr5 = CFG_PCMCIA_PBR5;
	pcmp->pcmc_por5 = CFG_PCMCIA_POR5
#if (CFG_PCMCIA_POR5 != 0)
			| timings
#endif
			;
	PRINTF ("PBR5: %08x  POR5: %08x\n", pcmp->pcmc_pbr5, pcmp->pcmc_por5);

	pcmp->pcmc_pbr6 = CFG_PCMCIA_PBR6;
	pcmp->pcmc_por6 = CFG_PCMCIA_POR6
#if (CFG_PCMCIA_POR6 != 0)
			| timings
#endif
			;
	PRINTF ("PBR6: %08x  POR6: %08x\n", pcmp->pcmc_pbr6, pcmp->pcmc_por6);

	pcmp->pcmc_pbr7 = CFG_PCMCIA_PBR7;
	pcmp->pcmc_por7 = CFG_PCMCIA_POR7
#if (CFG_PCMCIA_POR7 != 0)
			| timings
#endif
			;
	PRINTF ("PBR7: %08x  POR7: %08x\n", pcmp->pcmc_pbr7, pcmp->pcmc_por7);

}

#endif	/* CONFIG_IDE_8xx_DIRECT */

/* ------------------------------------------------------------------------- */

uint64_t get_ide_port(int dev, int port_arg)
{
    uint64_t port = port_arg;

   if (ide_dev_desc[dev].quirks & BLOCK_QUIRK_IDE_VIA)
   {
       // Solve VIA IDE problem
       if ( port == ATA_SECT_CNT )
          port = ATA_ERROR_REG;
       else if ( port == ATA_ERROR_REG )
          port = ATA_SECT_CNT;
       else if ( port == ATA_LBA_HIGH )
          port = ATA_DEV_HD;
       else if ( port == ATA_DEV_HD )
          port = ATA_LBA_HIGH;
   }

   /* In true ide mode, the bus is wired up in 16 bit mode, but Octeon's addr0
   ** line is not connected.  This allows us to access all of the 8 bit registers
   ** using the low byte lane of the bus.  We need to shift the register address left
   ** by one bit to get the register we want.
   */
   if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF_TRUE_IDE)
   {
       int __attribute__((unused)) orig_port = port;
       port = (port & ~0xff) | ((port & 0xFF) << 1);
       PRINTF("translated port from 0x%x to 0x%x\n", orig_port, port);
   }


   if (port & ATA_IO_DATA_FLAG)
      port = (port & 0xFF) + ide_dev_desc[dev].base_addr0;
   else if (port & ATA_IO_REG_FLAG)
      port = (port & 0xFF) + ide_dev_desc[dev].base_addr1;
   else if (port & ATA_IO_ALT_FLAG)
      port = (port & 0xFF) + ide_dev_desc[dev].base_addr2;

   return port;
}

#if defined(__PPC__) || defined(CONFIG_PXA_PCMCIA)
static void __inline__
ide_outb(int dev, int port, unsigned char val)
{
	PRINTF ("ide_outb (dev= %d, port= 0x%x, val= 0x%02x) : @ 0x%08lx\n",
		dev, port, val, (ATA_CURR_BASE(dev)+port));

	/* Ensure I/O operations complete */
	EIEIO;
	*((uchar *)(ATA_CURR_BASE(dev)+port)) = val;
}
#else	/* ! __PPC__ */
static void __inline__
ide_outb(int dev, int port, unsigned char val)
{
    uint64_t port_u64;

    port_u64 = get_ide_port(dev, port);

    PRINTF("ide_outb (0x%x) (0x%llx: 0x%x)\n", port, port_u64, val);

#ifdef OCTEON_CF_16_BIT_BUS
    /* Boot bus CF needs special handling for 16 bit boot bus */
    if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF_TRUE_IDE)
    {
        /* Only low byte is valid on the bus */
        cvmx_write64_uint16(port_u64, val);
    }
    else if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF)
    {
        ushort data,data1;

        if (port == ATA_DEV_HD )
        {
            /* If writing to byte in same short word as the command byte,
            ** we must send a dummy command as part of the write.
            ** Compact flash expects to be byte addressable, but in 16 bit
            ** mode Octeon only supports 16 bit operations. */
            data = 0x9500;
        }
        else
        {
#if 0
            /* Do a read/modify/write */
            data = inw((ATA_CURR_BASE(dev)+port)&~0x1);
            data = ((ATA_CURR_BASE(dev)+port) & 1) ? (data & 0xff) : (data & 0xFF00);
#else
            data = cvmx_read64_uint16(port_u64 & ~0x1ull);
            data = (port & 1) ? (data & 0xff) : (data & 0xFF00);
#endif
        }  
        data1 = val;
        data1 = (port & 1) ? data1 << 8 : data1 & 0xff;
        data = data | data1;
        cvmx_write64_uint16(port_u64 & ~1ull, data);

        /* Wait for dummy command to complete */
        if (port == ATA_DEV_HD )
            ide_wait (dev, IDE_TIME_OUT);	/* can't take over 500 ms */
    }
    else
#endif
    {
        cvmx_write64_uint8(port_u64, val);
    }
}
#endif	/* __PPC__ */


#ifdef OCTEON_CF_16_BIT_BUS
#define CF_IS_16_BIT    1
#else
#define CF_IS_16_BIT    0
#endif

#if defined(__PPC__) || defined(CONFIG_PXA_PCMCIA)
static unsigned char __inline__
ide_inb(int dev, int port)
{
	uchar val;
	/* Ensure I/O operations complete */
	EIEIO;
	val = *((uchar *)(ATA_CURR_BASE(dev)+port));
	PRINTF ("ide_inb (dev= %d, port= 0x%x) : @ 0x%08lx -> 0x%02x\n",
		dev, port, (ATA_CURR_BASE(dev)+port), val);
	return (val);
}
#else	/* ! __PPC__ */
static unsigned char __inline__
ide_inb(int dev, int port)
{
    unsigned char retval; 

    PRINTF("ide_inb: dev: %d, port: 0x%x\n", dev, port);
    uint64_t port_u64 = get_ide_port(dev, port);
    if (CF_IS_16_BIT && (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF))
        port_u64 ^= 1;

    retval = cvmx_read64_uint8(port_u64);
    PRINTF("ide_inb(0x%llx: 0x%x\n", port_u64, retval);

    return (retval);
}
#endif	/* __PPC__ */

#ifdef __PPC__
# ifdef CONFIG_AMIGAONEG3SE
static void
output_data_short(int dev, ulong *sect_buf, int words)
{
	ushort	*dbuf;
	volatile ushort	*pbuf;

	pbuf = (ushort *)(ATA_CURR_BASE(dev)+ATA_DATA_REG);
	dbuf = (ushort *)sect_buf;
	while (words--) {
		EIEIO;
		*pbuf = *dbuf++;
		EIEIO;
	}

	if (words&1)
	    *pbuf = 0;
}
# endif	/* CONFIG_AMIGAONEG3SE */
#endif /* __PPC_ */

/* We only need to swap data if we are running on a big endian cpu. */
/* But Au1x00 cpu:s already swaps data in big endian mode! */
#if defined(__LITTLE_ENDIAN) || defined(CONFIG_AU1X00)
#define input_swap_data(x,y,z) input_data(x,y,z)
#else

uint32_t input_swap32(uint32_t data)
{
    uint32_t tmp, tmp1;    
  
    tmp1 = (data >> 16) & 0xff;
    tmp = tmp1<<24;

    tmp1 = (data >> 24) & 0xff;
    tmp = tmp | (tmp1<<16);

    tmp1 = (data >> 0) & 0xff;
    tmp = tmp | (tmp1<<8);

    tmp1 = (data >> 8) & 0xff;
    tmp = tmp | (tmp1<<0);
    
    
    return(tmp);

}

static void
input_swap_data(int dev, ulong *sect_buf, int words)
{
#ifndef CONFIG_HMI10
  uint64_t port;

  port = get_ide_port(dev, ATA_DATA_REG);
  PRINTF("***input_swap_data: dev=%d, in input data base for read is %lx\n", dev, port);
  {
     uint32_t	*dbuf = (uint32_t *)sect_buf;
     words=words;
     while (words--) 
     {

#ifdef OCTEON_CF_16_BIT_BUS
         if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF_TRUE_IDE)
             *dbuf++ = ((cvmx_read64_uint16(port) << 16) | cvmx_read64_uint16(port));
         else if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF)
             *dbuf++ = (cvmx_read64_uint32(port));
         else
#endif
             *dbuf++ = input_swap32(cvmx_read64_uint32(port));
     }
  }

#else	/* CONFIG_HMI10 */
	uchar i;
	volatile uchar *pbuf_even = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_EVEN);
	volatile uchar *pbuf_odd  = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_ODD);
	ushort  *dbuf = (ushort *)sect_buf;

	while (words--) {
		for (i=0; i<2; i++) {
			*(((uchar *)(dbuf)) + 1) = *pbuf_even;
			*(uchar *)dbuf = *pbuf_odd;
			dbuf+=1;
		}
	}
#endif	/* CONFIG_HMI10 */
}
#endif	/* __LITTLE_ENDIAN || CONFIG_AU1X00 */


#if defined(__PPC__) || defined(CONFIG_PXA_PCMCIA)
static void
output_data(int dev, ulong *sect_buf, int words)
{
#ifndef CONFIG_HMI10
	ushort	*dbuf;
	volatile ushort	*pbuf;

	pbuf = (ushort *)(ATA_CURR_BASE(dev)+ATA_DATA_REG);
	dbuf = (ushort *)sect_buf;
	while (words--) {
		EIEIO;
		*pbuf = *dbuf++;
		EIEIO;
		*pbuf = *dbuf++;
	}
#else	/* CONFIG_HMI10 */
	uchar	*dbuf;
	volatile uchar	*pbuf_even;
	volatile uchar	*pbuf_odd;

	pbuf_even = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_EVEN);
	pbuf_odd  = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_ODD);
	dbuf = (uchar *)sect_buf;
	while (words--) {
		EIEIO;
		*pbuf_even = *dbuf++;
		EIEIO;
		*pbuf_odd = *dbuf++;
		EIEIO;
		*pbuf_even = *dbuf++;
		EIEIO;
		*pbuf_odd = *dbuf++;
	}
#endif	/* CONFIG_HMI10 */
}
#else	/* ! __PPC__ */
static void
output_data(int dev, ulong *sect_buf, int words)
{
  uint64_t port_u64;

  port_u64 = get_ide_port(dev, ATA_DATA_REG);

  PRINTF("output_data(): port: 0x%llx,  buf: %p, words: %d\n", port_u64, sect_buf, words);
  if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF)
  {
      //outsw(port_u64, sect_buf, words<<1);
      uint32_t	*dbuf = (uint32_t *)sect_buf;
      while (words--) 
      {
         cvmx_write_csr(port_u64, *dbuf++);
      } 
  }
  else
  {   
     uint32_t	*dbuf = (uint32_t *)sect_buf;
     while (words--) 
     {
	//octeon_write64_byte(OCTEON_PCI_IOSPACE_BASE+port, *dbuf++);
	cvmx_write_csr(OCTEON_PCI_IOSPACE_BASE+port_u64, *dbuf++);
     } 
  }

}
#endif	/* __PPC__ */

#if defined(__PPC__) || defined(CONFIG_PXA_PCMCIA)
static void
input_data(int dev, ulong *sect_buf, int words)
{
#ifndef CONFIG_HMI10
	ushort	*dbuf;
	volatile ushort	*pbuf;

	pbuf = (ushort *)(ATA_CURR_BASE(dev)+ATA_DATA_REG);
	dbuf = (ushort *)sect_buf;

	PRINTF("in input data base for read is %lx\n", (unsigned long) pbuf);

	while (words--) {
		EIEIO;
		*dbuf++ = *pbuf;
		EIEIO;
		*dbuf++ = *pbuf;
	}
#else	/* CONFIG_HMI10 */
	uchar	*dbuf;
	volatile uchar	*pbuf_even;
	volatile uchar	*pbuf_odd;

	pbuf_even = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_EVEN);
	pbuf_odd  = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_ODD);
	dbuf = (uchar *)sect_buf;
	while (words--) {
		EIEIO;
		*dbuf++ = *pbuf_even;
		EIEIO;
		*dbuf++ = *pbuf_odd;
		EIEIO;
		*dbuf++ = *pbuf_even;
		EIEIO;
		*dbuf++ = *pbuf_odd;
	}
#endif	/* CONFIG_HMI10 */
}
#else	/* ! __PPC__ */
static void
input_data(int dev, ulong *sect_buf, int words)
{
  uint64_t port;
  port = get_ide_port(dev, ATA_DATA_REG);
  PRINTF("***input_data !ppc: dev=%d, in input data base for read is %llx\n", dev, port);

  {
     uint32_t	*dbuf = (uint32_t *)sect_buf;
     while (words--) 
     {
#ifdef OCTEON_CF_16_BIT_BUS
        if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF_TRUE_IDE)
            *dbuf++ = input_swap32((cvmx_read64_uint16(port) << 16) | cvmx_read64_uint16(port));
        else if (ide_get_dev(dev)->quirks & BLOCK_QUIRK_IDE_CF)
            *dbuf++ = input_swap32(cvmx_read64_uint32(port));
        else
#endif
            *dbuf++ = cvmx_read64_uint32(port);
     } 
  }

}

#endif	/* __PPC__ */

#ifdef CONFIG_AMIGAONEG3SE
static void
input_data_short(int dev, ulong *sect_buf, int words)
{
	ushort	*dbuf;
	volatile ushort	*pbuf;

	pbuf = (ushort *)(ATA_CURR_BASE(dev)+ATA_DATA_REG);
	dbuf = (ushort *)sect_buf;
	while (words--) {
		EIEIO;
		*dbuf++ = *pbuf;
		EIEIO;
	}

	if (words&1) {
	    ushort dummy;
	    dummy = *pbuf;
	}
}
#endif

/* -------------------------------------------------------------------------
 */
static void ide_ident (block_dev_desc_t *dev_desc)
{
	ulong iobuf[ATA_SECTORWORDS];
	unsigned char c;
	hd_driveid_t *iop = (hd_driveid_t *)iobuf;

	memset(iobuf,0,sizeof(iobuf));

#ifdef CONFIG_AMIGAONEG3SE
	int max_bus_scan;
	int retries = 0;
	char *s;
	int do_retry = 0;
#endif

#if 0
	int mode, cycle_time;
#endif
	int device;
	device=dev_desc->dev;
	//printf ("  Device %d: ", device);
	printf ("  ide %d: ", device);
#if defined(CONFIG_PCI) && CFG_SIL_SATA_3124
	if (dev_desc->if_type == IF_TYPE_SATA)
	{
	   sata_ident(dev_desc);
	   return;
 	}
#endif

#ifdef CONFIG_AMIGAONEG3SE
	s = getenv("ide_maxbus");
	if (s) {
		max_bus_scan = simple_strtol(s, NULL, 10);
	} else {
		max_bus_scan = CFG_IDE_MAXBUS;
	}
	if (device >= max_bus_scan*2) {
		dev_desc->type=DEV_TYPE_UNKNOWN;
		return;
	}
#endif

	ide_led (DEVICE_LED(device), 1);	/* LED on	*/
	/* Select device
	 */
        /* NOTE: This is broken on 16-bit boot bus on Octeon.  The read/modify/write
        ** trick does not handle this properly */
	ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device));
//	dev_desc->if_type=IF_TYPE_IDE;
#ifdef CONFIG_ATAPI

#ifdef CONFIG_AMIGAONEG3SE
    do_retry = 0;
    retries = 0;

    /* Warning: This will be tricky to read */
    while (retries <= 1) {
#endif	/* CONFIG_AMIGAONEG3SE */

	/* check signature */
	if ((ide_inb(device,ATA_SECT_CNT) == 0x01) &&
		 (ide_inb(device,ATA_SECT_NUM) == 0x01) &&
		 (ide_inb(device,ATA_CYL_LOW)  == 0x14) &&
		 (ide_inb(device,ATA_CYL_HIGH) == 0xEB)) {
		/* ATAPI Signature found */
		dev_desc->if_type=IF_TYPE_ATAPI;
		/* Start Ident Command
		 */
		ide_outb (device, ATA_COMMAND, ATAPI_CMD_IDENT);
		/*
		 * Wait for completion - ATAPI devices need more time
		 * to become ready
		 */
		c = ide_wait (device, ATAPI_TIME_OUT);
	} else
#endif
	{
		/* Start Ident Command
		 */
		ide_outb (device, ATA_COMMAND, ATA_CMD_IDENT);

		/* Wait for completion
		 */
		c = ide_wait (device, IDE_TIME_OUT);
	}
	ide_led (DEVICE_LED(device), 0);	/* LED off	*/

	if (((c & ATA_STAT_DRQ) == 0) ||
	    ((c & (ATA_STAT_FAULT|ATA_STAT_ERR)) != 0) ) {
#ifdef CONFIG_AMIGAONEG3SE
		if (retries == 0) {
			do_retry = 1;
		} else {
			return;
		}
#else
		if ( device >= 2 && dev_desc->if_type!=IF_TYPE_ATAPI)	// If ATA identify fail, try ATAPI Identify for CD-ROM/DVD
		{
			//printf("***Try ATAPI Signature found 2\n");
			/* Try ATAPI Signature found */
			dev_desc->if_type=IF_TYPE_ATAPI;
			/* Start Ident Command
			 */

			ide_outb (device, ATA_COMMAND, ATAPI_CMD_IDENT);
			/*
			 * Wait for completion - ATAPI devices need more time
			 * to become ready
			 */
			c = ide_wait (device, ATAPI_TIME_OUT);
			if (((c & ATA_STAT_DRQ) == 0) ||
			    ((c & (ATA_STAT_FAULT|ATA_STAT_ERR)) != 0) ) {
	        		return;
			}
			//printf("****IDE %d ATAPI Identify Status 2 %x \n", device, ide_inb(device, ATA_COMMAND));
		}
		else
		return;
#endif	/* CONFIG_AMIGAONEG3SE */
	}

#ifdef CONFIG_AMIGAONEG3SE
	s = getenv("ide_doreset");
	if (s && strcmp(s, "on") == 0 && 1 == do_retry) {
		/* Need to soft reset the device in case it's an ATAPI...  */
		PRINTF("Retrying...\n");
		ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device));
		udelay(100000);
		ide_outb (device, ATA_COMMAND, 0x08);
		udelay (100000);	/* 100 ms */
		retries++;
	} else {
		retries = 100;
	}
    }	/* see above - ugly to read */
#endif	/* CONFIG_AMIGAONEG3SE */

	input_swap_data (device, iobuf, ATA_SECTORWORDS);

#ifdef IDE_DEBUG
// ** East

    int i, j;
    u8	*dbuf;
    dbuf = (u8 *)iobuf;
    
    printf("Dump Buffer: \n");
    for(i=0x00; i<512; i+=16)
    {
       printf("%04X: ", i);
       for(j=0; j<16; j++)
       {
          if ( j==7 )
             printf("%02x-",dbuf[i+j]);
          else
             printf("%02x ", dbuf[i+j]);
       }
       printf("  ");
       for(j=0; j<16; j++)
       {
          if ( dbuf[i+j]>=0x20 && dbuf[i+j]<='z' )
             printf("%c", dbuf[i+j]);
          else 
             printf(".");
       }
       printf("\n");
       if (i%256 == 0 && i!=0)
          printf("\n");
    }

#endif

#if 0
    printf("53:field valid: 0x%x\n", iop->field_valid);
    printf("63:MDMA info: 0x%x\n", iop->dma_mword);
    printf("65:min cycle: %d\n", iop->eide_dma_min);
    printf("66:rec cycle: %d\n", iop->eide_dma_time);
    printf("88:UDMA info: 0x%x\n", iop->dma_ultra);
    printf("163:ADV MDMA info: 0x%x\n", iop->words160_255[3]);
#endif

    /* Check for DMA capability and set device structure appropriately, if DMA has been configured for
    ** this device. */
    if (dev_desc->quirks & BLOCK_QUIRK_IDE_CF_TRUE_IDE && dev_desc->quirks & BLOCK_QUIRK_IDE_CF_TRUE_IDE_DMA)
    {
        int mwdma_mode;
        uint64_t tim_val = cvmx_compactflash_generate_dma_tim(4, (uint16_t *)iop, &mwdma_mode);
        if (tim_val)
        {
            dev_desc->dma_mode = mwdma_mode; 
            dev_desc->dma_channel = 0;  /* Hardwired on board */
            dev_desc->dma_type = CF_DMA_TYPE_MDMA;
            PRINTF("Using DMA for CF (mode %d)\n", dev_desc->dma_mode);
            cvmx_write_csr(CVMX_MIO_BOOT_DMA_TIMX(dev_desc->dma_channel), tim_val);
        }
        else
        {
            dev_desc->quirks &= ~BLOCK_QUIRK_IDE_CF_TRUE_IDE_DMA;
            dev_desc->dma_type = CF_DMA_TYPE_NONE;
        }
    }
    else
    {
        dev_desc->dma_mode = 0;
        dev_desc->dma_type = CF_DMA_TYPE_NONE;
    }


	ident_cpy (dev_desc->revision, iop->fw_rev, sizeof(dev_desc->revision));
	ident_cpy (dev_desc->vendor, iop->model, sizeof(dev_desc->vendor));
	ident_cpy (dev_desc->product, iop->serial_no, sizeof(dev_desc->product));
#ifdef __LITTLE_ENDIAN
	/*
	 * firmware revision and model number have Big Endian Byte
	 * order in Word. Convert both to little endian.
	 *
	 * See CF+ and CompactFlash Specification Revision 2.0:
	 * 6.2.1.6: Identfy Drive, Table 39 for more details
	 */

	strswab (dev_desc->revision);
	strswab (dev_desc->vendor);
#endif /* __LITTLE_ENDIAN */

	if ((iop->config & 0x0080)==0x0080)
		dev_desc->removable = 1;
	else
		dev_desc->removable = 0;

#if 0
	/*
	 * Drive PIO mode autoselection
	 */
	mode = iop->tPIO;

	printf ("tPIO = 0x%02x = %d\n",mode, mode);
	if (mode > 2) {		/* 2 is maximum allowed tPIO value */
		mode = 2;
		PRINTF ("Override tPIO -> 2\n");
	}
	if (iop->field_valid & 2) {	/* drive implements ATA2? */
		PRINTF ("Drive implements ATA2\n");
		if (iop->capability & 8) {	/* drive supports use_iordy? */
			cycle_time = iop->eide_pio_iordy;
		} else {
			cycle_time = iop->eide_pio;
		}
		PRINTF ("cycle time = %d\n", cycle_time);
		mode = 4;
		if (cycle_time > 120) mode = 3;	/* 120 ns for PIO mode 4 */
		if (cycle_time > 180) mode = 2;	/* 180 ns for PIO mode 3 */
		if (cycle_time > 240) mode = 1;	/* 240 ns for PIO mode 4 */
		if (cycle_time > 383) mode = 0;	/* 383 ns for PIO mode 4 */
	}
	printf ("PIO mode to use: PIO %d\n", mode);
#endif /* 0 */

#ifdef CONFIG_ATAPI
	if (dev_desc->if_type==IF_TYPE_ATAPI) {
		atapi_inquiry(dev_desc);
		return;
	}
#endif /* CONFIG_ATAPI */

#ifdef __BIG_ENDIAN
	/* swap shorts */
	dev_desc->lba = (iop->lba_capacity << 16) | (iop->lba_capacity >> 16);
#else	/* ! __BIG_ENDIAN */
	/*
	 * do not swap shorts on little endian
	 *
	 * See CF+ and CompactFlash Specification Revision 2.0:
	 * 6.2.1.6: Identfy Drive, Table 39, Word Address 57-58 for details.
	 */
	dev_desc->lba = iop->lba_capacity;
#endif	/* __BIG_ENDIAN */

#ifdef CONFIG_LBA48
	if (iop->command_set_2 & 0x0400) { /* LBA 48 support */
		dev_desc->lba48 = 1;
		dev_desc->lba = (unsigned long long)iop->lba48_capacity[0] |
						  ((unsigned long long)iop->lba48_capacity[1] << 16) |
						  ((unsigned long long)iop->lba48_capacity[2] << 32) |
						  ((unsigned long long)iop->lba48_capacity[3] << 48);
	} else {
		dev_desc->lba48 = 0;
	}
#endif /* CONFIG_LBA48 */
	/* assuming HD */
	dev_desc->type=DEV_TYPE_HARDDISK;
	dev_desc->blksz=ATA_BLOCKSIZE;
	dev_desc->lun=0; /* just to fill something in... */

#if 0 	/* only used to test the powersaving mode,
	 * if enabled, the drive goes after 5 sec
	 * in standby mode */
	ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device));
	c = ide_wait (device, IDE_TIME_OUT);
	ide_outb (device, ATA_SECT_CNT, 1);
	ide_outb (device, ATA_LBA_LOW,  0);
	ide_outb (device, ATA_LBA_MID,  0);
	ide_outb (device, ATA_LBA_HIGH, 0);
	ide_outb (device, ATA_DEV_HD,   ATA_LBA		|
				    ATA_DEVICE(device));
	ide_outb (device, ATA_COMMAND,  0xe3);
	udelay (50);
	c = ide_wait (device, IDE_TIME_OUT);	/* can't take over 500 ms */
#endif


        /* Enable DMA if requested */
        /* This doesn't seem to be required, although it may be on some devices */
        if (0 && (ide_get_dev(device)->quirks & BLOCK_QUIRK_IDE_CF_TRUE_IDE_DMA))
        {
            /* This does not seem to make a difference on Sandisk cards */
            printf("Setting DMA mode for device\n");
            ide_outb (device, ATA_SECT_CNT,  0x22);  // MWDMA mode 2
            ide_outb (device, ATA_ERROR_REG,  0x3); 
            ide_outb (device, ATA_COMMAND,  0xef);
            udelay (50);
            c = ide_wait (device, IDE_TIME_OUT);	/* can't take over 500 ms */
        }
}


/* ------------------------------------------------------------------------- */
ulong ide_read_dma(int device, lbaint_t blknr, ulong blkcnt, ulong *buffer)
{
    unsigned char c;
    ulong n=0;

    PRINTF ("ide_read_DMA dev %d start %llx, blocks %lX buffer at %lX\n",
            device, blknr, blkcnt, (ulong)buffer);

#define IDE_DMA_MAX_BLOCK_CHUNK     256
    while (blkcnt > 0) {
        int blocks;


            c = ide_wait (device, IDE_TIME_OUT);

            if (c & ATA_STAT_BUSY) {
                    printf ("IDE read: device %d not idle\n", device);
                    break;
            }
            /* Sleep of 20 required here or data is corrupted.... */
            udelay (20);

            if (blkcnt >= IDE_DMA_MAX_BLOCK_CHUNK)
                blocks = IDE_DMA_MAX_BLOCK_CHUNK;
            else
                blocks = blkcnt;

            cvmx_mio_boot_dma_cfgx_t dma_cfg;
            dma_cfg.u64 = 0;
            dma_cfg.s.en = 1;
            dma_cfg.s.swap8 = 1;
            dma_cfg.s.adr = cvmx_ptr_to_phys(buffer);
            dma_cfg.s.adr = ((uint32_t)buffer) & 0x7fffffff;
            // Todo fix use of addresses from bootmem_alloc
//            printf("ptr: %p, address: 0x%llx\n", buffer, (unsigned long long)dma_cfg.s.adr);
            dma_cfg.s.size = blocks * (512/2) - 1;
            cvmx_write_csr(CVMX_MIO_BOOT_DMA_CFGX(ide_get_dev(device)->dma_channel), dma_cfg.u64);
#if 0
            dma_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_DMA_CFGX(ide_get_dev(device)->dma_channel));
            if (!dma_cfg.s.en)
            {
                printf("ERROR DMA enable low before expected\n");
            }
#endif

            ide_outb (device, ATA_SECT_CNT, blocks);
            ide_outb (device, ATA_LBA_LOW,  (blknr >>  0) & 0xFF);
            ide_outb (device, ATA_LBA_MID,  (blknr >>  8) & 0xFF);
            ide_outb (device, ATA_LBA_HIGH, (blknr >> 16) & 0xFF);

            ide_outb (device, ATA_DEV_HD,   ATA_LBA		|
                                        ATA_DEVICE(device)	|
                                        ((blknr >> 24) & 0xF) );


            ide_outb (device, ATA_COMMAND,  ATA_CMD_RD_DMA);


            /* Poll for DMA completion */
            int to = 10000;
            while (--to)
            {
                dma_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_DMA_CFGX(0));
                if (!dma_cfg.s.en)
                    break;
                cvmx_wait(10000);
            }
            if (dma_cfg.s.en)
            {
                printf("ERROR: timeout waiting for DMA completion\n");
            }
            cvmx_write_csr(CVMX_MIO_BOOT_DMA_CFGX(0), 0ull);

            if (0)
            {

                int i, j;
                u8	*dbuf;
                dbuf = (u8 *)buffer;

                printf("Dumping buffer %p\n", buffer);
                for(i=0x00; i<512; i+=16)
                {
                   printf("%04X: ", i);
                   for(j=0; j<16; j++)
                   {
                      if ( j==7 )
                         printf("%02x-",dbuf[i+j]);
                      else
                         printf("%02x ", dbuf[i+j]);
                   }
                   printf("  ");
                   for(j=0; j<16; j++)
                   {
                      if ( dbuf[i+j]>=0x20 && dbuf[i+j]<='z' )
                         printf("%c", dbuf[i+j]);
                      else 
                         printf(".");
                   }
                   printf("\n");
                   if (i%256 == 0 && i!=0)
                      printf("\n");
                }

            }

            n += blocks;
            blknr += blocks;
            buffer += ATA_SECTORWORDS * blocks;
            blkcnt -= blocks;
    }

    (void) ide_inb (device, ATA_STATUS);	/* clear IRQ */

    /* This seems to be required to get some CF devices to not interfere with the
    ** boot bus on reset..... */

    ide_outb (device, ATA_COMMAND,  ATA_CMD_IDLE);
    udelay (20);
    return(n);

}
ulong ide_read (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer)
{
	ulong n = 0;
	unsigned char c;
	unsigned char pwrsave=0; /* power save */
#ifdef CONFIG_LBA48
	unsigned char lba48 = 0;

	if (blknr & 0x0000fffff0000000ull) {
		/* more than 28 bits used, use 48bit mode */
		lba48 = 1;
	}
#endif
	PRINTF ("ide_read dev %d start %llx, blocks %lX buffer at %lX\n",
		device, blknr, blkcnt, (ulong)buffer);

        if (1)
        {
            /* If DMA enabled, use dedicated read function */
            if (ide_get_dev(device)->dma_type == CF_DMA_TYPE_MDMA)
                return ide_read_dma(device,blknr,blkcnt,buffer);
        }


#if  defined(CONFIG_PCI) && CFG_SIL_SATA_3124
	if (ide_dev_desc[device].if_type == IF_TYPE_SATA)
	   return (sata_read(device, blknr, blkcnt, buffer));
#endif

        ide_led (DEVICE_LED(device), 1);	/* LED on	*/

#if (CFG_IDE_MAXDEVICE > 1)   /* No octeon boards need this - only select device if more than one allowed */
	/* Select device
	 */
	ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device));
	c = ide_wait (device, IDE_TIME_OUT);

	if (c & ATA_STAT_BUSY) {
		printf ("IDE read: device %d not ready\n", device);
		goto IDE_READ_E;
	}

	/* first check if the drive is in Powersaving mode, if yes,
	 * increase the timeout value */
	ide_outb (device, ATA_COMMAND,  ATA_CMD_CHK_PWR);
	udelay (50);

	c = ide_wait (device, IDE_TIME_OUT);	/* can't take over 500 ms */

	if (c & ATA_STAT_BUSY) {
		printf ("IDE read: device %d not ready\n", device);
		goto IDE_READ_E;
	}
	if ((c & ATA_STAT_ERR) == ATA_STAT_ERR) {
		PRINTF ("No Powersaving mode %X\n", c);
	} else {
		c = ide_inb(device,ATA_SECT_CNT);
		PRINTF("Powersaving %02X\n",c);
		if(c==0)
			pwrsave=1;
	}
#endif


	while (blkcnt-- > 0) {

		c = ide_wait (device, IDE_TIME_OUT);

		if (c & ATA_STAT_BUSY) {
			printf ("IDE read: device %d not ready\n", device);
			break;
		}
#ifdef CONFIG_LBA48
		if (lba48) {
			/* write high bits */
			ide_outb (device, ATA_SECT_CNT, 0);
			ide_outb (device, ATA_LBA_LOW,	(blknr >> 24) & 0xFF);
			ide_outb (device, ATA_LBA_MID,	(blknr >> 32) & 0xFF);
			ide_outb (device, ATA_LBA_HIGH, (blknr >> 40) & 0xFF);
		}
#endif
		ide_outb (device, ATA_SECT_CNT, 1);
		ide_outb (device, ATA_LBA_LOW,  (blknr >>  0) & 0xFF);
		ide_outb (device, ATA_LBA_MID,  (blknr >>  8) & 0xFF);
		ide_outb (device, ATA_LBA_HIGH, (blknr >> 16) & 0xFF);

#ifdef CONFIG_LBA48
		if (lba48) {
			ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device) );
			ide_outb (device, ATA_COMMAND, ATA_CMD_READ_EXT);

		} else
#endif
		{
#ifdef OCTEON_CF_16_BIT_BUS_TODO_OPTIMIZE
                    /* This is a case worth optimizing to avoid the dummy command, as this is done once per
                    ** sector. */
                    uint16_t data = (ATA_CMD_READ << 8) | (ATA_LBA | ATA_DEVICE(device) | ((blknr >> 24) & 0xF));
                    outw(data, (ATA_CURR_BASE(device) + ATA_DEV_HD));
#else
                    ide_outb (device, ATA_DEV_HD,   ATA_LBA		|
                                                ATA_DEVICE(device)	|
                                                ((blknr >> 24) & 0xF) );
                    ide_outb (device, ATA_COMMAND,  ATA_CMD_READ);
#endif
		}

		//udelay (50);
		//if ( device == 0 )	// *** East Add, for some 1GB CF card
	   	//   udelay (150);		

		if(pwrsave) {
			c = ide_wait (device, IDE_SPIN_UP_TIME_OUT);	/* may take up to 4 sec */
			pwrsave=0;
		} else {
			c = ide_wait (device, IDE_TIME_OUT);	/* can't take over 500 ms */
		}

		if ( (c & (ATA_STAT_BUSY|ATA_STAT_ERR) ) == 0 )	// Wait DRQ ready replace delay 200ns
		   c=ide_wait_drq(device, IDE_TIME_OUT);

//printf("****Read IDE %d ,blknr=%x, blkcnt=%d ", device, blknr, blkcnt);
//printf("cnt=%x, lba=%x %x %x, hd=%x\n", ide_inb(device, ATA_SECT_CNT), ide_inb(device, ATA_LBA_HIGH), ide_inb(device, ATA_LBA_MID), ide_inb(device, ATA_LBA_LOW), ide_inb(device, ATA_DEV_HD));
/*printf("0x01 ATA_ERROR_REG %02x \n", ide_inb(device, ATA_ERROR_REG));
printf("0x02 ATA_SECT_CNT  %02x \n", ide_inb(device, ATA_SECT_CNT));
printf("0x03 ATA_LBA_LOW   %02x \n", ide_inb(device, ATA_LBA_LOW));
printf("0x04 ATA_LBA_MID   %02x \n", ide_inb(device, ATA_LBA_MID));
printf("0x05 ATA_LBA_HIGH  %02x \n", ide_inb(device, ATA_LBA_HIGH));
printf("0x06 ATA_DEV_HD    %02x \n", ide_inb(device, ATA_DEV_HD));
printf("0x07 ATA_COMMAND   %02x \n", ide_inb(device, ATA_COMMAND));*/

		if ((c&(ATA_STAT_DRQ|ATA_STAT_BUSY|ATA_STAT_ERR)) != ATA_STAT_DRQ) {
#if defined(CFG_64BIT_LBA) && defined(CFG_64BIT_VSPRINTF)
			printf ("Error on read (no IRQ) dev %d blk %qd: status 0x%02x\n",
				device, blknr, c);
#else
			printf ("Error on read (no IRQ) dev %d blk %ld: status 0x%02x\n",
				device, (ulong)blknr, c);
#endif
			break;
		}

		input_data (device, buffer, ATA_SECTORWORDS);
		(void) ide_inb (device, ATA_STATUS);	/* clear IRQ */

		++n;
		++blknr;
		buffer += ATA_SECTORWORDS;
	}
#if (CFG_IDE_MAXDEVICE > 1)
IDE_READ_E:
#endif
	ide_led (DEVICE_LED(device), 0);	/* LED off	*/
	return (n);
}

/* ------------------------------------------------------------------------- */


ulong ide_write (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer)
{
	ulong n = 0;
	unsigned char c;
#ifdef CONFIG_LBA48
	unsigned char lba48 = 0;

	if (blknr & 0x0000fffff0000000ULL) {
		/* more than 28 bits used, use 48bit mode */
		lba48 = 1;
	}
#endif

	ide_led (DEVICE_LED(device), 1);	/* LED on	*/

#if defined(CONFIG_PCI) && CFG_SIL_SATA_3124
	if (ide_dev_desc[device].if_type == IF_TYPE_SATA)
	   return (sata_write(device, blknr, blkcnt, buffer));
#endif

	/* Select device
	 */
	ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device));

	while (blkcnt-- > 0) {

		c = ide_wait (device, IDE_TIME_OUT);

		if (c & ATA_STAT_BUSY) {
			printf ("IDE read: device %d not ready\n", device);
			goto WR_OUT;
		}
#ifdef CONFIG_LBA48
		if (lba48) {
			/* write high bits */
			ide_outb (device, ATA_SECT_CNT, 0);
			ide_outb (device, ATA_LBA_LOW,	(blknr >> 24) & 0xFF);
			ide_outb (device, ATA_LBA_MID,	(blknr >> 32) & 0xFF);
			ide_outb (device, ATA_LBA_HIGH, (blknr >> 40) & 0xFF);
		}
#endif
		ide_outb (device, ATA_SECT_CNT, 1);
		ide_outb (device, ATA_LBA_LOW,  (blknr >>  0) & 0xFF);
		ide_outb (device, ATA_LBA_MID,  (blknr >>  8) & 0xFF);
		ide_outb (device, ATA_LBA_HIGH, (blknr >> 16) & 0xFF);

#ifdef CONFIG_LBA48
		if (lba48) {
			ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device) );
			ide_outb (device, ATA_COMMAND,	ATA_CMD_WRITE_EXT);

		} else
#endif
		{
			ide_outb (device, ATA_DEV_HD,   ATA_LBA		|
						    ATA_DEVICE(device)	|
						    ((blknr >> 24) & 0xF) );
			ide_outb (device, ATA_COMMAND,  ATA_CMD_WRITE);
		}

		udelay (50);
		//udelay (100);		// *** East Add for some CF card

		c = ide_wait (device, IDE_TIME_OUT);	/* can't take over 500 ms */

		if ((c&(ATA_STAT_DRQ|ATA_STAT_BUSY|ATA_STAT_ERR)) != ATA_STAT_DRQ) {
#if defined(CFG_64BIT_LBA) && defined(CFG_64BIT_VSPRINTF)
			printf ("Error on write (no IRQ) dev %d blk %qd: status 0x%02x\n",
				device, blknr, c);
#else
			printf ("Error on write (no IRQ) dev %d blk %ld: status 0x%02x\n",
				device, (ulong)blknr, c);
#endif
			goto WR_OUT;
		}

		output_data (device, buffer, ATA_SECTORWORDS);
		c = ide_inb (device, ATA_STATUS);	/* clear IRQ */
		++n;
		++blknr;
		buffer += ATA_SECTORWORDS;
	}
WR_OUT:
	ide_led (DEVICE_LED(device), 0);	/* LED off	*/
	return (n);
}

/* ------------------------------------------------------------------------- */

/*
 * copy src to dest, skipping leading and trailing blanks and null
 * terminate the string
 * "len" is the size of available memory including the terminating '\0'
 */
static void ident_cpy (unsigned char *dst, unsigned char *src, unsigned int len)
{
	unsigned char *end, *last;

	last = dst;
	end  = src + len - 1;

	/* reserve space for '\0' */
	if (len < 2)
		goto OUT;

	/* skip leading white space */
	while ((*src) && (src<end) && (*src==' '))
		++src;

	/* copy string, omitting trailing white space */
	while ((*src) && (src<end)) {
		*dst++ = *src;
		if (*src++ != ' ')
			last = dst;
	}
OUT:
	*last = '\0';
}

/* ------------------------------------------------------------------------- */

/*
 * Wait until Busy bit is off, or timeout (in ms)
 * Return last status
 */
static uchar ide_wait (int dev, ulong t)
{
	ulong delay = 200 * t;
	uchar c;

	while ((c = ide_inb(dev, ATA_STATUS)) & ATA_STAT_BUSY) {
		udelay (5);
		if (delay-- == 0) {
			break;
		}
	}
	return (c);
}

static uchar ide_wait_drq (int dev, ulong t)
{
	ulong delay = 1000 * t;
	uchar c;
	
	while (((c = ide_inb(dev, ATA_STATUS)) & ATA_STAT_DRQ)!=ATA_STAT_DRQ) {
		udelay (1);
		if (delay-- == 0) {
			break;
		}
	}
	return (c);
}

/* ------------------------------------------------------------------------- */

#ifdef CONFIG_IDE_RESET
extern void ide_set_reset(int idereset);

static void ide_reset (void)
{
#if defined(CFG_PB_12V_ENABLE) || defined(CFG_PB_IDE_MOTOR)
	volatile immap_t *immr = (immap_t *)CFG_IMMR;
#endif
	int i;

	curr_device = -1;
	for (i=0; i<CFG_IDE_MAXBUS; ++i)
		ide_bus_ok[i] = 0;
	for (i=0; i<CFG_IDE_MAXDEVICE; ++i)
		ide_dev_desc[i].type = DEV_TYPE_UNKNOWN;

        ide_dev_desc_register_offset = 0;
	ide_set_reset (1); /* assert reset */

	WATCHDOG_RESET();

#ifdef CFG_PB_12V_ENABLE
	immr->im_cpm.cp_pbdat &= ~(CFG_PB_12V_ENABLE);	/* 12V Enable output OFF */
	immr->im_cpm.cp_pbpar &= ~(CFG_PB_12V_ENABLE);
	immr->im_cpm.cp_pbodr &= ~(CFG_PB_12V_ENABLE);
	immr->im_cpm.cp_pbdir |=   CFG_PB_12V_ENABLE;

	/* wait 500 ms for the voltage to stabilize
	 */
	for (i=0; i<500; ++i) {
		udelay (1000);
	}

	immr->im_cpm.cp_pbdat |=   CFG_PB_12V_ENABLE;	/* 12V Enable output ON */
#endif	/* CFG_PB_12V_ENABLE */

#ifdef CFG_PB_IDE_MOTOR
	/* configure IDE Motor voltage monitor pin as input */
	immr->im_cpm.cp_pbpar &= ~(CFG_PB_IDE_MOTOR);
	immr->im_cpm.cp_pbodr &= ~(CFG_PB_IDE_MOTOR);
	immr->im_cpm.cp_pbdir &= ~(CFG_PB_IDE_MOTOR);

	/* wait up to 1 s for the motor voltage to stabilize
	 */
	for (i=0; i<1000; ++i) {
		if ((immr->im_cpm.cp_pbdat & CFG_PB_IDE_MOTOR) != 0) {
			break;
		}
		udelay (1000);
	}

	if (i == 1000) {	/* Timeout */
		printf ("\nWarning: 5V for IDE Motor missing\n");
# ifdef CONFIG_STATUS_LED
#  ifdef STATUS_LED_YELLOW
		status_led_set  (STATUS_LED_YELLOW, STATUS_LED_ON );
#  endif
#  ifdef STATUS_LED_GREEN
		status_led_set  (STATUS_LED_GREEN,  STATUS_LED_OFF);
#  endif
# endif	/* CONFIG_STATUS_LED */
	}
#endif	/* CFG_PB_IDE_MOTOR */

	WATCHDOG_RESET();

	/* de-assert RESET signal */
	ide_set_reset(0);

	/* wait 250 ms */
	for (i=0; i<250; ++i) {
		udelay (1000);
	}
}

#endif	/* CONFIG_IDE_RESET */

/* ------------------------------------------------------------------------- */

#if defined(CONFIG_IDE_LED) && \
   !defined(CONFIG_AMIGAONEG3SE) && \
   !defined(CONFIG_KUP4K) && \
   !defined(CONFIG_KUP4X) && \
   !defined(CONFIG_HMI10)

static	uchar	led_buffer = 0;		/* Buffer for current LED status	*/

static void ide_led (uchar led, uchar status)
{
	uchar *led_port = LED_PORT;

	if (status)	{		/* switch LED on	*/
		led_buffer |=  led;
	} else {			/* switch LED off	*/
		led_buffer &= ~led;
	}

	*led_port = led_buffer;
}

#endif	/* CONFIG_IDE_LED */

/* ------------------------------------------------------------------------- */

#ifdef CONFIG_ATAPI
/****************************************************************************
 * ATAPI Support
 */


#undef	ATAPI_DEBUG

#ifdef	ATAPI_DEBUG
#define	AT_PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define AT_PRINTF(fmt,args...)
#endif

#if defined(__PPC__) || defined(CONFIG_PXA_PCMCIA)
/* since ATAPI may use commands with not 4 bytes alligned length
 * we have our own transfer functions, 2 bytes alligned */
static void
output_data_shorts(int dev, ushort *sect_buf, int shorts)
{
#ifndef CONFIG_HMI10
	ushort	*dbuf;
	volatile ushort	*pbuf;

	pbuf = (ushort *)(ATA_CURR_BASE(dev)+ATA_DATA_REG);
	dbuf = (ushort *)sect_buf;

	PRINTF("in output data shorts base for read is %lx\n", (unsigned long) pbuf);

	while (shorts--) {
		EIEIO;
		*pbuf = *dbuf++;
	}
#else	/* CONFIG_HMI10 */
	uchar	*dbuf;
	volatile uchar	*pbuf_even;
	volatile uchar	*pbuf_odd;

	pbuf_even = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_EVEN);
	pbuf_odd  = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_ODD);
	while (shorts--) {
		EIEIO;
		*pbuf_even = *dbuf++;
		EIEIO;
		*pbuf_odd = *dbuf++;
	}
#endif	/* CONFIG_HMI10 */
}

static void
input_data_shorts(int dev, ushort *sect_buf, int shorts)
{
#ifndef CONFIG_HMI10
	ushort	*dbuf;
	volatile ushort	*pbuf;

	pbuf = (ushort *)(ATA_CURR_BASE(dev)+ATA_DATA_REG);
	dbuf = (ushort *)sect_buf;

	PRINTF("in input data shorts base for read is %lx\n", (unsigned long) pbuf);

	while (shorts--) {
		EIEIO;
		*dbuf++ = *pbuf;
	}
#else	/* CONFIG_HMI10 */
	uchar	*dbuf;
	volatile uchar	*pbuf_even;
	volatile uchar	*pbuf_odd;

	pbuf_even = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_EVEN);
	pbuf_odd  = (uchar *)(ATA_CURR_BASE(dev)+ATA_DATA_ODD);
	while (shorts--) {
		EIEIO;
		*dbuf++ = *pbuf_even;
		EIEIO;
		*dbuf++ = *pbuf_odd;
	}
#endif	/* CONFIG_HMI10 */
}

#else	/* ! __PPC__ */
static void
output_data_shorts(int dev, ushort *sect_buf, int shorts)
{
  int port;
  int port_even;
  int port_odd;

  port = get_ide_port(dev, ATA_DATA_REG);
  port_even = get_ide_port(dev, ATA_DATA_EVEN);
  port_odd  = get_ide_port(dev, ATA_DATA_ODD);

  PRINTF("***input_data_short !ppc: dev=%d, in output data base for write is %lx\n", dev, port);
  if ( dev==0 || dev==1)
	outsw(ATA_CURR_BASE(dev)+port, sect_buf, shorts);
  else
  {
      u16 *dbuf = (u16 *)sect_buf;
      u16 val_16;

      while (shorts--) 
      {
	 val_16 = *dbuf++;
	 cvmx_write64_uint16(OCTEON_PCI_IOSPACE_BASE+port, val_16);
      }
  }
}


static void
input_data_shorts(int dev, ushort *sect_buf, int shorts)
{
  int port;
  int port_even;
  int port_odd;

  port = get_ide_port(dev, ATA_DATA_REG);
  port_even = get_ide_port(dev, ATA_DATA_EVEN);
  port_odd  = get_ide_port(dev, ATA_DATA_ODD);
  PRINTF("***input_data_shorts !ppc: dev=%d, in input data base for read is %lx\n", dev, port);
  if ( dev==0 || dev==1)
	insw(ATA_CURR_BASE(dev)+port, sect_buf, shorts);
  else
  {
     u16	*dbuf = (u16 *)sect_buf;
     
     shorts=shorts>>1;
     while (shorts--) 
     {
	*dbuf++ = cvmx_read64_uint16(OCTEON_PCI_IOSPACE_BASE+port);
	*dbuf++ = cvmx_read64_uint16(OCTEON_PCI_IOSPACE_BASE+port);

     } 
  }
}

#endif	/* __PPC__ */

/*
 * Wait until (Status & mask) == res, or timeout (in ms)
 * Return last status
 * This is used since some ATAPI CD ROMs clears their Busy Bit first
 * and then they set their DRQ Bit
 */
static uchar atapi_wait_mask (int dev, ulong t,uchar mask, uchar res)
{
	ulong delay = 10 * t;		/* poll every 100 us */
	uchar c;

	c = ide_inb(dev,ATA_DEV_CTL); /* prevents to read the status before valid */
	while (((c = ide_inb(dev, ATA_STATUS)) & mask) != res) {
		/* break if error occurs (doesn't make sense to wait more) */
		if((c & ATA_STAT_ERR)==ATA_STAT_ERR)
			break;
		udelay (100);
		if (delay-- == 0) {
			break;
		}
	}
	return (c);
}

/*
 * issue an atapi command
 */
unsigned char atapi_issue(int device,unsigned char* ccb,int ccblen, unsigned char * buffer,int buflen)
{
	unsigned char c,err,mask,res;
	int n;
	ide_led (DEVICE_LED(device), 1);	/* LED on	*/

	/* Select device
	 */
	mask = ATA_STAT_BUSY|ATA_STAT_DRQ;
	res = 0;
#ifdef	CONFIG_AMIGAONEG3SE
# warning THF: Removed LBA mode ???
#endif
	ide_outb (device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device));
	c = atapi_wait_mask(device,ATAPI_TIME_OUT,mask,res);
	if ((c & mask) != res) {
		printf ("ATAPI_ISSUE: device %d not ready status %X\n", device,c);
		err=0xFF;
		goto AI_OUT;
	}
	/* write taskfile */
	ide_outb (device, ATA_ERROR_REG, 0); /* no DMA, no overlaped */
	ide_outb (device, ATA_SECT_CNT, 0);
	ide_outb (device, ATA_SECT_NUM, 0);
	ide_outb (device, ATA_CYL_LOW,  (unsigned char)(buflen & 0xFF));
	ide_outb (device, ATA_CYL_HIGH, (unsigned char)((buflen>>8) & 0xFF));
#ifdef	CONFIG_AMIGAONEG3SE
# warning THF: Removed LBA mode ???
#endif
	ide_outb (device, ATA_DEV_HD,   ATA_LBA | ATA_DEVICE(device));

	ide_outb (device, ATA_COMMAND,  ATAPI_CMD_PACKET);
	udelay (50);

	mask = ATA_STAT_DRQ|ATA_STAT_BUSY|ATA_STAT_ERR;
	res = ATA_STAT_DRQ;
	c = atapi_wait_mask(device,ATAPI_TIME_OUT,mask,res);

	if ((c & mask) != res) { /* DRQ must be 1, BSY 0 */
		printf ("ATTAPI_ISSUE: Error (no IRQ) before sending ccb dev %d status 0x%02x\n",device,c);
		err=0xFF;
		goto AI_OUT;
	}

	output_data_shorts (device, (unsigned short *)ccb,ccblen/2); /* write command block */
 	/* ATAPI Command written wait for completition */
	udelay (5000); /* device must set bsy */

	mask = ATA_STAT_DRQ|ATA_STAT_BUSY|ATA_STAT_ERR;
	/* if no data wait for DRQ = 0 BSY = 0
	 * if data wait for DRQ = 1 BSY = 0 */
	res=0;
	if(buflen)
		res = ATA_STAT_DRQ;
	c = atapi_wait_mask(device,ATAPI_TIME_OUT,mask,res);
	if ((c & mask) != res ) {
		if (c & ATA_STAT_ERR) {
			err=(ide_inb(device,ATA_ERROR_REG))>>4;
			AT_PRINTF("atapi_issue 1 returned sense key %X status %02X\n",err,c);
		} else {
			printf ("ATTAPI_ISSUE: (no DRQ) after sending ccb (%x)  status 0x%02x\n", ccb[0],c);
			err=0xFF;
		}
		goto AI_OUT;
	}
	n=ide_inb(device, ATA_CYL_HIGH);
	n<<=8;
	n+=ide_inb(device, ATA_CYL_LOW);
	if(n>buflen) {
		printf("ERROR, transfer bytes %d requested only %d\n",n,buflen);
		err=0xff;
		goto AI_OUT;
	}
	if((n==0)&&(buflen<0)) {
		printf("ERROR, transfer bytes %d requested %d\n",n,buflen);
		err=0xff;
		goto AI_OUT;
	}
	if(n!=buflen) {
		AT_PRINTF("WARNING, transfer bytes %d not equal with requested %d\n",n,buflen);
	}
	if(n!=0) { /* data transfer */
		AT_PRINTF("ATAPI_ISSUE: %d Bytes to transfer\n",n);
		 /* we transfer shorts */
		n>>=1;
		/* ok now decide if it is an in or output */
		if ((ide_inb(device, ATA_SECT_CNT)&0x02)==0) {
			AT_PRINTF("Write to device\n");
			output_data_shorts(device,(unsigned short *)buffer,n);
		} else {
			AT_PRINTF("Read from device @ %p shorts %d\n",buffer,n);
			//input_data_shorts(device,(unsigned short *)buffer,n);
			input_data_shorts(device,(unsigned short *)buffer,n);
#ifdef IDE_DEBUG
// ** East

    int i, j;
    u8	*dbuf;
    dbuf = (u8 *)buffer;
    
    printf("Dump Buffer: buflen=%d\n", buflen);
    for(i=0x00; i<n*2 && i<512; i+=16)
    {
       printf("%04X: ", i);
       for(j=0; j<16; j++)
       {
          if ( j==7 )
             printf("%02x-",dbuf[i+j]);
          else
             printf("%02x ", dbuf[i+j]);
       }
       printf("  ");
       for(j=0; j<16; j++)
       {
          if ( dbuf[i+j]>=0x20 && dbuf[i+j]<='z' )
             printf("%c", dbuf[i+j]);
          else 
             printf(".");
       }
       printf("\n");
       if (i%256 == 0 && i!=0)
          printf("\n");
    }

#endif

		}
	}
	udelay(5000); /* seems that some CD ROMs need this... */
	mask = ATA_STAT_BUSY|ATA_STAT_ERR;
	res=0;
	c = atapi_wait_mask(device,ATAPI_TIME_OUT,mask,res);
	if ((c & ATA_STAT_ERR) == ATA_STAT_ERR) {
		err=(ide_inb(device,ATA_ERROR_REG) >> 4);
		AT_PRINTF("atapi_issue 2 returned sense key %X status %X\n",err,c);
	} else {
		err = 0;
	}
AI_OUT:
	ide_led (DEVICE_LED(device), 0);	/* LED off	*/
	return (err);
}

/*
 * sending the command to atapi_issue. If an status other than good
 * returns, an request_sense will be issued
 */

#define ATAPI_DRIVE_NOT_READY 	100
#define ATAPI_UNIT_ATTN		10

unsigned char atapi_issue_autoreq (int device,
				   unsigned char* ccb,
				   int ccblen,
				   unsigned char *buffer,
				   int buflen)
{
	unsigned char sense_data[18],sense_ccb[12];
	unsigned char res,key,asc,ascq;
	int notready,unitattn;

#ifdef CONFIG_AMIGAONEG3SE
	char *s;
	unsigned int timeout, retrycnt;

	s = getenv("ide_cd_timeout");
	timeout = s ? (simple_strtol(s, NULL, 10)*1000000)/5 : 0;

	retrycnt = 0;
#endif

	unitattn=ATAPI_UNIT_ATTN;
	notready=ATAPI_DRIVE_NOT_READY;

retry:
	res= atapi_issue(device,ccb,ccblen,buffer,buflen);
	if (res==0)
		return (0); /* Ok */

	if (res==0xFF)
		return (0xFF); /* error */

	AT_PRINTF("(auto_req)atapi_issue returned sense key %X\n",res);

	memset(sense_ccb,0,sizeof(sense_ccb));
	memset(sense_data,0,sizeof(sense_data));
	sense_ccb[0]=ATAPI_CMD_REQ_SENSE;
	sense_ccb[4]=18; /* allocation Length */

	res=atapi_issue(device,sense_ccb,12,sense_data,18);
	key=(sense_data[2]&0xF);
	asc=(sense_data[12]);
	ascq=(sense_data[13]);

	AT_PRINTF("ATAPI_CMD_REQ_SENSE returned %x\n",res);
	AT_PRINTF(" Sense page: %02X key %02X ASC %02X ASCQ %02X\n",
		sense_data[0],
		key,
		asc,
		ascq);

	if((key==0))
		return 0; /* ok device ready */

	if((key==6)|| (asc==0x29) || (asc==0x28)) { /* Unit Attention */
		if(unitattn-->0) {
			udelay(200*1000);
			goto retry;
		}
		printf("Unit Attention, tried %d\n",ATAPI_UNIT_ATTN);
		goto error;
	}
	if((asc==0x4) && (ascq==0x1)) { /* not ready, but will be ready soon */
		if (notready-->0) {
			udelay(200*1000);
			goto retry;
		}
		printf("Drive not ready, tried %d times\n",ATAPI_DRIVE_NOT_READY);
		goto error;
	}
	if(asc==0x3a) {
		AT_PRINTF("Media not present\n");
		goto error;
	}

#ifdef CONFIG_AMIGAONEG3SE
	if ((sense_data[2]&0xF)==0x0B) {
		AT_PRINTF("ABORTED COMMAND...retry\n");
		if (retrycnt++ < 4)
			goto retry;
		return (0xFF);
	}

	if ((sense_data[2]&0xf) == 0x02 &&
	    sense_data[12] == 0x04	&&
	    sense_data[13] == 0x01	) {
		AT_PRINTF("Waiting for unit to become active\n");
		udelay(timeout);
		if (retrycnt++ < 4)
			goto retry;
		return 0xFF;
	}
#endif	/* CONFIG_AMIGAONEG3SE */

	printf ("ERROR: Unknown Sense key %02X ASC %02X ASCQ %02X\n",key,asc,ascq);
error:
	AT_PRINTF ("ERROR Sense key %02X ASC %02X ASCQ %02X\n",key,asc,ascq);
	return (0xFF);
}


static void	atapi_inquiry(block_dev_desc_t * dev_desc)
{
	unsigned char ccb[12]; /* Command descriptor block */
	unsigned char iobuf[64]; /* temp buf */
	unsigned char c;
	int device;

	device=dev_desc->dev;
	dev_desc->type=DEV_TYPE_UNKNOWN; /* not yet valid */
	dev_desc->block_read=atapi_read;

	memset(ccb,0,sizeof(ccb));
	memset(iobuf,0,sizeof(iobuf));

	ccb[0]=ATAPI_CMD_INQUIRY;
	ccb[4]=40; /* allocation Legnth */
	c=atapi_issue_autoreq(device,ccb,12,(unsigned char *)iobuf,40);

	AT_PRINTF("ATAPI_CMD_INQUIRY returned %x\n",c);
	if (c!=0)
		return;

	/* copy device ident strings */
	ident_cpy(dev_desc->vendor,&iobuf[8],8);
	ident_cpy(dev_desc->product,&iobuf[16],16);
	ident_cpy(dev_desc->revision,&iobuf[32],5);

	dev_desc->lun=0;
	dev_desc->lba=0;
	dev_desc->blksz=0;
	dev_desc->type=iobuf[0] & 0x1f;

	if ((iobuf[1]&0x80)==0x80)
		dev_desc->removable = 1;
	else
		dev_desc->removable = 0;

	memset(ccb,0,sizeof(ccb));
	memset(iobuf,0,sizeof(iobuf));
	ccb[0]=ATAPI_CMD_START_STOP;
	ccb[4]=0x03; /* start */

	c=atapi_issue_autoreq(device,ccb,12,(unsigned char *)iobuf,0);

	AT_PRINTF("ATAPI_CMD_START_STOP returned %x\n",c);
	if (c!=0)
		return;

	memset(ccb,0,sizeof(ccb));
	memset(iobuf,0,sizeof(iobuf));
	c=atapi_issue_autoreq(device,ccb,12,(unsigned char *)iobuf,0);

	AT_PRINTF("ATAPI_CMD_UNIT_TEST_READY returned %x\n",c);
	if (c!=0)
		return;

	memset(ccb,0,sizeof(ccb));
	memset(iobuf,0,sizeof(iobuf));
	ccb[0]=ATAPI_CMD_READ_CAP;
	c=atapi_issue_autoreq(device,ccb,12,(unsigned char *)iobuf,8);
	AT_PRINTF("ATAPI_CMD_READ_CAP returned %x\n",c);
	if (c!=0)
		return;

	AT_PRINTF("Read Cap: LBA %02X%02X%02X%02X blksize %02X%02X%02X%02X\n",
		iobuf[0],iobuf[1],iobuf[2],iobuf[3],
		iobuf[4],iobuf[5],iobuf[6],iobuf[7]);

	dev_desc->lba  =((unsigned long)iobuf[0]<<24) +
			((unsigned long)iobuf[1]<<16) +
			((unsigned long)iobuf[2]<< 8) +
			((unsigned long)iobuf[3]);
	dev_desc->blksz=((unsigned long)iobuf[4]<<24) +
			((unsigned long)iobuf[5]<<16) +
			((unsigned long)iobuf[6]<< 8) +
			((unsigned long)iobuf[7]);
#ifdef CONFIG_LBA48
	dev_desc->lba48 = 0; /* ATAPI devices cannot use 48bit addressing (ATA/ATAPI v7) */
#endif
	return;
}


/*
 * atapi_read:
 * we transfer only one block per command, since the multiple DRQ per
 * command is not yet implemented
 */
#define ATAPI_READ_MAX_BYTES	2048	/* we read max 2kbytes */
#define ATAPI_READ_BLOCK_SIZE	2048	/* assuming CD part */
#define ATAPI_READ_MAX_BLOCK ATAPI_READ_MAX_BYTES/ATAPI_READ_BLOCK_SIZE	/* max blocks */

ulong atapi_read (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer)
{
	ulong n = 0;
	unsigned char ccb[12]; /* Command descriptor block */
	ulong cnt;

	AT_PRINTF("atapi_read dev %d start %lX, blocks %lX buffer at %lX\n",
		device, blknr, blkcnt, (ulong)buffer);

	do {
		if (blkcnt>ATAPI_READ_MAX_BLOCK) {
			cnt=ATAPI_READ_MAX_BLOCK;
		} else {
			cnt=blkcnt;
		}
		ccb[0]=ATAPI_CMD_READ_12;
		ccb[1]=0; /* reserved */
		ccb[2]=(unsigned char) (blknr>>24) & 0xFF; /* MSB Block */
		ccb[3]=(unsigned char) (blknr>>16) & 0xFF; /*  */
		ccb[4]=(unsigned char) (blknr>> 8) & 0xFF;
		ccb[5]=(unsigned char)  blknr      & 0xFF; /* LSB Block */
		ccb[6]=(unsigned char) (cnt  >>24) & 0xFF; /* MSB Block count */
		ccb[7]=(unsigned char) (cnt  >>16) & 0xFF;
		ccb[8]=(unsigned char) (cnt  >> 8) & 0xFF;
		ccb[9]=(unsigned char)  cnt	   & 0xFF; /* LSB Block */
		ccb[10]=0; /* reserved */
		ccb[11]=0; /* reserved */

		if (atapi_issue_autoreq(device,ccb,12,
					(unsigned char *)buffer,
					cnt*ATAPI_READ_BLOCK_SIZE) == 0xFF) {
			return (n);
		}
		n+=cnt;
		blkcnt-=cnt;
		blknr+=cnt;
		buffer+=cnt*(ATAPI_READ_BLOCK_SIZE/4); /* ulong blocksize in ulong */
	} while (blkcnt > 0);
	return (n);
}

/* ------------------------------------------------------------------------- */

#endif /* CONFIG_ATAPI */

U_BOOT_CMD(
	ide,  5,  1,  do_ide,
	"ide     - IDE sub-system\n",
	"reset - reset IDE controller\n"
	"ide info  - show available IDE devices\n"
	"ide device [dev] - show or set current device\n"
	"ide part [dev] - print partition table of one or all IDE devices\n"
	"ide read  addr blk# cnt\n"
	"ide write addr blk# cnt - read/write `cnt'"
	" blocks starting at block `blk#'\n"
	"    to/from memory address `addr'\n"
);

#endif	/* CONFIG_COMMANDS & CFG_CMD_IDE */
