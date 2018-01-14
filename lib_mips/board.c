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

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <devices.h>
#include <net.h>
#include "cvmx-bootloader.h"
#include <environment.h>
#include "octeon_boot.h"
#include "octeon-boot-info.h"
#include <lib_octeon.h>
#include <octeon_eeprom_types.h>
#include <lib_octeon_shared.h>
#include <flash.h>
#include "octeon-pci-console.h"
#include "cvmx-bootmem.h"
#include "octeon-model.h"
#include "cvmx-l2c.h"
#include "cvmx-core.h"
#include "nand.h"


#if (!defined(ENV_IS_EMBEDDED) ||  defined(CFG_ENV_IS_IN_NVRAM))
#define	TOTAL_MALLOC_LEN	(CFG_MALLOC_LEN + CFG_ENV_SIZE)
#else
#define	TOTAL_MALLOC_LEN	CFG_MALLOC_LEN
#endif


#define DRAM_LATE_ZERO_OFFSET  0x100000ull
#if 0
#define DEBUG
#endif

#ifdef DEBUG
#define DEBUG_PRINTF   printf
#else
#define DEBUG_PRINTF(...);
#endif

#ifdef CFG_PRINT_MPR
extern void print_mpr(void);
#else
void print_mpr(void)
{
}
#endif

/* Global variable use to locate pci console descriptor */
uint64_t pci_cons_desc_addr;

extern int timer_init(void);

extern int incaip_set_cpuclk(void);

extern char uboot_end_data;
extern char uboot_end;
extern char uboot_start;

ulong monitor_flash_len;
#define SET_K0(val)                 asm volatile ("add $26, $0, %[rt]" : : [rt] "d" (val):"memory")

extern const char version_string[];

static char *failed = "*** failed ***\n";

/*
 * Begin and End of memory area for malloc(), and current "brk"
 */
static uint32_t mem_malloc_start;
static uint32_t mem_malloc_end;
static uint32_t mem_malloc_brk;

/* Used for error checking on load commands */
ulong load_reserved_addr = 0; /* Base address of reserved memory for loading */
ulong load_reserved_size = 0; /* Size of reserved memory for loading */

#define STACK_SIZE  (16*1024UL + CFG_BOOTPARAMS_LEN)
/*
 * The Malloc area is immediately below the monitor copy in DRAM
 */

#if !CONFIG_OCTEON_UBOOT_TLB
#define MALLOC_IN_BOOTMEM
#endif


#ifndef MALLOC_IN_BOOTMEM
static void mem_malloc_init (void)
{
#ifndef CONFIG_OCTEON_UBOOT_TLB
	DECLARE_GLOBAL_DATA_PTR;

	ulong dest_addr = CFG_MONITOR_BASE + gd->reloc_off;

	mem_malloc_end = dest_addr;
	mem_malloc_start = dest_addr - TOTAL_MALLOC_LEN;
	mem_malloc_brk = mem_malloc_start;
#else

        DEBUG_PRINTF("start: %p, end: %p, len: %d (0x%x)\n", &uboot_start, &uboot_end, 
                     (char *)&uboot_end - (char *)&uboot_start, (char *)&uboot_end - (char *)&uboot_start);
        /* For mapped case, heap comes right after code/data */
	mem_malloc_start = ((uint32_t)&uboot_end + 0xFFFF) & ~0xFFFF;  /* Do same rounding as during allocation */
	mem_malloc_end = mem_malloc_start + TOTAL_MALLOC_LEN;
	mem_malloc_brk = mem_malloc_start;
        DEBUG_PRINTF("Malloc start: 0x%x, end: 0x%x, len: 0x%x\n", mem_malloc_start, mem_malloc_end, TOTAL_MALLOC_LEN);


#endif

#if !CONFIG_OCTEON_SIM_SPEED
	memset ((void *) mem_malloc_start,
		0,
		mem_malloc_end - mem_malloc_start);
#endif
}
#else
/* Put the malloc area for u-boot in a bootmem block to free up memory below 1 Mbyte.
** Note that this is freed when the application(s) start, so no structure that applications
** are supposed to read can be put into this region.
!!! This is not supported by TLB mapped u-boot builds.
*/
static void mem_malloc_init (void)
{
    int64_t addr;
    addr = cvmx_bootmem_phy_named_block_alloc(TOTAL_MALLOC_LEN, 0, 0x7fffffff, 128, "__tmp_bootloader_internal", CVMX_BOOTMEM_FLAG_END_ALLOC);
    if (addr < 0)
    {
        printf("ERROR: Unable to allocate memory for heap.... (1)\n");
        return;
    }

    mem_malloc_start = addr;
    DEBUG_PRINTF("Allocated malloc area from bootmem at address 0x%qx (0x%08x)\n", addr, mem_malloc_start);
    mem_malloc_end = mem_malloc_start + TOTAL_MALLOC_LEN;
    mem_malloc_brk = mem_malloc_start;

#if !CONFIG_OCTEON_SIM_SPEED
    memset (cvmx_phys_to_ptr(mem_malloc_start),
            0,
            mem_malloc_end - mem_malloc_start);
#endif
}
#endif

/* Check for incompatable options */
#if defined(CONFIG_OCTEON_UBOOT_TLB) && defined(MALLOC_IN_BOOTMEM)
#error U-Boot heap in bootmem is not supported in TLB mapped mode.
#endif
void *sbrk (ptrdiff_t increment)
{
	ulong old = mem_malloc_brk;
	ulong new = old + increment;

	if ((new < mem_malloc_start) || (new > mem_malloc_end)) {
                printf("ERROR: malloc heap exhausted!\n");
		return (NULL);
	}
	mem_malloc_brk = new;
	return ((void *) old);
}


static int init_func_ram (void)
{
    DECLARE_GLOBAL_DATA_PTR;

#if CONFIG_L2_ONLY
    gd->ram_size = 3*1024*1024;
#endif 
    puts ("DRAM:  ");
    if ((gd->ram_size) > 0) {
        print_size (gd->ram_size, "\n");
        return (0);
    }

    puts (failed);
    return (1);
}

static int display_banner(void)
{

	printf ("\n\n%s\n\n", version_string);
	return (0);
}
static int display_board_info(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    char buffer[32];
    printf("%s r1:%d, r2:%d, f:%d/%d, serial #: %s\n",
           cvmx_board_type_to_string(gd->board_desc.board_type),
           gd->board_desc.rev_major, gd->board_desc.rev_minor,
           CFG_FLASH_SIZE_MB, CFG_MAX_FLASH_SECT,
           gd->board_desc.serial_str);
    print_mpr();

    printf("Core clock: %d MHz, ",
           divide_nint(cvmx_clock_get_rate(CVMX_CLOCK_CORE), 1000000));

    if (OCTEON_IS_MODEL(OCTEON_CN6XXX))
        printf("IO clock: %d MHz, ", divide_nint(cvmx_clock_get_rate(CVMX_CLOCK_SCLK), 1000000));

#if 1
    printf("DDR clock: %d MHz (%d Mhz data rate)\n", gd->ddr_clock_mhz, gd->ddr_clock_mhz *2);
#else
    /* FIXME: This should be computed with hertz and proper rounding as follows.
       The change would have to ripple through gd_t and eeprom_tuples. */
    printf("DDR clock: %d MHz (%d Mhz data rate)\n",
           divide_nint(gd->ddr_clock_hertz, 1000000), divide_nint(gd->ddr_clock_hertz*2, 1000000));
#endif

    return (0);
}

#ifndef CFG_NO_FLASH
static void display_flash_config(ulong size)
{
	puts ("Flash: ");
	print_size (size, "\n");
}
#endif

int init_dram(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    int mem_mbytes = 0;
    uint32_t measured_ddr_hertz;
    uint32_t ddr_config_valid_mask;
    char *eptr;
    uint32_t ddr_hertz;
    uint32_t ddr_ref_hertz;
    int ram_resident = 0;
    uint32_t cpu_id = cvmx_get_proc_id();
    const ddr_configuration_t *ddr_config_ptr;



#if CONFIG_OCTEON_SIM_NO_DDR
#if CONFIG_OCTEON_NAND_STAGE2
    /* For simulating NAND boot, we don't use normal oct-sim helper, so we don't get the DRAM
    ** size in the special location */
    gd->ram_size = 128*1024*1024;
#else
    gd->ram_size = (uint64_t)(*((uint32_t *)0x9ffffff8) * 1024 * 1024);
#endif
    return (0);
#endif

    if (gd->flags & GD_FLG_RAM_RESIDENT)
        ram_resident = 1;

    if (ram_resident && getenv("dram_size_mbytes"))
    {
        /* DRAM size has been based by the remote entity that configured
        ** the DRAM controller, so we use that size rather than re-computing
        ** by reading the DIMM SPDs. */
        mem_mbytes = simple_strtoul(getenv("dram_size_mbytes"), NULL, 0);
        printf("Using DRAM size from environment: %d MBytes\n", mem_mbytes);
        printf("%s r1:%d, r2:%d, f:%d/%d, serial #: %s\n",
               cvmx_board_type_to_string(gd->board_desc.board_type),
               gd->board_desc.rev_major, gd->board_desc.rev_minor,
               CFG_FLASH_SIZE_MB, CFG_MAX_FLASH_SECT,
               gd->board_desc.serial_str);

        int ddr_int = 0;
        if (OCTEON_IS_MODEL(OCTEON_CN56XX))
        {
            /* Check to see if interface 0 is enabled */
            cvmx_l2c_cfg_t l2c_cfg;
            l2c_cfg.u64 = cvmx_read_csr(CVMX_L2C_CFG);
            if (!l2c_cfg.s.dpres0)
            {
                if (l2c_cfg.s.dpres1)
                    ddr_int = 1;
                else
                    printf("ERROR: no DRAM controllers initialized\n");
            }
        }
        gd->ddr_clock_mhz = divide_nint(measure_octeon_ddr_clock(cpu_id,NULL,gd->cpu_clock_mhz*1000*1000,0,0,ddr_int,0), 1000000);
    }
    else
    {
        ddr_config_ptr = lookup_ddr_config_structure(cpu_id, gd->board_desc.board_type, gd->board_desc.rev_major, gd->board_desc.rev_minor, &ddr_config_valid_mask);
        if (!ddr_config_ptr)
        {
            printf("ERROR: unable to determine DDR configuration for board type: %s (%d)\n", cvmx_board_type_to_string(gd->board_desc.board_type), gd->board_desc.board_type);
            return(-1);
        }

        /* Check for special case of mismarked 3005 samples, and adjust cpuid */
        if (cpu_id == OCTEON_CN3010_PASS1 && (cvmx_read_csr(CVMX_L2D_FUS3) & (1ull << 34)))
            cpu_id |= 0x10;
    
        ddr_hertz = gd->ddr_clock_mhz * 1000000;
        if ((eptr = getenv("ddr_clock_hertz")) != NULL) {
            ddr_hertz = simple_strtoul(eptr, NULL, 0);
            gd->ddr_clock_mhz = divide_nint(ddr_hertz, 1000000);
            printf("Parameter found in environment. ddr_clock_hertz = %d\n", ddr_hertz);
        }
    
        ddr_ref_hertz = gd->ddr_ref_hertz;
        if ((eptr = getenv("ddr_ref_hertz")) != NULL) {
            ddr_ref_hertz = simple_strtoul(eptr, NULL, 0);
            gd->ddr_ref_hertz = ddr_ref_hertz;
            printf("Parameter found in environment. ddr_ref_hertz = %d\n", ddr_ref_hertz);
        }
    
    
        mem_mbytes = octeon_ddr_initialize(cpu_id,
                                           cvmx_clock_get_rate(CVMX_CLOCK_SCLK),
                                           ddr_hertz,
                                           ddr_ref_hertz,
                                           ddr_config_valid_mask,
                                           ddr_config_ptr,
                                           &measured_ddr_hertz,
                                           gd->board_desc.board_type,
                                           gd->board_desc.rev_major,
                                           gd->board_desc.rev_minor);
    
    
    
   
        gd->ddr_clock_mhz = divide_nint(measured_ddr_hertz, 1000000);
    
        DEBUG_PRINTF("Measured DDR clock %d Hz\n", measured_ddr_hertz);

        if (measured_ddr_hertz != 0)
        {
            if (!gd->ddr_clock_mhz)
            {
                /* If ddr_clock not set, use measured clock and don't warn */
                gd->ddr_clock_mhz = divide_nint(measured_ddr_hertz, 1000000);
            }
            else if ((measured_ddr_hertz > ddr_hertz + 3000000) || (measured_ddr_hertz <  ddr_hertz - 3000000))
            {
                printf("\nWARNING:\n");
                printf("WARNING: Measured DDR clock mismatch! expected: %d MHz, measured: %d MHz, cpu clock: %d MHz\n",
                       divide_nint(ddr_hertz, 1000000), divide_nint(measured_ddr_hertz, 1000000), gd->cpu_clock_mhz);
                if (!(gd->flags & GD_FLG_RAM_RESIDENT))
                    printf("WARNING: Using measured clock for configuration.\n");
                printf("WARNING:\n\n");
                gd->ddr_clock_mhz = divide_nint(measured_ddr_hertz, 1000000);
            }
        }
    
    
        if (gd->flags & GD_FLG_CLOCK_DESC_MISSING)
        {
            printf("Warning: Clock descriptor tuple not found in eeprom, using defaults\n");
        }
        if (gd->flags & GD_FLG_BOARD_DESC_MISSING)
        {
            printf("Warning: Board descriptor tuple not found in eeprom, using defaults\n");
        }


    }



    if (mem_mbytes > 0)
    {
#if CFG_ENV_IS_IN_RAM
        uint8_t tmp_buf[CFG_ENV_SIZE];
#endif
        if (getenv("dram_size_mbytes"))
        {
            /* Override the actual DRAM size. */
            mem_mbytes = simple_strtoul(getenv("dram_size_mbytes"), NULL, 0);
            printf("!!! Overriding DRAM size: dram_size_mbytes = %d MBytes !!!\n", mem_mbytes);
        }
        gd->ram_size = (uint64_t)mem_mbytes * 1024 * 1024;

        cvmx_l2c_set_big_size((uint64_t)mem_mbytes, 0);

#if !defined(CONFIG_NO_RELOCATION) /* If already running in memory don't zero it. */
    /* Zero the low Megabyte of DRAM used by bootloader.  The rest is zeroed later.
    ** when running from DRAM*/

#if CFG_ENV_IS_IN_RAM
        memcpy(tmp_buf, (char *)U_BOOT_RAM_ENV_ADDR, CFG_ENV_SIZE);
#endif
#if !CONFIG_NO_CLEAR_DDR
        if (!(gd->flags & GD_FLG_MEMORY_PRESERVED))
            octeon_bzero64_pfs(0, DRAM_LATE_ZERO_OFFSET);
#endif
#if CFG_ENV_IS_IN_RAM
        memcpy((char *)U_BOOT_RAM_ENV_ADDR, tmp_buf, CFG_ENV_SIZE);
#endif

#endif /* !CONFIG_NO_RELOCATION */
        return 0;
    }
    else
        return -1;
}


static int init_baudrate (void)
{
	DECLARE_GLOBAL_DATA_PTR;

	char tmp[64];	/* long enough for environment variables */
	int i = getenv_r ("baudrate", tmp, sizeof (tmp));

	gd->baudrate = (i > 0)
			? (int) simple_strtoul (tmp, NULL, 10)
			: CONFIG_BAUDRATE;

	return (0);
}

#define DO_SIMPLE_DRAM_TEST_FROM_FLASH  0
#if DO_SIMPLE_DRAM_TEST_FROM_FLASH
/* Very simple DRAM test to run from flash for cases when bootloader boot
** can't complete.
*/
static int dram_test(void)
{
    uint32_t *addr;
    uint32_t *start = (void *)0x80400000;
    uint32_t *end =   (void *)0x81400000;
    uint32_t val, incr, readback, pattern;
    int error_limit;
    int i;

    pattern = 0;
    printf("Performing simple DDR test from flash.\n");

    #define DRAM_TEST_ERROR_LIMIT 200

    incr = 0x4321;
    for (i = 0;i < 10;i++) {
        error_limit = DRAM_TEST_ERROR_LIMIT;

        printf ("\rPattern %08lX  Writing..."
                "%12s"
                "\b\b\b\b\b\b\b\b\b\b",
                pattern, "");

        for (addr=start,val=pattern; addr<end; addr++) {
                *addr = val;
                val  += incr;
        }

        puts ("Reading...");

        for (addr=start,val=pattern; addr<end; addr++) {
                readback = *addr;
                if (readback != val && error_limit-- > 0) {
                    if (error_limit + 1 == DRAM_TEST_ERROR_LIMIT)
                        puts("\n");
                    printf ("Mem error @ 0x%08X: "
                            "found %08lX, expected %08lX, xor %08lX\n",
                            (uint)addr, readback, val, readback^val);
                }
                val += incr;
        }

        /*
         * Flip the pattern each time to make lots of zeros and
         * then, the next time, lots of ones.  We decrement
         * the "negative" patterns and increment the "positive"
         * patterns to preserve this feature.
         */
        if(pattern & 0x80000000) {
                pattern = -pattern;	/* complement & increment */
        }
        else {
                pattern = ~pattern;
        }
        if (error_limit <= 0)
        {
            printf("Too many errors, printing stopped\n");
        }
        incr = -incr;
    }

    puts("\n");
    return(0);

}
#endif

int init_dfm(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    int mem_mbytes = 0;

    mem_mbytes = octeon_dfm_initialize();

    if (mem_mbytes > 0) {
        gd->dfm_ram_size = (uint64_t)mem_mbytes * 1024 * 1024;
        return 0;
    } else {
        return -1;
    }
}

/* Clears DRAM, while taking care to not overwrite DRAM already in use
** by u-boot */
void octeon_clear_mem_pfs(uint64_t base_addr, uint64_t size)
{
    DECLARE_GLOBAL_DATA_PTR;

    uint64_t ub_base = gd->bd->bi_uboot_ram_addr;
    uint64_t ub_size = gd->bd->bi_uboot_ram_used_size;

    if (ub_base >= base_addr && ub_base < (base_addr + size))
    {
        /* We need to avoid overwriting the u-boot section */
        octeon_bzero64_pfs(MAKE_XKPHYS(base_addr), ub_base - base_addr);
        octeon_bzero64_pfs(MAKE_XKPHYS(ub_base + ub_size), size - (ub_base - base_addr) - (ub_size));
    }
    else
    {
        /* just clear it */
        octeon_bzero64_pfs(MAKE_XKPHYS(base_addr), size);
    }

}
/*
 * Breath some life into the board...
 *
 * The first part of initialization is running from Flash memory;
 * its main purpose is to initialize the RAM so that we
 * can relocate the monitor code to RAM.
 */

/*
 * All attempts to come up with a "common" initialization sequence
 * that works for all boards and architectures failed: some of the
 * requirements are just _too_ different. To get rid of the resulting
 * mess of board dependend #ifdef'ed code we now make the whole
 * initialization sequence configurable to the user.
 *
 * The requirements for any new initalization function is simple: it
 * receives a pointer to the "global data" structure as it's only
 * argument, and returns an integer return code, where 0 means
 * "continue" and != 0 means "fatal error, hang the system".
 */

int early_board_init(void);
int octeon_boot_bus_init_board(void);
int octeon_boot_bus_moveable_init(void);
int failsafe_scan(void);
int octeon_bist(void);
#if CFG_LATE_BOARD_INIT
int late_board_init(void);
#endif

typedef int (init_fnc_t) (void);

init_fnc_t *init_sequence[] = {

#if CONFIG_OCTEON_UBOOT_TLB && !CONFIG_OCTEON_SIM_NO_FLASH && !CONFIG_OCTEON_NAND_STAGE2
    failsafe_scan,
#endif
#if CONFIG_OCTEON && (!CONFIG_OCTEON_SIM_NO_FLASH)
#if CONFIG_BOOT_BUS_BOARD
    octeon_boot_bus_init_board,  // board specific boot bus init
#else
    octeon_boot_bus_init,  // need to map PAL to read clocks, speed up flash
#endif
#endif
        octeon_boot_bus_moveable_init,
    	timer_init,
#if CONFIG_OCTEON
    early_board_init,
#endif
        env_init,		/* initialize environment (Must be after early_board_init for NAND) */
	init_baudrate,		/* initialze baudrate settings */
	serial_init,		/* serial communications setup */
	console_init_f,
	display_banner,		/* say that we are here */
#if (!CONFIG_OCTEON_SIM_HW_DIFF)
        octeon_bist,                /* Do early so we can print results even if DRAM doesn't work */
#endif
#if !CONFIG_L2_ONLY
        init_dram,
#endif
#if DO_SIMPLE_DRAM_TEST_FROM_FLASH
    dram_test,
#endif
#if OCTEON_CONFIG_DFM
    init_dfm,
#endif
	checkboard,
    display_board_info,
	init_func_ram,
	NULL,
};


void *uboot_phys_to_kseg0_ptr(uint64_t addr)
{
    return((void *)MAKE_KSEG0((uint32_t)addr));
}
void board_init_f(ulong bootflag)
{
	DECLARE_GLOBAL_DATA_PTR;

        uint64_t new_gd_addr;
        gd_t gd_data;
	bd_t bd;  /* Local copy on stack, which we will copy to DRAM */
	init_fnc_t **init_fnc_ptr;
	ulong len = (ulong)&uboot_end - CFG_MONITOR_BASE;
        uint64_t addr;
	uint64_t u_boot_mem_top; /* top of memory, as used by u-boot */
#ifdef CONFIG_PURPLE
	void copy_code (ulong);
#endif
        int init_func_count = 0;
        uint64_t __attribute__((unused)) heap_base_addr;
        int __attribute__((unused)) map_size;

        /* Physical base address of u-boot code */
        uint64_t uboot_base_phys;
        uint64_t addr_sp_phys;
        uint64_t addr_bd_phys;  /* Address of board data structure in RAM */

        /* Offsets from u-boot base for stack/heap.  These
        ** offsets will be the same for both physical and
        ** virtual maps. */

        SET_K0(&gd_data);
        /* Pointer is writable since we allocated a register for it.  */
	memset ((void *)gd, 0, sizeof (gd_t));

        /* Round u-boot length up to nice boundary */

        len = (len + 0xFFFF) & ~0xFFFF;

#ifdef CONFIG_OCTEON_EBB6300
        /* 63XX_TODO.  Signal to MCU that we are running */
        twsii_mcu_read(2);twsii_mcu_read(3);
#endif
        /* Enable soft BIST so that BIST is run on soft resets. */
        if (!OCTEON_IS_MODEL(OCTEON_CN38XX_PASS2) && !OCTEON_IS_MODEL(OCTEON_CN31XX))
            cvmx_write_csr(CVMX_CIU_SOFT_BIST, 1);

        /* Change default setting for CN58XX pass 2 */
        if (OCTEON_IS_MODEL(OCTEON_CN58XX_PASS2))
            cvmx_write_csr(CVMX_MIO_FUS_EMA, 2);

#ifdef ENABLE_BOARD_DEBUG
        int gpio_status_data = 1;
#endif
	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		init_fnc_t* fnc = (init_fnc_t *)((uint32_t)(*init_fnc_ptr));
#ifdef ENABLE_BOARD_DEBUG
                gpio_status(gpio_status_data++);
#endif
		if ((*fnc)() != 0) {
			printf("hanging, init func: %d\n", init_func_count);
#ifdef CONFIG_OCTEON_EBB6300
// 63XX_HACK  sometimes dimms not detected....
                        printf("AUTO resetting board do to init function failing!\n");
                        do_reset (NULL, 0, 0, NULL);
#endif
			hang ();
		}
                init_func_count++;
	}

        /* Disable Fetch under fill on CN63XXp1 due to errata Core-14317 */
        if (OCTEON_IS_MODEL(OCTEON_CN63XX_PASS1_X))
        {
            uint64_t val;
            val = get_cop0_cvmctl_reg();
            val |= (1ull << 19); /*CvmCtl[DEFET] */
            set_cop0_cvmctl_reg(val);
        }

	/*
	 * Now that we have DRAM mapped and working, we can
	 * relocate the code and continue running from DRAM.
	 */
#if defined(CONFIG_NO_RELOCATION)
        /* If loaded into ram, we can skip relocation , and run from the spot we were loaded into */
	addr = CFG_MONITOR_BASE;
	u_boot_mem_top = CFG_SDRAM_BASE + MIN(gd->ram_size, (1*1024*1024));
#else


#ifdef CONFIG_OCTEON_UBOOT_TLB
        /* Put u-boot at the highest address available in DRAM.
        ** For 3XXX/5XXX chips, this will likely be in the range 0x410000000-0x41fffffff
        */
        if (gd->ram_size <= 256*1024*1024)
        {
            addr = MIN(gd->ram_size, (256*1024*1024));
            /* We need to leave some room for the named block structure that will
            ** be allocated when the bootmem allocator is initialized. */
            addr -= CVMX_BOOTMEM_NUM_NAMED_BLOCKS * sizeof(cvmx_bootmem_named_block_desc_t) + 128;
        }
        else
        {
            if (octeon_is_model(OCTEON_CN3XXX) || octeon_is_model(OCTEON_CN5XXX))
            {
                uint64_t offset = MIN(256*1024*1024, gd->ram_size - 256*1024*1024);
                addr = 0x410000000ULL + offset;
            }
            else
            {
                /* CN63XX and newer */
                uint64_t offset = gd->ram_size - 256*1024*1024;
                addr = 0x20000000ULL + offset;
            }
        }
#else
        /* Locate at top of first Megabyte */
        addr = CFG_SDRAM_BASE + 0x100000;
#endif

	u_boot_mem_top = addr;

        /* Print out any saved error messages that were logged during startup before
        ** the serial port was enabled.  This only works if the error message is a string
        ** constant in flash, but most things early on will be. */
        if (gd->err_msg)
            printf("Error message from before serial ready: %s\n", gd->err_msg);

		/* We can reserve some RAM "on top" here.
		 */

        DEBUG_PRINTF("U-boot link addr: 0x%x\n", CFG_MONITOR_BASE);
		/* round down to next 4 kB limit.
		 */
	addr &= ~(4096 - 1);

	DEBUG_PRINTF ("Top of RAM usable for U-Boot at: %08qx\n", addr);



		/* Reserve memory for U-Boot code, data & bss
		 */
	addr -= MAX(len, (512*1024));

#ifdef CONFIG_OCTEON_UBOOT_TLB
        /* In TLB mapped case, stack/heap go above (higher address) u-boot in DRAM,
        ** as the base address of u-boot has alignment constraints due to the TLB usage.
        ** Reserve space for them here (before u-boot base address is determined */
        addr -= STACK_SIZE;
        addr -= TOTAL_MALLOC_LEN;
#endif

#ifndef CONFIG_OCTEON_UBOOT_TLB
    /* round down to next 64k (allow us to create image at same addr for debugging)*/
	addr &= ~(64 * 1024 - 1);
#else
        /* Determine alignment required to use a single TLB entry, and
        ** round the DRAM address down to that alignment.   The minimum
        ** alignment is 4 MBytes, as this is required for PCI BAR mappings. */
        map_size = 1;
        while (map_size < u_boot_mem_top - addr)
            map_size = map_size << 1;
        map_size = map_size >> 1;  /* TLB page size is 1/2 of mapping alignment */

        DEBUG_PRINTF("Using TLB mapping size of 0x%x bytes for DRAM mapping.\n", map_size);
	addr &= ~(map_size - 1);
#endif

        uboot_base_phys = addr;

	DEBUG_PRINTF ("Reserving %ldk for U-Boot at: %08qx\n", MAX(len, (512*1024)) >> 10, uboot_base_phys);
#if !CONFIG_OCTEON_SIM_SPEED
        DEBUG_PRINTF("Clearing 0x%x bytes at addr 0x%qx\n", (uint32_t)(u_boot_mem_top - uboot_base_phys), uboot_base_phys);
        octeon_bzero64_pfs(MAKE_XKPHYS(uboot_base_phys), (u_boot_mem_top - uboot_base_phys + OCTEON_BZERO_PFS_STRIDE_MASK) & ~OCTEON_BZERO_PFS_STRIDE_MASK);
#endif

#endif  // CONFIG_NO_RELOCATION


#ifndef MALLOC_IN_BOOTMEM
	 	/* Reserve memory for malloc() arena, at lower address than u-boot code/data 
		 */

#ifdef CONFIG_OCTEON_UBOOT_TLB
        /* Put heap after code/data when using TLB */
        heap_base_addr = uboot_base_phys + len;
        DEBUG_PRINTF("uboot_base_phys: 0x%qx, len: 0x%x\n", uboot_base_phys, len);
        /* For TLB case, we want the stack to be TLB mapped.
        ** place it after the u-boot code/data/heap */
        addr_sp_phys = heap_base_addr + TOTAL_MALLOC_LEN + STACK_SIZE;
#else
        heap_base_addr = addr - TOTAL_MALLOC_LEN;
        addr_sp_phys = heap_base_addr;
#endif

	DEBUG_PRINTF ("Reserving %dk for malloc() at: %08qx\n",
			TOTAL_MALLOC_LEN >> 10, heap_base_addr);

#else
        addr_sp_phys = addr;

        DEBUG_PRINTF("No memory reserved for malloc(), heap in bootmem\n");
#endif


        DEBUG_PRINTF("Stack top phys: 0x%qx\n", addr_sp_phys);

	/*
	 * (permanently) allocate a Board Info struct
	 * and a permanent copy of the "global" data
	 */
	addr_sp_phys -= sizeof(bd_t);
        addr_bd_phys = addr_sp_phys;
#ifndef CONFIG_OCTEON_UBOOT_TLB
	gd->bd = (bd_t *)uboot_phys_to_kseg0_ptr(addr_bd_phys);
#else
        /* Make sure this a virtual pointer.  We know that this will always 
        ** be a 32 bit (virtual) address) */
	gd->bd = (bd_t *)(uint32_t)(CFG_MONITOR_BASE + (addr_bd_phys - uboot_base_phys));
#endif
	DEBUG_PRINTF ("Reserving %d Bytes for Board Info at: %08qx\n",
			sizeof(bd_t), addr_sp_phys);
	addr_sp_phys -= sizeof(gd_t);
	new_gd_addr = addr_sp_phys;
	DEBUG_PRINTF ("Reserving %d Bytes for Global Data at: %08qx\n",
			sizeof (gd_t), new_gd_addr);

	 	/* Reserve memory for boot params.
		 */
	addr_sp_phys -= CFG_BOOTPARAMS_LEN;
	bd.bi_boot_params = addr_sp_phys;
	DEBUG_PRINTF ("Reserving %dk for boot params from stack at: %08qx\n",
			CFG_BOOTPARAMS_LEN >> 10, addr_sp_phys);

	/*
	 * Finally, we set up a new (bigger) stack.
	 *
	 * Leave some safety gap for SP, force alignment on 16 byte boundary
	 * Clear initial stack frame
	 */
	addr_sp_phys -= 16;
	addr_sp_phys &= ~0xF;
    /* Changed to avoid lvalue cast warnings */
        memset64(MAKE_XKPHYS(addr_sp_phys), 0, 8);

    bd.bi_uboot_ram_addr = uboot_base_phys;
    bd.bi_uboot_ram_used_size = u_boot_mem_top - bd.bi_uboot_ram_addr;

    DEBUG_PRINTF("Stack Pointer at: %08qx, stack size: 0x%08x\n", addr_sp_phys, STACK_SIZE);
    DEBUG_PRINTF("Base DRAM address used by u-boot: 0x%08qx, size: 0x%x\n", bd.bi_uboot_ram_addr, bd.bi_uboot_ram_used_size);
    DEBUG_PRINTF("Top of fixed address reserved memory: 0x%08x\n", BOOTLOADER_END_RESERVED_SPACE);

#ifdef DEBUG
    #define PRINT_RESERVED_ADDR(X)  printf("Reserved address %s: 0x%08x\n", #X, X);
    PRINT_RESERVED_ADDR(EXCEPTION_BASE_BASE);
    PRINT_RESERVED_ADDR(BOOTLOADER_DEBUG_REG_SAVE_BASE);
    PRINT_RESERVED_ADDR(BOOTLOADER_DEBUG_STACK_BASE);
    PRINT_RESERVED_ADDR(BOOTLOADER_PCI_READ_BUFFER_BASE);
    PRINT_RESERVED_ADDR(BOOTLOADER_BOOTMEM_DESC_ADDR);
    PRINT_RESERVED_ADDR(BOOTLOADER_END_RESERVED_SPACE);
#endif

    if ((BOOTLOADER_END_RESERVED_SPACE & 0x7fffffff) >= (bd.bi_uboot_ram_addr & 0x7fffffff))
    {
        printf("\n\n*************************************\n");
        printf("ERROR: U-boot memory usage overlaps fixed address reserved area!\n");
        printf("U-boot base address: 0x%08qx, reserved top: 0x%08x\n", bd.bi_uboot_ram_addr, BOOTLOADER_END_RESERVED_SPACE);
        printf("*************************************\n\n");
    }
	/*
	 * Save local variables to board info struct
	 */
	bd.bi_memstart	= CFG_SDRAM_BASE;	/* start of  DRAM memory */
	bd.bi_memsize	= gd->ram_size;		/* size  of  DRAM memory in bytes */
	bd.bi_baudrate	= gd->baudrate;		/* Console Baudrate */

        DEBUG_PRINTF("gd address: %p,  new_addr: 0x%qx\n", gd, new_gd_addr);
        /* use 64 bit copy here, as new stack could be outside of 32 bit addressable space (physically addressed).
        ** gd pointer is kseg0 address in scratch, so cast to int32 is proper. */
	memcpy64(MAKE_XKPHYS(new_gd_addr),  (int32_t)gd, sizeof (gd_t));

        /* Copy bd structure from local to real physical location. We have filled in a local
        ** copy, as the final location's physical address may not be 32 bit addressable (but will
        ** have 32 bit virutal address. */
	memcpy64(MAKE_XKPHYS(addr_bd_phys),  (int32_t)&bd, sizeof (bd_t));


        

	/* On the purple board we copy the code in a special way
	 * in order to solve flash problems
	 */
#ifdef CONFIG_PURPLE
	copy_code(addr);
#endif
        DEBUG_PRINTF("relocating and jumping to code in DRAM at addr: 0x%qx\n", addr);
#ifndef CONFIG_OCTEON_UBOOT_TLB
        /* For non-tlb mode, we need to adjust some parameters */
        DEBUG_PRINTF("Stack top: 0x%x, virt new gd: %p, reloc_code: %p\n", (uint32_t)addr_sp_phys, (void *)(uint32_t)new_gd_addr, &relocate_code);
        relocate_code ((uint32_t)addr_sp_phys, (void *)(uint32_t)new_gd_addr, MAKE_KSEG0(addr), 0); /* map_size is not used, here, so pass 0 */
#else

        /*  Here we need to update the stack and gd addresses
        ** to be the virtual (32 bit) addresses that they will be after
        ** the relocation.
        */
        uint32_t stack_virt = CFG_MONITOR_BASE + addr_sp_phys - uboot_base_phys;
        uint32_t gd_virt =  CFG_MONITOR_BASE + new_gd_addr - uboot_base_phys;

        DEBUG_PRINTF("Virtual stack top: 0x%x, virt new gd: 0x%x\n", stack_virt, gd_virt);

	relocate_code (stack_virt, (void *)(uint32_t)gd_virt, MAKE_XKPHYS(addr), map_size);
#endif

	/* NOTREACHED - relocate_code() does not return */
}
/************************************************************************
 *
 * This is the next part if the initialization sequence: we are now
 * running from RAM and have a "normal" C environment, i. e. global
 * data can be written, BSS has been cleared, the stack size in not
 * that critical any more, etc.
 *
 *  Note that dest_addr must be a 32 bit address (since u-boot is a 32 bit app.)
 *
 ************************************************************************
 */

void board_init_r (gd_t *id, uint64_t dest_addr)
{
    DECLARE_GLOBAL_DATA_PTR;
    cmd_tbl_t *cmdtp;
    extern void malloc_bin_reloc (void);
#ifndef CFG_ENV_IS_NOWHERE
    extern char * env_name_spec;
#endif
    char *s, *e;
    bd_t *bd;
    int i;

/* Convince GCC to really do what we want, otherwise the old value is still used......*/
    SET_K0(id);

    gd->flags |= GD_FLG_RELOC;  /* tell others: relocation done */

    /* This is the address that u-boot has relocated itself to.  Use this 
    ** address for RELOCATED_TEXT_BASE in the makefile to enable the relocated
    ** build for debug info. */
    DEBUG_PRINTF ("Now running in RAM - U-Boot at: 0x%qx, CFG_MONITOR_BASE: 0x%x\n", dest_addr, CFG_MONITOR_BASE);

    gd->reloc_off = (dest_addr & 0xFFFFFFFF) - (uint64_t)CFG_MONITOR_BASE;

    monitor_flash_len = (ulong)&uboot_end_data - dest_addr;

    /*
     * We have to relocate the command table manually
     */
    if (gd->reloc_off)
    {
        DEBUG_PRINTF("Relocating command table, offset: 0x%qx, dest: 0x%qx, MB: 0x%x\n", gd->reloc_off, dest_addr, CFG_MONITOR_BASE);
        for (cmdtp = &__u_boot_cmd_start; cmdtp !=  &__u_boot_cmd_end; cmdtp++)
        {
            ulong addr;

            addr = (ulong) (cmdtp->cmd) + gd->reloc_off;
            cmdtp->cmd =
            (int (*)(struct cmd_tbl_s *, int, int, char *[]))addr;

            addr = (ulong)(cmdtp->name) + gd->reloc_off;
            cmdtp->name = (char *)addr;

            if (cmdtp->usage)
            {
                addr = (ulong)(cmdtp->usage) + gd->reloc_off;
                cmdtp->usage = (char *)addr;
            }
#ifdef	CFG_LONGHELP
            if (cmdtp->help)
            {
                addr = (ulong)(cmdtp->help) + gd->reloc_off;
                cmdtp->help = (char *)addr;
            }
#endif
        }
        /* there are some other pointer constants we must deal with */
#ifndef CFG_ENV_IS_NOWHERE
        env_name_spec += gd->reloc_off;
#endif

    }  /* if (gd-reloc_off) */
    else
        DEBUG_PRINTF("Not relocating command table\n");

#if CONFIG_OCTEON_UBOOT_TLB && !defined(CFG_NO_FLASH)
    /* Here we need to update the boot bus mappings to put the flash boot bus mappings
    ** back at the expected locations.  We no longer need the adjusted mappings for the
    ** TLB usage.
    */
    {
        cvmx_mio_boot_reg_cfgx_t __attribute__((unused)) reg_cfg;
        reg_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_REG_CFG0);
        reg_cfg.s.base = ((CFG_FLASH_BASE >> 16) & 0x1fff);
        cvmx_write_csr(CVMX_MIO_BOOT_REG_CFG0, reg_cfg.u64);
    }


#endif

#ifndef MALLOC_IN_BOOTMEM
    /* initialize malloc() area */
    mem_malloc_init();
    malloc_bin_reloc();
#endif

    bd = gd->bd;


#if !CONFIG_OCTEON_SIM_HW_DIFF
    if (octeon_is_model(OCTEON_CN38XX))
    {
        extern int octeon_ipd_bp_enable(void);
        extern int octeon_ipd_bp_verify(void);
        int loops = octeon_ipd_bp_enable();
        if (loops < 40)
        {
            if (octeon_ipd_bp_verify())
                printf("IPD backpressure workaround verified, took %d loops\n", loops);
            else
                printf("IPD backpressure workaround FAILED verfication. Adjust octeon_ipd_bp_enable() for this chip!\n");
        }
        else
            printf("IPD backpressure workaround FAILED. Adjust octeon_ipd_bp_enable() for this chip!\n");
    }

    /* Disable the watchdog timers, as these are still running after a soft reset as a PCI target */
    int num_cores = cvmx_octeon_num_cores();
    for (i = 0; i < num_cores; i++)
        cvmx_write_csr(CVMX_CIU_WDOGX(i), 0);

#if !CONFIG_L2_ONLY
    DEBUG_PRINTF("About to flush L2 cache\n");
    /* Flush and unlock L2 cache (soft reset does not unlock locked lines) */
    cvmx_l2c_flush();
#endif
#endif

#if !CONFIG_OCTEON_SIM_SPEED && !defined(CONFIG_NO_RELOCATION) && !defined(CONFIG_NO_CLEAR_DDR)

    if (!(gd->flags & GD_FLG_MEMORY_PRESERVED))
    {
    /* Zero all of memory above DRAM_LATE_ZERO_OFFSET */
    printf("Clearing DRAM.....");
    octeon_clear_mem_pfs(DRAM_LATE_ZERO_OFFSET, (uint64_t)(MIN(256*1024*1024, gd->ram_size) - DRAM_LATE_ZERO_OFFSET));
    puts(".");

    if (octeon_is_model(OCTEON_CN63XX))
    {
        if (gd->ram_size > 256*1024*1024)
        {
            octeon_clear_mem_pfs(0x20000000ull, (uint64_t)(gd->ram_size - 256*1024*1024));
        }

    } else { 

        if (gd->ram_size > 256*1024*1024)
        {
            octeon_clear_mem_pfs(0x410000000ull, (uint64_t)MIN((gd->ram_size - 256*1024*1024), 256*1024*1024));
            puts(".");
        }

        if (gd->ram_size > 512*1024*1024)
        {
            octeon_clear_mem_pfs(0x20000000ull, gd->ram_size - 512*1024*1024ull);
            puts(".");
        }

    }
    CVMX_SYNC;  /* Make sure all writes are complete before we continue */
    printf(" done\n");
    }
#endif

    /* Clear DDR/L2C ECC error bits */
    if (octeon_is_model(OCTEON_CN63XX))
    {
        cvmx_write_csr(CVMX_L2C_INT_REG, cvmx_read_csr(CVMX_L2C_INT_REG));
        cvmx_write_csr(CVMX_L2C_ERR_TDTX(0), cvmx_read_csr(CVMX_L2C_ERR_TDTX(0)));
        cvmx_write_csr(CVMX_L2C_ERR_TTGX(0), cvmx_read_csr(CVMX_L2C_ERR_TTGX(0)));
    }
    else
    {
        cvmx_write_csr(CVMX_L2D_ERR, cvmx_read_csr(CVMX_L2D_ERR));
        cvmx_write_csr(CVMX_L2T_ERR, cvmx_read_csr(CVMX_L2T_ERR));
    }
    if (octeon_is_model(OCTEON_CN56XX)) {
        cvmx_l2c_cfg_t l2c_cfg;
        l2c_cfg.u64 = cvmx_read_csr(CVMX_L2C_CFG);
        if (l2c_cfg.cn56xx.dpres0) {
            cvmx_write_csr(CVMX_LMCX_MEM_CFG0(0), cvmx_read_csr(CVMX_LMCX_MEM_CFG0(0)));
        }
        if (l2c_cfg.cn56xx.dpres1) {
            cvmx_write_csr(CVMX_LMCX_MEM_CFG0(1), cvmx_read_csr(CVMX_LMCX_MEM_CFG0(1)));
        }
    } else if (octeon_is_model(OCTEON_CN63XX)) {
        cvmx_write_csr(CVMX_LMCX_INT(0), cvmx_read_csr(CVMX_LMCX_INT(0)));
    } else {
        cvmx_write_csr(CVMX_LMC_MEM_CFG0, cvmx_read_csr(CVMX_LMC_MEM_CFG0));
    }
    DEBUG_PRINTF("BOOTLOADER_BOOTMEM_DESC_ADDR: 0x%x\n", BOOTLOADER_BOOTMEM_DESC_ADDR);
    DEBUG_PRINTF("BOOTLOADER_BOOTMEM_DESC_SPACE: 0x%x\n", BOOTLOADER_BOOTMEM_DESC_SPACE);
    DEBUG_PRINTF("size of bootmem desc: %d, location: %p\n", sizeof(cvmx_bootmem_desc_t), (void *)MAKE_KSEG0(BOOTLOADER_BOOTMEM_DESC_SPACE));

    if (!cvmx_bootmem_phy_mem_list_init(gd->ram_size, OCTEON_RESERVED_LOW_MEM_SIZE, (void *)MAKE_KSEG0(BOOTLOADER_BOOTMEM_DESC_SPACE)))
    {
        printf("FATAL: Error initializing free memory list\n");
        while (1);
    }

#ifdef CONFIG_OCTEON_UBOOT_TLB
    /* Here we allocate the memory that u-boot is _already_ using.  Note that if the location of u-boot changes, this may need to be done differently.
    ** In particular, this _won't_ work if u-boot is placed at the beginning of a DRAM region, as that is where the free block size/length is stored, and
    ** u-boot will be corrupted when the bootmem list is created.
    */
    int64_t __attribute__((unused))bm_val;
    if (0 > (bm_val = cvmx_bootmem_phy_named_block_alloc(gd->bd->bi_uboot_ram_used_size, gd->bd->bi_uboot_ram_addr, gd->bd->bi_uboot_ram_addr + gd->bd->bi_uboot_ram_used_size, 0, "__uboot_code_data", 0)))
    {
        printf("ERROR: Unable to allocate DRAM for u-boot from bootmem allocator.\n");
        DEBUG_PRINTF("ub used_size: 0x%x, addr: 0x%qx, bootmem_retval: 0x%qx\n", gd->bd->bi_uboot_ram_used_size, gd->bd->bi_uboot_ram_addr, bm_val);
    }
#endif

    /* setup cvmx_sysinfo structure so we can use simple executive functions */
    octeon_setup_cvmx_sysinfo();

#ifdef CONFIG_OCTEON
    {
        /* Generate the u-boot prompt based on various conditions */
        extern char uboot_prompt[];
        char board_name[30];
        strncpy(board_name, cvmx_board_type_to_string(cvmx_sysinfo_get()->board_type), 30);
        lowcase(board_name);

        if (gd->flags & GD_FLG_FAILSAFE_MODE)
            sprintf(uboot_prompt, "Octeon %s Failsafe bootloader# ", board_name);
        else if (gd->flags & GD_FLG_RAM_RESIDENT)
            sprintf(uboot_prompt, "Octeon %s(ram)# ", board_name);
        else
            sprintf(uboot_prompt, "Octeon %s# ", board_name);
    }

#endif

    /* Take all cores out of reset to reduce power usage. The secondary cores
        will spin on a wait loop conserving power until a NMI is received */
    cvmx_write_csr(CVMX_CIU_PP_RST, 0);

#ifdef MALLOC_IN_BOOTMEM
    /* initialize malloc() area */
    mem_malloc_init();
    malloc_bin_reloc();
#endif
    /* relocate environment function pointers etc. */
    env_relocate();

#ifdef DEBUG
    {
        char *ptr;
        ptr = malloc(100);
        DEBUG_PRINTF("u-boot malloc ptr: %p, stack ptr: %p\n", ptr, &ptr);
    }

    cvmx_bootmem_phy_list_print();
#endif

#if !CONFIG_OCTEON_SIM_MEM_LAYOUT || defined(CONFIG_NET_MULTI) || defined(OCTEON_RGMII_ENET) || defined(OCTEON_MGMT_ENET) 
    {
        char *eptr;
        uint32_t addr;
        uint32_t size;

        eptr = getenv("octeon_reserved_mem_load_size");

        if (!eptr || !strcmp("auto", eptr))
        {
            /* Pick a size that we think is appropriate.
            ** Please note that for small memory boards this guess will likely not be ideal.
            ** Please pick a specific size for boards/applications that require it. */
            size = MIN(96*1024*1024, (gd->ram_size/3) & ~0xFFFFF);
        }
        else
        {
            size = simple_strtol(eptr,NULL, 16);
        }
        if (size)
        {
            eptr = getenv("octeon_reserved_mem_load_base");
            if (!eptr || !strcmp("auto", eptr))
            {
                uint64_t mem_top;
                /* Leave some room for previous allocations that are made starting at the top of the low
                ** 256 Mbytes of DRAM */
                int adjust = 1024*1024;
                /* Put block at the top of DDR0, or bottom of DDR2 */
                if (gd->ram_size <= 512*1024*1024 || (size > gd->ram_size - 512*1024*1024))
                    mem_top = MIN(gd->ram_size - adjust, 256*1024*1024 - adjust);
                else
                {
                    /* We have enough room, so set mem_top so that block is at
                    ** base of  DDR2 segment */
                    mem_top = 512*1024*1024 + size;
                }
                addr = mem_top - size;
            }
            else
            {
                addr = simple_strtol(eptr,NULL, 16);
            }

            if (0 > cvmx_bootmem_phy_named_block_alloc(size, addr, addr + size, 0, OCTEON_BOOTLOADER_LOAD_MEM_NAME, 0))
            {
                printf("ERROR: Unable to allocate bootloader reserved memory (addr: 0x%x, size: 0x%x).\n", addr, size);
            }
            else
            {
                /* Set default load address to base of memory reserved for loading.
                ** The setting of the env. variable also sets the load_addr global variable.
                ** This environment variable is overridden each boot if a reserved block is created. */
                char str[20];
                sprintf(str, "0x%x", addr);
                setenv("loadaddr", str);
                load_reserved_addr = addr;
                load_reserved_size = size;
            }
        }
        else
            printf("WARNING: No reserved memory for image loading.\n");

    }
#endif

#if !CONFIG_OCTEON_SIM_MEM_LAYOUT
    /* Reserve memory for Linux kernel.  The kernel requires specific physical addresses,
    ** so by reserving this here we can load simple exec applications and do other memory
    ** allocation without interfering with loading a kernel image.  This is freed and used
    ** when loading a Linux image.  If a Linux image is not loaded, then this is freed just
    ** before the applications start.
    */
    {
        char *eptr;
        uint32_t addr;
        uint32_t size;
        eptr = getenv("octeon_reserved_mem_linux_size");
        if (!eptr ||!strcmp("auto", eptr))
        {
            size = (MIN(gd->ram_size, 256*1024*1024))/2;
        }
        else
        {
            size = simple_strtol(eptr,NULL, 16);
        }
        if (size)
        {
            eptr = getenv("octeon_reserved_mem_linux_base");
            if (!eptr ||!strcmp("auto", eptr))
            {
                /* Start right after the bootloader */
                addr = OCTEON_RESERVED_LOW_MEM_SIZE;
            }
            else
            {
                addr = simple_strtol(eptr,NULL, 16);
            }
            /* Allocate the memory, and print message if we fail */
            if (0 > cvmx_bootmem_phy_named_block_alloc(size, addr, addr + size, 0, OCTEON_LINUX_RESERVED_MEM_NAME, 0))
                printf("ERROR: Unable to allocate linux reserved memory (addr: 0x%x, size: 0x%x).\n", addr, size);
        }

    }
#endif

    /* Create a named block for idle cores to loop */
    int64_t idle_loop;
    if (0 > (idle_loop = cvmx_bootmem_phy_named_block_alloc(IDLE_CORE_BLOCK_SIZE, 0, 0, 0, IDLE_CORE_BLOCK_NAME, 0)))
    {
        printf("ERROR: Unable to allocate DRAM for secondary block from bootmem allocator.\n");
    }

    /* Initialize the named block with the following code so that cores will spin on a
       wait loop conserving power until a NMI is received. 

       42000020  1: wait
       1000fffe     b 1b
       00000000     nop
     */
     memset(cvmx_phys_to_ptr(idle_loop), 0, IDLE_CORE_BLOCK_SIZE);
     *(int64_t *)cvmx_phys_to_ptr(idle_loop) = 0x420000201000fffeull;
     CVMX_SYNCW;

    /* Put bootmem descriptor address in known location for host */
     {
         /* Make sure it is not in kseg0, as we want physical addresss */
         uint64_t *u64_ptr = (void *)(0x80000000 | BOOTLOADER_BOOTMEM_DESC_ADDR);
         *u64_ptr = ((uint32_t)__cvmx_bootmem_internal_get_desc_ptr()) & 0x7fffffffull;
         DEBUG_PRINTF("bootmem descriptor address (looked up): 0x%x\n", ((uint32_t)__cvmx_bootmem_internal_get_desc_ptr()) & 0x7fffffffull);
     }

#ifdef CFG_PCI_CONSOLE
     /* Check to see if we need to set up a PCI console block. Set up structure if 
     ** either count is set (and greater than 0) or pci_console_active is set */
     uboot_octeon_pci_console_setup();
#endif


#if (CONFIG_COMMANDS & CFG_CMD_OCTEON_NAND) || (CONFIG_COMMANDS & CFG_CMD_NAND)
    /* NAND must be done after bootmem allocator is set up, and before NOR probing
    ** to handle some corner cases. */
int oct_nand_probe(void);
    oct_nand_probe();
#endif


#ifndef CFG_NO_FLASH
    /* Do flash init _after_ NAND probing, as we may have done some chip select enable
    ** fixups as part of the NAND probing. */
    if (octeon_check_nor_flash_enabled(CFG_FLASH_BASE))
    {
        int size;
        size = flash_init();
        display_flash_config (size);
        bd->bi_flashstart = CFG_FLASH_BASE;
        bd->bi_flashsize = size;
#if CFG_MONITOR_BASE == CFG_FLASH_BASE
        bd->bi_flashoffset = monitor_flash_len; /* reserved area for U-Boot */
#else
        bd->bi_flashoffset = 0;
#endif
    }
    else
    {
        printf("Flash boot bus region not enabled, skipping NOR flash config\n");
        bd->bi_flashstart = 0;
        bd->bi_flashsize = 0;
    }

#endif  /* CFG_NO_FLASH */

#if (CONFIG_COMMANDS & CFG_CMD_OCTEON_NAND) || (CONFIG_COMMANDS & CFG_CMD_NAND)
        puts ("NAND:  ");
        nand_init ();           /* go init the NAND (do this after flash init....)*/
#endif


#ifdef DEBUG
     cvmx_bootmem_phy_list_print();
#endif


#if (CONFIG_OCTEON_EEPROM_TUPLES)
    {
        /* Read coremask from eeprom, and set environment variable if present.
        ** Print warning if not present in EEPROM.
        ** Currently ignores voltage/freq fields, and just uses first capability tuple
        ** This is only valid for certain parts.
        */
        int addr;
        uint8_t ee_buf[OCTEON_EEPROM_MAX_TUPLE_LENGTH];
        octeon_eeprom_chip_capability_t *cc_ptr = (void *)ee_buf;
        addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_CHIP_CAPABILITY_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
        if (addr >= 0 && 
            (((octeon_is_model(OCTEON_CN38XX) && (cvmx_read_csr(CVMX_CIU_FUSE) & 0xffff) == 0xffff && !cvmx_octeon_fuse_locked()))
            || octeon_is_model(OCTEON_CN56XX) || octeon_is_model(OCTEON_CN52XX) || octeon_is_model(OCTEON_CN63XX)
            ))
        {
            char tmp[10];
            sprintf(tmp,"0x%04x", cc_ptr->coremask);
            setenv("coremask_override", tmp);
            coremask_from_eeprom = cc_ptr->coremask;  /* Save copy for later verification */
        }
        else
        {
            /* No chip capability tuple found, so we need to check if we expect one.
            ** CN31XX chips will all have fuses blown appropriately.
            ** CN38XX chips may have fuses blown, but may not.  We will check to see if
            ** we need a chip capability tuple and only warn if we need it but don't have it.
            */

            if (octeon_is_model(OCTEON_CN38XX))
            {
                /* Here we only need it if no core fuses are blown and the lockdown fuse is not blown.
                ** In all other cases the cores fuses are definitive and we don't need a coremask override
                */
                if ((cvmx_read_csr(CVMX_CIU_FUSE) & 0xffff) == 0xffff && !cvmx_octeon_fuse_locked())
                {
                    printf("Warning: No chip capability tuple found in eeprom, coremask_override may be set incorrectly\n");
                }
                else
                {
                    /* Clear coremask_override as we have a properly fused part, and don't need it */
                    setenv("coremask_override", NULL);
                }
            }
            else
            {
                /* 31xx and 30xx will always have core fuses blown appropriately */

                setenv("coremask_override", NULL);
            }
        }
    }
#endif


    /* Set numcores env variable to indicate the number of cores available */
    char tmp[10];
    sprintf(tmp,"%d", octeon_get_available_core_count());
    setenv("numcores", tmp);



    /* board MAC address */
    s = getenv ("ethaddr");
    for (i = 0; i < 6; ++i)
    {
        bd->bi_enetaddr[i] = s ? simple_strtoul (s, &e, 16) : 0;
        if (s)
            s = (*e) ? e + 1 : e;
    }

    /* IP Address */
    bd->bi_ip_addr = getenv_IPaddr("ipaddr");

#if defined(CONFIG_PCI)
    /*
     * Do pci configuration
     */
    if (getenv("disable_pci"))
        printf("Skipping PCI due to 'disable_pci' environment variable\n");
    else
        pci_init();
#endif

/** leave this here (after malloc(), environment and PCI are working) **/
    /* Initialize devices */
    devices_init ();

    /* Must change std* variables after devices_init()
    ** Force bootup settings based on pci_console_active, overriding the std* variables */
    if (getenv("pci_console_active"))
    {
        printf("Using PCI console, serial port disabled.\n");
        setenv("stdin", "pci");
        setenv("stdout", "pci");
        setenv("stderr", "pci");
        octeon_led_str_write("PCI CONS");
    }
    else
    {
        setenv("stdin", "serial");
        setenv("stdout", "serial");
        setenv("stderr", "serial");
    }


    jumptable_init ();

    /* Initialize the console (after the relocation and devices init) */
    console_init_r ();
/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

    /* We want to print out the board/chip info again, as this is very useful for automated testing.
    ** and board identification. */
    if (getenv("pci_console_active"))
        display_board_info();

    /* Initialize from environment. Note that in most cases the environment
    ** variable will have been set from the reserved memory block allocated above.
    ** By also checking this here, this allows the load address to be set in cases
    ** where there is no reserved block configured. 
    ** If loadaddr is not set, set it to something.  This default should never really be used,
    ** but we need to handle this case. */
    if ((s = getenv ("loadaddr")) != NULL)
    {
        load_addr = simple_strtoul (s, NULL, 16);
    }
    else
    {
        uint32_t addr = 0x100000;
        char tmp[20];
#ifdef CFG_LOAD_ADDR
        addr = CFG_LOAD_ADDR;
#endif
        sprintf(tmp, "0x%x", addr);
#if !CONFIG_OCTEON_SIM_MEM_LAYOUT
        printf("WARNING: loadaddr not set, defaulting to %s.  Please either define a load reserved block or set the loadaddr environment variable.\n", tmp);
#endif
        setenv("loadaddr", tmp);
    }


#if !CONFIG_OCTEON_SIM_NO_FLASH && !defined(CFG_NO_FLASH) && !CONFIG_OCTEON_NAND_STAGE2
    {
        uint32_t octeon_find_and_validate_normal_bootloader(int rescan_flag);
        uint32_t get_image_size(const bootloader_header_t *header);
        char tmp[30];
	extern flash_info_t flash_info[];
        flash_info_t *info;
	int sector;
        unsigned long flash_unused_address, flash_unused_size;
        unsigned long uboot_flash_address, uboot_flash_size;
        /* Set up a variety of environment variables for use by u-boot and
        ** applications (through environment named block). */

        /* Set env_addr and env_size variables to the address and size of the u-boot environment. */
        sprintf(tmp,"0x%x", CFG_ENV_ADDR);
        setenv("env_addr", tmp);
        sprintf(tmp,"0x%x", CFG_ENV_SIZE);
        setenv("env_size", tmp);


        /* Describe physical flash */
        sprintf(tmp,"0x%x", CFG_FLASH_BASE);
        setenv("flash_base_addr", tmp);
        sprintf(tmp,"0x%x", CFG_FLASH_SIZE);
        setenv("flash_size", tmp);

        /* Here we want to set some environment variables that describe how
        ** the flash is currently used, and protect the region that has u-boot code.
        */
        if (gd->flags & GD_FLG_FAILSAFE_MODE)
        {
            uboot_flash_size = gd->uboot_flash_size;
            uboot_flash_address = gd->uboot_flash_address;
        }
        else
        {
            uboot_flash_address = octeon_find_and_validate_normal_bootloader(1);
            uboot_flash_size = get_image_size((bootloader_header_t *)(uboot_flash_address));
        }
        flash_unused_address = uboot_flash_address + uboot_flash_size;


	/* Align this to the correct sector */
        info = &flash_info[0];
	for (sector = 0; sector < info->sector_count; ++sector)
	    if (flash_unused_address <= info->start[sector]) 
		break;

        if (sector == info->sector_count)
	    printf("ERROR: No unused memory available in flash\n");

	flash_unused_address = info->start[sector];

        flash_unused_size = CFG_FLASH_SIZE - (flash_unused_address - CFG_FLASH_BASE);
        /* Set addr size for flash space that is available for application use */
        sprintf(tmp,"0x%x", uboot_flash_address);
        setenv("uboot_flash_addr", tmp);
        sprintf(tmp,"0x%x", flash_unused_address - uboot_flash_address);
        setenv("uboot_flash_size", tmp);

        sprintf(tmp,"0x%x", flash_unused_address);
        setenv("flash_unused_addr", tmp);
        sprintf(tmp,"0x%x", flash_unused_size);
        setenv("flash_unused_size", tmp);

        /* We now want to protect the flash from the base to the unused address */
	flash_protect (FLAG_PROTECT_SET,
		       CFG_FLASH_BASE,
		       flash_unused_address - 1,
		       info);

        /* Set obsolete bootloader_flash_update to do the new bootloaderupdate command. */
        setenv("bootloader_flash_update", "bootloaderupdate");
    }
#endif

#if (CONFIG_COMMANDS & CFG_CMD_NET)
    if ((s = getenv ("bootfile")) != NULL)
    {
        copy_filename (BootFile, s, sizeof (BootFile));
    }
#endif	/* CFG_CMD_NET */

#if defined(CONFIG_MISC_INIT_R)
    /* miscellaneous platform dependent initialisations */
    misc_init_r ();
#endif

#if CONFIG_OCTEON_LANBYPASS
    /* Set the LAN bypass defaults for Thunder */
    octeon_gpio_cfg_output(6); /* GPIO 6 controls the bypass enable */
    octeon_gpio_cfg_output(7); /* GPIO 7 controls the watchdog clear */
    octeon_gpio_cfg_input(5);  /* GPIO 5 tells you the bypass status */
    
    /* Startup with bypass disabled and watchdog cleared */
    octeon_gpio_clr(6);
    octeon_gpio_set(7);
#endif 

#if (CONFIG_COMMANDS & CFG_CMD_NET) && defined(CONFIG_NET_MULTI)
    puts ("Net:   ");
    eth_initialize(gd->bd);
#endif


#if CONFIG_OCTEON && !CONFIG_OCTEON_SIM_NO_FLASH && (CONFIG_COMMANDS & CFG_CMD_IDE)
#ifndef CONFIG_PCI  
    if (1 || octeon_cf_present())
#endif
    {
        /* Scan for IDE controllers and register them in the ide_dev_desc array. 
        ** The order of scanning will determine what order they are listed to the user in. */

        /* Scan the DMA capable slot first, so that it is contoller 0 on the ebh5600 board. (and will likely
        ** be the only one populated on later board builds */
#if defined(OCTEON_CF_TRUE_IDE_CS0_ADDR)
        uint32_t dma_flag = 0;
        if (getenv("use_cf_dma") && !getenv("disable_cf_dma"))
        {
            dma_flag = BLOCK_QUIRK_IDE_CF_TRUE_IDE_DMA;
        }
        /* Init IDE structures for CF interface on boot bus */
        ide_register_device(IF_TYPE_IDE, BLOCK_QUIRK_IDE_CF | BLOCK_QUIRK_IDE_CF_TRUE_IDE | dma_flag, 0, MAKE_XKPHYS(OCTEON_CF_TRUE_IDE_CS0_ADDR), MAKE_XKPHYS(OCTEON_CF_TRUE_IDE_CS0_ADDR), 0, 0);
        if (CFG_IDE_MAXDEVICE > 1)
        {
            /* This device won't every be used, but we want to have 2 devices per ide bus regargless.  If we
            ** are only supporting 1 device, skip this */
            ide_register_device(IF_TYPE_IDE, BLOCK_QUIRK_IDE_CF | BLOCK_QUIRK_IDE_CF_TRUE_IDE | dma_flag, 1, MAKE_XKPHYS(OCTEON_CF_TRUE_IDE_CS0_ADDR), MAKE_XKPHYS(OCTEON_CF_TRUE_IDE_CS0_ADDR), 0, 0);
        }
#endif

#ifdef OCTEON_CF_COMMON_BASE_ADDR
        /* Init IDE structures for CF interface on boot bus */
        ide_register_device(IF_TYPE_IDE, BLOCK_QUIRK_IDE_CF, 0, MAKE_XKPHYS(OCTEON_CF_COMMON_BASE_ADDR + CFG_ATA_DATA_OFFSET), MAKE_XKPHYS(OCTEON_CF_COMMON_BASE_ADDR + CFG_ATA_REG_OFFSET), 0, 0);
        if (CFG_IDE_MAXDEVICE > 1)
        {
            /* This device won't every be used, but we want to have 2 devices per ide bus regargless.  If we
            ** are only supporting 1 device, skip this */
            ide_register_device(IF_TYPE_IDE, BLOCK_QUIRK_IDE_CF, 1, MAKE_XKPHYS(OCTEON_CF_COMMON_BASE_ADDR + CFG_ATA_DATA_OFFSET), MAKE_XKPHYS(OCTEON_CF_COMMON_BASE_ADDR + CFG_ATA_REG_OFFSET), 0, 0);
        }
#endif

        /* Always do ide_init if PCI enabled to allow for PCI IDE devices */
        ide_init();
    }
#endif

#ifdef CONFIG_USB_OCTEON
    /* Scan for USB devices automatically on boot.
    ** The host port used can be selected with the 'usb_host_port' env variable,
    ** which can also disable the scanning completely. */
    int usb_storage_scan(void);
    {
        printf("\n");
        int usb_init(void);
        if (usb_init() >= 0)
            usb_storage_scan();
    }
#endif

#if CONFIG_OCTEON
    /* verify that boot_init_vector type is the correct size */
    if (BOOT_VECTOR_NUM_WORDS*4 != sizeof(boot_init_vector_t))
    {
        printf("ERROR: boot_init_vector_t size mismatch: define: %d, type: %d\n",BOOT_VECTOR_NUM_WORDS*8,  sizeof(boot_init_vector_t));
        while (1)
            ;
    }
#endif


#if CFG_LATE_BOARD_INIT
    late_board_init();
#endif

    /* main_loop() can return to retry autoboot, if so just run it again. */
    for (;;)
    {
        main_loop ();
    }

    /* NOTREACHED - no way out of command loop except booting */
}

void hang (void)
{
	puts ("### ERROR ### Please RESET the board ###\n");
	for (;;);
}

#ifdef ENABLE_BOARD_DEBUG
void gpio_status(int data)
{
    int i;
    for (i=0; i<7; ++i) {
        if ((data>>i)&1)
            octeon_gpio_set(8+i);
        else
            octeon_gpio_clr(8+i);
    }
}
#endif
