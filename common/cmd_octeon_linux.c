/*
 * Copyright (c) 2001 William L. Pitts
 * All rights reserved.
 * Copyright (c) 2003-2007 Cavium Networks (support@cavium.com).

 * Redistribution and use in source and binary forms are freely
 * permitted provided that the above copyright notice and this
 * paragraph and the following disclaimer are duplicated in all
 * such forms.
 *
 * This software is provided "AS IS" and without any express or
 * implied warranties, including, without limitation, the implied
 * warranties of merchantability and fitness for a particular
 * purpose.
 */


/**
 * @file
 *
 * $Id: cmd_octeon_linux.c 52149 2010-08-21 06:21:09Z cchavva $
 *
 */
#include <common.h>
#include <command.h>
#include <linux/ctype.h>
#include <net.h>
#include <elf.h>
#include <octeon_eeprom_types.h>
#include <lib_octeon.h>
#include <asm/mipsregs.h>
#include <asm/processor.h>
#include "octeon_boot.h"
#include "octeon-boot-info.h"
#include "cvmx-bootmem.h"
#include "cvmx-app-init.h"
#include "lib_octeon_shared.h"
extern cvmx_bootinfo_t cvmx_bootinfo_array[16];


#if (CONFIG_COMMANDS & CFG_CMD_OCTEON)

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

int valid_elf_image (unsigned long addr);  /* from cmd_elf.c */
unsigned long load_elf32_image (unsigned long addr); 
uint64_t load_elf64_image (unsigned long addr);

void start_linux(void);
void start_elf(void);
static int alloc_elf32_image (unsigned long addr);
static int alloc_elf64_linux_image (unsigned long addr);

volatile int start_core0 = 0;
extern uint32_t cur_exception_base;
extern boot_info_block_t boot_info_block_array[];

extern void InitTLBStart(void);  /* in start.S */
extern void start_app(void);

void setup_linux_app_boot_info(uint32_t linux_core_mask)
{
    DECLARE_GLOBAL_DATA_PTR;

    uint32_t fuse_coremask =  octeon_get_available_coremask();
    uint32_t app_coremask = fuse_coremask & ~linux_core_mask;
    linux_app_boot_info_t *labi = (linux_app_boot_info_t*)0x80000700;
    labi->labi_signature = LABI_SIGNATURE;
    labi->start_core0_addr = (uint32_t)&start_core0;
    if (labi->avail_coremask)
        labi->avail_coremask &= app_coremask & coremask_from_eeprom;
    else
        labi->avail_coremask = app_coremask & coremask_from_eeprom;
    labi->pci_console_active = (getenv("pci_console_active")) ? 1 : 0;
    labi->icache_prefetch_disable = (getenv("icache_prefetch_disable")) ? 1 : 0;
    labi->no_mark_private_data = (getenv("no_mark_private_data")) ? 1 : 0;
    labi->InitTLBStart_addr = MAKE_XKPHYS(cvmx_ptr_to_phys(&InitTLBStart));  /* Physical, with xkphys */
    labi->start_app_addr = (uint32_t)&start_app;  /* Virtual */
    labi->cur_exception_base = cur_exception_base;
    labi->compact_flash_common_base_addr = cvmx_bootinfo_array[0].compact_flash_common_base_addr;
    labi->compact_flash_attribute_base_addr = cvmx_bootinfo_array[0].compact_flash_attribute_base_addr;
    labi->led_display_base_addr = cvmx_bootinfo_array[0].led_display_base_addr;
    memcpy((void*)&labi->gd, (void *)gd, sizeof(gd_t));
    #ifdef DEBUG
    printf("sizeof(linux_app_boot_info_t) %d start_core0 address is %x\n", sizeof(*labi), labi->start_core0_addr);
    #endif
}



int do_bootoctlinux (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    uint32_t addr;		/* Address of the ELF image     */
    uint64_t entry_addr = 0;
    int i;
    int forceboot = 0;

    int32_t core_mask = 1;
    uint32_t boot_flags = 0;
    /* -------------------------------------------------- */
    int rcode = 0;
    int num_cores = 0;
    int skip_cores = 0;


#if CONFIG_OCTEON_SIM_SW_DIFF
    /* Default is to run on all cores on simulator*/
    core_mask = 0xffffULL & cvmx_read_csr(CVMX_CIU_FUSE);
#endif

    if (argc < 2)
            addr = load_addr;
    else
    {
        addr = simple_strtoul (argv[1], NULL, 16);
        if (!addr)
            addr = load_addr;
    }


    for (i = 2; i < argc; i++)
    {
        printf("argv[%d]: %s\n", i, argv[i]);
        if (!strncmp(argv[i], "coremask=0x", 11))
        {
            core_mask = simple_strtoul(argv[i] + 9, NULL, 0);
        }
        else if (!strncmp(argv[i], "coremask=", 9))
        {
            char tmp[20] = "0x";
            strncat(tmp, argv[i] + 9 , 10);
            core_mask = simple_strtoul(tmp, NULL, 0);
        }
        else if (!strncmp(argv[i], "forceboot", 9))
        {
            forceboot = 1;
        }
        else if (!strncmp(argv[i], "debug", 5))
        {
            boot_flags |= OCTEON_BL_FLAG_DEBUG;
        }
        else if (!strncmp(argv[i], "numcores=", 9))
        {
            num_cores = simple_strtoul(argv[i] + 9, NULL, 0);
        }
        else if (!strncmp(argv[i], "skipcores=", 10))
        {
            skip_cores = simple_strtoul(argv[i] + 10, NULL, 0);
        }
        else if (!strncmp(argv[i], "endbootargs", 12))
        {
            argc -= i + 1;
            argv = &argv[i + 1];
            break;  /* stop processing argument */
        }
    }

    /* numcores specification overrides a coremask on the same command line */
    if (num_cores)
    {
        core_mask = octeon_coremask_num_cores(num_cores + skip_cores) & ~octeon_coremask_num_cores(skip_cores);
    }

    /* Remove cores from coremask based on environment variable stored in flash */
    core_mask = validate_coremask(core_mask);
    if (!core_mask)
    {
        printf("Coremask is empty after coremask_override mask. Nothing to do.\n");
        return(0);
    }
    else if (core_mask < 0)
    {
        printf("Invalid coremask.\n");
        return(1);
    }

    if (!valid_elf_image (addr))
        return 1;

    /* Free memory that was reserved for kernel image.  Don't check return code,
    ** as this may be the second kernel loaded, and loading will fail later
    ** if the required address isn't available.
    */
    cvmx_bootmem_phy_named_block_free(OCTEON_LINUX_RESERVED_MEM_NAME, 0);


    if (((Elf32_Ehdr *)addr)->e_ident[EI_CLASS] == ELFCLASS32)
    {
        if (alloc_elf32_image(addr))
        {
            entry_addr = load_elf32_image (addr);
            /* make sure kseg0 addresses are sign extended */
            if ((int32_t)entry_addr < 0)
                entry_addr |= 0xffffffff00000000ull;
        }
    }
    else  /* valid_elf_image ensures only 32 and 64 bit class values */
    {
        if (alloc_elf64_linux_image(addr))
            entry_addr = load_elf64_image (addr);
    }
    if (!entry_addr)
    {
        printf ("## ERROR loading File!\n");
        return(-1);
    }

    if (core_mask & coremask_to_run)
    {
        printf("ERROR: Can't load code on core twice! (provided coremask(0x%x) overlaps previously loaded coremask(0x%x))\n",
               core_mask, coremask_to_run);
        return(-1);
    }

    printf ("## Loading Linux kernel with entry point: 0x%08qx ...\n", entry_addr);

    if (octeon_setup_boot_desc_block(core_mask, argc, argv, entry_addr, 0, 0, boot_flags, 0, OCTEON_BOOT_DESC_IMAGE_LINUX, 0))
    {
        printf("ERROR setting up boot descriptor block, core_mask: 0x%llx\n", core_mask);
        return(-1);
    }


    if (octeon_setup_boot_vector((uint32_t)start_linux, core_mask))
    {
        printf("ERROR setting boot vectors, core_mask: 0x%llx\n", core_mask);
        return(-1);
    }

    /* Add coremask to global mask of cores that have been set up and are runable */
    coremask_to_run |= core_mask;

    /* Check environment for forceboot flag */
    if (getenv("forceboot"))
        forceboot |= 1;

    setup_linux_app_boot_info(core_mask);
    printf("Bootloader: Done loading app on coremask: 0x%x\n", core_mask);
    /* Start other cores, but only if core zero is in mask */
    if ((core_mask & 1) || forceboot)
    {

        octeon_bootloader_shutdown();
        start_cores(coremask_to_run); /* Does not return */
    }

    return rcode;
}


/* Returns entry point from elf 32 image */
unsigned long elf32_entry (unsigned long addr)
{
    Elf32_Ehdr *ehdr;           /* Elf header structure pointer     */
    ehdr = (Elf32_Ehdr *) addr;
    return ehdr->e_entry;
}



int do_bootoctelf (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    unsigned long addr;		/* Address of the ELF image     */
    unsigned long mem_size = 0;     /* size of memory for ELF image */
    int i;
    int forceboot = 0;
    static uint32_t saved_exception_base = 0;

    int32_t core_mask = 1;
    uint32_t boot_flags = 0;
    /* -------------------------------------------------- */
    int rcode = 0;
    int num_cores = 0;
    int skip_cores = 0;


#if CONFIG_OCTEON_SIM_SW_DIFF
    /* Default is to run on all cores on simulator*/
    core_mask = 0xffffULL & cvmx_read_csr(CVMX_CIU_FUSE);
#endif

    if (argc < 2)
    {
        addr = load_addr;
    }
    else if (argc >= 3)
    {
        addr = simple_strtoul (argv[1], NULL, 16);
        if (!addr)
            addr = load_addr;
        mem_size = simple_strtoul (argv[2], NULL, 16);
    }
    else
    {
        addr = simple_strtoul (argv[1], NULL, 16);
        if (!addr)
            addr = load_addr;
        mem_size = 0;
        printf("Warning: No memory size provided, this application won't co-exist with simple exec. applications or Linux.\n");
    }


    for (i = 3; i < argc; i++)
    {
        if (!strncmp(argv[i], "coremask=0x", 11))
        {
            core_mask = simple_strtoul(argv[i] + 9, NULL, 0);
        }
        else if (!strncmp(argv[i], "coremask=", 9))
        {
            char tmp[20] = "0x";
            strncat(tmp, argv[i] + 9 , 10);
            core_mask = simple_strtoul(tmp, NULL, 0);
        }
        else if (!strncmp(argv[i], "forceboot", 9))
        {
            forceboot = 1;
        }
        else if (!strncmp(argv[i], "debug", 5))
        {
            boot_flags |= OCTEON_BL_FLAG_DEBUG;
        }
        else if (!strncmp(argv[i], "numcores=", 9))
        {
            num_cores = simple_strtoul(argv[i] + 9, NULL, 0);
        }
        else if (!strncmp(argv[i], "skipcores=", 10))
        {
            skip_cores = simple_strtoul(argv[i] + 10, NULL, 0);
        }
    }

    /* numcores specification overrides a coremask on the same command line */
    if (num_cores)
    {
        core_mask = octeon_coremask_num_cores(num_cores + skip_cores) & ~octeon_coremask_num_cores(skip_cores);
    }

    /* Remove cores from coremask based on environment variable stored in flash */
    core_mask = validate_coremask(core_mask);
    if (!core_mask)
    {
        printf("Coremask is empty after coremask_override mask. Nothing to do.\n");
        return(0);
    }
    else if (core_mask < 0)
    {
        printf("Invalid coremask.\n");
        return(1);
    }


    /* Force the exception base to be zero (IOS needs this), but still attempt
    ** to play well with others. */
    if (!saved_exception_base)
    {
        saved_exception_base = cur_exception_base;
        cur_exception_base = 0;
    }

    CVMX_POP(num_cores, core_mask);
    if (num_cores != 1)
    {
        printf("Warning: Coremask has %d cores set.  Application must support multiple cores for proper operation.\n", num_cores);
    }

    if (!valid_elf_image (addr))
        return 1;

    if (core_mask & coremask_to_run)
    {
        printf("ERROR: Can't load code on core twice! (provided coremask(0x%x) overlaps previously loaded coremask(0x%x))\n",
               core_mask, coremask_to_run);
        return(-1);
    }

    if (((Elf32_Ehdr *)addr)->e_ident[EI_CLASS] == ELFCLASS32)
    {
        if (mem_size)
        {
            uint32_t mem_base;
            mem_base = elf32_entry(addr) & 0x7fffffff; /* Mask of kseg0 bit, we want phys addr */
            mem_base &= ~0xfffff;  /* Round down to  Mbyte boundary */
            printf("Allocating memory for ELF: Base addr, 0x%x, size: 0x%x\n", mem_base, mem_size);

            /* For ELF images the user must specify the size of memory above the entry
            ** point of the elf image that is reserved for the ELF.  Here we allocate that memory
            ** so that other software can co-exist with with the ELF image. */
            if (0 > cvmx_bootmem_phy_alloc(mem_size, mem_base, mem_base + mem_size, 0, 0))
            {
                printf("Error allocating memory for ELF image!\n");
                return(0);
            }
        }
        addr = load_elf32_image (addr);
    }
    else  /* valid_elf_image ensures only 32 and 64 bit class values */
    {
        printf("ERROR: only loading of 32 bit  ELF images is supported\n");
    }

    if (!addr)
    {
        printf ("## ERROR loading File!\n");
        return(-1);
    }


    printf ("## Loading ELF image with entry point: 0x%08lx ...\n", addr);

    /* ELF images uses same setup as linux */
    if (octeon_setup_boot_desc_block(core_mask, argc, argv, addr, 0, 0, boot_flags, 0, OCTEON_BOOT_DESC_IMAGE_LINUX, 0))
    {
        printf("ERROR setting up boot descriptor block, core_mask: 0x%llx\n", core_mask);
        return(-1);
    }

    if (octeon_setup_boot_vector((uint32_t)start_elf, core_mask))
    {
        printf("ERROR setting boot vectors, core_mask: 0x%llx\n", core_mask);
        return(-1);
    }

    /* Add coremask to global mask of cores that have been set up and are runable */
    coremask_to_run |= core_mask;

    /* Check environment for forceboot flag */
    if (getenv("forceboot"))
        forceboot |= 1;

    printf("Bootloader: Done loading app on coremask: 0x%x\n", core_mask);
    /* Start other cores, but only if core zero is in mask */
    if ((core_mask & 1) || forceboot)
    {
        octeon_bootloader_shutdown();
        start_cores(coremask_to_run); /* Does not return */
    }


    cur_exception_base = saved_exception_base;
    return rcode;
}






void start_os_common(void)
{

    int core_num;
    uint64_t val;
    octeon_boot_descriptor_t *cur_boot_desc;
    boot_info_block_t *boot_info_ptr;
    extern void octeon_sync_cores(void);

    core_num = get_core_num();

    /* Set local cycle counter based on global counter in IPD */
    cvmx_set_cycle(cvmx_read_csr(CVMX_IPD_CLK_COUNT));

    /* look up boot descriptor block for this core */
    cur_boot_desc = &boot_desc[core_num];
    boot_info_ptr = &boot_info_block_array[core_num];

    set_except_base_addr(boot_info_ptr->exception_base);

    /* Check to see if we should enable icache prefetching.  On pass 1
    ** it should always be disabled */
    val = get_cop0_cvmctl_reg();
    if ((boot_info_ptr->flags & BOOT_INFO_FLAG_DISABLE_ICACHE_PREFETCH))
        val |= (1ull << 13);
    else
        val &= ~(1ull << 13);
    /* Disable Fetch under fill on CN63XXp1 due to errata Core-14317 */
    if (octeon_is_model(OCTEON_CN63XX_PASS1_X))
        val |= (1ull << 19); /*CvmCtl[DEFET] */
#ifdef PROFILE  /* Define this to disable conditional clocking for acurate profiling */
    val |= (1ull << 17);     /* DISCE for perf counters */
#endif
    set_cop0_cvmctl_reg(val & 0xfffff3);

    /* Disable core stalls on write buffer full */
    if (octeon_is_model(OCTEON_CN38XX_PASS2))
    {
        val = get_cop0_cvmmemctl_reg();
        val |= 1ull << 29;      /* set diswbfst */
#ifdef PROFILE
        val |= 1ull << 19;     /* Mclkalways, for perf counters */
#endif

#if 0
        /* 14:11 is wb thresh */
        val &= ~((0xf) << 11);
        val |= ((0xc) << 11);

        /* set lines of scratch */
        val &= ~0x3f;
        val |= 0x2;
#endif
        set_cop0_cvmmemctl_reg(val);
    }
    /* Sync up cores before starting main */
    octeon_sync_cores();

}
extern void asm_launch_linux_entry_point;
void start_linux(void)
{
    int core_num = get_core_num();
    boot_info_block_t *boot_info_ptr;
    start_os_common();

    /* look up boot descriptor block for this core */
    octeon_boot_descriptor_t *cur_boot_desc = &boot_desc[core_num];
    uint64_t boot_desc_addr;
#if CONFIG_OCTEON_UBOOT_TLB
    boot_desc_addr = MAKE_XKPHYS(uboot_tlb_ptr_to_phys(&boot_desc[core_num]));
#else
    boot_desc_addr = (uint32_t)&boot_desc[core_num];
#endif
    boot_info_ptr = &boot_info_block_array[core_num];

	/*
	 * pass address parameter as argv[0] (aka command name),
	 * and all remaining args
     * a0 = argc
     * a1 = argv (32 bit physical addresses, not pointers)
     * a2 = init core
     * a3 = boot descriptor address
     * a4/t0 = entry point (only used by assembly stub)
	 */
    /* Branch to start of linux kernel, passing args */
    /* NOTE - argv array is not 64 bit pointers, but is 32 bit addresses */
    /* Here we need to branch to a stub in ASM to clear the TLB entry used by
     * u-boot.  This stub is run from an XKPHYS address */

    uint64_t arg0 = cur_boot_desc->argc;
    uint64_t arg1 = (int32_t)cur_boot_desc->argv;
    uint64_t arg2 = !!(cur_boot_desc->flags & BOOT_FLAG_INIT_CORE);
    uint64_t arg3 = boot_desc_addr;
    uint64_t arg4 = boot_info_ptr->entry_point;
    uint64_t addr = MAKE_XKPHYS(uboot_tlb_ptr_to_phys(&asm_launch_linux_entry_point));

    /* Set up TLB registers to clear desired entry.  The actual
    ** tlbwi instruction is done in ASM when running from unmapped DRAM */
    write_64bit_cp0_register(CP0_ENTRYLO0, 0);
    write_32bit_cp0_register(CP0_PAGEMASK, 0);
    write_64bit_cp0_register(CP0_ENTRYLO1, 0);
    write_64bit_cp0_register(CP0_ENTRYHI, 0xFFFFFFFF91000000ull);
    write_32bit_cp0_register(CP0_INDEX, get_num_tlb_entries() - 1);

    asm volatile (
           "       .set push              \n"
           "       .set mips64              \n"
           "       .set noreorder           \n"
           "       move  $4, %[arg0] \n"
           "       move  $5, %[arg1] \n"
           "       move  $6, %[arg2] \n"
           "       move  $7, %[arg3] \n"
           "       move  $8, %[arg4] \n"
           "       j     %[addr]  \n"
           "       nop                      \n"
           "       .set pop              \n"
            : :[arg0] "r" (arg0), [arg1] "r" (arg1), [arg2] "r" (arg2), [arg3] "r" (arg3), [arg4] "r" (arg4), [addr] "r" (addr)
            : "$4","$5","$6","$7","$8" );
    /* Make sure that GCC doesn't use the explicit registers that we want to use  */



}
void start_elf(void)
{
    int core_num = get_core_num();
    boot_info_block_t *boot_info_ptr;
    start_os_common();

    /* look up boot descriptor block for this core */
    octeon_boot_descriptor_t *cur_boot_desc = &boot_desc[core_num];
    boot_info_ptr = &boot_info_block_array[core_num];



    /* Here we need to branch to a stub in ASM to clear the TLB entry used by
     * u-boot.  This stub is run from an XKPHYS address */
    uint64_t arg0 = 0;
    uint64_t arg1 = (uint32_t)cur_boot_desc;
    uint64_t arg2 = 0;
    uint64_t arg3 = 0;
    uint64_t arg4 = boot_info_ptr->entry_point;
    uint64_t addr = MAKE_XKPHYS(uboot_tlb_ptr_to_phys(&asm_launch_linux_entry_point));
    
    /* Set up TLB registers to clear desired entry.  The actual
    ** tlbwi instruction is done in ASM when running from unmapped DRAM */
    write_64bit_cp0_register(CP0_ENTRYLO0, 0);
    write_32bit_cp0_register(CP0_PAGEMASK, 0);
    write_64bit_cp0_register(CP0_ENTRYLO1, 0);
    write_64bit_cp0_register(CP0_ENTRYHI, 0xFFFFFFFF91000000ull);
    write_32bit_cp0_register(CP0_INDEX, get_num_tlb_entries() - 1);

    asm volatile (
           "       .set push              \n"
           "       .set mips64              \n"
           "       .set noreorder           \n"
           "       move  $4, %[arg0] \n"
           "       move  $5, %[arg1] \n"
           "       move  $6, %[arg2] \n"
           "       move  $7, %[arg3] \n"
           "       move  $8, %[arg4] \n"
           "       j     %[addr]  \n"
           "       nop                      \n"
           "       .set pop              \n"
            : :[arg0] "r" (arg0), [arg1] "r" (arg1), [arg2] "r" (arg2), [arg3] "r" (arg3), [arg4] "r" (arg4), [addr] "r" (addr)
            : "$4","$5","$6","$7","$8" );
    /* Make sure that GCC doesn't use the explicit registers that we want to use  */
}





/**
 * Parse the ELF file and do bootmem allocs for
 * the memory. 
 * 
 * @param addr   address of ELF image
 * 
 * @return 0 on failure
 *         !0 on success
 */
static int alloc_elf32_image (unsigned long addr)
{
    Elf32_Ehdr *ehdr;       /* Elf header structure pointer     */
    Elf32_Shdr *shdr;       /* Section header structure pointer */
    Elf32_Phdr *phdr;               /* Segment header ptr */
    unsigned char *strtab = 0;  /* String table pointer             */
    int i;              /* Loop counter                     */

    /* -------------------------------------------------- */

    ehdr = (Elf32_Ehdr *) cvmx_phys_to_ptr(addr);

    /* Find the section header string table for output info */
    shdr = (Elf32_Shdr *) cvmx_phys_to_ptr(addr + ehdr->e_shoff +
                           (ehdr->e_shstrndx * sizeof (Elf32_Shdr)));

    if (shdr->sh_type == SHT_STRTAB)
        strtab = (unsigned char *) cvmx_phys_to_ptr(addr + shdr->sh_offset);

    for (i = 0; i < ehdr->e_phnum; ++i)
    {
        phdr = (Elf32_Phdr *)cvmx_phys_to_ptr(addr + ehdr->e_phoff +
                                        (i * sizeof (Elf32_Phdr)));
        if (phdr->p_type != PT_LOAD)
        {
            printf("Skipping non LOAD program header (type 0x%x)\n", phdr->p_type);
        }
        else if (phdr->p_memsz > 0)
        {
            if (0 > cvmx_bootmem_phy_alloc(phdr->p_memsz, (uint32_t)phdr->p_paddr & 0x7fffffff, 0, 0, 0))
            {
                printf("Error allocating memory for elf image!\n");
                return(0);
            }
            printf("Allocated memory for ELF segment: addr: 0x%x, size 0x%x\n", (uint32_t)phdr->p_paddr & 0x7fffffff, (uint32_t)phdr->p_memsz);
        }
    }
    return 1;
}

/**
 * Parse the Linux ELF file and do bootmem allocs for
 * the memory.  This does special handling for mapped kernels that
 * can be loaded at any memory location, as apposed to unmapped kernels that must
 * be loaded at the physical address in the ELF header.
 * 
 * For mapped kernels, we do an unconstrained bootmem alloc, and then
 * update the ELF header in memory to reflect where the segments should be copied to.
 * The generic ELF loading code that is used after this relies on the ELF header to
 * describe the physical addresses that should be used.  
 * 
 * NOTE: We update the physical load address and the entry point address in the elf image
 * so the the ELF loading routine will do the correct thing.
 * 
 * @param addr   address of ELF image
 * 
 * @return 0 on failure
 *         !0 on success
 */
static int alloc_elf64_linux_image (unsigned long addr)
{

    Elf64_Ehdr *ehdr;       /* Elf header structure pointer     */
    Elf64_Shdr *shdr;       /* Section header structure pointer */
    Elf64_Phdr *phdr;       /* Segment header ptr */
    unsigned char *strtab = 0;  /* String table pointer             */
    int i;              /* Loop counter                     */
    uint64_t phys_addr;
    int mapped_segment = 0;
    int64_t alloc_addr;

    /* -------------------------------------------------- */

    ehdr = (Elf64_Ehdr *) addr;
    /*
     * WARNING: here we do casts from 64 bit addresses to 32 bit
     * pointers.  This only works if the 64 addresses are in the
     * 32 bit compatibility space.
     */


    /* Find the section header string table for output info */
    shdr = (Elf64_Shdr *)(uint32_t)(addr + ehdr->e_shoff +
                                    (ehdr->e_shstrndx * sizeof (Elf64_Shdr)));

    if (shdr->sh_type == SHT_STRTAB)
        strtab = (unsigned char *) (uint32_t)(addr + shdr->sh_offset);

    for (i = 0; i < ehdr->e_phnum; ++i)
    {
        phdr = (Elf64_Phdr *)(uint32_t)(addr + ehdr->e_phoff +
                                        (i * sizeof (Elf64_Phdr)));
        if (phdr->p_memsz > 0 && phdr->p_type == PT_LOAD)
        {

            if (((phdr->p_vaddr >= 0xFFFFFFFFc0000000ULL && phdr->p_vaddr <= 0xFFFFFFFFFFFFFFFFULL)
                || (phdr->p_vaddr >= 0xC000000000000000ULL && phdr->p_vaddr <  0xFFFFFFFF80000000ULL)
                || (phdr->p_vaddr >= 0x0000000000000000ULL && phdr->p_vaddr <  0x8000000000000000ULL)
                ))
                mapped_segment = 1;
            else
                mapped_segment = 0;

            /* change virtual xkphys address to plain physical address that bootmem functions operate on. */
            phys_addr = octeon_fixup_xkphys(phdr->p_paddr);
            /* Only strip bit 63 if it is the only high bit set, for
            ** compatability addresses, we need to do more */
            if ((phys_addr & (0xffffffffull << 32)) == (0xffffffffull << 32))
                phys_addr &= 0x7fffffffull;
            else
                phys_addr &= ~(0x1ull << 63);

            if (!mapped_segment)
            {
                printf("Allocating memory for ELF segment: addr: 0x%llx (adjusted to: 0x%llx), size 0x%x\n", phdr->p_paddr, phys_addr, (uint32_t)phdr->p_memsz);
                alloc_addr = cvmx_bootmem_phy_alloc(phdr->p_memsz, phys_addr, 0, 0, 0);
            }
            else
            {
                /* We have a mapped kernel, so allocate anywhere */
                uint64_t alignment = 0x100000;
                /* Get power of 2 >= memory size, then adjust for MIPS dual-page TLB entries */
                while (alignment < phdr->p_memsz) alignment <<= 1;  
                alignment = alignment >> 1;
                printf("Allocating memory for mapped kernel segment, alignment: 0x%llx\n", alignment);
                alloc_addr = cvmx_bootmem_phy_alloc(phdr->p_memsz, 0, 0, alignment, 0);
                if (alloc_addr > 0)
                {
                    /* If we succeed, then we need to fixup the ELF header so the loader will do the right thing. */
                    //printf("Original paddr: 0x%llx, entry: 0x%llx\n", phdr->p_paddr, ehdr->e_entry);
                    uint64_t entry_offset = ehdr->e_entry - phdr->p_paddr;
                    phdr->p_paddr = alloc_addr;
                    ehdr->e_entry = phdr->p_paddr + entry_offset;
                    //printf("New paddr: 0x%llx, entry: 0x%llx\n", phdr->p_paddr, ehdr->e_entry);

                }
            }
            if (0 > alloc_addr)
            {
                printf("Error allocating memory for elf image!\n");
                return(0);
            }
            printf("Allocated memory for ELF segment: addr: 0x%llx, size 0x%x\n", phdr->p_paddr, (uint32_t)phdr->p_memsz);
        }
    }
    return 1;
}

U_BOOT_CMD(
	bootoctlinux,      32,      0,      do_bootoctlinux,
	"bootoctlinux - Boot from a linux ELF image in memory\n",
	"elf_address [coremask=mask_to_run | numcores=core_cnt_to_run] [forceboot] [skipcores=core_cnt_to_skip] [endbootargs] [app_args ...]\n"
    "elf_address - address of ELF image to load. If 0, default load address is used.\n"
    "coremask    - mask of cores to run on.  Anded with coremask_override environment\n"
    "              variable to ensure only working cores are used\n"
    "numcores    - number of cores to run on.  Runs on specified number of cores, taking into\n"
    "              account the coremask_override.\n"
    "skipcores   - only meaningful with numcores.  Skips this many cores (starting from 0) when\n"
    "              loading the numcores cores. For example, setting skipcores to 1 will skip core 0\n"
    "              and load the application starting at the next available core.\n"
    "forceboot   - if set, boots application even if core 0 is not in mask\n"
    "endbootargs - if set, bootloader does not process any further arguments and only passes\n"
    "              the arguments that follow to the kernel.  If not set, the kernel\n"
    "              gets the entire commnad line as arguments.\n"
    "\n"
);

U_BOOT_CMD(
	bootoctelf,      32,     0,      do_bootoctelf,
	"bootoctelf - Boot a generic ELF image in memory. NOTE: This command does not support\n"
        "             simple executive applications, use bootoct for those.\n",
	"elf_address [mem_size [coremask=mask_to_run | numcores=core_cnt_to_run] [skipcores=core_cnt_to_skip] [forceboot] ]\n"
    "elf_address - address of ELF image to load. If 0, default load address is used.\n"
    "mem_size    - amount of memory to reserve for ELF file.  Starts at ELF entry point rounded down to 1 MByte alignment.\n"
    "              No memory allocated if not specified.  (Program will not coexist with simple executive\n"
    "              applications or Linux if memory is not allocated properly.\n"
    "coremask    - mask of cores to run on.  Anded with coremask_override environment\n"
    "              variable to ensure only working cores are used.  Note: Most ELF files\n"
    "              are only suitable for booting on a single core.\n"
    "forceboot   - if set, boots application even if core 0 is not in mask\n"
    "numcores    - number of cores to run on.  Runs on specified number of cores, taking into\n"
    "              account the coremask_override.\n"
    "skipcores   - only meaningful with numcores.  Skips this many cores (starting from 0) when\n"
    "              loading the numcores cores. For example, setting skipcores to 1 will skip core 0\n"
    "              and load the application starting at the next available core.\n"
    "\n"
);
extern boot_info_block_t boot_info_block_array[];

#endif	/* CFG_CMD_ELF */
