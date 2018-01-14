/*
 * (C) Copyright 2004,2005
 * Cavium Networks
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


/**
 *
 * $Id: octeon_boot.c 58004 2011-03-14 19:43:27Z cchavva $
 *
 */


#if !defined(__U_BOOT__)
#include "bootoct.h"
#else
#include <common.h>
#include <command.h>
#include <exports.h>
#include <linux/ctype.h>
#include <lib_octeon.h>
#include <octeon_eeprom_types.h>
#include <lib_octeon_shared.h>
#include <environment.h>
#include <asm/mipsregs.h>
#include <asm/processor.h>
#include "octeon_boot.h"
#include "octeon-boot-info.h"
#include "cvmx-app-init.h"
#include "cvmx-bootmem.h"
#include "cvmx-sysinfo.h"

/************************************************************************/
/******* Global variable definitions ************************************/
/************************************************************************/
uint32_t cur_exception_base = EXCEPTION_BASE_INCR;
octeon_boot_descriptor_t boot_desc[16];
cvmx_bootinfo_t cvmx_bootinfo_array[16];

boot_info_block_t boot_info_block_array[16];
uint32_t coremask_to_run;
uint64_t boot_cycle_adjustment;
extern volatile int start_core0;
/************************************************************************/
/************************************************************************/


//#define DEBUG
#ifdef DEBUG
#define dprintf printf
#else
#define dprintf(...)
#endif


/************************************************************************/
uint32_t get_except_base_reg(void)
{
    uint32_t tmp;

    asm volatile (
    "    .set push              \n"
    "    .set mips64            \n"
    "    .set noreorder         \n"
    "    mfc0   %[tmp], $15, 1  \n"
    "    .set pop               \n"
     : [tmp] "=&r" (tmp) : );

    return(tmp);
}
void set_except_base_addr(uint32_t addr)
{

    asm volatile (
          "  .set push                  \n"
          "  .set mips64                \n"
          "  .set noreorder             \n"
          "  mtc0   %[addr], $15, 1     \n"
          "  .set pop                   \n"
         : :[addr] "r" (addr) );
}
uint64_t get_cop0_cvmctl_reg(void)
{
    uint32_t tmp_low, tmp_hi;

    asm volatile (
               "   .set push               \n"
               "   .set mips64                  \n"
               "   .set noreorder               \n"
               "   dmfc0 %[tmpl], $9, 7         \n"
               "   dadd   %[tmph], %[tmpl], $0   \n"
               "   dsrl  %[tmph], 32            \n"
               "   dsll  %[tmpl], 32            \n"
               "   dsrl  %[tmpl], 32            \n"
               "   .set pop                 \n"
                  : [tmpl] "=&r" (tmp_low), [tmph] "=&r" (tmp_hi) : );

    return(((uint64_t)tmp_hi << 32) + tmp_low);
}
void set_cop0_cvmctl_reg(uint64_t value)
{
    uint32_t val_low  = value & 0xffffffff;
    uint32_t val_high = value  >> 32;

    asm volatile (
        "  .set push                         \n"
        "  .set mips64                       \n"
        "  .set noreorder                    \n"
        /* Standard twin 32 bit -> 64 bit construction */
        "  dsll  %[valh], 32                 \n"
        "  dsll  %[vall], 32          \n"
        "  dsrl  %[vall], 32          \n"
        "  daddu %[valh], %[valh], %[vall]   \n"
        /* Combined value is in valh */
        "  dmtc0 %[valh],$9, 7               \n"
        "  .set pop                          \n"
         :: [valh] "r" (val_high), [vall] "r" (val_low) );


}
uint64_t get_cop0_cvmmemctl_reg(void)
{
    uint32_t tmp_low, tmp_hi;

    asm volatile (
               "   .set push                  \n"
               "   .set mips64                  \n"
               "   .set noreorder               \n"
               "   dmfc0 %[tmpl], $11, 7         \n"
               "   dadd   %[tmph], %[tmpl], $0   \n"
               "   dsrl  %[tmph], 32            \n"
               "   dsll  %[tmpl], 32            \n"
               "   dsrl  %[tmpl], 32            \n"
               "   .set pop                     \n"
                  : [tmpl] "=&r" (tmp_low), [tmph] "=&r" (tmp_hi) : );

    return(((uint64_t)tmp_hi << 32) + tmp_low);
}
void set_cop0_cvmmemctl_reg(uint64_t value)
{
    uint32_t val_low  = value & 0xffffffff;
    uint32_t val_high = value  >> 32;
    

    asm volatile (
        "  .set push                         \n"
        "  .set mips64                       \n"
        "  .set noreorder                    \n"
        /* Standard twin 32 bit -> 64 bit construction */
        "  dsll  %[valh], 32                 \n"
        "  dsll  %[vall], 32          \n"
        "  dsrl  %[vall], 32          \n"
        "  daddu %[valh], %[valh], %[vall]   \n"
        /* Combined value is in valh */
        "  dmtc0 %[valh],$11, 7               \n"
        "  .set pop                      \n"
         :: [valh] "r" (val_high), [vall] "r" (val_low) );


}

uint32_t get_core_num(void)
{

    return(0x3FF & get_except_base_reg());
}

uint32_t get_except_base_addr(void)
{

    return(~0x3FF & get_except_base_reg());
}

void copy_default_except_handlers(uint32_t addr_arg)
{
    uint8_t *addr = cvmx_phys_to_ptr(addr_arg);
    uint32_t nop_loop[2] = {0x1000ffff, 0x0};
    /* Set up some dummy loops for exception handlers to help with debug */
    memset(addr, 0, 0x800);
    memcpy(addr + 0x00, nop_loop, 8);
    memcpy(addr + 0x80, nop_loop, 8);
    memcpy(addr + 0x100, nop_loop, 8);
    memcpy(addr + 0x180, nop_loop, 8);
    memcpy(addr + 0x200, nop_loop, 8);
    memcpy(addr + 0x280, nop_loop, 8);
    memcpy(addr + 0x300, nop_loop, 8);
    memcpy(addr + 0x380, nop_loop, 8);
    memcpy(addr + 0x400, nop_loop, 8);
    memcpy(addr + 0x480, nop_loop, 8);
    memcpy(addr + 0x500, nop_loop, 8);
    memcpy(addr + 0x580, nop_loop, 8);
    memcpy(addr + 0x600, nop_loop, 8);
    memcpy(addr + 0x680, nop_loop, 8);


}
/**
 * Provide current cycle counter as a return value
 * 
 * @return current cycle counter
 */
uint64_t octeon_get_cycles(void)
{
    uint32_t tmp_low, tmp_hi;

    asm volatile (
               "   .set push                  \n"
               "   .set mips64                  \n"
               "   .set noreorder               \n"
               "   dmfc0 %[tmpl], $9, 6         \n"
               "   dadd   %[tmph], %[tmpl], $0   \n"
               "   dsrl  %[tmph], 32            \n"
               "   dsll  %[tmpl], 32            \n"
               "   dsrl  %[tmpl], 32            \n"
               "   .set pop                 \n"
                  : [tmpl] "=&r" (tmp_low), [tmph] "=&r" (tmp_hi) : );

    return(((uint64_t)tmp_hi << 32) + tmp_low);
}

void octeon_delay_cycles(uint64_t cycles)
{
    uint64_t start = octeon_get_cycles();
    while (start + cycles > octeon_get_cycles())
        ;
}


uint32_t octeon_get_cop0_status(void)
{
    uint32_t tmp;

    asm volatile (
               "   .set push                    \n"
               "   .set mips64                  \n"
               "   .set noreorder               \n"
               "   mfc0 %[tmp], $12, 0         \n"
               "   .set pop                     \n"
                  : [tmp] "=&r" (tmp) : );

    return tmp;
}
void cvmx_set_cycle(uint64_t cycle)
{
    uint32_t val_low  = cycle & 0xffffffff;
    uint32_t val_high = cycle  >> 32;

    asm volatile (
        "  .set push                         \n"
        "  .set mips64                       \n"
        "  .set noreorder                    \n"
        /* Standard twin 32 bit -> 64 bit construction */
        "  dsll  %[valh], 32                 \n"
        "  dsll  %[vall], 32          \n"
        "  dsrl  %[vall], 32          \n"
        "  daddu %[valh], %[valh], %[vall]   \n"
        /* Combined value is in valh */
        "  dmtc0 %[valh],$9, 6               \n"
        "  .set pop                          \n"
         :: [valh] "r" (val_high), [vall] "r" (val_low) );

}
#endif




#if CONFIG_OCTEON_SIM_SETUP
/* This address on the boot bus is used by the oct-sim script to pass
** flags to the bootloader.
*/
#define OCTEON_BOOTLOADER_FLAGS_ADDR    (0x9ffffff0ull)  /* 32 bit uint */
#endif


int octeon_setup_boot_desc_block(uint32_t core_mask, int argc, char **argv, uint64_t entry_point, uint32_t stack_size, uint32_t heap_size, uint32_t boot_flags, uint64_t stack_heep_base_addr, uint32_t image_flags, uint32_t config_flags) 
{
    /* Set up application descriptors for all cores running this application.
    ** These are only used during early boot - the simple exec init code copies any information
    ** of interest before the application starts.  This is stored in the top of the heap,
    ** and is guaranteed to be looked at before malloc is called.
    */
#if !defined(__U_BOOT__)
    cvmx_sysinfo_t *si = cvmx_sysinfo_get();
    linux_app_boot_info_t *labi;
    labi = &global_linux_boot_info->labi;
#else
    DECLARE_GLOBAL_DATA_PTR;
#endif
    /* Set up space for passing arguments to application */
    int argv_size = 0;
    int i;
    uint32_t argv_array[OCTEON_ARGV_MAX_ARGS] = {0};
    int64_t argv_data;

    for (i = 0; i < argc; i++)
        argv_size += strlen(argv[i]) + 1;
    
    /* Allocate permanent storage for the argv strings. 
    ** This must be from 32 bit addressable memory. */

    char tmp_block_name[512];
    sprintf(tmp_block_name, "argv_data_%08x", core_mask);
    argv_data = cvmx_bootmem_phy_named_block_alloc(argv_size + 64, 0ull, 0x7fffffffull, 0ull, tmp_block_name, 0);
    
    if (argv_data >= 0)
    {
        char *argv_ptr = cvmx_phys_to_ptr(argv_data);

        for (i = 0; i  < argc; i++)
        {
            /* Argv array is of 32 bit physical addresses */
            argv_array[i] = (uint32_t)argv_ptr;
#if !defined(__U_BOOT__)
            memcpy64((uint64_t)((uint32_t)argv_ptr), (uint64_t)((uint32_t)argv[i]), (strlen(argv[i]) + 1));
            argv_ptr += strlen(argv[i]) + 1;
#else
            argv_ptr = strcpy(argv_ptr, argv[i]) + strlen(argv[i]) + 1;
            argv_size += strlen(argv[i]) + 1;
#endif
        }
    }
    else
    {
        printf("ERROR: unable to allocate memory for argument data\n");
    }


#if OCTEON_APP_INIT_H_VERSION >= 1  /* The UART1 flag is new */
    /* If console is on uart 1 pass this to executive */
    {
#if defined(__U_BOOT__)
        DECLARE_GLOBAL_DATA_PTR;
#endif
        if (gd->console_uart == 1)
            boot_flags |= OCTEON_BL_FLAG_CONSOLE_UART1;
    }
#endif

#if !defined(__U_BOOT__)
    if(labi->pci_console_active)
#else
    if (getenv("pci_console_active"))
#endif
    {
#if OCTEON_APP_INIT_H_VERSION >= 2  /* The PCI flag is new */
        boot_flags |= OCTEON_BL_FLAG_CONSOLE_PCI;
#else
        boot_flags |=  1 << 4;
#endif
    }

    /* Bootloader boot_flags only used in simulation environment, default
    ** to no magic as required by hardware */
    boot_flags |= OCTEON_BL_FLAG_NO_MAGIC;

#if CONFIG_OCTEON_SIM_SETUP && !CONFIG_OCTEON_NO_BOOT_BUS
    /* Get debug and no_magic flags from oct-sim script if running on simulator */
    boot_flags &= ~(OCTEON_BL_FLAG_DEBUG | OCTEON_BL_FLAG_NO_MAGIC);
    boot_flags |= *(uint32_t *)OCTEON_BOOTLOADER_FLAGS_ADDR & (OCTEON_BL_FLAG_DEBUG | OCTEON_BL_FLAG_NO_MAGIC);
#endif

    int core;
    for (core = coremask_iter_init(core_mask); core >= 0; core = coremask_iter_next())
    {

#if (OCTEON_CURRENT_DESC_VERSION != 6) && (OCTEON_CURRENT_DESC_VERSION != 7)
#error Incorrect boot descriptor version for this bootloader release, check toolchain version!
#endif


#if !defined(__U_BOOT__)
        if (labi->icache_prefetch_disable)
#else
        if (getenv("icache_prefetch_disable"))
#endif
            boot_info_block_array[core].flags |= BOOT_INFO_FLAG_DISABLE_ICACHE_PREFETCH;
        boot_info_block_array[core].entry_point = entry_point;
        boot_info_block_array[core].exception_base = cur_exception_base;

        /* Align initial stack value to 16 byte alignment */
        boot_info_block_array[core].stack_top =  (stack_heep_base_addr + stack_size) & (~0xFull);

        if (image_flags & OCTEON_BOOT_DESC_IMAGE_LINUX)
        {
            int64_t addr;
            addr = cvmx_bootmem_phy_alloc(sizeof(boot_desc[0]), 0, 0x7fffffff, 0, 0);
            if (addr < 0)
            {
                printf("ERROR allocating memory for boot descriptor block\n");
                return(-1);
            }
            boot_info_block_array[core].boot_desc_addr = addr;
            addr = cvmx_bootmem_phy_alloc(sizeof(cvmx_bootinfo_t), 0, 0x7fffffff, 0, 0);
            if (addr < 0)
            {
                printf("ERROR allocating memory for cvmx bootinfo block\n");
                return(-1);
            }
            boot_desc[core].cvmx_desc_vaddr = addr;
        }
        else
        {
            /* For simple exec we put these in Heap mapping */
            boot_desc[core].heap_base = cvmx_bootinfo_array[core].heap_base 
                                      = stack_heep_base_addr + stack_size;
            boot_desc[core].heap_end  = cvmx_bootinfo_array[core].heap_end 
                                      = stack_heep_base_addr + stack_size + heap_size;
            /* Put boot descriptor at top of heap, align address */
            boot_info_block_array[core].boot_desc_addr = (boot_desc[core].heap_end - sizeof(boot_desc[0])) & ~0x7ULL;
            /* Put boot descriptor below boot descriptor near top of heap, align address */
            boot_desc[core].cvmx_desc_vaddr = (boot_info_block_array[core].boot_desc_addr - sizeof(cvmx_bootinfo_t)) & (~0x7ull);
        }


        /* The boot descriptor will be copied into the top of the heap for the core.  The
        ** application init code is responsible for copying the parts of the descriptor block
        ** that the application is interested in, as space is not reserved in the heap for
        ** this data.
        ** Note that the argv strings themselves are put into permanently allocated storage.
        */

        /* Most fields in the boot (app) descriptor are depricated, and have been
        ** moved to the cvmx_bootinfo structure.  They are set here to ease tranistion,
        ** but may be removed in the future.
        */

        boot_desc[core].argc = argc;
        memcpy(boot_desc[core].argv, argv_array, sizeof(argv_array));
        boot_desc[core].desc_version = OCTEON_CURRENT_DESC_VERSION;
        boot_desc[core].desc_size = sizeof(boot_desc[0]);
        /* boot_flags set later, copied from bootinfo block */
        boot_desc[core].debugger_flags_base_addr = BOOTLOADER_DEBUG_FLAGS_BASE;

        /* Set 'master' core for application.  This is the only core that will do certain
        ** init code such as copying the exception vectors for the app, etc
        ** NOTE: This is deprecated, and the application should use the coremask
        ** to choose a core to to setup with.
        */
        if (core == coremask_iter_get_first_core())
            cvmx_bootinfo_array[core].flags |= BOOT_FLAG_INIT_CORE;
        /* end of valid fields in boot descriptor */

#if (CVMX_BOOTINFO_MAJ_VER != 1)
#error Incorrect cvmx descriptor version for this bootloader release, check simple executive version!
#endif

        cvmx_bootinfo_array[core].flags |= boot_flags;
        cvmx_bootinfo_array[core].major_version = CVMX_BOOTINFO_MAJ_VER;
        cvmx_bootinfo_array[core].minor_version = CVMX_BOOTINFO_MIN_VER;
        cvmx_bootinfo_array[core].core_mask = core_mask;
        cvmx_bootinfo_array[core].dram_size = gd->ram_size/(1024 * 1024);  /* Convert from bytes to Megabytes */
#if !defined(__U_BOOT__)
        cvmx_bootinfo_array[core].phy_mem_desc_addr = si->phy_mem_desc_addr;
#else
        cvmx_bootinfo_array[core].phy_mem_desc_addr = ((uint32_t)__cvmx_bootmem_internal_get_desc_ptr()) & 0x7FFFFFF;
#endif
        cvmx_bootinfo_array[core].exception_base_addr = cur_exception_base;
        cvmx_bootinfo_array[core].dclock_hz = gd->ddr_clock_mhz * 1000000;
        cvmx_bootinfo_array[core].eclock_hz = gd->cpu_clock_mhz * 1000000;
        cvmx_bootinfo_array[core].board_type = gd->board_desc.board_type;
        cvmx_bootinfo_array[core].board_rev_major = gd->board_desc.rev_major;
        cvmx_bootinfo_array[core].board_rev_minor = gd->board_desc.rev_minor;
        cvmx_bootinfo_array[core].mac_addr_count = gd->mac_desc.count;
        cvmx_bootinfo_array[core].stack_top =  boot_info_block_array[core].stack_top;
        cvmx_bootinfo_array[core].stack_size =  stack_size;
        memcpy(cvmx_bootinfo_array[core].mac_addr_base, (void *)gd->mac_desc.mac_addr_base, sizeof(cvmx_bootinfo_array[core].mac_addr_base));
        strncpy(cvmx_bootinfo_array[core].board_serial_number, (char *)(gd->board_desc.serial_str),CVMX_BOOTINFO_OCTEON_SERIAL_LEN);

        dprintf("board type is: %d, %s\n", cvmx_bootinfo_array[core].board_type, cvmx_board_type_to_string(cvmx_bootinfo_array[core].board_type));
#if (CVMX_BOOTINFO_MIN_VER >= 1)

#if !defined(__U_BOOT__)
        cvmx_bootinfo_array[core].compact_flash_common_base_addr = labi->compact_flash_common_base_addr;
        cvmx_bootinfo_array[core].compact_flash_attribute_base_addr = labi->compact_flash_attribute_base_addr;
        cvmx_bootinfo_array[core].led_display_base_addr = labi->led_display_base_addr;
#else
#ifdef OCTEON_CF_COMMON_BASE_ADDR
        cvmx_bootinfo_array[core].compact_flash_common_base_addr = OCTEON_CF_COMMON_BASE_ADDR;
#else
        cvmx_bootinfo_array[core].compact_flash_common_base_addr = 0;
#endif
#ifdef OCTEON_CF_ATTRIB_BASE_ADDR
        cvmx_bootinfo_array[core].compact_flash_attribute_base_addr = OCTEON_CF_ATTRIB_BASE_ADDR;
#else
        cvmx_bootinfo_array[core].compact_flash_attribute_base_addr = 0;
#endif


#ifdef OCTEON_CF_TRUE_IDE_CS0_ADDR
        /* Use true IDE mode interface when defined */
        cvmx_bootinfo_array[core].compact_flash_common_base_addr = OCTEON_CF_TRUE_IDE_CS0_ADDR;
#ifdef OCTEON_CF_TRUE_IDE_CS1_ADDR
        cvmx_bootinfo_array[core].compact_flash_attribute_base_addr = OCTEON_CF_TRUE_IDE_CS1_ADDR;
#endif
#endif

#if CONFIG_OCTEON_EBT3000
        /* Rev 1 boards use 4 segment display at slightly different address */
        if (gd->board_desc.rev_major == 1)
            cvmx_bootinfo_array[core].led_display_base_addr  = OCTEON_CHAR_LED_BASE_ADDR;
        else
            cvmx_bootinfo_array[core].led_display_base_addr  = OCTEON_CHAR_LED_BASE_ADDR + 0xf8;
#else
#ifdef OCTEON_CHAR_LED_BASE_ADDR
        if (OCTEON_CHAR_LED_BASE_ADDR)
            cvmx_bootinfo_array[core].led_display_base_addr = OCTEON_CHAR_LED_BASE_ADDR + 0xf8;
        else
#endif
            cvmx_bootinfo_array[core].led_display_base_addr = 0;
#endif
#endif

#endif  /* (CVMX_BOOTINFO_MAJ_VER >= 1) */

#if (CVMX_BOOTINFO_MIN_VER >= 2)
        /* Set DFA reference if available */
        cvmx_bootinfo_array[core].dfa_ref_clock_hz = gd->clock_desc.dfa_ref_clock_mhz_x_8/8*1000000;

        /* Copy boot_flags settings to config flags */
        if (boot_flags & OCTEON_BL_FLAG_DEBUG)
            cvmx_bootinfo_array[core].config_flags |= CVMX_BOOTINFO_CFG_FLAG_DEBUG;
#if OCTEON_APP_INIT_H_VERSION >= 3
        if (boot_flags & OCTEON_BL_FLAG_BREAK)
            cvmx_bootinfo_array[core].config_flags |= CVMX_BOOTINFO_CFG_FLAG_BREAK;
#endif
        if (boot_flags & OCTEON_BL_FLAG_NO_MAGIC)
            cvmx_bootinfo_array[core].config_flags |= CVMX_BOOTINFO_CFG_FLAG_NO_MAGIC;
        /* Set config flags */
        if (CONFIG_OCTEON_PCI_HOST)
            cvmx_bootinfo_array[core].config_flags |= CVMX_BOOTINFO_CFG_FLAG_PCI_HOST;
#ifdef CONFIG_OCTEON_PCI_TARGET
        else if (CONFIG_OCTEON_PCI_TARGET)
            cvmx_bootinfo_array[core].config_flags |= CVMX_BOOTINFO_CFG_FLAG_PCI_TARGET;
#endif

        cvmx_bootinfo_array[core].config_flags |= config_flags;
#endif  /* (CVMX_BOOTINFO_MAJ_VER >= 2) */

        /* Fill in remaining fields of boot_desc array */
        boot_desc[core].core_mask       = cvmx_bootinfo_array[core].core_mask;       
        boot_desc[core].flags           = cvmx_bootinfo_array[core].flags;       
        boot_desc[core].eclock_hz       = cvmx_bootinfo_array[core].eclock_hz;       

        /* Copy deprecated fields from cvmx_bootinfo array to boot descriptor for compatibility.  */
        boot_desc[core].dram_size       = cvmx_bootinfo_array[core].dram_size; /* Erroneously used in SDK 1.8.1, so keep for a while */
        /* End deprecated boot descriptor fields */

        if (image_flags & OCTEON_BOOT_DESC_IMAGE_LINUX)
        {
            // copy boot desc for use by app
            memcpy((void *)(uint32_t)boot_info_block_array[core].boot_desc_addr, (void *)&boot_desc[core], boot_desc[core].desc_size);
            memcpy((void *)(uint32_t)boot_desc[core].cvmx_desc_vaddr, (void *)&cvmx_bootinfo_array[core], sizeof(cvmx_bootinfo_t));
        }
        else
        {
            /* Copy boot descriptor and cmvx_bootinfo structures to correct virtual address
            ** for the application to read them.         */
#if !defined(__U_BOOT__)
            mem_copy_tlb(boot_info_block_array[core].tlb_entries, boot_info_block_array[core].boot_desc_addr, &boot_desc[core], boot_desc[core].desc_size);
            mem_copy_tlb(boot_info_block_array[core].tlb_entries, boot_desc[core].cvmx_desc_vaddr, &cvmx_bootinfo_array[core], sizeof(cvmx_bootinfo_t));
#else   
            /* Cast to int32_t, as we need sign extension in the address */
            mem_copy_tlb(boot_info_block_array[core].tlb_entries, boot_info_block_array[core].boot_desc_addr, (int32_t)(&boot_desc[core]), boot_desc[core].desc_size);
            mem_copy_tlb(boot_info_block_array[core].tlb_entries, boot_desc[core].cvmx_desc_vaddr,  (int32_t)(&cvmx_bootinfo_array[core]), sizeof(cvmx_bootinfo_t));
#endif
        }

    }
    copy_default_except_handlers(cur_exception_base);
    cur_exception_base += EXCEPTION_BASE_INCR;
#if !defined(__U_BOOT__)
    labi->cur_exception_base += EXCEPTION_BASE_INCR;
#endif

    dprintf("stack expected: 0x%llx, actual: 0x%llx\n", stack_heep_base_addr + stack_size, boot_desc[0].stack_top);
    dprintf("heap_base expected: 0x%llx, actual: 0x%llx\n", stack_heep_base_addr + stack_size, boot_desc[0].heap_base);
    dprintf("heap_top expected: 0x%llx, actual: 0x%llx\n", stack_heep_base_addr + stack_size + heap_size, boot_desc[0].heap_end);
    dprintf("Entry point (virt): 0x%llx\n", entry_point);

    return(0);
}









uint32_t coremask_iter_core;
uint32_t coremask_iter_mask;
int coremask_iter_first_core;

int coremask_iter_init(uint32_t mask)
{
    coremask_iter_mask = mask;
    coremask_iter_core = 0;
    coremask_iter_first_core = -1;  /* Set to invalid value */
    return(coremask_iter_next());
}
int coremask_iter_next(void)
{

    while (!((1 << coremask_iter_core) & coremask_iter_mask) && coremask_iter_core < 16)
        coremask_iter_core++;
    if (coremask_iter_core > 15)
        return(-1);
    
    /* Set first core */
    if (coremask_iter_first_core < 0)
        coremask_iter_first_core = coremask_iter_core;
    
    return(coremask_iter_core++);
}
int coremask_iter_get_first_core(void)
{
    return(coremask_iter_first_core);
}


#if defined(__U_BOOT__)
#if !CONFIG_OCTEON_SIM_NO_FLASH
int octeon_cf_present(void)
{
#ifdef OCTEON_CF_ATTRIB_CHIP_SEL
    uint32_t *ptr = (void *)(OCTEON_CF_COMMON_BASE_ADDR);
    return (*ptr != 0xffffffff);
#else
    return 0;
#endif
}
#endif



/* octeon_write64_byte and octeon_read64_byte are only used by the debugger stub.
** The debugger will generate KSEG0 addresses that are not in the 64 bit compatibility
** space, so we detect that and fix it up.  This should probably be addressed in the 
** debugger itself, as this fixup makes some valid 64 bit address inaccessible.
*/
#define DO_COMPAT_ADDR_FIX
void octeon_write64_byte(uint64_t csr_addr, uint8_t val)
{

    volatile uint32_t addr_low  = csr_addr & 0xffffffff;
    volatile uint32_t addr_high = csr_addr  >> 32;
    
    if (!addr_high && (addr_low & 0x80000000))
    {
#ifdef DO_COMPAT_ADDR_FIX
        addr_high = ~0;
#endif
#if 0
        char tmp_str[500];
        sprintf(tmp_str, "fixed read64_byte at low  addr: 0x%x\n", addr_low);
        simprintf(tmp_str);
        sprintf(tmp_str, "fixed read64_byte at high addr: 0x%x\n", addr_high);
        simprintf(tmp_str);
#endif

    }

    asm volatile (
      "  .set push                         \n"
      "  .set mips64                       \n"
      "  .set noreorder                    \n"
      /* Standard twin 32 bit -> 64 bit construction */
      "  dsll  %[addrh], 32                 \n"
      "  dsll  %[addrl], 32          \n"
      "  dsrl  %[addrl], 32          \n"
      "  daddu %[addrh], %[addrh], %[addrl]   \n"
      /* Combined value is in addrh */
      "  sb %[val], 0(%[addrh])   \n"
      "  .set pop                          \n"
      : :[val] "r" (val), [addrh] "r" (addr_high), [addrl] "r" (addr_low): "memory");


}

uint8_t octeon_read64_byte(uint64_t csr_addr)
{
    
    uint8_t val;

    uint32_t addr_low  = csr_addr & 0xffffffff;
    uint32_t addr_high = csr_addr  >> 32;

    if (!addr_high && (addr_low & 0x80000000))
    {
#ifdef DO_COMPAT_ADDR_FIX
        addr_high = ~0;
#endif
#if 0
        char tmp_str[500];
        sprintf(tmp_str, "fixed read64_byte at low  addr: 0x%x\n", addr_low);
        simprintf(tmp_str);
        sprintf(tmp_str, "fixed read64_byte at high addr: 0x%x\n", addr_high);
        simprintf(tmp_str);
#endif

    }


    asm volatile (
                  "  .set push                         \n"
                  "  .set mips64                       \n"
                  "  .set noreorder                    \n"
                  /* Standard twin 32 bit -> 64 bit construction */
                  "  dsll  %[addrh], 32                 \n"
                  "   dsll  %[addrl], 32          \n"
                  "  dsrl  %[addrl], 32          \n"
                  "  daddu %[addrh], %[addrh], %[addrl]   \n"
                  /* Combined value is in addrh */
                  "  lb    %[val], 0(%[addrh])   \n"
                  "  .set pop                          \n"
                  :[val] "=&r" (val) : [addrh] "r" (addr_high), [addrl] "r" (addr_low): "memory");

    return(val);

}


/**
 * Memset in 64 bit chunks to a 64 bit addresses.  Needs
 * wrapper to be useful.
 * 
 * @param start_addr start address (must be 8 byte alligned)
 * @param val        value to write
 * @param byte_count number of bytes - must be multiple of 8.
 */
static void octeon_memset64_64only(uint64_t start_addr_arg, uint8_t val, uint64_t byte_count_arg)
{
    volatile uint64_t byte_count = byte_count_arg;
    volatile uint64_t start_addr = start_addr_arg;
    if (byte_count & 0x7)
        return;

    if (!byte_count)
        return;

    volatile uint32_t val_low = val | ((uint32_t)val << 8) | ((uint32_t)val << 16) | ((uint32_t)val << 24);

    volatile uint64_t val64 = ((uint64_t)val_low << 32) | val_low;

#if 0
    printf("octeon_memset64_64only: start: 0x%qx, val: 0x%qx, count: 0x%qx\n", start_addr, val64, byte_count);
#endif

    asm volatile (
      "  .set push                         \n"
      "  .set mips64                       \n"
      "  .set noreorder                    \n"
        /* Now do real work..... */
      "  1:                               \n" 
        "    daddi  %[cnt], -8              \n"
      "      sd    %[val], 0(%[addr])      \n"
      "      bne   $0, %[cnt], 1b          \n"
      "      daddi  %[addr], 8              \n"
      "  .set pop                           \n"
      : : [cnt] "r" (byte_count), [addr] "r" (start_addr),[val] "r" (val64): "memory");

}

/**
 * memcpy in 64 bit chunks to a 64 bit addresses.  Needs wrapper
 * to be useful.
 * 
 * @param dest_addr destination address (must be 8 byte alligned)
 * @param src_addr destination address (must be 8 byte alligned)
 * @param byte_count number of bytes - must be multiple of 8.
 */
static void octeon_memcpy64_64only(uint64_t dest_addr, uint64_t src_addr, uint64_t byte_count)
{
    if (byte_count & 0x7)
        return;

    volatile uint32_t count_low  = byte_count & 0xffffffff;
    volatile uint32_t count_high = byte_count  >> 32;

    volatile uint32_t src_low  = src_addr & 0xffffffff;
    volatile uint32_t src_high = src_addr  >> 32;
    volatile uint32_t dest_low  = dest_addr & 0xffffffff;
    volatile uint32_t dest_high = dest_addr  >> 32;
    uint32_t tmp;


    asm volatile (
      "  .set push                         \n"
      "  .set mips64                       \n"
      "  .set noreorder                    \n"
      /* Standard twin 32 bit -> 64 bit construction */
      "  dsll  %[cnth], 32                 \n"
      "  dsll  %[cntl], 32          \n"
      "  dsrl  %[cntl], 32          \n"
      "  daddu %[cnth], %[cnth], %[cntl]   \n"
      /* Combined value is in cnth */
      /* Standard twin 32 bit -> 64 bit construction */
      "  dsll  %[srch], 32                 \n"
      "  dsll  %[srcl], 32          \n"
      "  dsrl  %[srcl], 32          \n"
      "  daddu %[srch], %[srch], %[srcl]   \n"
      /* Combined value is in srch */
      /* Standard twin 32 bit -> 64 bit construction */
      "  dsll  %[dsth], 32                 \n"
      "  dsll  %[dstl], 32          \n"
      "  dsrl  %[dstl], 32          \n"
      "  daddu %[dsth], %[dsth], %[dstl]   \n"
      /* Combined value is in dsth */ 
        /* Now do real work..... */
      "  1:                               \n" 
      "      ld    %[tmp], 0(%[srch])      \n"
      "      daddi  %[cnth], -8              \n"
      "      sd    %[tmp], 0(%[dsth])      \n"
      "      daddi  %[dsth], 8              \n"
      "      bne   $0, %[cnth], 1b          \n"
      "      daddi  %[srch], 8              \n"
      "  .set pop                           \n"
      : [tmp] "=&r" (tmp): [cnth] "r" (count_high), [cntl] "r" (count_low), [dsth] "r" (dest_high), [dstl] "r" (dest_low), [srcl] "r" (src_low), [srch] "r" (src_high): "memory");


}

uint64_t memcpy64(uint64_t dest_addr, uint64_t src_addr, uint64_t count)
{

    /* Caller must make sure that the 64 bit src/dest addresses are
    ** valid.  They are directly accessed in this function */

    uint64_t to_copy = count;

    if ((src_addr & 0x7) != (dest_addr & 0x7))
    {
        while (to_copy--)
        {
            cvmx_write64_uint8(dest_addr++, cvmx_read64_uint8(src_addr++));
        }
    }
    else
    {
        while ((dest_addr & 0x7) && to_copy)
        {
            to_copy--;
            cvmx_write64_uint8(dest_addr++, cvmx_read64_uint8(src_addr++));
        }
        if (to_copy >= 8)
        {
        octeon_memcpy64_64only(dest_addr, src_addr, to_copy & ~0x7ull);
        dest_addr += to_copy & ~0x7ull;
        src_addr += to_copy & ~0x7ull;
        to_copy &= 0x7ull;
        }
        while (to_copy--)
        {
            cvmx_write64_uint8(dest_addr++, cvmx_read64_uint8(src_addr++));
        }
    }
    return count;
}



#if 0
/* 63xx_Hack - takes 64 bit addresses, but must be directly accessible */
void checkmem64(uint64_t start_addr, uint8_t value, int len)
{
    int error_count = 0;
    int bad_flag = 0;
    uint8_t *ptr = cvmx_phys_to_ptr(start_addr);
    uint8_t *end_ptr = ptr + len;
    uint8_t *change_ptr = ptr;

    printf("Checking %d bytes at addr: %p\n", len, ptr);
    while (ptr < end_ptr)
    {
        if (*ptr != value)
        {
            error_count++;
            if (!bad_flag)
            {
                bad_flag = 1;
                printf("addr %p bad (0x%x), prev good addr: %p\n", ptr, *ptr, change_ptr);
                change_ptr = ptr;
            }
        }
        else
        {
            if (bad_flag)
            {
                bad_flag = 0;
                printf("addr %p good (0x%x), prev bad addr: %p\n", ptr, *ptr, change_ptr);
                change_ptr = ptr;
            }
        }

        ptr++;
    }

    printf("total errors: %d, start: %p, len: %d\n", error_count, cvmx_phys_to_ptr(start_addr), len);


}
#endif

void memset64(uint64_t start_addr, uint8_t value, uint64_t len)
{

    if (!(start_addr & 0xffffffff00000000ull) && (start_addr & (1 << 31)))
        start_addr |= 0xffffffff00000000ull;
    start_addr = MAKE_XKPHYS(start_addr);

    while ((start_addr & 0x7) && len-- > 0)
    {
        cvmx_write64_uint8(start_addr++,value);
    }
    if (len >= 8)
    {
        octeon_memset64_64only(start_addr,value, len & ~0x7ull);
        start_addr += len & ~0x7ull;
        len &= 0x7;
    }
    while (len-- > 0)
    {
        cvmx_write64_uint8(start_addr++,value);
    }

}
#endif


uint32_t mem_copy_tlb(octeon_tlb_entry_t_bl *tlb_array, uint64_t dest_virt, uint64_t src_addr, uint32_t len)
{

    uint64_t cur_src;
    uint32_t cur_len;
    uint64_t dest_phys;
    uint32_t chunk_len;
    int tlb_index;
    /* Caller must make sure that src_addr address is valid - it is used directly */
    cur_src = src_addr;


    cur_len = len;
    dprintf("tlb_ptr: %p, dvirt: 0x%llx, sphy: 0x%llx, len: %d\n", tlb_array, dest_virt, src_addr, len);

    for (tlb_index = 0; tlb_index < get_num_tlb_entries() && cur_len > 0; tlb_index++)
    {
        uint32_t page_size = TLB_PAGE_SIZE_FROM_EXP(tlb_array[tlb_index].page_size);


        /* We found a page0 mapping that contains the start of the block */
        if (dest_virt >= (tlb_array[tlb_index].virt_base) &&  dest_virt < (tlb_array[tlb_index].virt_base + page_size))
        {
            dest_phys = tlb_array[tlb_index].phy_page0 + (dest_virt - tlb_array[tlb_index].virt_base);
            chunk_len = MIN((tlb_array[tlb_index].virt_base + page_size) - dest_virt, cur_len);


            dest_phys = MAKE_XKPHYS(dest_phys);
            dprintf("Copying(0) 0x%x bytes from 0x%llx to 0x%llx\n", chunk_len, cur_src, dest_phys);
            memcpy64(dest_phys, cur_src, chunk_len);

            cur_len -= chunk_len;
            dest_virt += chunk_len;
            cur_src += chunk_len;


        }
        /* We found a page1 mapping that contains the start of the block */
        if (cur_len > 0 && dest_virt >= (tlb_array[tlb_index].virt_base + page_size) &&  dest_virt < (tlb_array[tlb_index].virt_base + 2*page_size))
        {
            dest_phys = tlb_array[tlb_index].phy_page1 + (dest_virt - (tlb_array[tlb_index].virt_base + page_size));
            chunk_len = MIN((tlb_array[tlb_index].virt_base + 2 * page_size) - dest_virt, cur_len);

            dest_phys = MAKE_XKPHYS(dest_phys);
            dprintf("Copying(1) 0x%x bytes from 0x%llx to 0x%llx\n", chunk_len, cur_src, dest_phys);
            memcpy64(dest_phys, cur_src, chunk_len);
            cur_len -= chunk_len;
            dest_virt += chunk_len;
            cur_src += chunk_len;



        }
    }
    if (cur_len != 0)
    {
        printf("ERROR copying memory using TLB mappings, cur_len: %d !\n", cur_len);
        return(~0UL);
    }
    else
    {
        return(len);
    }
    

  
}


#if defined(__U_BOOT__)
/**
 * Specialized function for quickly clearing DRAM during boot.
 * Clears memory in 1kbyte chunks, as this speeds things up when
 * clearing DRAM while running from flash.
 * Caller must provide cache line aligned address, and count that is
 * multiple of OCTEON_BZERO_PFS_STRIDE Bytes.
 * 
 * @param start_addr Start address.  Must be cache line aligned.
 *                   
 * @param count      Number of bytes to clear.  Must be multiple of OCTEON_BZERO_PFS_STRIDE.
 */
void octeon_bzero64_pfs(uint64_t start_addr_arg, uint64_t count_arg)
{
    // not sure why the volatiles need to be here
    // without them calling memset in function breaks - wrong args given
    volatile uint64_t count = count_arg;
    volatile uint64_t start_addr = start_addr_arg;
    uint32_t extra_bytes = 0;
    uint64_t extra_addr;

    if (count == 0)
        return;

    if (start_addr & CVMX_CACHE_LINE_MASK)
    {
        printf("ERROR: bad alignment in octeon_bzero64_pfs\n");
        return;
    }

    /* Handle lengths that aren't a multiple of the stride by doing
    ** a normal memset on the remaining (small) amount of memory */
    extra_bytes = count & OCTEON_BZERO_PFS_STRIDE_MASK;
    count -= extra_bytes;
    extra_addr = start_addr_arg + count;

#if 0
    printf("octeon_bzero64_pfs: start: 0x%qx, count: 0x%qx\n", start_addr, count);
#endif

    if (OCTEON_IS_MODEL(OCTEON_CN6XXX))
    {
        /* NOTE: All prefetches (including prepare for store) are dropped when STATUS[ERL] == 1,
        ** so prepare for store _doesn't_ work here. */
        asm volatile (
          "  .set push                   \n"
          "  .set arch=octeon2                   \n"
          "  .set noreorder                    \n"
            /* Now do real work..... */
          " 1:                               \n" 
          "  daddi  %[cnt], -128              \n"
          "  zcbt   (%[addr])      \n"
          "  bne    $0, %[cnt], 1b          \n"
          "  daddi  %[addr],128              \n"
          "  .set pop                       \n"
          : [cnt] "+r" (count), [addr] "+r" (start_addr) : : "memory");
    }
    else
    {
#if !CONFIG_OCTEON_SIM_HW_DIFF
/* The simulator is pessimistic in its simulation of prepare for store operations, and
** randomizes the cache lines.  U-boot uses it in a manner where we do get all zeros,
** but we can't run this code in simulation */
        asm volatile (
          "  .set push                   \n"
          "  .set mips64                       \n"
          "  .set noreorder                    \n"
            /* Now do real work..... */
          " 1:                               \n" 
          "  daddi  %[cnt], -1024              \n"
          "  pref 30, 0(%[addr])      \n"
          "  pref 30, 128(%[addr])      \n"
          "  pref 30, 256(%[addr])      \n"
          "  pref 30, 384(%[addr])      \n"
          "  pref 30, 512(%[addr])      \n"
          "  pref 30, 640(%[addr])      \n"
          "  pref 30, 768(%[addr])      \n"
          "  pref 30, 896(%[addr])      \n"
          "  bne   $0, %[cnt], 1b          \n"
          "  daddi  %[addr],1024              \n"
          "  .set pop                       \n"
          : [cnt] "+r" (count), [addr] "+r" (start_addr) : : "memory");

        /* Invalidate the DCACHE, as the PFS will zero in L2 the 
        ** memory, but won't invalidate the local core's DCACHE. */
        CVMX_DCACHE_INVALIDATE;
#else
        memset64(start_addr, 0, count);
#endif
    }



    if (extra_bytes)
        memset64(extra_addr, 0, extra_bytes);

}

uint64_t uboot_tlb_ptr_to_phys(void *ptr)
{
    uint64_t eh_val = (int)ptr;

    /* Clear low bits of address to properly form entry_hi */
    eh_val &= ~0ull << 13;

    /* Here we need to probe the TLB and determine what
    ** phsical address coresponds to the provided pointer.
    ** This needs to work both when running from flash and
    ** when running from DRAM.
    */
    write_64bit_cp0_register(CP0_ENTRYHI, eh_val);
    __asm__ __volatile__("tlbp");
    int index = read_32bit_cp0_register(CP0_INDEX);

    /* Return all 0xdeadbeefdeadbeef on error (0 is valid, so can't use that */
    if (index <0)
        return(0xdeadbeefdeadbeefULL);

    /* Read matching TLB entry */
    __asm__ __volatile__("tlbr");

    uint64_t lo0 = (read_64bit_cp0_register(CP0_ENTRYLO0) & ~(0x7ULL)) << 6;
    uint64_t lo1 = (read_64bit_cp0_register(CP0_ENTRYLO1) & ~(0x7ULL)) << 6;
    uint32_t pagemask = read_32bit_cp0_register(CP0_PAGEMASK);
    pagemask |= 0x1fff;  /* Fill in low bits of pagemask */

    /* First get offset into mapping, to determine which physical
    ** page it is in. Pagemask is mask for physical page size, and each
    ** TLB entry maps two pages (lo0/lo1). */
    uint32_t offset = (uint32_t)ptr & pagemask;
    uint64_t physical_addr = lo0;
    if (offset > (pagemask >> 1))
        physical_addr = lo1;

    physical_addr += (uint32_t)ptr & (pagemask >> 1);

    return(physical_addr);
}

extern void idle_core_start_app(void);  /* in cmd_octeon.c */
extern void InitTLBStart(void);  /* in start.S */
extern void OcteonBreak(void);  /* in start.S */
int octeon_setup_boot_vector(uint32_t func_addr, uint32_t core_mask)
{
    DECLARE_GLOBAL_DATA_PTR;
    // setup boot vectors for other cores.....
    int core;
    boot_init_vector_t *boot_vect = (boot_init_vector_t *)(BOOT_VECTOR_BASE);

    for (core = coremask_iter_init(core_mask); core >= 0; core = coremask_iter_next())
    {
        /* Have other cores jump directly to DRAM, rather than running out of flash.  This
        ** address is set here because is is the address in DRAM, rather than the address in FLASH
        */

#if 0
        printf("u-boot ram base: 0x%qx\n", gd->bd->bi_uboot_ram_addr);

        printf("address of initTLBSTart: %p, start: %p\n", &InitTLBStart, &uboot_start);

        printf("addr to jump to: 0x%qx\n", gd->bd->bi_uboot_ram_addr  + ((char *)&InitTLBStart - (char *)&uboot_start));
#endif
#if !CONFIG_OCTEON_UBOOT_TLB
        boot_vect[core].code_addr = (uint32_t)&InitTLBStart;
#else
        /* Here we need to figure out the physical address of the 
         ** InitTLBStart function. */
        boot_vect[core].code_addr = MAKE_XKPHYS(uboot_tlb_ptr_to_phys(&InitTLBStart));
        dprintf("Setting core %d code_addr address(0x%p) to: 0x%qx\n", core, &(boot_vect[core].code_addr), boot_vect[core].code_addr);
#endif
        if (func_addr)
        {
            /* The app_start_func_addr will be a virtual address that is used
            ** after the TLB has been set up. */
            boot_vect[core].app_start_func_addr = func_addr;
//            printf("Setting core %d app_start address(0x%p) to: 0x%qx\n", core, &(boot_vect[core].app_start_func_addr), boot_vect[core].app_start_func_addr);
        }
        else
        {
            /* If fewer than all cores are run, set rest to stop, otherwise
            ** simulator runs forever */
            boot_vect[core].app_start_func_addr = (uint32_t)&idle_core_start_app;
        }

        boot_vect[core].k0_val = (uint32_t)gd;  /* pass global data pointer */

        boot_vect[core].boot_info_addr = (uint32_t)(&boot_info_block_array[core]);
        dprintf("boot_info_addr: 0x%qx\n", boot_vect[core].boot_info_addr);

    }

    dprintf("Address of start app: 0x%llx\n", boot_vect[coremask_iter_get_first_core()].app_start_func_addr);

    OCTEON_SYNC;
    return(0);
}


uint32_t get_coremask_override(void)
{
    uint32_t coremask_override = 0xffff;
    char *cptr;
    cptr = getenv("coremask_override");
    if (cptr)
    {
        coremask_override = simple_strtol(cptr, NULL, 16);
    }
    return(coremask_override);

}
#define OCTEON_POP(result, input) asm ("pop %[rd],%[rs]" : [rd] "=d" (result) : [rs] "d" (input))
/**
 * Generates a coremask with a given number of cores in it given
 * the coremask value (defines the available cores) and the
 * number of cores requested.
 * 
 * @param num_cores number of cores requested
 * 
 * @return coremask with requested number of cores, or all that are available.
 *         Prints a warning if fewer than the requested number of cores are returned.
 */
uint32_t octeon_coremask_num_cores(int num_cores)
{
    int shift;
    uint32_t mask=1;
    int cores;

    if (!num_cores)
        return(0);

    uint32_t coremask_override = octeon_get_available_coremask();
    for (shift = 1;shift <=16; shift++)
    {
        OCTEON_POP(cores,coremask_override & mask);
        if (cores == num_cores)
            return(coremask_override & mask);
        mask = (mask << 1) | 1;
    }
    printf("####################################################################################\n");
    printf("### Warning: only %02d cores available, not running with requested number of cores ###\n", cores);
    printf("####################################################################################\n");
    return(coremask_override & mask);

}


/* Save a copy of the coremask from the eeprom so that we can print a warning
** if a user enables broken cores.  This is set to the value read from the eeprom
** when the eeprom is read in board.c. */
uint32_t coremask_from_eeprom = 0xffff;  /* Default value, changed when read from eeprom */

/* Validate the coremask that is passed to a boot* function. */
int32_t validate_coremask(uint32_t core_mask)
{
    uint32_t coremask_override;
    coremask_override = get_coremask_override();

    uint32_t fuse_coremask = octeon_get_available_coremask();

    if ((core_mask & fuse_coremask) != core_mask)
    {
        printf("ERROR: Can't boot cores that don't exist! (available coremask: 0x%x)\n", fuse_coremask);
        return(-1);
    }

    if ((core_mask & coremask_override) != core_mask)
    {
        core_mask &= coremask_override;
        printf("Notice: coremask changed to 0x%x based on coremask_override of 0x%x\n", core_mask, coremask_override);
    }

    if (core_mask & ~coremask_from_eeprom)
    {
        /* The user has manually changed the coremask_override env. variable to run code
        ** on known bad cores.  Print a warning message.
        */
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING: You have changed the coremask_override and are running code on non-functional cores.\n");
        printf("WARNING: The program may crash and/or produce unreliable results.\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        printf("WARNING:\n");
        int delay = 5;
        do
        {
            printf("%d\n", delay);
            cvmx_wait(600000000ULL);  /* Wait a while to make sure warning is seen. */
        } while (delay-- > 0);
    }
    return core_mask;
}

/* Returns the available coremask either from env or fuses.
** If the fuses are blown and locked, they are the definitive coremask.*/
uint32_t octeon_get_available_coremask(void)
{
    uint32_t cores;
    uint32_t ciu_fuse = (uint32_t)(cvmx_read_csr(CVMX_CIU_FUSE) & 0xffff);

    if (octeon_is_model(OCTEON_CN38XX))
    {
        /* Here we only need it if no core fuses are blown and the lockdown fuse is not blown.
        ** In all other cases the cores fuses are definitive and we don't need a coremask override
        */
        if (ciu_fuse == 0xffff && !cvmx_octeon_fuse_locked())
        {
            /* get coremask from environment */
            return(get_coremask_override());
        }
    }
    else if (octeon_is_model(OCTEON_CN58XX)|| octeon_is_model(OCTEON_CN56XX))
    {
        /* Some early chips did not have fuses blown to follow the even/odd
        ** requirements.  Set the coremask to take this possibility into account. */
        uint32_t coremask = 0;

        /* Do even cores */
        OCTEON_POP(cores, ciu_fuse & 0x5555);
        coremask |= 0x5555 >> (16 - 2*cores);
        /* Do odd cores */
        OCTEON_POP(cores, ciu_fuse & 0xAAAA);
        coremask |= 0xAAAA >> (16 - 2*cores);
//        printf("CN58XX fuse: 0x%04x, coremask: 0x%04x\n", ciu_fuse, coremask);
        return(coremask);
    }

    /* Get number of cores from fuse register, convert to coremask */
    OCTEON_POP(cores, ciu_fuse);
    return((1<<cores)-1);
}

uint32_t octeon_get_available_core_count(void)
{
    uint32_t coremask, cores;
    coremask = octeon_get_available_coremask();
    OCTEON_POP(cores, coremask);
    return(cores);
}

static void setup_env_named_block(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    int i;
    char *env_ptr = (void *)gd->env_addr;
    int env_size;

    /* Determine the actual size of the environment that is used,
    ** look for two NULs in a row */
    for (i = 0; i < ENV_SIZE - 1; i++ )
    {
        if (env_ptr[i] == 0 && env_ptr[i+1] == 0)
            break;
    }
    env_size = i + 1;

    int64_t addr;
    /* Allocate a named block for the environment */
    addr = cvmx_bootmem_phy_named_block_alloc(env_size + 1, 0ull, 0x40000000ull, 0ull, OCTEON_ENV_MEM_NAME, CVMX_BOOTMEM_FLAG_END_ALLOC);
    if (addr >= 0)
    {
        char *named_ptr = cvmx_phys_to_ptr(addr);
        if (named_ptr)
        {
            memset(named_ptr, 0x0, env_size + 1);
            memcpy(named_ptr, env_ptr, env_size);
        }
    }
}
void start_cores(uint32_t coremask_to_start)
{

    dprintf("Bringing coremask: 0x%x out of reset!\n", coremask_to_start);


    /* Set cores that are not going to be started to branch to a 'break'
    ** instruction at boot
    */
    octeon_setup_boot_vector(0, ~coremask_to_start & 0xffff);

    setup_env_named_block();

    /* Clear cycle counter so that all cores will be close */
    dprintf("Bootloader: Starting app at cycle: %d\n", (uint32_t)boot_cycle_adjustment);

    boot_cycle_adjustment = octeon_get_cycles();
    OCTEON_SYNC;

    /* Cores have already been taken out of reset to conserve power. We need to send a
        NMI to get the cores out of their wait loop */
    cvmx_write_csr(CVMX_CIU_NMI, -2 & octeon_get_available_coremask());


    if (!(coremask_to_start & 1)) {
#if CONFIG_OCTEON_SIM_HW_DIFF
        OCTEON_BREAK;  /* On simulator, we end simulation of core... */
#endif    
        /* core 0 not being run, so just loop here */
        while (!start_core0) {
            ;
        }
    }
    /* Run the app_start_func for the core - if core 0 is not being run, this will
    ** branch to a 'break' instruction to stop simulation
    */
    ((void (*)(void)) (((boot_init_vector_t *)(BOOT_VECTOR_BASE))[0].app_start_func_addr)) ();
}

void octeon_sync_cores(void)
{
    static int core_one = 0;
    static volatile int init_complete = 0;
    int first_core = (core_one == 0);

    core_one |= (1 << get_core_num());

    if (first_core)
    {
        /* Delay so that we are sure we are last */
        octeon_delay_cycles(30000);
        init_complete = 1;
        OCTEON_SYNCW;
    }

    while (!init_complete)
    {
        /* Spin waiting for init to complete */
    }
}

uint64_t octeon_read_gpio()
{
    return(cvmx_read_csr(CVMX_GPIO_RX_DAT));
}

void octeon_gpio_cfg_output(int bit) {
    uint64_t val = cvmx_read_csr(CVMX_GPIO_BIT_CFGX(bit));
    cvmx_write_csr(CVMX_GPIO_BIT_CFGX(bit), val | 1);
}

void octeon_gpio_cfg_input(int bit) {
    uint64_t val = cvmx_read_csr(CVMX_GPIO_BIT_CFGX(bit));
    cvmx_write_csr(CVMX_GPIO_BIT_CFGX(bit), val & ~1ull);
}

void octeon_gpio_set(int bit) {
    cvmx_write_csr(CVMX_GPIO_TX_SET, 1 << bit);
}

void octeon_gpio_clr(int bit) {
    cvmx_write_csr(CVMX_GPIO_TX_CLR, 1 << bit);
}

int octeon_gpio_value(int bit) {
    return (cvmx_read_csr(CVMX_GPIO_RX_DAT) >> bit) & 1;
}

#if (CONFIG_OCTEON_EBT3000 || CONFIG_OCTEON_EBT5800)
int octeon_ebt3000_rev1(void)
{
    return(!(octeon_read64_byte(OCTEON_PAL_BASE_ADDR+0) == 0xa5 && octeon_read64_byte(OCTEON_PAL_BASE_ADDR+1) == 0x5a));
}
#endif

#if (BOARD_MCU_AVAILABLE)
int octeon_mcu_read_cpu_ref(void)
{

    if (twsii_mcu_read(0x00)==0xa5 && twsii_mcu_read(0x01)==0x5a)
        return(((twsii_mcu_read(6)<<8)+twsii_mcu_read(7))/(8));
    else
        return(-1);

}

int octeon_mcu_read_ddr_clock(void)
{

    if (twsii_mcu_read(0x00)==0xa5 && twsii_mcu_read(0x01)==0x5a)
        return((((twsii_mcu_read(4)<<8)+twsii_mcu_read(5))/(1+octeon_read64_byte(OCTEON_PAL_BASE_ADDR+7))));
    else
        return(-1);

}
#endif


/* Restore the core configuration to be closer to standard
** MIPS configuration.  This is used by NAND boot stage 2
** to set up the environment before starting stage 3.
** We don't free named blocks here, because that may corrupt
** the stage 3 image that has been loaded.  Stage 3 is responsible
** for its own memory management.
*/
void octeon_setup_ram_env(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    char *c_ptr = (void *)0x1004;
    uint32_t *ulptr = (void *)0x1000;

    c_ptr += sprintf(c_ptr, "autoload=n") + 1;
    c_ptr += sprintf(c_ptr, "dram_size_mbytes=%d", gd->ram_size/(1024*1024)) + 1;
    *c_ptr = '\0';  /* Terminate with two NULLS */

    *ulptr = crc32(0, (void *)0x1004, 0xffc);

}


void octeon_restore_std_mips_config(void)
{
    uint64_t val;

#ifdef OCTEON_INTERNAL_ENET
int octeon_network_hw_shutdown(void);
    octeon_network_hw_shutdown();
#endif

#ifdef OCTEON_MGMT_ENET
int octeon_mgmt_port_shutdown(void);
    octeon_mgmt_port_shutdown();
#endif

#ifdef CONFIG_USB_OCTEON
int usb_stop(void);
    usb_stop();
#endif

    /* Disable CVMSEG */
    /* Should entire register be set to default ? */
    val = get_cop0_cvmmemctl_reg();
    val &= ~0x1ffull;
    set_cop0_cvmmemctl_reg(val);
}


int octeon_bootloader_shutdown(void)
{
#ifdef OCTEON_INTERNAL_ENET
int octeon_network_hw_shutdown(void);
    octeon_network_hw_shutdown();
#endif

#ifdef CONFIG_USB_OCTEON
int usb_stop(void);
    usb_stop();
#endif
    /* Free temp blocks last, as previous systems being shut down
    ** may still rely on them */
    octeon_free_tmp_named_blocks();
    return 0;
}

void octeon_free_tmp_named_blocks(void)
{
    int i;
    cvmx_bootmem_desc_t *bootmem_desc_ptr = __cvmx_bootmem_internal_get_desc_ptr();

    cvmx_bootmem_named_block_desc_t *named_block_ptr = cvmx_phys_to_ptr(bootmem_desc_ptr->named_block_array_addr);
    for (i = 0; i < bootmem_desc_ptr->named_block_num_blocks; i++)
    {
        if (named_block_ptr[i].size && !strncmp(OCTEON_BOOTLOADER_NAMED_BLOCK_TMP_PREFIX,named_block_ptr[i].name, strlen(OCTEON_BOOTLOADER_NAMED_BLOCK_TMP_PREFIX)))
        {
            /* Found temp block, so free */
            cvmx_bootmem_phy_named_block_free(named_block_ptr[i].name, 0);
        }

    }

}


/* Checks to see if the just completed load is within the bounds of the reserved memory block,
** and prints a warning if it is not. */
void octeon_check_mem_load_range(void)
{
    /* Check to see if we have overflowed the reserved memory for loading */
    if (load_reserved_size)
    {
        ulong faddr = 0;
        ulong fsize = 0;
        char *s;

        if ((s = getenv ("fileaddr")) != NULL)
            faddr = simple_strtoul (s, NULL, 16);
        if ((s = getenv ("filesize")) != NULL)
            fsize = simple_strtoul (s, NULL, 16);
        if (faddr < load_reserved_addr
            || faddr > (load_reserved_addr + load_reserved_size)
            || (faddr + fsize) > (load_reserved_addr + load_reserved_size))
        {
            /* We have loaded some data outside of the reserved area.*/
            printf("WARNING: Data loaded outside of the reserved load area, memory corruption may occur.\n");
            printf("WARNING: Please refer to the bootloader memory map documentation for more information.\n");
        }

    }
}


/* We need to provide a sysinfo structure so that we can use parts of the simple executive */
void octeon_setup_cvmx_sysinfo(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    cvmx_sysinfo_minimal_initialize(0x1fffffff & (unsigned long)__cvmx_bootmem_internal_get_desc_ptr(), gd->board_desc.board_type, 
                                    gd->board_desc.rev_major, gd->board_desc.rev_minor, 
                                    gd->cpu_clock_mhz * 1000000);
}
#endif
