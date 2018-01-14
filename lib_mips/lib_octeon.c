/***********************license start************************************
 * Copyright (c) 2004-2007 Cavium Networks (support@cavium.com). All rights
 * reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *     * Neither the name of Cavium Networks nor the names of
 *       its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
 * AND WITH ALL FAULTS AND CAVIUM NETWORKS MAKES NO PROMISES, REPRESENTATIONS
 * OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 * RESPECT TO THE SOFTWARE, INCLUDING ITS CONDITION, ITS CONFORMITY TO ANY
 * REPRESENTATION OR DESCRIPTION, OR THE EXISTENCE OF ANY LATENT OR PATENT
 * DEFECTS, AND CAVIUM SPECIFICALLY DISCLAIMS ALL IMPLIED (IF ANY) WARRANTIES
 * OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS FOR A PARTICULAR
 * PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET
 * POSSESSION OR CORRESPONDENCE TO DESCRIPTION.  THE ENTIRE RISK ARISING OUT
 * OF USE OR PERFORMANCE OF THE SOFTWARE LIES WITH YOU.
 *
 *
 * For any questions regarding licensing please contact marketing@caviumnetworks.com
 *
 ***********************license end**************************************/

/**
 *
 * $Id: lib_octeon.c 58032 2011-03-15 19:34:17Z cchavva $
 *
 */


#include <common.h>
#include <command.h>
#include <exports.h>
#include <linux/ctype.h>
#include "octeon_boot.h"
#include "lib_octeon.h"
#include "octeon_hal.h"
#include "cvmx-twsi.h"
#include "cvmx-bootloader.h"
#include "cvmx-clock.h"
#include "octeon-pci-console.h"
#include <octeon_eeprom_types.h>
#include <lib_octeon_shared.h>


//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINTF   printf
#else
#define DEBUG_PRINTF(...);
#endif

int init_twsi_bus(void);

extern uint64_t pci_cons_desc_addr;


/* Stub this out here, as we don't support real CSR decoding in u-boot due to size
** limitations */
void cvmx_csr_db_decode(int identifier, uint64_t address, uint64_t value)
{
    printf(" 0x%llx: 0x%llx\n", address, value);
}


/* PCI console setup function.  Reads environment variables and sets up PCI console
** structures accordingly. */
int uboot_octeon_pci_console_setup(void)
{
    if (pci_cons_desc_addr)
        return 0;

    char *ep = getenv("pci_console_count");
    if ((ep && (simple_strtoul(ep, NULL, 10) > 0)) || getenv("pci_console_active"))
    {
        /* Initialize PCI console data structures */
        int console_count;
        int console_size;
        ep = getenv("pci_console_count");
        if (ep)
            console_count = simple_strtoul(ep, NULL, 10);
        else
            console_count = 1;

        if (console_count < 1)
            console_count = 1;

        ep = getenv("pci_console_size");
        if (ep)
            console_size = simple_strtoul(ep, NULL, 10);
        else
            console_size = 1024;

        if (console_size < 128)
            console_size = 128;

        pci_cons_desc_addr = octeon_pci_console_init(console_count, console_size);
        if (!pci_cons_desc_addr)
        {
            printf("WARNING: PCI console init FAILED\n");
            return -1;
        }
        else
        {
            printf("PCI console init succeeded, %d consoles, %d bytes each\n", console_count, console_size);
            return 0;
        }
    }
    return 0;
}



/******************  Begin u-boot eeprom hooks ******************************/
/* support for u-boot i2c functions is limited to the standard serial eeprom on the board.
** The do not support reading either the MCU or the DIMM eeproms
*/


/**
 * Reads bytes from eeprom and copies to DRAM.
 * Only supports address size of 2 (16 bit internal address.)
 * 
 * @param chip   chip address
 * @param addr   internal address
 * @param alen   address length
 * @param buffer memory buffer pointer
 * @param len    number of bytes to read
 * 
 * @return 0 on Success
 *         1 on Failure
 */
int  i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{

    DEBUG_PRINTF ("%s: Reading device: %#04x address %#04x.\n alen=%d, len=%d",
                  __FUNCTION__, chip, addr, alen, len);

    if (alen > 2 || !buffer || !len)
        return(1);

    while (len--)
    {
        uint64_t data;
        int tmp;
        tmp = cvmx_twsix_read_ia(0, chip, addr++, 1, alen, &data);
        if (tmp < 0)
            return(1);
        *buffer++ = (uchar)(data & 0xff);
    }

    return(0);

}



/**
 * Reads bytes from memory and copies to eeprom.
 * Only supports address size of 2 (16 bit internal address.)
 * 
 * We can only write two bytes at a time to the eeprom, so in some cases
 * we need to to a read/modify/write.
 * 
 * Note: can't do single byte write to last address in EEPROM
 * 
 * @param chip   chip address
 * @param addr   internal address
 * @param alen   address length, must be 0, 1, or 2
 * @param buffer memory buffer pointer
 * @param len    number of bytes to write
 * 
 * @return 0 on Success
 *         1 on Failure
 */
int  i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
    int retval;

    DEBUG_PRINTF ("%s: Writing device: %#04x address %#04x data %#04x, alen=%d, len=%d\n",
                  __FUNCTION__, chip, addr, *buffer, alen, len);

    if (alen > 2 || !buffer || !len)
        return(1);

    while (len-- > 0)
    {
        retval = cvmx_twsix_write_ia(0, chip, addr++, 1, alen, *buffer++);
        if (retval < 0)
            return(1);

    }

    return(0);

}
/*-----------------------------------------------------------------------
 * Probe to see if a chip is present using single byte read.  Also good for checking for the
 * completion of EEPROM writes since the chip stops responding until
 * the write completes (typically 10mSec).
 */
int i2c_probe(uchar addr)
{
    uint64_t data;
    if (cvmx_twsix_read(0, addr, 1, &data) < 0)
        return(1);
    else
        return(0); /* probed OK */
}

/******************  End u-boot eeprom hooks ******************************/


#if (CONFIG_OCTEON_EBT3000 || CONFIG_OCTEON_EBT5800)
int octeon_ebt3000_get_board_major_rev(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    return(gd->board_desc.rev_major);
}
int octeon_ebt3000_get_board_minor_rev(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    return(gd->board_desc.rev_minor);
}
#endif

int octeon_show_info(void)
{
    return (!!(octeon_read_gpio() & OCTEON_GPIO_SHOW_FREQ));
}


#ifdef OCTEON_CHAR_LED_BASE_ADDR
void octeon_led_str_write_std(const char *str)
{
    DECLARE_GLOBAL_DATA_PTR;
    if ((gd->board_desc.board_type == CVMX_BOARD_TYPE_EBT3000) && (gd->board_desc.rev_major == 1))
    {
        char *ptr = (char *)(OCTEON_CHAR_LED_BASE_ADDR + 4);
        int i;
        for (i=0; i<4; i++)
        {
            if (*str)
                ptr[3 - i] = *str++;
            else
                ptr[3 - i] = ' ';
        }
    }
    else
    {
        /* rev 2 ebt3000 or ebt5800 or kodama board */
        char *ptr = (char *)(OCTEON_CHAR_LED_BASE_ADDR | 0xf8);
        int i;
        for (i=0; i<8; i++)
        {
            if (*str)
                ptr[i] = *str++;
            else
                ptr[i] = ' ';
        }
    }
}
#else
void octeon_led_str_write_std(const char *str)
{
    /* empty function for when there is no LED */
}
#endif


int displayErrorReg_1(const char *reg_name, uint64_t addr, uint64_t expected, uint64_t mask)
{
    uint64_t bist_val;
    bist_val = cvmx_read_csr(addr);
    bist_val = (bist_val & mask) ^ expected;
    if (bist_val)
    {
        printf("BIST FAILURE: %s, error bits ((bist_val & mask) ^ expected): 0x%llx\n", reg_name, bist_val);
        return(1);
    }
    return(0);
}
#define COP0_CVMMEMCTL_REG $11,7
int displayErrorRegC0_cvmmem(const char *reg_name, uint64_t expected, uint64_t mask)
{
    uint64_t bist_val;

    {
        uint32_t tmp_low, tmp_hi;
    
        asm volatile (
                   "   .set push                    \n"
                   "   .set mips64                  \n"
                   "   .set noreorder               \n"
                   "   dmfc0 %[tmpl], $11, 7         \n"
                   "   dadd   %[tmph], %[tmpl], $0   \n"
                   "   dsrl  %[tmph], 32            \n"
                   "   dsll  %[tmpl], 32            \n"
                   "   dsrl  %[tmpl], 32            \n"
                   "   .set pop                 \n"
                      : [tmpl] "=&r" (tmp_low), [tmph] "=&r" (tmp_hi) : );
    
        bist_val = (((uint64_t)tmp_hi << 32) | tmp_low);
    }
    bist_val = (bist_val & mask) ^ expected;
    if (bist_val)
    {
        printf("BIST FAILURE: %s, error bits: 0x%llx\n", reg_name, bist_val);
        return(1);
    }
    return(0);
}
int displayErrorRegC0_Icache(const char *reg_name, uint64_t expected, uint64_t mask)
{
    uint64_t bist_val;
    {
        uint32_t tmp_low, tmp_hi;
    
        asm volatile (
                   "   .set push                    \n"
                   "   .set mips64                  \n"
                   "   .set noreorder               \n"
                   "   dmfc0 %[tmpl], $27, 0         \n"
                   "   dadd   %[tmph], %[tmpl], $0   \n"
                   "   dsrl  %[tmph], 32            \n"
                   "   dsll  %[tmpl], 32            \n"
                   "   dsrl  %[tmpl], 32            \n"
                   "   .set pop                     \n"
                      : [tmpl] "=&r" (tmp_low), [tmph] "=&r" (tmp_hi) : );
    
        bist_val = (((uint64_t)tmp_hi << 32) | tmp_low);
    }
    bist_val = (bist_val & mask) ^ expected;
    if (bist_val)
    {
        printf("BIST FAILURE: %s, error bits: 0x%llx\n", reg_name, bist_val);
        return(1);
    }
    return(0);
}




/* We want a mask that is all 1's in bit positions not covered
 * by the coremask once it is shifted.
 */
static inline uint64_t make_cm_mask(uint64_t cm, int shift)
{
    uint64_t cm_override_mask;
    if (OCTEON_IS_MODEL(OCTEON_CN63XX))
        cm_override_mask = 0x3f;
    else
    {
        printf("ERROR: unsupported OCTEON model\n");
        return ~0ull;
    }

    return ((~0ull & ~(cm_override_mask << shift)) | (cm << shift));
}
int octeon_bist_6XXX(void)
{
    /* Mask POW BIST_STAT with coremask so we don't report errors on known bad cores  */
    char *cm_str = getenv("coremask_override");
    uint64_t cm_override = ~0ull;
    int bist_failures = 0;

    bist_failures += displayErrorRegC0_cvmmem( "COP0_CVMMEMCTL_REG",       0ull,			0xfc00000000000000ull );

    bist_failures += displayErrorRegC0_Icache( "COP0_ICACHEERR_REG",     0x1ffull << 40 | 0x1ffull << 52,	0x1ffull << 40 | 0x1ffull << 52);
    if (cm_str)
        cm_override = simple_strtoul(cm_str, NULL, 0);

    bist_failures += displayErrorReg_1  (    "POW_BIST_STAT",           CVMX_POW_BIST_STAT,        0ull,			make_cm_mask(cm_override, 16));


    bist_failures += displayErrorReg_1  ( "CVMX_AGL_GMX_BIST",     CVMX_AGL_GMX_BIST,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_CIU_BIST",     CVMX_CIU_BIST,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_DFA_BIST0",     CVMX_DFA_BIST0,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_DFA_BIST1",     CVMX_DFA_BIST1,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_DPI_BIST_STATUS",     CVMX_DPI_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_FPA_BIST_STATUS",     CVMX_FPA_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_GMXX_BIST0",     CVMX_GMXX_BIST(0),     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_IOB_BIST_STATUS",     CVMX_IOB_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_IPD_BIST_STATUS",     CVMX_IPD_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_KEY_BIST_REG",     CVMX_KEY_BIST_REG,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_MIO_BOOT_BIST_STAT",     CVMX_MIO_BOOT_BIST_STAT,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_MIXX_BIST0",     CVMX_MIXX_BIST(0),     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_MIXX_BIST1",     CVMX_MIXX_BIST(1),     0,			~0ull );

    if (!OCTEON_IS_MODEL(OCTEON_CN63XX_PASS1_X))
    {
        int i;
        cvmx_mio_rst_ctlx_t mio_rst;
        /* Only some of the PCIe BIST can be checked before the ports are configured.  Here we only check to see if they are
        ** configured, but do not configure them.  PCIe BIST is also checked by the cvmx_pcie_initialize() function, so PCIe
        ** BIST is checked if used.
        */

        bist_failures += displayErrorReg_1  ( "CVMX_PCSXX_BIST_STATUS_REG",     CVMX_PCSXX_BIST_STATUS_REG(0),     0,			~0ull );
        for (i = 0; i < 2; i++)
        {
            mio_rst.u64 = cvmx_read_csr(CVMX_MIO_RST_CTLX(i));
            if (mio_rst.cn63xx.prtmode == 1)
            {
                if (mio_rst.cn63xx.rst_done)
                {
                    bist_failures += displayErrorReg_1  ( "CVMX_PEMX_BIST_STATUS(i)",     CVMX_PEMX_BIST_STATUS(i),     0,    ~0ull );
                    bist_failures += displayErrorReg_1  ( "CVMX_PEMX_BIST_STATUS2(i)",     CVMX_PEMX_BIST_STATUS2(i),     0,  ~0ull );
                }
                else
                {
                    printf("Skipping PCIe port %d BIST, reset not done. (port not configured)\n", i);
                }
            }
            else
            {
                printf("Skipping PCIe port %d BIST, in EP mode, can't tell if clocked.\n", i);
            }
        }
    }
    else
    {
        printf("NOTICE: Skipping PCIe BIST registers on CN63XX pass 1.\n");
    }
    if (0)
    {
        cvmx_uctlx_if_ena_t usb_if_en;
        usb_if_en.u64 = cvmx_read_csr(CVMX_UCTLX_IF_ENA(0));
        usb_if_en.s.en = 1;
        cvmx_write_csr(CVMX_UCTLX_IF_ENA(0), usb_if_en.u64);
        cvmx_read_csr(CVMX_UCTLX_IF_ENA(0));
        cvmx_wait(1000000);


        bist_failures += displayErrorReg_1  ( "CVMX_UCTLX_BIST_STATUS",     CVMX_UCTLX_BIST_STATUS(0),     0,			~0ull );
    }
    else
    {
        printf("NOTICE: Skipping USB BIST, controller not configured.\n");
    }

    bist_failures += displayErrorReg_1  ( "CVMX_PEXP_SLI_BIST_STATUS",     CVMX_PEXP_SLI_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_PIP_BIST_STATUS",     CVMX_PIP_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_PKO_REG_BIST_RESULT",     CVMX_PKO_REG_BIST_RESULT,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_RAD_REG_BIST_RESULT",     CVMX_RAD_REG_BIST_RESULT,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_RNM_BIST_STATUS",     CVMX_RNM_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_TIM_REG_BIST_RESULT",     CVMX_TIM_REG_BIST_RESULT,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_TRA_BIST_STATUS",     CVMX_TRA_BIST_STATUS,     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_ZIP_CMD_BIST_RESULT",     CVMX_ZIP_CMD_BIST_RESULT,     0,			~0ull );


    bist_failures += displayErrorReg_1  ( "CVMX_L2C_BST",     CVMX_L2C_BST,     0,			make_cm_mask(cm_override, 32));
    bist_failures += displayErrorReg_1  ( "CVMX_L2C_BST_MEMX(0)",     CVMX_L2C_BST_MEMX(0),     0,			0x1f );
    bist_failures += displayErrorReg_1  ( "CVMX_L2C_BST_TDTX(0)",     CVMX_L2C_BST_TDTX(0),     0,			~0ull );
    bist_failures += displayErrorReg_1  ( "CVMX_L2C_BST_TTGX(0)",     CVMX_L2C_BST_TTGX(0),     0,			~0ull );

#if 1
    if (bist_failures)
    {
        printf("BIST Errors found (%d).\n", bist_failures);
        printf("'1' bits above indicate unexpected BIST status.\n");
    }
    else
    {
        printf("BIST check passed.\n");
    }
#endif



    if (0)
    {
        /* Print TDTX bist on LED */

        char str[10];
        uint64_t val = cvmx_read_csr(CVMX_L2C_BST_TDTX(0));
        int ival = (val >> 8) & 0xff;
        if (!bist_failures)
            sprintf(str, "PASS    ");
        else if (bist_failures == 1 && ival && !(val & ~0xFF00ull))
            sprintf(str, "%02x    %d ", ival, __builtin_popcount(ival));
        else
            sprintf(str, "FAIL    ");

        octeon_led_str_write(str);

    }
    return(0);
}



int octeon_bist(void)
{
    /* Mask POW BIST_STAT with coremask so we don't report errors on known bad cores  */
    uint32_t val = (uint32_t)cvmx_read_csr(CVMX_POW_BIST_STAT);
    char *cm_str = getenv("coremask_override");
    int bist_failures = 0;

    if (octeon_is_model(OCTEON_CN6XXX))
        return(octeon_bist_6XXX());


    if (cm_str)
    {
        uint32_t cm_override = simple_strtoul(cm_str, NULL, 0);
        /* Shift coremask override to match up with core BIST bits in register */
        cm_override = cm_override << 16;
        val &= cm_override;
    }
    if (val)
    {
        printf("BIST FAILURE: POW_BIST_STAT: 0x%08x\n", val);
        bist_failures++;
    }
    bist_failures += displayErrorRegC0_cvmmem( "COP0_CVMMEMCTL_REG",       0ull,			0xfc00000000000000ull );
    bist_failures += displayErrorRegC0_Icache( "COP0_ICACHEERR_REG",     0x007f7f0000000000ull,	0x007f7f0000000000ull | 0x3full <<32 );

    /*                    NAME                   REGISTER              EXPECTED            MASK   */
    bist_failures += displayErrorReg_1  ( "GMX0_BIST",           CVMX_GMXX_BIST(0),        0ull,			0xffffffffffffffffull );
    if (!octeon_is_model(OCTEON_CN31XX) && !octeon_is_model(OCTEON_CN30XX) && !octeon_is_model(OCTEON_CN50XX)
        && !octeon_is_model(OCTEON_CN52XX)
        )
        bist_failures += displayErrorReg_1  ( "GMX1_BIST",           CVMX_GMXX_BIST(1),        0ull,			0xffffffffffffffffull );
    bist_failures += displayErrorReg_1  ( "IPD_BIST_STATUS",     CVMX_IPD_BIST_STATUS,     0ull,			0xffffffffffffffffull );
    if (!octeon_is_model(OCTEON_CN31XX) && !octeon_is_model(OCTEON_CN30XX) && !octeon_is_model(OCTEON_CN50XX) && !octeon_is_model(OCTEON_CN52XX))
        bist_failures += displayErrorReg_1  ( "KEY_BIST_REG",        CVMX_KEY_BIST_REG,        0ull,			0xffffffffffffffffull );
    if (octeon_is_model(OCTEON_CN63XX))
    {
        bist_failures += displayErrorReg_1  ( "L2C_BST",            CVMX_L2C_BST,              0,			0xffffffffffffffffull );
        bist_failures += displayErrorReg_1  ( "L2C_BST_MEMX",       CVMX_L2C_BST_MEMX(0),      0,			0xffffffffffffffffull );
        bist_failures += displayErrorReg_1  ( "L2C_BST_TDTX",       CVMX_L2C_BST_TDTX(0),      0,			0xffffffffffffffffull );
        bist_failures += displayErrorReg_1  ( "L2C_BST_TTGX",       CVMX_L2C_BST_TTGX(0),      0,			0xffffffffffffffffull );
    }
    else
    {
        bist_failures += displayErrorReg_1  ( "L2D_BST0",            CVMX_L2D_BST0,            0ull,		    1ull<<34 );
        bist_failures += displayErrorReg_1  ( "L2C_BST0",            CVMX_L2C_BST0,            0,			0x1full );
        bist_failures += displayErrorReg_1  ( "L2C_BST1",            CVMX_L2C_BST1,            0,			0xffffffffffffffffull );
        bist_failures += displayErrorReg_1  ( "L2C_BST2",            CVMX_L2C_BST2,            0,			0xffffffffffffffffull );
    }
    bist_failures += displayErrorReg_1  ( "CIU_BIST",            CVMX_CIU_BIST,            0,			0xffffffffffffffffull );
    bist_failures += displayErrorReg_1  ( "PKO_REG_BIST_RESULT", CVMX_PKO_REG_BIST_RESULT, 0,			0xffffffffffffffffull );
    if (!octeon_is_model(OCTEON_CN56XX) && !octeon_is_model(OCTEON_CN52XX)) {
        bist_failures += displayErrorReg_1  ( "NPI_BIST_STATUS",     CVMX_NPI_BIST_STATUS,     0,			0xffffffffffffffffull );
    }
    if ((!octeon_is_model(OCTEON_CN56XX)) && (!octeon_is_model(OCTEON_CN56XX))) {
        bist_failures += displayErrorReg_1  ( "PIP_BIST_STATUS",     CVMX_PIP_BIST_STATUS,     0,			0xffffffffffffffffull );
    }
    bist_failures += displayErrorReg_1  ( "RNM_BIST_STATUS",     CVMX_RNM_BIST_STATUS,     0,			0xffffffffffffffffull );
    if (!octeon_is_model(OCTEON_CN31XX) && !octeon_is_model(OCTEON_CN30XX) && !octeon_is_model(OCTEON_CN56XX)
        && !octeon_is_model(OCTEON_CN50XX) && !octeon_is_model(OCTEON_CN52XX)
        )
    {
        bist_failures += displayErrorReg_1  ( "SPX0_BIST_STAT",      CVMX_SPXX_BIST_STAT(0),   0,			0xffffffffffffffffull );
        bist_failures += displayErrorReg_1  ( "SPX1_BIST_STAT",      CVMX_SPXX_BIST_STAT(1),   0,			0xffffffffffffffffull );
    }
    bist_failures += displayErrorReg_1  ( "TIM_REG_BIST_RESULT", CVMX_TIM_REG_BIST_RESULT, 0,			0xffffffffffffffffull );
    if (!octeon_is_model(OCTEON_CN30XX) && !octeon_is_model(OCTEON_CN50XX))
        bist_failures += displayErrorReg_1  ( "TRA_BIST_STATUS",     CVMX_TRA_BIST_STATUS,     0,			0xffffffffffffffffull );
    bist_failures += displayErrorReg_1  ( "MIO_BOOT_BIST_STAT",  CVMX_MIO_BOOT_BIST_STAT,  0,			0xffffffffffffffffull );
    bist_failures += displayErrorReg_1  ( "IOB_BIST_STATUS",     CVMX_IOB_BIST_STATUS,     0,			0xffffffffffffffffull );
    if (!octeon_is_model(OCTEON_CN30XX) && !octeon_is_model(OCTEON_CN56XX)
        && !octeon_is_model(OCTEON_CN50XX) && !octeon_is_model(OCTEON_CN52XX)
        )
    {
        bist_failures += displayErrorReg_1  ( "DFA_BST0",            CVMX_DFA_BST0,            0,			0xffffffffffffffffull );
        bist_failures += displayErrorReg_1  ( "DFA_BST1",            CVMX_DFA_BST1,            0,			0xffffffffffffffffull );
    }
    bist_failures += displayErrorReg_1  ( "FPA_BIST_STATUS",     CVMX_FPA_BIST_STATUS,     0,			0xffffffffffffffffull );
    if (!octeon_is_model(OCTEON_CN30XX) && !octeon_is_model(OCTEON_CN50XX)&& !octeon_is_model(OCTEON_CN52XX))
        bist_failures += displayErrorReg_1  ( "ZIP_CMD_BIST_RESULT", CVMX_ZIP_CMD_BIST_RESULT, 0,			0xffffffffffffffffull );

    if (!octeon_is_model(OCTEON_CN38XX) && !octeon_is_model(OCTEON_CN58XX) && !octeon_is_model(OCTEON_CN52XX))
    {
        bist_failures += displayErrorReg_1("USBNX_BIST_STATUS(0)", CVMX_USBNX_BIST_STATUS(0), 0 , ~0ull);
    }
    if ((octeon_is_model(OCTEON_CN56XX)) || (octeon_is_model(OCTEON_CN52XX)))
    {
        bist_failures += displayErrorReg_1("AGL_GMX_BIST", CVMX_AGL_GMX_BIST, 0 , ~0ull);
        bist_failures += displayErrorReg_1("IOB_BIST_STATUS", CVMX_IOB_BIST_STATUS, 0 , ~0ull);
        bist_failures += displayErrorReg_1("MIXX_BIST(0)", CVMX_MIXX_BIST(0), 0 , ~0ull);
        bist_failures += displayErrorReg_1("NPEI_BIST_STATUS", CVMX_PEXP_NPEI_BIST_STATUS, 0 , ~0ull);
        bist_failures += displayErrorReg_1("RAD_REG_BIST_RESULT", CVMX_RAD_REG_BIST_RESULT, 0 , ~0ull);
        bist_failures += displayErrorReg_1("RNM_BIST_STATUS", CVMX_RNM_BIST_STATUS, 0 , ~0ull);
    }
    if (octeon_is_model(OCTEON_CN56XX))
    {
        bist_failures += displayErrorReg_1("PCSX0_BIST_STATUS_REG(0)", CVMX_PCSXX_BIST_STATUS_REG(0), 0 , ~0ull);
        bist_failures += displayErrorReg_1("PCSX1_BIST_STATUS_REG(1)", CVMX_PCSXX_BIST_STATUS_REG(1), 0 , ~0ull);
    }
    if (bist_failures)
    {
        printf("BIST Errors found.\n");
        printf("'1' bits above indicate unexpected BIST status.\n");
    }
    else
    {
        printf("BIST check passed.\n");
    }
    return(0);
}

int octeon_check_mem_errors(void)
{
    /* Check for DDR/L2C memory errors */
    uint64_t val;
    int errors = 0;
    int ddr_ecc_errors = 0;

#if CONFIG_L2_ONLY
    return(0);
#endif

    if (OCTEON_IS_MODEL(OCTEON_CN6XXX))
    {
        cvmx_l2c_err_tdtx_t d_err;
        cvmx_l2c_err_ttgx_t t_err;
        d_err.u64 = cvmx_read_csr(CVMX_L2C_ERR_TDTX(0));
        if (d_err.s.dbe || d_err.s.sbe || d_err.s.vdbe || d_err.s.vsbe)
        {
            printf("WARNING: L2D ECC errors detected!(syn: 0x%x, wayidx: 0x%x, type: 0x%x\n",
                   t_err.s.syn, t_err.s.wayidx, t_err.s.type);
            cvmx_write_csr(CVMX_L2C_ERR_TDTX(0), d_err.u64);
            errors++;
        }
        t_err.u64 = cvmx_read_csr(CVMX_L2C_ERR_TTGX(0));
        if (t_err.s.dbe || t_err.s.sbe)
        {
            printf("WARNING: L2T ECC errors detected!(syn: 0x%x, wayidx: 0x%x, type: 0x%x\n",
                   t_err.s.syn, t_err.s.wayidx, t_err.s.type);
            cvmx_write_csr(CVMX_L2C_ERR_TTGX(0), t_err.u64);
            errors++;
        }
        if (t_err.s.noway)
            printf("WARNING: L2C NOWAY error: no way available for allocation\n");
    }
    else
    {
        val = cvmx_read_csr(CVMX_L2D_ERR);
        if (val & 0x18ull)
        {
            printf("WARNING: L2D ECC errors detected!\n");
            cvmx_write_csr(CVMX_L2D_ERR, val);
            errors++;
        }
        val = cvmx_read_csr(CVMX_L2T_ERR);
        if (val & 0x18ull)
        {
            printf("WARNING: L2T ECC errors detected!\n");
            cvmx_write_csr(CVMX_L2T_ERR, val);
            errors++;
        }
    }

    if (octeon_is_model(OCTEON_CN63XX))
    {
        cvmx_lmcx_int_t lmc_int;
        lmc_int.u64 = cvmx_read_csr(CVMX_LMCX_INT(0));
        if (lmc_int.s.sec_err || lmc_int.s.ded_err)
            printf("WARNING: DDR ECC errors detected!\n");
        if (lmc_int.s.nxm_wr_err)
            printf("WARNING: DDR write to nonexistant memory\n");

        cvmx_write_csr(CVMX_LMCX_INT(0), lmc_int.u64);
    }
    else if (octeon_is_model(OCTEON_CN56XX)) {
	/* The cn56xx has multiple ddr interfaces that can be enabled
	** individually. Confirm that the interface is enabled before
	** checking for ECC errors.  Otherwise the read will timeout
	** with a bus error. */
        cvmx_l2c_cfg_t l2c_cfg;
        l2c_cfg.u64 = cvmx_read_csr(CVMX_L2C_CFG);
        if (l2c_cfg.cn56xx.dpres0) {
	    val = cvmx_read_csr(CVMX_LMCX_MEM_CFG0(0));
	    if (val & (0xFFull << 21)) {
		cvmx_write_csr(CVMX_LMCX_MEM_CFG0(0), val);
		errors++;
		ddr_ecc_errors++;
	    }
        }
        if (l2c_cfg.cn56xx.dpres1) {
	    val = cvmx_read_csr(CVMX_LMCX_MEM_CFG0(1));
	    if (val & (0xFFull << 21)) {
            cvmx_write_csr(CVMX_LMCX_MEM_CFG0(1), val);
		errors++;
		ddr_ecc_errors++;
	    }
        }
    } else {
	val = cvmx_read_csr(CVMX_LMC_MEM_CFG0);
	if (val & (0xFFull << 21))
	{
	    cvmx_write_csr(CVMX_LMC_MEM_CFG0, val);
	    errors++;
            ddr_ecc_errors++;
	}
    }

    if (ddr_ecc_errors) {
	printf("WARNING: DDR ECC errors detected!\n");
    }
    return(errors);
}

/* Checks to see if a boot bus region is enabled for the specified address.
** This is used to conditionalize at runtime the CFI flash probing, as this is
** required generic board support.
*/
int octeon_check_nor_flash_enabled(uint32_t addr)
{
    int i;
    cvmx_mio_boot_reg_cfgx_t reg_cfg;
    uint32_t reg_addr;
    uint32_t reg_size;

    addr &= 0x1fffffff;
    for (i = 0; i < 8;i++)
    {
        reg_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_REG_CFGX(i));
        reg_addr = reg_cfg.s.base << 16;
        reg_size = (reg_cfg.s.size + 1) << 16;

        if (addr >= reg_addr && addr < (reg_addr + reg_size) && reg_cfg.s.en)
            return 1;
    }
    return 0;
}

/* Boot bus init for flash and peripheral access */
#define FLASH_RoundUP(_Dividend, _Divisor) (((_Dividend)+(_Divisor))/(_Divisor))
#ifndef CFG_NO_FLASH
flash_info_t flash_info[CFG_MAX_FLASH_BANKS];	/* info for FLASH chips */
#endif
void set_flash_clock(void)
{
    cvmx_mio_boot_reg_timx_t __attribute__((unused)) reg_tim;
    uint64_t clock_mhz, clock_period;

    clock_mhz = cvmx_clock_get_rate(CVMX_CLOCK_SCLK)/1000000;
    clock_period = 1000000 / clock_mhz; /* clk period (psecs) */

   /* Set timing to be valid for all CPU clocks up to clock_period */
    reg_tim.u64 = 0;
    reg_tim.s.pagem = 0;
    reg_tim.s.wait = 0x3f;
    reg_tim.s.adr = FLASH_RoundUP((FLASH_RoundUP(10000ULL, clock_period) - 1), 4);
    reg_tim.s.pause = 0;
    reg_tim.s.ce = FLASH_RoundUP((FLASH_RoundUP(50000ULL, clock_period) - 1), 4);;
    if (octeon_is_model(OCTEON_CN31XX))
        reg_tim.s.ale = 4; /* Leave ALE at 34 nS */
    reg_tim.s.oe = FLASH_RoundUP((FLASH_RoundUP(50000ULL, clock_period) - 1), 4);
    reg_tim.s.rd_hld = FLASH_RoundUP((FLASH_RoundUP(25000ULL, clock_period) - 1), 4);
    reg_tim.s.wr_hld = FLASH_RoundUP((FLASH_RoundUP(35000ULL, clock_period) - 1), 4);
    reg_tim.s.we = FLASH_RoundUP((FLASH_RoundUP(35000ULL, clock_period) - 1), 4);
    reg_tim.s.page = FLASH_RoundUP((FLASH_RoundUP(25000ULL, clock_period) - 1), 4);
    cvmx_write_csr(CVMX_MIO_BOOT_REG_TIM0, reg_tim.u64);
}

int octeon_boot_bus_init(void)
{
    cvmx_mio_boot_reg_cfgx_t __attribute__((unused)) reg_cfg;
    cvmx_mio_boot_reg_timx_t __attribute__((unused)) reg_tim;
    uint64_t clock_mhz, clock_period;

#ifdef OCTEON_CHAR_LED_CHIP_SEL
    /* Setup region for 4 char LED display. Do this first so it can be used to debug later things
    ** in this function. */
    reg_cfg.u64 = 0;
    reg_cfg.s.en = 1;
    reg_cfg.s.base = ((OCTEON_CHAR_LED_BASE_ADDR >> 16) & 0x1fff);  /* Mask to put physical address in boot bus window */
    cvmx_write_csr(CVMX_MIO_BOOT_REG_CFGX(4), reg_cfg.u64);
#endif

    clock_mhz = cvmx_clock_get_rate(CVMX_CLOCK_SCLK)/1000000;
    clock_period = 1000000 / clock_mhz; /* clk period (psecs) */

#ifdef CFG_FLASH_SIZE
    /* Remap flash part so that it is all addressable on boot bus, with alias
    ** at 0x1fc00000 so that the data mapped at the default location (0x1fc00000) is
    ** still available at the same address.  Note that this is different if a TLB mapped
    ** u-boot (see below.) */
    reg_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_REG_CFG0);
    /* Don't set enable, as for NAND boot CS0 will be disabled,
    ** and enabling it here complicates things.  We still have to handle
    ** the case of NAND on CS0 and it enabled, as this can happen when
    ** in NAND boot mode but the board is booted over pci/ejtag. */

    reg_cfg.s.tim_mult = 0;
    reg_cfg.s.rd_dly = 0;
    reg_cfg.s.we_ext = 0;
    reg_cfg.s.oe_ext = 0;
    reg_cfg.s.orbit = 0;

    reg_cfg.s.size = ((CFG_FLASH_SIZE + 0x400000) >> 16) - 1;  /* In 64k blocks, + 4MByte alias of low 4Mbytes of flash */
    reg_cfg.s.base = ((CFG_FLASH_BASE >> 16) & 0x1fff);  /* Mask to put physical address in boot bus window */

#if CONFIG_OCTEON_UBOOT_TLB
    DECLARE_GLOBAL_DATA_PTR;
    if (!(gd->flags & GD_FLG_RAM_RESIDENT))
    {
        /* If booting from RAM, we want to set the boot bus base to the normal, aliased value */
        /* Base has previously been adjusted to have the booting u-boot image starting
        ** at 0x1FC00000 so that we can TLB map it with a single entry.
        ** This will be adjusted to the normal settings once we are running
        ** from RAM and don't need the TLB mapping to point to flash. */
        cvmx_mio_boot_reg_cfgx_t __attribute__((unused)) reg_cfg1;
        reg_cfg1.u64 = cvmx_read_csr(CVMX_MIO_BOOT_REG_CFG0);

        reg_cfg.s.base = reg_cfg1.s.base - (CFG_FLASH_SIZE >> 16);
    }
#endif
    cvmx_write_csr(CVMX_MIO_BOOT_REG_CFG0, reg_cfg.u64);

    /* Set timing to be valid for all CPU clocks up to clock_period */
    reg_tim.u64 = 0;
    reg_tim.s.pagem = 0;
    reg_tim.s.wait = 0x3f;
    reg_tim.s.adr = FLASH_RoundUP((FLASH_RoundUP(10000ULL, clock_period) - 1), 4);
    reg_tim.s.pause = 0;
    reg_tim.s.ce = FLASH_RoundUP((FLASH_RoundUP(50000ULL, clock_period) - 1), 4);;
    if (octeon_is_model(OCTEON_CN31XX))
        reg_tim.s.ale = 4; /* Leave ALE at 34 nS */
    reg_tim.s.oe = FLASH_RoundUP((FLASH_RoundUP(50000ULL, clock_period) - 1), 4);
    reg_tim.s.rd_hld = FLASH_RoundUP((FLASH_RoundUP(25000ULL, clock_period) - 1), 4);
    reg_tim.s.wr_hld = FLASH_RoundUP((FLASH_RoundUP(35000ULL, clock_period) - 1), 4);
    reg_tim.s.we = FLASH_RoundUP((FLASH_RoundUP(35000ULL, clock_period) - 1), 4);
    reg_tim.s.page = FLASH_RoundUP((FLASH_RoundUP(25000ULL, clock_period) - 1), 4);
    cvmx_write_csr(CVMX_MIO_BOOT_REG_TIM0, reg_tim.u64);
#endif

    /* Set up regions for CompactFlash */
    /* Attribute memory region */
#ifdef OCTEON_CF_ATTRIB_CHIP_SEL
    reg_cfg.u64 = 0;
    reg_cfg.s.en = 1;
#ifdef OCTEON_CF_16_BIT_BUS
    reg_cfg.s.width = 1;
#endif
    reg_cfg.s.base = ((OCTEON_CF_ATTRIB_BASE_ADDR >> 16) & 0x1fff);  /* Mask to put physical address in boot bus window */
    cvmx_write_csr(CVMX_MIO_BOOT_REG_CFGX(OCTEON_CF_ATTRIB_CHIP_SEL), reg_cfg.u64);

    reg_tim.u64 = 0;
    reg_tim.s.wait = 0x3f;
    reg_tim.s.page = 0x3f;
    reg_tim.s.wr_hld = FLASH_RoundUP((FLASH_RoundUP(70000ULL, clock_period) - 2), 4);
    reg_tim.s.rd_hld = FLASH_RoundUP((FLASH_RoundUP(100000ULL, clock_period) - 1), 4);
    reg_tim.s.we = FLASH_RoundUP((FLASH_RoundUP(150000ULL, clock_period) - 1), 4);
    reg_tim.s.oe = FLASH_RoundUP((FLASH_RoundUP(270000ULL, clock_period) - 1), 4);
    reg_tim.s.ce = FLASH_RoundUP((FLASH_RoundUP(30000ULL, clock_period) - 2), 4);
    cvmx_write_csr(CVMX_MIO_BOOT_REG_TIMX(OCTEON_CF_ATTRIB_CHIP_SEL), reg_tim.u64);
#endif

#ifdef OCTEON_CF_COMMON_CHIP_SEL
    /* Common memory region */
    reg_cfg.u64 = 0;
    reg_cfg.s.en = 1;
#ifdef OCTEON_CF_16_BIT_BUS
    reg_cfg.s.width = 1;
#endif
    reg_cfg.s.base = ((OCTEON_CF_COMMON_BASE_ADDR >> 16) & 0x1fff);  /* Mask to put physical address in boot bus window */
    cvmx_write_csr(CVMX_MIO_BOOT_REG_CFGX(OCTEON_CF_COMMON_CHIP_SEL), reg_cfg.u64);

    reg_tim.u64 = 0;
    reg_tim.s.wait = (FLASH_RoundUP(30000ULL, clock_period) - 1);
    reg_tim.s.waitm = 1;
    reg_tim.s.page = 0x3f;
    reg_tim.s.wr_hld = FLASH_RoundUP((FLASH_RoundUP(30000ULL, clock_period) - 1), 4);
    reg_tim.s.rd_hld = FLASH_RoundUP((FLASH_RoundUP(100000ULL, clock_period) - 1), 4);
    reg_tim.s.we = FLASH_RoundUP((FLASH_RoundUP(150000ULL, clock_period) - 1), 4);
    reg_tim.s.oe = FLASH_RoundUP((FLASH_RoundUP(125000ULL, clock_period) - 1), 4);
    reg_tim.s.ce = FLASH_RoundUP((FLASH_RoundUP(30000ULL, clock_period) - 2), 4);
    cvmx_write_csr(CVMX_MIO_BOOT_REG_TIMX(OCTEON_CF_COMMON_CHIP_SEL), reg_tim.u64);
#endif


#ifdef OCTEON_CF_TRUE_IDE_CS0_CHIP_SEL
    reg_cfg.u64 = 0;
    reg_cfg.s.en = 1;
    reg_cfg.s.width = 1;
    reg_cfg.s.base = ((OCTEON_CF_TRUE_IDE_CS0_ADDR >> 16) & 0x1fff);  /* Mask to put physical address in boot bus window */
    cvmx_write_csr(CVMX_MIO_BOOT_REG_CFGX(OCTEON_CF_TRUE_IDE_CS0_CHIP_SEL), reg_cfg.u64);

    reg_tim.u64 = 0;
    reg_tim.s.wait = (FLASH_RoundUP(300000ULL, clock_period) - 1);
    reg_tim.s.page = 0x3f;
    reg_tim.s.wr_hld = FLASH_RoundUP((FLASH_RoundUP(300000ULL, clock_period) - 1), 4);
    reg_tim.s.rd_hld = FLASH_RoundUP((FLASH_RoundUP(1000000ULL, clock_period) - 1), 4);
    reg_tim.s.we = FLASH_RoundUP((FLASH_RoundUP(1500000ULL, clock_period) - 1), 4);
    reg_tim.s.oe = FLASH_RoundUP((FLASH_RoundUP(1250000ULL, clock_period) - 1), 4);
    reg_tim.s.ce = FLASH_RoundUP((FLASH_RoundUP(300000ULL, clock_period) - 2), 4);
    cvmx_write_csr(CVMX_MIO_BOOT_REG_TIMX(OCTEON_CF_TRUE_IDE_CS0_CHIP_SEL), reg_tim.u64);
#endif


#ifdef OCTEON_CF_TRUE_IDE_CS1_CHIP_SEL
    reg_cfg.u64 = 0;
    reg_cfg.s.en = 1;
    reg_cfg.s.width = 1;
    reg_cfg.s.base = ((OCTEON_CF_TRUE_IDE_CS1_ADDR >> 16) & 0x1fff);  /* Mask to put physical address in boot bus window */
    cvmx_write_csr(CVMX_MIO_BOOT_REG_CFGX(OCTEON_CF_TRUE_IDE_CS1_CHIP_SEL), reg_cfg.u64);

    reg_tim.u64 = 0;
    reg_tim.s.wait = (FLASH_RoundUP(30000ULL, clock_period) - 1);
    reg_tim.s.page = 0x3f;
    reg_tim.s.wr_hld = FLASH_RoundUP((FLASH_RoundUP(30000ULL, clock_period) - 1), 4);
    reg_tim.s.rd_hld = FLASH_RoundUP((FLASH_RoundUP(100000ULL, clock_period) - 1), 4);
    reg_tim.s.we = FLASH_RoundUP((FLASH_RoundUP(150000ULL, clock_period) - 1), 4);
    reg_tim.s.oe = FLASH_RoundUP((FLASH_RoundUP(125000ULL, clock_period) - 1), 4);
    reg_tim.s.ce = FLASH_RoundUP((FLASH_RoundUP(30000ULL, clock_period) - 2), 4);
    cvmx_write_csr(CVMX_MIO_BOOT_REG_TIMX(OCTEON_CF_TRUE_IDE_CS1_CHIP_SEL), reg_tim.u64);
#endif



#ifdef OCTEON_PAL_CHIP_SEL
    /* Setup region for PAL access */
    reg_cfg.u64 = 0;
    reg_cfg.s.en = 1;
    reg_cfg.s.base = ((OCTEON_PAL_BASE_ADDR >> 16) & 0x1fff);  /* Mask to put physical address in boot bus window */
    cvmx_write_csr(CVMX_MIO_BOOT_REG_CFGX(OCTEON_PAL_CHIP_SEL), reg_cfg.u64);
#endif

    init_twsi_bus();

    return(0);
}

/* Reads the CPU clock multiplier and returns it.  Does
** the correct thing based on Octeon model */


int octeon_get_cpu_multiplier(void)
{

    if (OCTEON_IS_MODEL(OCTEON_CN6XXX))
    {
        cvmx_mio_rst_boot_t rst_boot;
        rst_boot.u64 = cvmx_read_csr(CVMX_MIO_RST_BOOT);
        return(rst_boot.s.c_mul);
    }
    else
    {
        uint64_t data;
        if (OCTEON_IS_MODEL(OCTEON_CN52XX) || OCTEON_IS_MODEL(OCTEON_CN56XX))
            data = cvmx_read_csr(CVMX_PEXP_NPEI_DBG_DATA);
        else
            data = cvmx_read_csr(CVMX_DBG_DATA);

        data = data >> 18;
        data &= 0x1f;
        return(data);
    }
}

uint64_t octeon_get_ioclk_hz(void)
{
    if (OCTEON_IS_MODEL(OCTEON_CN6XXX))
    {
        cvmx_mio_rst_boot_t rst_boot;
        rst_boot.u64 = cvmx_read_csr(CVMX_MIO_RST_BOOT);
        return(rst_boot.s.pnr_mul * 50000000);
    }
    else
    {
        /* For older chips, this is the same as the core clock */
        return(octeon_get_cpu_multiplier() * 50000000);
    }


}


int init_twsi_bus(void)
{
    int M_divider, N_divider;
    cvmx_mio_tws_sw_twsi_t sw_twsi;
    uint64_t io_clock_mhz;

    io_clock_mhz = cvmx_clock_get_rate(CVMX_CLOCK_SCLK)/1000000;

    /* Slow down the TWSI clock, as the default is too fast in some
    ** cases. */

    /*
    ** Set the TWSI clock to a conservative 100KHz.  Compute the
    ** clocks M divider based on the core clock.
    **
    **  TWSI freq = (core freq) / (20 x (M+1) x (thp+1) x 2^N)
    **	M = ((core freq) / (20 x (TWSI freq) x (thp+1) x 2^N)) - 1
    */

#ifndef TWSI_BUS_FREQ
#define TWSI_BUS_FREQ 	(100000) /* 100KHz */
#endif
#ifndef TWSI_THP
#define TWSI_THP	    (24) /* TCLK half period (default 24) */
#endif

    for (N_divider = 0; N_divider < 8; ++N_divider) {
        M_divider = ((io_clock_mhz * 1000000) / (20 * TWSI_BUS_FREQ * (TWSI_THP+1) * (1<<N_divider))) - 1;
        if (M_divider < 16) break;
    }

    sw_twsi.u64 = 0;
    sw_twsi.s.v = 1;
    sw_twsi.s.op = 0x6;
    sw_twsi.s.r = 0;            /* Select CLKCTL when R = 0 */
    sw_twsi.s.eop_ia = TWSI_CLKCTL_STAT;
    sw_twsi.s.d = ((M_divider&0xf) << 3) | ((N_divider&0x7) << 0);
    cvmx_write_csr(CVMX_MIO_TWS_SW_TWSI, sw_twsi.u64);

    /* 
    ** If TWSI_BUS_SLAVE_ADDRESS is defined use that address to
    ** override the default Oction TWSI Slave Address.
    */
#if TWSI_BUS_SLAVE_ADDRESS
    sw_twsi.u64 = 0;
    sw_twsi.s.v = 1;
    sw_twsi.s.op = 0x6;
    sw_twsi.s.r = 0;
    sw_twsi.s.eop_ia = TWSI_SLAVE_ADD;
    sw_twsi.s.d = TWSI_BUS_SLAVE_ADDRESS << 1;
    cvmx_write_csr(CVMX_MIO_TWS_SW_TWSI, sw_twsi.u64);
#endif

    return 0;
}


uint32_t calculate_header_crc(const bootloader_header_t *header);
int validate_bootloader_header(const bootloader_header_t *header)
{
    volatile uint32_t tmp;

    tmp = header->magic;

    if (header->magic == BOOTLOADER_HEADER_MAGIC)
    {
        /* Check the CRC of the header */
        if (calculate_header_crc(header) == header->hcrc) 
	    return 1;
        else
            return 0;
    }
    return 0;
}


/**
 * This function is called by the failsafe bootloader to find and validate
 * a normal bootloader image to boot.  It does this by scanning for a header
 * (using the magic number.)  If it finds a header, it checks the CRC of the header.
 * If the CRC is valid, then it checks the CRC of the entire image.  If the
 * image CRC is valid, then the normal image is booted.
 * If a valid normal image is not found, then the failsafe image is booted.
 * 
 * @param reloc_offset
 *               Relocation offset as computed by the assembly code.  This represents
 *               the offset from the address at which the image was linked at to
 *               the address it is being run from.
 * 
 * @return Address of the valid bootloader to jump to, or 0 if no valid bootloader is found.
 */
extern int _start;  /* symbol at start of code */
#ifndef CFG_NO_FLASH
uint32_t octeon_find_and_validate_normal_bootloader(int rescan_flag)
{

    bootloader_header_t *header;
    unsigned char *scan_ptr;
    unsigned char *scan_end;


    if (!rescan_flag)
    {
        /* Speed up flash */
        set_flash_clock();

        cvmx_wait_usec(100000);
        printf("\nLooking for valid bootloader image....\n");
        /* Read our header to figure out where to start scanning */
        header = (void *)&_start;
#if CONFIG_OCTEON_UBOOT_TLB
        /* In TLB mode, this will be a virtual address, but we need to start execution
        ** of the normal bootloader from the physical address space.
        */
        uint64_t addr = uboot_tlb_ptr_to_phys(header);
        header = (void *)(0xA0000000 | (uint32_t)addr);
#endif

    }
    else
        header = (void *)CFG_FLASH_BASE;


    /* Give up if we can't read our own header - something is really wrong */
    if (!validate_bootloader_header(header) && !rescan_flag)
    {
        printf("ERROR: can't validate our own header.\n");
        return 0;
    }

    scan_ptr = (void *)header;
    scan_end = scan_ptr + MAX_NOR_SEARCH_ADDR;

#ifndef CONFIG_OCTEON_NO_FAILSAFE
    /* Scan for header starting at end of failsafe image, if we expect one to exist. */
    if (rescan_flag)
        scan_ptr += 0x30000;
    else
        scan_ptr += ((header->dlen + header->hlen + LOOKUP_STEP)/LOOKUP_STEP) * LOOKUP_STEP;
#endif

    for (;scan_ptr <= scan_end; scan_ptr += LOOKUP_STEP)
    {
        header = (bootloader_header_t *)scan_ptr;
        if (validate_bootloader_header(header))
        {

            /* We have a valid header, now see if the image is valid */
            if (header->dcrc == crc32(0, scan_ptr + header->hlen, header->dlen))
            {
                /* We have found a valid image, so boot it */
                return((uint32_t)scan_ptr);
            }
            else
            {
                printf("WARNING: Ignoring corrupt image at addres %p\n", scan_ptr);
            }
        }
    }

    return(0);

}
#endif

uint32_t get_image_size(const bootloader_header_t *header);
#ifndef OCTEON_GPIO_FAILSAFE_BIT
#define OCTEON_GPIO_FAILSAFE_BIT 0
#endif
/* Scan the NOR flash for a valid 'normal' (non-failsafe) image to boot.
** This does it's own serial port initialization since we want to call it
** very early in the boot process.
*/
int failsafe_scan(void)
{
    uint32_t __attribute__((unused)) addr;
    uint64_t data;

    /* Manually set up the uart here so that we can output status.  The uart will be re-initialized
    ** later in boot as well. */
    octeon_uart_setup2(0, octeon_get_ioclk_hz(), 115200);

#if CONFIG_OCTEON_UBOOT_TLB
    DECLARE_GLOBAL_DATA_PTR;
    /* Determine where we are booting from.  Could be one of
    ** 0x1fc00000 - base of flash, so we could be the failsafe image.
    ** 0x1fcxxxxx - non-base of flash, we are not the failsafe image
    ** RAM        - we are booting from DRAM,
    */
    {
#ifndef CFG_NO_FLASH
        uint64_t phy_addr = uboot_tlb_ptr_to_phys(&_start);
        if (phy_addr >= 0x10000000ull && phy_addr < 0x20000000ull)
        {

            cvmx_mio_boot_reg_cfgx_t __attribute__((unused)) reg_cfg;
            reg_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_REG_CFG0);
            gd->uboot_flash_address = CFG_FLASH_BASE + ((0x1fc0 - reg_cfg.s.base) << 16);
            gd->uboot_flash_size = get_image_size((bootloader_header_t *)(0x1fc00000));
#ifndef CONFIG_OCTEON_NO_FAILSAFE
            if (reg_cfg.s.base == 0x1fc0)
                gd->flags |= GD_FLG_FAILSAFE_MODE;
#endif
        }
        else
#endif
        {
            gd->flags |= GD_FLG_RAM_RESIDENT;
            gd->flags |= (GD_FLG_DDR0_CLK_INITIALIZED);
            gd->flags |= (GD_FLG_DDR1_CLK_INITIALIZED);
            return(0);
        }


    }

    if (!(gd->flags & GD_FLG_FAILSAFE_MODE))
        return(0);
#endif

#ifndef CFG_NO_GPIO_FAILSAFE
    data = cvmx_read_csr(CVMX_GPIO_RX_DAT);
    if (data & (1 << OCTEON_GPIO_FAILSAFE_BIT))
    {
        printf("GPIO bit %d set, booting failsafe image\n", OCTEON_GPIO_FAILSAFE_BIT);
        return(0);
    }
#endif

#ifndef CFG_NO_FLASH
    addr = octeon_find_and_validate_normal_bootloader(0);
    if (addr)
    {
        printf("Jumping to start of image at address 0x%08x\n", addr);
        ((void (*)(void)) addr) ();
    }
#endif
    printf("No valid bootloader found, booting failsafe image.\n");
    return 0;
}

int octeon_boot_bus_moveable_init(void)
{
    extern void debugHandler_entrypoint(void);
    extern void SecondaryCoreInit(void);
    const uint64_t *handler_code;
    int count;

    /* Install the 2nd moveable region to the debug exception main entry
        point. This way the debugger will work even if there isn't any
        flash. */
    handler_code = (const uint64_t *)debugHandler_entrypoint;
    cvmx_write_csr(CVMX_MIO_BOOT_LOC_ADR, 0x80);
    for (count=0; count<128/8; count++)
        cvmx_write_csr(CVMX_MIO_BOOT_LOC_DAT, *handler_code++);
    cvmx_write_csr(CVMX_MIO_BOOT_LOC_CFGX(1), (1ull << 31) | (0x1fc00480 >> 4));

    /* Install the first movable region at the reset vector so we can boot
        other cores */
    cvmx_write_csr(CVMX_MIO_BOOT_LOC_ADR, 0x0);  /* Auto increments after write */
    handler_code = (const uint64_t *)SecondaryCoreInit;
    for (count=0; count<128/8; count++)
        cvmx_write_csr(CVMX_MIO_BOOT_LOC_DAT, *handler_code++);

    cvmx_write_csr(CVMX_MIO_BOOT_LOC_CFGX(0), (1ull << 31) | (0x1fc00000 >> 4));
    return 0;
}

char * lowcase(char *str_arg)
{
    char *str = str_arg;
    while (*str)
    {
        *str = tolower(*str);
        str++;
    }
    return(str_arg);
}

