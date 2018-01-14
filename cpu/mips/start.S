/*
 *  Startup Code for MIPS32 CPU-core
 *
 *  Copyright (c) 2003  Wolfgang Denk <wd@denx.de>
 *  Copyright 2004, 2005 Cavium Networks.
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


#define COP0_HWRENA_REG $7,0
#define COP0_COUNT_REG $9,0
#define COP0_CVMCOUNT_REG $9,6
#define COP0_CVMCTL_REG $9,7
#define COP0_COMPARE_REG $11,0
#define COP0_CVMMEMCTL_REG $11,7
#define COP0_STATUS_REG $12,0
#define COP0_CAUSE_REG $13,0
#define COP0_PAGEGRAIN_REG $5,1
#define COP0_ENTRYLO0_REG $2,0
#define COP0_ENTRYLO1_REG $3,0
#define COP0_PAGEMASK_REG $5,0
#define COP0_ENTRYHI_REG $10,0
#define COP0_INDEX_REG $0,0
#define COP0_CONFIG1_REG $16,1
#define COP0_CONFIG3_REG $16,3
#define COP0_CONFIG4_REG $16,4
#define COP0_CONTEXT_REG $4,0
#define COP0_XCONTEXT_REG $20,0
#define COP0_WIRED_REG $6,0
#define COP0_EPC_REG $14,0
#define COP0_ERROREPC_REG $30,0
#define COP0_BADVADDR_REG $8,0
#define COP0_DEPC_REG $24,0
#define COP0_DEBUG_REG $23,0
#define COP0_MDEBUG_REG $22,0
#define COP0_DESAVE_REG $31,0
#define COP0_PERF_CNT0 $25,1
#define COP0_PERF_CNT1 $25,3
#define COP0_EBASE $15,1
#define COP0_PROC_ID $15,0

#include <config.h>
#include <version.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>

#include "cvmx-bootloader.h"

#include "octeon_mem_map.h"
#define  OCTEON_GPIO_RX_DAT                                     0x8001070000000880
#define  OCTEON_GPIO_TX_CLR                                     0x8001070000000890
#define  OCTEON_GPIO_TX_SET                                     0x8001070000000888


#define RVECENT(f,n) \
   b f; nop
#define XVECENT(f,bev) \
   b f     ;           \
   li k0,bev


/* func argument is used to create a  mark, must be unique */
#define GETOFFSET(reg, func)      \
	bal 	func ##_mark;     \
	nop;                      \
	.word .;                  \
func ##_mark:	                  \
	lw	reg, 0(ra);       \
	dsubu	reg, ra, reg      

#define JAL(func)      		  \
	bal      func ##_mark;    \
	nop;                      \
	.word .;                  \
func ##_mark:             	  \
	lw	t8, 0(ra);	  \
	dsubu    t8, ra, t8; 	  \
        la      t9, func;         \
        daddu     t9, t9, t8;       \
        jalr    t9;               \
        nop

/* 7-segment display */
#if CONFIG_OCTEON_SIM_HW_DIFF
.macro  _led_write_chars_4 r0,r1,d0,d1,d2,d3
        nop
.endm
.macro  _led_write_chars_8 r0,r1,d0,d1,d2,d3,d4,d5,d6,d7
        nop
.endm
.macro  _led_write_one_char r0,r1,off
        nop
.endm
#else
#if CONFIG_OCTEON_KODAMA
#define LED_REG_VAL     0x80001a00
#define LED_BASE_ADDR   0xba000000
#else
#define LED_REG_VAL     0x80001d02
#define LED_BASE_ADDR   0xbd020000
#endif
.macro  _led_write_one_char r0,r1,off
        sb      \r1, \off(\r0)
.endm
.macro  _led_write_chars_top4 r0,r1,d0,d1,d2,d3
        /* Map LED display */
        dli     \r0, 0x8001180000000020
        dli     \r1, LED_REG_VAL
        sd      \r1, 0(\r0); 
        /* Write string */
        li      \r0, LED_BASE_ADDR   /* Base address of LED */
        li      \r1, \d0
        _led_write_one_char \r0,\r1,0xf8
        li      \r1, \d1
        _led_write_one_char \r0,\r1,0xf9
        li      \r1, \d2
        _led_write_one_char \r0,\r1,0xfa
        li      \r1, \d3
        _led_write_one_char \r0,\r1,0xfb
.endm
.macro  _led_write_chars_bot4 r0,r1,d0,d1,d2,d3
        /* Map LED display */
        dli     \r0, 0x8001180000000020
        dli     \r1, LED_REG_VAL
        sd      \r1, 0(\r0); 
        /* Write string */
        li      \r0, LED_BASE_ADDR   /* Base address of LED */
        li      \r1, \d0
        _led_write_one_char \r0,\r1,0xfc
        li      \r1, \d1
        _led_write_one_char \r0,\r1,0xfd
        li      \r1, \d2
        _led_write_one_char \r0,\r1,0xfe
        li      \r1, \d3
        _led_write_one_char \r0,\r1,0xff
.endm
.macro  _led_write_chars_8 r0,r1,d0,d1,d2,d3,d4,d5,d6,d7
        /* Map LED display */
        dli     \r0, 0x8001180000000020
        dli     \r1, LED_REG_VAL
        sd      \r1, 0(\r0); 
        /* Write string */
        li      \r0, LED_BASE_ADDR   /* Base address of LED */
        li      \r1, \d0
        _led_write_one_char \r0,\r1,0xf8
        li      \r1, \d1
        _led_write_one_char \r0,\r1,0xf9
        li      \r1, \d2
        _led_write_one_char \r0,\r1,0xfa
        li      \r1, \d3
        _led_write_one_char \r0,\r1,0xfb
        li      \r1, \d4
        _led_write_one_char \r0,\r1,0xfc
        li      \r1, \d5
        _led_write_one_char \r0,\r1,0xfd
        li      \r1, \d6
        _led_write_one_char \r0,\r1,0xfe
        li      \r1, \d7
        _led_write_one_char \r0,\r1,0xff
.endm
.macro  _dump_reg32 r0,r1,reg
        bal     1f
        nop
        .byte   '0'
        .byte   '1'
        .byte   '2'
        .byte   '3'
        .byte   '4'
        .byte   '5'
        .byte   '6'
        .byte   '7'
        .byte   '8'
        .byte   '9'
        .byte   'A'
        .byte   'B'
        .byte   'C'
        .byte   'D'
        .byte   'E'
        .byte   'F'
1:
        /* Map LED display */
        dli     \r0, 0x8001180000000020
        dli     \r1, LED_REG_VAL
        sd      \r1, 0(\r0); 
        li      \r0, LED_BASE_ADDR   /* Base address of LED */
        /* Write string */
        srl     \r1, \reg, 28
        andi    \r1, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xf8(\r0)
        srl     \r1, \reg, 24
        andi    \r1, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xf9(\r0)
        srl     \r1, \reg, 20
        andi    \r1, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xfa(\r0)
        srl     \r1, \reg, 16
        andi    \r1, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xfb(\r0)
        srl     \r1, \reg, 12
        andi    \r1, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xfc(\r0)
        srl     \r1, \reg, 8
        andi    \r1, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xfd(\r0)
        srl     \r1, \reg, 4
        andi    \r1, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xfe(\r0)
        andi    \r1, \reg, 0xf
        add     \r1, ra
        lb      \r1, (\r1)
        sb      \r1, 0xff(\r0)
.endm
#endif /* CONFIG_OCTEON_SIM_HW_DIFF */

.macro  _get_gpio r0
        dli     \r0, 0x8001070000000880
        ld      \r0, (\r0)
        nop
.endm

.macro  __acquire_gp r0,r1
        /* Branch and link to get current PC in ra */
        bal     1f
        nop
        .word   _GLOBAL_OFFSET_TABLE_  /* This contains the linked address of the GOT */
        .word   . - 4                  /* This contains the link time address of the prev. word,
                                        which is also what the link time expected PC value is */
1:      move    gp, ra     /* Move current PC into gp register */
        lw      \r0, 0(ra) /* Load linked address of the GOT into r0 */
        lw      \r1, 4(ra) /* Load the link time address of the GOT storage location into r1 */
        sub     \r0, \r1   /* Subtract r1 from r0. */
                           /* r0 = offset link-time GOT address and the link time expected PC */
                           /* Add this offset to the current PC (copied into gp above), */
                           /* so that gp now has the current runtime GOT address */
        add     gp, \r0    /* calculate current location of offset table */
.endm
        .set noreorder
    
#ifdef CONFIG_OCTEON
#undef mips64
#endif

	.extern _GLOBAL_OFFSET_TABLE_

        .globl _start
        .text
_start:
        RVECENT(reset,0)        /* U-boot entry point */
        /* The above jump instruction/nop are considered part of the bootloader_header_t structure
        ** but are not changed when the header is updated. */

#if 0
        RVECENT(reset,1)        /* software reboot */
        RVECENT(romReserved,2)
        RVECENT(romReserved,3)
        RVECENT(romReserved,4)
        RVECENT(romReserved,5)
        RVECENT(romReserved,6)
        RVECENT(romReserved,7)
        RVECENT(romReserved,8)
        RVECENT(romReserved,9)
        RVECENT(romReserved,10)
        RVECENT(romReserved,11)
        RVECENT(romReserved,12)
        RVECENT(romReserved,13)
        RVECENT(romReserved,14)
        RVECENT(romReserved,15)
        RVECENT(romReserved,16)
        RVECENT(romReserved,17)
        RVECENT(romReserved,18)
        RVECENT(romReserved,19)
        RVECENT(romReserved,20)
        RVECENT(romReserved,21)
        RVECENT(romReserved,22)
        RVECENT(romReserved,23)
        RVECENT(romReserved,24)
        RVECENT(romReserved,25)
        RVECENT(romReserved,26)
        RVECENT(romReserved,27)
        RVECENT(romReserved,28)
        RVECENT(romReserved,29)
        RVECENT(romReserved,30)
        RVECENT(romReserved,31)
        RVECENT(romReserved,32)
        RVECENT(romReserved,33)
        RVECENT(romReserved,34)
        RVECENT(romReserved,35)
        RVECENT(romReserved,36)
        RVECENT(romReserved,37)
        RVECENT(romReserved,38)
        RVECENT(romReserved,39)
        RVECENT(romReserved,40)
        RVECENT(romReserved,41)
        RVECENT(romReserved,42)
        RVECENT(romReserved,43)
        RVECENT(romReserved,44)
        RVECENT(romReserved,45)
        RVECENT(romReserved,46)
        RVECENT(romReserved,47)
        RVECENT(romReserved,48)
        RVECENT(romReserved,49)
        RVECENT(romReserved,50)
        RVECENT(romReserved,51)
        RVECENT(romReserved,52)
        RVECENT(romReserved,53)
        RVECENT(romReserved,54)
        RVECENT(romReserved,55)
        RVECENT(romReserved,56)
        RVECENT(romReserved,57)
        RVECENT(romReserved,58)
        RVECENT(romReserved,59)
        RVECENT(romReserved,60)
        RVECENT(romReserved,61)
        RVECENT(romReserved,62)
        RVECENT(romReserved,63)
#else
        /* Leave room for bootloader_header_t header at start of binary. This header
        ** is used to identify the board the bootloader is for, what address it is linked at,
        ** failsafe/normal, etc.  It also contains a CRC of the entire image.
        */
	

        .org 0x200
#endif
        XVECENT(romExcHandle,0x200)     /* bfc00200: R4000 tlbmiss vector */
        RVECENT(romReserved,65)
        RVECENT(romReserved,66)
        RVECENT(romReserved,67)
        RVECENT(romReserved,68)
        RVECENT(romReserved,69)
        RVECENT(romReserved,70)
        RVECENT(romReserved,71)
        RVECENT(romReserved,72)
        RVECENT(romReserved,73)
        RVECENT(romReserved,74)
        RVECENT(romReserved,75)
        RVECENT(romReserved,76)
        RVECENT(romReserved,77)
        RVECENT(romReserved,78)
        RVECENT(romReserved,79)
        XVECENT(romExcHandle,0x280)     /* bfc00280: R4000 xtlbmiss vector */
        RVECENT(romReserved,81)
        RVECENT(romReserved,82)
        RVECENT(romReserved,83)
        RVECENT(romReserved,84)
        RVECENT(romReserved,85)
        RVECENT(romReserved,86)
        RVECENT(romReserved,87)
        RVECENT(romReserved,88)
        RVECENT(romReserved,89)
        RVECENT(romReserved,90)
        RVECENT(romReserved,91)
        RVECENT(romReserved,92)
        RVECENT(romReserved,93)
        RVECENT(romReserved,94)
        RVECENT(romReserved,95)
        XVECENT(romExcHandle,0x300)     /* bfc00300: R4000 cache vector */
        RVECENT(romReserved,97)
        RVECENT(romReserved,98)
        RVECENT(romReserved,99)
        RVECENT(romReserved,100)
        RVECENT(romReserved,101)
        RVECENT(romReserved,102)
        RVECENT(romReserved,103)
        RVECENT(romReserved,104)
        RVECENT(romReserved,105)
        RVECENT(romReserved,106)
        RVECENT(romReserved,107)
        RVECENT(romReserved,108)
        RVECENT(romReserved,109)
        RVECENT(romReserved,110)
        RVECENT(romReserved,111)
        XVECENT(romExcHandle,0x380)     /* bfc00380: R4000 general vector */
        RVECENT(romReserved,113)
        RVECENT(romReserved,114)
        RVECENT(romReserved,115)
        RVECENT(romReserved,116)
        RVECENT(romReserved,116)
        RVECENT(romReserved,118)
        RVECENT(romReserved,119)
        RVECENT(romReserved,120)
        RVECENT(romReserved,121)
        RVECENT(romReserved,122)
        RVECENT(romReserved,123)
        RVECENT(romReserved,124)
        RVECENT(romReserved,125)
        RVECENT(romReserved,126)
        RVECENT(romReserved,127)
        XVECENT(romExcHandle,0x400)     /* bfc00400: */
        RVECENT(romReserved,129)
        RVECENT(romReserved,130)
        RVECENT(romReserved,131)
        RVECENT(romReserved,132)
        RVECENT(romReserved,133)
        RVECENT(romReserved,134)
        RVECENT(romReserved,135)
        RVECENT(romReserved,136)
        RVECENT(romReserved,137)
        RVECENT(romReserved,138)
        RVECENT(romReserved,139)
        RVECENT(romReserved,140)
        RVECENT(romReserved,141)
        RVECENT(romReserved,142)
        RVECENT(romReserved,143)
        RVECENT(debugHandler,0x480)     /* bfc00480:  Debug vector*/
        RVECENT(romReserved,145)
        RVECENT(romReserved,146)
        RVECENT(romReserved,147)
        RVECENT(romReserved,148)
        RVECENT(romReserved,149)
        RVECENT(romReserved,150)
        RVECENT(romReserved,151)
        RVECENT(romReserved,152)
        RVECENT(romReserved,153)
        RVECENT(romReserved,154)
        RVECENT(romReserved,155)
        RVECENT(romReserved,156)
        RVECENT(romReserved,157)
        RVECENT(romReserved,158)
        RVECENT(romReserved,159)
        /* Reserve extra space so that when we use the boot bus local memory
        ** segment to remap the debug exception vector we don't overwrite
        ** anything useful */
    
    
        .align 8
reset:
        nop

        /* In PCI the code before InitTLBStart is only executed on core 0.
        ** All other cores branch from the boot bus local memory section
        ** to the InitTLBStart in the relocated u-boot image.
        */
        mfc0    a5, COP0_STATUS_REG
        li      v0,0xE0  // enable 64 bit mode for CSR access
        or      v0, v0, a5
        mtc0    v0, COP0_STATUS_REG


#if CONFIG_OCTEON
        /* Speed up flash.  Here we use a hardcoded value that will
        ** work up to 1 GHz.  The speed will be adjusted again once
        ** the actual speed of the CPU is determined, but this setting here
        ** makes things much faster, as the hardware default at boot is
        ** very slow. */
        dli     a4, 0x0007fc024724d343
        dli     a5, 0x8001180000000040  /* MIO_BOOT_REG_TIM0 */
        sd      a4, 0(a5)

#ifdef ENABLE_BOARD_DEBUG
	/* Config output bits */
	dli     a4, 0x8001070000000840 /* GPIO 8 */
        ld      a5, 0(a4)
        or      a5, 1
        sd      a5, 0(a4)
	dli     a4, 0x8001070000000848 /* GPIO 9 */
        ld      a5, 0(a4)
        or      a5, 1
        sd      a5, 0(a4)
	dli     a4, 0x8001070000000850 /* GPIO 10 */
        ld      a5, 0(a4)
        or      a5, 1
        sd      a5, 0(a4)
	dli     a4, 0x8001070000000858 /* GPIO 11 */
        ld      a5, 0(a4)
        or      a5, 1
        sd      a5, 0(a4)
	dli     a4, 0x8001070000000860 /* GPIO 12 */
        ld      a5, 0(a4)
        or      a5, 1
        sd      a5, 0(a4)
	dli     a4, 0x8001070000000868 /* GPIO 13 */
        ld      a5, 0(a4)
        or      a5, 1
        sd      a5, 0(a4)
	dli     a4, 0x8001070000000870 /* GPIO 14 */
        ld      a5, 0(a4)
        or      a5, 1
        sd      a5, 0(a4)

	/* Clear output bits */
       	dli     a4, OCTEON_GPIO_TX_CLR
        li      a5, 0x7f                /* Clear bits 14:8 */
        sll     a5, 8
        sd      a5, 0(a4)
#endif

/* Start of Octeon setup */

#if defined(CONFIG_OCTEON_PCIX_WORKAROUND)
        /* Only core 0 executes this. */
        mfc0    a0, COP0_EBASE
        andi    a0, 0xFF   // get core
        bne     a0, zero, pcix_workaround_end
        nop

        /* Check chip revision, and skip PCIX workaround if we are not on pass 1 */
        mfc0    a4, COP0_PROC_ID
        andi    a4, 0xff
        bnez    a4, pcix_workaround_end
        nop

        /* Set valid value for PCI_CTL_STATUS_2[TSR_HWM] */
        dli     a5, 0x80011F0000001188 
        dli     a4, (1<<4)
        sw      a4, 0(a5)
pcix_workaround_end:
#endif

    /* Check what core we are - if core 0, branch to init tlb
    ** loop in flash.  Otherwise, look up address of init tlb
    ** loop that was saved in the boot vector block.
    */
        mfc0    a0, COP0_EBASE
        andi    a0, 0xFF   // get core
#if 0
        /* This code is used to generate the stub that is loaded
        ** by the oct-pci-boot program */
        li      a5, 0x400000
        bnez     a0, later_cores_ram_boot
        nop
        j       a5
        nop
later_cores_ram_boot:         
#else
        beqz    a0, InitTLBStart_local
        nop
#endif
        break
        /* We should never get here - non-zero cores now go directly to
        ** tlb init from the boot stub in movable region. */
        
        .globl InitTLBStart
InitTLBStart:
InitTLBStart_local:


#if CONFIG_OCTEON
        /* If we don't have working memory yet configure a bunch of
    	** scratch memory, and set the stack pointer to the top
        ** of it.  This allows us to go to C code without having
    	** memory set up
        */
#define SCRATCH_STACK_LINES 0x36   /* MAX is 0x36 */
        dmfc0   v0, COP0_CVMMEMCTL_REG
        dsrl    v0, 9
        dsll    v0, 9
        ori     v0, 0x100 | SCRATCH_STACK_LINES   // setup SCRATCH_STACK_LINES scratch lines of scratch
        dmtc0   v0, COP0_CVMMEMCTL_REG
        li      sp, 0xffff8000 + (SCRATCH_STACK_LINES * 128)   // set stack to top of scratch memory
        /* Clear scratch for CN63XX pass 2.0 errata Core-15169*/
        li      t0, 0xffff8000
clear_scratch:
        sd      zero, 0(t0)
        addi    t0, 8
        bne     t0, sp, clear_scratch
        nop
#endif




        /* This code run on all cores - core 0 from flash,
    ** the rest from DRAM.  When booting from PCI, non-zero cores
    ** come directly here from the boot vector - no earlier code in this
    ** file is executed.
    */

    /* Some generic initialization is done here as well, as we need this done on
    ** all cores even when booting from PCI
    */
        /* Clear watch registers.
         */
        mtc0    zero, CP0_WATCHLO
        mtc0    zero, CP0_WATCHHI

        /* STATUS register */
        mfc0    k0, CP0_STATUS
        li      k1, ~ST0_IE
        and     k0, k1
        mtc0    k0, CP0_STATUS

        /* CAUSE register */
        mtc0    zero, CP0_CAUSE

        /* Init Timer */
        dmtc0   zero, CP0_COUNT
        dmtc0   zero, CP0_COMPARE


        mfc0    a5, COP0_STATUS_REG
        li      v0,0xE0  // enable 64 bit mode for CSR access
        or      v0, v0, a5
        mtc0    v0, COP0_STATUS_REG

    
        dli     v0, 1 << 29  /* Enable large physical address support in TLB */
        mtc0    v0, COP0_PAGEGRAIN_REG

InitTLB:
        dmtc0   zero, COP0_ENTRYLO0_REG
        dmtc0   zero, COP0_ENTRYLO1_REG
        mtc0    zero, COP0_PAGEMASK_REG
        dmtc0   zero, COP0_CONTEXT_REG
        dli     v0, 0xFFFFFFFF90000000 // Use an offset into kseg0 so we won't conflict with Mips1 legacy TLB clearing
        mfc0    a0, COP0_CONFIG1_REG
        srl     a0, a0, 25
        mfc0    a1, COP0_CONFIG3_REG // Check if config4 reg present 
        bbit0   a1, 31, 2f
        and     a0, a0, 0x3F         // a0 now has the max mmu entry index
        mfc0    a1, COP0_CONFIG4_REG
        bbit0   a1, 14, 2f           // check config4[MMUExtDef]
        nop
        ins     a0, a1, 6, 8         // append config4[MMUSizeExt] to most significant bit of config1[MMUSize-1]
        and     a0, a0, 0x3fff       // a0 now includes max entries for cn6xxx 
2:
        dmtc0   zero, COP0_XCONTEXT_REG
        mtc0    zero, COP0_WIRED_REG
InitTLBloop:
        dmtc0   v0, COP0_ENTRYHI_REG
        nop
        nop
        tlbp
        nop
        nop
        mfc0    v1, COP0_INDEX_REG
        daddiu  v0, v0, 1<<13
        bgez    v1, InitTLBloop
        nop
        nop

        mtc0    a0, COP0_INDEX_REG
        nop
        nop
        tlbwi
        nop
        nop
        bne     a0, zero, InitTLBloop
        addiu   a0, -1

#ifdef ENABLE_BOARD_DEBUG
	/* Set GPIO output bits */
       	dli     a4, OCTEON_GPIO_TX_SET
        li      a5, 0x7e
        sll     a5, 8
        sd      a5, 0(a4)
#endif

        mthi    zero
        mtlo    zero

        /* Set up status register */        
        mfc0    v0, COP0_STATUS_REG
        li      a4, 1 << 28  // enable cop0 access
        or      v0, a4
        li      a4, 1 << 30  // enable cop2 access
        or      v0, a4

        /* Must leave BEV set here, as DRAM is not configured for core 0.
        ** Also, BEV must be 1 later on when the exception base address is set. */

        li      a4, ~0xff00 // mask all interrupts
        and     v0, a4
        li      a4, ~0xff 
	and     v0, a4
	li      a4, ~(1 << 19)  /* Clear NMI (used to start cores other than core 0) */
        and     v0, a4
        ori     v0, 0xE5  // enable 64 bit, disable interrupts 
        mtc0    v0, COP0_STATUS_REG


        dli     v0,0xC000000F // enable all readhw locations
        mtc0    v0, COP0_HWRENA_REG
        
        dmfc0   v0, COP0_CVMCTL_REG
        mfc0    a4, COP0_PROC_ID
        li      a5, 0x000d0000 // Octeon pass1 chip id
        bne     a4, a5, skip_icachetch_disable
        nop
        ori     v0, 1<<13   // disable icache prefectch - errata core 8 (pass1 only)
skip_icachetch_disable:
        ori     v0, 1<<14   // enable fixup of unaligned mem access
        dmtc0   v0, COP0_CVMCTL_REG

        /* Setup scratch memory.  This is also done in
        ** cvmx_user_app_init, and this code will be removed
        ** from the bootloader in the near future.
        */
        dmfc0   v0, COP0_CVMMEMCTL_REG
        mfc0    a4, COP0_PROC_ID
        li      a5, 0x000d9000 // Octeon pass1 chip id
        bgt     a5, a4, 71f
        ori     v0, 0x104      // setup 4 lines of scratch
        ori     a6, a5, 8      // Octeon cn63xx pass2 chip id
        bge     a4, a6, 71f
        nop
        li      a6, 4
        ins     v0, a6, 11, 4  // Set WBTHRESH=4 as per Core-14752 errata
71:
        dmtc0   v0, COP0_CVMMEMCTL_REG
        

        // clear these to avoid immediate interrupt in noperf mode
        dmtc0   zero, COP0_COMPARE_REG            // clear timer interrupt
        dmtc0   zero, COP0_COUNT_REG            // clear timer interrupt
        dmtc0   zero, COP0_PERF_CNT0              // clear perfCnt0
        dmtc0   zero, COP0_PERF_CNT1              // clear perfCnt1




#if CONFIG_OCTEON_UBOOT_TLB
        /* Set up TLB mappings for u-boot code in flash.
        */

/************************************************************************************************/
/* Start of GP hack.  This needs to be done once properly for all code.
** old relocation hacks need to be removed. */
        /* Branch and link to get current PC in ra */
        bal     2f
        nop
        .word   _GLOBAL_OFFSET_TABLE_  /* This contains the linked address of the GOT */
        /* The ra register now contains the runtime address of the above memory location */
        .word   . - 4                  /* This contains the link time address of the previous word, */
2:

        move    gp, ra    /* Move current PC into gp register */
        lw      a5, 0(ra) /* Load linked address of the GOT into a5 */
        lw      a6, 4(ra) /* Load the link time address of the GOT storage location into a6 */
        sub     a5, a6    /* Subtract a6 from t1. */
        /* a5 now contains the difference between the link-time GOT table address and the link time expected PC */

        /* Add this difference to the current PC (copied into gp above) so that gp now has the current runtime
        ** GOT table address */
        daddu   gp, a5  # calculate current location of offset table
/* End of GP hack. */
/************************************************************************************************/


        /* Use a bal to get the current PC into ra.  Since this bal is to
        ** the address immediately following the delay slot, the ra is
        ** the address of the label.  We then use this to get the actual
        ** address that we are executing from. */
        bal     __dummy
        nop
__dummy:

        /* Get the actual address that we are running at */
        la      a6, _start  // Linked address of _start
        la      a7, __dummy
        dsubu   t0, a7, a6  // offset of __dummy label from _start
        dsubu   a7, ra, t0  // a7 now has actual address of _start

        /* Save actual _start address in s7.  This is where we
        ** are executing from, as opposed to where the code is 
        ** linked. */
        move s7, a7


        /* s7 has actual address of _start.  If this is
        ** on the boot bus, it will be between 0xBFC000000 and 0xBFFFFFFF.
        ** If it is on the boot bus, use 0xBFC00000 as the physical address
        ** for the TLB mapping, as we will be adjusting the boot bus
        ** to make this adjustment.
        ** If we are running from DRAM (remote-boot), then we want to use the real
        ** address in DRAM.
        */

        /* Check to see if we are running from flash - we expect that to
        ** be 0xb0000000-0xbfffffff (0x10000000-0x1fffffff, unmapped/uncached) */
        li      t2, 0xb0000000
        dsubu   t2, s7
        bgez    t2, uboot_in_ram
        nop

        /* This code is only executed if booting from flash. */
        /*  For flash boot (_not_ RAM boot), we do a workaround for
        ** an LLM errata on CN38XX and CN58XX parts. */

#ifdef CONFIG_OCTEON_LLM_WORKAROUND
        /* Workaround for LLM bug where resetting
        ** the chip during LLM activity causes problems.
        ** Fix is to do another reset.  Note that this workaround
        ** does not handle the case where the user application
        ** does a soft reset during LLM activity.
        */
        mfc0    a0, COP0_EBASE
        andi    a0, 0xFF   // get core
        bne     a0, zero, llm_workaround_end
        nop

        mfc0    v0, COP0_STATUS_REG
        srl     v0, 20
        andi    v0, 1
        bne     v0, zero, llm_workaround_end  /* soft reset, so just continue */
        nop

        /* This was a hard reset, so we will do a soft reset here */
        dli     a4, 0x8001070000000740
        li      a5, 1
        sd      a5, 0(a4)

        /* Loop forever after issuing soft reset */ 
llm_workaround_loop:
        j       llm_workaround_loop
        nop
llm_workaround_end:    
#endif

        /* Use BFC00000 as physical address for TLB mappings when booting
        ** from flash, as we will adjust the boot bus mappings to make this
        ** mapping correct. */
        li      a7, 0xBFC00000
        dsubu   s6, s7, a7  /* Save flash offset in s6 */

uboot_in_ram:
        /* U-boot address is now in reg a7, and is 4 MByte aligned.
        ** (boot bus addressing has been adjusted to make this happen for flash,
        ** and for DRAM this alignment must be provided by the remote boot utility. */

        /* U-boot now starts at 0xBFC00000.  Use a single 4 MByte TLB mapping
        ** to map u-boot. */
        move a0, a6  /* Virtual addr in a0 */
        move a1, a7  /* Physical addr in a1 */

        /* Now we need to remove the MIPS address space bits.  For this we
        ** need to determine if it is a 32 bit compatability address or not. */

        li      t0, 0x80000000 /* 'lowest' address in compatability space */
        dsubu   t0, t0, a1
        blt     t0, zero, compat_space
        nop

        /* We have a xkphys address, so strip off top bit */
        dli     t0, 0x7fffffffffffffff
        and     a1, a1, t0
        b       addr_fixup_done
        nop



compat_space:
        dli  a2, 0x1fffffff
        and  a1, a1, a2  /* Mask phy addr to remove address space bits */

addr_fixup_done:
        /* Currenty the u-boot image size is limited to 4 MBytes.  In order to support larger
        ** images the flash mapping will need to be changed to be able to access more than that
        ** before C code is run.  Until that is done, we just use a 4 MByte mapping for the 
        ** secondary cores as well. */

        li      a2, 2*1024*1024  /* page size (only support 4 Meg binary size for now for core 0) */

        mfc0    a4, COP0_EBASE
        andi    a4, 0xFF   // get core
        beq     a4, zero, core_0_tlb
        nop

        /* Now determine how big a mapping to use for secondary cores,
        ** which need to map all of u-boot + heap in DRAM */
        /* Here we look at the alignment of the the physical address,
        ** and use the largest page size possible.  In some cases
        ** this can result in an oversize mapping, but for secondary cores
        ** this mapping is very short lived. */

        /* Physical address in a1 */
        li      a2, 1
foo:
        sll     a2, 1
        and     a5, a1, a2
        beq     a5, zero, foo
        nop

        /* a2 no contains largest page size we can use */
core_0_tlb:
        JAL(single_tlb_setup)
    


        /* If we are already running from ram, we don't need to muck
        ** with boot bus mappings. */
        li      t2, 0xb0000000
        dsubu   t2, s7
        bgez    t2, uboot_in_ram2
        nop

        /* We now have the TLB set up, so we need to remap the boot bus.
        ** This is tricky, as we are running from flash, and will be changing
        ** the addressing of the flash.
        */

        /* Enable movable boot bus region 0, at address 0x10000000 */
        dli     a4, 0x8001180000000080
        dli     a5, 0x81000000
        sd      a5, 0(a4)

        /* Copy code to that remaps the boot bus to movable region */

        dli     a4, 0x8001180000000090  /* Boot_loc_adr */
        sd      zero, 0(a4)
        dli     a4, 0x8001180000000098  /* boot_loc_dat */

        la      a6, change_boot_mappings
        GETOFFSET(a5, change_boot_mappings);
        daddu     a5, a5, a6

        ld      a7, 0(a5)
        sd      a7, 0(a4)
        ld      a7, 8(a5)
        sd      a7, 0(a4)
        ld      a7, 16(a5)
        sd      a7, 0(a4)
        ld      a7, 24(a5)
        sd      a7, 0(a4)
        ld      a7, 32(a5)
        sd      a7, 0(a4)

        /* Read from an RML register to ensure that the previous writes have completed
        ** before we branch to the movable region. */
        dli     a4, 0x8001180000000080
        ld      zero, 0(a4)


        /* Compute value for boot bus configuration register */

        /* Read region 0 config so we can _modify_ the base address field */
        dli     a4, 0x8001180000000000   /* region 0 config */
        ld      a0, 0(a4)
        dli     a4, 0xf0000000           /* Mask off bits we want to save */
        and     a4, a4, a0
        dli     a0, 0x0fff0000            /* Force size to max */
        or      a4, a4, a0

        move    a5, s6
        srl     a5, 16          /* Convert to 64k blocks, as used by boot bus config */
        li      a6, 0x1fc0      /* 'normal' boot bus base config value */
        subu    a6, a6, a5      /* Subtract offset */
        or      a0, a4, a6      /* combine into register value to pass to boot bus routine */

        /* Branch there */
        la      a1, __mapped_continue_label
        li      a4, 0x10000000
        j       a4
        nop
        break  /* We never get here, as we go directly to __mapped_continue_label */





uboot_in_ram2:
        /* Now jump to address in TLB mapped memory to continue execution */
        la a4, __mapped_continue_label
        j  a4
        nop

__mapped_continue_label:

        nop
        nop
        nop
        nop




#endif   /* CONFIG_OCTEON_UBOOT_TLB */

        /* Check if we are core 0, if we are not then we need
        ** to vector to code in DRAM to do application setup, and
        ** skip the rest of the bootloader.  Only core 0 runs the bootloader
        ** and sets up the tables that the other cores will use for configuration
        */
        mfc0    a0, COP0_EBASE
        andi    a0, 0xFF   // get core
        beqz    a0, core_0_cont1
        nop
        // other cores look up addr from dram
        // DRAM controller already set up by first core
        li      a1, BOOT_VECTOR_NUM_WORDS
        sll     a1, 2   // convert to word addresses
        mul     a0, a0, a1
        li      a1, BOOT_VECTOR_BASE
        add     a1, a1, a0

        /* No cache init required */
        lw      t9, 8(a1)
        lw      k0, 12(a1)
        j       t9
        nop
core_0_cont1:
#endif        


    /* From here on, only core 0 runs, other cores have branched
    ** away.
    */

#if (CONFIG_OCTEON && !CONFIG_NO_RELOCATION)
#else
        li      a4, CFG_SDRAM_BASE + CFG_INIT_SP_OFFSET
        la      sp, 0(a4)
#endif



        /* Initialize GOT pointer.
        ** Global symbols can't be resolved before this is done, and as such we can't
        ** use any global symbols in this code.  We use the bal/ move xxx,ra combination to access
        ** data in a PC relative manner to avoid this.  This code will correctly set the
        ** gp regardless of whether the code has already been relocated or not.
        ** This code determines the current gp by computing the link time (gp - pc)
        ** and adding this to the current pc.
        ** runtime_gp = runtime_pc + (linktime_gp - linktime_pc)
        ** U-boot is running from the address it is linked at at this time, so this
        ** general case code is not strictly necessary here.
        */

        /* Branch and link to get current PC in ra */
        bal     1f
        nop
/*        .extern _GLOBAL_OFFSET_TABLE_	 moved higher up - used in image header */
        .word   _GLOBAL_OFFSET_TABLE_  /* This contains the linked address of the GOT */
        /* The ra register now contains the runtime address of the above memory location */

        .word   . - 4                  /* This contains the link time address of the previous word,
                                        which is also what the link time expected PC value is */
1:
#if 1
        move    gp, ra    /* Move current PC into gp register */
        lw      a5, 0(ra) /* Load linked address of the GOT into a5 */
        lw      a6, 4(ra) /* Load the link time address of the GOT storage location into a6 */
        sub     a5, a6    /* Subtract a6 from t1. */
        /* a5 now contains the difference between the link-time GOT table address and the link time expected PC */

        /* Add this difference to the current PC (copied into gp above) so that gp now has the current runtime
        ** GOT table address */
        daddu   gp, a5  # calculate current location of offset table
#else
        /* This single line is equivalent to the above if and only if the code is running
        ** from the address it is linked at. */
        lw      gp, 0(ra)
#endif


        
        
#ifndef CONFIG_OCTEON
        /* must not use .cprestore here, don't know how to avoid warning */
        /* Initialize any external memory.
         */
        jal     memsetup
    	nop
        /* Initialize caches...
         */
        sync
        cache 0, 0($0)
        sync
        cache 9, 0($0)
        sync
        
        jal     mips_cache_reset
        nop

        /* ... and enable them.
         */
        li      a4, CONF_CM_CACHABLE_NONCOHERENT
        mtc0    a4, CP0_CONFIG


        /* Set up temporary stack.
         */
        li      a0, CFG_INIT_SP_OFFSET
        jal     mips_cache_lock
        nop

#endif   /* !CONFIG_OCTEON */

        
#ifdef ENABLE_BOARD_DEBUG
	/* Set GPIO output bits */
       	dli     a4, OCTEON_GPIO_TX_SET
        li      a5, 0x7d
        sll     a5, 8
        sd      a5, 0(a4)
#endif

        la      t9, board_init_f  /* doesn't return... */
        j       t9
        nop

/*
 * void relocate_code (addr_sp, gd, addr_moni)
 *
 * This "function" does not return, instead it continues in RAM
 * after relocating the monitor code.
 *
 * a0 = addr_sp
 * a1 = gd address (on stack)
 * a2 = destination address (physical)
 * a3 = TLB page size (when TLB mapping used
 */
        .globl  relocate_code
        .ent    relocate_code
relocate_code:
#ifdef ENABLE_BOARD_DEBUG
	/* Set GPIO output bits */
       	dli     a4, OCTEON_GPIO_TX_SET
        li      a5, 0x7c
        sll     a5, 8
        sd      a5, 0(a4)
#endif

        move    v0, a1  /* Save gd address */

        move    sp, a0          /* Set new stack pointer                */

       
        li      a4, CFG_MONITOR_BASE /* Text base, 0xBFC00000 */
        la      a7, in_ram
        lw      a6, -12(a7)     /* a6 <-- uboot_end_data        */
        move    a5, a2

#if !CONFIG_OCTEON_UBOOT_TLB
        /*
         * Fix GOT pointer:
         *
         * New GOT-PTR = (old GOT-PTR - CFG_MONITOR_BASE) + Destination Address
         */
        move    t2, gp
        sub     gp, CFG_MONITOR_BASE  
        add     gp, a2                  /* gp now adjusted              */
        sub     t2, gp, t2              /* t2 <-- relocation offset     */
#endif

        /*
         * a4 = source address
         * a5 = target address
         * a6 = source end address
         */
        /* On the purple board we copy the code earlier in a special way
         * in order to solve flash problems
         */
#ifndef CONFIG_PURPLE

#ifndef CONFIG_OCTEON
1:
        lw      a7, 0(a4)
        sw      a7, 0(a5)
        addu    a4, 4
        blt     a4, a6, 1b
        addu    a5, 4                   /* delay slot                   */
#else
/* Use 64 bit copies to relocate code for speed.  We need to be careful to not copy too much
** as BSS comes immediately after the initialized data, and bss clearing is done _before_ the copy, 
** so if too much is copied we get garbage in some bss variable(s).
** The Linker script is constructed to align the end of the initialized data so that we can use
** 8 byte chunks.
*/ 
        beq     a4, a5, copyDone
1:
        ld      a7, 0(a4)
        sd      a7, 0(a5)
        daddu   a4, 8
        blt     a4, a6, 1b
        daddu   a5, 8                   /* delay slot                   */
#endif
    
#endif

        /* If caches were enabled, we would have to flush them here.
         */
copyDone:
#ifdef ENABLE_BOARD_DEBUG
	/* Set GPIO output bits */
       	dli     a4, OCTEON_GPIO_TX_SET
        li      a5, 0x7b
        sll     a5, 8
        sd      a5, 0(a4)
#endif

        /* Jump to where we've relocated ourselves.
         */
#if !CONFIG_OCTEON_UBOOT_TLB
        addi    a4, a2, in_ram - _start
    	synci 	0($0)  /* Flush icache (only required when relocating from ddr->ddr, as in pci boot */
        j       a4
        nop
#else

        /* We now need to redo the TLB.  We can call it directly
        ** since we are now running from the linked address.
        */
        /* Now replace the single TLB mapping that was set up in flash. */
        move    a1, a2

        la      a0, _start            
        /* Mapping size in a3 from above */
        move    a2, a3
        jal     single_tlb_setup
        nop




        /* We aren't changing execution (virtual) addresses, 
        ** so we don't need any address fixups here.
        */
        la      a4, in_ram
        j       a4
        nop
#endif

        .word   uboot_end_data
        .word   uboot_end
        .word   num_got_entries

in_ram:
#if !CONFIG_OCTEON_UBOOT_TLB
        /* Now we want to update GOT.
         */
        lw      a7, -4(a4)      /* a7 <-- num_got_entries       */
        addi    t0, gp, 8       /* Skipping first two entries.  */
        li      a6, 2
1:
        lw      a5, 0(t0)
        beqz    a5, 2f
        add     a5, t2
        sw      a5, 0(t0)
2:
        addi    a6, 1
        blt     a6, a7, 1b
        addi    t0, 4           /* delay slot                   */

        /* Clear BSS.
         */
        lw      a5, -12(a4)     /* a5 <-- uboot_end_data        */
        lw      a6, -8(a4)      /* a6 <-- uboot_end             */
        add     a5, t2          /* adjust pointers              */
        add     a6, t2

        sub     a5, 4
1:      addi    a5, 4
        bltl    a5, a6, 1b
        sw      zero, 0(a5)     /* delay slot                   */
#endif



        dmfc0   a0, COP0_CVMMEMCTL_REG
        srl     a0, 9
        sll     a0, 9
        mfc0    a4, COP0_PROC_ID
        li      a5, 0x000d9000 // Octeon cn63xx pass1 chip id
        bgt     a5, a4, 51f
        ori     a0, 0x104      // setup 4 lines of scratch
        ori     a6, a5, 8      // Octeon cn63xx pass2 chip id
        bge     a4, a6, 51f
        nop
        li      a6, 4
        ins     a0, a6, 11, 4  // Set WBTHRESH=4 as per Core-14752 errata
51:
        dmtc0   a0, COP0_CVMMEMCTL_REG


#ifdef ENABLE_BOARD_DEBUG
	/* Set GPIO output bits */
       	dli     a4, OCTEON_GPIO_TX_SET
        li      a5, 0x7a
        sll     a5, 8
        sd      a5, 0(a4)
#endif

        move    a0, a1
        move    a1, a2
#if CONFIG_OCTEON_UBOOT_TLB
        move    a0, v0  /* Saved gd pointer */
        li      a1, CFG_MONITOR_BASE  /* We are still running at the linked address */
#endif
        la      t9, board_init_r  /* doesn't return, runs main_loop() */
        j       t9
        nop

        .end    relocate_code


        /* Exception handlers.
         */
romReserved:
        b romReserved
        nop

        .globl   romExcHandle
romExcHandle:
        /* setup stub and stack area on scratchpad */
	.set	push
        .set    noat
	cache	1, 0($0) /* Invalidate D-Cache */
        dmfc0   k0, COP0_CVMMEMCTL_REG
	dsrl	k0, 9
	dsll	k0, 9
        ori     k0, 0x136            /* CVMSEGENAK | 0x36 */
        dmtc0   k0, COP0_CVMMEMCTL_REG
        dmfc0   k0, COP0_CVMMEMCTL_REG

/* Note: locally defined for bootloader purpose */
#define CVMX_SCRATCH_BASE       -32768 /* 0xffffffffffff8000 */
#define CVMX_SCRATCH_SIZE       (6912)
	    /* Store 64 bit CPU Registers */
	    la	k1, (CVMX_SCRATCH_BASE + CVMX_SCRATCH_SIZE)

        /* Clear scratch for CN63XX pass 2.0 errata Core-15169*/
	    la	k0, CVMX_SCRATCH_BASE
clear_scratch_exc:
        sd  zero, 0(k0)
        addi   k0, 8
        bne    k0, k1, clear_scratch_exc
        nop
        subu    k1, (96 + 1) * 8
        move    k0, k1

        sd	$0, 0(k1)
        sd	$1, 8(k1)
	.set	pop
        sd	$2, 16(k1)
        sd	$3, 24(k1)
        sd	$4, 32(k1)
        sd	$5, 40(k1)
        sd	$6, 48(k1)
        sd	$7, 56(k1)
        sd	$8, 64(k1)
        sd	$9, 72(k1)
        sd	$10, 80(k1)
        sd	$11, 88(k1)
        sd	$12, 96(k1)
        sd	$13, 104(k1)
        sd	$14, 112(k1)
        sd	$15, 120(k1)
        sd	$16, 128(k1)
        sd	$17, 136(k1)
        sd	$18, 144(k1)
        sd	$19, 152(k1)
        sd	$20, 160(k1)
        sd	$21, 168(k1)
        sd	$22, 176(k1)
        sd	$23, 184(k1)
        sd	$24, 192(k1)
        sd	$25, 200(k1)
        sd	$26, 208(k1)
        sd	$27, 216(k1)
        sd	$28, 224(k1)
        sd	$29, 232(k1)
        sd	$30, 240(k1)
        sd	$31, 248(k1)

        /* ...
         * COP0 are read from c-asm macros
        */

        /* acquire gp and set new sp onto scratchpad and,
         * invoke c-wrapper for crash reporter
        */
        move    sp, k1
        __acquire_gp    a5,a6

/* show TRAP and Core # */
_show_trap_and_core_num:
#if !CONFIG_OCTEON_SIM_HW_DIFF
        /* Print "TRAP" on rev 2 and later boards */
        _led_write_chars_8 k0,k1,'T','R','A','P',' ',' ',' ',' '
        li      k0, LED_BASE_ADDR   /* Base address of LED */
        li      k1, 0x23
        _led_write_one_char k0, k1, 0xfd

        bal     1f
        nop
        .byte   '0'
        .byte   '1'
        .byte   '2'
        .byte   '3'
        .byte   '4'
        .byte   '5'
        .byte   '6'
        .byte   '7'
        .byte   '8'
        .byte   '9'
        .byte   'A'
        .byte   'B'
        .byte   'C'
        .byte   'D'
        .byte   'E'
        .byte   'F'
1:      mfc0    k1, COP0_EBASE
        nop
        srl     k1, 4
        andi    k1, 0xF
        add     k1, ra
        lb      k1, (k1)
        nop
        _led_write_one_char k0, k1, 0xfe
        mfc0    k1, COP0_EBASE
        nop
        andi    k1, 0xF
        add     k1, ra
        lb      k1, (k1)
        nop
        _led_write_one_char k0, k1, 0xff
/* show TRAP and Core # */
#endif /* !CONFIG_OCTEON_SIM_HW_DIFF */

#if CONFIG_OCTEON_UBOOT_TLB
// Exception dumping should be improved, but still
// needs to be in assembly (or at least completely position independent)
        j       exception_handler_stage2
        nop

#else
        .extern  __cvmx_interrupt_default_exception_handler
        la      t9, __cvmx_interrupt_default_exception_handler
        move    a0, sp
        nop
        jal     t9
        nop
#endif
#if CONFIG_OCTEON_SIM_HW_DIFF
        /* End simulation with break instruction */
        break
        break
#endif /*CONFIG_OCTEON_SIM_HW_DIFF*/
romExcLoop:
        nop
        b       romExcLoop
        nop
#define COP0_DESAVE_REG $31,0
#define BOOTLOADER_DEBUG_TRAMPOLINE_DIV_4	(BOOTLOADER_DEBUG_TRAMPOLINE_CORE >> 2)
debugHandler:
        .globl debugHandler_entrypoint
	and     v0, a4
	.ent   debugHandler_entrypoint
        .align 8  /* Copied 8 bytes at a time, so make sure aligned */
debugHandler_entrypoint:	
        dmtc0   k0, COP0_DESAVE_REG
        li      k0, 0x80000000
        lw      k0, BOOTLOADER_DEBUG_TRAMPOLINE(k0)
        bnez    k0, debugHandler_notzero
	nop

        mfc0    k0, $15, 1  // read exception base reg.
        andi    k0, 0xff    // mask off core ID
        sll     k0, 2       // multiply by 4 words
        addiu   k0,  BOOTLOADER_DEBUG_TRAMPOLINE_DIV_4 
        addiu   k0,  BOOTLOADER_DEBUG_TRAMPOLINE_DIV_4 
        addiu   k0,  BOOTLOADER_DEBUG_TRAMPOLINE_DIV_4 
        addiu   k0,  BOOTLOADER_DEBUG_TRAMPOLINE_DIV_4 // add base offset - after exeption vectors for all cores

        rotr    k0, k0, 31   // set bit 31 for kseg0 access
        addi    k0, 1
        rotr    k0, k0, 1

        lw      k0, 0(k0)
        beqz    k0, debugHandler_zero
        nop

debugHandler_notzero:
        j       k0
        dmfc0   k0, COP0_DESAVE_REG
debugHandler_zero:
        deret
        dmfc0   k0, COP0_DESAVE_REG
    
        .globl OcteonBreak
OcteonBreak:
        break
        break
	.end  debugHandler_entrypoint


        /* This is the entry point of all cores except for Core 0. This code
            is installed by uboot at the reset vector in bootbus moveable
            region 0. Secondary cores never execute out of flash. */
        .globl SecondaryCoreInit
        .ent   SecondaryCoreInit
        .align 8
SecondaryCoreInit:
        /* Enable 64 bit addressing */
        mfc0    v0, COP0_STATUS_REG
        or      v0, 0xE0
        mtc0    v0, COP0_STATUS_REG

        /* Handle Core-14345 errata, (only in Octeon2 pass1), 
            clear L1 Dcache virtual tags if the core hit an NMI */
        cache   17, 0($0)

        /* Read the core number from EBASE so we can calculate where our jump
            location is stored in ram. This should always be InitTLBStart or
            zero. */
        mfc0    a0, COP0_EBASE
        andi    a0, 0xFF   // get core
        li      a1, BOOT_VECTOR_NUM_WORDS
        sll     a1, 2   // convert to word addresses
        mul     a0, a0, a1
        li      a1, BOOT_VECTOR_BASE
        add     a1, a1, a0

        /* If the execution address is zero then we need to sleep until we
            receive a NMI. If non-zero, then we should load the stack pointer
            and jump to the new execution address */
        ld      a0, 0(a1)
        bnez    a0, 2f
        nop
1:      wait
        b       1b
2:      nop     
        j       a0
        sd      $0, 0(a1) /* Overwrite the execution address so a core will hang if run again */
        .end   SecondaryCoreInit

        .globl simple_tlb_setup
        .ent   simple_tlb_setup
        .align 8
        /*
        ** a0  Virtual address
        ** a1  Physical address
        ** a2  start TLB index 
        ** a3  length to map 
        */

#define SIMPLE_TLB_PAGE_SIZE    (32*1024)
simple_tlb_setup:

        /* Format physical address for entry low */
        nop
        dsrl    a1, a1, 12
        dsll    a1, a1, 6
        ori     a1, a1, 0x7         // set DVG bits

        li       a4, SIMPLE_TLB_PAGE_SIZE
        dadd     a5, a4, a4 // mapping size
        dsll     a6, a4, 1
        daddi    a6, a6, -1 // pagemask
        dsrl     a4, a4, 6  // adjust for adding with entrylo


simple_tlb_setup_loop:
        mtc0     a6, COP0_PAGEMASK_REG
        mtc0    a2, COP0_INDEX_REG
        daddi    a2, a2, -1

        dmtc0   a1, COP0_ENTRYLO0_REG
        dadd     a1, a1, a4

        dmtc0   a1, COP0_ENTRYLO1_REG
        dadd     a1, a1, a4

        dmtc0   a0, COP0_ENTRYHI_REG
        dadd     a0, a0, a5

        ehb
        tlbwi
        dsub     a3, a3, a5
        bgtz     a3, simple_tlb_setup_loop
        nop

        jr  ra
        nop
        .end   simple_tlb_setup


        .globl single_tlb_setup
        .ent   single_tlb_setup
        .align 8
        /* Sets up a single TLB entry.  Virtual/physical addresses
        ** must be properly aligned.
        ** a0  Virtual address
        ** a1  Physical address
        ** a2  page (_not_ mapping) size
        */
single_tlb_setup:

        /* Determine the number of TLB entries available, and 
        ** use the top one. */
        mfc0    a3, COP0_CONFIG1_REG
        srl     a3, a3, 25
        mfc0    a5, COP0_CONFIG3_REG // Check if config4 reg present 
        bbit0   a5, 31, single_tlb_setup_cont
        and     a3, a3, 0x3F         // a3 now has the max mmu entry index
        mfc0    a5, COP0_CONFIG4_REG
        bbit0   a5, 14, single_tlb_setup_cont           // check config4[MMUExtDef]
        nop
        ins     a3, a5, 6, 8         // append config4[MMUSizeExt] to most significant bit of config1[MMUSize-1]
        and     a3, a3, 0x3fff       // a3 now includes max entries for cn6xxx 

single_tlb_setup_cont:

        /* Format physical address for entry low */
        nop
        dsrl    a1, a1, 12
        dsll    a1, a1, 6
        ori     a1, a1, 0x7         // set DVG bits

        move    a4, a2
        dadd    a5, a4, a4 // mapping size
        dsll    a6, a4, 1
        daddi   a6, a6, -1 // pagemask
        dsrl    a4, a4, 6  // adjust for adding with entrylo

        /* Now set up mapping */
        mtc0    a6, COP0_PAGEMASK_REG
        mtc0    a3, COP0_INDEX_REG

        dmtc0   a1, COP0_ENTRYLO0_REG
        dadd    a1, a1, a4

        dmtc0   a1, COP0_ENTRYLO1_REG
        dadd    a1, a1, a4

        dmtc0   a0, COP0_ENTRYHI_REG
        dadd    a0, a0, a5

        ehb
        tlbwi
        jr  ra
        nop
        .end   single_tlb_setup


/* This code is moved to a movable boot bus region,
** and it is responsible for changing the flash mappings and
** jumping to run from the TLB mapped address.
*/
change_boot_mappings:
        .align 8
        dli a4, 0x8001180000000000   /* region 0 config */
        sd  a0, 0(a4)
        sync
        j a1        /* Jump to new TLB mapped location */
        nop



/* Clear the u-boot TLB mapping, and eret to the start of
** the simple executive application.  The TLB index to be written
** is already set up, as is the error EPC value.
*/
        .globl tlbwi_and_eret
        .ent   tlbwi_and_eret
tlbwi_and_eret:

        tlbwi                        
        nop                        
        nop                        
        eret                     
        cache 0, 0($0)              /* Flush icache in delay slot*/

        .end tlbwi_and_eret
/*
 * Launch 64-bit Linux kernel entry point from a 32-bit U-boot
 * a0-a3 normal args, set up by C code.  We never come back,
 * so we keep this simple.
 * a4 is entry point
 * Calling C code sets up TLB to be ready for a write that clears the TLB
 * entry that u-boot uses.  This code is executed from XKPHYS address space
 * to allow the TLB entry to be removed.
 */
        .globl asm_launch_linux_entry_point
        .ent   asm_launch_linux_entry_point
asm_launch_linux_entry_point:
        tlbwi                        
        j       a4
        cache   0, 0($0)              /* Flush icache in delay slot*/
        .end   asm_launch_linux_entry_point



/* Basic exception handler (dump registers) in all ASM.  When using the TLB for
** mapping u-boot C code, we can't branch to that C code for exception handling
** (TLB is disabled for some exceptions. */
#define UART_PORT           0       /* Output to the first uart. This can be 0 or 1 */
#define UART_BASE(port)     0x8001180000000800+0x400*(port)
#define UART_LCR            0x18
#define UART_FCR            0x50
#define UART_DLL            0x80
#define UART_DLH            0x88
#define UART_LSR            0x28
#define UART_THR            0x40
#define UART_USR            0x138
#define CVMX_GPIO_TX_CLEAR  0x8001070000000890
#define CVMX_GPIO_TX_SET    0x8001070000000888
#define COP0_CVMCTL         $9,7
#define COP0_CVMMMECTL      $11,7
#define COP0_STATUS         $12,0
#define COP0_CAUSE          $13,0
#define COP0_EPC            $14,0
#define COP0_EBASE          $15,1

#
# void uart_write_char(char c)
#
    .ent uart_write_char
uart_write_char:
    dli     a4, UART_BASE(UART_PORT)
1:
    ld      a5, UART_LSR(a4)/* Read LSR so we can see if the FIFO has room */
    and     a5, 1<<5        /* Bit 5 signals that the TX FIFO has room */
    beqz    a5, 1b          /* Loop if there isn't any room */
     nop
    sd     a0, UART_THR(a4) /* write the char in the delay slot */
    j ra                    /* We're done, just return to the caller */
     nop
    .end uart_write_char

#
# void uart_write_string(const char *str)
#
    .ent uart_write_string
uart_write_string:
    dli     a4, UART_BASE(UART_PORT)
    move    a5, a0          /* Save a0 since C code expects that */
1:
    lbu     a6, 0(a5)       /* Read next byte of string */
    beqz    a6, 3f          /* Jump out if the byte is a zero */
     daddu   a5, 1          /* Increment out pointer to the next byte */
2:
    ld      a7, UART_LSR(a4)/* Read LSR so we can see if the FIFO has room */
    and     a7, 1<<5        /* Bit 5 signals that the TX FIFO has room */
    beqz    a7, 2b          /* Loop if there isn't any room */
     nop
    b       1b              /* Jump to process the next byte and */
     sd      a6, UART_THR(a4) /* write the current byte in the delay slot */
3:
    j ra                    /* We're done, just return to the caller */
     nop
    .end uart_write_string

#
# void uart_write_hex(uint64_t number)
#
    .ent uart_write_hex
uart_write_hex:
    move    t9, a0          /* Save a0 since C code expects that */
    move    t8, ra          /* Save ra */
    bal     uart_write_char /* Output '0' */
     li     a0, '0'
    bal     uart_write_char /* Output 'x' */
     li     a0, 'x'
    li      a7, 15          /* We need 16 digits total */
    move    a6, t9
    move    v0, $0          /* v0 being non zero signals we've passed the zero padding */
1:
    dext    a0, a6, 60, 4   /* Extract [63:60] for display in hex */
    or      v0, a0          /* OR in the current digit, when non zero print */
    beqz    v0, 2f
    addiu   a0, '0'         /* Add '0' to convert to '0'-'9' */
    slti    a4, a0, 0x3a    /* Flag if 0-9 instead of a-f */
    addiu   a5, a0, 0x41-0x30-10 /* Add 'a' */
    bal     uart_write_char /* Output char */
     movz   a0, a5, a4      /* Choose if 0-9 or a-f */
2:
    dsll    a6, 4           /* Shift to the next digit */
    bnez    a7, 1b          /* Loop while we still have digits */
     addu    a7, -1         /* One less digit to do */
    bnez    v0, 3f          /* We need to write a 0 if we didn't write any digits */
     nop
    bal     uart_write_char /* Output '0' */
     li     a0, '0'
3:
    move    ra, t8          /* Restore ra */
    j ra                    /* We're done, just return to the caller */
     move   a0, t9          /* Restore a0 */
    .end uart_write_hex




    .set noat
    .ent exception_handler_stage2
exception_handler_stage2:
    li      k0, -32768
    sd      $0, 0*8(k0)
    sd      $1, 1*8(k0)
    sd      $2, 2*8(k0)
    sd      $3, 3*8(k0)
    sd      $4, 4*8(k0)
    sd      $5, 5*8(k0)
    sd      $6, 6*8(k0)
    sd      $7, 7*8(k0)
    sd      $8, 8*8(k0)
    sd      $9, 9*8(k0)
    sd      $10, 10*8(k0)
    sd      $11, 11*8(k0)
    sd      $12, 12*8(k0)
    sd      $13, 13*8(k0)
    sd      $14, 14*8(k0)
    sd      $15, 15*8(k0)
    sd      $16, 16*8(k0)
    sd      $17, 17*8(k0)
    sd      $18, 18*8(k0)
    sd      $19, 19*8(k0)
    sd      $20, 20*8(k0)
    sd      $21, 21*8(k0)
    sd      $22, 22*8(k0)
    sd      $23, 23*8(k0)
    sd      $24, 24*8(k0)
    sd      $25, 25*8(k0)
    sd      $26, 26*8(k0)
    sd      $27, 27*8(k0)
    sd      $28, 28*8(k0)
    sd      $29, 29*8(k0)
    sd      $30, 30*8(k0)
    sd      $31, 31*8(k0)
    li      a1, 0
    li      a2, 31
1:
    jal     uart_write_char
    li      a0, 'R'
    jal     uart_write_char
    li      a0, 'e'
    jal     uart_write_char
    li      a0, 'g'
    jal     uart_write_char
    li      a0, ':'
    jal     uart_write_char
    li      a0, ' '
    jal     uart_write_hex
    move    a0, a1
    jal     uart_write_char
    li      a0, ' '
    jal     uart_write_hex
    ld      a0, 0(k0)
    jal     uart_write_char
    li      a0, '\r'
    jal     uart_write_char
    li      a0, '\n'
    addu    k0, 8
    bne     a1, a2, 1b
    addu    a1, 1
    dla     a0, MESSAGE_STATUS
    jal     uart_write_string
    nop
    jal     uart_write_hex
    dmfc0   a0, COP0_STATUS
    jal     uart_write_char
    li      a0, '\r'
    jal     uart_write_char
    li      a0, '\n'
    dla     a0, MESSAGE_CAUSE
    jal     uart_write_string
    nop
    jal     uart_write_hex
    dmfc0   a0, COP0_CAUSE
    jal     uart_write_char
    li      a0, '\r'
    jal     uart_write_char
    li      a0, '\n'
    dla     a0, MESSAGE_EPC
    jal     uart_write_string
    nop
    jal     uart_write_hex
    dmfc0  a0, COP0_EPC
    jal     uart_write_char
    li     a0, '\r'
    jal     uart_write_char
    li     a0, '\n'
    dla     a0, MESSAGE_BADVADDR
    jal     uart_write_string
    nop
    jal     uart_write_hex
    dmfc0  a0, COP0_BADVADDR_REG
    jal     uart_write_char
    li     a0, '\r'
    jal     uart_write_char
    li     a0, '\n'
2:  b       2b
     nop
MESSAGE_STATUS:     .string "status:   "
MESSAGE_CAUSE:      .string "cause:    "
MESSAGE_EPC:        .string "epc:      "
MESSAGE_BADVADDR:   .string "badvaddr: "
    .end exception_handler_stage2

    .ent exception_handler_stage1
exception_handler_stage1:
    j       exception_handler_stage2
     nop
    .end exception_handler_stage1
