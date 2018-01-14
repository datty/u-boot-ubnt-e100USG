/*
 * (C) Copyright 2000-2003
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

/*
 * Misc boot support
 */
#include <common.h>
#include <command.h>
#include <net.h>
#include <octeon_boot.h>
#include <cvmx.h>
#include <asm/mipsregs.h>
#include <asm/processor.h>
extern void asm_launch_linux_entry_point;


/* -------------------------------------------------------------------- */

int do_go (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
#if defined(CONFIG_I386)
	DECLARE_GLOBAL_DATA_PTR;
#endif
	ulong	addr, rc;
	int     rcode = 0;

	if (argc < 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	addr = simple_strtoul(argv[1], NULL, 16);

	printf ("## Starting application at 0x%08lX ...\n", addr);
#ifdef CONFIG_OCTEON
        octeon_restore_std_mips_config();

        /* Set up the environment in RAM to pass off to next application
        ** or bootloader.  Required for NAND boot.
        */
        octeon_setup_ram_env();
#endif
	/*
	 * pass address parameter as argv[0] (aka command name),
	 * and all remaining args
	 */
#if defined(CONFIG_I386)
	/*
	 * x86 does not use a dedicated register to pass the pointer
	 * to the global_data
	 */
	argv[0] = (char *)gd;
#endif
#if !defined(CONFIG_NIOS)
#if !CONFIG_OCTEON_UBOOT_TLB
	rc = ((ulong (*)(int, char *[]))addr) (--argc, &argv[1]);
#else
        /* Here we need to do some special handling so that we can clear the TLB entry
        ** that is currently used to map u-boot code/data */
        uint64_t arg0 = --argc;
        uint64_t arg1 = (int32_t)&argv[1];
        uint64_t arg4 = (int32_t)MAKE_KSEG0(addr);
        uint64_t asm_addr = MAKE_XKPHYS(uboot_tlb_ptr_to_phys(&asm_launch_linux_entry_point));
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
               "       move  $8, %[arg4] \n"
               "       j     %[addr]  \n"
               "       nop                      \n"
               "       .set pop              \n"
                : :[arg0] "r" (arg0), [arg1] "r" (arg1), [arg4] "r" (arg4), [addr] "r" (asm_addr)
                : "$4","$5", "$8");
        /* Make sure that GCC doesn't use the explicit registers that we want to use  */
        rc = 0;  // Fake this for now
#endif
#else
	/*
	 * Nios function pointers are address >> 1
	 */
	rc = ((ulong (*)(int, char *[]))(addr>>1)) (--argc, &argv[1]);
#endif
	if (rc != 0) rcode = 1;

	printf ("## Application terminated, rc = 0x%lX\n", rc);
	return rcode;
}

/* -------------------------------------------------------------------- */

U_BOOT_CMD(
	go, CFG_MAXARGS, 1,	do_go,
	"go      - start application at address 'addr'\n",
	"addr [arg ...]\n    - start application at address 'addr'\n"
	"      passing 'arg' as arguments\n"
);

extern int do_reset (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

U_BOOT_CMD(
	reset, 1, 0,	do_reset,
	"reset   - Perform RESET of the CPU\n",
	NULL
);

#if CONFIG_OCTEON_NAC38
int do_power (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
   
   if (strncmp(argv[1],"off",3) == 0) 
   {
      cvmx_write_csr(CVMX_GPIO_TX_SET, 0x1000);		// Set GPIO 12 to 1
      cvmx_write_csr(CVMX_GPIO_BIT_CFGX(12), 0x01);	// Set GPIO 12 to output pin
      cvmx_write_csr(CVMX_GPIO_TX_SET, 0x1000);		// Set GPIO 12 to 1
   }
   else if (strncmp(argv[1],"down",4) == 0) 
   {
      cvmx_write_csr(CVMX_GPIO_TX_SET, 0x1000);		// Set GPIO 12 to 1
      cvmx_write_csr(CVMX_GPIO_BIT_CFGX(12), 0x01);	// Set GPIO 12 to output pin
      cvmx_write_csr(CVMX_GPIO_TX_SET, 0x1000);		// Set GPIO 12 to 1
   }
   else if (strncmp(argv[1],"loss_en",7) == 0) 
   {
      cvmx_write_csr(CVMX_GPIO_TX_SET, 0x40);		// Set GPIO 6 to 1
      cvmx_write_csr(CVMX_GPIO_BIT_CFGX(6), 0x01);	// Set GPIO 6 to output pin
      cvmx_write_csr(CVMX_GPIO_TX_SET, 0x40);		// Set GPIO 6 to 1
      udelay(200);
      cvmx_write_csr(CVMX_GPIO_TX_CLR, 0x40);		// Set GPIO 6 to 0
   }
   return 0;
}

U_BOOT_CMD(
	power, 2, 1,	do_power,
	"power   - Perform Power Off\n",
	"off     - Power off the System\n"
	"power down    - Power off the System\n"
	"power loss_en - Enable Power Loss function(power on after power loss)\n"
);
#endif

