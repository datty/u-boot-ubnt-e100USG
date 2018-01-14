/***********************license start************************************
 * Copyright (c) 2005-2007 Cavium Networks (support@cavium.com). All rights
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
#include <common.h>
#ifdef CONFIG_PCI
#include <pci.h>
#include <asm/addrspace.h>

#include "octeon_boot.h"
#include "octeon_hal.h"

int cvmx_pcie_rc_initialize(int pcie_port);
uint8_t cvmx_pcie_config_read8(int pcie_port, int bus, int dev, int fn, int reg);
uint16_t cvmx_pcie_config_read16(int pcie_port, int bus, int dev, int fn, int reg);
uint32_t cvmx_pcie_config_read32(int pcie_port, int bus, int dev, int fn, int reg);
void cvmx_pcie_config_write8(int pcie_port, int bus, int dev, int fn, int reg, uint8_t val);
void cvmx_pcie_config_write16(int pcie_port, int bus, int dev, int fn, int reg, uint16_t val);
void cvmx_pcie_config_write32(int pcie_port, int bus, int dev, int fn, int reg, uint32_t val);

/*
 *	Access Octeon PCIe Config / IACK / Special Space
 */

static int octeon_pcie_write_config_byte (struct pci_controller *hose, pci_dev_t dev,
					 int reg, u8 val)
{
    cvmx_pcie_config_write8(hose->pcie_port, PCI_BUS(dev) - hose->first_busno + 1, PCI_DEV(dev), PCI_FUNC(dev), reg, val );
    return 0;

}
static int octeon_pcie_write_config_word (struct pci_controller *hose, pci_dev_t dev,
					 int reg, u16 val)
{
    cvmx_pcie_config_write16(hose->pcie_port, PCI_BUS(dev) - hose->first_busno + 1, PCI_DEV(dev), PCI_FUNC(dev), reg, val );
    return 0;

}
static int octeon_pcie_write_config_dword (struct pci_controller *hose, pci_dev_t dev,
					 int reg, u32 val)
{
    cvmx_pcie_config_write32(hose->pcie_port, PCI_BUS(dev) - hose->first_busno + 1, PCI_DEV(dev), PCI_FUNC(dev), reg, val );
    return 0;

}
static int octeon_pcie_read_config_byte (struct pci_controller *hose, pci_dev_t dev,
					int reg, u8 * val)
{
	if ( ((PCI_BUS(dev) - hose->first_busno) == 0) && (PCI_DEV(dev) != 0) ) {
		*val = 0xff;
	} else {
        	*val = (u8) cvmx_pcie_config_read8(hose->pcie_port, PCI_BUS(dev) - hose->first_busno + 1, PCI_DEV(dev), PCI_FUNC(dev), reg );
	}
	return 0;
}
static int octeon_pcie_read_config_word (struct pci_controller *hose, pci_dev_t dev,
					int reg, u16 * val)
{	
	if ( ((PCI_BUS(dev) - hose->first_busno) == 0) && (PCI_DEV(dev) != 0) ) {
		*val = 0xffff;
	} else {
        	*val = (u16) cvmx_pcie_config_read16(hose->pcie_port, PCI_BUS(dev) - hose->first_busno + 1, PCI_DEV(dev), PCI_FUNC(dev), reg );
	}
        return 0;
}
static int octeon_pcie_read_config_dword (struct pci_controller *hose, pci_dev_t dev,
					int reg, u32 * val)
{
	if ( ((PCI_BUS(dev) - hose->first_busno) == 0) && (PCI_DEV(dev) != 0) ) {
		*val = 0xffffffff;
	} else {
        	*val = (u32) cvmx_pcie_config_read32(hose->pcie_port, PCI_BUS(dev) - hose->first_busno + 1, PCI_DEV(dev), PCI_FUNC(dev), reg );
	}
        return 0;
}

/*
 *	Initialize OCTEON PCIE
 */

/* Initialize IO and Memory base address for PCIE */
#define PCIE_PNP_IO0_BASE    0x00001000
#define PCIE_PNP_IO0_SIZE    0x000ff000
#define PCIE_PNP_IO1_BASE    0x00100000
#define PCIE_PNP_IO1_SIZE    0xffffffff
#define PCIE_PNP_MEM0_BASE   0xf0000000
#define PCIE_PNP_MEM0_SIZE   0x08000000
#define PCIE_PNP_MEM1_BASE   0xf8000000
#define PCIE_PNP_MEM1_SIZE   0x08000000

static struct pci_config_table pci_board_config_table[] = {
  /* vendor, device, class */
  /* bus, dev, func */
  { }
};

struct pci_controller hose_pcie[2];

/*
 *	Initialize Module
 */

#define SWAP_ADDR32_LE_TO_BE(x)         ((x)^ 0x4)   /* Only needed for NPI register access */
void init_octeon_pcie (void)
{
	int	rc = 0;
	struct pci_controller *hose;
        DECLARE_GLOBAL_DATA_PTR;

	/* Multi-Hose */
	int first_busno = OCTEON_FIRST_PCIE_BUSNO;
	int i;

        printf("Starting PCIE\n" );

	for(i = 0; i < 2; i++) {

		rc = cvmx_pcie_rc_initialize(i);

		if (rc != 0) 
			continue;

		hose = &hose_pcie[i];
		hose->config_table = pci_board_config_table;

		hose->first_busno = first_busno;
		hose->last_busno = 0xff;
	
		hose->pcie_port = i;

		if (i == 0) {
			/* PCI I/O space (Sub-DID == 2) */
			pci_set_region (hose->regions + 0, 0, 0, 0, 0,
				PCIE_PNP_IO0_BASE, PCIE_PNP_IO0_BASE, PCIE_PNP_IO0_SIZE, PCI_REGION_IO);
			/* PCI memory space (Sub-DID == 3) */
			pci_set_region (hose->regions + 1, 0, 0, 0, 0,
				PCIE_PNP_MEM0_BASE, PCIE_PNP_MEM0_BASE, PCIE_PNP_MEM0_SIZE, PCI_REGION_MEM);
		}
		else {
			/* PCI I/O space (Sub-DID == 2) */
			pci_set_region (hose->regions + 0, 0, 0, 0, 0,
				PCIE_PNP_IO1_BASE, PCIE_PNP_IO1_BASE, PCIE_PNP_IO1_SIZE, PCI_REGION_IO);
			/* PCI memory space (Sub-DID == 3) */
			pci_set_region (hose->regions + 1, 0, 0, 0, 0,
				PCIE_PNP_MEM1_BASE, PCIE_PNP_MEM1_BASE, PCIE_PNP_MEM1_SIZE, PCI_REGION_MEM);
		}

		hose->region_count = 2;

		hose->read_byte = octeon_pcie_read_config_byte;
		hose->read_word = octeon_pcie_read_config_word;
		hose->read_dword = octeon_pcie_read_config_dword;
		hose->write_byte = octeon_pcie_write_config_byte;
		hose->write_word = octeon_pcie_write_config_word;
		hose->write_dword = octeon_pcie_write_config_dword;

		pci_register_hose (hose);

		hose->last_busno = pci_hose_scan (hose);

		hose->first_busno += first_busno;	
		hose->last_busno  += first_busno;

		printf( "PCIe: port=%d, first_bus=%d, last_bus=%d\n", hose->pcie_port, hose->first_busno, hose->last_busno );

		first_busno = hose->last_busno + 1;


                /* Setup BAR1 to map bus address 0x0 to the base of u-boot's TLB mapping.  This allows
                ** us to have u-boot located anywhere in memory (including above 32 bit addressable space),
                ** and still have 32 bit PCI devices have access to memory that is statically allocated or malloced
                ** by u-boot, both of which are TLB mapped. */
                cvmx_write_csr(CVMX_PESCX_P2N_BAR1_START(i), 0);

                /* Disable bar0/bar2, as we are not using them here */
                cvmx_write_csr(CVMX_PESCX_P2N_BAR0_START(i), -1);
                cvmx_write_csr(CVMX_PESCX_P2N_BAR2_START(i), -1);


                /* Select 64 MByte mapping size for bar 1 on both ports */
                cvmx_npei_ctl_status2_t ctls2;
                ctls2.u64 = cvmx_read_csr(CVMX_NPEI_CTL_STATUS2);
                ctls2.s.c0_b1_s = 1;
                ctls2.s.c1_b1_s = 1;
                cvmx_write_csr(CVMX_NPEI_CTL_STATUS2, ctls2.u64);

                /* Configure the regions in bar 1 to map to the DRAM used
                ** by u-boot. */
                int base_entry = 16*i;
                int j;
                cvmx_npei_bar1_indexx_t npei_bar1_indexx;
                npei_bar1_indexx.u32 = 0;
                npei_bar1_indexx.s.addr_v = 1;
                npei_bar1_indexx.s.end_swp = 1;
                npei_bar1_indexx.s.ca = 1;
                /* Round down to 4MByte boundary to meet BAR mapping requirements */
                uint64_t bar_base = gd->bd->bi_uboot_ram_addr & ~0x3fffffull;

                for (j = 0; j < 16; j++)
                {
                    npei_bar1_indexx.s.addr_idx = (bar_base + 4*1024*1024*j) >> 22;

                    cvmx_write64_uint32(SWAP_ADDR32_LE_TO_BE(CVMX_PEXP_NPEI_BAR1_INDEXX(base_entry + j)), npei_bar1_indexx.u32);
                }
	}
}

int octeon_pci_port_num(uint32_t addr)
{
	return (addr >= PCIE_PNP_IO1_BASE);
} 


/* Do physical/virtual to bus mapping.  This mapping assumes that all 'physical'
** addresses come from u-boot data or heap. */
uint32_t octeon_pci_phys_to_bus(uint32_t mem_addr)
{
    DECLARE_GLOBAL_DATA_PTR;
    uint64_t phys_addr;

    uint64_t bar_base = gd->bd->bi_uboot_ram_addr & ~0x3fffffull;
    phys_addr = cvmx_ptr_to_phys((void *)mem_addr);

    /* We only have a limited amount of DRAM mapped with BAR1, so this translation
    ** is only valid for that range. */
    if (phys_addr < bar_base
        || phys_addr > gd->bd->bi_uboot_ram_addr + gd->bd->bi_uboot_ram_used_size)
    {
        printf("ERROR: Invalid address 0x%x (0x%qx physical) in octeon_pci_phys_to_bus\n", mem_addr, phys_addr);
        return(0xdeadbeef);
    }
    uint64_t bus_addr = phys_addr - bar_base;
#if 0
    printf("mem: 0x%x, phys: 0x%qx, ub-phys: 0x%qx, bus64: 0x%qx, bus32: 0x%x\n",
           mem_addr, phys_addr, gd->bd->bi_uboot_ram_addr, bus_addr, (uint32_t)bus_addr);
#endif


    return((uint32_t)bus_addr);
}

/* We expect all bus addresses to correspond to the TLB mapped u-boot
* data and heap area.  If it is outside of this region, it is an error. */
uint32_t octeon_pci_bus_to_phys(uint32_t bus_addr)
{
    DECLARE_GLOBAL_DATA_PTR;

    /* Extra offset to acount for the 4 MByte PCI BAR alignment restriciton - the u-boot
    ** base may not be aligned properly */
    uint32_t align_offset = (gd->bd->bi_uboot_ram_addr & 0x3fffff) ? 0x400000 - (gd->bd->bi_uboot_ram_addr & 0x3fffff) : 0;
    if (bus_addr > gd->bd->bi_uboot_ram_used_size + align_offset)
    {
        printf("ERROR: Invalid address 0x%x octeon_pci_bus_to_phys\n", bus_addr);
        return(0xdeadbeef);
    }

    uint64_t phys_addr = bus_addr + gd->bd->bi_uboot_ram_addr - align_offset;

    return((uint32_t)cvmx_phys_to_ptr(phys_addr));

}

#endif /* CONFIG_PCI */
