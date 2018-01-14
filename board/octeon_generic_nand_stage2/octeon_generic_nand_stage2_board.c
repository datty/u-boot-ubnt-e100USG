/*
 * (C) Copyright 2004-2007
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

#include <common.h>
#include <command.h>
#include <asm/au1x00.h>
#include <asm/mipsregs.h>
#include "octeon_boot.h"
#include <pci.h>
#include <miiphy.h>
#include <lib_octeon_shared.h>
#include <lib_octeon.h>


int checkboard (void)
{
    /* Enable MDIO interfaces */
    cvmx_write_csr(CVMX_SMIX_EN(0), 1);
    cvmx_write_csr(CVMX_SMIX_EN(1), 1);

    return 0;
}



int early_board_init(void)
{
    DECLARE_GLOBAL_DATA_PTR;

    memset((void *)&(gd->mac_desc), 0x0, sizeof(octeon_eeprom_mac_addr_t));
    memset((void *)&(gd->clock_desc), 0x0, sizeof(octeon_eeprom_clock_desc_t));
    memset((void *)&(gd->board_desc), 0x0, sizeof(octeon_eeprom_board_desc_t));

    /* NOTE: this is early in the init process, so the serial port is not yet configured */

    gd->board_desc.board_type = NAND_STAGE2_BOARD_TYPE;
    if (OCTEON_IS_MODEL(OCTEON_CN3XXX) || OCTEON_IS_MODEL(OCTEON_CN5XXX))
        gd->ddr_clock_mhz = DEFAULT_DDR2_CLOCK_FREQ_MHZ;
    else
        gd->ddr_clock_mhz = DEFAULT_DDR3_CLOCK_FREQ_MHZ;

    gd->ddr_ref_hertz = DEFAULT_DDR_REF_FREQUENCY_MHZ;

    /* CN52XX has a fixed internally generated 50 MHz reference clock */


    /* Make up some MAC addresses */
    gd->mac_desc.count = 255;
    gd->mac_desc.mac_addr_base[0] = 0x00;
    gd->mac_desc.mac_addr_base[1] = 0xDE;
    gd->mac_desc.mac_addr_base[2] = 0xAD;
    gd->mac_desc.mac_addr_base[3] = (gd->board_desc.rev_major<<4) | gd->board_desc.rev_minor;
    gd->mac_desc.mac_addr_base[4] = gd->board_desc.serial_str[0];
    gd->mac_desc.mac_addr_base[5] = 0x00;


    /* Read CPU clock multiplier */
    gd->cpu_clock_mhz = octeon_get_cpu_multiplier() * DEFAULT_CPU_REF_FREQUENCY_MHZ;


    /* Set this up here, as we want to use it during early NAND boot.
    ** Later this will be configured again to add the bootmem pointer */
    cvmx_sysinfo_minimal_initialize(0, gd->board_desc.board_type, 
                                    gd->board_desc.rev_major, gd->board_desc.rev_minor, 
                                    gd->cpu_clock_mhz * 1000000);
    return 0;

}
void octeon_led_str_write(const char *str)
{
}
