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
#include <lib_octeon_shared.h>
#include <lib_octeon.h>


#if defined(CONFIG_PCI)
extern void init_octeon_pci (void);

void pci_init_board (void)
{
}
#endif




int checkboard (void)
{

#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)
    /* Enable MDIO */
    cvmx_write_csr(CVMX_SMI_EN, 1);
#endif

    return 0;
}



int early_board_init(void)
{
    int cpu_ref = DEFAULT_CPU_REF_FREQUENCY_MHZ;

    DECLARE_GLOBAL_DATA_PTR;

    memset((void *)&(gd->mac_desc), 0x0, sizeof(octeon_eeprom_mac_addr_t));
    memset((void *)&(gd->clock_desc), 0x0, sizeof(octeon_eeprom_clock_desc_t));
    memset((void *)&(gd->board_desc), 0x0, sizeof(octeon_eeprom_board_desc_t));

    /* NOTE: this is early in the init process, so the serial port is not yet configured */

    gd->board_desc.board_type = GENERIC_BOARD_TYPE;

    /* Determine board type/rev */
    strncpy((char *)(gd->board_desc.serial_str), "unknown", SERIAL_LEN);
    cpu_ref = DEFAULT_CPU_REF_FREQUENCY_MHZ;
    gd->ddr_clock_mhz = 0;

    {
        /* Make up some MAC addresses */
        gd->mac_desc.count = 255;
        gd->mac_desc.mac_addr_base[0] = 0x00;
        gd->mac_desc.mac_addr_base[1] = 0xDE;
        gd->mac_desc.mac_addr_base[2] = 0xAD;
        gd->mac_desc.mac_addr_base[3] = (gd->board_desc.rev_major<<4) | gd->board_desc.rev_minor;
        gd->mac_desc.mac_addr_base[4] = gd->board_desc.serial_str[0];
        gd->mac_desc.mac_addr_base[5] = 0x00;
    }



    /* Read CPU clock multiplier */
    gd->cpu_clock_mhz = octeon_get_cpu_multiplier() * cpu_ref;

    return 0;

}
void octeon_led_str_write(const char *str)
{
}
