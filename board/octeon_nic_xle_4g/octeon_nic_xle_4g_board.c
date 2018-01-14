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


#if defined(CONFIG_PCI)
extern void init_octeon_pci (void);

void pci_init_board (void)
{
}
#endif




int checkboard (void)
{

    octeon_gpio_cfg_output(0); /* GPIO 0 #run_led */
    octeon_gpio_cfg_output(5); /* GPIO 5 phy #reset */
    octeon_gpio_set(0);      /* off run_led */
    octeon_gpio_clr(5);      /* #reset = low */
    udelay(5000);
    udelay(5000);
    octeon_gpio_set(5);      /* #reset = high */
    udelay(5000);
    udelay(5000);

#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)
    /* Enable MDIO */
    cvmx_write_csr(CVMX_SMI_EN, 1);

    /* Configure the PHYS for the proper LED mode */
    {
        int i;
        for (i = 1; i <= 4; i++)
        {
            unsigned short val;
            val = miiphy_read_wrapper(i,3);
            if ((val & 0x000f) == 4)  /* revision C1 */
            { /* Revision C1 workaround */
              miiphy_write(i,22,0x00ff);
              miiphy_write(i,24,0x1111);
              miiphy_write(i,23,0x2012);
            }
            udelay(1000);
            miiphy_write(i,22,3);
            miiphy_write(i,16,0x1017); /* led0:speed, page3 16reg */
            miiphy_write(i,22,2);
            miiphy_write(i,26,3);     /* Tuning the Serdes setting to Reduce Power */
            miiphy_write(i,22,0); /* page 0 */
        }
    }
#endif
    return 0;
}



int early_board_init(void)
{
    int cpu_ref = DEFAULT_CPU_REF_FREQUENCY_MHZ;
    /* Populate global data from eeprom */
    uint8_t ee_buf[OCTEON_EEPROM_MAX_TUPLE_LENGTH];
    int addr;

    DECLARE_GLOBAL_DATA_PTR;

    memset((void *)&(gd->mac_desc), 0x0, sizeof(octeon_eeprom_mac_addr_t));
    memset((void *)&(gd->clock_desc), 0x0, sizeof(octeon_eeprom_clock_desc_t));
    memset((void *)&(gd->board_desc), 0x0, sizeof(octeon_eeprom_board_desc_t));

    /* NOTE: this is early in the init process, so the serial port is not yet configured */

    /* Determine board type/rev */
    strncpy((char *)(gd->board_desc.serial_str), "unknown", SERIAL_LEN);
    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_BOARD_DESC_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    if (addr >= 0)
    {
        memcpy((void *)&(gd->board_desc), ee_buf, sizeof(octeon_eeprom_board_desc_t));
    }
    else
    {
        gd->flags |= GD_FLG_BOARD_DESC_MISSING;
        gd->board_desc.rev_minor = 0;
        gd->board_desc.board_type = CVMX_BOARD_TYPE_NIC_XLE_4G;
        gd->board_desc.rev_major = 3;
    }

    gd->ddr_clock_mhz = 200;

    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_CLOCK_DESC_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    if (addr >= 0)
    {
        memcpy((void *)&(gd->clock_desc), ee_buf, sizeof(octeon_eeprom_clock_desc_t));
        gd->ddr_clock_mhz = gd->clock_desc.ddr_clock_mhz;
    }

    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_MAC_ADDR_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    if (addr >= 0)
    {
        memcpy((void *)&(gd->mac_desc), ee_buf, sizeof(octeon_eeprom_mac_addr_t));
    }
    else
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
    uint64_t data;
    if (OCTEON_IS_MODEL(OCTEON_CN56XX) || OCTEON_IS_MODEL(OCTEON_CN52XX))
        data = cvmx_read_csr(CVMX_PEXP_NPEI_DBG_DATA);
    else
        data = cvmx_read_csr(CVMX_DBG_DATA);
    data = data >> 18;
    data &= 0x1f;


    gd->cpu_clock_mhz = data * cpu_ref;

    return 0;

}
void octeon_led_str_write(const char *str)
{
}
