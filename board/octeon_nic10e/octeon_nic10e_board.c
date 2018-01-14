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
#include <cvmx-helper-jtag.h>


#if defined(CONFIG_PCI)
extern void init_octeon_pcie (void);

void pci_init_board (void)
{
	init_octeon_pcie();
}
#endif

#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)
/* Configure the PHYS for the proper LED mode */
void marvellphy_led_mode(void)
{
    int i;
    unsigned short phy_identifier;

    miiphy_read(0x81, 2, &phy_identifier);

    if (phy_identifier != 0x0141) /* Is it a Marvell PHY? */
        return;

    for (i = 0x81; i <= 0x84; i++)
    {
        miiphy_write(i, 0x16, 3);   /* set register page */
        /* Write LED modes */
        miiphy_write(i, 0x10, 0x5777);
        miiphy_write(i, 0x13, 0x0070);
        miiphy_write(i, 0x16, 0);   /* set register page */
    }
}
#endif

int checkboard (void)
{
    DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)
    {

        /* Enable MDIO */
        cvmx_write_csr(CVMX_SMIX_EN(0), 1);
        cvmx_write_csr(CVMX_SMIX_EN(1), 1);
        marvellphy_led_mode();
    }
#endif


    /* Enable USB supply */
    octeon_gpio_cfg_output(6);
    octeon_gpio_set(6);

    return 0;
}



int early_board_init(void)
{

    DECLARE_GLOBAL_DATA_PTR;
    {
        /* configure clk_out pin */
        cvmx_mio_fus_pll_t fus_pll;

        fus_pll.u64 = cvmx_read_csr(CVMX_MIO_FUS_PLL);
        fus_pll.cn63xx.c_cout_rst = 1;
        cvmx_write_csr(CVMX_MIO_FUS_PLL, fus_pll.u64);

        /* Sel::  0:rclk, 1:pllout 2:psout 3:gnd */
        fus_pll.cn63xx.c_cout_sel = 0;
        cvmx_write_csr(CVMX_MIO_FUS_PLL, fus_pll.u64);
        fus_pll.cn63xx.c_cout_rst = 0;
        cvmx_write_csr(CVMX_MIO_FUS_PLL, fus_pll.u64);
    }

    memset((void *)&(gd->mac_desc), 0x0, sizeof(octeon_eeprom_mac_addr_t));
    memset((void *)&(gd->clock_desc), 0x0, sizeof(octeon_eeprom_clock_desc_t));
    memset((void *)&(gd->board_desc), 0x0, sizeof(octeon_eeprom_board_desc_t));

    /* NOTE: this is early in the init process, so the serial port is not yet configured */

    /* Populate global data from eeprom */
    uint8_t ee_buf[OCTEON_EEPROM_MAX_TUPLE_LENGTH];
    int addr;

    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_DDR_CLOCK_DESC_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    if (addr >= 0)
    {
        octeon_eeprom_ddr_clock_desc_t *ddr_clock_ptr = (void *)ee_buf;
        gd->ddr_clock_mhz = ddr_clock_ptr->ddr_clock_hz/1000000;
    }
    else
    {
        gd->ddr_clock_mhz = NIC10E_DEF_DRAM_FREQ;
        gd->flags |= GD_FLG_CLOCK_DESC_MISSING;
    }

    /* CN63XX has a fixed 50 MHz reference clock */
    gd->ddr_ref_hertz = 50000000;

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
        gd->board_desc.board_type = CVMX_BOARD_TYPE_NIC10E;
        /* Try to determine board rev by looking at PAL */
        gd->board_desc.rev_major = 1;
    }


    gd->ddr_clock_mhz = NIC10E_DEF_DRAM_FREQ;

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
    gd->cpu_clock_mhz = octeon_get_cpu_multiplier() * 50;

    return 0;

}
void octeon_led_str_write(const char *str)
{
   
}
