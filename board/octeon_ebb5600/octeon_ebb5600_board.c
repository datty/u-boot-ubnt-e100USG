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
        uchar qlm_switch_addr;

        /* Enable MDIO */
        cvmx_write_csr(CVMX_SMIX_EN(1), 1);

        qlm_switch_addr = 0x3;      /* Q L M 1 */
        i2c_write(0x71, 0, 1, &qlm_switch_addr, 1);
        udelay(11000);              /* Let the write complete */

        marvellphy_led_mode();

        qlm_switch_addr = 0xC;      /* Q L M 3 */
        i2c_write(0x71, 0, 1, &qlm_switch_addr, 1);
        udelay(11000);              /* Let the write complete */

        marvellphy_led_mode();
    }
#endif
    if (octeon_show_info())
    {

        int pal_rev_maj = 0;
        int pal_rev_min = 0;
        int mcu_rev_maj = 0;
        int mcu_rev_min = 0;

        if (twsii_mcu_read(0x00)==0xa5 && twsii_mcu_read(0x01)==0x5a)
        {
            gd->mcu_rev_maj = mcu_rev_maj = twsii_mcu_read(2);
            gd->mcu_rev_min = mcu_rev_min = twsii_mcu_read(3);
        }

        printf("PAL rev: %d.%02d, MCU rev: %d.%02d, CPU voltage: ?.???\n",
               pal_rev_maj, pal_rev_min, mcu_rev_maj, mcu_rev_min);
    }

    if (octeon_show_info())
    {
        char tmp[10];
        sprintf(tmp, "%d ?.?? ", gd->cpu_clock_mhz);
        octeon_led_str_write(tmp);
    }
    else
    {
        octeon_led_str_write("Boot    ");
    }

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

    /* Populate global data from eeprom */
    uint8_t ee_buf[OCTEON_EEPROM_MAX_TUPLE_LENGTH];
    int addr;

    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_CLOCK_DESC_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    if (addr >= 0)
    {
        memcpy((void *)&(gd->clock_desc), ee_buf, sizeof(octeon_eeprom_clock_desc_t));
        gd->ddr_clock_mhz = gd->clock_desc.ddr_clock_mhz;
    }
    else
    {
        gd->ddr_clock_mhz = EBB5600_DEF_DRAM_FREQ;
    }

    /* CN56XX has a fixed internally generated 50 MHz reference clock */
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
        gd->board_desc.board_type = CVMX_BOARD_TYPE_EBB5600;
        /* Try to determine board rev by looking at PAL */
        gd->board_desc.rev_major = 1;
    }

    /* Even though the CPU ref freq is stored in the clock descriptor, we don't read it here.  The MCU
    ** reads it and configures the clock, and we read how the clock is actually configured.
    ** The bootloader does not need to read the clock descriptor tuple for normal operation on
    ** rev 2 and later boards
    */
    cpu_ref = octeon_mcu_read_cpu_ref();


    /* Some sanity checks */
    if (cpu_ref <= 0)
    {
        /* Default if cpu reference clock reading fails. */
        cpu_ref = DEFAULT_CPU_REF_FREQUENCY_MHZ;
    }
    if (gd->ddr_clock_mhz < 100 || gd->ddr_clock_mhz > 2000)
    {
        gd->ddr_clock_mhz = EBB5600_DEF_DRAM_FREQ;
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
    gd->cpu_clock_mhz = octeon_get_cpu_multiplier() * cpu_ref;

    octeon_led_str_write("Booting.");
    return 0;

}
void octeon_led_str_write(const char *str)
{
    octeon_led_str_write_std(str);
}
