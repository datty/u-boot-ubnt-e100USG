/***********************license start************************************
 * Copyright (c) 2009 Cavium Networks (support@cavium.com). All rights
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
#include <command.h>
#include <asm/au1x00.h>
#include <asm/mipsregs.h>
#include "octeon_boot.h"
#include <pci.h>
#include <miiphy.h>
#include <lib_octeon_shared.h>
#include <lib_octeon.h>
#include <cvmx-mdio.h>


#if defined(CONFIG_PCI)
extern void init_octeon_pci (void);

void pci_init_board (void)
{
}
#endif




int checkboard (void)
{
    DECLARE_GLOBAL_DATA_PTR;

    octeon_gpio_cfg_output(0); /* GPIO 0 #run_led */
    octeon_gpio_cfg_output(5); /* GPIO 5 phy #reset */
    octeon_gpio_set(0);      /* off run_led */
    octeon_gpio_clr(5);      /* #reset = low */
    udelay(5000);
    udelay(5000);
    octeon_gpio_set(5);      /* #reset = high */
    udelay(5000);

#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)
    /* Enable MDIO */
    cvmx_write_csr(CVMX_SMI_EN, 1);
    udelay(5000);

    {
      uint64_t clk_mod;
      int val;

      octeon_gpio_cfg_output(15); /* GPIO 15 LP_XGMX */
      octeon_gpio_clr(15);        /* LP_XGMX = low */
      cvmx_write_csr(CVMX_PCSXX_MISC_CTL_REG(0), 0xc); /* PCSX0 lane swap */

      val = (310 * gd->cpu_clock_mhz / 1000) & 0xff; /* Minimum clock period */
      clk_mod = 0x1001000;   /* set Clause 45 mode */
      clk_mod = clk_mod | val | (val&0xf)<<8 | 1<<15 | (val>>4)<<16 | 1<<24;
      cvmx_write_csr(CVMX_SMIX_CLK(0), clk_mod);

      val = cvmx_mdio_45_read(0, 0, 1, 0xe800);
      if (val == 0x8486) {
          val = cvmx_mdio_45_read(0, 0, 1, 0xe607);
          printf("Ethernet 0 SFP+ module is %s\n", (val&0x4000) ? "not present" : "present"); /* Check module present */
          /* TXDOUT invert polarity */
          val = cvmx_mdio_45_read(0, 0, 1, 0x8000);
          val = val & ~(1 << 7);
          cvmx_mdio_45_write(0, 0, 1, 0x8000, val);
          /* DEV_GPIO_CTRL tx/rx led, wis_intb */
          cvmx_mdio_45_write(0, 0, 1, 0xe901, 0x283a);
      }
      else
          printf("Phy0 does not exist!!! \n");

      val = cvmx_mdio_45_read(0, 1, 1, 0xe800);
      if (val == 0x8486) {
          val = cvmx_mdio_45_read(0, 1, 1, 0xe607);
          printf("Ethernet 1 SFP+ module is %s\n", (val&0x4000) ? "not present" : "present");
          /* TXDOUT invert polarity */
          val = cvmx_mdio_45_read(0, 1, 1, 0x8000);
          val = val & ~(1 << 7);
          cvmx_mdio_45_write(0, 1, 1, 0x8000, val);
          /* DEV_GPIO_CTRL tx/rx led, wis_intb */
          cvmx_mdio_45_write(0, 1, 1, 0xe901, 0x283a);
      }
      else
          printf("Phy1 does not exist!!! \n");
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
        gd->board_desc.board_type = CVMX_BOARD_TYPE_NIC_XLE_10G;
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



    /* FIXME. This seems to be needed for XAUI to work - not sure why..... */
    { /* Write Mac address for host use */
      uint8_t *ptr = (uint8_t *)gd->mac_desc.mac_addr_base;
      uint64_t mac = 0;
      int i;
      for (i=0; i<6; i++)
         mac = (mac<<8) | (uint64_t)(ptr[i]);
      cvmx_write_csr(CVMX_GMXX_SMACX(0,0), mac);
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
