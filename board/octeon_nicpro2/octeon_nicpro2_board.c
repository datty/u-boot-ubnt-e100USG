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
#include <cvmx.h>
#include <lib_octeon_shared.h>
#include <lib_octeon.h>
#include <cvmx-twsi.h>


#if defined(CONFIG_PCI)
extern void init_octeon_pci (void);

void pci_init_board (void)
{
	init_octeon_pci();
}
#endif

/* Boot bus init for flash and peripheral access */
#define FLASH_RoundUP(_Dividend, _Divisor) (((_Dividend)+(_Divisor))/(_Divisor))
int octeon_boot_bus_init_board(void)
{
    cvmx_mio_boot_reg_timx_t __attribute__((unused)) reg_tim;
    uint64_t ECLK_MHZ, ECLK_PERIOD;

    /* Read CPU clock multiplier */
    uint64_t data = cvmx_read_csr(CVMX_DBG_DATA);
    data = data >> 18;
    data &= 0x1f;
    data *= 50; /* cpu_ref = 50 default */
    ECLK_MHZ = data;
    ECLK_PERIOD = 1000000 / ECLK_MHZ; /* eclk period (psecs) */

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

    int M_divider, N_divider;
    for (N_divider = 0; N_divider < 8; ++N_divider) {
        M_divider = ((ECLK_MHZ * 1000000) / (20 * TWSI_BUS_FREQ * (TWSI_THP+1) * (1<<N_divider))) - 1;
        if (M_divider < 16) break;
    }

    cvmx_mio_tws_sw_twsi_t sw_twsi;
    sw_twsi.u64 = 0;
    sw_twsi.s.v = 1;
    sw_twsi.s.op = 0x6;
    sw_twsi.s.eop_ia = 0x3;
    sw_twsi.s.d = ((M_divider&0xf) << 3) | ((N_divider&0x7) << 0);
    cvmx_write_csr(CVMX_MIO_TWS_SW_TWSI, sw_twsi.u64);


#if CONFIG_RAM_RESIDENT
    /* Install the 2nd moveable region to the debug exception main entry
        point. This way the debugger will work even if there isn't any
        flash. */
    extern void debugHandler_entrypoint(void);
    const uint64_t *handler_code = (const uint64_t *)debugHandler_entrypoint;
    int count;
    cvmx_write_csr(CVMX_MIO_BOOT_LOC_CFGX(1), (1<<31) | (0x1fc00480>>7<<3));
    cvmx_write_csr(CVMX_MIO_BOOT_LOC_ADR, 0x80);
    for (count=0; count<128/8; count++)
        cvmx_write_csr(CVMX_MIO_BOOT_LOC_DAT, *handler_code++);
#endif

    return(0);
}



int checkboard (void)
{
    DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)
    /* Enable MDIO */
    cvmx_write_csr(CVMX_SMI_EN, 1);
#endif

    if (gd->board_desc.rev_major >= 2)
    {
        /* Disable access to the DIMM TWSI eeproms so that secondary tswi address of the SFP modules
        ** does not conflict */
        octeon_gpio_cfg_output(5);
        octeon_gpio_clr(5);


        /* Enable SFP output using twsi GPIO chip */

        /* Set bits 3, 7, 11, 15 to output.  Registers 2/3 control
        ** the direction of the 16 bits.  1 -> input, 0 -> output.
        ** All the other signals are inputs, so this is a boot time configuration
        ** that only needs to be done once. */
        cvmx_twsix_write_ia(0, 0x27, 2, 1, 1, 0x77);
        cvmx_twsix_write_ia(0, 0x27, 3, 1, 1, 0x77);

        /* Set the tx disable signal low for the 4 SFP modules.  This
        ** Enables transmit for the 4 SFP modules. */
        cvmx_twsix_write_ia(0, 0x27, 6, 1, 1, 0x77);
        cvmx_twsix_write_ia(0, 0x27, 7, 1, 1, 0x77);
        /* Reads from addresses 0/1 provide the current
        ** state of the input pins */

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
    }


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
        gd->board_desc.board_type = CVMX_BOARD_TYPE_NICPRO2;
        /* Try to determine board rev by looking at PAL */
        gd->board_desc.rev_major = 1;
    }

    cpu_ref = DEFAULT_CPU_REF_FREQUENCY_MHZ;
    gd->ddr_clock_mhz = NICPRO2_REV1_DEF_DRAM_FREQ;


    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_CLOCK_DESC_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    if (addr >= 0)
    {
        memcpy((void *)&(gd->clock_desc), ee_buf, sizeof(octeon_eeprom_clock_desc_t));
        gd->ddr_clock_mhz = gd->clock_desc.ddr_clock_mhz;
    }

    /*
    ** For the cn58xx the DDR reference clock frequency is used to
    ** configure the appropriate internal DDR_CK/DDR2_REF_CLK ratio in
    ** order to generate the ddr clock frequency specified by
    ** ddr_clock_mhz.  Not used for CN38XX.
    */
#define FIXED_DDR_REF_CLOCK_HERTZ	133333333;
    gd->ddr_ref_hertz = FIXED_DDR_REF_CLOCK_HERTZ;

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
    uint64_t data = cvmx_read_csr(CVMX_DBG_DATA);
    data = data >> 18;
    data &= 0x1f;


    gd->cpu_clock_mhz = data * cpu_ref;

    /* adjust for 33.33 Mhz clock */
    if (cpu_ref == 33)
        gd->cpu_clock_mhz += (data)/4 + data/8;

    if (gd->cpu_clock_mhz < 100 || gd->cpu_clock_mhz > 1100)
    {
        gd->cpu_clock_mhz = DEFAULT_ECLK_FREQ_MHZ;
    }

    octeon_led_str_write("Booting.");
    return 0;

}
void octeon_led_str_write(const char *str)
{
//    octeon_led_str_write_std(str);
}
