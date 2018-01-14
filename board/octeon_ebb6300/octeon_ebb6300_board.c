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

char *isp_voltage_labels[] = {
    NULL,        /* Chan 0  */
    "DCOK_QLM",  /* Chan 1  */
    NULL,        /* Chan 2  */
    "1.2V",      /* Chan 3  */
    NULL,        /* Chan 4  */
    "2.5V",      /* Chan 5  */
    "DDR1 1.5V", /* Chan 6  */
    "DDR0 1.5V", /* Chan 7  */
    "Core VDD",  /* Chan 8  */
    "3.3V",      /* Chan 9  */
    "5.0V",      /* Chan 10 */
    "12V / 3",   /* Chan 11 */
    "VCCA",      /* VCCA    */
    "VCCINP",    /* VCCINP  */
    0
};

extern void print_isp_volt(uint16_t isp_dev_addr, uint8_t adc_chan);
extern int read_ispPAC_mvolts(uint16_t isp_dev_addr, uint8_t adc_chan);

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

/* Raise an integer to a power */
static uint64_t ipow(uint64_t base, uint64_t exp)
{
    uint64_t result = 1;
    while (exp)     {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }
    return result;
} 

int checkboard (void)
{
    DECLARE_GLOBAL_DATA_PTR;

    int mVolts = 0;

#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)
    {

        /* Enable MDIO */
        cvmx_write_csr(CVMX_SMIX_EN(0), 1);
        cvmx_write_csr(CVMX_SMIX_EN(1), 1);
        marvellphy_led_mode();
    }
#endif
    if (octeon_show_info())
    {

#define AVERAGE_ITERATIONS 10


        int mcu_rev_maj = 0;
        int mcu_rev_min = 0;
        int iterations;
        int mVoltsMin, mVoltsMax, mVoltsAvg;

        if (twsii_mcu_read(0x00)==0xa5 && twsii_mcu_read(0x01)==0x5a)
        {
            gd->mcu_rev_maj = mcu_rev_maj = twsii_mcu_read(2);
            gd->mcu_rev_min = mcu_rev_min = twsii_mcu_read(3);
        }

#if 0
        {
            int x;

            /* Enable this to display the voltages on all the channels. */
            printf("System Power Status:\n");
            for (x=0; x<14 ; x++){
                print_isp_volt(BOARD_ISP_TWSI_ADDR, x);
            }
        }
#endif

        mVoltsMin = 99999;  /* Init minimum check */
        mVoltsMax = 0;      /* Init maximum check */
        mVoltsAvg = 0;
        iterations = AVERAGE_ITERATIONS + 2; /* Will throw out the min and max */
        while (iterations--)
        {
            mVolts = read_ispPAC_mvolts(BOARD_ISP_TWSI_ADDR, 8); /* Read Core VDD */
            mVoltsMin = min(mVoltsMin, mVolts);
            mVoltsMax = max(mVoltsMax, mVolts);
            mVoltsAvg += mVolts;
            //printf("Read Core VDD %d mV\n", mVolts);
        }
        mVoltsAvg = mVoltsAvg - mVoltsMin - mVoltsMax; /* Throw out the min and max */
        mVolts = divide_nint(mVoltsAvg, AVERAGE_ITERATIONS);

        printf("MCU rev: %d.%02d, CPU voltage: %d.%03d\n",
               gd->mcu_rev_maj, gd->mcu_rev_min, mVolts/1000, mVolts%1000);

    }

    if (octeon_show_info())
    {
#define LED_CHARACTERS 8
        char tmp[10];
        int characters, idx = 0, value = mVolts;

        idx = sprintf(tmp, "%d ", gd->cpu_clock_mhz);
        characters = LED_CHARACTERS - idx;

        if (value/1000) {
            idx += sprintf(tmp+idx, "%d", value/1000);
            characters = LED_CHARACTERS - idx;
        }

        characters -= 1;        /* Account for decimal point */

        value %= 1000;
        value = divide_nint(value, ipow(10, max(3-characters, 0)));

        idx += sprintf(tmp+idx, ".%0*d", min(3, characters), value);

        /* Display CPU speed and voltage on display */
        //printf("tmp: \"%s\"\n", tmp);
        octeon_led_str_write(tmp);
    }
    else
    {
        octeon_led_str_write("Boot    ");
    }

#if 0  /* DEBUG use only */
    {

        char *eptr = getenv("jtag_ic50");
        if (eptr)
        {
            /* Do experimental JTAG changes for pass 1.1 */
            int qlm_num;

            int ic50_val = simple_strtol(eptr, NULL, 10);
            int ir50_val = ic50_val;
            eptr = getenv("jtag_ir50");

            if (eptr)
                ir50_val = simple_strtol(eptr, NULL, 10);
            else
                printf("Warning: jtag_ir50 val not set, using ic50 value\n");

            printf("Setting QLM DAC ir50=0x%x, ic50=0x%x via JTAG chain\n", ir50_val, ic50_val);

            /* New values are xored with default values, which are 0x11 */
            ic50_val ^= 0x11;
            ir50_val ^= 0x11;

            cvmx_helper_qlm_jtag_init();
            for (qlm_num = 0; qlm_num < 3; qlm_num++)
            {
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 34);
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ir50_val); /* ir50dac */
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ic50_val); /* ic50dac */
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 300-44);
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 34);
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ir50_val); /* ir50dac */
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ic50_val); /* ic50dac */
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 300-44);
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 34);
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ir50_val); /* ir50dac */
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ic50_val); /* ic50dac */
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 300-44);
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 34);
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ir50_val); /* ir50dac */
                cvmx_helper_qlm_jtag_shift(qlm_num, 5, ic50_val); /* ic50dac */
                cvmx_helper_qlm_jtag_shift_zeros(qlm_num, 300-44);
                cvmx_helper_qlm_jtag_update(qlm_num);
            }
        }
    }
#endif

    return 0;
}



int early_board_init(void)
{
    int cpu_ref = DEFAULT_CPU_REF_FREQUENCY_MHZ;

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
        gd->ddr_clock_mhz = EBB6300_DEF_DRAM_FREQ;
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
        gd->board_desc.board_type = CVMX_BOARD_TYPE_EBB6300;
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
        gd->ddr_clock_mhz = EBB6300_DEF_DRAM_FREQ;
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
