/*
 * (C) Copyright 2004,2005
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





#define VIA_IDE_NATIVE_MODE     1
#define VIA_PMIO_BASE_ADDR      0x480L


void via_ide_init(void)
{
   pci_dev_t dev;
   uint8_t value;
   uint32_t via_ide_port[3];

   dev=pci_find_device(0x1106, 0x0571, 0);	// find Via IDE
   if ( dev >= 0 )
   {
      //printf("IDE ");
      pci_write_config_word(dev, 0x04, 0x0087);	// Enable Bus Master I/O space,Memory space
      pci_write_config_byte(dev, 0x0d, 0x40);	// Latency Timer
      pci_read_config_byte(dev, 0x40, &value);
      pci_write_config_byte(dev, 0x40, value|0x02);// Enable Primary Channel
      pci_write_config_byte(dev, 0x41, 0xC2);	// Primary IDE Read Prefetch Buffer
      pci_write_config_byte(dev, 0x43, 0x05);	// FIFO configuration 1/2

      pci_write_config_byte(dev, 0x3c, 0x0e); 	// Interrupt Line
      pci_write_config_byte(dev, 0x44, 0x1c); 	// Miscellaneous Control
      pci_write_config_byte(dev, 0x45, 0x00); 	// Miscellaneous Control

      pci_write_config_byte(dev, 0x46, 0xc0); 	// Enable DMA FIFO flush  
      pci_write_config_dword(dev, 0x48, 0x2022a8a8);	// Drive Timing Control, PIO Mode 4(0x20)
      pci_write_config_byte(dev, 0x4c, 0xff);		// Address Setup Time
      pci_write_config_byte(dev, 0x54, 0x04);		// UltraDMA FIFO control


      pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &via_ide_port[0]); 
      pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &via_ide_port[1]); 
      pci_read_config_dword(dev, PCI_BASE_ADDRESS_1, &via_ide_port[2]);

      /* Mask of non-address bits at bottom of BAR registers */
      via_ide_port[0] = via_ide_port[0] & 0xfffffff8;
      via_ide_port[1] = via_ide_port[1] & 0xfffffff8;
      via_ide_port[2] = via_ide_port[2] & 0xfffffff8;
      int dev;
      dev = ide_register_device(IF_TYPE_IDE, BLOCK_QUIRK_IDE_VIA, 0, OCTEON_PCI_IOSPACE_BASE + via_ide_port[0], OCTEON_PCI_IOSPACE_BASE + via_ide_port[1], OCTEON_PCI_IOSPACE_BASE + via_ide_port[2], 0);
      dev = ide_register_device(IF_TYPE_IDE, BLOCK_QUIRK_IDE_VIA, 1, OCTEON_PCI_IOSPACE_BASE + via_ide_port[0], OCTEON_PCI_IOSPACE_BASE + via_ide_port[1], OCTEON_PCI_IOSPACE_BASE + via_ide_port[2], 0);

      //printf("(ideport 0x%x-0x%x, 0x%x, ", via_ide_port[0], via_ide_port[0]+0x08, via_ide_port[2]+6);
      
   }

}










extern void init_octeon_pci (void);

void pci_init_board (void)
{
	init_octeon_pci();
//        via_ide_init();
}
#endif

int late_board_init(void)
{
    return 0;
}


int nac38_twsi_write8(int dev_addr, int ia, uint8_t val)
{
    return cvmx_twsix_write_ia(0, dev_addr, ia, 1, 1, val);
}

int nac38_twsi_read(int dev_addr, int ia)
{
    return cvmx_twsix_read_ia8(0, dev_addr, ia, 1);

}

void W83792d_ASR(int asr_type, int asr_timer)
{
    unsigned char  temp;

    if ( asr_type == 0 )		// Disable ASR
    {
	temp = 0xcc;	
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x01, temp);

	temp = 0x00;		// Disable 4 Minutes Watchdog
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x04, temp);

	temp = nac38_twsi_read(CFG_I2C_SYSMON_W83792D_ADDR, 0x40);
	temp = temp & (~0x10); 
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x40, temp);
    }
    else if ( asr_type == 1 ) 	// Enable ASR
    {
   	// Enable Watchdog Function
	temp = nac38_twsi_read(CFG_I2C_SYSMON_W83792D_ADDR, 0x40);
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x40, temp | 0x10);

   	// read to clear stus
	temp = nac38_twsi_read(CFG_I2C_SYSMON_W83792D_ADDR, 0x03);

	temp = 0x00;	// Disable 4 Minutes 	
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x04, temp);

	temp = asr_timer;	// set ASR Timer
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x04, temp);

	temp = 0x33;	// Enable Hard watchdog
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x01, temp);
    }
    else			// Trigger ASR
    {
	temp = asr_timer;	// set ASR Timer
	nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x04, temp);
    }
}

void init_w83792d(void)
{
    unsigned char temp3_cfg;

    temp3_cfg = nac38_twsi_read(CFG_I2C_SYSMON_W83792D_ADDR, 0x4b);
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x4b, temp3_cfg | 0x04);	// enable fan 7

    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x59, 0x10);	// set tempearture1 to therml diode, 2 and 3 to thermistor 

    temp3_cfg = nac38_twsi_read(CFG_I2C_SYSMON_W83792D_ADDR, 0x5d);		
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x5d, temp3_cfg | 0x01);	// enable vbat

    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x47, 0x99);	// enable fan 1, 2
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x5b, 0x99);	// enable fan 3, 4
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x5c, 0x19);	// enable fan 5
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x9e, 0x09);	// enable fan 7

    // Disable ASR
    W83792d_ASR(0, 0);

    // Smart Fan Initial
    char *s;
    if ((s = getenv("CPU_Temp")) != NULL) {
         unsigned smart_fan_target;
         smart_fan_target = (unsigned) simple_strtoul(s, NULL, 0);
       if ( smart_fan_target > 65 ) {
          printf(" Overriding CPU_Temp  %2d  \n", smart_fan_target);
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x85, 0x41);	// TTarget_MB   65 C
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x86, 0x41);	// TTarget_CPU1  65 C
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x96, 0x41);	// TTarget_CPU2   65 C
       }
       else {
          printf(" Using CPU_Temp from environment: %2d  \n", smart_fan_target);
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x85, smart_fan_target);
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x86, smart_fan_target);
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x96, smart_fan_target);
       }
    }
    else
    {
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x85, 0x41);	// TTarget_MB   65 C
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x86, 0x41);	// TTarget_CPU1  65 C
          nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x96, 0x41);	// TTarget_CPU2   65 C
    }


    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x80, 0x2D);	// Fan Pre-Scale Register,  PWM frequency < 20kHz
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x82, 0x2D);
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x93, 0x2D);
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0xA0, 0x2D);
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0xA1, 0x2D);
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0xA2, 0x2D);

    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x81, 0x8B);	// Set to PWM mode, FAN_OUT1 to sync. with CPU temperature
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x83, 0x8B);	// Set to PWM mode, FAN_OUT2 to sync. with SYS temperature
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x94, 0x8B);	// Set to PWM mode, FAN_OUT3 to sync. with HDD temperature
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0xA3, 0xAB);	// Set FAN_OUT4 to sync. with CPU temperature, PWM mode
//    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0xA4, 0xAB);	// Set FAN_OUT5 to sync. with CPU temperature, PWM mode

    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x87, 0x00);	// Set Tolerance of target temperature to zero degree
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x97, 0x00);

    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x88, 0x33);	// Set DC mode  FAN Start and Non-Stop duty cycle ratio to 3/15
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x89, 0x33);
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x98, 0x33);

    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x8C, 0x00);	// Set FAN Stop Time to zero, then FAN will not stop
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x8D, 0x00);
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x9A, 0x00);

    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x8E, 0x60);	// Set FAN Step Down Time (times in 10ths of seconds supposedly)
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x8F, 0x60);	// Set FAN Step Up Time 
    nac38_twsi_write8(CFG_I2C_SYSMON_W83792D_ADDR, 0x84, 0x15);	// Enable Smart FAN control

#if 0
   printf("Dump w83792d \n");
    for(i=0; i<256; i+=16)
    {
       printf("%04X: ", i);
       for(j=0; j<16; j++)
       {
          if ( j==7 )
             printf("%02x-", nac38_twsi_read(CFG_I2C_SYSMON_W83792D_ADDR, i+j) & 0xff);
          else
             printf("%02x ", nac38_twsi_read(CFG_I2C_SYSMON_W83792D_ADDR, i+j) & 0xff);
       }
       printf("\n");
    }
#endif
}

int checkboard (void)
{
    cvmx_write_csr(CVMX_SMI_EN, 1);  /* Enable MDIO */
    init_w83792d();
    return 0;
}


int early_board_init(void)
{
    int cpu_ref = NAC38_REV1_DEF_REF_CLOCK_FREQ;

    DECLARE_GLOBAL_DATA_PTR;

    memset((void *)&(gd->mac_desc), 0x0, sizeof(octeon_eeprom_mac_addr_t));
    memset((void *)&(gd->clock_desc), 0x0, sizeof(octeon_eeprom_clock_desc_t));
    memset((void *)&(gd->board_desc), 0x0, sizeof(octeon_eeprom_board_desc_t));

    /* NOTE: this is early in the init process, so the serial port is not yet configured */

    /* Populate global data from eeprom */
    uint8_t ee_buf[OCTEON_EEPROM_MAX_TUPLE_LENGTH];
    int addr;


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
        gd->board_desc.board_type = CVMX_BOARD_TYPE_NAC38;
        gd->board_desc.rev_major = 1;       
        gd->board_desc.rev_minor = 0;       
        gd->board_desc.chip_type = CVMX_CHIP_TYPE_OCTEON_SAMPLE;       
        gd->board_desc.chip_rev_major = 1 + (cvmx_get_proc_id() & 0xff); /* 0 = pass1, 1 = pass2 */
        gd->board_desc.chip_rev_minor = (gd->board_desc.chip_rev_major == 0) ? 3 : 0;
    }

    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, EEPROM_CLOCK_DESC_TYPE, 0, ee_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    if (addr >= 0)
    {
        memcpy((void *)&(gd->clock_desc), ee_buf, sizeof(octeon_eeprom_clock_desc_t));
        gd->ddr_clock_mhz = gd->clock_desc.ddr_clock_mhz;
        cpu_ref = gd->clock_desc.cpu_ref_clock_mhz_x_8/8;
    }
    else
    {
        gd->flags |= GD_FLG_CLOCK_DESC_MISSING;
        /* Default values */
        cpu_ref = NAC38_REV1_DEF_REF_CLOCK_FREQ;
        gd->ddr_clock_mhz = NAC38_REV1_DEF_DRAM_FREQ;
    }

    /* Some sanity checks */
    if (cpu_ref <= 0)
    {
        cpu_ref = NAC38_REV1_DEF_REF_CLOCK_FREQ;
    }
    if (gd->ddr_clock_mhz <= 0)
    {
        gd->ddr_clock_mhz = NAC38_REV1_DEF_DRAM_FREQ;
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
    uint64_t data = cvmx_read_csr(CVMX_DBG_DATA);
    data = data >> 18;
    data &= 0x1f;


    gd->cpu_clock_mhz = data * cpu_ref;

    /* adjust for 33.33 Mhz clock */
    if (cpu_ref == 33)
        gd->cpu_clock_mhz += (data)/4 + data/8;

    if (gd->cpu_clock_mhz < 100 || gd->cpu_clock_mhz > 600)
    {
        gd->cpu_clock_mhz = DEFAULT_ECLK_FREQ_MHZ;
    }



    /*
    ** For the cn58xx the DDR reference clock frequency is used to
    ** configure the appropriate internal DDR_CK/DDR2_REF_CLK ratio in
    ** order to generate the ddr clock frequency specified by
    ** ddr_clock_mhz.  For cn38xx this setting should match the state
    ** of the DDR2 output clock divide by 2 pin DDR2_PLL_DIV2.
    **
    ** For DDR@_PLL_DIV2 = 0 the DDR_CK/DDR2_REF_CLK ratio is 4
    ** For DDR@_PLL_DIV2 = 1 the DDR_CK/DDR2_REF_CLK ratio is 2
    */
#define FIXED_DDR_REF_CLOCK_RATIO	2

    /*
    ** More details about DDR clock configuration used for LMC
    ** configuration for the CN58XX.  Not used for CN38XX.  Since the
    ** reference clock frequency is not known it is inferred from the
    ** specified DCLK frequency divided by the DDR_CK/DDR2_REF_CLK
    ** ratio.
    */
    gd->ddr_ref_hertz = gd->ddr_clock_mhz * 1000000 / FIXED_DDR_REF_CLOCK_RATIO;


    /* Enable the LAN ports that are on the bypass relays */
    cvmx_write_csr(CVMX_GPIO_BIT_CFGX(5), 1);
    cvmx_write_csr(CVMX_GPIO_TX_SET, 0x20);
    return 0;

}

void octeon_led_str_write(const char *str)
{
    return;
}
