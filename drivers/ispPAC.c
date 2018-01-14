/***********************license start***************
 * Copyright (c) 2008  Cavium Networks (support@cavium.com). All rights
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
 * This Software, including technical data, may be subject to U.S.  export
 * control laws, including the U.S.  Export Administration Act and its
 * associated regulations, and may be subject to export or import regulations
 * in other countries.  You warrant that You will comply strictly in all
 * respects with all such regulations and acknowledge that you have the
 * responsibility to obtain licenses to export, re-export or import the
 * Software.
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

/*
** Support functions for the ispPAC-POWR1220AT8 In-System Programmable
** Power Supply Monitoring, Sequencing and Margining Controller.
*/

#include <common.h>
#include <i2c.h>

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINTF   printf
#else
#define DEBUG_PRINTF(...);
#endif


#define ISP_VMON0 0x00
#define ISP_VMON1 0x01
#define ISP_VMON2 0x02
#define ISP_OUT 0x03
#define ISP_OUT0 0x03
#define ISP_OUT1 0x04
#define ISP_OUT2 0x05
#define ISP_INPUT 0x06
#define ISP_ADC_LO 0x07
#define ISP_ADC_HI 0x08
#define ISP_ADC_MUX 0x09
#define ISP_GP_OUT0 0x0E
#define ISP_GP_OUT1 0x0F
#define ISP_GP_OUT2 0x10 
#define ISP_RESET 0x12


extern char *isp_voltage_labels[];

uint8_t read_isp8(uint16_t isp_dev_addr, uint8_t isp_addr)
{
    uint8_t data;

    DEBUG_PRINTF ("%s: Reading device: %#04x address %#04x.\n",
                  __FUNCTION__, isp_dev_addr, isp_addr);

    if(i2c_read(isp_dev_addr, isp_addr, 1, &data, 1) != 0) {
        printf ("%s: Error reading the chip.\n", __FUNCTION__);
    }

    DEBUG_PRINTF ("%s: Data %#08x.\n", __FUNCTION__, data);
    return((uint8_t)data);
}

unsigned int write_isp8(uint16_t isp_dev_addr, uint8_t isp_addr, uint8_t isp_data)
{
    int status = 0;
    uint8_t data;

    DEBUG_PRINTF ("%s: Writing device: %#04x address %#04x data %#04x\n",
                  __FUNCTION__, isp_dev_addr, isp_addr, isp_data);

    data = isp_data;
    if(i2c_write(isp_dev_addr, isp_addr, 1, &data, 1) != 0) {
        printf ("%s: Error writing the chip.\n", __FUNCTION__);
        status = 1;
    }

    DEBUG_PRINTF ("%s: %s\n", __FUNCTION__, status ? "FAILURE" : "SUCCESS");
    return(status);
}


int read_ispPAC_mvolts(uint16_t isp_dev_addr, uint8_t adc_chan)
{
    int millivolts;
    int timeout;

    /* Try big range first */
    if (write_isp8(isp_dev_addr, ISP_ADC_MUX, (adc_chan | 0x10))) {
        return -1;
    }

    /* Poll the DONE bit until the conversion is completed. */
    timeout = 10;
    while (!(read_isp8(isp_dev_addr, ISP_ADC_LO) & 0x01))
    {
        if (!timeout--)
        {
            printf("%s: Voltage conversion timed out\n", __FUNCTION__);
            return -1;
        }
    }
  
    /* Check to see if it's less than 2V  */
    if (read_isp8(isp_dev_addr, ISP_ADC_HI) < 0x3F) {
        /* Switch to little range */
        write_isp8(isp_dev_addr, ISP_ADC_MUX, adc_chan);
    }
  
    /* Poll the DONE bit until the conversion is completed. */
    timeout = 10;
    while (!(read_isp8(isp_dev_addr, ISP_ADC_LO) & 0x01))
    {
        if (!timeout--)
        {
            printf("%s: Voltage conversion timed out\n", __FUNCTION__);
            return -1;
        }
    }
  
    millivolts = (read_isp8(isp_dev_addr, ISP_ADC_LO)) >> 4;
    //DEBUG_PRINTF ("millivolts = 0x%x (%d)\n", millivolts, millivolts);
    millivolts = (read_isp8(isp_dev_addr, ISP_ADC_HI)) << 4 | millivolts ; 
    //DEBUG_PRINTF ("millivolts = 0x%x (%d)\n", millivolts, millivolts);
    millivolts = millivolts * 2 ;      /* 2mv per tick in either mode */
    DEBUG_PRINTF ("millivolts = 0x%x (%d)\n\n", millivolts, millivolts);

    return(millivolts);
}

void print_isp_volt(uint16_t isp_dev_addr, uint8_t adc_chan)
{
    char *label;
    int mVolts = read_ispPAC_mvolts(isp_dev_addr, adc_chan);

    label = isp_voltage_labels[adc_chan];

    if (label != NULL){
        printf("Channel %2d %12s: %d.%03dV\n",
               adc_chan, label, mVolts/1000, mVolts%1000);
    }
}
