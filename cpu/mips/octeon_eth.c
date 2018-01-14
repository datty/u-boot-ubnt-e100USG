/***********************license start************************************
 * Copyright (c) 2005-2007 Cavium Networks (support@cavium.com). All rights
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

/**
 *
 * $Id: octeon_eth.c 54445 2010-10-29 22:59:18Z cchavva $
 * 
 */
 
 
#include <common.h>
#include <command.h>
#include <exports.h>
#include <linux/ctype.h>
#include <lib_octeon.h>
#include <octeon_eeprom_types.h>
#include <net.h>
#include <miiphy.h>
#include "octeon_boot.h"
#include "octeon_eeprom_types.h"
#include "lib_octeon_shared.h"
#include "cvmx-config.h"
#include "cvmx-wqe.h"
#include "cvmx-pow.h"
#include "cvmx-pko.h"
#include "cvmx-ipd.h"
#include "cvmx-pip.h"
#include "cvmx-spi.h"
#include "cvmx-mdio.h"
#include "cvmx-helper.h"
#include "cvmx-mgmt-port.h"
#include "cvmx-helper-util.h"
#include "cvmx-bootmem.h"

#define DEBUG 1
#undef  DEBUG

#ifdef DEBUG
#define dprintf printf
#else
#define dprintf(...)
#endif

/* Global flag indicating common hw has been set up */
int octeon_global_hw_inited = 0;

/* pointer to device to use for loopback.  Only needed for IPD reset workaround */
struct eth_device *loopback_dev;

int octeon_eth_send(struct eth_device *dev, volatile void *packet, int length);
int octeon_ebt3000_rgmii_init(struct eth_device *dev, bd_t * bis);
void octeon_eth_rgmii_enable(struct eth_device *dev);


/************************************************************************/
/* Ethernet device private data structure for octeon ethernet */
typedef struct
{
    uint32_t port;
    uint32_t interface;
    uint32_t queue;
    uint32_t initted_flag;
    uint32_t link_speed:2;   /* current link status, use to reconfigure on status changes */
    uint32_t link_duplex:1;
    uint32_t link_status:1;
    uint32_t loopback:1;
    uint32_t enabled:1;
} octeon_eth_info_t;

#if  defined(OCTEON_INTERNAL_ENET) || defined(OCTEON_MGMT_ENET)

/* Variable shared between ethernet routines. */
static int card_number;
static int available_mac_count = -1;
static uint8_t mac_addr[6];
static uint64_t port_state[50];  /* Save previous link state so we only configure/print on transitions */


#define USE_HW_TCPUDP_CHECKSUM  0

/* Make sure that we have enough buffers to keep prefetching blocks happy.
** Absolute minimum is probably about 200. */
#define NUM_PACKET_BUFFERS  1000

#define PKO_SHUTDOWN_TIMEOUT_VAL     100
/****************************************************************/


static void cvm_oct_fill_hw_memory(uint64_t pool, uint64_t size, uint64_t elements)
{

    int64_t memory;
    static int alloc_count = 0;
    char tmp_name[64];

    dprintf("cvm_oct_fill_hw_memory: pool: 0x%llx, size: 0xx%llx, count: 0x%llx\n", pool, size, elements);
    sprintf(tmp_name, "%s_fpa_alloc_%d", OCTEON_BOOTLOADER_NAMED_BLOCK_TMP_PREFIX, alloc_count++);
    memory = cvmx_bootmem_phy_named_block_alloc(size * elements, 0, 0x40000000, 128, tmp_name, CVMX_BOOTMEM_FLAG_END_ALLOC);
    if (memory < 0)
    {
        printf("Unable to allocate %lu bytes for FPA pool %ld\n", elements*size, pool);
        return;
    }
    while (elements--)
    {
        cvmx_fpa_free((void *)(uint32_t)(memory + elements*size), pool, 0);
    }
}


/**
 * Set the hardware MAC address for a device
 *
 * @param interface    interface of port to set
 * @param index    index of port to set MAC address for
 * @param addr   Address structure to change it too. 
 * @return Zero on success
 */
static int cvm_oct_set_mac_address(int interface, int index, void *addr)
{
    cvmx_gmxx_prtx_cfg_t gmx_cfg;


    int i;
    uint8_t *ptr = addr;
    uint64_t mac = 0;
    for (i=0; i<6; i++)
        mac = (mac<<8) | (uint64_t)(ptr[i]);

    /* Disable interface */
    gmx_cfg.u64 = cvmx_read_csr(CVMX_GMXX_PRTX_CFG(index, interface));
    cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, interface), gmx_cfg.u64 & ~1ull);

    cvmx_write_csr(CVMX_GMXX_SMACX(index, interface), mac);
    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM0(index, interface), ptr[0]);
    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM1(index, interface), ptr[1]);
    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM2(index, interface), ptr[2]);
    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM3(index, interface), ptr[3]);
    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM4(index, interface), ptr[4]);
    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM5(index, interface), ptr[5]);


    cvmx_gmxx_rxx_adr_ctl_t control;
    control.u64 = 0;
    control.s.bcst = 1;     /* Allow broadcast MAC addresses */
    control.s.mcst = 1; /* Force reject multicast packets */
    control.s.cam_mode = 1; /* Filter packets based on the CAM */
    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CTL(index, interface), control.u64);

    cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM_EN(index, interface), 1);


    /* Return interface to previous enable state */
    cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, interface), gmx_cfg.u64);

    return 0;
}

/**
 * Configure common hardware for all interfaces
 */
static void cvm_oct_configure_common_hw(void)
{

    if (getenv("disable_spi"))
    {

        /* Do this here so that we can disabled this after boot but before
        ** we use networking.
        */
        cvmx_spi_callbacks_t spi_callbacks_struct;
        cvmx_spi_get_callbacks(&spi_callbacks_struct);
        spi_callbacks_struct.reset_cb            = 0;
        spi_callbacks_struct.calendar_setup_cb   = 0;
        spi_callbacks_struct.clock_detect_cb     = 0;
        spi_callbacks_struct.training_cb         = 0;
        spi_callbacks_struct.calendar_sync_cb    = 0;
        spi_callbacks_struct.interface_up_cb     = 0;
        printf("SPI interface disabled with 'disable_spi' environment variable\n");
        cvmx_spi_set_callbacks(&spi_callbacks_struct);
    }
    /* Setup the FPA */
    cvmx_fpa_enable();

    cvm_oct_fill_hw_memory(CVMX_FPA_WQE_POOL, CVMX_FPA_WQE_POOL_SIZE, NUM_PACKET_BUFFERS);
#if CVMX_FPA_OUTPUT_BUFFER_POOL != CVMX_FPA_PACKET_POOL
    cvm_oct_fill_hw_memory(CVMX_FPA_OUTPUT_BUFFER_POOL, CVMX_FPA_OUTPUT_BUFFER_POOL_SIZE, 128);
#endif
    cvm_oct_fill_hw_memory(CVMX_FPA_PACKET_POOL, CVMX_FPA_PACKET_POOL_SIZE, NUM_PACKET_BUFFERS);

    cvmx_helper_initialize_packet_io_global();
    cvmx_helper_initialize_packet_io_local();

    /* Set POW get work timeout to maximum value */
    cvmx_write_csr(CVMX_POW_NW_TIM, 0x3ff);
}

/**
 * Packet transmit
 *
 * @param skb    Packet to send
 * @param dev    Device info structure
 * @return Always returns zero
 */
static int cvm_oct_xmit(struct eth_device *dev, void *packet, int len)
{
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;
    cvmx_pko_command_word0_t    pko_command;
    cvmx_buf_ptr_t              hw_buffer;


    dprintf("cvm_oct_xmit addr: %p, len: %d\n", packet, len);


    /* Build the PKO buffer pointer */
    hw_buffer.u64 = 0;
    hw_buffer.s.addr = cvmx_ptr_to_phys(packet);
    hw_buffer.s.pool = CVMX_FPA_PACKET_POOL;
    hw_buffer.s.size = CVMX_FPA_PACKET_POOL_SIZE;

    /* Build the PKO command */
    pko_command.u64 = 0;
    pko_command.s.subone0 = 1;
    pko_command.s.dontfree = 0;
    pko_command.s.segs = 1;
    pko_command.s.total_bytes = len;
    /* Send the packet to the output queue */

    cvmx_pko_send_packet_prepare(priv->port, priv->queue, 0);
    if (cvmx_pko_send_packet_finish(priv->port, priv->queue, pko_command, hw_buffer, 0))
    {
        printf("Failed to send the packet\n");
    }
	return 0;
}

/**
 * Configure the RGMII port for the negotiated speed
 *
 * @param dev    Linux device for the RGMII port
 */
static void cvm_oct_configure_rgmii_speed(struct eth_device *dev)
{
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;

    cvmx_helper_link_info_t link_state = cvmx_helper_link_get(priv->port);

    if (link_state.u64 != port_state[priv->port])
    {

        printf("%s: ", dev->name);
        if (!link_state.s.link_up)
            printf("Down ");
        else
        {
            printf("Up ");
            printf("%d Mbps ", link_state.s.speed);
            if (link_state.s.full_duplex)
                printf("Full duplex ");
            else
                printf("Half duplex ");
        }
        printf("(port %2d)\n", priv->port);
        cvmx_helper_link_set(priv->port, link_state);
        port_state[priv->port] = link_state.u64;
    }
}

#if defined(OCTEON_MGMT_ENET)
int octeon_mgmt_port_reset_port(int port)
{
    cvmx_mixx_ctl_t mix_ctl;
    if (OCTEON_IS_MODEL(OCTEON_CN52XX) && (port < 0 || port > 2))
        return -1;
    else if (OCTEON_IS_MODEL(OCTEON_CN56XX) && (port < 0 || port > 1))
        return -1;
    else if (OCTEON_IS_MODEL(OCTEON_CN63XX) && (port < 0 || port > 2))
        return -1;

    mix_ctl.u64 = cvmx_read_csr(CVMX_MIXX_CTL(port));
    mix_ctl.s.en = 0;
    cvmx_write_csr(CVMX_MIXX_CTL(port), mix_ctl.u64);
    do
    {
        mix_ctl.u64 = cvmx_read_csr(CVMX_MIXX_CTL(port));
    } while (mix_ctl.s.busy);
    mix_ctl.s.reset = 1;
    cvmx_write_csr(CVMX_MIXX_CTL(port), mix_ctl.u64);
    cvmx_read_csr(CVMX_MIXX_CTL(port));
    return 0;
}

/* Really shutdown the management ports, including resetting the hardware
** units.  This is needed for NAND boot stage2 -> stage3 handoff.  The MGMT ports
** need to be reset before they are handed off to stage 3.
** This is not needed with newer MGMT init code that properly handles this
** case at init time. */
int octeon_mgmt_port_shutdown(void)
{
    octeon_mgmt_port_reset_port(0);
    octeon_mgmt_port_reset_port(1);
    cvmx_bootmem_free_named("cvmx_mgmt_port");
    return 0;
}
#endif

int octeon_network_hw_shutdown(void)
{
    int retval = 0;
    int i, count;

    if (octeon_global_hw_inited)
    {
        retval = cvmx_helper_shutdown_packet_io_global();
        cvmx_helper_shutdown_packet_io_local();
        /* Free FPA buffers from all the pools before disabling */
        for (i = 0; i < 8; i++)
        {
            count = 0;
            while (cvmx_fpa_alloc(i))
                count++;
            dprintf("Freed %d buffers from pool %d\n", count, i);
        }
        /* Also disable FPA, it is not done by helper functions. */
        cvmx_fpa_ctl_status_t fpa_ctl_status;
        fpa_ctl_status.u64 = cvmx_read_csr(CVMX_FPA_CTL_STATUS);
        fpa_ctl_status.s.reset = 1;
        cvmx_write_csr(CVMX_FPA_CTL_STATUS, fpa_ctl_status.u64);
    }

    /* Free command queue named block, as we have reset the hardware */
    cvmx_bootmem_phy_named_block_free("cvmx_cmd_queues", 0);
    if (retval < 0)
    {
        /* Make this a fatal error since the error message is easily missed and ignoring
        ** it can lead to very strange networking behavior in the application.
        */
        printf("FATAL ERROR: Network shutdown failed.  Please reset the board.\n");
        while (1)
            ;
    }
    return(retval);
}

/*******************  Octeon networking functions ********************/
int octeon_ebt3000_rgmii_init(struct eth_device *dev, bd_t * bis)         /* Initialize the device	*/
{
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;
    int interface = priv->interface;
    int port = priv->port;

    dprintf("%s(), dev_ptr: %p, port: %d, queue: %d\n", __FUNCTION__, dev, priv->port, priv->queue);

    if (priv->initted_flag)
        return 1;

    if (!octeon_global_hw_inited)
    {
        cvm_oct_configure_common_hw();
    }

    /* Ignore backpressure on RGMII ports */
    cvmx_write_csr(CVMX_GMXX_TX_OVR_BP(interface), 0xf << 8 | 0xf);

    cvm_oct_set_mac_address(interface, port & 0x3, dev->enetaddr);

    if (!octeon_global_hw_inited)
    {
        cvmx_helper_ipd_and_packet_input_enable();
        octeon_global_hw_inited = 1;
    }
    priv->initted_flag = 1;
    return(1);
}




#ifdef OCTEON_SPI4000_ENET
int octeon_spi4000_init(struct eth_device *dev, bd_t * bis)         /* Initialize the device	*/
{
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;
    static char spi4000_inited[2] = {0};
    dprintf("octeon_spi4000_init(), dev_ptr: %p, port: %d, queue: %d\n", dev, priv->port, priv->queue);


    if (priv->initted_flag)
        return 1;

    if (!octeon_global_hw_inited)
    {
        cvm_oct_configure_common_hw();
    }

    if (!spi4000_inited[priv->interface])
    {
        spi4000_inited[priv->interface] = 1;
        if (__cvmx_helper_spi_enable(priv->interface) < 0)
        {
            printf("ERROR initializing spi4000 on Octeon Interface %d\n", priv->interface);
            return 0;
        }
    }

    if (!octeon_global_hw_inited)
    {
        cvmx_helper_ipd_and_packet_input_enable();
        octeon_global_hw_inited = 1;
    }
    priv->initted_flag = 1;
    return(1);
}
#endif




void octeon_eth_rgmii_enable(struct eth_device *dev)
{
    octeon_eth_info_t *oct_eth_info;
    oct_eth_info = (octeon_eth_info_t *)dev->priv;

    if (CVMX_HELPER_INTERFACE_MODE_RGMII == cvmx_helper_interface_get_mode(oct_eth_info->interface)
        || CVMX_HELPER_INTERFACE_MODE_GMII == cvmx_helper_interface_get_mode(oct_eth_info->interface))
    {
        uint64_t tmp;
        tmp = cvmx_read_csr(CVMX_ASXX_RX_PRT_EN(oct_eth_info->interface));
        tmp |= (1ull << (oct_eth_info->port & 0x3));
        cvmx_write_csr(CVMX_ASXX_RX_PRT_EN(oct_eth_info->interface), tmp);
        tmp = cvmx_read_csr(CVMX_ASXX_TX_PRT_EN(oct_eth_info->interface));
        tmp |= (1ull << (oct_eth_info->port & 0x3));
        cvmx_write_csr(CVMX_ASXX_TX_PRT_EN(oct_eth_info->interface), tmp);
    }


}




/* Keep track of packets sent on each interface so we can
** autonegotiate before we send the first one.  This is cleared in
** eth_try_another. */
int packets_sent;  
int octeon_eth_send(struct eth_device *dev, volatile void *packet, int length)	   /* Send a packet	*/
{
    dprintf("ethernet TX! ptr: %p, len: %d\n", packet, length);
    /* We need to copy this to a FPA buffer, then give that to TX */

#if defined(DEBUG) && 0
    /* Dump out packet contents */
    {
	int i, j;
	unsigned char *up = packet;

	for (i = 0; (i + 16) < length; i += 16)
	{
	    printf("%04x ", i);
	    for (j = 0; j < 16; ++j)
	    {
		printf("%02x ", up[i+j]);
	    }
	    printf("    ");
	    for (j = 0; j < 16; ++j)
	    {
		printf("%c", ((up[i+j] >= ' ') && (up[i+j] <= '~')) ? up[i+j] : '.');
	    }
	    printf("\n");
	}
	printf("%04x ", i);
	for (j = 0; i+j < length; ++j)
	{
	    printf("%02x ", up[i+j]);
	}
	for (; j < 16; ++j)
	{
	    printf("   ");
	}
	printf("    ");
	for (j = 0; i+j < length; ++j)
	{
	    printf("%c", ((up[i+j] >= ' ') && (up[i+j] <= '~')) ? up[i+j] : '.');
	}
	printf("\n");
    }
#endif

    /* This is needed since we disable rx/tx in halt */
    octeon_eth_rgmii_enable(dev);
    if (packets_sent++ == 0)
        cvm_oct_configure_rgmii_speed(dev);

    void *fpa_buf = cvmx_fpa_alloc(CVMX_FPA_PACKET_POOL);
    if (!fpa_buf)
    {
        printf("ERROR allocating buffer for packet!\n");
    }
    memcpy(fpa_buf, (void *)packet, length);
    cvm_oct_xmit(dev, fpa_buf, length);
    return(0);
}

int octeon_eth_recv(struct eth_device *dev)         /* Check for received packets	*/
{
    cvmx_wqe_t *work = cvmx_pow_work_request_sync(1);
    if (!work)
    {
        /* Poll for link status change, only if work times out.
        ** On some interfaces this is really slow.
        ** getwork timeout is set to maximum. */
        cvm_oct_configure_rgmii_speed(dev);
        return(0);
    }
    if (work->word2.s.rcv_error)
    {
        /* Work has error, so drop */
        printf("Error packet received (code %d), dropping\n", work->word2.s.err_code);
        cvmx_helper_free_packet_data(work);
        cvmx_fpa_free(work, CVMX_FPA_WQE_POOL, 0);
        return(0);
    }


    void *packet_data = cvmx_phys_to_ptr(work->packet_ptr.u64 & 0xffffffffffull);
    int length = work->len;

    dprintf("############# got work: %p, len: %d, packet_ptr: %p\n",
            work, length, packet_data);
#if defined(DEBUG) && 0
    /* Dump out packet contents */
    {
	int i, j;
	unsigned char *up = packet_data;

	for (i = 0; (i + 16) < length; i += 16)
	{
	    printf("%04x ", i);
	    for (j = 0; j < 16; ++j)
	    {
		printf("%02x ", up[i+j]);
	    }
	    printf("    ");
	    for (j = 0; j < 16; ++j)
	    {
		printf("%c", ((up[i+j] >= ' ') && (up[i+j] <= '~')) ? up[i+j] : '.');
	    }
	    printf("\n");
	}
	printf("%04x ", i);
	for (j = 0; i+j < length; ++j)
	{
	    printf("%02x ", up[i+j]);
	}
	for (; j < 16; ++j)
	{
	    printf("   ");
	}
	printf("    ");
	for (j = 0; i+j < length; ++j)
	{
	    printf("%c", ((up[i+j] >= ' ') && (up[i+j] <= '~')) ? up[i+j] : '.');
	}
	printf("\n");
    }
#endif

    NetReceive (packet_data, length);
    cvmx_helper_free_packet_data(work);
    cvmx_fpa_free(work, CVMX_FPA_WQE_POOL, 0);
    /* Free WQE and packet data */
    return(length);
}
void octeon_eth_halt(struct eth_device *dev)			/* stop SCC			*/
{
    /* Disable reception on this port at the GMX block */
    octeon_eth_info_t *oct_eth_info;

    oct_eth_info = (octeon_eth_info_t *)dev->priv;

    if (CVMX_HELPER_INTERFACE_MODE_RGMII == cvmx_helper_interface_get_mode(oct_eth_info->interface)
        || CVMX_HELPER_INTERFACE_MODE_GMII == cvmx_helper_interface_get_mode(oct_eth_info->interface))
    {
        int index = oct_eth_info->port & 0x3;
        uint64_t tmp;
        cvmx_gmxx_prtx_cfg_t gmx_cfg;
        tmp = cvmx_read_csr(CVMX_ASXX_RX_PRT_EN(oct_eth_info->interface));
        tmp &= ~(1ull << (oct_eth_info->port & 0x3));
        cvmx_write_csr(CVMX_ASXX_RX_PRT_EN(oct_eth_info->interface), tmp);   /* Disable the RGMII RX ports */
        tmp = cvmx_read_csr(CVMX_ASXX_TX_PRT_EN(oct_eth_info->interface));
        tmp &= ~(1ull << (oct_eth_info->port & 0x3));
        cvmx_write_csr(CVMX_ASXX_TX_PRT_EN(oct_eth_info->interface), tmp);   /* Disable the RGMII TX ports */

        /* Disable MAC filtering */
        gmx_cfg.u64 = cvmx_read_csr(CVMX_GMXX_PRTX_CFG(index, oct_eth_info->interface));
        cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, oct_eth_info->interface), gmx_cfg.u64 & ~1ull);
        cvmx_write_csr(CVMX_GMXX_RXX_ADR_CTL(index, oct_eth_info->interface), 1);
        cvmx_write_csr(CVMX_GMXX_RXX_ADR_CAM_EN(index, oct_eth_info->interface), 0);
        cvmx_write_csr(CVMX_GMXX_PRTX_CFG(index, oct_eth_info->interface), gmx_cfg.u64);
    }

}

/*********************************************************
Only handle built-in RGMII/SGMII/XAUI/GMII ports here
*********************************************/
int octeon_eth_initialize (bd_t * bis)
{
    DECLARE_GLOBAL_DATA_PTR;
    struct eth_device *dev;
    octeon_eth_info_t *oct_eth_info;
    int port;
    int interface;
    uint32_t *mac_inc_ptr = (uint32_t *)(&mac_addr[2]);
    int num_ports;
    int num_ints;

    dprintf("%s called\n", __FUNCTION__ );
    if (available_mac_count == -1)
    {
        available_mac_count = gd->mac_desc.count;
        memcpy(mac_addr, (uint8_t *)(gd->mac_desc.mac_addr_base), 6);
    }
    if (getenv("disable_networking"))
    {
        printf("Networking disabled with 'disable_networking' environment variable, skipping RGMII interface\n");
        return 0;
    }
    if (available_mac_count <= 0)
    {
        printf("No available MAC addresses for RGMII interface, skipping\n");
        return 0;
    }
    /* Initialize port state array to invalid values */
    memset(port_state, 0xff, sizeof(port_state));

    /* Do board specific init in early_board_init or checkboard if possible. */

    /* NOTE: on 31XX based boards, the GMXX_INF_MODE register must be set appropriately
    ** before this code is run (in checkboard for instance).  The hardware is configured based
    ** on the settings of GMXX_INF_MODE. */

    /* We only care about the first two (real packet interfaces)
    ** This will ignore PCI, loop, etc interfaces. */
    num_ints = cvmx_helper_get_number_of_interfaces();
    dprintf("Num interfaces: %d\n", num_ints);
    if (num_ints > 2)
        num_ints = 2;

    /* Check to see what interface and ports we should use */
    for (interface = 0; interface < num_ints; interface++)
    {
        if (CVMX_HELPER_INTERFACE_MODE_RGMII == cvmx_helper_interface_get_mode(interface)
            || CVMX_HELPER_INTERFACE_MODE_GMII == cvmx_helper_interface_get_mode(interface)
            || CVMX_HELPER_INTERFACE_MODE_SGMII == cvmx_helper_interface_get_mode(interface)
            || CVMX_HELPER_INTERFACE_MODE_XAUI == cvmx_helper_interface_get_mode(interface))
        {
            cvmx_helper_interface_probe(interface);
            num_ports = cvmx_helper_ports_on_interface(interface);
            dprintf("Interface %d is %s, ports: %d\n", interface, cvmx_helper_interface_mode_to_string(cvmx_helper_interface_get_mode(interface)), num_ports);
            /* RGMII or SGMII */
            int index = 0;
            for (index = 0, port = 16 * interface; (port < num_ports + 16 * interface) && available_mac_count-- > 0; port++, index++)
            {
                dev = (struct eth_device *) malloc(sizeof(*dev));

                /* Save pointer for port 16 or 0 to use in IPD reset workaround */
                if (port == 16 || port == 0)
                    loopback_dev = dev;

                oct_eth_info = (octeon_eth_info_t *) malloc(sizeof(octeon_eth_info_t));

                oct_eth_info->port = port;
                oct_eth_info->queue = cvmx_pko_get_base_queue(oct_eth_info->port);
                oct_eth_info->interface = interface;
                oct_eth_info->initted_flag = 0;

                dprintf("Setting up port: %d, queue: %d, int: %d, device: octeth%d\n", oct_eth_info->port, oct_eth_info->queue, oct_eth_info->interface, card_number);
                sprintf (dev->name, "octeth%d", card_number);
                card_number++;

                dev->priv = (void *)oct_eth_info;
                dev->iobase = 0;
                dev->init = octeon_ebt3000_rgmii_init;
                dev->halt = octeon_eth_halt;
                dev->send = octeon_eth_send;
                dev->recv = octeon_eth_recv;
                memcpy(dev->enetaddr, mac_addr, 6);
                (*mac_inc_ptr)++;  /* increment MAC address */

                eth_register (dev);

            }
#if 0
            /* Check to see if we have any outstanding doorbells in any queues we might use.
            ** Warn if we find any, as that means the hardware is not in the expected (reset) state.
            ** The PKO has not been enabled at this point, so this may not be a good place to check this.
            */
            for (port = 16 * interface; (port < num_ports + 16 * interface); port++)
            {
                uint32_t dbell = cvmx_cmd_queue_length(CVMX_CMD_QUEUE_PKO(cvmx_pko_get_base_queue(port)));
                if (dbell)
                {
                    printf("WARNING: queue %d has a non-zero doorbell count of %d\n", cvmx_pko_get_base_queue(port), dbell);
                }
            }
#endif
        }
    }


    return card_number;
}



#ifdef OCTEON_SPI4000_ENET
int octeon_spi4000_initialize(bd_t * bis)
{
    DECLARE_GLOBAL_DATA_PTR;
    struct eth_device *dev;
    octeon_eth_info_t *oct_eth_info;
    int port;
    int interface = 0;
    uint32_t *mac_inc_ptr = (uint32_t *)(&mac_addr[2]);
    cvmx_spi_callbacks_t spi_callbacks_struct;

    if (available_mac_count == -1)
    {
        available_mac_count = gd->mac_desc.count;
        memcpy(mac_addr, (void *)(gd->mac_desc.mac_addr_base), 6);
    }
    if (getenv("disable_networking"))
    {
        printf("Networking disabled with 'disable_networking' environment variable, skipping SPI interface\n");
        return 0;
    }

    /* Re-setup callbacks here, as the initialization done at compile time points
    ** to the link addresses in flash.
    */
    cvmx_spi_get_callbacks(&spi_callbacks_struct);
    spi_callbacks_struct.reset_cb            = cvmx_spi_reset_cb;
    spi_callbacks_struct.calendar_setup_cb   = cvmx_spi_calendar_setup_cb;
    spi_callbacks_struct.clock_detect_cb     = cvmx_spi_clock_detect_cb;
    spi_callbacks_struct.training_cb         = cvmx_spi_training_cb;
    spi_callbacks_struct.calendar_sync_cb    = cvmx_spi_calendar_sync_cb;
    spi_callbacks_struct.interface_up_cb     = cvmx_spi_interface_up_cb;
    cvmx_spi_set_callbacks(&spi_callbacks_struct);

    if (!cvmx_spi4000_is_present(interface))
        return 0;
    if (available_mac_count <= 0)
    {
        printf("No available MAC addresses for SPI interface, skipping\n");
        return 0;
    }

    for (port = 0; port < 10 && available_mac_count-- > 0; port++)
    {
        dev = (struct eth_device *) malloc(sizeof(*dev));
        oct_eth_info = (octeon_eth_info_t *) malloc(sizeof(octeon_eth_info_t));

        oct_eth_info->port = port;
        oct_eth_info->queue = cvmx_pko_get_base_queue(oct_eth_info->port);
        oct_eth_info->interface = interface;
        oct_eth_info->initted_flag = 0;

        dprintf("Setting up SPI4000 port: %d, queue: %d, int: %d\n", oct_eth_info->port, oct_eth_info->queue, oct_eth_info->interface);
        sprintf (dev->name, "octspi%d", port);
        card_number++;

        dev->priv = (void *)oct_eth_info;
        dev->iobase = 0;
        dev->init = octeon_spi4000_init;
        dev->halt = octeon_eth_halt;
        dev->send = octeon_eth_send;
        dev->recv = octeon_eth_recv;
        memcpy(dev->enetaddr, mac_addr, 6);
        (*mac_inc_ptr)++;  /* increment MAC address */

        eth_register (dev);
    }
    return card_number;
}
#endif  /* OCTEON_SPI4000_ENET */

#ifdef OCTEON_SPI_IXF18201_ENET
int octeon_spi_ixf18201_init(struct eth_device *dev, bd_t * bis)         /* Initialize the device	*/
{
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;
    static char spi_inited[2] = {0};
    dprintf("octeon_spi_init(), dev_ptr: %p, port: %d, queue: %d\n", dev, priv->port, priv->queue);


    if (priv->initted_flag)
        return 1;

    if (!octeon_global_hw_inited)
    {
        cvm_oct_configure_common_hw();
    }

    if (!spi_inited[priv->interface])
    {
        spi_inited[priv->interface] = 1;
        if (__cvmx_helper_spi_enable(priv->interface) < 0)
        {
            printf("ERROR initializing spi on Octeon Interface %d\n", priv->interface);
            return 0;
        }
    }

    if (!octeon_global_hw_inited)
    {
        cvmx_helper_ipd_and_packet_input_enable();
        octeon_global_hw_inited = 1;
    }
    priv->initted_flag = 1;
    return(1);
}
int octeon_spi_ixf18201_initialize(bd_t * bis)
{
    DECLARE_GLOBAL_DATA_PTR;
    struct eth_device *dev;
    octeon_eth_info_t *oct_eth_info;
    int port;
    int interface = 0;
    uint32_t *mac_inc_ptr = (uint32_t *)(&mac_addr[2]);
    cvmx_spi_callbacks_t spi_callbacks_struct;

    if (available_mac_count == -1)
    {
        available_mac_count = gd->mac_desc.count;
        memcpy(mac_addr, (void *)(gd->mac_desc.mac_addr_base), 6);
    }
    if (getenv("disable_networking"))
    {
        printf("Networking disabled with 'disable_networking' environment variable, skipping SPI interface\n");
        return 0;
    }

    /* Re-setup callbacks here, as the initialization done at compile time points
    ** to the link addresses in flash.
    */
    cvmx_spi_get_callbacks(&spi_callbacks_struct);
    spi_callbacks_struct.reset_cb            = cvmx_spi_reset_cb;
    spi_callbacks_struct.calendar_setup_cb   = cvmx_spi_calendar_setup_cb;
    spi_callbacks_struct.clock_detect_cb     = cvmx_spi_clock_detect_cb;
    spi_callbacks_struct.training_cb         = cvmx_spi_training_cb;
    spi_callbacks_struct.calendar_sync_cb    = cvmx_spi_calendar_sync_cb;
    spi_callbacks_struct.interface_up_cb     = cvmx_spi_interface_up_cb;
    cvmx_spi_set_callbacks(&spi_callbacks_struct);

    if (available_mac_count <= 0)
    {
        printf("No available MAC addresses for SPI interface, skipping\n");
        return 0;
    }

    int eth_num=0;
    for (interface = 0;interface < 2;interface++)
    {
        int ports = 1;  /* We only want to set up the real ports here */
        for (port = 0; port < ports && available_mac_count-- > 0; port++)
        {
            dev = (struct eth_device *) malloc(sizeof(*dev));
            oct_eth_info = (octeon_eth_info_t *) malloc(sizeof(octeon_eth_info_t));

            oct_eth_info->port = port;
            oct_eth_info->queue = cvmx_pko_get_base_queue(oct_eth_info->port);
            oct_eth_info->interface = interface;
            oct_eth_info->initted_flag = 0;

            dprintf("Setting up SPI port: %d, queue: %d, int: %d\n", oct_eth_info->port, oct_eth_info->queue, oct_eth_info->interface);
            sprintf (dev->name, "octspi%d", eth_num++);
            card_number++;

            dev->priv = (void *)oct_eth_info;
            dev->iobase = 0;
            dev->init = octeon_spi_ixf18201_init;
            dev->halt = octeon_eth_halt;
            dev->send = octeon_eth_send;
            dev->recv = octeon_eth_recv;
            memcpy(dev->enetaddr, mac_addr, 6);
            (*mac_inc_ptr)++;  /* increment MAC address */

            eth_register (dev);
        }
    }
    return card_number;
}
#endif  /* OCTEON_SPI_IXF18201_ENET */

#endif   /* OCTEON_INTERNAL_ENET */


#if defined(CONFIG_MII) || (CONFIG_COMMANDS & CFG_CMD_MII)



#ifdef CONFIG_MII_CLAUSE_45
int  miiphy_read45(unsigned char addr, unsigned char dev, unsigned short reg, unsigned short * value)
{

    int bus_id = !!(addr & 0x80);
    int retval = cvmx_mdio_45_read(bus_id, addr, dev, reg);
    if (retval < 0)
    {
        *value = 0xBADD;
        return -1;
    }
    *value = (unsigned short)retval;
    return 0;
}
int miiphy_write45(unsigned char  addr,
                   unsigned char dev, 
		 unsigned short  reg,
		 unsigned short value)
{
    int bus_id = !!(addr & 0x80);
    if (cvmx_mdio_45_write(bus_id, addr, dev, reg, value) < 0)
        return(-1);
    else
        return(0);
}
#endif

/**
 * Perform an MII read. Called by the generic MII routines
 *
 * @param dev      Device to perform read for
 * @param phy_id   The MII phy id
 * @param location Register location to read
 * @return Result from the read or zero on failure
 */

int  miiphy_read(unsigned char addr, unsigned char reg, unsigned short * value)
{

    int bus_id = !!(addr & 0x80);
    int retval = cvmx_mdio_read(bus_id, addr, reg);
    if (retval < 0)
        return -1;
    *value = (unsigned short)retval;
    return 0;
}

/* Wrapper to make non-error checking code simpler */
int miiphy_read_wrapper(unsigned char  addr, unsigned char  reg)
{
    unsigned short value;

    if (miiphy_read(addr, reg, &value) < 0)
    {
        printf("ERROR: miiphy_read_wrapper(0x%x, 0x%x) timed out.\n", addr, reg);
        return (-1);
    }

    return(value);
}



int miiphy_write(unsigned char  addr,
		 unsigned char  reg,
		 unsigned short value)
{
    int bus_id = !!(addr & 0x80);
    if (cvmx_mdio_write(bus_id, addr, reg, value) < 0)
        return(-1);
    else
        return(0);
}

#endif /* MII */



#if defined(OCTEON_MGMT_ENET) && (CONFIG_COMMANDS & CFG_CMD_NET)


int octeon_mgmt_eth_init(struct eth_device *dev, bd_t * bis)         /* Initialize the device	*/
{
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;
    dprintf("%s(), dev_ptr: %p\n", __FUNCTION__, dev);

    if (priv->initted_flag)
        return 1;

    priv->enabled = 0;

    priv->initted_flag = 1;
    return(1);
}
int octeon_mgmt_eth_send(struct eth_device *dev, volatile void *packet, int length)	   /* Send a packet	*/
{
    int retval;
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;

    if (!priv->enabled)
    {
       priv->enabled = 1;
       cvmx_mgmt_port_enable(priv->port);
    }
    retval = cvmx_mgmt_port_send(priv->port, length, (void *)packet);

    if (packets_sent++ == 0)
        cvmx_mgmt_port_link_set(priv->port, cvmx_mgmt_port_link_get(priv->port));

    return retval;

}
#define MGMT_BUF_SIZE   1700
int octeon_mgmt_eth_recv(struct eth_device *dev)         /* Check for received packets	*/
{
    uchar buffer[MGMT_BUF_SIZE];
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;


    if (!priv->enabled)
    {
       priv->enabled = 1;
       cvmx_mgmt_port_enable(priv->port);
    }
    int length = cvmx_mgmt_port_receive(priv->port, MGMT_BUF_SIZE, buffer);

    if (length > 0)
    {
#if defined(DEBUG) && 0
    /* Dump out packet contents */
    {
	int i, j;
	unsigned char *up = buffer;

	for (i = 0; (i + 16) < length; i += 16)
	{
	    printf("%04x ", i);
	    for (j = 0; j < 16; ++j)
	    {
		printf("%02x ", up[i+j]);
	    }
	    printf("    ");
	    for (j = 0; j < 16; ++j)
	    {
		printf("%c", ((up[i+j] >= ' ') && (up[i+j] <= '~')) ? up[i+j] : '.');
	    }
	    printf("\n");
	}
	printf("%04x ", i);
	for (j = 0; i+j < length; ++j)
	{
	    printf("%02x ", up[i+j]);
	}
	for (; j < 16; ++j)
	{
	    printf("   ");
	}
	printf("    ");
	for (j = 0; i+j < length; ++j)
	{
	    printf("%c", ((up[i+j] >= ' ') && (up[i+j] <= '~')) ? up[i+j] : '.');
	}
	printf("\n");
    }
#endif
        dprintf("MGMT port got %d byte packet\n", length);
        NetReceive (buffer, length);
    }
    else if (length < 0)
    {
        printf("MGMT port rx error: %d\n", length);
    }


    return(length);
}

void octeon_mgmt_eth_halt(struct eth_device *dev)			/* stop SCC			*/
{
    octeon_eth_info_t* priv = (octeon_eth_info_t*)dev->priv;
    priv->enabled = 0;
    cvmx_mgmt_port_disable(priv->port);
}
int octeon_mgmt_eth_initialize (bd_t * bis)
{
    DECLARE_GLOBAL_DATA_PTR;
    struct eth_device *dev;
    octeon_eth_info_t *oct_eth_info;
    uint32_t *mac_inc_ptr = (uint32_t *)(&mac_addr[2]);
    int mgmt_port_count = 1;
    int cur_port;
    int retval;

    if (OCTEON_IS_MODEL(OCTEON_CN52XX) || OCTEON_IS_MODEL(OCTEON_CN63XX))
        mgmt_port_count = 2;


    dprintf("%s called\n", __FUNCTION__ );
    if (available_mac_count == -1)
    {
        available_mac_count = gd->mac_desc.count;
        memcpy(mac_addr, (uint8_t *)(gd->mac_desc.mac_addr_base), 6);
    }
    if (getenv("disable_networking"))
    {
        printf("Networking disabled with 'disable_networking' environment variable, skipping RGMII interface\n");
        return 0;
    }

    if (available_mac_count <= 0)
    {
        printf("No available MAC addresses for Management interface(s), skipping\n");
        return 0;
    }


    for (cur_port = 0; (cur_port < mgmt_port_count) && available_mac_count-- > 0; cur_port++)
    {
        uint64_t mac = 0x0;
        /* Initialize port state array to invalid values */
        memset(port_state, 0xff, sizeof(port_state));

        dev = (struct eth_device *) malloc(sizeof(*dev));

        oct_eth_info = (octeon_eth_info_t *) malloc(sizeof(octeon_eth_info_t));

        oct_eth_info->port = cur_port;
        /* These don't apply to the management ports */
        oct_eth_info->queue = ~0;
        oct_eth_info->interface = ~0;
        oct_eth_info->initted_flag = 0;

        dprintf("Setting up mgmt eth port\n");
        sprintf (dev->name, "octmgmt%d", cur_port);

        dev->priv = (void *)oct_eth_info;
        dev->iobase = 0;
        dev->init = octeon_mgmt_eth_init;
        dev->halt = octeon_mgmt_eth_halt;
        dev->send = octeon_mgmt_eth_send;
        dev->recv = octeon_mgmt_eth_recv;
        memcpy(dev->enetaddr, mac_addr, 6);

        (*mac_inc_ptr)++;  /* increment MAC address */

        eth_register (dev);

        if (CVMX_MGMT_PORT_SUCCESS != (retval = cvmx_mgmt_port_initialize(oct_eth_info->port)))
        {
            printf("ERROR: cvmx_mgmt_port_initialize() failed: %d\n", retval);
        }
        memcpy(((char *)&mac) + 2, dev->enetaddr,6);
        cvmx_mgmt_port_set_mac(oct_eth_info->port, mac);
        cvmx_helper_link_info_t link = cvmx_mgmt_port_link_get(oct_eth_info->port);
        dprintf("Port: %d, speed: %d, up: %d, duplex: %d\n", cur_port, link.s.speed, link.s.link_up, link.s.full_duplex);
        cvmx_mgmt_port_link_set(oct_eth_info->port, link);
    }

    return card_number;

}

#endif
