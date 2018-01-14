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


/* This file sets configuration options that are common to all Octeon
** bootloaders, and is included by each board's config header file. */

#ifndef __OCTEON_COMMON_H__
#define __OCTEON_COMMON_H__

#define LOOKUP_STEP (64*1024)
#define FAILSAFE_BASE 0x1fc00000
#define MAX_PROMPT_LEN  (64)

/* Enable buffered flash (NOR) writes, which speeds up writing considerably.
** This does not work reliably on all flash parts, so it must be tested if enabled.
*/
//#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 1

#if CONFIG_OCTEON_PCI_HOST
#ifndef OCTEON_FIRST_PCIE_BUSNO
	/* Starting PCIE bus number.  Note that this must be 1 for the IDT 
	 * PCIe switch.
	 */
#  define OCTEON_FIRST_PCIE_BUSNO		0
# endif
#endif

#endif
