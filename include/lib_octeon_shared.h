/***********************license start************************************
 * Copyright (c) 2004-2007 Cavium Networks (support@cavium.com). All rights
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


#ifndef __LIB_OCTEON_SHARED_H__
#define __LIB_OCTEON_SHARED_H__

#include "cvmx-lmcx-defs.h"

/* Structure for bootloader pci IO buffers */
typedef struct
{
    uint32_t owner;
    uint32_t len;
    char data[0];
} octeon_pci_io_buf_t;
#define BOOTLOADER_PCI_READ_BUFFER_STR_LEN  (BOOTLOADER_PCI_READ_BUFFER_SIZE - 8)
#define BOOTLOADER_PCI_WRITE_BUFFER_STR_LEN  (BOOTLOADER_PCI_WRITE_BUFFER_SIZE - 8)

#define BOOTLOADER_PCI_READ_BUFFER_OWNER_ADDR   (BOOTLOADER_PCI_READ_BUFFER_BASE + 0)
#define BOOTLOADER_PCI_READ_BUFFER_LEN_ADDR   (BOOTLOADER_PCI_READ_BUFFER_BASE + 4)
#define BOOTLOADER_PCI_READ_BUFFER_DATA_ADDR   (BOOTLOADER_PCI_READ_BUFFER_BASE + 8)

enum octeon_pci_io_buf_owner
{
    OCTEON_PCI_IO_BUF_OWNER_INVALID = 0,  /* Must be zero, set when memory cleared */
    OCTEON_PCI_IO_BUF_OWNER_OCTEON = 1,
    OCTEON_PCI_IO_BUF_OWNER_HOST = 2,
};

/* data field addresses in the DDR2 SPD eeprom */
typedef enum ddr2_spd_addrs {
    DDR2_SPD_BYTES_PROGRAMMED	= 0,
    DDR2_SPD_TOTAL_BYTES	= 1,
    DDR2_SPD_MEM_TYPE		= 2,
    DDR2_SPD_NUM_ROW_BITS	= 3,
    DDR2_SPD_NUM_COL_BITS	= 4,
    DDR2_SPD_NUM_RANKS		= 5,
    DDR2_SPD_CYCLE_CLX		= 9,
    DDR2_SPD_CONFIG_TYPE	= 11,
    DDR2_SPD_REFRESH		= 12,
    DDR2_SPD_SDRAM_WIDTH	= 13,
    DDR2_SPD_BURST_LENGTH	= 16,
    DDR2_SPD_NUM_BANKS		= 17,
    DDR2_SPD_CAS_LATENCY	= 18,
    DDR2_SPD_DIMM_TYPE		= 20,
    DDR2_SPD_CYCLE_CLX1		= 23,
    DDR2_SPD_CYCLE_CLX2		= 25,
    DDR2_SPD_TRP		= 27,
    DDR2_SPD_TRRD 		= 28,
    DDR2_SPD_TRCD 		= 29,
    DDR2_SPD_TRAS 		= 30,
    DDR2_SPD_TWR 		= 36,
    DDR2_SPD_TWTR 		= 37,
    DDR2_SPD_TRFC_EXT		= 40,
    DDR2_SPD_TRFC 		= 42,
    DDR2_SPD_CHECKSUM		= 63,
    DDR2_SPD_MFR_ID		= 64
} ddr2_spd_addr_t;

/* data field addresses in the DDR2 SPD eeprom */
typedef enum ddr3_spd_addrs {
    DDR3_SPD_BYTES_PROGRAMMED				=  0,
    DDR3_SPD_REVISION					=  1,
    DDR3_SPD_KEY_BYTE_DEVICE_TYPE			=  2,
    DDR3_SPD_KEY_BYTE_MODULE_TYPE			=  3,
    DDR3_SPD_DENSITY_BANKS				=  4, 
    DDR3_SPD_ADDRESSING_ROW_COL_BITS			=  5,
    DDR3_SPD_NOMINAL_VOLTAGE				=  6,
    DDR3_SPD_MODULE_ORGANIZATION			=  7,
    DDR3_SPD_MEMORY_BUS_WIDTH				=  8,
    DDR3_SPD_FINE_TIMEBASE_DIVIDEND_DIVISOR		=  9,
    DDR3_SPD_MEDIUM_TIMEBASE_DIVIDEND			= 10,
    DDR3_SPD_MEDIUM_TIMEBASE_DIVISOR			= 11,
    DDR3_SPD_MINIMUM_CYCLE_TIME_TCKMIN			= 12,
    DDR3_SPD_CAS_LATENCIES_LSB				= 14,
    DDR3_SPD_CAS_LATENCIES_MSB				= 15,
    DDR3_SPD_MIN_CAS_LATENCY_TAAMIN			= 16,
    DDR3_SPD_MIN_WRITE_RECOVERY_TWRMIN			= 17,
    DDR3_SPD_MIN_RAS_CAS_DELAY_TRCDMIN			= 18,
    DDR3_SPD_MIN_ROW_ACTIVE_DELAY_TRRDMIN		= 19,
    DDR3_SPD_MIN_ROW_PRECHARGE_DELAY_TRPMIN		= 20,
    DDR3_SPD_UPPER_NIBBLES_TRAS_TRC			= 21,
    DDR3_SPD_MIN_ACTIVE_PRECHARGE_LSB_TRASMIN		= 22,
    DDR3_SPD_MIN_ACTIVE_REFRESH_LSB_TRCMIN		= 23,
    DDR3_SPD_MIN_REFRESH_RECOVERY_LSB_TRFCMIN		= 24,
    DDR3_SPD_MIN_REFRESH_RECOVERY_MSB_TRFCMIN           = 25,
    DDR3_SPD_MIN_INTERNAL_WRITE_READ_CMD_TWTRMIN        = 26,
    DDR3_SPD_MIN_INTERNAL_READ_PRECHARGE_CMD_TRTPMIN    = 27,
    DDR3_SPD_UPPER_NIBBLE_TFAW                          = 28,
    DDR3_SPD_MIN_FOUR_ACTIVE_WINDOW_TFAWMIN             = 29,
    DDR3_SPD_ADDRESS_MAPPING                            = 63,
    DDR3_SPD_CYCLICAL_REDUNDANCY_CODE_LOWER_NIBBLE      = 126,
    DDR3_SPD_CYCLICAL_REDUNDANCY_CODE_UPPER_NIBBLE      = 127
} ddr3_spd_addr_t;


/*
** DRAM Module Organization
**
** Octeon:
** Octeon can be configured to use two pairs of DIMM's, lower and
** upper, providing a 128/144-bit interface or one to four DIMM's
** providing a 64/72-bit interface.  This structure contains the TWSI
** addresses used to access the DIMM's Serial Presence Detect (SPD)
** EPROMS and it also implies which DIMM socket organization is used
** on the board.  Software uses this to detect the presence of DIMM's
** plugged into the sockets, compute the total memory capacity, and
** configure DRAM controller.  All DIMM's must be identical.
**
** CN31XX:
** Octeon CN31XX can be configured to use one to four DIMM's providing
** a 64/72-bit interface.  This structure contains the TWSI addresses
** used to access the DIMM's Serial Presence Detect (SPD) EPROMS and
** it also implies which DIMM socket organization is used on the
** board.  Software uses this to detect the presence of DIMM's plugged
** into the sockets, compute the total memory capacity, and configure
** DRAM controller.  All DIMM's must be identical.
*/


/* Structure that provides DIMM information, either in the form of an SPD TWSI address,
** or a pointer to an array that contains SPD data.  One of the two fields must be valid. */
typedef struct {
    uint8_t spd_addrs[2];    /* TWSI address of SPD, 0 if not used */
    uint8_t *spd_ptrs[2];   /* pointer to SPD data array, NULL if not used */
} dimm_config_t;


/* dimm_config_t value that terminates list */
#define DIMM_CONFIG_TERMINATOR  {{0,0}, {NULL, NULL}}

typedef struct {
    uint8_t odt_ena;
    uint64_t odt_mask;
    cvmx_lmcx_modereg_params1_t odt_mask1;
    uint8_t qs_dic;
    uint64_t rodt_ctl;
    uint8_t dic;
} dimm_odt_config_t;

typedef struct {
    uint32_t ddr_board_delay;
    uint8_t lmc_delay_clk;
    uint8_t lmc_delay_cmd;
    uint8_t lmc_delay_dq; 
} ddr_delay_config_t;

/*
  The parameters below make up the custom_lmc_config data structure.
  This structure is used to customize the way that the LMC DRAM
  Controller is configured for a particular board design.

  The HRM describes LMC Read Leveling which supports automatic
  selection of per byte-lane delays.  When measuring the read delays
  the LMC configuration software sweeps through a range of settings
  for LMC0_COMP_CTL2[RODT_CTL], the Octeon II on-die-termination
  resistance and LMC0_MODEREG_PARAMS1[RTT_NOM_XX], the DRAM
  on-die-termination resistance.  The minimum and maximum parameters
  for rtt_nom_idx and rodt_ctl listed below determine the ranges of
  ODT settings used for the measurements.  Note that for rtt_nom an
  index is used into a sorted table rather than the direct csr setting
  in order to optimize the sweep.

   .min_rtt_nom_idx: 1=120ohms, 2=60ohms, 3=40ohms, 4=30ohms, 5=20ohms
   .max_rtt_nom_idx: 1=120ohms, 2=60ohms, 3=40ohms, 4=30ohms, 5=20ohms
   .min_rodt_ctl: 1=20ohms, 2=30ohms, 3=40ohms, 4=60ohms, 5=120ohms
   .max_rodt_ctl: 1=20ohms, 2=30ohms, 3=40ohms, 4=60ohms, 5=120ohms

   The settings below control the Octeon II drive strength for the CK,
   ADD/CMD, and DQ/DQS signals.  1=24ohms, 2=26.67ohms, 3=30ohms,
   4=34.3ohms, 5=40ohms, 6=48ohms, 6=60ohms.

   .dqx_ctl: Drive strength control for DDR_DQX/DDR_DQS_X_P/N drivers.
   .ck_ctl: Drive strength control for DDR_CK_X_P/DDR_DIMMX_CSX_L/DDR_DIMMX_ODT_X drivers.
   .cmd_ctl: Drive strength control for CMD/A/RESET_L/CKEX drivers.

   The LMC controller software selects the most optimal CAS Latency
   that complies with the appropriate SPD values and the frequency
   that the DRAMS are being operated.  When operating the DRAMs at
   frequencies substantially lower than their rated frequencies it
   might be necessary to limit the minimum CAS Latency the LMC
   controller software is allowed to select in order to make the DRAM
   work reliably.

   .min_cas_latency: Minimum allowed CAS Latency


   The value used for LMC0_RLEVEL_CTL[OFFSET_EN] determine how the
   read-leveling information that the Octeon II gathers is interpreted
   to determine the per-byte read delays.

   .offset_en: Value used for LMC0_RLEVEL_CTL[OFFSET_EN].
   .offset_udimm: Value used for LMC0_RLEVEL_CTL[OFFSET] for UDIMMS.
   .offset_rdimm: Value used for LMC0_RLEVEL_CTL[OFFSET] for RDIMMS.


   The LMC configuration software sweeps through a range of ODT
   settings while measuring the per-byte read delays.  During those
   measurements the software makes an assessment of the quality of the
   measurements in order to determine which measurements provide the
   most accurate delays.  The automatic settings provide the option to
   allow that same assessment to determine the most optimal RODT_CTL
   and/or RTT_NOM settings.

   The automatic approach might provide the best means to determine
   the settings used for initial poweron of a new design.  However,
   the final settings should be determined by board analysis, testing,
   and experience.

   .ddr_rtt_nom_auto: 1 means automatically set RTT_NOM value.
   .ddr_rodt_ctl_auto: 1 means automatically set RODT_CTL value.

   .rlevel_compute: Enables software interpretation of per-byte read
                    delays using the measurements collected by the
                    Octeon II rather than completely relying on the
                    Octeon II to determine the delays.  1=software
                    computation is recomended since a more complete
                    analysis is implemented in software.

   .rlevel_comp_offset: Set to 2 unless instructed differently by Cavium.
*/

typedef struct {
    uint8_t min_rtt_nom_idx;
    uint8_t max_rtt_nom_idx;
    uint8_t min_rodt_ctl;
    uint8_t max_rodt_ctl;
    uint8_t dqx_ctl;
    uint8_t ck_ctl;
    uint8_t cmd_ctl;
    uint8_t min_cas_latency;
    uint8_t offset_en;
    uint8_t offset_udimm;
    uint8_t offset_rdimm;
    uint8_t rlevel_compute;
    uint8_t ddr_rtt_nom_auto;
    uint8_t ddr_rodt_ctl_auto;
    uint8_t rlevel_comp_offset;
} ddr3_custom_config_t;

#define DDR_CFG_T_MAX_DIMMS     5
typedef struct {
    dimm_config_t dimm_config_table[DDR_CFG_T_MAX_DIMMS];
    dimm_odt_config_t odt_1rank_config[4];
    dimm_odt_config_t odt_2rank_config[4];
    ddr_delay_config_t unbuffered;
    ddr_delay_config_t registered;
    ddr3_custom_config_t custom_lmc_config;
} ddr_configuration_t;


/* Structures/defines for organizing the DDR configuration parameters
** for all boards.  These are used by both u-boot and oct-remote-boot to
** look up the DDR parameters.
*/

/* Maximum number of configurations per board. Some boards support different chips,
** and each chip needs a different configuration.  Others need different configurations
** based on revision.  Revisions MUST be listed in ascending order (configs for higher
** revisions must have higher indexes in the array.)
*/
#define MAX_DDR_CONFIGS_PER_BOARD    2 
typedef struct
{
    uint32_t chip_model;  /* OCTEON model that this config is for */
    /* Maximum board revision level that this configuration supports. */
    uint8_t max_maj_rev; /* 0 means all revs */
    uint8_t max_min_rev;
    ddr_configuration_t ddr_config[2];  /* One config for each interface */
} ddr_config_table_t;

/* Entry for the board table.  This table has an entry for each board,
** and allows the configuration to be looked up from the table.  The table
** is populated with static initializers, and is shared between u-boot and
** oct-remote-boot.
** Some fields are of interest to only u-boot or oct-remote-boot.
*/
typedef struct
{
    int board_type;
    ddr_config_table_t chip_ddr_config[MAX_DDR_CONFIGS_PER_BOARD];
    int default_ddr_clock_hz;
    int default_ddr_ref_hz;
    int default_cpu_ref_hz;
    int eeprom_addr;
    char bootloader_name[100];   /* Only used if default needs to be overridden */
} board_table_entry_t;


extern const dimm_odt_config_t disable_odt_config[];
extern const dimm_odt_config_t single_rank_odt_config[];
extern const dimm_odt_config_t dual_rank_odt_config[];

extern const board_table_entry_t octeon_board_ddr_config_table[];


int validate_dimm(dimm_config_t dimm_config, int dimm);
int validate_spd_checksum(int twsi_addr, int silent);

int init_octeon_dram_interface(uint32_t cpu_id,
                               const ddr_configuration_t *ddr_configuration,
                               uint32_t ddr_hertz,
                               uint32_t cpu_hertz,
                               uint32_t ddr_ref_hertz,
                               int board_type,
                               int board_rev_maj,
                               int board_rev_min,
                               int ddr_interface_num,
                               uint32_t ddr_interface_mask);
int initialize_ddr_clock(uint32_t cpu_id,
                         const ddr_configuration_t *ddr_configuration,
                         uint32_t cpu_hertz,
                         uint32_t ddr_hertz,
                         uint32_t ddr_ref_hertz,
                         int ddr_interface_num,
                         uint32_t ddr_interface_mask);
uint32_t measure_octeon_ddr_clock(uint32_t cpu_id,
                                  const ddr_configuration_t *ddr_configuration,
                                  uint32_t cpu_hertz,
                                  uint32_t ddr_hertz,
                                  uint32_t ddr_ref_hertz,
                                  int ddr_interface_num,
                                  uint32_t ddr_interface_mask);

int octeon_ddr_initialize(uint32_t cpu_id,
                          uint32_t cpu_hertz,
                          uint32_t ddr_hertz,
                          uint32_t ddr_ref_hertz,
                          uint32_t ddr_interface_mask,
                          const ddr_configuration_t *ddr_config_array,
                          uint32_t *measured_ddr_hertz,
                          int board_type,
                          int board_rev_maj,
                          int board_rev_min);

int octeon_dfm_initialize(void);

int twsii_mcu_read(uint8_t twsii_addr);


/* Look up the ddr configuration information based on chip and board type */
const ddr_configuration_t *lookup_ddr_config_structure(uint32_t cpu_id, int board_type, int board_rev_maj, int board_rev_min, uint32_t *interface_mask);
const board_table_entry_t *lookup_board_table_entry(int board_type, char *board_name);

/* Allow legacy code to encode bus number in the upper bits of the address
** These are only supported in read_spd() */
#define OCTEON_TWSI_BUS_IN_ADDR_BIT       12
#define OCTEON_TWSI_BUS_IN_ADDR_MASK      (1 << OCTEON_TWSI_BUS_IN_ADDR_BIT)

int octeon_tlv_get_tuple_addr(uint8_t dev_addr, uint16_t type, uint8_t major_version, uint8_t *eeprom_buf, uint32_t buf_len);

int  octeon_tlv_eeprom_get_next_tuple(uint8_t dev_addr, uint16_t addr, uint8_t *buf_ptr, uint32_t buf_len);

#ifdef ENABLE_BOARD_DEBUG
void gpio_status(int data);
#endif

/* Divide and round results to the nearest integer. */
uint64_t divide_nint(uint64_t dividend, uint64_t divisor);

/* Divide and round results up to the next higher integer. */
uint64_t divide_roundup(uint64_t dividend, uint64_t divisor);


#define rttnom_none   0         /* Rtt_Nom disabled */
#define rttnom_60ohm  1         /* RZQ/4  = 240/4  =  60 ohms */
#define rttnom_120ohm 2         /* RZQ/2  = 240/2  = 120 ohms */
#define rttnom_40ohm  3         /* RZQ/6  = 240/6  =  40 ohms */
#define rttnom_20ohm  4         /* RZQ/12 = 240/12 =  20 ohms */
#define rttnom_30ohm  5         /* RZQ/8  = 240/8  =  30 ohms */
#define rttnom_rsrv1  6         /* Reserved */
#define rttnom_rsrv2  7         /* Reserved */

#define rttwr_none    0         /* Dynamic ODT off */
#define rttwr_60ohm   1         /* RZQ/4  = 240/4  =  60 ohms */
#define rttwr_120ohm  2         /* RZQ/2  = 240/2  = 120 ohms */
#define rttwr_rsrv1   3         /* Reserved */


/*
** Generic copy of these parameters. Modify and enable the local
** version in include/configs for board specific customizations.
*/

/* LMC0_MODEREG_PARAMS1 */
#define MODEREG_PARAMS1_1RANK_1SLOT             \
    { .cn63xx = { .pasr_00      = 0,            \
                  .asr_00       = 0,            \
                  .srt_00       = 0,            \
                  .rtt_wr_00    = 0,            \
                  .dic_00       = 0,            \
                  .rtt_nom_00   = rttnom_40ohm, \
                  .pasr_01      = 0,            \
                  .asr_01       = 0,            \
                  .srt_01       = 0,            \
                  .rtt_wr_01    = 0,            \
                  .dic_01       = 0,            \
                  .rtt_nom_01   = 0,            \
                  .pasr_10      = 0,            \
                  .asr_10       = 0,            \
                  .srt_10       = 0,            \
                  .rtt_wr_10    = 0,            \
                  .dic_10       = 0,            \
                  .rtt_nom_10   = 0,            \
                  .pasr_11      = 0,            \
                  .asr_11       = 0,            \
                  .srt_11       = 0,            \
                  .rtt_wr_11    = 0,            \
                  .dic_11       = 0,            \
                  .rtt_nom_11   = 0,            \
        }                                       \
    }

#define MODEREG_PARAMS1_1RANK_2SLOT             \
    { .cn63xx = { .pasr_00      = 0,            \
                  .asr_00       = 0,            \
                  .srt_00       = 0,            \
                  .rtt_wr_00    = rttwr_120ohm, \
                  .dic_00       = 0,            \
                  .rtt_nom_00   = rttnom_40ohm, \
                  .pasr_01      = 0,            \
                  .asr_01       = 0,            \
                  .srt_01       = 0,            \
                  .rtt_wr_01    = 0,            \
                  .dic_01       = 0,            \
                  .rtt_nom_01   = 0,            \
                  .pasr_10      = 0,            \
                  .asr_10       = 0,            \
                  .srt_10       = 0,            \
                  .rtt_wr_10    = rttwr_120ohm, \
                  .dic_10       = 0,            \
                  .rtt_nom_10   = rttnom_40ohm, \
                  .pasr_11      = 0,            \
                  .asr_11       = 0,            \
                  .srt_11       = 0,            \
                  .rtt_wr_11    = 0,            \
                  .dic_11       = 0,            \
                  .rtt_nom_11   = 0             \
        }                                       \
    }

#define MODEREG_PARAMS1_2RANK_1SLOT             \
    { .cn63xx = { .pasr_00      = 0,            \
                  .asr_00       = 0,            \
                  .srt_00       = 0,            \
                  .rtt_wr_00    = 0,            \
                  .dic_00       = 0,            \
                  .rtt_nom_00   = rttnom_40ohm, \
                  .pasr_01      = 0,            \
                  .asr_01       = 0,            \
                  .srt_01       = 0,            \
                  .rtt_wr_01    = 0,            \
                  .dic_01       = 0,            \
                  .rtt_nom_01   = 0,            \
                  .pasr_10      = 0,            \
                  .asr_10       = 0,            \
                  .srt_10       = 0,            \
                  .rtt_wr_10    = 0,            \
                  .dic_10       = 0,            \
                  .rtt_nom_10   = 0,            \
                  .pasr_11      = 0,            \
                  .asr_11       = 0,            \
                  .srt_11       = 0,            \
                  .rtt_wr_11    = 0,            \
                  .dic_11       = 0,            \
                  .rtt_nom_11   = 0,            \
        }                                       \
    }

#define MODEREG_PARAMS1_2RANK_2SLOT                     \
    { .cn63xx = { .pasr_00      = 0,                    \
                  .asr_00       = 0,                    \
                  .srt_00       = 0,                    \
                  .rtt_wr_00    = 0,                    \
                  .dic_00       = 1,                    \
                  .rtt_nom_00   = rttnom_120ohm,        \
                  .pasr_01      = 0,                    \
                  .asr_01       = 0,                    \
                  .srt_01       = 0,                    \
                  .rtt_wr_01    = 0,                    \
                  .dic_01       = 1,                    \
                  .rtt_nom_01   = rttnom_40ohm,         \
                  .pasr_10      = 0,                    \
                  .asr_10       = 0,                    \
                  .srt_10       = 0,                    \
                  .rtt_wr_10    = 0,                    \
                  .dic_10       = 1,                    \
                  .rtt_nom_10   = rttnom_120ohm,        \
                  .pasr_11      = 0,                    \
                  .asr_11       = 0,                    \
                  .srt_11       = 0,                    \
                  .rtt_wr_11    = 0,                    \
                  .dic_11       = 1,                    \
                  .rtt_nom_11   = rttnom_40ohm,         \
        }                                               \
    }

#define CN31XX_DRAM_ODT_1RANK_CONFIGURATION \
    /* DIMMS   ODT_ENA  WODT_CTL0       WODT_CTL1      QS_DIC RODT_CTL DIC */ \
    /* =====   ======= ============ ================== ====== ======== === */ \
    /*   1 */ {   0,    0x00000100, {.u64=0x00000000},    1,   0x0000,  0  },  \
    /*   2 */ {   0,    0x01000400, {.u64=0x00000000},    1,   0x0000,  0  },  \
    /*   3 */ {   0,    0x01000400, {.u64=0x00000400},    2,   0x0000,  0  },  \
    /*   4 */ {   0,    0x01000400, {.u64=0x04000400},    2,   0x0000,  0  }

#define CN31XX_DRAM_ODT_2RANK_CONFIGURATION \
    /* DIMMS   ODT_ENA  WODT_CTL0     WODT_CTL1        QS_DIC RODT_CTL DIC */ \
    /* =====   ======= ============ ================== ====== ======== === */ \
    /*   1 */ {   0,    0x00000101, {.u64=0x00000000},    1,   0x0000,  0  },  \
    /*   2 */ {   0,    0x01010404, {.u64=0x00000000},    1,   0x0000,  0  },  \
    /*   3 */ {   0,    0x01010404, {.u64=0x00000404},    2,   0x0000,  0  },  \
    /*   4 */ {   0,    0x01010404, {.u64=0x04040404},    2,   0x0000,  0  }

/* CN30xx is the same as CN31xx */
#define CN30XX_DRAM_ODT_1RANK_CONFIGURATION CN31XX_DRAM_ODT_1RANK_CONFIGURATION
#define CN30XX_DRAM_ODT_2RANK_CONFIGURATION CN31XX_DRAM_ODT_2RANK_CONFIGURATION

#define CN38XX_DRAM_ODT_1RANK_CONFIGURATION \
    /* DIMMS   ODT_ENA LMC_ODT_CTL  reserved       QS_DIC RODT_CTL DIC */ \
    /* =====   ======= ============ ============== ====== ======== === */ \
    /*   1 */ {   0,    0x00000001, {.u64=0x0000},    1,   0x0000,  0  },  \
    /*   2 */ {   0,    0x00010001, {.u64=0x0000},    2,   0x0000,  0  },  \
    /*   3 */ {   0,    0x01040104, {.u64=0x0000},    2,   0x0000,  0  },  \
    /*   4 */ {   0,    0x01040104, {.u64=0x0000},    2,   0x0000,  0  }

#define CN38XX_DRAM_ODT_2RANK_CONFIGURATION \
    /* DIMMS   ODT_ENA LMC_ODT_CTL  reserved       QS_DIC RODT_CTL DIC */ \
    /* =====   ======= ============ ============== ====== ======== === */ \
    /*   1 */ {   0,    0x00000011, {.u64=0x0000},    1,   0x0000,  0  },  \
    /*   2 */ {   0,    0x00110011, {.u64=0x0000},    2,   0x0000,  0  },  \
    /*   3 */ {   0,    0x11441144, {.u64=0x0000},    3,   0x0000,  0  },  \
    /*   4 */ {   0,    0x11441144, {.u64=0x0000},    3,   0x0000,  0  }

/* Note: CN58XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN58XX_DRAM_ODT_1RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA LMC_ODT_CTL  reserved       QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ============== ====== ======== === */ \
    /*   1 */ {   2,    0x00000001,  {.u64=0x0000},    2,   0x00000001,  0  },  \
    /*   2 */ {   2,    0x00010001,  {.u64=0x0000},    2,   0x00010001,  0  },  \
    /*   3 */ {   2,    0x01040104,  {.u64=0x0000},    2,   0x01040104,  0  },  \
    /*   4 */ {   2,    0x01040104,  {.u64=0x0000},    2,   0x01040104,  0  }

/* Note: CN58XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN58XX_DRAM_ODT_2RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA LMC_ODT_CTL  reserved       QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ============== ====== ======== === */ \
    /*   1 */ {   2,    0x00000011,  {.u64=0x0000},    2,   0x00000011,  0  },  \
    /*   2 */ {   2,    0x00110011,  {.u64=0x0000},    2,   0x00110011,  0  },  \
    /*   3 */ {   2,    0x11441144,  {.u64=0x0000},    2,   0x11441144,  0  },  \
    /*   4 */ {   2,    0x11441144,  {.u64=0x0000},    2,   0x11441144,  0  }

/* Note: CN50XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN50XX_DRAM_ODT_1RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA LMC_ODT_CTL  reserved       QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ============== ====== ======== === */ \
    /*   1 */ {   2,    0x00000001,  {.u64=0x0000},    1,   0x0000,  0  },  \
    /*   2 */ {   2,    0x00010001,  {.u64=0x0000},    2,   0x0000,  0  },  \
    /*   3 */ {   2,    0x01040104,  {.u64=0x0000},    2,   0x0000,  0  },  \
    /*   4 */ {   2,    0x01040104,  {.u64=0x0000},    2,   0x0000,  0  }

/* Note: CN50XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN50XX_DRAM_ODT_2RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA LMC_ODT_CTL  reserved       QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ============== ====== ======== === */ \
    /*   1 */ {   2,    0x00000011,  {.u64=0x0000},    1,   0x0000,  0  },  \
    /*   2 */ {   2,    0x00110011,  {.u64=0x0000},    2,   0x0000,  0  },  \
    /*   3 */ {   2,    0x11441144,  {.u64=0x0000},    3,   0x0000,  0  },  \
    /*   4 */ {   2,    0x11441144,  {.u64=0x0000},    3,   0x0000,  0  }

/* Note: CN56XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN56XX_DRAM_ODT_1RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA  WODT_CTL0     WODT_CTL1        QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ================== ====== ======== === */ \
    /*   1 */ {   2,     0x00000100, {.u64=0x00000000},    3,   0x0001,  0  },  \
    /*   2 */ {   2,     0x01000400, {.u64=0x00000000},    3,   0x0104,  0  },  \
    /*   3 */ {   2,     0x01000400, {.u64=0x00000400},    3,   0x0104,  0  },  \
    /*   4 */ {   2,     0x01000400, {.u64=0x04000400},    3,   0x0104,  0  }

/* Note: CN56XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN56XX_DRAM_ODT_2RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA  WODT_CTL0     WODT_CTL1        QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ================== ====== ======== === */ \
    /*   1 */ {   2,     0x00000101, {.u64=0x00000000},    3,   0x0011,  0  },  \
    /*   2 */ {   2,     0x01010404, {.u64=0x00000000},    3,   0x1144,  0  },  \
    /*   3 */ {   2,     0x01010404, {.u64=0x00000404},    3,   0x1144,  0  },  \
    /*   4 */ {   2,     0x01010404, {.u64=0x04040404},    3,   0x1144,  0  }

/* Note: CN52XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN52XX_DRAM_ODT_1RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA  WODT_CTL0     WODT_CTL1        QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ================== ====== ======== === */ \
    /*   1 */ {   2,     0x00000100, {.u64=0x00000000},    3,   0x0001,  0  },  \
    /*   2 */ {   2,     0x01000400, {.u64=0x00000000},    3,   0x0104,  0  },  \
    /*   3 */ {   2,     0x01000400, {.u64=0x00000400},    3,   0x0104,  0  },  \
    /*   4 */ {   2,     0x01000400, {.u64=0x04000400},    3,   0x0104,  0  }

/* Note: CN52XX RODT_ENA 0 = disabled, 1 = Weak Read ODT, 2 = Strong Read ODT */
#define CN52XX_DRAM_ODT_2RANK_CONFIGURATION \
    /* DIMMS   RODT_ENA  WODT_CTL0     WODT_CTL1        QS_DIC RODT_CTL DIC */ \
    /* =====   ======== ============ ================== ====== ======== === */ \
    /*   1 */ {   2,     0x00000101, {.u64=0x00000000},    3,   0x0011,  0  },  \
    /*   2 */ {   2,     0x01010404, {.u64=0x00000000},    3,   0x1144,  0  },  \
    /*   3 */ {   2,     0x01010404, {.u64=0x00000404},    3,   0x1144,  0  },  \
    /*   4 */ {   2,     0x01010404, {.u64=0x04040404},    3,   0x1144,  0  }

#define CN63XX_DRAM_ODT_1RANK_CONFIGURATION \
    /* DIMMS   reserved  WODT_MASK     LMCX_MODEREG_PARAMS1         RODT_CTL    RODT_MASK    reserved */ \
    /* =====   ======== ============== ============================ ========= ============== ======== */ \
    /*   1 */ {   0,    0x00000001ULL, MODEREG_PARAMS1_1RANK_1SLOT,    3,     0x00000000ULL,  0  },      \
    /*   2 */ {   0,    0x00050005ULL, MODEREG_PARAMS1_1RANK_2SLOT,    3,     0x00010004ULL,  0  }

#define CN63XX_DRAM_ODT_2RANK_CONFIGURATION \
    /* DIMMS   reserved  WODT_MASK     LMCX_MODEREG_PARAMS1         RODT_CTL    RODT_MASK    reserved */ \
    /* =====   ======== ============== ============================ ========= ============== ======== */ \
    /*   1 */ {   0,    0x00000101ULL, MODEREG_PARAMS1_2RANK_1SLOT,    3,     0x00000000ULL,    0  },    \
    /*   2 */ {   0,    0x06060909ULL, MODEREG_PARAMS1_2RANK_2SLOT,    3,     0x02020808ULL,    0  }

#endif  /*  __LIB_OCTEON_SHARED_H__  */
