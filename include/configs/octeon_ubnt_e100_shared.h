#ifndef __OCTEON_UBNT_E100_SHARED_H__
#define __OCTEON_UBNT_E100_SHARED_H__

/* UBNT E100 uses static SPD data */
#ifndef __ASSEMBLY__
extern unsigned char e100_ddr2_spd_data[];
#endif

#define UBNT_E100_DDR_CFG {{0, 0}, {e100_ddr2_spd_data, NULL}}

#define UBNT_E100_DDR_BOARD_DELAY	500
#define UBNT_E100_LMC_DELAY_CLK		0
#define UBNT_E100_LMC_DELAY_CMD  	0
#define UBNT_E100_LMC_DELAY_DQ		0

#define UBNT_E100_ODT_CFG \
    { /* odt_ena */     2, \
      /* odt_mask */    0x00000001, \
      /* odt_mask1 */   {.u64=0x0000}, \
      /* qs_dic */      3, \
      /* rodt_ctl */    0x00000001, \
      /* dic */         0 }

#define OCTEON_UBNT_E100_DDR_CONFIGURATION   \
{ \
    .dimm_config_table = {UBNT_E100_DDR_CFG, DIMM_CONFIG_TERMINATOR}, \
    .unbuffered = { \
        .ddr_board_delay = UBNT_E100_DDR_BOARD_DELAY, \
        .lmc_delay_clk = UBNT_E100_LMC_DELAY_CLK, \
        .lmc_delay_cmd = UBNT_E100_LMC_DELAY_CMD, \
        .lmc_delay_dq = UBNT_E100_LMC_DELAY_DQ, \
    }, \
    .registered = { /* dummy */ \
        .ddr_board_delay = 0, \
        .lmc_delay_clk = 0, \
        .lmc_delay_cmd = 0, \
        .lmc_delay_dq = 0, \
    }, \
    .odt_1rank_config = { UBNT_E100_ODT_CFG }, \
    .odt_2rank_config = NULL \
}

#endif   /* __OCTEON_UBNT_E100_SHARED_H__ */
