
/***********************license start************************************
 * Copyright (c) 2003-2007 Cavium Networks (support@cavium.com). All rights
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
 * @file 
 *
 * $Id: cmd_qlm.c 52004 2010-08-18 00:53:21Z cchavva $
 * 
 */

#include <common.h>
#include <command.h>
#include <exports.h>
#include <linux/ctype.h>
#include <net.h>
#include <elf.h>
#include <asm/mipsregs.h>
#include <asm/processor.h>
#include "cvmx-helper-jtag.h"
#include "octeon_boot.h"

//#define CONFIG_CMD_QLM 1
#ifdef CONFIG_CMD_QLM


#define JTAG_CLOCK          25          /* Mhz */
#define QLM_LANES           4           /* Number of lanes for each QLM */
#define OCTEON_REF_CLOCK    50000000    /* 50Mhz */

typedef struct
{
    const char *name;
    int stop_bit;
    int start_bit;
} field_info_t;

const int num_qlm_cn56xx = 4;
const int chain_length_cn56xx = 268;
const field_info_t field_info_cn56xx[] =
{
    {"prbs_error_count",        267, 220},       // BIST/PRBS error count (only valid if pbrs_lock asserted)
    {"prbs_unlock_count",       219, 212},       // BIST/PRBS unlock count (only valid if pbrs_lock asserted)
    {"prbs_locked",             211, 211},       // BIST/PRBS lock (asserted after QLM achieves lock)
    {"reset_prbs",              210, 210},       // BIST/PRBS reset (write 0 to reset)
    {"run_prbs",                209, 209},       // run PRBS test pattern
    {"run_bist",                208, 208},       // run bist (May only work for PCIe ?)

    {"unknown",                 207, 202},       //

    {"biasdrvsel",              201,        199},           //   assign biasdrvsel          = fus_cfg_reg[201:199] ^ jtg_cfg_reg[201:199] ^ ((pi_qlm_cfg == 2'h0) ? 3'h4 : (pi_qlm_cfg == 2'h2) ? 3'h7 : 3'h2);
    {"biasbuffsel",             198,        196},           //   assign biasbuffsel         = fus_cfg_reg[198:196] ^ jtg_cfg_reg[198:196] ^ 3'h4;
    {"tcoeff",                  195,        192},           //   assign tcoeff              = fus_cfg_reg[195:192] ^ jtg_cfg_reg[195:192] ^ (pi_qlm_cfg[1] ? 4'h0 : 4'hc);

    {"mb5000",                  181,        181},           //   assign mb5000              = fus_cfg_reg[181]     ^ jtg_cfg_reg[181]     ^ 1'h0;
    {"interpbw",                180,        176},           //   assign interpbw            = fus_cfg_reg[180:176] ^ jtg_cfg_reg[180:176] ^ ((qlm_spd == 2'h0) ? 5'h1f : (qlm_spd == 2'h1) ? 5'h10 : 5'h0);
    {"mb",                      175,        172},           //   assign mb                  = fus_cfg_reg[175:172] ^ jtg_cfg_reg[175:172] ^ 4'h0;
    {"bwoff",                   171,        160},           //   assign bwoff               = fus_cfg_reg[171:160] ^ jtg_cfg_reg[171:160] ^ 12'h0;

    {"bg_ref_sel",              153,        153},           //   assign bg_ref_sel          = fus_cfg_reg[153]     ^ jtg_cfg_reg[153]     ^ 1'h0;
    {"div2en",                  152,        152},           //   assign div2en              = fus_cfg_reg[152]     ^ jtg_cfg_reg[152]     ^ 1'h0;
    {"trimen",                  151,        150},           //   assign trimen              = fus_cfg_reg[151:150] ^ jtg_cfg_reg[151:150] ^ 2'h0;
    {"clkr",                    149,        144},           //   assign clkr                = fus_cfg_reg[149:144] ^ jtg_cfg_reg[149:144] ^ 6'h0;
    {"clkf",                    143,        132},           //   assign clkf                = fus_cfg_reg[143:132] ^ jtg_cfg_reg[143:132] ^ 12'h18;
    {"bwadj",                   131,        120},           //   assign bwadj               = fus_cfg_reg[131:120] ^ jtg_cfg_reg[131:120] ^ 12'h30;
    {"shlpbck",                 119,        118},           //   assign shlpbck             = fus_cfg_reg[119:118] ^ jtg_cfg_reg[119:118] ^ 2'h0;
    {"serdes_pll_byp",          117,        117},           //   assign serdes_pll_byp      = fus_cfg_reg[117]     ^ jtg_cfg_reg[117]     ^ 1'h0;
    {"ic50dac",                 116,        112},           //   assign ic50dac             = fus_cfg_reg[116:112] ^ jtg_cfg_reg[116:112] ^ 5'h11;
    {"sl_posedge_sample",       111,        111},           //   assign sl_posedge_sample   = fus_cfg_reg[111]     ^ jtg_cfg_reg[111]     ^ 1'h0;
    {"sl_enable",               110,        110},           //   assign sl_enable           = fus_cfg_reg[110]     ^ jtg_cfg_reg[110]     ^ 1'h0;
    {"rx_rout_comp_bypass",     109,        109},           //   assign rx_rout_comp_bypass = fus_cfg_reg[109]     ^ jtg_cfg_reg[109]     ^ 1'h0;
    {"ir50dac",                 108,        104},           //   assign ir50dac             = fus_cfg_reg[108:104] ^ jtg_cfg_reg[108:104] ^ 5'h11;
    {"rx_res_offset",           103,        100},           //   assign rx_res_offset       = fus_cfg_reg[103:100] ^ jtg_cfg_reg[103:100] ^ 4'h2;
    {"rx_rout_comp_value",      99,         96},            //   assign rx_rout_comp_value  = fus_cfg_reg[99:96]   ^ jtg_cfg_reg[99:96]   ^ 4'h7;
    {"tx_rout_comp_value",      95,         92},            //   assign tx_rout_comp_value  = fus_cfg_reg[95:92]   ^ jtg_cfg_reg[95:92]   ^ 4'h7;
    {"tx_res_offset",           91,         88},            //   assign tx_res_offset       = fus_cfg_reg[91:88]   ^ jtg_cfg_reg[91:88]   ^ 4'h1;
    {"tx_rout_comp_bypass",     87,         87},            //   assign tx_rout_comp_bypass = fus_cfg_reg[87]      ^ jtg_cfg_reg[87]      ^ 1'h0;
    {"idle_dac",                86,         84},            //   assign idle_dac            = fus_cfg_reg[86:84]   ^ jtg_cfg_reg[86:84]   ^ 3'h4;
    {"hyst_en",                 83,         83},            //   assign hyst_en             = fus_cfg_reg[83]      ^ jtg_cfg_reg[83]      ^ 1'h1;
    {"rndt",                    82,         82},            //   assign rndt                = fus_cfg_reg[82]      ^ jtg_cfg_reg[82]      ^ 1'h0;

    {"cfg_tx_com",              79,         79},            //   CN52XX cfg_tx_com     = fus_cfg_reg[79] ^ jtg_cfg_reg[79] ^ 1'h0;
    {"cfg_cdr_errcor",          78,         78},            //   CN52XX cfg_cdr_errcor = fus_cfg_reg[78] ^ jtg_cfg_reg[78] ^ 1'h0;
    {"cfg_cdr_secord",          77,         77},            //   CN52XX cfg_cdr_secord = fus_cfg_reg[77] ^ jtg_cfg_reg[77] ^ 1'h1;
    {"cfg_cdr_rotate",          76,         76},            //   assign cfg_cdr_rotate      = fus_cfg_reg[76]      ^ jtg_cfg_reg[76]      ^ 1'h0;
    {"cfg_cdr_rqoffs",          75,         68},            //   assign cfg_cdr_rqoffs      = fus_cfg_reg[75:68]   ^ jtg_cfg_reg[75:68]   ^ 8'h40;
    {"cfg_cdr_incx",            67,         64},            //   assign cfg_cdr_incx        = fus_cfg_reg[67:64]   ^ jtg_cfg_reg[67:64]   ^ 4'h2;
    {"cfg_cdr_state",           63,         56},            //   assign cfg_cdr_state       = fus_cfg_reg[63:56]   ^ jtg_cfg_reg[63:56]   ^ 8'h0;
    {"cfg_cdr_bypass",          55,         55},            //   assign cfg_cdr_bypass      = fus_cfg_reg[55]      ^ jtg_cfg_reg[55]      ^ 1'h0;
    {"cfg_tx_byp",              54,         54},            //   assign cfg_tx_byp          = fus_cfg_reg[54]      ^ jtg_cfg_reg[54]      ^ 1'h0;
    {"cfg_tx_val",              53,         44},            //   assign cfg_tx_val          = fus_cfg_reg[53:44]   ^ jtg_cfg_reg[53:44]   ^ 10'h0;
    {"cfg_rx_pol_set",          43,         43},            //   assign cfg_rx_pol_set      = fus_cfg_reg[43]      ^ jtg_cfg_reg[43]      ^ 1'h0;
    {"cfg_rx_pol_clr",          42,         42},            //   assign cfg_rx_pol_clr      = fus_cfg_reg[42]      ^ jtg_cfg_reg[42]      ^ 1'h0;
    {"cfg_cdr_bw_ctl",          41,         40},            //   assign cfg_cdr_bw_ctl      = fus_cfg_reg[41:40]   ^ jtg_cfg_reg[41:40]   ^ 2'h0;
    {"cfg_rst_n_set",           39,         39},            //   assign cfg_rst_n_set       = fus_cfg_reg[39]      ^ jtg_cfg_reg[39]      ^ 1'h0;
    {"cfg_rst_n_clr",           38,         38},            //   assign cfg_rst_n_clr       = fus_cfg_reg[38]      ^ jtg_cfg_reg[38]      ^ 1'h0;
    {"cfg_tx_clk2",             37,         37},            //   assign cfg_tx_clk2         = fus_cfg_reg[37]      ^ jtg_cfg_reg[37]      ^ 1'h0;
    {"cfg_tx_clk1",             36,         36},            //   assign cfg_tx_clk1         = fus_cfg_reg[36]      ^ jtg_cfg_reg[36]      ^ 1'h0;
    {"cfg_tx_pol_set",          35,         35},            //   assign cfg_tx_pol_set      = fus_cfg_reg[35]      ^ jtg_cfg_reg[35]      ^ 1'h0;
    {"cfg_tx_pol_clr",          34,         34},            //   assign cfg_tx_pol_clr      = fus_cfg_reg[34]      ^ jtg_cfg_reg[34]      ^ 1'h0;
    {"cfg_tx_one",              33,         33},            //   assign cfg_tx_one          = fus_cfg_reg[33]      ^ jtg_cfg_reg[33]      ^ 1'h0;
    {"cfg_tx_zero",             32,         32},            //   assign cfg_tx_zero         = fus_cfg_reg[32]      ^ jtg_cfg_reg[32]      ^ 1'h0;
    {"cfg_rxd_wait",            31,         28},            //   assign cfg_rxd_wait        = fus_cfg_reg[31:28]   ^ jtg_cfg_reg[31:28]   ^ 4'h3;
    {"cfg_rxd_short",           27,         27},            //   assign cfg_rxd_short       = fus_cfg_reg[27]      ^ jtg_cfg_reg[27]      ^ 1'h0;
    {"cfg_rxd_set",             26,         26},            //   assign cfg_rxd_set         = fus_cfg_reg[26]      ^ jtg_cfg_reg[26]      ^ 1'h0;
    {"cfg_rxd_clr",             25,         25},            //   assign cfg_rxd_clr         = fus_cfg_reg[25]      ^ jtg_cfg_reg[25]      ^ 1'h0;
    {"cfg_loopback",            24,         24},            //   assign cfg_loopback        = fus_cfg_reg[24]      ^ jtg_cfg_reg[24]      ^ 1'h0;
    {"cfg_tx_idle_set",         23,         23},            //   assign cfg_tx_idle_set     = fus_cfg_reg[23]      ^ jtg_cfg_reg[23]      ^ 1'h0;
    {"cfg_tx_idle_clr",         22,         22},            //   assign cfg_tx_idle_clr     = fus_cfg_reg[22]      ^ jtg_cfg_reg[22]      ^ 1'h0;
    {"cfg_rx_idle_set",         21,         21},            //   assign cfg_rx_idle_set     = fus_cfg_reg[21]      ^ jtg_cfg_reg[21]      ^ 1'h0;
    {"cfg_rx_idle_clr",         20,         20},            //   assign cfg_rx_idle_clr     = fus_cfg_reg[20]      ^ jtg_cfg_reg[20]      ^ 1'h0;
    {"cfg_rx_idle_thr",         19,         16},            //   assign cfg_rx_idle_thr     = fus_cfg_reg[19:16]   ^ jtg_cfg_reg[19:16]   ^ 4'h0;
    {"cfg_com_thr",             15,         12},            //   assign cfg_com_thr         = fus_cfg_reg[15:12]   ^ jtg_cfg_reg[15:12]   ^ 4'h3;
    {"cfg_rx_offset",           11,         8},             //   assign cfg_rx_offset       = fus_cfg_reg[11:8]    ^ jtg_cfg_reg[11:8]    ^ 4'h4;
    {"cfg_skp_max",             7,          4},             //   assign cfg_skp_max         = fus_cfg_reg[7:4]     ^ jtg_cfg_reg[7:4]     ^ 4'hc;
    {"cfg_skp_min",             3,          0},             //   assign cfg_skp_min         = fus_cfg_reg[3:0]     ^ jtg_cfg_reg[3:0]     ^ 4'h4;
    {NULL,                      -1,         -1}
};


const int num_qlm_cn52xx = 2;
const int chain_length_cn52xx = 268;
const field_info_t field_info_cn52xx[] =
{
    {"prbs_error_count",        267, 220},       // BIST/PRBS error count (only valid if pbrs_lock asserted)
    {"prbs_unlock_count",       219, 212},       // BIST/PRBS unlock count (only valid if pbrs_lock asserted)
    {"prbs_locked",             211, 211},       // BIST/PRBS lock (asserted after QLM achieves lock)
    {"reset_prbs",              210, 210},       // BIST/PRBS reset (write 0 to reset)
    {"run_prbs",                209, 209},       // run PRBS test pattern
    {"run_bist",                208, 208},       // run bist (May only work for PCIe ?)

    {"unknown",                 207, 202},       //

    {"biasdrvsel",              201,        199},           //   assign biasdrvsel          = fus_cfg_reg[201:199] ^ jtg_cfg_reg[201:199] ^ ((pi_qlm_cfg == 2'h0) ? 3'h4 : (pi_qlm_cfg == 2'h2) ? 3'h7 : 3'h2);
    {"biasbuffsel",             198,        196},           //   assign biasbuffsel         = fus_cfg_reg[198:196] ^ jtg_cfg_reg[198:196] ^ 3'h4;
    {"tcoeff",                  195,        192},           //   assign tcoeff              = fus_cfg_reg[195:192] ^ jtg_cfg_reg[195:192] ^ (pi_qlm_cfg[1] ? 4'h0 : 4'hc);

    {"mb5000",                  181,        181},           //   assign mb5000              = fus_cfg_reg[181]     ^ jtg_cfg_reg[181]     ^ 1'h0;
    {"interpbw",                180,        176},           //   assign interpbw            = fus_cfg_reg[180:176] ^ jtg_cfg_reg[180:176] ^ ((qlm_spd == 2'h0) ? 5'h1f : (qlm_spd == 2'h1) ? 5'h10 : 5'h0);
    {"mb",                      175,        172},           //   assign mb                  = fus_cfg_reg[175:172] ^ jtg_cfg_reg[175:172] ^ 4'h0;
    {"bwoff",                   171,        160},           //   assign bwoff               = fus_cfg_reg[171:160] ^ jtg_cfg_reg[171:160] ^ 12'h0;

    {"bg_ref_sel",              153,        153},           //   assign bg_ref_sel          = fus_cfg_reg[153]     ^ jtg_cfg_reg[153]     ^ 1'h0;
    {"div2en",                  152,        152},           //   assign div2en              = fus_cfg_reg[152]     ^ jtg_cfg_reg[152]     ^ 1'h0;
    {"trimen",                  151,        150},           //   assign trimen              = fus_cfg_reg[151:150] ^ jtg_cfg_reg[151:150] ^ 2'h0;
    {"clkr",                    149,        144},           //   assign clkr                = fus_cfg_reg[149:144] ^ jtg_cfg_reg[149:144] ^ 6'h0;
    {"clkf",                    143,        132},           //   assign clkf                = fus_cfg_reg[143:132] ^ jtg_cfg_reg[143:132] ^ 12'h18;
    {"bwadj",                   131,        120},           //   assign bwadj               = fus_cfg_reg[131:120] ^ jtg_cfg_reg[131:120] ^ 12'h30;
    {"shlpbck",                 119,        118},           //   assign shlpbck             = fus_cfg_reg[119:118] ^ jtg_cfg_reg[119:118] ^ 2'h0;
    {"serdes_pll_byp",          117,        117},           //   assign serdes_pll_byp      = fus_cfg_reg[117]     ^ jtg_cfg_reg[117]     ^ 1'h0;
    {"ic50dac",                 116,        112},           //   assign ic50dac             = fus_cfg_reg[116:112] ^ jtg_cfg_reg[116:112] ^ 5'h11;
    {"sl_posedge_sample",       111,        111},           //   assign sl_posedge_sample   = fus_cfg_reg[111]     ^ jtg_cfg_reg[111]     ^ 1'h0;
    {"sl_enable",               110,        110},           //   assign sl_enable           = fus_cfg_reg[110]     ^ jtg_cfg_reg[110]     ^ 1'h0;
    {"rx_rout_comp_bypass",     109,        109},           //   assign rx_rout_comp_bypass = fus_cfg_reg[109]     ^ jtg_cfg_reg[109]     ^ 1'h0;
    {"ir50dac",                 108,        104},           //   assign ir50dac             = fus_cfg_reg[108:104] ^ jtg_cfg_reg[108:104] ^ 5'h11;
    {"rx_res_offset",           103,        100},           //   assign rx_res_offset       = fus_cfg_reg[103:100] ^ jtg_cfg_reg[103:100] ^ 4'h2;
    {"rx_rout_comp_value",      99,         96},            //   assign rx_rout_comp_value  = fus_cfg_reg[99:96]   ^ jtg_cfg_reg[99:96]   ^ 4'h7;
    {"tx_rout_comp_value",      95,         92},            //   assign tx_rout_comp_value  = fus_cfg_reg[95:92]   ^ jtg_cfg_reg[95:92]   ^ 4'h7;
    {"tx_res_offset",           91,         88},            //   assign tx_res_offset       = fus_cfg_reg[91:88]   ^ jtg_cfg_reg[91:88]   ^ 4'h1;
    {"tx_rout_comp_bypass",     87,         87},            //   assign tx_rout_comp_bypass = fus_cfg_reg[87]      ^ jtg_cfg_reg[87]      ^ 1'h0;
    {"idle_dac",                86,         84},            //   assign idle_dac            = fus_cfg_reg[86:84]   ^ jtg_cfg_reg[86:84]   ^ 3'h4;
    {"hyst_en",                 83,         83},            //   assign hyst_en             = fus_cfg_reg[83]      ^ jtg_cfg_reg[83]      ^ 1'h1;
    {"rndt",                    82,         82},            //   assign rndt                = fus_cfg_reg[82]      ^ jtg_cfg_reg[82]      ^ 1'h0;

    {"cfg_tx_com",              79,         79},            //   CN52XX cfg_tx_com     = fus_cfg_reg[79] ^ jtg_cfg_reg[79] ^ 1'h0;
    {"cfg_cdr_errcor",          78,         78},            //   CN52XX cfg_cdr_errcor = fus_cfg_reg[78] ^ jtg_cfg_reg[78] ^ 1'h0;
    {"cfg_cdr_secord",          77,         77},            //   CN52XX cfg_cdr_secord = fus_cfg_reg[77] ^ jtg_cfg_reg[77] ^ 1'h1;
    {"cfg_cdr_rotate",          76,         76},            //   assign cfg_cdr_rotate      = fus_cfg_reg[76]      ^ jtg_cfg_reg[76]      ^ 1'h0;
    {"cfg_cdr_rqoffs",          75,         68},            //   assign cfg_cdr_rqoffs      = fus_cfg_reg[75:68]   ^ jtg_cfg_reg[75:68]   ^ 8'h40;
    {"cfg_cdr_incx",            67,         64},            //   assign cfg_cdr_incx        = fus_cfg_reg[67:64]   ^ jtg_cfg_reg[67:64]   ^ 4'h2;
    {"cfg_cdr_state",           63,         56},            //   assign cfg_cdr_state       = fus_cfg_reg[63:56]   ^ jtg_cfg_reg[63:56]   ^ 8'h0;
    {"cfg_cdr_bypass",          55,         55},            //   assign cfg_cdr_bypass      = fus_cfg_reg[55]      ^ jtg_cfg_reg[55]      ^ 1'h0;
    {"cfg_tx_byp",              54,         54},            //   assign cfg_tx_byp          = fus_cfg_reg[54]      ^ jtg_cfg_reg[54]      ^ 1'h0;
    {"cfg_tx_val",              53,         44},            //   assign cfg_tx_val          = fus_cfg_reg[53:44]   ^ jtg_cfg_reg[53:44]   ^ 10'h0;
    {"cfg_rx_pol_set",          43,         43},            //   assign cfg_rx_pol_set      = fus_cfg_reg[43]      ^ jtg_cfg_reg[43]      ^ 1'h0;
    {"cfg_rx_pol_clr",          42,         42},            //   assign cfg_rx_pol_clr      = fus_cfg_reg[42]      ^ jtg_cfg_reg[42]      ^ 1'h0;
    {"cfg_cdr_bw_ctl",          41,         40},            //   assign cfg_cdr_bw_ctl      = fus_cfg_reg[41:40]   ^ jtg_cfg_reg[41:40]   ^ 2'h0;
    {"cfg_rst_n_set",           39,         39},            //   assign cfg_rst_n_set       = fus_cfg_reg[39]      ^ jtg_cfg_reg[39]      ^ 1'h0;
    {"cfg_rst_n_clr",           38,         38},            //   assign cfg_rst_n_clr       = fus_cfg_reg[38]      ^ jtg_cfg_reg[38]      ^ 1'h0;
    {"cfg_tx_clk2",             37,         37},            //   assign cfg_tx_clk2         = fus_cfg_reg[37]      ^ jtg_cfg_reg[37]      ^ 1'h0;
    {"cfg_tx_clk1",             36,         36},            //   assign cfg_tx_clk1         = fus_cfg_reg[36]      ^ jtg_cfg_reg[36]      ^ 1'h0;
    {"cfg_tx_pol_set",          35,         35},            //   assign cfg_tx_pol_set      = fus_cfg_reg[35]      ^ jtg_cfg_reg[35]      ^ 1'h0;
    {"cfg_tx_pol_clr",          34,         34},            //   assign cfg_tx_pol_clr      = fus_cfg_reg[34]      ^ jtg_cfg_reg[34]      ^ 1'h0;
    {"cfg_tx_one",              33,         33},            //   assign cfg_tx_one          = fus_cfg_reg[33]      ^ jtg_cfg_reg[33]      ^ 1'h0;
    {"cfg_tx_zero",             32,         32},            //   assign cfg_tx_zero         = fus_cfg_reg[32]      ^ jtg_cfg_reg[32]      ^ 1'h0;
    {"cfg_rxd_wait",            31,         28},            //   assign cfg_rxd_wait        = fus_cfg_reg[31:28]   ^ jtg_cfg_reg[31:28]   ^ 4'h3;
    {"cfg_rxd_short",           27,         27},            //   assign cfg_rxd_short       = fus_cfg_reg[27]      ^ jtg_cfg_reg[27]      ^ 1'h0;
    {"cfg_rxd_set",             26,         26},            //   assign cfg_rxd_set         = fus_cfg_reg[26]      ^ jtg_cfg_reg[26]      ^ 1'h0;
    {"cfg_rxd_clr",             25,         25},            //   assign cfg_rxd_clr         = fus_cfg_reg[25]      ^ jtg_cfg_reg[25]      ^ 1'h0;
    {"cfg_loopback",            24,         24},            //   assign cfg_loopback        = fus_cfg_reg[24]      ^ jtg_cfg_reg[24]      ^ 1'h0;
    {"cfg_tx_idle_set",         23,         23},            //   assign cfg_tx_idle_set     = fus_cfg_reg[23]      ^ jtg_cfg_reg[23]      ^ 1'h0;
    {"cfg_tx_idle_clr",         22,         22},            //   assign cfg_tx_idle_clr     = fus_cfg_reg[22]      ^ jtg_cfg_reg[22]      ^ 1'h0;
    {"cfg_rx_idle_set",         21,         21},            //   assign cfg_rx_idle_set     = fus_cfg_reg[21]      ^ jtg_cfg_reg[21]      ^ 1'h0;
    {"cfg_rx_idle_clr",         20,         20},            //   assign cfg_rx_idle_clr     = fus_cfg_reg[20]      ^ jtg_cfg_reg[20]      ^ 1'h0;
    {"cfg_rx_idle_thr",         19,         16},            //   assign cfg_rx_idle_thr     = fus_cfg_reg[19:16]   ^ jtg_cfg_reg[19:16]   ^ 4'h0;
    {"cfg_com_thr",             15,         12},            //   assign cfg_com_thr         = fus_cfg_reg[15:12]   ^ jtg_cfg_reg[15:12]   ^ 4'h3;
    {"cfg_rx_offset",           11,         8},             //   assign cfg_rx_offset       = fus_cfg_reg[11:8]    ^ jtg_cfg_reg[11:8]    ^ 4'h4;
    {"cfg_skp_max",             7,          4},             //   assign cfg_skp_max         = fus_cfg_reg[7:4]     ^ jtg_cfg_reg[7:4]     ^ 4'hc;
    {"cfg_skp_min",             3,          0},             //   assign cfg_skp_min         = fus_cfg_reg[3:0]     ^ jtg_cfg_reg[3:0]     ^ 4'h4;
    {NULL,                      -1,         -1}
};


const int num_qlm_cn63xx = 3;
const int chain_length_cn63xx = 300;
const field_info_t field_info_cn63xx[] =
{
    {"prbs_err_cnt",        299, 252},  // prbs_err_cnt[47..0]
    {"prbs_lock",           251, 251},  // prbs_lock
    {"jtg_prbs_rst_n",      250, 250},  // jtg_prbs_rst_n
    {"jtg_run_prbs31",      249, 249},  // jtg_run_prbs31
    {"jtg_run_prbs7",       248, 248},  // jtg_run_prbs7
    {"Unused1",             247, 245},  // 0
    {"cfg_pwrup_set",       244, 244},  // cfg_pwrup_set
    {"cfg_pwrup_clr",       243, 243},  // cfg_pwrup_clr
    {"cfg_rst_n_set",       242, 242},  // cfg_rst_n_set
    {"cfg_rst_n_clr",       241, 241},  // cfg_rst_n_clr
    {"cfg_tx_idle_set",     240, 240},  // cfg_tx_idle_set
    {"cfg_tx_idle_clr",     239, 239},  // cfg_tx_idle_clr
    {"cfg_tx_byp",          238, 238},  // cfg_tx_byp
    {"cfg_tx_byp_inv",      237, 237},  // cfg_tx_byp_inv
    {"cfg_tx_byp_val",      236, 227},  // cfg_tx_byp_val[9..0]
    {"cfg_loopback",        226, 226},  // cfg_loopback
    {"shlpbck",             225, 224},  // shlpbck[1..0]
    {"sl_enable",           223, 223},  // sl_enable
    {"sl_posedge_sample",   222, 222},  // sl_posedge_sample
    {"trimen",              221, 220},  // trimen[1..0]
    {"serdes_tx_byp",       219, 219},  // serdes_tx_byp
    {"serdes_pll_byp",      218, 218},  // serdes_pll_byp
    {"lowf_byp",            217, 217},  // lowf_byp
    {"spdsel_byp",          216, 216},  // spdsel_byp
    {"div4_byp",            215, 215},  // div4_byp
    {"clkf_byp",            214, 208},  // clkf_byp[6..0]
    {"Unused2",             207, 206},  // 0
    {"biasdrv_hs_ls_byp",   205, 201},  // biasdrv_hs_ls_byp[4..0]
    {"tcoeff_hf_ls_byp",    200, 197},  // tcoeff_hf_ls_byp[3..0]
    {"biasdrv_hf_byp",      196, 192},  // biasdrv_hf_byp[4..0]
    {"tcoeff_hf_byp",       191, 188},  // tcoeff_hf_byp[3..0]
    {"Unused3",             187, 186},  // 0
    {"biasdrv_lf_ls_byp",   185, 181},  // biasdrv_lf_ls_byp[4..0]
    {"tcoeff_lf_ls_byp",    180, 177},  // tcoeff_lf_ls_byp[3..0]
    {"biasdrv_lf_byp",      176, 172},  // biasdrv_lf_byp[4..0]
    {"tcoeff_lf_byp",       171, 168},  // tcoeff_lf_byp[3..0]
    {"Unused4",             167, 167},  // 0
    {"interpbw",            166, 162},  // interpbw[4..0]
    {"pll_cpb",             161, 159},  // pll_cpb[2..0]
    {"pll_cps",             158, 156},  // pll_cps[2..0]
    {"pll_diffamp",         155, 152},  // pll_diffamp[3..0]
    {"Unused5",             151, 150},  // 0
    {"cfg_rx_idle_set",     149, 149},  // cfg_rx_idle_set
    {"cfg_rx_idle_clr",     148, 148},  // cfg_rx_idle_clr
    {"cfg_rx_idle_thr",     147, 144},  // cfg_rx_idle_thr[3..0]
    {"cfg_com_thr",         143, 140},  // cfg_com_thr[3..0]
    {"cfg_rx_offset",       139, 136},  // cfg_rx_offset[3..0]
    {"cfg_skp_max",         135, 132},  // cfg_skp_max[3..0]
    {"cfg_skp_min",         131, 128},  // cfg_skp_min[3..0]
    {"cfg_fast_pwrup",      127, 127},  // cfg_fast_pwrup
    {"Unused6",             126, 100},  // 0
    {"detected_n",           99,  99},  // detected_n
    {"detected_p",           98,  98},  // detected_p
    {"dbg_res_rx",           97,  94},  // dbg_res_rx[3..0]
    {"dbg_res_tx",           93,  90},  // dbg_res_tx[3..0]
    {"cfg_tx_pol_set",       89,  89},  // cfg_tx_pol_set
    {"cfg_tx_pol_clr",       88,  88},  // cfg_tx_pol_clr
    {"cfg_rx_pol_set",       87,  87},  // cfg_rx_pol_set
    {"cfg_rx_pol_clr",       86,  86},  // cfg_rx_pol_clr
    {"cfg_rxd_set",          85,  85},  // cfg_rxd_set
    {"cfg_rxd_clr",          84,  84},  // cfg_rxd_clr
    {"cfg_rxd_wait",         83,  80},  // cfg_rxd_wait[3..0]
    {"cfg_cdr_limit",        79,  79},  // cfg_cdr_limit
    {"cfg_cdr_rotate",       78,  78},  // cfg_cdr_rotate
    {"cfg_cdr_bw_ctl",       77,  76},  // cfg_cdr_bw_ctl[1..0]
    {"cfg_cdr_trunc",        75,  74},  // cfg_cdr_trunc[1..0]
    {"cfg_cdr_rqoffs",       73,  64},  // cfg_cdr_rqoffs[9..0]
    {"cfg_cdr_inc2",         63,  58},  // cfg_cdr_inc2[5..0]
    {"cfg_cdr_inc1",         57,  52},  // cfg_cdr_inc1[5..0]
    {"fusopt_voter_sync",    51,  51},  // fusopt_voter_sync
    {"rndt",                 50,  50},  // rndt
    {"hcya",                 49,  49},  // hcya
    {"hyst",                 48,  48},  // hyst
    {"idle_dac",             47,  45},  // idle_dac[2..0]
    {"bg_ref_sel",           44,  44},  // bg_ref_sel
    {"ic50dac",              43,  39},  // ic50dac[4..0]
    {"ir50dac",              38,  34},  // ir50dac[4..0]
    {"tx_rout_comp_bypass",  33,  33},  // tx_rout_comp_bypass
    {"tx_rout_comp_value",   32,  29},  // tx_rout_comp_value[3..0]
    {"tx_res_offset",        28,  25},  // tx_res_offset[3..0]
    {"rx_rout_comp_bypass",  24,  24},  // rx_rout_comp_bypass
    {"rx_rout_comp_value",   23,  20},  // rx_rout_comp_value[3..0]
    {"rx_res_offset",        19,  16},  // rx_res_offset[3..0]
    {"rx_cap_gen2",          15,  12},  // rx_cap_gen2[3..0]
    {"rx_eq_gen2",           11,   8},  // rx_eq_gen2[3..0]
    {"rx_cap_gen1",           7,   4},  // rx_cap_gen1[3..0]
    {"rx_eq_gen1",            3,   0},  // rx_eq_gen1[3..0]
    {NULL, -1, -1}
};

void usage(const char *argv[])
{
    printf("\n"
           "Usage:\n"
           "  %s <qlm>\n"
           "    Read and display all QLM jtag settings.\n"
           "\n"
           "  %s <qlm> <lane> <name> <value>\n"
           "    Write a QLM \"lane\" jtag setting of \"name\" as \"value\".\n"
           "\n"
           "    qlm     Which QLM to access.\n"
           "    lane    Which lane to write a setting for.\n"
           "    name    Name of qlm setting to write.\n"
           "    value   The value can be in decimal of hex (0x...).\n"
           "\n"
           "Incorrect settings can damage chips, so be careful!\n"
           "\n", "qlm", "qlm");
}

uint64_t convert_number(const char *str)
{
    unsigned long long result;
    result = simple_strtoul (str, NULL, 10);
    return result;
}


/**
 * Display the state of all register for all lanes
 * on a QLM.
 */
void display_registers(int qlm, int chain_length, const field_info_t *field_info)
{
    int field_num = 0;
    int lane;

    printf("%29s", "Field[<stop bit>:<start bit>]");
    for (lane=0; lane<QLM_LANES; lane++)
        printf("\t      Lane %d", lane);
    printf("\n");

    while (field_info[field_num].name)
    {
        printf("%20s[%3d:%3d]", field_info[field_num].name,
            field_info[field_num].stop_bit, field_info[field_num].start_bit);
        int width = field_info[field_num].stop_bit - field_info[field_num].start_bit + 1;
        for (lane=0; lane<QLM_LANES; lane++)
        {
            int before = chain_length*lane + field_info[field_num].start_bit;
            if (width > 32)
                continue; // FIXME
            cvmx_helper_qlm_jtag_capture(qlm);
            cvmx_helper_qlm_jtag_shift_zeros(qlm, before);
            uint32_t val = cvmx_helper_qlm_jtag_shift(qlm, width, 0);
            printf("\t%4u (0x%04x)", val, val);
        }
        printf("\n");
        field_num++;
    }
}

/**
 * Write a single by doing a read/modify/write on
 * the entire JTAG chain.
 *
 * @param qlm       QLM to write to
 * @param chain_length
 *                  Length of a single lane's jtag chain
 * @param field     field to modify
 * @param lane      Lane to write t0
 * @param new_value New value for the register
 */
void write_register(int qlm, int chain_length, const field_info_t *field, int lane, uint32_t new_value)
{
    int total_length = chain_length*QLM_LANES;
    uint32_t shift_values[total_length/32+1];
    int bits;

    /* Capture the JTAG data current data */
    cvmx_helper_qlm_jtag_capture(qlm);

    /* Shift out data and shift in zeros */
    bits = 0;
    while (bits < total_length)
    {
        int width = total_length - bits;
        if (width > 32)
            width = 32;
        shift_values[bits/32] = cvmx_helper_qlm_jtag_shift(qlm, width, 0);
        bits += width;
    }

    /* Update to the zero data. The contents of the JTAG chain will now the
        fuse defaults which are XORed with it */
    cvmx_helper_qlm_jtag_update(qlm);
    /* Capture the JTAG data to get XOR reference */
    cvmx_helper_qlm_jtag_capture(qlm);

    /* Put new data in our local array */
    for (bits=field->start_bit+lane*chain_length; bits<=field->stop_bit+lane*chain_length; bits++)
    {
        if (new_value & 1)
            shift_values[bits/32] |= 1<<(bits&31);
        else
            shift_values[bits/32] &= ~(1<<(bits&31));
        new_value>>=1;
    }

    /* Shift out data and xor with reference */
    bits = 0;
    uint32_t xor = cvmx_helper_qlm_jtag_shift(qlm, 32, 0);
    while (bits < total_length)
    {
        if (bits + 32 > total_length)
            xor = cvmx_helper_qlm_jtag_shift(qlm, total_length-bits, shift_values[bits/32] ^ xor);
        else
            xor = cvmx_helper_qlm_jtag_shift(qlm, 32, shift_values[bits/32] ^ xor);
        bits += 32;
    }

    /* Update the new data */
    cvmx_helper_qlm_jtag_update(qlm);
}


int do_qlm (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

    int num_qlm;
    int chain_length;
    const field_info_t *field_info;
    int qlm;

    /* Make sure we got the correct number of arguments */
    if ((argc != 2) && (argc != 5))
    {
        usage(argv);
        return -1;
    }


    if (OCTEON_IS_MODEL(OCTEON_CN56XX))
    {
        num_qlm = num_qlm_cn56xx;
        chain_length = chain_length_cn56xx;
        field_info = field_info_cn56xx;
    }
    else if (OCTEON_IS_MODEL(OCTEON_CN52XX))
    {
        num_qlm = num_qlm_cn52xx;
        chain_length = chain_length_cn52xx;
        field_info = field_info_cn52xx;
    }
    else if (OCTEON_IS_MODEL(OCTEON_CN63XX))
    {
        num_qlm = num_qlm_cn63xx;
        chain_length = chain_length_cn63xx;
        field_info = field_info_cn63xx;
    }
    else
    {
        printf("Unsupport Octeon model\n");
        return -1;
    }

    qlm = convert_number (argv[1]);

    if ((qlm < 0) || (qlm >= num_qlm))
    {
        printf("Invalid qlm number\n");
        return -1;
    }

    cvmx_helper_qlm_jtag_init();

    if (argc == 5)
    {
        int lane = convert_number(argv[2]);
        int value = convert_number(argv[4]);
        int field_num = 0;
        while (field_info[field_num].name)
        {
            if (strcmp(field_info[field_num].name, argv[3]) == 0)
            {
                write_register(qlm, chain_length, &field_info[field_num], lane, value);
                break;
            }
            field_num++;
        }
        if (!field_info[field_num].name)
        {
            printf("Illegal field name \"%s\"\n", argv[3]);
            return -1;
        }
    }
    else
        display_registers(qlm, chain_length, field_info);
    return 0;



}

U_BOOT_CMD(
	qlm,      128,      5,      do_qlm,
	"qlm -  Octeon QLM debug function\n",
);
#endif
