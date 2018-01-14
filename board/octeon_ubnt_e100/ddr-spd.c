/* Serial Presence Detect (SPD) for DDR2 SDRAM - JEDEC Standard No. 21-C */
/* ===================================================================== */

/* Notes: "R" indicates which bytes are REQUIRED to configure the
   Octeon DDR2 Controller. */

const unsigned char e100_ddr2_spd_data[] = {
/* Byte                                                                     */
/* Number  Function                                                         */
/* ======  ===============================================================  */
/*  0      Number of Serial PD Bytes written during module production       */
    0x80,
/*  1      Total number of Bytes in Serial PD device                        */
    0x08, /* log2(256) */
/*  2 R    Fundamental Memory Type (FPM, EDO, SDRAM, DDR, DDR2)             */
    0x08,
/*  3 R    Number of Row Addresses on this assembly                         */
    0x0e,
/*  4 R    Number of Column Addresses on this assembly                      */
    0x0a,
/*  5 R    Number of DIMM Ranks                                             */
    0x00, /* 1 rank */
/*  6      Data Width of this assembly                                      */
    0x00,
/*  7      Reserved                                                         */
    0x00,
/*  8      Voltage Interface Level of this assembly                         */
    0x00,
/*  9 R    SDRAM Cycle time at Maximum Supported CAS Latency (CL)           */
    0x37, /* 0011 0111 => 3.7 ns */
/* 10      SDRAM Access from Clock tAC                                      */
    0x00,
/* 11 R    DIMM configuration type (Non-parity, Parity or ECC)              */
    0x00, /* non-ECC */
/* 12 R    Refresh Rate/Type                                                */
    0x02, /* 128 KHz */
/* 13 R    Primary SDRAM Width                                              */
    0x08,
/* 14      Error Checking SDRAM Width                                       */
    0x00,
/* 15      Reserved                                                         */
    0x00,
/* 16 R    SDRAM Device Attributes: Burst Lengths Supported                 */
    0x0c, /* 8 and 4 */
/* 17 R    SDRAM Device Attributes: Number of Banks on SDRAM Device         */
    0x08,
/* 18 R    SDRAM Device Attributes: CAS Latency                             */
    0x70, /* 6, 5, 4 */
/* 19      Reserved                                                         */
    0x00,
/* 20 R    DIMM Type Information                                            */
    0x00, /* not registered */
/* 21      SDRAM Module Attributes                                          */
    0x00,
/* 22      SDRAM Device Attributes: General                                 */
    0x00,
/* 23 R    Minimum Clock Cycle at CLX-1                                     */
    0x37, /* 0011 0111 => 3.7 ns */
/* 24      Maximum Data Access Time (tAC) from Clock at CLX-1               */
    0x00,
/* 25 R    Minimum Clock Cycle at CLX-2                                     */
    0x37, /* 0011 0111 => 3.7 ns < tCLK 3.765 ns => force CAS 4 */
/* 26      Maximum Data Access Time (tAC) from Clock at CLX-2               */
    0x00,
/* 27 R    Minimum Row Precharge Time (tRP)                                 */
    0x3c, /* 0011 1100 => 15 ns */
/* 28 R    Minimum Row Active to Row Active delay (tRRD)                    */
    0x1e, /* 0001 1110 => 7.5 ns */
/* 29 R    Minimum RAS to CAS delay (tRCD)                                  */
    0x3c, /* 0011 1100 => 15 ns */
/* 30 R    Minimum Active to Precharge Time (tRAS)                          */
    0x2d, /* 45 ns */
/* 31      Module Rank Density                                              */
    0x00,
/* 32      Address and Command Input Setup Time Before Clock (tIS)          */
    0x00,
/* 33      Address and Command Input Hold Time After Clock (tIH)            */
    0x00,
/* 34      Data Input Setup Time Before Clock (tDS)                         */
    0x00,
/* 35      Data Input Hold Time After Clock (tDH)                           */
    0x00,
/* 36 R    Write recovery time (tWR)                                        */
    0x3c, /* 0011 1100 => 15 ns */
/* 37 R    Internal write to read command delay (tWTR)                      */
    0x1e, /* 0001 1110 => 7.5 ns */
/* 38      Internal read to precharge command delay (tRTP)                  */
    0x00,
/* 39      Memory Analysis Probe Characteristics                            */
    0x00,
/* 40 R    Extension of Byte 41 tRC and Byte 42 tRFC                        */
    0x06, /* 0000 011 0 => +0.5 ns tRFC */
/* 41      SDRAM Device Minimum Active to Active/Auto Refresh Time (tRC)    */
    0x00,
/* 42 R    SDRAM Min Auto-Ref to Active/Auto-Ref Command Period (tRFC)      */
    0x7f, /* 127 + 0.5 (from 40) ns */
/* 43      SDRAM Device Maximum device cycle time (tCKmax)                  */
    0x00,
/* 44      SDRAM Device maximum skew between DQS and DQ signals (tDQSQ)     */
    0x00,
/* 45      SDRAM Device Maximum Read DataHold Skew Facktor (tQHS)           */
    0x00,
/* 46      PLL Relock Time                                                  */
    0x00,
/* 47-xx   IDD in SPD - To be defined                                       */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
/*         Reserved                                                         */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
/* xx-61   Reserved                                                         */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
/* 62      SPD Revision                                                     */
    0x00,
/* 63      Checksum for Bytes 0-62                                          */
    0x00,
/* 64-71   Manufacturers JEDEC ID Code                                      */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
/* 72      Module Manufacturing Location                                    */
    0x00,
/* 73-90   Module Part Number                                               */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
/* 91-92   Module Revision Code                                             */
    0x00,
    0x00,
/* 93-94   Module Manufacturing Date                                        */
    0x00,
    0x00,
/* 95-98   Module Serial Number                                             */
    0x00,
    0x00,
    0x00,
    0x00,
/* 99-127  Manufacturers Specific Data                                      */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};
