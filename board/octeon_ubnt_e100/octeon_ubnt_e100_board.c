/*
 * Copyright (C) 2011 Ubiquiti Networks, Inc.
 * All Rights Reserved.
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
#include <miiphy.h>
#include <flash.h>
#include <malloc.h>

#if (CONFIG_COMMANDS & CFG_CMD_MII)
#define mdio_write(x,y,z) miiphy_write(x,y,z)
#define mdio_read(x,y,z) miiphy_read(x,y,z)

#endif /* MII */

#if defined(CONFIG_PCI)
void pci_init_board (void)
{
}
#endif


int octeon_get_board_major_rev(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    return(gd->board_desc.rev_major);
}

int octeon_get_board_minor_rev(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    return(gd->board_desc.rev_minor);
}

#define BOARD_E101_MAJOR 1
#define BOARD_E102_MAJOR 2
#define BOARD_E120_MAJOR 0 /* UniFi Security Gateway */

int checkboard (void)
{
    DECLARE_GLOBAL_DATA_PTR;
    cvmx_smix_clk_t sclk;

    /* TYPE=0 EN=1 P0MII=0 => all ports (0, 1, and 2) are RGMII */
    cvmx_write_csr (CVMX_GMXX_INF_MODE (0), 0x2);

    /* Enable SMI */
    sclk.u64 = cvmx_read_csr(CVMX_SMI_CLK);
    sclk.s.phase = 0x20;
    cvmx_write_csr(CVMX_SMI_CLK, sclk.u64);

    cvmx_write_csr (CVMX_SMI_EN, 0x1);
    cvmx_read_csr(CVMX_SMI_EN);

    if (gd->board_desc.rev_major == 0xff) {
        int i = 0;
        unsigned char m = BOARD_E101_MAJOR;
        unsigned char a[3] = {4, 5, 6};
        unsigned short r[3];
        unsigned short v[3] = {0xd030, 0xfff0, 0xd070};
        for (i = 0; i < 3; i++) {
            (void) miiphy_read(a[i], 3, &r[i]);
            if ((r[i] & 0xfff0) != v[i]) {
                m = BOARD_E102_MAJOR;
                break;
            }
        }
        gd->board_desc.rev_major = m;
    }

    return 0;
}

static void *translate_flash_addr(void *base)
{
    void *ptr = base;
#if CONFIG_OCTEON_UBOOT_TLB
    cvmx_mio_boot_reg_cfgx_t __attribute__((unused)) reg_cfg;
    reg_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_REG_CFG0);
    uint32_t normal_base = ((CFG_FLASH_BASE >> 16) & 0x1fff);
    uint32_t offset = normal_base - reg_cfg.s.base;

    offset = offset << 16;
    ptr = (void *)((char *) base - offset);
#endif
    return ptr;
}

static void *get_ubnt_gd_addr(void)
{
    return translate_flash_addr((void *) CFG_UBNT_GD_ADDR);
}

static void *get_ubnt_eeprom_addr(void)
{
    return translate_flash_addr((void *) CFG_UBNT_EEPROM_ADDR);
}

#define UBNT_MAX_DESC_LEN 64

static int validate_and_fix_gd_entry(octeon_eeprom_header_t *hdr)
{
    int i;
    uint16_t csum = 0;
    uint8_t *p = (uint8_t *) hdr;
    octeon_eeprom_board_desc_t *bd = (octeon_eeprom_board_desc_t *) hdr;

    be16_to_cpus(hdr->type);
    switch (hdr->type) {
    case EEPROM_MAC_ADDR_TYPE:
        break;
    case EEPROM_BOARD_DESC_TYPE:
        be16_to_cpus(bd->board_type);
        break;
    default:
        return 0;
    }

    be16_to_cpus(hdr->length);
    if (hdr->length > UBNT_MAX_DESC_LEN) {
        return 0;
    }

    be16_to_cpus(hdr->checksum);

    for (i = 0; i < hdr->length; i++) {
        csum += p[i];
    }
    csum -= (hdr->checksum & 0xff);
    csum -= (hdr->checksum >> 8);
    if (csum != hdr->checksum) {
        return 0;
    }

    return 1;
}

void print_mpr()
{
    void *eptr = get_ubnt_eeprom_addr();
    uint8_t edata[32];
    uint32_t mpt;

    memcpy(edata, eptr, 32);
    mpt = (edata[16] << 16) + (edata[17] << 8) + edata[18];
    printf("MPR 13-%05u-%02u\n", mpt, edata[19]);
}

int early_board_init(void)
{
    int cpu_ref = 50;

    DECLARE_GLOBAL_DATA_PTR;
    void *gdptr = get_ubnt_gd_addr();

    memcpy((void *) &(gd->mac_desc), gdptr + CFG_UBNT_GD_MAC_DESC_OFF,
           sizeof(octeon_eeprom_mac_addr_t));
    memcpy((void *) &(gd->board_desc), gdptr + CFG_UBNT_GD_BOARD_DESC_OFF,
           sizeof(octeon_eeprom_board_desc_t));
    memset((void *)&(gd->clock_desc), 0x0, sizeof(octeon_eeprom_clock_desc_t));

    if (!validate_and_fix_gd_entry(&(gd->mac_desc.header))
            || gd->mac_desc.header.type != EEPROM_MAC_ADDR_TYPE) {
        printf("Invalid MAC descriptor entry, using defaults\n");
        gd->mac_desc.count = 8;
        gd->mac_desc.mac_addr_base[0] = 0x00;
        gd->mac_desc.mac_addr_base[1] = 0xbe;
        gd->mac_desc.mac_addr_base[2] = 0xef;
        gd->mac_desc.mac_addr_base[3] = 0x10;
        gd->mac_desc.mac_addr_base[4] = 0x00;
        gd->mac_desc.mac_addr_base[5] = 0x00;
    }

    if (!validate_and_fix_gd_entry(&(gd->board_desc.header))
            || gd->board_desc.header.type != EEPROM_BOARD_DESC_TYPE
            || (   (gd->board_desc.board_type != CVMX_BOARD_TYPE_UBNT_E100)
                && (gd->board_desc.board_type != CVMX_BOARD_TYPE_UBNT_E120))) {
        printf("Invalid board descriptor entry, using defaults\n");
        strncpy((char *)(gd->board_desc.serial_str), "fffffffffffffffff",
                SERIAL_LEN);
        gd->board_desc.rev_minor = 0xff;
        gd->board_desc.board_type = CVMX_BOARD_TYPE_UBNT_E100;
        gd->board_desc.rev_major = 0xff;
    }

    cpu_ref = 50;
    gd->ddr_ref_hertz = 50000000;
    gd->ddr_clock_mhz = 266;

    /* Read CPU clock multiplier */
    cvmx_dbg_data_t dbg_data;
    dbg_data.u64 = cvmx_read_csr(CVMX_DBG_DATA);
    int mul = dbg_data.cn30xx.c_mul;
    gd->cpu_clock_mhz = mul * cpu_ref;

    if (gd->cpu_clock_mhz < 100 || gd->cpu_clock_mhz > 900)
    {
        gd->cpu_clock_mhz = DEFAULT_ECLK_FREQ_MHZ;
    }

    cvmx_mpi_cfg_t mpi_cfg;
    mpi_cfg.u64 = 0;
    mpi_cfg.cn50xx.clkdiv = 32;
    mpi_cfg.cn50xx.csena = 1;
    mpi_cfg.cn50xx.clk_cont = 0;
    mpi_cfg.cn50xx.enable = 1;
    cvmx_write_csr(CVMX_MPI_CFG, mpi_cfg.u64);

    return 0;
}

void octeon_led_str_write(const char *str)
{
}

/****** ubntw commands ******/
static void fill_checksum(octeon_eeprom_header_t *hdr)
{
    int i;
    uint16_t csum = 0;
    uint8_t *p = (uint8_t *) hdr;
    unsigned int len = ntohs(hdr->length);

    for (i = 0; i < len; i++) {
        csum += p[i];
    }
    hdr->checksum = htons(csum);
}

static int ubnt_write_flash(void *dst, void *src, int size)
{
    ulong offset, blk_size, off_in_blk, blk_base, blk_head_addr;
    int r, ret;
    void *buf;

    ret = 0;
    offset = ((unsigned int)dst) - CFG_FLASH_BASE;
    if ((offset < 0) || (offset > CFG_FLASH_SIZE)) {
        return 1;
    }

    if ((CFG_FLASH_SIZE - offset) <= (64 * 1024)) {
        blk_size = 8 * 1024;
        blk_base = CFG_FLASH_BASE + CFG_FLASH_SIZE - (64 * 1024);
    } else {
        blk_size = 64 * 1024;
        blk_base = CFG_FLASH_BASE;
    }

    off_in_blk = offset & (blk_size - 1);
    if ((off_in_blk + size) > blk_size) {
        /* FIXME: multiple blocks support */
        return 1;
    }

    if (!(buf = (void *) malloc(blk_size))) {
        printf("%s: Failed to allocate memory(%d)\n", __FUNCTION__, blk_size);
        return 1;
    }

    blk_head_addr = blk_base + (offset - off_in_blk);

    memcpy(buf, (void *)blk_head_addr, blk_size);
    memcpy(buf + off_in_blk, src, size);

    if ((r = flash_sect_protect(0, blk_head_addr, blk_head_addr + blk_size - 1))) {
        printf("%s: Failed on flash_sect_protect from:0x%08x, len:%d\n", __FUNCTION__, blk_head_addr, blk_size);
        flash_perror(r);
        ret = 1;
        goto out;
    }
    if ((r = flash_sect_erase(blk_head_addr, blk_head_addr + blk_size - 1))) {
        printf("%s: Failed on flash_sect_erase from:0x%08x, len:%d\n", __FUNCTION__, blk_head_addr, blk_size);
        flash_perror(r);
        ret = 1;
        goto out;
    }
    if ((r = flash_write(buf, blk_head_addr, blk_size))) {
        printf("%s: Failed on flash_sect_write from:0x%08x, len:%d\n", __FUNCTION__, blk_head_addr, blk_size);
        flash_perror(r);
        ret = 1;
        goto out;
    }

out:
    free(buf);
    printf("\n");
    return ret;
}

static int write_ubnt_gd_entries(octeon_eeprom_header_t **hdr)
{
    int i, r, ret = 0;
    unsigned int gdoff;
    void *gdptr = get_ubnt_gd_addr();
    void *buf;

    if (!hdr) {
        printf("Invalid GD entry\n");
        return 1;
    }

    if (!(buf = (void *) malloc(CFG_UBNT_GD_SIZE))) {
        printf("Failed to allocate memory\n");
        return 1;
    }

    memcpy(buf, gdptr, CFG_UBNT_GD_SIZE);

    for (i = 0; hdr[i]; i++) {
        unsigned int len = ntohs(hdr[i]->length);

        fill_checksum(hdr[i]);

        switch (ntohs(hdr[i]->type)) {
        case EEPROM_MAC_ADDR_TYPE:
            gdoff = CFG_UBNT_GD_MAC_DESC_OFF;
            break;
        case EEPROM_BOARD_DESC_TYPE:
            gdoff = CFG_UBNT_GD_BOARD_DESC_OFF;
            break;
        default:
            printf("Unsupported UBNT GD type\n");
            ret = 1;
            goto out;
        }

        memcpy(buf + gdoff, hdr[i], len);
    }

    if ((r = ubnt_write_flash(gdptr, buf, CFG_UBNT_GD_SIZE))) {
        ret = 1; 
        goto out;
    }

out:
    free(buf);
    printf("\n");
    return ret;
}

struct board_type_info {
    const char *type;
    uint16_t board_type;
    uint8_t major;
    uint8_t dr_id[4];
};

static struct board_type_info board_types[] = {
    {"e101", CVMX_BOARD_TYPE_UBNT_E100, BOARD_E101_MAJOR, {0xee, 0x02, 0x07, 0x77}},
    {"e102", CVMX_BOARD_TYPE_UBNT_E100, BOARD_E102_MAJOR, {0xee, 0x01, 0x07, 0x77}},
    {"e120", CVMX_BOARD_TYPE_UBNT_E120, BOARD_E120_MAJOR, {0xee, 0x21, 0x07, 0x77}},
    {NULL, 0xff, 0xff, {0xff, 0xff, 0xff, 0xff}}
};

static struct board_type_info *find_board_type(const char *btype)
{
    int i;

    for (i = 0; board_types[i].type; i++)
        if (strcmp(btype, board_types[i].type) == 0)
            return &(board_types[i]);

    return NULL;
}

static int write_ubnt_eeprom(struct board_type_info *binfo,
                             octeon_eeprom_mac_addr_t *mac,
                             uint8_t *mpr)
{
    int i, r;
    uint8_t data[32];
    uint8_t *d = binfo->dr_id;
    void *eptr = get_ubnt_eeprom_addr();

    memset(data, 0, 32);
    for (i = 0; i < 6; i++) {
        data[i] = mac->mac_addr_base[i];
        data[i + 6] = mac->mac_addr_base[i];
    }
    data[6] |= 0x02;
    for (i = 0; i < 4; i++) {
        data[(12 + i)] = d[i];
        data[(16 + i)] = mpr[i];
    }

    if ((r = ubnt_write_flash(eptr, data, 32))) {
        return 1;
    }

    return 0;
}

static void fill_mac_desc(octeon_eeprom_mac_addr_t *mac, const char *mstr,
                          const char *mcount)
{
    int i;
    uint64_t maddr;

    mac->header.type = htons(EEPROM_MAC_ADDR_TYPE);
    mac->header.length = htons(sizeof(octeon_eeprom_mac_addr_t));
    mac->header.minor_version = 0;
    mac->header.major_version = 1;
    mac->header.checksum = 0;

    maddr = simple_strtoull(mstr, NULL, 16);
    cpu_to_be64s(maddr);
    mac->count = simple_strtoul(mcount, NULL, 10);
    for (i = 0; i < 6; i++) {
        mac->mac_addr_base[i] = (maddr >> ((5 - i) * 8)) & 0xff;
    }
}

static void fill_board_desc(octeon_eeprom_board_desc_t *bd,
                            struct board_type_info *binfo,
                            const char *b_minor, const char *serial)
{
    bd->header.type = htons(EEPROM_BOARD_DESC_TYPE);
    bd->header.length = htons(sizeof(octeon_eeprom_board_desc_t));
    bd->header.minor_version = 0;
    bd->header.major_version = 1;
    bd->header.checksum = 0;

    bd->board_type = htons(binfo->board_type);
    bd->rev_major = binfo->major;
    bd->rev_minor = simple_strtoul(b_minor, NULL, 10);
    strncpy((char *) (bd->serial_str), serial, SERIAL_LEN);
    (bd->serial_str)[SERIAL_LEN - 1] = 0;
}

static void show_mac_desc(octeon_eeprom_mac_addr_t *mac)
{
    if (!mac) {
        return;
    }

    printf("MAC base: %02x:%02x:%02x:%02x:%02x:%02x, count: %u\n",
           mac->mac_addr_base[0], mac->mac_addr_base[1],
           mac->mac_addr_base[2], mac->mac_addr_base[3],
           mac->mac_addr_base[4], mac->mac_addr_base[5], mac->count);
}

static void show_board_desc(octeon_eeprom_board_desc_t *bd)
{
    if (!bd) {
        return;
    }

    printf("Board type: %s (%u.%u)\nSerial number: %s\n",
           cvmx_board_type_to_string(ntohs(bd->board_type)),
           bd->rev_major, bd->rev_minor, bd->serial_str);
}

int do_ubntw(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    char op;

    if (!argv[1]) {
        printf("Operation required\n");
        return 1;
    }

    op = argv[1][0];
    if (op == 'm') {
        octeon_eeprom_mac_addr_t mac;
        octeon_eeprom_header_t *hdrs[] = {
            ((octeon_eeprom_header_t *) &mac),
            NULL
        };

        if (argc < 4) {
            printf("MAC base address and count required\n");
            return 1;
        }

        fill_mac_desc(&mac, argv[2], argv[3]);

        if (write_ubnt_gd_entries(hdrs) != 0) {
            printf("Write failed\n");
            return 1;
        }

        show_mac_desc(&mac);
        return 0;
    } else if (op == 'b') {
        octeon_eeprom_board_desc_t bd;
        octeon_eeprom_header_t *hdrs[] = {
            ((octeon_eeprom_header_t *) &bd),
            NULL
        };
        struct board_type_info *binfo;

        if (argc < 5) {
            printf("Required arguments missing\n");
            return 1;
        }

        if (!(binfo = find_board_type(argv[2]))) {
            printf("Invalid type\n");
            return 1;
        }

        fill_board_desc(&bd, binfo, argv[3], argv[4]);

        if (write_ubnt_gd_entries(hdrs) != 0) {
            printf("Write failed\n");
            return 1;
        }

        show_board_desc(&bd);
        return 0;
    } else if (op == 'a') {
        octeon_eeprom_mac_addr_t mac;
        octeon_eeprom_board_desc_t bd;
        octeon_eeprom_header_t *hdrs[] = {
            ((octeon_eeprom_header_t *) &mac),
            ((octeon_eeprom_header_t *) &bd),
            NULL
        };
        struct board_type_info *binfo;
        uint8_t mpr[4] = {0xff, 0xff, 0xff, 0xff};

        if (argc < 7) {
            printf("Required arguments missing\n");
            return 1;
        }

        if (!(binfo = find_board_type(argv[4]))) {
            printf("Invalid type\n");
            return 1;
        }

        fill_mac_desc(&mac, argv[2], argv[3]);
        fill_board_desc(&bd, binfo, argv[5], argv[6]);

        mpr[3] = simple_strtoul(argv[5], NULL, 10);
        if (argc == 8) {
            uint32_t pt = simple_strtoul(argv[7], NULL, 10);
            mpr[0] = (pt >> 16) & 0xff;
            mpr[1] = (pt >> 8) & 0xff;
            mpr[2] = pt & 0xff;
        }

        if (write_ubnt_gd_entries(hdrs) != 0) {
            printf("Write data failed\n");
            return 1;
        }
        if (write_ubnt_eeprom(binfo, &mac, mpr) != 0) {
            printf("Write failed\n");
            return 1;
        }

        show_mac_desc(&mac);
        show_board_desc(&bd);
        return 0;
    }

    printf("Invalid operation\n");
    return 1;
}

U_BOOT_CMD(ubntw, 8, 1, do_ubntw,
    "ubntw      - ubntw commands\n",
    "mac <mac> <count>   - write MAC address\n"
    "ubntw board <type> <minor> <serial>   - write board info\n"
    "ubntw all <mac> <count> <type> <minor> <serial> [<pt>]   - write all\n");

/*** LED control ***/
static uint32_t e101_led_vals[] = { 0x1, 0x2, 0x0, 0x3 };
static uint32_t e102_led_vals[] = { 0xc5, 0xcf, 0xca, 0x0 };

static void _set_led(uint32_t val)
{
    switch (cvmx_sysinfo_get()->board_type) {
    case CVMX_BOARD_TYPE_UBNT_E100:
        switch (cvmx_sysinfo_get()->board_rev_major) {
        case BOARD_E101_MAJOR:
            miiphy_write(0x18, 0x0, 0x0);
            miiphy_write(0x11, 0xe, 0xff02);
            miiphy_write(0x11, 0xf, (0x03fc | e101_led_vals[val]));
            break;
        case BOARD_E102_MAJOR:
        default:
            break;
        }
        break;
    case CVMX_BOARD_TYPE_UBNT_E120:
        switch (cvmx_sysinfo_get()->board_rev_major) {
        case BOARD_E120_MAJOR:
            /* FIXME: change to system LED? */
            miiphy_write(0x5, 0x19, (0x3000 | e102_led_vals[val]));
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void board_set_led_blink(void)
{
    _set_led(0);
}

void board_set_led_on(void)
{
    _set_led(1);
}

void board_set_led_off(void)
{
    _set_led(2);
}

void board_set_led_normal(void)
{
    _set_led(3);
}
