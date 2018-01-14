/*
 * (C) Copyright 2000, 2001
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
 *
 */

#include <common.h>
#include <config.h>
#include <command.h>
#include <i2c.h>
#include "cvmx.h"
#include "octeon-model.h"

#if (CONFIG_COMMANDS & CFG_CMD_EEPROM) || defined(CFG_ENV_IS_IN_EEPROM)

extern void eeprom_init  (void);
extern int  eeprom_read  (unsigned dev_addr, unsigned offset,
			  uchar *buffer, unsigned cnt);
extern int  eeprom_write (unsigned dev_addr, unsigned offset,
			  uchar *buffer, unsigned cnt);
#endif


#if defined(CFG_EEPROM_X40430)
	/* Maximum number of times to poll for acknowledge after write */
#define MAX_ACKNOWLEDGE_POLLS	10
#endif

/* ------------------------------------------------------------------------- */

#if (CONFIG_COMMANDS & CFG_CMD_EEPROM)
int do_eeprom ( cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	const char *const fmt =
		"\nEEPROM @0x%lX %s: addr %08lx  off %04lx  count %ld ... ";

#if defined(CFG_I2C_MULTI_EEPROMS)
	if (argc == 6) {
		ulong dev_addr = simple_strtoul (argv[2], NULL, 16);
		ulong addr = simple_strtoul (argv[3], NULL, 16);
		ulong off  = simple_strtoul (argv[4], NULL, 16);
		ulong cnt  = simple_strtoul (argv[5], NULL, 16);
#else
	if (argc == 5) {
		ulong dev_addr = CFG_DEF_EEPROM_ADDR;
		ulong addr = simple_strtoul (argv[2], NULL, 16);
		ulong off  = simple_strtoul (argv[3], NULL, 16);
		ulong cnt  = simple_strtoul (argv[4], NULL, 16);
#endif /* CFG_I2C_MULTI_EEPROMS */

# ifndef CONFIG_SPI
		eeprom_init ();
# endif /* !CONFIG_SPI */

		if (strcmp (argv[1], "read") == 0) {
			int rcode;

			printf (fmt, dev_addr, argv[1], addr, off, cnt);

			rcode = eeprom_read (dev_addr, off, (uchar *) addr, cnt);

			puts ("done\n");
			return rcode;
		} else if (strcmp (argv[1], "write") == 0) {
			int rcode;

			printf (fmt, dev_addr, argv[1], addr, off, cnt);

			rcode = eeprom_write (dev_addr, off, (uchar *) addr, cnt);

			puts ("done\n");
			return rcode;
		}
	}

	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
}
#endif	/* CFG_CMD_EEPROM */

/*-----------------------------------------------------------------------
 *
 * for CFG_I2C_EEPROM_ADDR_LEN == 2 (16-bit EEPROM address) offset is
 *   0x000nxxxx for EEPROM address selectors at n, offset xxxx in EEPROM.
 *
 * for CFG_I2C_EEPROM_ADDR_LEN == 1 (8-bit EEPROM page address) offset is
 *   0x00000nxx for EEPROM address selectors and page number at n.
 */

#if (CONFIG_COMMANDS & CFG_CMD_EEPROM) || defined(CFG_ENV_IS_IN_EEPROM)

#ifndef CONFIG_SPI
#if !defined(CFG_I2C_EEPROM_ADDR_LEN) || CFG_I2C_EEPROM_ADDR_LEN < 1 || CFG_I2C_EEPROM_ADDR_LEN > 2
#error CFG_I2C_EEPROM_ADDR_LEN must be 1 or 2
#endif
#endif

int eeprom_read (unsigned dev_addr, unsigned offset, uchar *buffer, unsigned cnt)
{
	unsigned end = offset + cnt;
	unsigned blk_off;
	int rcode = 0;

	/* Read data until done or would cross a page boundary.
	 * We must write the address again when changing pages
	 * because the next page may be in a different device.
	 */
	while (offset < end) {
		unsigned alen, len, maxlen;
#if CFG_I2C_EEPROM_ADDR_LEN == 1 && !defined(CONFIG_SPI_X)
		uchar addr[2];

		blk_off = offset & 0xFF;	/* block offset */

		addr[0] = offset >> 8;		/* block number */
		addr[1] = blk_off;		/* block offset */
		alen	= 2;
#else
		uchar addr[3];

		blk_off = offset & 0xFF;	/* block offset */

		addr[0] = offset >> 16;		/* block number */
		addr[1] = offset >>  8;		/* upper address octet */
		addr[2] = blk_off;		/* lower address octet */
		alen	= 3;
#endif	/* CFG_I2C_EEPROM_ADDR_LEN, CONFIG_SPI_X */

		addr[0] |= dev_addr;		/* insert device address */

		maxlen = 0x100 - blk_off;
		if (maxlen > I2C_RXTX_LEN)
			maxlen = I2C_RXTX_LEN;
		len    = end - offset;
		if (len > maxlen)
			len = maxlen;
#ifdef CONFIG_SPI
		spi_read (addr, alen, buffer, len);
#else
		if (i2c_read (addr[0], offset, alen-1, buffer, len) != 0)
			rcode = 1;
#endif
		buffer += len;
		offset += len;
	}
	return rcode;
}

/*-----------------------------------------------------------------------
 *
 * for CFG_I2C_EEPROM_ADDR_LEN == 2 (16-bit EEPROM address) offset is
 *   0x000nxxxx for EEPROM address selectors at n, offset xxxx in EEPROM.
 *
 * for CFG_I2C_EEPROM_ADDR_LEN == 1 (8-bit EEPROM page address) offset is
 *   0x00000nxx for EEPROM address selectors and page number at n.
 */

int eeprom_write (unsigned dev_addr, unsigned offset, uchar *buffer, unsigned cnt)
{
	unsigned end = offset + cnt;
	unsigned blk_off;
	int rcode = 0;

#if defined(CFG_EEPROM_X40430)
	uchar	contr_r_addr[2];
	uchar	addr_void[2];
	uchar	contr_reg[2];
	uchar	ctrl_reg_v;
	int	i;
#endif

	/* Write data until done or would cross a write page boundary.
	 * We must write the address again when changing pages
	 * because the address counter only increments within a page.
	 */

	while (offset < end) {
		unsigned alen, len, maxlen;
#if CFG_I2C_EEPROM_ADDR_LEN == 1 && !defined(CONFIG_SPI_X)
		uchar addr[2];

		blk_off = offset & 0xFF;	/* block offset */

		addr[0] = offset >> 8;		/* block number */
		addr[1] = blk_off;		/* block offset */
		alen	= 2;
#else
		uchar addr[3];

		blk_off = offset & 0xFF;	/* block offset */

		addr[0] = offset >> 16;		/* block number */
		addr[1] = offset >>  8;		/* upper address octet */
		addr[2] = blk_off;		/* lower address octet */
		alen	= 3;
#endif	/* CFG_I2C_EEPROM_ADDR_LEN, CONFIG_SPI_X */

		addr[0] |= dev_addr;		/* insert device address */

#if defined(CFG_EEPROM_PAGE_WRITE_BITS)

#define	EEPROM_PAGE_SIZE	(1 << CFG_EEPROM_PAGE_WRITE_BITS)
#define	EEPROM_PAGE_OFFSET(x)	((x) & (EEPROM_PAGE_SIZE - 1))

		maxlen = EEPROM_PAGE_SIZE - EEPROM_PAGE_OFFSET(blk_off);
#else
		maxlen = 0x100 - blk_off;
#endif
		if (maxlen > I2C_RXTX_LEN)
			maxlen = I2C_RXTX_LEN;

		len = end - offset;
		if (len > maxlen)
			len = maxlen;
#ifdef CONFIG_SPI
		spi_write (addr, alen, buffer, len);
#else
#if defined(CFG_EEPROM_X40430)
		/* Get the value of the control register.
		 * Set current address (internal pointer in the x40430)
		 * to 0x1ff.
		 */
		contr_r_addr[0] = 9;
		contr_r_addr[1] = 0xff;
		addr_void[0]    = 0;
		addr_void[1]    = addr[1];
#ifdef CFG_I2C_EEPROM_ADDR
		contr_r_addr[0] |= CFG_I2C_EEPROM_ADDR;
		addr_void[0]    |= CFG_I2C_EEPROM_ADDR;
#endif
		contr_reg[0] = 0xff;
		if (i2c_read (contr_r_addr[0], contr_r_addr[1], 1, contr_reg, 1) != 0) {
			rcode = 1;
		}
		ctrl_reg_v = contr_reg[0];

		/* Are any of the eeprom blocks write protected?
		 */
		if (ctrl_reg_v & 0x18) {
			ctrl_reg_v &= ~0x18;   /* reset block protect bits  */
			ctrl_reg_v |=  0x02;   /* set write enable latch    */
			ctrl_reg_v &= ~0x04;   /* clear RWEL                */

			/* Set write enable latch.
			 */
			contr_reg[0] = 0x02;
			if (i2c_write (contr_r_addr[0], 0xff, 1, contr_reg, 1) != 0) {
				rcode = 1;
			}

			/* Set register write enable latch.
			 */
			contr_reg[0] = 0x06;
			if (i2c_write (contr_r_addr[0], 0xFF, 1, contr_reg, 1) != 0) {
				rcode = 1;
			}

			/* Modify ctrl register.
			 */
			contr_reg[0] = ctrl_reg_v;
			if (i2c_write (contr_r_addr[0], 0xFF, 1, contr_reg, 1) != 0) {
				rcode = 1;
			}

			/* The write (above) is an operation on NV memory.
			 * These can take some time (~5ms), and the device
			 * will not respond to further I2C messages till
			 * it's completed the write.
			 * So poll device for an I2C acknowledge.
			 * When we get one we know we can continue with other
			 * operations.
			 */
			contr_reg[0] = 0;
			for (i = 0; i < MAX_ACKNOWLEDGE_POLLS; i++) {
				if (i2c_read (addr_void[0], addr_void[1], 1, contr_reg, 1) == 0)
					break;	/* got ack */
#if defined(CFG_EEPROM_PAGE_WRITE_DELAY_MS)
				udelay(CFG_EEPROM_PAGE_WRITE_DELAY_MS * 1000);
#endif
			}
			if (i == MAX_ACKNOWLEDGE_POLLS) {
				puts ("EEPROM poll acknowledge failed\n");
				rcode = 1;
			}
		}

		/* Is the write enable latch on?.
		 */
		else if (!(ctrl_reg_v & 0x02)) {
			/* Set write enable latch.
			 */
			contr_reg[0] = 0x02;
			if (i2c_write (contr_r_addr[0], 0xFF, 1, contr_reg, 1) != 0) {
			       rcode = 1;
			}
		}
		/* Write is enabled ... now write eeprom value.
		 */
#endif
		if (i2c_write (addr[0], offset, alen-1, buffer, len) != 0)
			rcode = 1;

#endif
		buffer += len;
		offset += len;

#if defined(CFG_EEPROM_PAGE_WRITE_DELAY_MS)
		udelay(CFG_EEPROM_PAGE_WRITE_DELAY_MS * 1000);
#endif
	}
	return rcode;
}

#ifndef CONFIG_SPI
int
eeprom_probe (unsigned dev_addr, unsigned offset)
{
	unsigned char chip;

	/* Probe the chip address
	 */
#if CFG_I2C_EEPROM_ADDR_LEN == 1 && !defined(CONFIG_SPI_X)
	chip = offset >> 8;		/* block number */
#else
	chip = offset >> 16;		/* block number */
#endif	/* CFG_I2C_EEPROM_ADDR_LEN, CONFIG_SPI_X */

	chip |= dev_addr;		/* insert device address */

	return (i2c_probe (chip));
}
#endif

/*-----------------------------------------------------------------------
 * Set default values
 */
#ifndef	CFG_I2C_SPEED
#define	CFG_I2C_SPEED	50000
#endif

#ifndef	CFG_I2C_SLAVE
#define	CFG_I2C_SLAVE	0xFE
#endif

void eeprom_init  (void)
{
#if defined(CONFIG_SPI)
	spi_init_f ();
#endif
#if defined(CONFIG_HARD_I2C) || \
    defined(CONFIG_SOFT_I2C)
	i2c_init (CFG_I2C_SPEED, CFG_I2C_SLAVE);
#endif
}
/*-----------------------------------------------------------------------
 */
#endif	/* CFG_CMD_EEPROM */
/***************************************************/

#if (CONFIG_COMMANDS & CFG_CMD_EEPROM)

#ifdef CFG_I2C_MULTI_EEPROMS
U_BOOT_CMD(
	eeprom,	6,	1,	do_eeprom,
	"eeprom  - EEPROM sub-system\n",
	"read  devaddr addr off cnt\n"
	"eeprom write devaddr addr off cnt\n"
	"       - read/write `cnt' bytes from `devaddr` EEPROM at offset `off'\n"
);
#else /* One EEPROM */
U_BOOT_CMD(
	eeprom,	5,	1,	do_eeprom,
	"eeprom  - EEPROM sub-system\n",
	"read  addr off cnt\n"
	"eeprom write addr off cnt\n"
	"       - read/write `cnt' bytes at EEPROM offset `off' to/from memory address 'addr'\n"
);
#endif /* CFG_I2C_MULTI_EEPROMS */



#if (CONFIG_OCTEON_EEPROM_TUPLES)


#include <octeon_eeprom_types.h>
#include <lib_octeon.h>
#include <lib_octeon_shared.h>



int write_tlv_tuple(octeon_eeprom_header_t *tlv_hdr_ptr)
{


    uint16_t type = tlv_hdr_ptr->type;
    uint8_t major_version = tlv_hdr_ptr->major_version;
    uint16_t checksum = 0;
    int i;
    uint8_t *eeprom_buf = (void *)tlv_hdr_ptr;
    uint8_t tmp_buf[OCTEON_EEPROM_MAX_TUPLE_LENGTH];

    tlv_hdr_ptr->checksum = 0;
    for (i = 0; i < tlv_hdr_ptr->length; i++)
        checksum += eeprom_buf[i];

    tlv_hdr_ptr->checksum = checksum;


    int addr;

    addr = octeon_tlv_get_tuple_addr(CFG_DEF_EEPROM_ADDR, type, major_version, tmp_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);
    /* Mask addr since it might be negative if no existing tuple found */
    addr &= 0xffff;
    printf("Writing tuple type %d to addr: 0x%x\n", type, addr);

#if 0
    for (i = 0; i < tlv_hdr_ptr->length; i++)
    {
        printf("cd[0x%02x]:0x%02x\n", i, eeprom_buf[i]);
    }
#endif

    if (eeprom_write (CFG_DEF_EEPROM_ADDR, addr, (uchar *) eeprom_buf, tlv_hdr_ptr->length))
    {
        printf("Error writing tuple type %d to addr: 0x%x\n", type, addr);
        return(-1);
    }
    return(0);
}



/* Deletes a tuple at a given address in the eeprom */
int delete_tuple(int delete_addr)
{

    /* only used for one structure at a time */
    uint16_t eeprom_buf16[OCTEON_EEPROM_MAX_TUPLE_LENGTH/2 + 1];
    uint8_t *eeprom_buf = (void *)eeprom_buf16;
    octeon_eeprom_header_t *tlv_hdr_ptr = (void *)eeprom_buf;
    int delete_len = octeon_tlv_eeprom_get_next_tuple(CFG_DEF_EEPROM_ADDR, delete_addr, eeprom_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH);

    if (delete_len < 0)
        return -1;

    printf("Deleting %s (0x%x) tuple at address 0x%x, length %d\n", eeprom_type_to_string(tlv_hdr_ptr->type), tlv_hdr_ptr->type, delete_addr, delete_len);
    /* Erase the tuple that is to be deleted */
    memset(eeprom_buf, 0xff, delete_len);
    eeprom_write (CFG_DEF_EEPROM_ADDR, delete_addr, (uchar *) eeprom_buf, delete_len);


    /* Now copy any tuples after the deleted one to keep the tuples packet as this is required */
    int write_addr = delete_addr;
    int read_addr = delete_addr + delete_len;
    int cur_len;
    while ((cur_len = octeon_tlv_eeprom_get_next_tuple(CFG_DEF_EEPROM_ADDR, read_addr, eeprom_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH)) > 0)
    {
        /* We have read the next tuple we want to keep into the eeprom_buf, so now write it over the tuple
        ** we want to delete */
        printf("Copying tuple type %d (len 0x%x)to addr: 0x%x\n", tlv_hdr_ptr->type, tlv_hdr_ptr->length, write_addr);

        if (eeprom_write (CFG_DEF_EEPROM_ADDR, write_addr, (uchar *) eeprom_buf, tlv_hdr_ptr->length))
        {
            printf("Error writing tuple type %d to addr: 0x%x\n", tlv_hdr_ptr->type, write_addr);
            return(-1);
        }
        if (read_addr >= write_addr + cur_len)
        {
            /* Erase TLV header what we just copied if the copy did not overwrite it. */
            memset(eeprom_buf, 0xff, 8);
            eeprom_write (CFG_DEF_EEPROM_ADDR, read_addr, (uchar *) eeprom_buf, 8);
        }
        read_addr += cur_len;
        write_addr += cur_len;
    }

    return 0;
}

#define MIN_TLV_COMMAND_CHARS   3
int do_tlv_eeprom ( cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{

    DECLARE_GLOBAL_DATA_PTR;
    /* only used for one structure at a time */
    uint16_t eeprom_buf16[OCTEON_EEPROM_MAX_TUPLE_LENGTH/2 + 1];
    uint8_t *eeprom_buf = (void *)eeprom_buf16;

    /* Provide a number of different pointer types to access the eeprom buffer.
    ** There is only one user of the buffer at a time.
    */
    octeon_eeprom_header_t *tlv_hdr_ptr = (void *)eeprom_buf;
    octeon_eeprom_clock_desc_t *cd_ptr = (void *)eeprom_buf;
    octeon_eeprom_board_desc_t *bd_ptr = (void *)eeprom_buf;
    octeon_eeprom_chip_capability_t *cc_ptr = (void *)eeprom_buf;
    octeon_eeprom_mac_addr_t *ma_ptr = (void *)eeprom_buf;
    octeon_eeprom_voltage_multiplier_t *vm_ptr = (void *)eeprom_buf;
    octeon_eeprom_multiplier_desc_t *mult_ptr = (void *)eeprom_buf;
    octeon_eeprom_voltage_desc_t *volt_ptr = (void *)eeprom_buf;
    octeon_eeprom_ddr_clock_desc_t *ddr_clock_ptr = (void *)eeprom_buf;

    uint16_t addr = 0;
#ifdef   CONFIG_EEPROM_TLV_BASE_ADDRESS
    addr = CONFIG_EEPROM_TLV_BASE_ADDRESS;
    printf("CONFIG_EEPROM_TLV_BASE_ADDRESS: 0x%x\n", CONFIG_EEPROM_TLV_BASE_ADDRESS);
#endif
    int len;

    eeprom_init ();

    if (argc < 2 || (argc == 2 && !strcmp(argv[1], "help")) || strlen(argv[1]) < MIN_TLV_COMMAND_CHARS)
    {
        puts(cmdtp->help);
        /* Print out board type table here */
        int i;
        printf("Board types: \n");
        for (i = 1; i < CVMX_BOARD_TYPE_MAX; i++)
        {
            printf("%20s\t", cvmx_board_type_to_string(i));
            if ((i - 1)%3 == 2)
                printf("\n");
        }
        printf("\n");
        return 1;
    }
    if (strncmp (argv[1], "delete", strlen(argv[1])) == 0)
    {
        int delete_addr = simple_strtoul (argv[2], NULL, 0);

        if (delete_tuple(delete_addr) < 0)
        {
            printf("Error deleting tuple (address %d (0x%x).\n", delete_addr, delete_addr);
            return -1;
        }
    }
    else if (strncmp (argv[1], "display", strlen(argv[1])) == 0) {

        while ((len = octeon_tlv_eeprom_get_next_tuple(CFG_DEF_EEPROM_ADDR, addr, eeprom_buf, OCTEON_EEPROM_MAX_TUPLE_LENGTH)) > 0)
        {

            printf("==========================================\n");
            printf("%s (0x%x) tuple found: at addr 0x%x\n", eeprom_type_to_string(tlv_hdr_ptr->type), tlv_hdr_ptr->type, addr);
            printf("type: 0x%x, len: 0x%x, csum: 0x%x, maj_ver: %d, min_ver: %d\n",
                   tlv_hdr_ptr->type, tlv_hdr_ptr->length, tlv_hdr_ptr->checksum, tlv_hdr_ptr->major_version, tlv_hdr_ptr->minor_version);
            switch (tlv_hdr_ptr->type)
            {

            case EEPROM_CLOCK_DESC_TYPE:
                if (tlv_hdr_ptr->major_version > OCTEON_EEPROM_CLOCK_DESC_MAJOR_VER)
                {
                    printf("WARNING: tuple version mismatch, display may be wrong. (compiled for version: %d)\n",OCTEON_EEPROM_CLOCK_DESC_MAJOR_VER); 
                }


                if (tlv_hdr_ptr->major_version == 1)
                {
                    struct octeon_eeprom_clock_desc_v1 clock_v1;
                    memcpy(&clock_v1, tlv_hdr_ptr, sizeof(clock_v1));
                    printf("DDR clock: %d Mhz (raw: 0x%x)\n", clock_v1.ddr_clock_mhz, clock_v1.ddr_clock_mhz);
                    printf("CPU ref clock: %d Mhz (raw: 0x%x)\n", clock_v1.cpu_ref_clock_mhz_x_8 >> 3, clock_v1.cpu_ref_clock_mhz_x_8);
                    break;
                }

                printf("DDR clock: %d Mhz (raw: 0x%x)\n", cd_ptr->ddr_clock_mhz, cd_ptr->ddr_clock_mhz);
                printf("CPU ref clock: %d Mhz (raw: 0x%x)\n", cd_ptr->cpu_ref_clock_mhz_x_8>> 3, cd_ptr->cpu_ref_clock_mhz_x_8);
                printf("DFA ref clock: %d Mhz (raw: 0x%x)\n", cd_ptr->dfa_ref_clock_mhz_x_8 >> 3, cd_ptr->dfa_ref_clock_mhz_x_8);
                break;

            case EEPROM_BOARD_DESC_TYPE:
                if (tlv_hdr_ptr->major_version != OCTEON_EEPROM_BOARD_DESC_MAJOR_VER)
                {
                    printf("WARNING: tuple version mismatch, display may be wrong. (compiled for version: %d)\n",OCTEON_EEPROM_BOARD_DESC_MAJOR_VER); 
                }

                printf("Board type: %s (0x%x) \n", cvmx_board_type_to_string(bd_ptr->board_type), bd_ptr->board_type);
                printf("Board revision major:%d, minor:%d\n", bd_ptr->rev_major, bd_ptr->rev_minor);
                bd_ptr->serial_str[OCTEON_SERIAL_LEN -1] = 0;
                printf("Board ser #: %s\n", bd_ptr->serial_str);



                break;

            case EEPROM_CHIP_CAPABILITY_TYPE:
                printf("Coremask: 0x%04x, voltage_x100: %d, cpu_freq_mhz: %d\n",
                       cc_ptr->coremask, cc_ptr->voltage_x100, cc_ptr->cpu_freq_mhz);


                break;
            case EEPROM_MAC_ADDR_TYPE:
                printf("MAC base: %02x:%02x:%02x:%02x:%02x:%02x, count: %d\n",
                       ma_ptr->mac_addr_base[0],
                       ma_ptr->mac_addr_base[1],
                       ma_ptr->mac_addr_base[2],
                       ma_ptr->mac_addr_base[3],
                       ma_ptr->mac_addr_base[4],
                       ma_ptr->mac_addr_base[5],
                       ma_ptr->count);
                break;
            case EEPROM_VOLT_MULT_TYPE:
                printf("Voltage: %d millivolts\n", vm_ptr->voltage_millivolts);
                printf("CPU multiplier: %d\n", vm_ptr->cpu_multiplier);
                break;
            case EEPROM_DDR_CLOCK_DESC_TYPE:
                printf("DDR ref Hz: %d, clock Hz: %d\n", ddr_clock_ptr->ddr_ref_hz, ddr_clock_ptr->ddr_clock_hz);
                break;
            case EEPROM_VOLTAGE_TYPE:
                printf("CPU voltage: %d Millivolts\n", volt_ptr->octeon_core_voltage_mv);
                break;

            case EEPROM_MULTIPLIER_TYPE:
                printf("CPU Multiplier: %d (0x%x), IO Multiplier: %d (0x%x)\n", mult_ptr->octeon_core_mult, mult_ptr->octeon_core_mult,
                       mult_ptr->octeon_io_mult, mult_ptr->octeon_io_mult);
                break;
            case EEPROM_DEFAULT_MODULE_CAPABILITY_TYPE:
                break;
            case EEPROM_DELETE_ME:
                
                break;




            default:
                    /* Default case handled outside of switch statement */
                break;

            }

            addr += len;
        }
        if (addr)
            printf("==========================================\n");

    } else if ((strncmp (argv[1], "set", strlen(argv[1])) == 0) && strlen(argv[2]) >= MIN_TLV_COMMAND_CHARS && argc > 3) {
        memset(eeprom_buf, 0, OCTEON_EEPROM_MAX_TUPLE_LENGTH);


        if (strncmp (argv[2], "clock", strlen(argv[2])) == 0 && argc == (3 + 4))
        {
            if (OCTEON_IS_MODEL(OCTEON_CN6XXX))
            {
                printf("ERROR: this tuple type not supported on CN6XXX)\n");
                printf("       use the ddrclock tuple to set ddr frequency.\n");
                return(1);
            }

            printf("Setting clock descriptor...\n");
            tlv_hdr_ptr->type = EEPROM_CLOCK_DESC_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_clock_desc_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_CLOCK_DESC_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_CLOCK_DESC_MINOR_VER;


            cd_ptr->ddr_clock_mhz  = simple_strtoul (argv[3], NULL, 0);
            cd_ptr->cpu_ref_clock_mhz_x_8  = simple_strtoul (argv[4], NULL, 0);
            cd_ptr->dfa_ref_clock_mhz_x_8  = simple_strtoul (argv[5], NULL, 0);
            cd_ptr->spi_clock_mhz  = simple_strtoul (argv[6], NULL, 0);

            printf("ddr: %d, cpu ref: %d, dfa ref: %d spi: %d\n", 
                   cd_ptr->ddr_clock_mhz, 
                   cd_ptr->cpu_ref_clock_mhz_x_8, 
                   cd_ptr->dfa_ref_clock_mhz_x_8, 
                   cd_ptr->spi_clock_mhz);


            if ((gd->board_desc.board_type == CVMX_BOARD_TYPE_EBT3000 && gd->mcu_rev_maj <= 2 && gd->mcu_rev_min <= 11)
                || (gd->board_desc.board_type == CVMX_BOARD_TYPE_EBH3000 && gd->mcu_rev_maj < 1 && gd->mcu_rev_min < 15))
            {
                /* MCU rev 2.11 and lower only understand rev 1 clock descriptors. */
                printf("Using version 1 clock descriptor for ebt3000 board\n");
                struct octeon_eeprom_clock_desc_v1 clock_v1;
                clock_v1.header.type = EEPROM_CLOCK_DESC_TYPE;
                clock_v1.header.major_version = 1;
                clock_v1.header.minor_version = 0;
                clock_v1.header.length = sizeof(clock_v1);
                clock_v1.ddr_clock_mhz = cd_ptr->ddr_clock_mhz;
                clock_v1.cpu_ref_clock_mhz_x_8 = cd_ptr->cpu_ref_clock_mhz_x_8;
                clock_v1.spi_clock_mhz = cd_ptr->spi_clock_mhz;
                write_tlv_tuple((octeon_eeprom_header_t *)&clock_v1);
            }
            else
            {
                write_tlv_tuple(tlv_hdr_ptr);
            }

        } 
        else if (strncmp (argv[2], "board", strlen(argv[2])) == 0 && argc == (3 + 4))
        {
            int i;

            // bd_type bd_rv_mj bd_rv_mn chip_type ch_rv_mj ch_rv_mn serial_str


            printf("Setting board descriptor...\n");
            tlv_hdr_ptr->type = EEPROM_BOARD_DESC_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_board_desc_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_BOARD_DESC_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_BOARD_DESC_MINOR_VER;


            bd_ptr->board_type = 0;
            for (i = 1; i <CVMX_BOARD_TYPE_MAX; i++)
            {
                if (!strnicmp(argv[3], cvmx_board_type_to_string(i), 100))
                {
                    bd_ptr->board_type = i;
                    break;
                }
            }
            if (!bd_ptr->board_type)
            {
                printf("ERROR: unrecognized board name: %s\n", argv[3]);
                return(0);
            }

            bd_ptr->rev_major      = simple_strtoul (argv[4], NULL, 0);
            bd_ptr->rev_minor      = simple_strtoul (argv[5], NULL, 0);
            memset(bd_ptr->serial_str, 0, OCTEON_SERIAL_LEN);
            strncpy((char *)bd_ptr->serial_str, argv[6], OCTEON_SERIAL_LEN);

            printf("Board type: %d, rev: %d.%d, serial: %s\n",
                   bd_ptr->board_type,    
                   bd_ptr->rev_major,     
                   bd_ptr->rev_minor,     
                   bd_ptr->serial_str);

            write_tlv_tuple(tlv_hdr_ptr);
        }
        else if (strncmp (argv[2], "capability", strlen(argv[2])) == 0 && argc <= (3 + 3))
        {
            printf("Setting chip capability descriptor...\n");
            tlv_hdr_ptr->type = EEPROM_CHIP_CAPABILITY_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_chip_capability_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_CHIP_CAPABILITY_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_CHIP_CAPABILITY_MINOR_VER;

            cc_ptr->coremask     = simple_strtoul (argv[3], NULL, 16);

            write_tlv_tuple(tlv_hdr_ptr);

        }
        else if (strncmp (argv[2], "mac", strlen(argv[2])) == 0 && argc == (3 + 2))
        {
            int i;
            printf("Setting mac address descriptor...\n");
            tlv_hdr_ptr->type = EEPROM_MAC_ADDR_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_mac_addr_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_MAC_ADDR_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_MAC_ADDR_MINOR_VER;

        	char *tmp = argv[3];
        	char *end;
        
        	for (i=0; i<6; i++) {
        		ma_ptr->mac_addr_base[i] = tmp ? simple_strtoul(tmp, &end, 16) : 0;
        		if (tmp)
        			tmp = (*end) ? end+1 : end;
        	}


            ma_ptr->count     = simple_strtoul (argv[4], NULL, 0);

            printf("Setting mac count to: %d\n", ma_ptr->count);

            write_tlv_tuple(tlv_hdr_ptr);
        }
        else if (strncmp (argv[2], "voltmult", 8) == 0 && argc == (3 + 2))
        {
            if (OCTEON_IS_MODEL(OCTEON_CN6XXX))
            {
                printf("ERROR: this tuple type not supported on CN6XXX)\n");
                return(1);
            }
            tlv_hdr_ptr->type = EEPROM_VOLT_MULT_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_voltage_multiplier_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_VOLT_MULT_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_VOLT_MULT_MINOR_VER;

            vm_ptr->voltage_millivolts     = simple_strtoul (argv[3], NULL, 0);
            vm_ptr->cpu_multiplier     = simple_strtoul (argv[4], NULL, 0);
            printf("Setting voltage to %d millivolts and cpu multiplier to %d\n",
                   vm_ptr->voltage_millivolts, vm_ptr->cpu_multiplier);
            write_tlv_tuple(tlv_hdr_ptr);
        }
        else if (strncmp (argv[2], "volt", 4) == 0 && argc == (3 + 1))
        {
            if (!OCTEON_IS_MODEL(OCTEON_CN6XXX))
            {
                printf("ERROR: this tuple type is only supported on CN6XXX)\n");
                return(1);
            }
            tlv_hdr_ptr->type = EEPROM_VOLTAGE_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_voltage_desc_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_VOLTAGE_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_VOLTAGE_MINOR_VER;

            volt_ptr->octeon_core_voltage_mv = simple_strtoul (argv[3], NULL, 0);
            printf("Setting core voltage to %d millivolts\n", vm_ptr->voltage_millivolts);
            write_tlv_tuple(tlv_hdr_ptr);
        }
        else if (strncmp (argv[2], "mult", 4) == 0 && argc == (3 + 2))
        {
            if (!OCTEON_IS_MODEL(OCTEON_CN6XXX))
            {
                printf("ERROR: this tuple type is only supported on CN6XXX)\n");
                return(1);
            }
            tlv_hdr_ptr->type = EEPROM_MULTIPLIER_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_multiplier_desc_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_MULTIPLIER_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_MULTIPLIER_MINOR_VER;

            mult_ptr->octeon_core_mult = simple_strtoul (argv[3], NULL, 0);
            mult_ptr->octeon_io_mult = simple_strtoul (argv[4], NULL, 0);
            printf("Setting core multiplier to: %d, and IO multiplier to: %d\n",
                   mult_ptr->octeon_core_mult, mult_ptr->octeon_io_mult);

            write_tlv_tuple(tlv_hdr_ptr);
        }
        else if (strncmp (argv[2], "ddrclock", 8) == 0 && argc == (3 + 1))
        {
            if (!OCTEON_IS_MODEL(OCTEON_CN6XXX))
            {
                printf("ERROR: this tuple type is only supported on CN6XXX)\n");
                return(1);
            }
            tlv_hdr_ptr->type = EEPROM_DDR_CLOCK_DESC_TYPE;
            tlv_hdr_ptr->length = sizeof(octeon_eeprom_ddr_clock_desc_t);
            tlv_hdr_ptr->checksum = 0; 
            tlv_hdr_ptr->major_version = OCTEON_EEPROM_DDR_CLOCK_DESC_MAJOR_VER;
            tlv_hdr_ptr->minor_version = OCTEON_EEPROM_DDR_CLOCK_DESC_MINOR_VER;

            ddr_clock_ptr->ddr_ref_hz = 0; /* Use default */
            ddr_clock_ptr->ddr_clock_hz = simple_strtoul (argv[3], NULL, 0);
            printf("Setting ddr clock to: %d Hz\n",
                   ddr_clock_ptr->ddr_clock_hz);

            write_tlv_tuple(tlv_hdr_ptr);
        }
        else
        {
            puts(cmdtp->help);
            return 1;

        }



    }
    else
    {
        puts(cmdtp->help);
        return 1;

    }

    return(0);
}

U_BOOT_CMD(
	tlv_eeprom,	12,	1,	do_tlv_eeprom,
	"tlv_eeprom  - EEPROM data parsing for ebt3000 board\n",
	"tlv_eeprom [display|set|delete|help]\n"
	"tlv_eeprom display\n"
	"       - display contents of eeprom data structures\n"
	"tlv_eeprom set\n"
	"       - set contents of eeprom data structures, values in hex or decimal\n"
	"       set clock <ddr_clock in Mhz> <cpu_ref_clock in Mhz * 8> <dfa_ref_clock in Mhz * 8> <spi_clock in Mhz> (CN3XXX/CN5XXX)\n"
	"       set board <board type> <major rev> <minor rev>  <serial string>\n"
        "       set mac    <mac base x:x:x:x:x:x> <mac addr count>\n"
        "       set voltmult <voltage in millivolts> <cpu multiplier> (CN3XXX/CN5XXX)\n"
        "       set volt <core voltage in millivolts> (CN6XXX)\n"
        "       set mult <cpu multiplier> <IO multiplier> (CN6XXX)\n"
        "       set ddrclock <ddr clock Hz> (CN6XXX)\n"
        "       set capability [coremask_override]\n"
	"tlv_eeprom delete <tuple addr>\n"
	"       - deletes tuple at address and compacts any following tuples.\n"
        "\n"
        "Chip types have been deprecated, and can no longer be set.\n"
        "\n"
        "(use tlv_eeprom help for list of board types)\n"
        ""
);

#endif













#endif	/* CFG_CMD_EEPROM */
