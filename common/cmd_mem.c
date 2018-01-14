/*
 * (C) Copyright 2000
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
 */

/*
 * Memory Functions
 *
 * Copied from FADS ROM, Dan Malek (dmalek@jlc.net)
 */

#include <common.h>
#include <command.h>
#if (CONFIG_COMMANDS & CFG_CMD_MMC)
#include <mmc.h>
#endif
#ifdef CONFIG_HAS_DATAFLASH
#include <dataflash.h>
#endif

#include "octeon_boot.h"
#include "cvmx-bootmem.h"
#include "cvmx-swap.h"

#if (CONFIG_COMMANDS & (CFG_CMD_MEMORY	| \
			CFG_CMD_I2C	| \
			CFG_CMD_ITEST	| \
			CFG_CMD_PCI	| \
			CMD_CMD_PORTIO	) )
int cmd_get_data_size(char* arg, int default_size)
{
	/* Check for a size specification .b, .w, .l, or .d.
	 */
	int len = strlen(arg);
	if (len > 2 && arg[len-2] == '.') {
		switch(arg[len-1]) {
		case 'b':
			return 1;
		case 'w':
			return 2;
		case 'l':
			return 4;
		case 'd':
			return 8;
		case 's':
			return -2;
		default:
			return -1;
		}
	}
	return default_size;
}
#endif

#if (CONFIG_COMMANDS & CFG_CMD_MEMORY)

#ifdef	CMD_MEM_DEBUG
#define	PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define PRINTF(fmt,args...)
#endif

static int mod_mem(cmd_tbl_t *, int, int, int, char *[]);

/* Display values from last command.
 * Memory modify remembered values are different from display memory.
 */
uint	dp_last_addr, dp_last_size;
uint	dp_last_length = 0x40;
uint	mm_last_addr, mm_last_size;

static	ulong	base_address = 0;
static	uint64_t	base_address64 = (1ull << 63);

/* Memory Display
 *
 * Syntax:
 *	md{.b, .w, .l} {addr} {len}
 */
#define DISP_LINE_LEN	16
int do_mem_md ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong	addr, length;
	ulong	i, nbytes, linebytes;
	u_char	*cp;
	int	size;
	int rc = 0;

	/* We use the last specified parameters, unless new ones are
	 * entered.
	 */
	addr = dp_last_addr;
	size = dp_last_size;
	length = dp_last_length;

	if (argc < 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	if ((flag & CMD_FLAG_REPEAT) == 0) {
		/* New command specified.  Check for a size specification.
		 * Defaults to long if no or incorrect specification.
		 */
		if ((size = cmd_get_data_size(argv[0], 4)) < 0)
			return 1;

		/* Address is specified since argc > 1
		*/
		addr = simple_strtoul(argv[1], NULL, 16);
		addr += base_address;

		/* If another parameter, it is the length to display.
		 * Length is the number of objects, not number of bytes.
		 */
		if (argc > 2)
			length = simple_strtoul(argv[2], NULL, 16);
	}

	/* Print the lines.
	 *
	 * We buffer all read data, so we can make sure data is read only
	 * once, and all accesses are with the specified bus width.
	 */
	nbytes = length * size;
	do {
		char	linebuf[DISP_LINE_LEN];
		uint	*uip = (uint   *)linebuf;
		ushort	*usp = (ushort *)linebuf;
		u_char	*ucp = (u_char *)linebuf;
#ifdef CONFIG_HAS_DATAFLASH
		int rc;
#endif
		printf("%08lx:", addr);
		linebytes = (nbytes>DISP_LINE_LEN)?DISP_LINE_LEN:nbytes;

#ifdef CONFIG_HAS_DATAFLASH
		if ((rc = read_dataflash(addr, (linebytes/size)*size, linebuf)) == DATAFLASH_OK){
			/* if outside dataflash */
			/*if (rc != 1) {
				dataflash_perror (rc);
				return (1);
			}*/
			for (i=0; i<linebytes; i+= size) {
				if (size == 4) {
					printf(" %08x", *uip++);
				} else if (size == 2) {
					printf(" %04x", *usp++);
				} else {
					printf(" %02x", *ucp++);
				}
				addr += size;
			}

		} else {	/* addr does not correspond to DataFlash */
#endif
		for (i=0; i<linebytes; i+= size) {
			if (size == 4) {
				printf(" %08x", (*uip++ = *((uint *)addr)));
			} else if (size == 2) {
				printf(" %04x", (*usp++ = *((ushort *)addr)));
			} else {
				printf(" %02x", (*ucp++ = *((u_char *)addr)));
			}
			addr += size;
		}
#ifdef CONFIG_HAS_DATAFLASH
		}
#endif
		puts ("    ");
		cp = (uchar *)linebuf;
		for (i=0; i<linebytes; i++) {
			if ((*cp < 0x20) || (*cp > 0x7e))
				putc ('.');
			else
				printf("%c", *cp);
			cp++;
		}
		putc ('\n');
		nbytes -= linebytes;
		if (ctrlc()) {
			rc = 1;
			break;
		}
	} while (nbytes > 0);

	dp_last_addr = addr;
	dp_last_length = length;
	dp_last_size = size;
	return (rc);
}

int do_mem_mm ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return mod_mem (cmdtp, 1, flag, argc, argv);
}
int do_mem_nm ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return mod_mem (cmdtp, 0, flag, argc, argv);
}


int do_read_cmp ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 3)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint32_t addr;
    uint32_t val;
    uint32_t *ptr;
    addr = simple_strtoul(argv[1], NULL, 16);
    addr += base_address;

    val = simple_strtoul(argv[2], NULL, 16);
    printf("Looking for value other than 0x%x, addr: 0x%x\n", val, addr);
    ptr = (void *)addr;
    while (*ptr++ == val)
        ;
    ptr--;
    printf("Found 0x%x at addr %p\n", *ptr, ptr);

    return(0);
}

int do_read64 ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 2)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr, val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;

    printf("attempting to read from addr: 0x%llx\n", addr);
    val = cvmx_read_csr(addr);
    printf("0x%llx: 0x%llx\n", addr, val);

    return(0);
}
int do_read64b ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 2)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr;
    uint8_t val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;

    printf("attempting to read from addr: 0x%llx\n", addr);
    val = octeon_read64_byte(addr);
    printf("0x%llx: 0x%x\n", addr, val);

    return(0);
}
int do_read64l ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 2)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr;
    uint32_t val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;

    printf("attempting to read from addr: 0x%llx\n", addr);
    val = cvmx_read64_uint32(addr);
    printf("0x%llx: 0x%x\n", addr, val);

    return(0);
}
int do_read64s ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 2)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr;
    uint32_t val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;

    printf("attempting to read from addr: 0x%llx\n", addr);
    val = cvmx_read64_uint16(addr);
    printf("0x%llx: 0x%x\n", addr, val);

    return(0);
}

int do_write64 ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 3)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr, val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;
    val = simple_strtoull(argv[2], NULL, 16);

    printf("writing 0x%llx to addr: 0x%llx\n", val, addr);

    cvmx_write_csr(addr, val);
    return(0);
}

int do_write64b ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 3)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr;
    uint8_t val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;
    val = simple_strtoull(argv[2], NULL, 16);

    printf("writing 0x%x to addr: 0x%llx\n", val, addr);

    octeon_write64_byte(addr, val);
    return(0);
}
int do_write64s ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 3)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr;
    uint32_t val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;
    val = simple_strtoull(argv[2], NULL, 16);

    printf("writing 0x%x to addr: 0x%llx\n", val, addr);

    cvmx_write64_uint16(addr, val);
    return(0);
}
int do_write64l ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if ((argc != 3)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
    uint64_t addr;
    uint32_t val;
    addr = simple_strtoull(argv[1], NULL, 16);
    addr += base_address64;
    val = simple_strtoull(argv[2], NULL, 16);

    printf("writing 0x%x to addr: 0x%llx\n", val, addr);

    cvmx_write64_uint32(addr, val);
    return(0);
}


int do_mem_mw ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong	addr, writeval, count;
	int	size;

	if ((argc < 3) || (argc > 4)) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 1)
		return 1;

    if (size == 8)
    {
        uint64_t addr, writeval, count;
        addr = simple_strtoull(argv[1], NULL, 16);
        writeval = simple_strtoull(argv[2], NULL, 16);
        if (argc == 4) {
            count = simple_strtoull(argv[3], NULL, 16);
        } else {
            count = 1;
        }

        while (count-- > 0) {
            cvmx_write_csr(addr, writeval);
            addr += 8;
        }
    }
    else
    {
	/* Address is specified since argc > 1
	*/
	addr = simple_strtoul(argv[1], NULL, 16);
	addr += base_address;

	/* Get the value to write.
	*/
	writeval = simple_strtoul(argv[2], NULL, 16);


	/* Count ? */
	if (argc == 4) {
		count = simple_strtoul(argv[3], NULL, 16);
	} else {
		count = 1;
	}

	while (count-- > 0) {
		if (size == 4)
			*((ulong  *)addr) = (ulong )writeval;
		else if (size == 2)
			*((ushort *)addr) = (ushort)writeval;
		else
			*((u_char *)addr) = (u_char)writeval;
		addr += size;
	}
    }
    return 0;
}

int do_mem_cmp (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong	addr1, addr2, count, ngood;
	int	size;
	int     rcode = 0;

	if (argc != 4) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 0)
		return 1;

	addr1 = simple_strtoul(argv[1], NULL, 16);
	addr1 += base_address;

	addr2 = simple_strtoul(argv[2], NULL, 16);
	addr2 += base_address;

	count = simple_strtoul(argv[3], NULL, 16);

#ifdef CONFIG_HAS_DATAFLASH
	if (addr_dataflash(addr1) | addr_dataflash(addr2)){
		puts ("Comparison with DataFlash space not supported.\n\r");
		return 0;
	}
#endif

	ngood = 0;

	while (count-- > 0) {
		if (size == 4) {
			ulong word1 = *(ulong *)addr1;
			ulong word2 = *(ulong *)addr2;
			if (word1 != word2) {
				printf("word at 0x%08lx (0x%08lx) "
					"!= word at 0x%08lx (0x%08lx)\n",
					addr1, word1, addr2, word2);
				rcode = 1;
				break;
			}
		}
		else if (size == 2) {
			ushort hword1 = *(ushort *)addr1;
			ushort hword2 = *(ushort *)addr2;
			if (hword1 != hword2) {
				printf("halfword at 0x%08lx (0x%04x) "
					"!= halfword at 0x%08lx (0x%04x)\n",
					addr1, hword1, addr2, hword2);
				rcode = 1;
				break;
			}
		}
		else {
			u_char byte1 = *(u_char *)addr1;
			u_char byte2 = *(u_char *)addr2;
			if (byte1 != byte2) {
				printf("byte at 0x%08lx (0x%02x) "
					"!= byte at 0x%08lx (0x%02x)\n",
					addr1, byte1, addr2, byte2);
				rcode = 1;
				break;
			}
		}
		ngood++;
		addr1 += size;
		addr2 += size;
	}

	printf("Total of %ld %s%s were the same\n",
		ngood, size == 4 ? "word" : size == 2 ? "halfword" : "byte",
		ngood == 1 ? "" : "s");
	return rcode;
}

int do_mem_cp ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong	addr, dest, count;
	int	size;

	if (argc != 4) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 0)
		return 1;

	addr = simple_strtoul(argv[1], NULL, 16);
	addr += base_address;

	dest = simple_strtoul(argv[2], NULL, 16);
	dest += base_address;

	count = simple_strtoul(argv[3], NULL, 16);

	if (count == 0) {
		puts ("Zero length ???\n");
		return 1;
	}

#ifndef CFG_NO_FLASH
	/* check if we are copying to Flash */
	if ( (addr2info(dest) != NULL)
#ifdef CONFIG_HAS_DATAFLASH
	   && (!addr_dataflash(addr))
#endif
	   ) {
		int rc;

		puts ("Copy to Flash... ");

		rc = flash_write ((char *)addr, dest, count*size);
		if (rc != 0) {
			flash_perror (rc);
			return (1);
		}
		puts ("done\n");
		return 0;
	}
#endif

#if (CONFIG_COMMANDS & CFG_CMD_MMC)
	if (mmc2info(dest)) {
		int rc;

		puts ("Copy to MMC... ");
		switch (rc = mmc_write ((uchar *)addr, dest, count*size)) {
		case 0:
			putc ('\n');
			return 1;
		case -1:
			puts ("failed\n");
			return 1;
		default:
			printf ("%s[%d] FIXME: rc=%d\n",__FILE__,__LINE__,rc);
			return 1;
		}
		puts ("done\n");
		return 0;
	}

	if (mmc2info(addr)) {
		int rc;

		puts ("Copy from MMC... ");
		switch (rc = mmc_read (addr, (uchar *)dest, count*size)) {
		case 0:
			putc ('\n');
			return 1;
		case -1:
			puts ("failed\n");
			return 1;
		default:
			printf ("%s[%d] FIXME: rc=%d\n",__FILE__,__LINE__,rc);
			return 1;
		}
		puts ("done\n");
		return 0;
	}
#endif

#ifdef CONFIG_HAS_DATAFLASH
	/* Check if we are copying from RAM or Flash to DataFlash */
	if (addr_dataflash(dest) && !addr_dataflash(addr)){
		int rc;

		puts ("Copy to DataFlash... ");

		rc = write_dataflash (dest, addr, count*size);

		if (rc != 1) {
			dataflash_perror (rc);
			return (1);
		}
		puts ("done\n");
		return 0;
	}

	/* Check if we are copying from DataFlash to RAM */
	if (addr_dataflash(addr) && !addr_dataflash(dest) && (addr2info(dest)==NULL) ){
		int rc;
		rc = read_dataflash(addr, count * size, (char *) dest);
		if (rc != 1) {
			dataflash_perror (rc);
			return (1);
		}
		return 0;
	}

	if (addr_dataflash(addr) && addr_dataflash(dest)){
		puts ("Unsupported combination of source/destination.\n\r");
		return 1;
	}
#endif

	while (count-- > 0) {
		if (size == 4)
			*((ulong  *)dest) = *((ulong  *)addr);
		else if (size == 2)
			*((ushort *)dest) = *((ushort *)addr);
		else
			*((u_char *)dest) = *((u_char *)addr);
		addr += size;
		dest += size;
	}
	return 0;
}

int do_mem_base (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if (argc > 1) {
		/* Set new base address.
		*/
		base_address64 = simple_strtoull(argv[1], NULL, 16);
        base_address = (uint32_t)base_address64;
	}
	/* Print the current base address.
	*/
	printf("Base Address64: 0x%16qx, ", base_address64);
	printf("Base Address: 0x%08lx\n", base_address);
	return 0;
}

int do_mem_loop (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong	addr, length, i, junk;
	int	size;
	volatile uint	*longp;
	volatile ushort *shortp;
	volatile u_char	*cp;

	if (argc < 3) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	/* Check for a size spefication.
	 * Defaults to long if no or incorrect specification.
	 */
	if ((size = cmd_get_data_size(argv[0], 4)) < 0)
		return 1;

	/* Address is always specified.
	*/
	addr = simple_strtoul(argv[1], NULL, 16);

	/* Length is the number of objects, not number of bytes.
	*/
	length = simple_strtoul(argv[2], NULL, 16);

	/* We want to optimize the loops to run as fast as possible.
	 * If we have only one object, just run infinite loops.
	 */
	if (length == 1) {
		if (size == 4) {
			longp = (uint *)addr;
			for (;;)
				i = *longp;
		}
		if (size == 2) {
			shortp = (ushort *)addr;
			for (;;)
				i = *shortp;
		}
		cp = (u_char *)addr;
		for (;;)
			i = *cp;
	}

	if (size == 4) {
		for (;;) {
			longp = (uint *)addr;
			i = length;
			while (i-- > 0)
				junk = *longp++;
		}
	}
	if (size == 2) {
		for (;;) {
			shortp = (ushort *)addr;
			i = length;
			while (i-- > 0)
				junk = *shortp++;
		}
	}
	for (;;) {
		cp = (u_char *)addr;
		i = length;
		while (i-- > 0)
			junk = *cp++;
	}
}

/*
 * Perform a memory test. A more complete alternative test can be
 * configured using CFG_ALT_MEMTEST. The complete test loops until
 * interrupted by ctrl-c or by a failure of one of the sub-tests.
 */
int do_mem_mtest (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	vu_long	*addr, *start, *end;
	ulong	val;
	ulong	readback;

#if defined(CFG_ALT_MEMTEST)
	vu_long	addr_mask;
	vu_long	offset;
	vu_long	test_offset;
	vu_long	pattern;
	vu_long	temp;
	vu_long	anti_pattern;
	vu_long	num_words;
#if defined(CFG_MEMTEST_SCRATCH)
	vu_long *dummy = (vu_long*)CFG_MEMTEST_SCRATCH;
#else
	vu_long *dummy = NULL;
#endif
	int	j;
	int iterations = 1;

	static const ulong bitpattern[] = {
		0x00000001,	/* single bit */
		0x00000003,	/* two adjacent bits */
		0x00000007,	/* three adjacent bits */
		0x0000000F,	/* four adjacent bits */
		0x00000005,	/* two non-adjacent bits */
		0x00000015,	/* three non-adjacent bits */
		0x00000055,	/* four non-adjacent bits */
		0xaaaaaaaa,	/* alternating 1/0 */
	};
#else
	ulong	incr;
	ulong	pattern;
	int     rcode = 0;
#endif

	if (argc > 1) {
		start = (ulong *)simple_strtoul(argv[1], NULL, 16);
	} else {
		start = (ulong *)CFG_MEMTEST_START;
	}

	if (argc > 2) {
		end = (ulong *)simple_strtoul(argv[2], NULL, 16);
	} else {
		end = (ulong *)(CFG_MEMTEST_END);
	}

	if (argc > 3) {
		pattern = (ulong)simple_strtoul(argv[3], NULL, 16);
	} else {
		pattern = 0;
	}

#if defined(CFG_ALT_MEMTEST)
	printf ("Testing %08x ... %08x:\n", (uint)start, (uint)end);
	PRINTF("%s:%d: start 0x%p end 0x%p\n",
		__FUNCTION__, __LINE__, start, end);

	for (;;) {
		if (ctrlc()) {
			putc ('\n');
			return 1;
		}

		printf("Iteration: %6d\r", iterations);
		PRINTF("Iteration: %6d\n", iterations);
		iterations++;

		/*
		 * Data line test: write a pattern to the first
		 * location, write the 1's complement to a 'parking'
		 * address (changes the state of the data bus so a
		 * floating bus doen't give a false OK), and then
		 * read the value back. Note that we read it back
		 * into a variable because the next time we read it,
		 * it might be right (been there, tough to explain to
		 * the quality guys why it prints a failure when the
		 * "is" and "should be" are obviously the same in the
		 * error message).
		 *
		 * Rather than exhaustively testing, we test some
		 * patterns by shifting '1' bits through a field of
		 * '0's and '0' bits through a field of '1's (i.e.
		 * pattern and ~pattern).
		 */
		addr = start;
		for (j = 0; j < sizeof(bitpattern)/sizeof(bitpattern[0]); j++) {
		    val = bitpattern[j];
		    for(; val != 0; val <<= 1) {
			*addr  = val;
			*dummy  = ~val; /* clear the test data off of the bus */
			readback = *addr;
			if(readback != val) {
			     printf ("FAILURE (data line): "
				"expected %08lx, actual %08lx\n",
					  val, readback);
			}
			*addr  = ~val;
			*dummy  = val;
			readback = *addr;
			if(readback != ~val) {
			    printf ("FAILURE (data line): "
				"Is %08lx, should be %08lx\n",
					val, readback);
			}
		    }
		}

		/*
		 * Based on code whose Original Author and Copyright
		 * information follows: Copyright (c) 1998 by Michael
		 * Barr. This software is placed into the public
		 * domain and may be used for any purpose. However,
		 * this notice must not be changed or removed and no
		 * warranty is either expressed or implied by its
		 * publication or distribution.
		 */

		/*
		 * Address line test
		 *
		 * Description: Test the address bus wiring in a
		 *              memory region by performing a walking
		 *              1's test on the relevant bits of the
		 *              address and checking for aliasing.
		 *              This test will find single-bit
		 *              address failures such as stuck -high,
		 *              stuck-low, and shorted pins. The base
		 *              address and size of the region are
		 *              selected by the caller.
		 *
		 * Notes:	For best results, the selected base
		 *              address should have enough LSB 0's to
		 *              guarantee single address bit changes.
		 *              For example, to test a 64-Kbyte
		 *              region, select a base address on a
		 *              64-Kbyte boundary. Also, select the
		 *              region size as a power-of-two if at
		 *              all possible.
		 *
		 * Returns:     0 if the test succeeds, 1 if the test fails.
		 *
		 * ## NOTE ##	Be sure to specify start and end
		 *              addresses such that addr_mask has
		 *              lots of bits set. For example an
		 *              address range of 01000000 02000000 is
		 *              bad while a range of 01000000
		 *              01ffffff is perfect.
		 */
		addr_mask = ((ulong)end - (ulong)start)/sizeof(vu_long);
		pattern = (vu_long) 0xaaaaaaaa;
		anti_pattern = (vu_long) 0x55555555;

		PRINTF("%s:%d: addr mask = 0x%.8lx\n",
			__FUNCTION__, __LINE__,
			addr_mask);
		/*
		 * Write the default pattern at each of the
		 * power-of-two offsets.
		 */
		for (offset = 1; (offset & addr_mask) != 0; offset <<= 1) {
			start[offset] = pattern;
		}

		/*
		 * Check for address bits stuck high.
		 */
		test_offset = 0;
		start[test_offset] = anti_pattern;

		for (offset = 1; (offset & addr_mask) != 0; offset <<= 1) {
		    temp = start[offset];
		    if (temp != pattern) {
			printf ("\nFAILURE: Address bit stuck high @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx\n",
				(ulong)&start[offset], pattern, temp);
			return 1;
		    }
		}
		start[test_offset] = pattern;

		/*
		 * Check for addr bits stuck low or shorted.
		 */
		for (test_offset = 1; (test_offset & addr_mask) != 0; test_offset <<= 1) {
		    start[test_offset] = anti_pattern;

		    for (offset = 1; (offset & addr_mask) != 0; offset <<= 1) {
			temp = start[offset];
			if ((temp != pattern) && (offset != test_offset)) {
			    printf ("\nFAILURE: Address bit stuck low or shorted @"
				" 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx\n",
				(ulong)&start[offset], pattern, temp);
			    return 1;
			}
		    }
		    start[test_offset] = pattern;
		}

		/*
		 * Description: Test the integrity of a physical
		 *		memory device by performing an
		 *		increment/decrement test over the
		 *		entire region. In the process every
		 *		storage bit in the device is tested
		 *		as a zero and a one. The base address
		 *		and the size of the region are
		 *		selected by the caller.
		 *
		 * Returns:     0 if the test succeeds, 1 if the test fails.
		 */
		num_words = ((ulong)end - (ulong)start)/sizeof(vu_long) + 1;

		/*
		 * Fill memory with a known pattern.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
			start[offset] = pattern;
		}

		/*
		 * Check each location and invert it for the second pass.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
		    temp = start[offset];
		    if (temp != pattern) {
			printf ("\nFAILURE (read/write) @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx)\n",
				(ulong)&start[offset], pattern, temp);
			return 1;
		    }

		    anti_pattern = ~pattern;
		    start[offset] = anti_pattern;
		}

		/*
		 * Check each location for the inverted pattern and zero it.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
		    anti_pattern = ~pattern;
		    temp = start[offset];
		    if (temp != anti_pattern) {
			printf ("\nFAILURE (read/write): @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx)\n",
				(ulong)&start[offset], anti_pattern, temp);
			return 1;
		    }
		    start[offset] = 0;
		}
	}

#else /* The original, quickie test */
	incr = 1;
	for (;;) {
            int max_print = 50;
		if (ctrlc()) {
			putc ('\n');
			return 1;
		}

		printf ("\rPattern %08lX  Writing..."
			"%12s"
			"\b\b\b\b\b\b\b\b\b\b",
			pattern, "");

		for (addr=start,val=pattern; addr<end; addr++) {
			*addr = val;
			val  += incr;
		}

		puts ("Reading...");

		for (addr=start,val=pattern; addr<end; addr++) {
			readback = *addr;
			if (readback != val) {
				printf ("\nMem error @ 0x%08X: "
					"found %08lX, expected %08lX\n",
					(uint)addr, readback, val);
				rcode = 1;
                                if (max_print-- <= 0)
                                {
                                    printf("Too many errors, aborting test\n");
                                    octeon_check_mem_errors();
                                    return(1);
                                }
			}
			val += incr;
		}

		/*
		 * Flip the pattern each time to make lots of zeros and
		 * then, the next time, lots of ones.  We decrement
		 * the "negative" patterns and increment the "positive"
		 * patterns to preserve this feature.
		 */
		if(pattern & 0x80000000) {
			pattern = -pattern;	/* complement & increment */
		}
		else {
			pattern = ~pattern;
		}
		incr = -incr;

                octeon_check_mem_errors();

	}
	return rcode;
#endif
}


/* Modify memory.
 *
 * Syntax:
 *	mm{.b, .w, .l} {addr}
 *	nm{.b, .w, .l} {addr}
 */
static int
mod_mem(cmd_tbl_t *cmdtp, int incrflag, int flag, int argc, char *argv[])
{
	ulong	addr, i;
	int	nbytes, size;
	extern char console_buffer[];

	if (argc != 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

#ifdef CONFIG_BOOT_RETRY_TIME
	reset_cmd_timeout();	/* got a good command to get here */
#endif
	/* We use the last specified parameters, unless new ones are
	 * entered.
	 */
	addr = mm_last_addr;
	size = mm_last_size;

	if ((flag & CMD_FLAG_REPEAT) == 0) {
		/* New command specified.  Check for a size specification.
		 * Defaults to long if no or incorrect specification.
		 */
		if ((size = cmd_get_data_size(argv[0], 4)) < 0)
			return 1;

		/* Address is specified since argc > 1
		*/
		addr = simple_strtoul(argv[1], NULL, 16);
		addr += base_address;
	}

#ifdef CONFIG_HAS_DATAFLASH
	if (addr_dataflash(addr)){
		puts ("Can't modify DataFlash in place. Use cp instead.\n\r");
		return 0;
	}
#endif

	/* Print the address, followed by value.  Then accept input for
	 * the next value.  A non-converted value exits.
	 */
	do {
		printf("%08lx:", addr);
		if (size == 4)
			printf(" %08x", *((uint   *)addr));
		else if (size == 2)
			printf(" %04x", *((ushort *)addr));
		else
			printf(" %02x", *((u_char *)addr));

		nbytes = readline (" ? ");
		if (nbytes == 0 || (nbytes == 1 && console_buffer[0] == '-')) {
			/* <CR> pressed as only input, don't modify current
			 * location and move to next. "-" pressed will go back.
			 */
			if (incrflag)
				addr += nbytes ? -size : size;
			nbytes = 1;
#ifdef CONFIG_BOOT_RETRY_TIME
			reset_cmd_timeout(); /* good enough to not time out */
#endif
		}
#ifdef CONFIG_BOOT_RETRY_TIME
		else if (nbytes == -2) {
			break;	/* timed out, exit the command	*/
		}
#endif
		else {
			char *endp;
			i = simple_strtoul(console_buffer, &endp, 16);
			nbytes = endp - console_buffer;
			if (nbytes) {
#ifdef CONFIG_BOOT_RETRY_TIME
				/* good enough to not time out
				 */
				reset_cmd_timeout();
#endif
				if (size == 4)
					*((uint   *)addr) = i;
				else if (size == 2)
					*((ushort *)addr) = i;
				else
					*((u_char *)addr) = i;
				if (incrflag)
					addr += size;
			}
		}
	} while (nbytes);

	mm_last_addr = addr;
	mm_last_size = size;
	return 0;
}

#ifndef CONFIG_CRC32_VERIFY

int do_mem_crc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong addr, length;
	ulong crc;
	ulong *ptr;

	if (argc < 3) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	addr = simple_strtoul (argv[1], NULL, 16);
	addr += base_address;

	length = simple_strtoul (argv[2], NULL, 16);

	crc = crc32 (0, (const uchar *) addr, length);

	printf ("CRC32 for %08lx ... %08lx ==> %08lx\n",
			addr, addr + length - 1, crc);

	if (argc > 3) {
		ptr = (ulong *) simple_strtoul (argv[3], NULL, 16);
		*ptr = crc;
	}

	return 0;
}

#else	/* CONFIG_CRC32_VERIFY */

int do_mem_crc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong addr, length;
	ulong crc;
	ulong *ptr;
	ulong vcrc;
	int verify;
	int ac;
	char **av;

	if (argc < 3) {
  usage:
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	av = argv + 1;
	ac = argc - 1;
	if (strcmp(*av, "-v") == 0) {
		verify = 1;
		av++;
		ac--;
		if (ac < 3)
			goto usage;
	} else
		verify = 0;

	addr = simple_strtoul(*av++, NULL, 16);
	addr += base_address;
	length = simple_strtoul(*av++, NULL, 16);

	crc = crc32(0, (const uchar *) addr, length);

	if (!verify) {
		printf ("CRC32 for %08lx ... %08lx ==> %08lx\n",
				addr, addr + length - 1, crc);
		if (ac > 2) {
			ptr = (ulong *) simple_strtoul (*av++, NULL, 16);
			*ptr = crc;
		}
	} else {
		vcrc = simple_strtoul(*av++, NULL, 16);
		if (vcrc != crc) {
			printf ("CRC32 for %08lx ... %08lx ==> %08lx != %08lx ** ERROR **\n",
					addr, addr + length - 1, crc, vcrc);
			return 1;
		}
	}

	return 0;

}
#endif	/* CONFIG_CRC32_VERIFY */


/**
 * Calculate the MD5 hash of a block of data  (from sdk/examples/crypto)
 *
 * @param md5        Filled with the 16 byte MD5 hash
 * @param buffer     Input data
 * @param buffer_len Inout data length
 */
static void hash_md5(uint8_t *md5, const uint8_t *buffer, int buffer_len)
{
    const uint64_t bits = cvmx_cpu_to_le64(buffer_len * 8); /* MD5 expects little endian */
    const uint64_t *ptr = (const uint64_t *)buffer;
    uint8_t chunk[64];
    uint64_t dummy;

    /* Set the IV to the MD5 magic start value */
    CVMX_MT_HSH_IV(0x0123456789abcdefull, 0);
    CVMX_MT_HSH_IV(0xfedcba9876543210ull, 1);

    /* MD5 input is in the following form:
        1) User data
        2) Byte 0x80
        3) Optional zero padding
        4) Original Data length in bits as an 8 byte unsigned integer
        Zero padding is added to make the 1-4 an even multiple of 64 bytes */

    /* Iterate through 64 bytes at a time */
    while (buffer_len >= 64)
    {
        CVMX_MT_HSH_DAT(*ptr++, 0);
        CVMX_MT_HSH_DAT(*ptr++, 1);
        CVMX_MT_HSH_DAT(*ptr++, 2);
        CVMX_MT_HSH_DAT(*ptr++, 3);
        CVMX_MT_HSH_DAT(*ptr++, 4);
        CVMX_MT_HSH_DAT(*ptr++, 5);
        CVMX_MT_HSH_DAT(*ptr++, 6);
        if (OCTEON_IS_MODEL(OCTEON_CN63XX_PASS1_X)) // 63XX pass1 errata
            CVMX_MF_HSH_IV(dummy, 0);   
        CVMX_MT_HSH_STARTMD5(*ptr++);
        buffer_len-=64;
    }

    /* The rest of the data will need to be copied into a chunk */
    if (buffer_len > 0)
        memcpy(chunk, ptr, buffer_len);
    chunk[buffer_len] = 0x80;
    memset(chunk + buffer_len + 1, 0, 64 - buffer_len - 1);

    ptr = (const uint64_t *)chunk;
    CVMX_MT_HSH_DAT(*ptr++, 0);
    CVMX_MT_HSH_DAT(*ptr++, 1);
    CVMX_MT_HSH_DAT(*ptr++, 2);
    CVMX_MT_HSH_DAT(*ptr++, 3);
    CVMX_MT_HSH_DAT(*ptr++, 4);
    CVMX_MT_HSH_DAT(*ptr++, 5);
    CVMX_MT_HSH_DAT(*ptr++, 6);

    if (OCTEON_IS_MODEL(OCTEON_CN63XX_PASS1_X)) // 63XX pass1 errata
        CVMX_MF_HSH_IV(dummy, 0);   
    /* Check to see if there is room for the bit count */
    if (buffer_len < 56)
        CVMX_MT_HSH_STARTMD5(bits);
    else
    {
        CVMX_MT_HSH_STARTMD5(*ptr);
        /* Another block was needed */
        CVMX_MT_HSH_DATZ(0);
        CVMX_MT_HSH_DATZ(1);
        CVMX_MT_HSH_DATZ(2);
        CVMX_MT_HSH_DATZ(3);
        CVMX_MT_HSH_DATZ(4);
        CVMX_MT_HSH_DATZ(5);
        CVMX_MT_HSH_DATZ(6);
        if (OCTEON_IS_MODEL(OCTEON_CN63XX_PASS1_X)) // 63XX pass1 errata
            CVMX_MF_HSH_IV(dummy, 0);   
        CVMX_MT_HSH_STARTMD5(bits);
    }

    /* Get the final MD5 */
    CVMX_MF_HSH_IV(((uint64_t*)md5)[0], 0);
    CVMX_MF_HSH_IV(((uint64_t*)md5)[1], 1);
}

int do_mem_md5 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong addr, length;
	uint64_t *ptr;
	int verify;
	int ac;
	char **av;
        uint8_t md5[16];
        uint64_t *md5_dw = (uint64_t *)md5;

        memset(md5, 0, 16);

	if (argc < 3) {
  usage:
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	av = argv + 1;
	ac = argc - 1;
	if (strcmp(*av, "-v") == 0) {
		verify = 1;
		av++;
		ac--;
		if (ac < 3)
			goto usage;
	} else
		verify = 0;

	addr = simple_strtoul(*av++, NULL, 16);
	addr += base_address;
	length = simple_strtoul(*av++, NULL, 16);

	hash_md5(md5, (const uchar *) addr, length);

	if (!verify) {
		printf ("MD5 for %08lx ... %08lx ==> %016llx%016llx\n",
				addr, addr + length - 1, md5_dw[0], md5_dw[1]);
		if (ac > 2) {
			ptr = (uint64_t *) simple_strtoul (*av++, NULL, 16);
			*ptr++ = md5_dw[0];
			*ptr = md5_dw[1];
		}
	} else {
                printf("verify not supported\n");
                return 1;
	}

	return 0;

}


#include "octeon_boot.h"
int do_bootmem_named_alloc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    char *name;
    int64_t addr = 0;
    uint64_t max_addr = 0;
    uint64_t size;
    char buf[20];

    setenv("named_block_addr", NULL);
    setenv("named_block_size", NULL);

    if (argc < 3)
    {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
    }
    name = argv[1];
    size = simple_strtoull(argv[2], NULL, 16);
    if (argc == 4)
    {
        addr = simple_strtoull(argv[3], NULL, 16);
        max_addr = addr + size;
    }

#ifdef DEBUG
    printf("Name: %s, size: 0x%llx, addr: 0x%llx\n", name, size, addr);
#endif

    addr = cvmx_bootmem_phy_named_block_alloc(size, addr, max_addr, 0, name, 0);
    if (addr < 0)
    {
        printf("Named allocation failed!\n");
        return(1);
    }
    printf("Allocated 0x%llx bytes at address: 0x%llx, name: %s\n", size, addr, name);

    sprintf(buf, "0x%llx", addr);
    setenv("named_block_addr", buf);
    sprintf(buf, "0x%llx", size);
    setenv("named_block_size", buf);


#ifdef DEBUG
    printf("\n\n===========================================\n");
    printf("Dump of named blocks:\n");
    cvmx_bootmem_phy_named_block_print();
    cvmx_bootmem_phy_list_print();
#endif

    return(0);
}

int do_bootmem_named_free (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    if (argc < 2)
    {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
    }
    if (!cvmx_bootmem_phy_named_block_free(argv[1], 0))
    {
        printf("Error freeing block: %s\n", argv[1]);
        return(1);
    }
    return(0);
}

int do_bootmem_named_print (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    cvmx_bootmem_phy_named_block_print();
    return(0);
}
int do_bootmem_free_print (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    cvmx_bootmem_phy_list_print();
    return(0);
}

/**************************************************/
#if (CONFIG_COMMANDS & CFG_CMD_MEMORY)
U_BOOT_CMD(
	md,     3,     1,      do_mem_md,
	"md      - memory display\n",
	"[.b, .w, .l] address [# of objects]\n    - memory display\n"
);


U_BOOT_CMD(
	mm,     2,      1,       do_mem_mm,
	"mm      - memory modify (auto-incrementing)\n",
	"[.b, .w, .l] address\n" "    - memory modify, auto increment address\n"
);


U_BOOT_CMD(
	nm,     2,	    1,     	do_mem_nm,
	"nm      - memory modify (constant address)\n",
	"[.b, .w, .l] address\n    - memory modify, read and keep address\n"
);

U_BOOT_CMD(
	mw,    4,    1,     do_mem_mw,
	"mw      - memory write (fill)\n",
	"[.b, .w, .l] address value [count]\n    - write memory\n"
);

U_BOOT_CMD(
	cp,    4,    1,    do_mem_cp,
	"cp      - memory copy\n",
	"[.b, .w, .l] source target count\n    - copy memory\n"
);

U_BOOT_CMD(
	cmp,    4,     1,     do_mem_cmp,
	"cmp     - memory compare\n",
	"[.b, .w, .l] addr1 addr2 count\n    - compare memory\n"
);

#ifndef CONFIG_CRC32_VERIFY

U_BOOT_CMD(
	crc32,    4,    1,     do_mem_crc,
	"crc32   - checksum calculation\n",
	"address count [addr]\n    - compute CRC32 checksum [save at addr]\n"
);

#else	/* CONFIG_CRC32_VERIFY */

U_BOOT_CMD(
	crc32,    5,    1,     do_mem_crc,
	"crc32   - checksum calculation\n",
	"address count [addr]\n    - compute CRC32 checksum [save at addr]\n"
	"-v address count crc\n    - verify crc of memory area\n"
);

#endif	/* CONFIG_CRC32_VERIFY */

U_BOOT_CMD(
	md5,    5,    1,     do_mem_md5,
	"md5   - MD5 hash calculation\n",
	"address count [addr]\n    - compute MD5 [save at addr]\n"
#if 0
	"-v address count MD5\n    - verify MD5 of memory area\n"
#endif
);

U_BOOT_CMD(
	base,    2,    1,     do_mem_base,
	"base    - print or set address offset\n",
	"\n    - print address offset for memory commands\n"
	"base off\n    - set address offset for memory commands to 'off'\n"
);
U_BOOT_CMD(
	read64,    2,    1,     do_read64,
	"read64    - read 64 bit word from 64 bit address\n",
	"\n    - read 64 bit word from 64 bit address\n"
	"read64 addr\n    - read 64 bit word from 64 bit address\n"
);
U_BOOT_CMD(
	write64,    4,    1,     do_write64,
	"write64    - write 64 bit word to 64 bit address\n",
	"\n    - write 64 bit word to 64 bit address\n"
	"write64 addr val\n    - write 64 bit word to 64 bit address\n"
);
U_BOOT_CMD(
	namedalloc,    4,    0,     do_bootmem_named_alloc,
	"namedalloc    - Allocate a named bootmem block\n",
	"\n    - Allocate a named bootmem block\n"
	"namedalloc name size [address]\n"
    "    - Allocate a named bootmem block with a given name and size at an\n"
    "      optional fixed address.  Sets environment variables named_block_addr,\n"
    "      named_block_size to address and size of block allocated.\n"
);

U_BOOT_CMD(
	namedfree,    2,    0,     do_bootmem_named_free,
	"namedfree    - Free a named bootmem block\n",
	"\n    - Free a named bootmem block\n"
	"namedfree name\n"
    "    - Free a named bootmem block.\n"
);
U_BOOT_CMD(
	namedprint,    1,    0,     do_bootmem_named_print,
	"namedprint    - Print list of named bootmem blocks\n",
	"\n    - Print list of named bootmem blocks\n"
	"namedprint\n"
    "    - Print list of named bootmem blocks.\n"
);
U_BOOT_CMD(
	freeprint,    1,    0,     do_bootmem_free_print,
	"freeprint    - Print list of free bootmem blocks\n",
	"\n    - Print list of free bootmem blocks\n"
	"freeprint\n"
    "    - Print list of free bootmem blocks.\n"
);


U_BOOT_CMD(
	read64b,    2,    1,     do_read64b,
	"read64b    - read 8 bit word from 64 bit address\n",
	"\n    - read 8 bit word from 64 bit address \n"
	"read64b addr\n    - read 8 bit word from 64 bit address\n"
);
U_BOOT_CMD(
	read64s,    2,    1,     do_read64s,
	"read64s    - read 16 bit word from 64 bit address\n",
	"\n    - read 16 bit word from 64 bit address \n"
	"read64s addr\n    - read 16 bit word from 64 bit address\n"
);
U_BOOT_CMD(
	read64l,    2,    1,     do_read64l,
	"read64l    - read 32 bit word from 64 bit address\n",
	"\n    - read 32 bit word from 64 bit address \n"
	"read64l addr\n    - read 32 bit word from 64 bit address\n"
);
U_BOOT_CMD(
	write64b,    4,    1,     do_write64b,
	"write64b    - write 8 bit word to 64 bit address\n",
	"\n    - write 8 bit word to 64 bit address\n"
	"write64b addr val\n    - write 8 bit word to 64 bit address\n"
);
U_BOOT_CMD(
	write64s,    4,    1,     do_write64s,
	"write64s    - write 16 bit word to 64 bit address\n",
	"\n    - write 16 bit word to 64 bit address\n"
	"write64s addr val\n    - write 16 bit word to 64 bit address\n"
);
U_BOOT_CMD(
	write64l,    4,    1,     do_write64l,
	"write64l    - write 32 bit word to 64 bit address\n",
	"\n    - write 32 bit word to 64 bit address\n"
	"write64l addr val\n    - write 32 bit word to 64 bit address\n"
);

U_BOOT_CMD(
	read_cmp,    4,    1,     do_read_cmp,
	"read_cmp    - read and compare memory to val\n",
	"\n    - read and compare memory to val\n"
	"read_cmp addr val\n    - read and compare memory to val\n"
);

U_BOOT_CMD(
	loop,    3,    1,    do_mem_loop,
	"loop    - infinite loop on address range\n",
	"[.b, .w, .l] address number_of_objects\n"
	"    - loop on a set of addresses\n"
);

U_BOOT_CMD(
	mtest,    4,    1,     do_mem_mtest,
	"mtest   - simple RAM test\n",
	"[start [end [pattern]]]\n"
	"    - simple RAM read/write test\n"
);

#endif
#endif	/* CFG_CMD_MEMORY */
