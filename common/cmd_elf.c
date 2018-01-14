/*
 * Copyright (c) 2001 William L. Pitts
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are freely
 * permitted provided that the above copyright notice and this
 * paragraph and the following disclaimer are duplicated in all
 * such forms.
 *
 * This software is provided "AS IS" and without any express or
 * implied warranties, including, without limitation, the implied
 * warranties of merchantability and fitness for a particular
 * purpose.
 */

#include <common.h>
#include <command.h>
#include <linux/ctype.h>
#include <net.h>
#include <elf.h>
#include "octeon_boot.h"


#if (CONFIG_COMMANDS & CFG_CMD_ELF)

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

int valid_elf_image (unsigned long addr);

unsigned long load_elf32_image (unsigned long addr);
uint64_t load_elf64_image (unsigned long addr);

/* ======================================================================
 * Interpreter command to boot an arbitrary ELF image from memory.
 * ====================================================================== */
int do_bootelf (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned long addr;		/* Address of the ELF image     */
	unsigned long rc;		/* Return value from user code  */

	/* -------------------------------------------------- */
	int rcode = 0;

	if (argc < 2)
		addr = load_addr;
	else
        {
            addr = simple_strtoul (argv[1], NULL, 16);
            if (!addr)
		addr = load_addr;
        }

	if (!valid_elf_image (addr))
		return 1;

    if (((Elf32_Ehdr *)addr)->e_ident[EI_CLASS] == ELFCLASS32)
    {
        addr = load_elf32_image (addr);
    } 
    else  /* valid_elf_image ensures only 32 and 64 bit class values */
    {
        addr = (uint32_t)load_elf64_image (addr);
    }


	printf ("## Starting application at 0x%08lx ...\n", addr);


#ifdef CONFIG_OCTEON
        octeon_restore_std_mips_config();
#endif
	/*
	 * pass address parameter as argv[0] (aka command name),
	 * and all remaining args
	 */
	rc = ((ulong (*)(int, char *[])) addr) (--argc, &argv[1]);
	if (rc != 0)
		rcode = 1;

	printf ("## Application terminated, rc = 0x%lx\n", rc);
	return rcode;
}
/* ======================================================================
 * Interpreter command to boot an arbitrary ELF image from memory.
 * ====================================================================== */

U_BOOT_CMD(
	bootelf,      2,      0,      do_bootelf,
	"bootelf - Boot from an ELF image in memory\n",
	" [address] - load address of ELF image.\n"
	" If address is 0, default load address is used.\n"
);

#ifdef CONFIG_CMD_BOOTVX
/* ======================================================================
 * Interpreter command to boot VxWorks from a memory image.  The image can
 * be either an ELF image or a raw binary.  Will attempt to setup the
 * bootline and other parameters correctly.
 * ====================================================================== */
int do_bootvx ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
#if defined(CONFIG_WALNUT405)	|| \
    defined(CONFIG_CPCI405)	|| \
    defined(CONFIG_OCRTC)	|| \
    defined(CONFIG_ORSG)
	DECLARE_GLOBAL_DATA_PTR;
#endif

	unsigned long addr;		/* Address of image            */
	unsigned long bootaddr;		/* Address to put the bootline */
	char *bootline;			/* Text of the bootline        */
	char *tmp;			/* Temporary char pointer      */

#if defined(CONFIG_4xx) || defined(CONFIG_IOP480)
	char build_buf[80];		/* Buffer for building the bootline */
#endif
	/* -------------------------------------------------- */

	/*
	 * Check the loadaddr variable.
	 * If we don't know where the image is then we're done.
	 */

	if ((tmp = getenv ("loadaddr")) != NULL) {
		addr = simple_strtoul (tmp, NULL, 16);
	} else {
		puts ("No load address provided\n");
		return 1;
	}

#if (CONFIG_COMMANDS & CFG_CMD_NET)
	/* Check to see if we need to tftp the image ourselves before starting */

	if ((argc == 2) && (strcmp (argv[1], "tftp") == 0)) {
		if (NetLoop (TFTP) <= 0)
			return 1;
		printf ("Automatic boot of VxWorks image at address 0x%08lx ... \n", addr);
	}
#endif

	/* This should equate
	 * to NV_RAM_ADRS + NV_BOOT_OFFSET + NV_ENET_OFFSET
	 * from the VxWorks BSP header files.
	 * This will vary from board to board
	 */

#if defined(CONFIG_WALNUT405)
	tmp = (char *) CFG_NVRAM_BASE_ADDR + 0x500;
	memcpy ((char *) tmp, (char *) &gd->bd->bi_enetaddr[3], 3);
#elif defined(CONFIG_CPCI405)
	tmp = (char *) CFG_NVRAM_BASE_ADDR + CFG_NVRAM_VXWORKS_OFFS;
	memcpy ((char *) tmp, (char *) &gd->bd->bi_enetaddr[0], 6);
#elif defined(CONFIG_OCRTC) || defined(CONFIG_ORSG)
	tmp = (char *) CFG_ETHERNET_MAC_ADDR;
	memcpy ((char *) tmp, (char *) &gd->bd->bi_enetaddr[0], 6);
#else
	puts ("## Ethernet MAC address not copied to NV RAM\n");
#endif

	/*
	 * Use bootaddr to find the location in memory that VxWorks
	 * will look for the bootline string. The default value for
	 * PowerPC is LOCAL_MEM_LOCAL_ADRS + BOOT_LINE_OFFSET which
	 * defaults to 0x4200
	 */

	if ((tmp = getenv ("bootaddr")) == NULL)
		bootaddr = 0x4200;
	else
		bootaddr = simple_strtoul (tmp, NULL, 16);

	/*
	 * Check to see if the bootline is defined in the 'bootargs'
	 * parameter. If it is not defined, we may be able to
	 * construct the info
	 */

	if ((bootline = getenv ("bootargs")) != NULL) {
		memcpy ((void *) bootaddr, bootline, MAX(strlen(bootline), 255));
		flush_cache (bootaddr, MAX(strlen(bootline), 255));
	} else {
#if defined(CONFIG_4xx)
		sprintf (build_buf, "ibmEmac(0,0)");

		if ((tmp = getenv ("hostname")) != NULL) {
			sprintf (&build_buf[strlen (build_buf - 1)],
				"host:%s ", tmp);
		} else {
			sprintf (&build_buf[strlen (build_buf - 1)],
				": ");
		}

		if ((tmp = getenv ("ipaddr")) != NULL) {
			sprintf (&build_buf[strlen (build_buf - 1)],
				"e=%s ", tmp);
		}
		memcpy ((void *)bootaddr, build_buf, MAX(strlen(build_buf), 255));
		flush_cache (bootaddr, MAX(strlen(build_buf), 255));
#elif defined(CONFIG_IOP480)
		sprintf (build_buf, "dc(0,0)");

		if ((tmp = getenv ("hostname")) != NULL) {
			sprintf (&build_buf[strlen (build_buf - 1)],
				"host:%s ", tmp);
		} else {
			sprintf (&build_buf[strlen (build_buf - 1)],
				": ");
		}

		if ((tmp = getenv ("ipaddr")) != NULL) {
			sprintf (&build_buf[strlen (build_buf - 1)],
				"e=%s ", tmp);
		}
		memcpy ((void *) bootaddr, build_buf, MAX(strlen(build_buf), 255));
		flush_cache (bootaddr, MAX(strlen(build_buf), 255));
#else

		/*
		 * I'm not sure what the device should be for other
		 * PPC flavors, the hostname and ipaddr should be ok
		 * to just copy
		 */

		puts ("No bootargs defined\n");
		return 1;
#endif
	}

	/*
	 * If the data at the load address is an elf image, then
	 * treat it like an elf image. Otherwise, assume that it is a
	 * binary image
	 */

	if (valid_elf_image (addr)) {
		addr = load_elf32_image (addr);
	} else {
		puts ("## Not an ELF image, assuming binary\n");
		/* leave addr as load_addr */
	}

	printf ("## Using bootline (@ 0x%lx): %s\n", bootaddr,
			(char *) bootaddr);
	printf ("## Starting vxWorks at 0x%08lx ...\n", addr);

	((void (*)(void)) addr) ();

	puts ("## vxWorks terminated\n");
	return 1;
}
U_BOOT_CMD(
	bootvx,      2,      0,      do_bootvx,
	"bootvx  - Boot vxWorks from an ELF image\n",
	" [address] - load address of vxWorks ELF image.\n"
);
#endif

/* ======================================================================
 * Determine if a valid ELF image exists at the given memory location.
 * First looks at the ELF header magic field, the makes sure that it is
 * executable and makes sure that it is for a PowerPC.
 * ====================================================================== */
int valid_elf_image (unsigned long addr)
{
	Elf32_Ehdr *ehdr;		/* Elf header structure pointer */

	/* -------------------------------------------------- */

	ehdr = (Elf32_Ehdr *) cvmx_phys_to_ptr(addr);

	if (!IS_ELF (*ehdr)) {
		printf ("## No elf image at address 0x%08lx\n", addr);
		return 0;
	}
    if (ehdr->e_ident[EI_CLASS] == ELFCLASS32)
    {
        printf("ELF file is 32 bit\n");
    }
    else if (ehdr->e_ident[EI_CLASS] == ELFCLASS64)
    {
        printf("ELF file is 64 bit\n");
    }
    else
    {
        printf("ERROR: invalid elf class\n");
        return(0);
    }

	if (ehdr->e_type != ET_EXEC) {
		printf ("## Not an Executable elf image at address 0x%08lx\n",
			addr);
		return 0;
	}

#if 0
	if (ehdr->e_machine != EM_PPC) {
		printf ("## Not a PowerPC elf image at address 0x%08lx\n",
			addr);
		return 0;
	}
#endif

	return 1;
}




/* ======================================================================
 * A very simple elf loader, assumes the image is valid, returns the
 * entry point address.
 * ====================================================================== */
unsigned long load_elf32_image (unsigned long addr)
{
	Elf32_Ehdr *ehdr;		/* Elf header structure pointer     */
	Elf32_Shdr *shdr;		/* Section header structure pointer */
	unsigned char *strtab = 0;	/* String table pointer             */
	unsigned char *image;		/* Binary image pointer             */
	int i;				/* Loop counter                     */

	/* -------------------------------------------------- */

	ehdr = (Elf32_Ehdr *) cvmx_phys_to_ptr(addr);

	/* Find the section header string table for output info */
	shdr = (Elf32_Shdr *) cvmx_phys_to_ptr(addr + ehdr->e_shoff +
			       (ehdr->e_shstrndx * sizeof (Elf32_Shdr)));

	if (shdr->sh_type == SHT_STRTAB)
		strtab = cvmx_phys_to_ptr(addr + shdr->sh_offset);

	/* Load each appropriate section */
	for (i = 0; i < ehdr->e_shnum; ++i) {
		shdr = (Elf32_Shdr *) cvmx_phys_to_ptr(addr + ehdr->e_shoff +
				       (i * sizeof (Elf32_Shdr)));

		if (!(shdr->sh_flags & SHF_ALLOC)
		   || shdr->sh_addr == 0 || shdr->sh_size == 0) {
			continue;
		}

		if (strtab) {
			printf ("%sing %s @ 0x%08lx (%ld bytes)\n",
				(shdr->sh_type == SHT_NOBITS) ?
					"Clear" : "Load",
				&strtab[shdr->sh_name],
				(unsigned long) shdr->sh_addr,
				(long) shdr->sh_size);
		}

		if (shdr->sh_type == SHT_NOBITS) {
			memset (shdr->sh_addr, 0, shdr->sh_size);
		} else {
			image = cvmx_phys_to_ptr(addr + shdr->sh_offset);
			memcpy (shdr->sh_addr,
				image,
				shdr->sh_size);
		}
		flush_cache (shdr->sh_addr, shdr->sh_size);
	}

	return ehdr->e_entry;
}

/* ======================================================================
 * A very simple elf loader, assumes the image is valid, returns the
 * entry point address.
 * ====================================================================== */
uint64_t load_elf64_image (unsigned long addr)
{

	Elf64_Ehdr *ehdr;		/* Elf header structure pointer     */
	Elf64_Shdr *shdr;		/* Section header structure pointer */
	unsigned char *strtab = 0;	/* String table pointer             */
	unsigned char *image;		/* Binary image pointer             */
	int i;				/* Loop counter                     */

	/* -------------------------------------------------- */

	ehdr = (Elf64_Ehdr *) addr;
    /*
     * Take care to strip of the cache coherency bits from the XKPHYS addresses
     * using octeon_fixup_xkphys().
     * Use 64 bit memset/memcpy that take 64 bit addresses (xkphys) to clear/copy memory 
     */

	if (ehdr->e_type == ET_EXEC && ehdr->e_phoff && ehdr->e_phnum) {
		/* Load program headers.  */
		for (i = 0; i < ehdr->e_phnum; i++) {
			Elf64_Phdr *phdr = (Elf64_Phdr *)(uint32_t)(addr + ehdr->e_phoff +
							  (i * ehdr->e_phentsize));
			if (phdr->p_type == PT_LOAD) {
				/* Load it. */
				printf ("Processing PHDR %d\n", i);
				image = (unsigned char *)(uint32_t)(phdr->p_offset + addr);
				if (phdr->p_filesz) {
					printf ("  Loading %lx bytes at %llx\n",
						phdr->p_filesz,
						octeon_fixup_xkphys(phdr->p_paddr));
					memcpy64 (octeon_fixup_xkphys(phdr->p_paddr),
						  (int32_t)image,
						  phdr->p_filesz);
				}
				if (phdr->p_memsz  > phdr->p_filesz) {
					printf ("  Clearing %lx bytes at %llx\n",
						phdr->p_memsz - phdr->p_filesz,
						octeon_fixup_xkphys(phdr->p_paddr) + phdr->p_filesz);

					memset64 (octeon_fixup_xkphys(phdr->p_paddr) + phdr->p_filesz,
						  0,
						  phdr->p_memsz - phdr->p_filesz);
				}
				flush_cache (phdr->p_paddr, phdr->p_memsz);
			}
		}
		return octeon_fixup_xkphys(ehdr->e_entry);
	}
	/* Otherwise do it the old way with section headers.  */

	/* Find the section header string table for output info */
	shdr = (Elf64_Shdr *)(uint32_t)(addr + ehdr->e_shoff +
			       (ehdr->e_shstrndx * sizeof (Elf64_Shdr)));

	if (shdr->sh_type == SHT_STRTAB)
		strtab = (unsigned char *) (uint32_t)(addr + shdr->sh_offset);

	/* Load each appropriate section */
	for (i = 0; i < ehdr->e_shnum; ++i) {
		shdr = (Elf64_Shdr *)(uint32_t)(addr + ehdr->e_shoff +
				       (i * sizeof (Elf64_Shdr)));

		if (!(shdr->sh_flags & SHF_ALLOC)
		   || shdr->sh_addr == 0 || shdr->sh_size == 0) {
			continue;
		}

		if (strtab) {
			printf ("%sing %s @ 0x%08qx (0x%lx bytes)\n",
				(shdr->sh_type == SHT_NOBITS) ?
					"Clear" : "Load",
				&strtab[shdr->sh_name],
				shdr->sh_addr,
				(long) shdr->sh_size);
		}

		if (shdr->sh_type == SHT_NOBITS) {
			memset64 (octeon_fixup_xkphys(shdr->sh_addr), 0, shdr->sh_size);
		} else {
			image = (unsigned char *) addr + shdr->sh_offset;
			memcpy64 (octeon_fixup_xkphys(shdr->sh_addr),
                                  (int32_t)image,
				shdr->sh_size);
		}
		flush_cache (shdr->sh_addr, shdr->sh_size);
	}

	return octeon_fixup_xkphys(ehdr->e_entry);
}



/* ====================================================================== */

#endif	/* CFG_CMD_ELF */
