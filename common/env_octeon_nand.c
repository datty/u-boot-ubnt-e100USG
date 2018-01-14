/*
 */

#include <common.h>

#if defined(CFG_ENV_IS_IN_OCTEON_NAND) /* Environment is in NAND, using Octeon ECC */

#ifndef CONFIG_OCTEON_NAND_STAGE2
#error ERROR: Environment in NAND is only supported for NAND stage2 configurations
#endif


#include <command.h>
#include <environment.h>
#include <linux/stddef.h>
#include <malloc.h>
#include <octeon_boot.h>
#include <octeon_nand.h>

#include "cvmx.h"
#include "cvmx-nand.h"
#include "cvmx-bootloader.h"

env_t *env_ptr = NULL;

char * env_name_spec = "OCTEON_NAND";


extern char (*env_get_char)(int);
extern uchar env_get_char_memory (int index);

int eraseenv(int exclude_latest);


int oct_nand_env_copy(void * buffer, int max_len, uint64_t *version);


uint64_t oct_nand_env_search(uint64_t max_addr);
/* Temporary DMA buffer for when we are searching for the header. It is in the
    text section so that NAND stage 1 will lock it in L2.  This is required for NAND stage2
    builds, as these buffers are used before DRAM is set up while the code is locked in L2. */
static char dma_buffer[2600] __attribute__ ((section (".data_in_text"))) __attribute__ ((aligned(8)));
static char env_buffer[4096] __attribute__ ((section (".data_in_text"))) __attribute__ ((aligned(8)));
static uint64_t early_env_addr __attribute__ ((section (".data_in_text")));

/* Counter to save the 'version' of the environment that we have loaded.
** We search through the NAND, and only use the highest numbered environment.
** we use the 'address' field of the header to track this.
*/
static uint64_t env_version;
 
/* Read from the 'special' ??? environment, before running from RAM. */
uchar env_get_char_spec (int index)
{
    return(env_buffer[index]);
    return 0;
}

/* Copy environment from special early boot location to normal location */
void env_relocate_spec (void)
{
    bootloader_header_t header_copy;
    /* env_buffer has been relocated, so can't use it here.
    ** Also has probably been cleared even if we could locate it.
    ** Read the environment from NAND again. */
    if (early_env_addr)
    {
        oct_nand_copy_from_boot_nand(&header_copy,early_env_addr, sizeof(header_copy));
        env_version = header_copy.address;

        oct_nand_copy_from_boot_nand(env_ptr->data, early_env_addr + header_copy.hlen, MIN(header_copy.dlen, ENV_SIZE));
    }
}

int saveenv(void)
{
    /* Save environment to flash.  For NAND this is a multi-step process, as
    ** we need to zero other environment images, and then find a free spot to
    ** write the new one. */
    unsigned char *buf = (unsigned char *)dma_buffer;
    uint64_t blank_addr;
    int chip = oct_nand_get_cur_chip();
    int page_size = cvmx_nand_get_page_size(chip);
    int oob_size = octeon_nand_get_oob_size(chip);
    uint64_t block_size = page_size * cvmx_nand_get_pages_per_block(chip);
    int read_size = page_size + oob_size;
    int i;
    /* For writing to nand, we need the header and the data
    ** contiguous */
    uint64_t new_buf_storage[ENV_SIZE/8 + 256/8] = { 0 };
    unsigned char *new_buf = (unsigned char *)new_buf_storage;
    bootloader_header_t *header_ptr = (void *)new_buf;
    uint32_t crc;



    /* Erase any old environments that we can.  We leave the current version
    ** alone */
    eraseenv(1);


    /* First we need to find a blank spot for the new environment to go.
    ** We need to be able to fit the environment + the header.
    ** If we find a spot, write the new environment, else
    ** report error.*/

    /* Find blank spot */
    int conseq_blank_pages = 0;
    int blank;
    read_size = page_size + oob_size;
    for (blank_addr = 0x0; blank_addr < MAX_NAND_SEARCH_ADDR; blank_addr += page_size)
    {
        int bytes = cvmx_nand_page_read(chip, blank_addr, cvmx_ptr_to_phys(buf), read_size);
        if (bytes != read_size)
        {
            printf("ERROR: error reading from NAND at addr: 0x%llx (bytes: %d)\n", blank_addr, bytes);
            goto error_out;
        }




        blank=1;
        for (i = 0; i < read_size;i++)
        {
            if (buf[i] != 0xff)
            {
                blank=0;
                break;
            }
        }
        if (blank)
        {
            if ((conseq_blank_pages + 1) * page_size >= ENV_SIZE + sizeof(bootloader_header_t))
            {
                /* We're done, we have found a large enough blank spot.  Now do some
                ** other checks to see if we will use it. */

                /* If we cross a block boundary, keep looking.  We want to be able to later
                ** identify blocks that have nothing but env in them so that we can erase them.
                ** It an env image stradles a block, then the second block will have unidentifiable
                ** pages in the beginning of it, and won't be automatically erasable. */
                if (blank_addr/block_size != ((blank_addr + (conseq_blank_pages + 1)* page_size))/block_size)
                {
                    /* Set blank addr to start of new block, and adjust blank count */
//                    printf("Crossing block: addr: 0x%llx, conseq: %d\n", blank_addr, conseq_blank_pages);
                    uint64_t new_blank_addr = ((blank_addr + (conseq_blank_pages + 1) * page_size)) & ~(block_size -1);
                    conseq_blank_pages -= (new_blank_addr - blank_addr)/page_size;
                    blank_addr = new_blank_addr;
//                    printf("new addr: 0x%llx, conseq: %d\n", blank_addr, conseq_blank_pages);
                    continue;
                }
                blank_addr -= conseq_blank_pages * page_size;
                break;
            }
            else
                conseq_blank_pages++;    /* Keep looking */

        }
        else
        {
            /* If it is not blank, check to see if we have found a valid environment image.
            ** If so, skip it and keep looking.  If we find anything else, skip to the next block,
            ** as we don't want to mix environments with anything else.
            */

            bootloader_header_t *header = (void *)buf;

            if (header->magic == BOOTLOADER_HEADER_MAGIC 
                && header->image_type == BL_HEADER_IMAGE_UBOOT_ENV
                && oct_nand_validate_image(blank_addr) >= 0)
            {
                /* We found a valid envronment image, so skip it. */
                blank_addr += ((header->hlen + header->dlen + page_size - 1) & ~(page_size - 1)) - page_size;
                conseq_blank_pages = 0;
                continue;
            }

            /* We had something else, skip the block */
            blank_addr += block_size - (blank_addr & (block_size - 1)) - page_size;
            conseq_blank_pages = 0;
        }
    }

    if (blank_addr  && blank_addr < MAX_NAND_SEARCH_ADDR)
    {
//        printf("Found blank spot at addr: 0x%llx\n", blank_addr);
    }
    else
    {
        printf("No blank pages found\n");
        goto error_out;
    }

    /* Create header for environment data */
    printf("Putting new environment at addr 0x%llx\n", blank_addr);


    /* Copy data and ompute data crc */
    memcpy(new_buf + sizeof(bootloader_header_t), env_ptr->data, ENV_SIZE);
    crc = crc32(0,new_buf + sizeof(bootloader_header_t),ENV_SIZE);


    /* Header is big endian */
    header_ptr->image_type = BL_HEADER_IMAGE_UBOOT_ENV;
    header_ptr->magic = (BOOTLOADER_HEADER_MAGIC);
    header_ptr->maj_rev = (BOOTLOADER_HEADER_CURRENT_MAJOR_REV);
    header_ptr->min_rev = (BOOTLOADER_HEADER_CURRENT_MINOR_REV);
    header_ptr->dlen = (ENV_SIZE);
    header_ptr->dcrc = (crc);
    header_ptr->board_type = (0);  
    header_ptr->address = ++env_version;  /* We may write several environments before rebooting */
    header_ptr->image_type = (BL_HEADER_IMAGE_UBOOT_ENV);
    header_ptr->hlen = (sizeof(bootloader_header_t));
    memset(header_ptr->comment_string, 0xff, BOOTLOADER_HEADER_COMMENT_LEN);
    memset(header_ptr->version_string, 0xff, BOOTLOADER_HEADER_VERSION_LEN);


    /* Now compute header CRC over all of the header excluding the CRC*/
    header_ptr->hcrc = crc32(0, (void *)header_ptr, offsetof(bootloader_header_t,hcrc));
    header_ptr->hcrc = crc32(header_ptr->hcrc, (void *)&(header_ptr->hlen), sizeof(bootloader_header_t) - offsetof(bootloader_header_t,hlen));



    /* Now write new block to NAND */
    if(0 > oct_nand_boot_write(blank_addr, new_buf, ENV_SIZE + sizeof(bootloader_header_t), 1))
    {
        printf("ERROR writing new environment to NAND\n");
        goto error_out;
    }

    return 0;

error_out:
    printf("ERROR: unable to update environment\n");
    return -1;
}



int memcheck(unsigned char *buf, unsigned char val, int len)
{
    int i;
    for (i = 0;i < len;i++)
    {
        if (buf[i] != val)
            return(buf[i] > val);
    }
    return(0);
}

int eraseenv(int exclude_latest)
{
    unsigned char *buf = (unsigned char *)dma_buffer;
    uint64_t nand_addr;
    int chip = oct_nand_get_cur_chip();
    int page_size = cvmx_nand_get_page_size(chip);
    uint64_t block_size = page_size * cvmx_nand_get_pages_per_block(chip);
    int oob_size = octeon_nand_get_oob_size(chip);
    int read_size = page_size + oob_size;

    bootloader_header_t header_copy;
    /* Here we want to scan NAND to find blocks that are composed of nothing
    ** but env images and erased pages.  The blocks like this that we find
    ** we then erase.
    ** As soon as we find something 'other', we skip to the next block.  When we reach
    ** the end of a block, we erase it.
    */


    uint64_t max_addr = 0x800000;

    int env_found = 0;
    for (nand_addr = 0; nand_addr < max_addr; nand_addr += page_size)
    {

        /* Clear env_found flag at start of every block */
        if (!(nand_addr & (block_size - 1)))
            env_found = 0;

        /* Skip all pad blocks. */
        if (oct_nand_block_is_bad(chip, nand_addr, 1))
            continue;

        int bytes = cvmx_nand_page_read(chip, nand_addr, cvmx_ptr_to_phys(buf), read_size);
        if (bytes != read_size)
        {
            printf("Error reading NAND addr %d (bytes_read: %d, expected: %d)\n", nand_addr, bytes, read_size);
            return -1;
        }
        int errors = cvmx_nand_correct_boot_ecc(buf);
        if (errors <= 1)
        {
            const bootloader_header_t *header = (const bootloader_header_t *)buf;

            /* Block is good, see if it contains a bootloader image header */
            if ((header->magic == BOOTLOADER_HEADER_MAGIC))
            {
                /* Check the CRC of the header */
                uint32_t crc;
                crc = crc32(0, (void *)header, 12);
                crc = crc32(crc,  (void *)header + 16, header->hlen - 16);
                
                if (crc == header->hcrc)
                {
                    header_copy = *header;
                    if (oct_nand_validate_image(nand_addr) >= 0)
                    {
                        if (header_copy.image_type == BL_HEADER_IMAGE_UBOOT_ENV
                            && !(exclude_latest && env_version == header->address))
                        {
                            env_found = 1;  /* Track that we have found an environment variable in this block */

                            /* Skip the image, as we have validated it */
                            nand_addr += ((header->dlen + page_size - 1) & ~(page_size - 1)) - page_size;

                            /* Check to see if we are at the end of a block */
                            if (!((nand_addr + page_size) & (block_size - 1)))
                            {
                                /* We are at the end of a block so erase it.  We have verified that
                                ** the block has nothing but valid environtment pages in it or already erased pages,
                                ** so it is OK to erase. */
                                printf("eraseenv: erasing NAND block at addr: 0x%llx\n", nand_addr & ~(block_size - 1));
                                if (CVMX_NAND_SUCCESS != cvmx_nand_block_erase(chip, nand_addr & ~(block_size - 1)))
                                {
                                    printf("ERROR erasing block!\n");
                                }
                            }
                        }
                        else
                        {
                            /* Skip the block if it is not an environment, or if we are saving
                            ** the latest and the latest version is in this block.
                            ** If we are at the start of a block, we need to skip the whole block.
                            */
                            nand_addr = ((nand_addr + block_size) & ~(block_size - 1)) - page_size;
                        }
                        continue;  
                        /* Cases below are handled by blank check code */
                    }
                }
            }
        }

        /* Check to see if page is erased.  If it is, check next page.  If not, skip to next block */
        if (memcheck(buf, 0xff, read_size))
        {
            /* Block is not erased, skip block. */
            nand_addr += ((nand_addr + block_size - page_size) & ~(block_size - 1)) - page_size;
        }
        else
        {
            /* Check to see if we are at the end of a block, and should erase */
            /* Check to see if we are at the end of a block */
            if (!((nand_addr + page_size) & (block_size - 1)) && env_found)
            {
                /* We are at the end of a block so erase it */
                printf("Erasing block at addr 0x%llx\n", nand_addr & ~(block_size - 1));
                if (CVMX_NAND_SUCCESS != cvmx_nand_block_erase(chip, nand_addr & ~(block_size - 1)))
                {
                    printf("ERROR erasing block!\n");
                }
            }
        }
    }


    /* Search for an environment, to see if we were able to get them all */
    if (!exclude_latest && oct_nand_env_search(max_addr))
    {
        printf("\nWarning: Unable to erase all environment versions.  This is most likely\n"
               "due to environment images being in the same erase block as other non-environment\n"
               "and non-blank pages\n\n");

    }

    return 0;

}

/* This won't work unless env_init is moved to after the serial port is active.
** Normally this is done before that (so that baud rate, etc can be in the environment.
*/
#if 1
#define special_debug_printf(...)
#else
#define special_debug_printf    printf
#endif

/* Look for an environment in NAND, and return the address of
** the first valid one found.
** 0 is invalid (stage 1 is there), so return 0 if not found.*/
uint64_t oct_nand_env_search(uint64_t max_addr)
{
    unsigned char *buf = (unsigned char *)dma_buffer;

    int page_size;
    int chip = oct_nand_get_cur_chip();
    uint64_t nand_addr = 0;
    const bootloader_header_t *header = (const bootloader_header_t *)buf;
    page_size = cvmx_nand_get_page_size(chip);
    int read_size = CVMX_NAND_BOOT_ECC_BLOCK_SIZE + 8;
    
    uint64_t cur_env_version = 0;  /* Track the highest version we have seen */
    uint64_t cur_env_nand_addr = 0; /* Address of current highest version */

    if (chip < 0)
    {
        special_debug_printf("No nand chips found\n");
        return 0;
    }



    special_debug_printf("Using NAND chip %d, page size: %d\n", chip, page_size);
    if (!page_size)
    {
        special_debug_printf("ERROR: No NAND configured (run probe)\n");
        return 0;
    }
    special_debug_printf("Scanning NAND for environment\n");
    while (nand_addr < max_addr)
    {
        
        nand_addr = oct_nand_image_search(nand_addr, max_addr, BL_HEADER_IMAGE_UBOOT_ENV);
        if (!nand_addr)
            break;  /* Nothing more found, so exit loop */

        int bytes = cvmx_nand_page_read(chip, nand_addr, cvmx_ptr_to_phys(buf), read_size);
        if (bytes != read_size)
        {
            special_debug_printf("Error reading NAND addr %d (bytes_read: %d, expected: %d)\n", nand_addr, bytes, read_size);
            return -1;
        }

        /* The image search function only gives us validated images, so we know we have a valid environment block */
        special_debug_printf("\nFound environement at addr: 0x%llx, size: 0x%x: , counter: 0x%llx", nand_addr, header->dlen, header->address);

        /* Update our tracking of the most recent environment.
        ** If we have identical counters, use the one found at a higher address. */
        if (header->address >= cur_env_version)
        {
            cur_env_version = header->address;
            cur_env_nand_addr = nand_addr;
        }
        /* Skip the image we just checked */
        nand_addr += ((header->hlen + header->dlen + page_size - 1) & ~(page_size - 1));

    }


    special_debug_printf("Done looking for environment, most up to date env addr: 0x%llx, version 0x%llx\n",
                         cur_env_nand_addr, cur_env_version);
    return(cur_env_nand_addr);

}





/************************************************************************
 * Initialize Environment use
 *
 * We are still running from ROM, so data use is limited
 * Use a (moderately small) buffer on the stack
 */
int env_init(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    bootloader_header_t header_copy;

    /* Now we want to scan for a environment block */
    if (oct_nand_probe() < 0)
    {
        special_debug_printf("ERROR initializing NAND\n");
        return 0; /* don't hang the board */
    }
    special_debug_printf("NAND initialized\n");



    uint64_t nand_addr = oct_nand_env_search(MAX_NAND_SEARCH_ADDR);

    if (nand_addr)
    {
        oct_nand_copy_from_boot_nand(&header_copy,nand_addr, sizeof(header_copy));
 
        oct_nand_copy_from_boot_nand(env_buffer ,nand_addr + header_copy.hlen, MIN(header_copy.dlen, sizeof(env_buffer)));
        gd->env_addr  = (uint32_t)env_buffer;
        gd->env_valid = 1;
        early_env_addr = nand_addr;
        return 0;
    }


    return 0;
}

#endif /* CFG_ENV_IS_IN_OCTEON_NAND */
