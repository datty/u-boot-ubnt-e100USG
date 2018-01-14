/*
 */

#include <common.h>

/* Environment is in DRAM, and was placed there by an external entity.
** This is only valid for "ram based" bootloader images which don't initialized
** the dram controller. (pci/remote boot, nand stage 3)
*/
#if defined(CFG_ENV_IS_IN_RAM) 

#include <command.h>
#include <environment.h>
#include <linux/stddef.h>
#include <malloc.h>
#include "cvmx-bootloader.h"


extern uchar default_environment[];
extern int default_environment_size;
env_t *env_ptr = NULL;

char * env_name_spec = "RAM";

/* Read from the 'special' ??? environment, before running from RAM. */
uchar env_get_char_spec (int index)
{
    DECLARE_GLOBAL_DATA_PTR;
    return ( *((uchar *)(gd->env_addr + index)) );
}

/* Copy environment from special early boot location to normal location.
** env_ptr has been moved by caller to a malloced buffer. */
void env_relocate_spec (void)
{
    memcpy(env_ptr->data, ((env_t *)U_BOOT_RAM_ENV_ADDR)->data, ENV_SIZE);
    /* Clear memory that we borrowed */
    memset((void *)U_BOOT_RAM_ENV_ADDR, 0, CFG_ENV_SIZE);
}

int saveenv(void)
{
    printf("saveenv not supported for environment in RAM.\n");
    return -1;
}

/* This won't work unless env_init is moved to after the serial port is active.
** Normally this is done before that (so that baud rate, etc can be in the environment.
*/
#if 1
#define special_debug_printf(...)
#else
#define special_debug_printf    printf
#endif

/************************************************************************
 * Initialize Environment use
 *
 * We are still running from ROM, so data use is limited
 * Use a (moderately small) buffer on the stack
 */
int env_init(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    env_ptr = (void *)U_BOOT_RAM_ENV_ADDR;

    if (crc32(0, env_ptr->data, ENV_SIZE) == env_ptr->crc)
    {
        gd->env_addr  = (ulong)&(env_ptr->data);
        gd->env_valid = 1;
    }
    else
    {
        special_debug_printf("computed: 0x%08x, read: 0x%08x\n", crc32(0, env_ptr->data, ENV_SIZE), env_ptr->crc);
        special_debug_printf("c_ptr: %p, size: 0x%x\n", env_ptr->data, ENV_SIZE);
        gd->env_addr  = (ulong)&default_environment[0];
        gd->env_valid = 0;
    }
    return 0;
}

#endif /* CFG_ENV_IS_IN_OCTEON_NAND */
