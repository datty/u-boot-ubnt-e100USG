#!/bin/sh

# Note that the UBOOT_RELOCATED_TEXT_BASE will need to be adjusted when the
# relocation address changes due to size changes of the bootloader.

UBOOT_START=`mips64-octeon-linux-gnu-nm -Bn u-boot | grep _start | head -1 | awk '{print $1}' | tr '[a-f]' '[A-F]'`
UBOOT_END=`mips64-octeon-linux-gnu-nm -Bn u-boot | grep uboot_end | tail -1 | awk '{print $1}' | tr '[a-f]' '[A-F]'`

# addr = CFG_SDRAM_BASE + MIN(gd->ram_size, (1*1024*1024));
CFG_SDRAM_BASE=80100000

UBOOT_RELOCATED_TEXT_BASE=`bc << EOF
define and(x,y) {
 auto z, t, xx, yy, os;
 os=scale;scale=0
 z=0;t=1;for(;x||y;){
  xx=x/2;yy=y/2
  z+=t*((x-2*xx)&&(y-2*yy))
  t*=2;x=xx;y=yy
 }
 scale=os;return (z)
}

obase = 16
ibase = 16

uboot_length = $UBOOT_END - $UBOOT_START

# round down to next 4 kB limit.
# addr &= ~(4096 - 1);

uboot_addr = and($CFG_SDRAM_BASE, FFFFF000)

# Reserve memory for U-Boot code, data & bss
# addr -= MAX(len, (512*1024));

if (uboot_length > 80000) {
    uboot_addr -= uboot_length
} else {
    uboot_addr -= 80000
}

# round down to next 64k (allow us to create image at same addr for debugging)*/
# addr &= ~(64 * 1024 - 1);

and(uboot_addr, FFFF0000)

quit
EOF
`
echo "0x$UBOOT_RELOCATED_TEXT_BASE"
