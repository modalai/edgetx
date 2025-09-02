/* Highest heap address */
HEAP_ADDRESS = ORIGIN(RAM_D2) + LENGTH(RAM_D2) - 16;

/* Highest address of the user mode stack */
STACK_ADDRESS = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM) - 16;    /* end of RAM */

/* Maximum bootloader code size */
BOOTLOADER_SIZE = 0x20000; /* Align to 128KB sector boundary on STM32H753 */

/* Required amount of stack for interrupt stack (Main stack) */
MAIN_STACK_SIZE = 8192;

/* Generate a link error if heap and stack don't fit into RAM */
MIN_HEAP_SIZE = 0;      /* required amount of heap  */

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 2048K
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3 (xrw)   : ORIGIN = 0x38000000, LENGTH = 64K
}

REGION_ALIAS("REGION_TEXT_STORAGE", FLASH);
REGION_ALIAS("REGION_TEXT",         ITCMRAM); /* Todo: setting this to ITCMRAM this should fix bootloader instruction cache?*/
REGION_ALIAS("REGION_ISR_VECT",     DTCMRAM);
REGION_ALIAS("REGION_DATA",         DTCMRAM);
REGION_ALIAS("REGION_BSS",          RAM_D1);
REGION_ALIAS("REGION_RAM",          RAM_D1); /* used for DMA, capable regions: D1,D2*/
