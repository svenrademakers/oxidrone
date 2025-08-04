/* STM32H743VIT6 Linker Script
 *
 * Flash: 2 MB starting at 0x0800_0000
 * SRAM: Multiple banks, here we lump them together for simplicity:
 *   DTCM SRAM 128 KB @ 0x2000_0000
 *   AXI SRAM 512 KB @ 0x2400_0000
 *   SRAM1/2/3/4 in D2 domain
 */

MEMORY
{
    /* Main flash bank */
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K

    /* Tightly Coupled Memory (fastest, no bus arbitration) */
    DTCM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K

    /* AXI SRAM (biggest contiguous block, good for heap/stack) */
    RAM (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K

    /* Optional: SRAM1/2/3/4 in D2 domain (used for DMA, Ethernet, etc.) */
    SRAM1 (xrw) : ORIGIN = 0x30000000, LENGTH = 128K
    SRAM2 (xrw) : ORIGIN = 0x30020000, LENGTH = 128K
    SRAM3 (xrw) : ORIGIN = 0x30040000, LENGTH = 32K
    SRAM4 (xrw) : ORIGIN = 0x38000000, LENGTH = 64K
}

/* Define the default memory regions for embedded Rust runtime (cortex-m-rt) */
REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);   /* where .data gets loaded into */
REGION_ALIAS("REGION_BSS", RAM);    /* zero-initialized vars */
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
