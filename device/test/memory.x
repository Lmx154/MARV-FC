/* RP2350A Pico 2 memory layout for test binary
 *
 * According to RP2350A docs / Pico 2 board, we assume 2 MiB XIP flash
 * starting at 0x1000_0000 and 520 KiB SRAM total (512 KiB striped as
 * main RAM plus two 4 KiB banks).
 */

MEMORY {
    /* Internal XIP flash on Pico 2 (RP2350A) */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2M

    /* Main SRAM (striped banks) used for data/stack/heap */
    RAM : ORIGIN = 0x20000000, LENGTH = 512K

    /* Direct-mapped banks (optional, left unused by default) */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

/*
 * Boot ROM / picotool integration sections, mirroring rp235x layout
 * but sized for the above FLASH region.
 */

SECTIONS {
    /* Boot info block lives right after vector table in first 4 KiB */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH

} INSERT AFTER .vector_table;

/* Ensure .text starts after boot info block */
_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    /* Picotool binary info entries */
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    /* End block for optional signatures, etc. */
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH

} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
