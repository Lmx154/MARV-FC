MEMORY {
    /*
     * The RP2350 has either external or internal flash.
     *
     * 2 MiB is a safe default here, although a Pico 2 has 4 MiB.
     * The last 8 KB are not program space: they are the SETTINGS region (below), so FLASH
     * ends at 0x101FE000.
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2040K
    /*
     * The settings record, the last 8 KB of the 2 MiB flash (two 4 KB erase sectors).
     * Design: _plan/decision-foundation-20261113.md in the MARV-V2-drivers repository, item 1
     * (FLASH shrinks to 2040K; the settings region is 0x101FE000 to 0x10200000).
     * Nothing is placed here by the linker; the firmware reads and writes it by address.
     */
    SETTINGS : ORIGIN = 0x101FE000, LENGTH = 8K
    /*
     * RAM consists of 8 banks, SRAM0-SRAM7, with a striped mapping.
     * This is usually good for performance, as it distributes load on
     * those banks evenly.
     */
    RAM : ORIGIN = 0x20000000, LENGTH = 512K
    /*
     * RAM banks 8 and 9 use a direct mapping. They can be used to have
     * memory areas dedicated for some specific job, improving predictability
     * of access times.
     * Example: Separate stacks for core0 and core1.
     */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
    /*
     * The 16 MiB PSRAM in socket U24 (QMI chip select 1), through its UNCACHED alias (decision-psram-offload-20261121,
     * D2/P6): the cached alias 0x11000000 would write 2.6 times slower and put lines in the XIP cache shared with
     * core 0's flash code. Only src/psram.rs's `PsramBuf` statics go here, core 0's thread-mode tasks only, and
     * nothing in it is initialised or atomic: the chip may be missing, and an atomic read-modify-write outside
     * 0x20000000-0x20082000 hangs the core (RP2350 datasheet 2.1.6.2).
     */
    PSRAM : ORIGIN = 0x15000000, LENGTH = 16M
}

SECTIONS {
    /* ### Boot ROM info
     *
     * Goes after .vector_table, to keep it in the first 4K of flash
     * where the Boot ROM (and picotool) can find it
     */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH

} INSERT AFTER .vector_table;

/* .text starts after the boot info and the load image of .ram_text: see the end of this file. */

SECTIONS {
    /* ### Picotool 'Binary Info' Entries
     *
     * Picotool looks through this block (as we have pointers to it in our
     * header) to find interesting information.
     */
    .bi_entries : ALIGN(4)
    {
        /* We put this in the header */
        __bi_entries_start = .;
        /* Here are the entries */
        KEEP(*(.bi_entries));
        /* Keep this block a nice round size */
        . = ALIGN(4);
        /* We put this in the header */
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    /* ### Boot ROM extra info
     *
     * Goes after everything in our program, so it can contain a signature.
     */
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH

} INSERT AFTER .uninit;

/* The PSRAM statics (src/psram.rs): NOLOAD, so probe-rs never programs 0x15000000 and nothing is copied at start-up;
 * their contents are undefined until core 0 has written them. */
SECTIONS {
    .psram (NOLOAD) : ALIGN(8)
    {
        __spsram = .;
        *(.psram .psram.*);
        . = ALIGN(8);
        __epsram = .;
    } > PSRAM
} INSERT AFTER .uninit;
ASSERT(__spsram >= ORIGIN(PSRAM) && __epsram <= ORIGIN(PSRAM) + LENGTH(PSRAM) && ORIGIN(PSRAM) == 0x15000000, "psram: .psram outside the uncached alias 0x15000000-0x16000000");

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);

/* Flight core in SRAM (decision-ram-code-20261115). INSERT BEFORE .text is load-bearing: an input
 * section goes to the FIRST rule in script order that matches it; after cortex-m-rt's .text/.rodata
 * these rules would match nothing. Patterns are v0-mangled names. They are the mechanism, not the
 * proof: tools/check-ram-code.py is the guard. The copy is `__wrap___pre_init` in src/ram.rs. */
SECTIONS {
    .ram_text : ALIGN(8)
    {
        __sram_text = .;
        *(.text.SIO_IRQ_FIFO .text.DMA_IRQ_0 .text.IO_IRQ_BANK0 .text.PIO2_IRQ_0)
        *(.text._critical_section_1_0_acquire .text._critical_section_1_0_release .text._embassy_time_now)
        *(.text.__aeabi_mem* .text.*17compiler_builtins3mem*)
        /* 64-bit division: dshot's PioDshot::new (core 1's start) divides u64 by u32 in some inlining layouts. */
        *(.text.__aeabi_uldivmod .text.__udivmoddi4 .text.*17compiler_builtins3int19specialized_div_rem*)
        /* The shadow Eskf (packet E2): signed 64-bit division and i64-to-float; the Stepper's array compare. */
        *(.text.__aeabi_ldivmod .text.__divmoddi4 .text.__aeabi_l2f .text.memcmp)
        *(.text.*8icm45686* .text.*11lsm6dsk320x* .text.*5dshot*)
        *(.text.*9fw_flight3imu* .text.*9fw_flight8liveness* .text.*9fw_flight3esc*)
        /* The flight step (IMU1's task calls it): fw_flight's step, gnc and gate, and the binary's Stepper; flight::stat (core 0) stays in flash. */
        *(.text.*9fw_flight4step* .text.*9fw_flight3gnc* .text.*9fw_flight4gate* .text.*16marv_v2_firmware6flight*7Stepper*)
        /* What the step runs (step 5): the estimator, controller, allocator and their helpers, the gate's handshake
         * calls, libm, and the soft double-precision and integer-to-float routines libm's sinf/cosf and Mahony call. */
        *(.text.*9fw_flight5alloc* .text.*9fw_flight6mahony* .text.*9fw_flight3pid* .text.*9fw_flight4quat* .text.*9fw_flight6tuning*)
        *(.text.*7fw_core9handshake* .text.*4libm4math*)
        *(.text.__aeabi_d* .text.__aeabi_f2d .text.__aeabi_i2d .text.__aeabi_ul2f .text.*17compiler_builtins5float*)
        *(.text.*16marv_v2_firmware4imu1* .text.*16marv_v2_firmware4imu2*)
        /* The N0 bench (decision-nav-eskf-20261120): fw_flight::eskf_n0 and the bench binary eskf_bench; nothing in the flight image matches. */
        *(.text.*9fw_flight7eskf_n0* .text.*10eskf_bench*)
        /* MahonyBaro (decision-nav-eskf-20261120, B1): the flight image's steering estimator since packet E2. */
        *(.text.*9fw_flight11mahony_baro*)
        /* Eskf, the shadow filter (decision-nav-eskf-20261120, E1b): the flight image runs it in shadow since packet E2. */
        *(.text.*9fw_flight4eskf*)
        /* EskfSteer and SteerMix (Amendments G and G-1): the steering estimator the flight image runs since packet P-G6. */
        *(.text.*9fw_flight8eskf_out*)
        /* The flight log (decision-flight-logging-20261119): the dflog encoders the Stepper calls; core 0's stay in flash. */
        *(.text.*7fw_core5dflog10encode_imu* .text.*7fw_core5dflog10encode_nav* .text.*7fw_core5dflog11encode_setp*)
        *(.text.*7fw_core5dflog10encode_dmd* .text.*7fw_core5dflog10encode_mot* .text.*7fw_core5dflog10encode_esc*)
        *(.text.*7fw_core5dflog10encode_rcs*)
        /* The navigation records (packet E2). */
        *(.text.*7fw_core5dflog11encode_nkf1* .text.*7fw_core5dflog11encode_nkf2* .text.*7fw_core5dflog10encode_nki*)
        /* MODE and SPN (decision-nav-eskf-20261120, 8c, F1, F6). */
        *(.text.*7fw_core5dflog11encode_mode* .text.*7fw_core5dflog10encode_spn*)
        /* The hands-off state machine (decision-mavlink-mode-20261123, H2b-2b), stepped by Flight::step, and the MSG record
         * the Stepper writes at each change of its state. */
        *(.text.*9fw_flight9hands_off* .text.*7fw_core5dflog10encode_msg*)
        /* The RPM notch and its two records (decision-rpm-notch-20261123): fw_flight::notch, PIDT and NTCH. sinf and cosf are
         * libm::math (above). */
        *(.text.*9fw_flight5notch* .text.*7fw_core5dflog11encode_pidt* .text.*7fw_core5dflog11encode_ntch*)
        /* The shadow guidance (decision-guidance-shadow-20261123): fw_flight::guidance, run by the Stepper, and the GDN record. */
        *(.text.*9fw_flight8guidance* .text.*7fw_core5dflog10encode_gdn*)
        /* The station's flight commands (packet H2c): the Stepper takes and answers them; decode and exchange (core 0) stay in flash. */
        *(.text.*9fw_flight7station* .text.*7fw_core10flight_cmd*7Mailbox4take* .text.*7fw_core10flight_cmd*7Mailbox6answer*)
        *(.text.*7fw_core10flight_cmd10request_of*)
        /* esc: only its core-1 items (the driver's build, the task); its core-0 readers stay in flash. */
        *(.text.*16marv_v2_firmware3esc6driver* .text.*16marv_v2_firmware3esc4task* .text.*16marv_v2_firmware3esc11___task_task*)
        *(.text.*16marv_v2_firmware10imu_inject* .text.*16marv_v2_firmware14flight_latency*)
        *(.text.*16marv_v2_firmware10core1_main* .text.*16marv_v2_firmware12___fbeat_task* .text.*16marv_v2_firmware6now_us*)
        *(.text.*10embassy_rp3spi* .text.*10embassy_rp3dma* .text.*10embassy_rp4gpio* .text.*10embassy_rp3pio* .text.*8pio_core*)
        *(.text.*10embassy_rp9multicore* .text.*10embassy_rp*6RegExt*)
        *(.text.*16embassy_executor3raw5waker* .text.*16embassy_executor3raw*8Executor* .text.*16embassy_executor8platform6thread*)
        *(.text.*12atomic_waker* .text.*12embassy_time5delay9block_for* .text.*12embassy_time7instant*)
        *(.text.*7fw_core6health* .text.*7fw_core5xcore* .text.*7fw_core8rotation* .text.*16embedded_hal_bus3spi* .text.*15embassy_futures4join*)
        /* rustc merges identical functions into aliases, and the section keeps the name of the one it
         * kept: raw::Executor::spawn is SyncExecutor::spawn's, raw::Executor::poll is
         * InterruptExecutor::on_interrupt's, _critical_section_1_0_* are RpSpinlockCs's Impl. */
        *(.text.*16embassy_executor3raw*12SyncExecutor* .text.*16embassy_executor8platform9interrupt*17InterruptExecutor12on_interrupt)
        *(.text.*10embassy_rp21critical_section_impl*)
        . = ALIGN(4);
        __eram_code = .;
        *(.rodata .rodata.*)
        . = ALIGN(4);
        __eram_text = .;
    } > RAM AT> FLASH
    __siram_text = LOADADDR(.ram_text);
} INSERT BEFORE .text;

_stext = ALIGN(LOADADDR(.ram_text) + SIZEOF(.ram_text), 8);
__vector_table_end = ADDR(.vector_table) + SIZEOF(.vector_table);
ASSERT(SIO_IRQ_FIFO >= ORIGIN(RAM) && SIO_IRQ_FIFO < ORIGIN(RAM) + LENGTH(RAM), "ram-code: SIO_IRQ_FIFO not in RAM: .ram_text rules match nothing");
ASSERT(DMA_IRQ_0 >= ORIGIN(RAM) && DMA_IRQ_0 < ORIGIN(RAM) + LENGTH(RAM), "ram-code: DMA_IRQ_0 not in RAM");
ASSERT(IO_IRQ_BANK0 >= ORIGIN(RAM) && IO_IRQ_BANK0 < ORIGIN(RAM) + LENGTH(RAM), "ram-code: IO_IRQ_BANK0 not in RAM");
ASSERT(__erodata == __srodata, "ram-code: constants left in flash");
ASSERT(__siram_text % 4 == 0 && __eram_text % 4 == 0, "ram-code: .ram_text not word aligned");
/* Core 0's stack: from _stack_start (the top of RAM) down to _stack_end, the end of the statics.
 * cortex-m-rt's own PROVIDE(_stack_end = .) follows .uninit, but .end_block (above) is inserted after
 * .uninit and leaves the location counter in FLASH, so that default is a flash address. Defining the
 * symbol here makes the PROVIDE a no-op. embassy_rp::install_core0_stack_guard (src/main.rs) writes
 * it to MSPLIM; ALIGN(8) because MSPLIM bits 2:0 read as zero. */
_stack_end = ALIGN(__euninit, 8);
ASSERT(_stack_end >= ORIGIN(RAM) && _stack_end < ORIGIN(RAM) + LENGTH(RAM), "stack guard: _stack_end not in RAM");
/* Core 0's stack budget (stack S1: static call chains of the release ELF, push + sub sp along direct
 * calls and tail calls, thunks resolved; interrupt priorities as set: SWI_IRQ_0 P3, SWI_IRQ_1 P1,
 * the hardware interrupts P0; each nesting level adds a 104 B FP exception frame, counted as 108 B).
 * Restated 2026-11-21 for packet P-G6b (the steering estimator SteerMix over STEER_CORE and VERT, three Eskf-sized
 * builders in src/flight.rs; image of branch p2-g6, 16 668 B frame for each of the two big builders):
 * At boot, before rc::start and the watchdog monitor: the thread base 240 B (main, Executor::run, poll) + the main
 * task's poll 4 384 B + the largest builder flight::steer_core()/vert() 16 668 B (they run one after the other, not
 * nested) + one P0 interrupt (TIMER0_IRQ_0, 224 + 108 B) = 21 624 B. The same count on the previous image (df7e46e,
 * steer() 33 320 B, poll 3 264 B) gives 37 156 B against the 38 236 B of S1's count, so 1 080 B is added for
 * the difference between the two counts: 22 704 B. After start-up (S1, unchanged by P-G6b: no core-0 task other than
 * main's poll and flight::stat changed): 240 + the deepest other thread task (sd, 13 088 B) + SWI_IRQ_0 (rc, 11 880 + 24 + 108) +
 * SWI_IRQ_1 (monitor, 400 + 24 + 108) + one P0 interrupt (332) = 26 204 B. The budget is the larger chain, 26 204 B,
 * plus the 2 KiB margin for what the analysis cannot bound (calls through vtables and wakers, the boot ROM's flash
 * routines). Redo the figures when a frame grows; the `[STAT] watchdog:` line prints both cores' high-water marks.
 * Measured stack of that image: 31 440 B (_stack_end 0x20078530). */
ASSERT(_stack_start - _stack_end >= 26204 + 2048, "stack: core 0's stack under its budget (26 204 B after start-up + 2 KiB)");
