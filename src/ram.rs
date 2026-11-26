//! The flight core runs from internal SRAM: the copy of `.ram_text` (core 1's code and all constants, selected in `memory.x`) from its load image in flash to SRAM.
//! cortex-m-rt calls `__pre_init` before it initialises `.data` and `.bss`; embassy-rp already defines it, so the link wraps it (`--wrap=__pre_init`, `build.rs`) and this copy runs first, then embassy-rp's.
//! Assembly, because no Rust may run before RAM is initialised.

core::arch::global_asm!(
    ".section .text.__wrap___pre_init,\"ax\",%progbits",
    ".global __wrap___pre_init",
    ".type __wrap___pre_init,%function",
    ".thumb_func",
    "__wrap___pre_init:",
    "ldr r0, =__sram_text",
    "ldr r1, =__eram_text",
    "ldr r2, =__siram_text",
    "0:",
    "cmp r1, r0",
    "beq 1f",
    "ldm r2!, {{r3}}",
    "stm r0!, {{r3}}",
    "b 0b",
    "1:",
    "dsb",
    "isb",
    "b.w __real___pre_init",
    ".ltorg",
    ".size __wrap___pre_init, . - __wrap___pre_init",
);
