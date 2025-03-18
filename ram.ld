/* RT1062 RAM-based Linker Script */
MEMORY {
    OCRAM (rwx) : ORIGIN = 0x20200000, LENGTH = 128K  /* SRAM */
}

ENTRY(Reset_Handler)

SECTIONS {
    .vectors : {
        KEEP(*(.vectors))
    } > OCRAM

    .text : {
        *(.text*)
        *(.rodata*)
    } > OCRAM

    .data : {
        *(.data*)
    } > OCRAM

    .bss : {
        *(.bss*)
        *(COMMON)
    } > OCRAM
}

