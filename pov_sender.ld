MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
	. = ALIGN(4);
        _etext = .;
    } > FLASH

    .data :
    {
        _data = .;
        *(vtable)
        *(.data*)
	. = ALIGN(4);
        _edata = .;
    } > SRAM AT > FLASH

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
	. = ALIGN(4);
        _ebss = .;
    } > SRAM
}
