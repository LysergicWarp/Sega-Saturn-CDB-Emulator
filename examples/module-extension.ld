/* module-extension.s at the start of the expansion area, where the expansion loader decodes it.
   The whole image is decoded there, and it must end before the module slot at 0x0907c000. */
ENTRY(resident_start)
SECTIONS
{
    .text 0x0907b000 : { *(.text) }
    /DISCARD/ : { *(.data) *(.bss) *(.comment) }
}
ASSERT(resident_end <= 0x0907c000, "resident image reaches the module slot")
