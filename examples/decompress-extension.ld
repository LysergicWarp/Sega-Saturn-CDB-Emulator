/* decompress-extension.s at the start of the expansion area, where the expansion loader decodes it.
   The loader takes up to 0x5000 encoded bytes, the last 8 the check value, so the image is at most
   0x4ff8 bytes. */
ENTRY(resident_start)
SECTIONS
{
    .text 0x0907b000 : { *(.text) }
    /DISCARD/ : { *(.data) *(.bss) *(.comment) }
}
ASSERT(resident_end <= 0x0907fff8, "resident image exceeds the expansion area")
