/* transform-module.s in the module slot of module-extension.s, where its load copies the image. */
ENTRY(module_start)
SECTIONS
{
    .text 0x0907c000 : { *(.text) }
    /DISCARD/ : { *(.data) *(.bss) *(.comment) }
}
ASSERT(module_end <= 0x09080000, "module image exceeds the module slot")
