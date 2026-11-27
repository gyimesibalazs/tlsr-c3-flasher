/* LN882H loader: everything in SRAM0 from 0x20000000, where the ROM loads it. The boot header comes first, the
   vector table at +0x100 (the header points to it), then code, constants and data; the zeroed data and the stack
   follow the image and are not part of it. The ROM's own data is in SRAM3 (0x20040000 on), far above. */
ENTRY(Reset_Handler)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  .header 0x20000000 : { KEEP(*(.boot_header)) } > RAM
  .vectors 0x20000100 : { KEEP(*(.vectors)) } > RAM
  .text : { *(.text .text.*) *(.rodata .rodata.*) . = ALIGN(4); } > RAM
  .data : { *(.data .data.*) . = ALIGN(4); } > RAM
  .bss (NOLOAD) : { __bss_start = .; *(.bss .bss.* COMMON) . = ALIGN(4); __bss_end = .; } > RAM
  .stack (NOLOAD) : { . = ALIGN(8); . = . + 0x1000; __stack_top = .; } > RAM
  /DISCARD/ : { *(.ARM.exidx* .ARM.extab* .comment .note*) }
}
