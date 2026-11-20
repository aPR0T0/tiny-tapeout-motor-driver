/* tt_um_apr0t0_foc_mcu: code and constants execute in place from QSPI flash,
   128 bytes of on-chip RAM for .data/.bss/stack. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 16M
  RAM   (rwx) : ORIGIN = 0x10000000, LENGTH = 128
}

ENTRY(_start)

SECTIONS
{
  .text : {
    KEEP(*(.text.start))
    *(.text .text.*)
    *(.rodata .rodata.* .srodata .srodata.*)
    . = ALIGN(4);
  } > FLASH

  .data : {
    _data_start = .;
    *(.data .data.* .sdata .sdata.*)
    . = ALIGN(4);
    _data_end = .;
  } > RAM AT > FLASH
  _data_load = LOADADDR(.data);

  .bss (NOLOAD) : {
    _bss_start = .;
    *(.bss .bss.* .sbss .sbss.* COMMON)
    . = ALIGN(4);
    _bss_end = .;
  } > RAM

  _stack_top = ORIGIN(RAM) + LENGTH(RAM);
  _ram_used  = _bss_end - ORIGIN(RAM);
  ASSERT(_ram_used <= 64, "static RAM use > 64 bytes, leaves too little stack")

  /DISCARD/ : { *(.comment) *(.riscv.attributes) *(.note*) *(.eh_frame*) }
}
