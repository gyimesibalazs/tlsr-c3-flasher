@ Start-up of the LN882H loader: the boot header the ROM reads, the vector table, and the reset handler.

  .syntax unified
  .cpu cortex-m4
  .thumb

@ The header at the load address. The ROM takes the load address and the vector table address from it; build.py
@ fills the length, the CRC-32 of the image from crc_offset on, and the CRC-32 of the header's first 20 bytes.
  .section .boot_header, "a"
  .word 0x20000000                  @ load address
  .short 0xFFFF                     @ image length (build.py)
  .short 0x0020                     @ crc_offset: the image CRC covers the image from here
  .word 0xFFFFFFFF                  @ image CRC-32 (build.py)
  .word __vectors                   @ vector table, 256-byte aligned
  .word 0                           @ code read protection flag: off
  .word 0xFFFFFFFF                  @ header CRC-32 (build.py)
  .word 0                           @ version
  .word 0                           @ reserved

@ Interrupts stay off, so any entry but the reset is a fault or a stray interrupt: Fault_Handler reports it.
  .section .vectors, "a"
  .global __vectors
__vectors:
  .word __stack_top
  .word Reset_Handler
  .rept 14
  .word Fault_Handler
  .endr
  .rept 38                          @ the chip's interrupts 0..37
  .word Fault_Handler
  .endr

  .text
  .thumb_func
  .global Reset_Handler
Reset_Handler:
  mrs r7, primask
  cpsid i
  ldr r4, =0xE000ED08               @ VTOR
  ldr r5, [r4]                      @ what the ROM left, for the info answer
  mov r6, sp
  mrs r0, control
  orr r7, r7, r0, lsl #8
  mrs r0, ipsr
  orr r7, r7, r0, lsl #16           @ IPSR << 16 | CONTROL << 8 | PRIMASK
  movs r0, #0
  msr control, r0                   @ main stack, privileged, no floating point context (the loader is soft-float)
  isb
  ldr r0, =__stack_top
  msr msp, r0
  ldr r0, =__vectors
  str r0, [r4]
  dsb
  ldr r0, =__bss_start
  ldr r1, =__bss_end
  movs r2, #0
1:
  cmp r0, r1
  bhs 2f
  str r2, [r0], #4
  b 1b
2:
  ldr r0, =rom_vtor
  str r5, [r0]
  ldr r0, =rom_sp
  str r6, [r0]
  ldr r0, =rom_ctrl
  str r7, [r0]
  bl main
  b .

  .thumb_func
  .global Fault_Handler
Fault_Handler:
  mrs r0, msp
  mrs r1, ipsr
  b fault_report

  .bss
  .align 2
  .global rom_vtor, rom_sp, rom_ctrl
rom_vtor: .space 4
rom_sp: .space 4
rom_ctrl: .space 4
