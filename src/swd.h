// Arm Serial Wire Debug, bit-banged: line protocol, debug port, memory access port.
#pragma once
#include <stdint.h>

enum SwdAck { SWD_OK = 1, SWD_WAIT = 2, SWD_FAULT = 4, SWD_NOACK = 7, SWD_PARITY = 8 };

void swd_begin();                       // take over the pins (SWS drivers must be torn down first)
void swd_end();                         // release the pins
void swd_set_khz(uint32_t khz);
uint32_t swd_khz();

// line reset + JTAG-to-SWD (and the dormant wake-up when that is not enough); reads IDCODE
int swd_connect(uint32_t *idcode);
// multi-drop SW-DP (RP2040): dormant wake-up, line reset, TARGETSEL, then IDCODE
int swd_connect_multidrop(uint32_t targetsel, uint32_t *idcode);
// debug power-up, sticky errors cleared
int swd_power_up();

int swd_dp_read(uint8_t addr, uint32_t *v);
int swd_dp_write(uint8_t addr, uint32_t v);
int swd_ap_read(uint8_t ap, uint8_t addr, uint32_t *v);
int swd_ap_write(uint8_t ap, uint8_t addr, uint32_t v);

// memory through MEM-AP `ap`, 32-bit words, addr and len multiples of 4
int swd_mem_read(uint8_t ap, uint32_t addr, uint8_t *out, int len);
int swd_mem_write(uint8_t ap, uint32_t addr, const uint8_t *data, int len);

// one 16-bit write (STM32F0/F1/F3 flash programming needs half-word bus writes)
int swd_mem_write16(uint8_t ap, uint32_t addr, uint16_t v);

// Cortex-M core registers (r0..r12 = 0..12, sp 13, lr 14, pc 15, xpsr 16, msp 17, psp 18); core halted
int swd_core_reg_read(uint8_t reg, uint32_t *v);
int swd_core_reg_write(uint8_t reg, uint32_t v);

// cJTAG (IEEE 1149.7, two wires: TCKC = SWCLK wire, TMSC = SWDIO wire, OScan1 format) to a TI ICEPick-C
// router (CC13xx / CC26xx): brings the TAP online, enables the Cortex-M debug TAP behind the router and
// switches the memory / core functions above to its JTAG-DP. Returns the ICEPick IDCODE and the DAP IDCODE.
int ti_cjtag_connect(uint32_t *icepick_id, uint32_t *dap_id);

const char *swd_ack_name(int ack);
