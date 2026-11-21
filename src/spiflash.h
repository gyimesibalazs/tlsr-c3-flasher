// SPI NOR flash (25-series) on the flasher's existing wires: CS = reset wire, CLK = SWCLK wire,
// DI = SWS wire (driven through its resistor), DO = UART bridge TX wire (GPIO 21).
#pragma once
#include <stdint.h>

void spif_begin(uint32_t khz);              // pins to SPI, CS high
void spif_end();                            // every pin back to its idle state (inputs, reset wire released)
bool spif_active();
uint32_t spif_khz();

void spif_xfer(const uint8_t *out, int nout, uint8_t *in, int nin);   // one CS-low transaction
void spif_addr4(bool on);                   // 4-byte addresses with the 4-byte opcodes (0x13, 0x12, 0x21, 0xDC)
bool spif_addr4_on();

bool spif_read(uint32_t addr, uint8_t *out, int n);
uint8_t spif_status();

enum {
  SPIF_OK = 0,
  SPIF_ERR_WEL = 1,                         // write enable did not stick: write protect pin or locked status
  SPIF_ERR_TIMEOUT = 2,
};
int spif_erase(uint32_t addr, uint8_t op, uint32_t timeout_ms);   // op 0x20 (4 k), 0x52 (32 k), 0xD8 (64 k)
int spif_erase_chip(uint32_t timeout_ms);
int spif_program(uint32_t addr, const uint8_t *data, int n);      // within one 256-byte page
