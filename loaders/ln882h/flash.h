// SPI NOR flash commands over hal_spi: standard single-line commands only, 24-bit addresses.
#pragma once
#include <stdint.h>

#define FL_SECTOR 0x1000u
#define FL_BLOCK 0x10000u
#define FL_PAGE 0x100u

extern uint32_t fl_detail;              // the detail of the last error (address or status register)

int fl_wake(void);                                       // AB: out of deep power-down
int fl_id(uint32_t *id);                                 // 9F: maker, type, capacity
int fl_uid(uint8_t uid[16]);                             // 4B: unique id
int fl_otp_mac(uint8_t mac[6]);                          // 48: security register, the MAC at 0x100
int fl_status(uint8_t sr[3]);                            // 05, 35, 15
int fl_read(uint32_t addr, uint8_t *buf, uint32_t n);    // 03
int fl_erase(uint32_t addr, int block);                  // 20 (4 k) or D8 (64 k), waits until done
int fl_program(uint32_t addr, const uint8_t *data, uint32_t n);   // 02, n <= 256 inside one page, waits
int fl_write_sr(const uint8_t *sr, uint32_t n);          // 01 with 1 or 2 bytes, waits
