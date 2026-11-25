// Beken BK72xx (BK7231N, BK7231T) over its UART download protocol: catch at power-up, link, baud rate, flash
// read / erase / write / CRC, the flash chip's status registers, reboot. See bk72xx.cpp for the protocol.
#pragma once
#include <stdint.h>

enum {
  BK_OK = 0,
  BK_ERR_TIMEOUT = 1,   // no answer in time
  BK_ERR_STATUS = 2,    // the ROM answered with a non-zero status
  BK_ERR_ECHO = 3,      // the answer names another address or value than the command
  BK_ERR_CRC = 4,       // a block did not match the ROM's CRC of it, after the retries
  BK_ERR_GUARD = 5,     // erase / write inside the protected bootloader area (BKG)
  BK_ERR_ALIGN = 6,     // address or length not on a 4 k (64 k) boundary
  BK_ERR_NOLINK = 7,    // not linked: BKL first
  BK_ERR_PROTO = 8,     // the command does not exist in this protocol (BK7231T bootloader)
  BK_ERR_SIZE = 9,      // length out of range
};

#define BK_SECTOR 4096u
#define BK_BLOCK 65536u
#define BK_BOOT_END 0x11000u   // the bootloader in flash: 0x0 .. 0x11000 on BK7231N and BK7231T

struct BkLink {
  bool rom;             // true: BootROM protocol (BK7231N), false: the flash bootloader's (BK7231T)
  uint32_t baud;        // the speed in use after the link
  uint32_t resets;      // reset pulses until the answer
  uint32_t answer_ms;   // from the last reset to the first answer
  bool fallback;        // the requested speed failed, 115200 is used
  uint32_t rx_before;   // bytes that came from the target before it answered (its own firmware)
};

void bk_set_reset(void (*reset)());     // the flasher's reset: RST (to CEN) low, power pins off and on
bool bk_begin();                        // UART_NUM_1 at 115200: TX on GPIO 21, RX on the SWS wire
void bk_end();                          // driver removed, both wires inputs
bool bk_active();
bool bk_linked();

int bk_link(uint32_t catch_ms, uint32_t baud, BkLink *res);   // on an error bk_detail() says what was seen
int bk_read(uint32_t addr, uint8_t *out, int len);             // any range; every 4 k block is checked by CRC
int bk_write4k(uint32_t addr, const uint8_t *data);            // one erased 4 k sector
int bk_erase(uint32_t addr, bool block64);                     // 4 k sector or 64 k block
int bk_crc(uint32_t addr, uint32_t len, uint32_t *crc);        // zlib CRC-32 of the range, computed by the target

const char *bk_err_text(int r);         // error name, with the detail of the last failure
const char *bk_detail();

// The BK... text commands of the host protocol (see README). Returns false if the command is not one of them.
bool bk_command(int argc, char **argv, void (*out)(const char *line));
