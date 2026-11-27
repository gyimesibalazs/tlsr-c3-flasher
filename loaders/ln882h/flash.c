// SPI NOR flash commands for the LN882H loader (the standard 25-series command set; see flash.h).
#include "flash.h"
#include "hal.h"
#include "loader.h"

// Status polls before a program, erase or status write counts as hung. A poll is a 2-byte transfer, at least a few
// microseconds at the loader's SPI clock, so this is ten seconds or more: far above a 64 k block erase (2 s at most
// on the usual 2 Mbyte parts).
#define FL_POLLS 4000000u

uint32_t fl_detail;

static int cmd1(uint8_t op) {
  return hal_spi(&op, 1, 0, 0);
}

static int cmd_addr(uint8_t op, uint32_t addr, const uint8_t *data, uint32_t n, uint8_t *rx, uint32_t nrx) {
  uint8_t tx[4 + FL_PAGE];
  tx[0] = op;
  tx[1] = (uint8_t)(addr >> 16);
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)addr;
  for (uint32_t i = 0; i < n; i++) tx[4 + i] = data[i];
  return hal_spi(tx, 4 + n, rx, nrx);
}

static int sr1(uint8_t *v) {
  uint8_t op = 0x05;
  return hal_spi(&op, 1, v, 1);
}

static int wait_ready(void) {
  uint8_t s = 0;
  for (uint32_t i = 0; i < FL_POLLS; i++) {
    int e = sr1(&s);
    if (e) return e;
    if (!(s & 1)) return LN_OK;
  }
  fl_detail = s;
  return LN_E_BUSY;
}

// 06, then the latch must show in status register 1 (it does not when the flash ignores the command)
static int write_enable(void) {
  uint8_t s = 0;
  int e = cmd1(0x06);
  if (!e) e = sr1(&s);
  if (e) return e;
  if (!(s & 2)) {
    fl_detail = s;
    return LN_E_WEL;
  }
  return LN_OK;
}

int fl_wake(void) {
  int e = cmd1(0xAB);
  hal_spin(50);                         // tRES1: 3..30 us on the usual parts
  return e;
}

int fl_id(uint32_t *id) {
  uint8_t op = 0x9F, b[3];
  int e = hal_spi(&op, 1, b, 3);
  *id = (uint32_t)b[0] << 16 | (uint32_t)b[1] << 8 | b[2];
  return e;
}

int fl_uid(uint8_t uid[16]) {
  uint8_t tx[5] = {0x4B, 0, 0, 0, 0};   // four dummy bytes
  return hal_spi(tx, 5, uid, 16);
}

int fl_otp_mac(uint8_t mac[6]) {
  uint8_t tx[5] = {0x48, 0x00, 0x01, 0x00, 0};   // security register address 0x000100, one dummy byte
  return hal_spi(tx, 5, mac, 6);
}

int fl_status(uint8_t sr[3]) {
  static const uint8_t ops[3] = {0x05, 0x35, 0x15};
  for (int i = 0; i < 3; i++) {
    int e = hal_spi(&ops[i], 1, &sr[i], 1);
    if (e) return e;
  }
  return LN_OK;
}

int fl_read(uint32_t addr, uint8_t *buf, uint32_t n) {
  return cmd_addr(0x03, addr, 0, 0, buf, n);
}

int fl_erase(uint32_t addr, int block) {
  int e = write_enable();
  if (!e) e = cmd_addr(block ? 0xD8 : 0x20, addr, 0, 0, 0, 0);
  if (!e) e = wait_ready();
  return e;
}

int fl_program(uint32_t addr, const uint8_t *data, uint32_t n) {
  int e = write_enable();
  if (!e) e = cmd_addr(0x02, addr, data, n, 0, 0);
  if (!e) e = wait_ready();
  return e;
}

int fl_write_sr(const uint8_t *sr, uint32_t n) {
  uint8_t tx[3] = {0x01, sr[0], n > 1 ? sr[1] : 0};
  int e = write_enable();
  if (!e) e = hal_spi(tx, 1 + n, 0, 0);
  if (!e) e = wait_ready();
  return e;
}
