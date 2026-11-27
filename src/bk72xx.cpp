// Beken BK72xx over its UART download protocol, written from the protocol as ltchiptool / bk7231tools (MIT)
// implement it and as OpenBeken's BK7231GUIFlashTool uses it (that one only read for the protocol facts).
//
// The download port is the chip's UART1 (P10 = RX1, P11 = TX1), 8N1, 115200 baud after reset. Commands:
//   short  01 E0 FC <n> <code> <n-1 bytes>                    n = 1 + payload, below 0xFF
//   long   01 E0 FC FF F4 <len:u16> <code> <len-1 bytes>      len = 1 + payload
// Answers:
//   short  04 0E <n> 01 E0 FC <code> <n-4 bytes>
//   long   04 0E FF 01 E0 FC F4 <len:u16> <code> <len-1 bytes>
// All numbers little endian. The codes used here:
//   short 0x00 link check (answer code 0x01, one byte 0x00); 0x03 read register (addr -> addr, value);
//         0x0F set baud rate (baud:u32, delay_ms:u8; the same 5 bytes come back after delay_ms, already at the new
//         speed); 0x10 CRC (start, end -> crc); 0x0E reboot (0xA5, no answer)
//   long  0x09 read 4 k (addr -> status, addr, 4096 bytes); 0x07 write 4 k (addr, 4096 bytes -> status, addr);
//         0x0F erase (0x20 = 4 k sector / 0xD8 = 64 k block, addr -> status, size, addr); 0x0C read flash status
//         register (opcode -> status, opcode, value); 0x0D write it (opcode, 1 or 2 bytes -> status, opcode,
//         bytes); 0x0E flash id (opcode 0x9F as u32 -> status, one byte, the 3 JEDEC bytes)
// The CRC is CRC-32 with the zlib polynomial, started at 0xFFFFFFFF and not inverted at the end. The BK7231N's
// BootROM includes the end address in the range, the BK7231T's bootloader does not; that difference also tells
// the two apart here. The BootROM answers only right after a reset: the link check is repeated from the moment the
// target gets its power back. The BK7231T has no ROM loader: its download code is the bootloader in flash, which
// knows only link, baud rate, CRC, read, write, erase and reboot, refuses reads below 0x11000 (they work at the
// same address plus the flash size: the flash ignores the upper address bits) and write protects the flash after
// every CRC until the next link check.

#include <Arduino.h>
#include <stdarg.h>
#include "driver/gpio.h"
#include "config.h"
#include "bk72xx.h"
#include "tuart.h"

static const uint32_t BK_LINK_BAUD = 115200;    // the ROM's speed after reset
static const uint32_t BK_CYCLE_MS = 1200;       // reset period while catching (power-off time included)
static const uint8_t BK_BAUD_DELAY_MS = 50;     // the ROM switches its speed this long after the command
static const uint32_t BK_PROBE_ADDR = BK_BOOT_END;   // a block every protocol can read: the start of the app

static void (*reset_fn)() = NULL;
static bool on = false, linked = false, rom = false;
static uint32_t guard = BK_BOOT_END;            // erase / write refused below this address (BKG)
static uint32_t wrap = 0x200000;                // flash size: addresses wrap there (from the flash id when known)
static char detail[200];

static uint8_t blk[BK_SECTOR];                  // last block read (cache of bk_read)
static uint32_t blk_addr = 0xFFFFFFFF;
static uint8_t aux[BK_SECTOR];                  // the probe block at the link, then the staging buffer of BKB / BKW

static uint8_t rb[256];                         // received bytes not consumed yet
static int rb_pos = 0, rb_len = 0;

// ---------------------------------------------------------------- CRC-32 (zlib)

static uint32_t crc_tab[256];

static uint32_t crc32z(uint32_t crc, const uint8_t *d, uint32_t n) {
  if (!crc_tab[1]) {
    for (uint32_t i = 0; i < 256; i++) {
      uint32_t c = i;
      for (int b = 0; b < 8; b++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
      crc_tab[i] = c;
    }
  }
  crc = ~crc;
  for (uint32_t i = 0; i < n; i++) crc = crc_tab[(crc ^ d[i]) & 0xff] ^ (crc >> 8);
  return ~crc;
}

// ---------------------------------------------------------------- UART

static void put_le32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_le32(const uint8_t *p) {
  return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put(const uint8_t *d, int n) {
  tuart_write(d, n);
}

static void rx_reset() {
  rb_pos = rb_len = 0;
  tuart_flush_input();
}

static int32_t left_ms(uint32_t deadline) { return (int32_t)(deadline - millis()); }

static int rx_byte(uint32_t deadline) {
  if (rb_pos < rb_len) return rb[rb_pos++];
  for (;;) {
    size_t avail = tuart_available();
    int32_t left = left_ms(deadline);
    int got;
    if (avail) got = tuart_read(rb, (int)min(avail, sizeof(rb)), 0);
    else got = tuart_read(rb, 1, left > 0 ? left : 0);
    if (got > 0) {
      rb_len = got;
      rb_pos = 1;
      return rb[0];
    }
    if (left_ms(deadline) <= 0) return -1;
  }
}

static bool rx_bytes(uint8_t *dst, int n, uint32_t deadline) {
  int k = 0;
  while (k < n && rb_pos < rb_len) dst[k++] = rb[rb_pos++];
  while (k < n) {
    int32_t left = left_ms(deadline);
    int got = tuart_read(dst + k, n - k, left > 0 ? left : 0);
    if (got > 0) k += got;
    else if (left <= 0) return false;
  }
  return true;
}

static void send_short(uint8_t code, const uint8_t *p, int n) {
  const uint8_t h[5] = {0x01, 0xE0, 0xFC, (uint8_t)(n + 1), code};
  rx_reset();                                   // stale bytes (answers to repeated link checks) go first
  put(h, 5);
  put(p, n);
}

static void send_long(uint8_t code, const uint8_t *p, int n, const uint8_t *p2 = NULL, int n2 = 0) {
  uint32_t len = 1 + n + n2;
  const uint8_t h[8] = {0x01, 0xE0, 0xFC, 0xFF, 0xF4, (uint8_t)len, (uint8_t)(len >> 8), code};
  rx_reset();
  put(h, 8);
  put(p, n);
  put(p2, n2);
}

// The answer to `code`; anything before it (noise, a stale answer) is skipped. `want`: payload bytes read even
// when the length field says fewer (some BootROM revisions give the flash id answer one byte too short).
// Returns the payload length, or -1 when nothing fitting came in time.
static int recv(uint8_t code, bool lng, uint8_t *out, int max, int want, uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  int prev = -1;
  for (;;) {
    int b = rx_byte(deadline);
    if (b < 0) return -1;
    if (!(prev == 0x04 && b == 0x0E)) {
      prev = b;
      continue;
    }
    prev = -1;
    int n = rx_byte(deadline);
    if (n < 0) return -1;
    if (lng != (n == 0xFF)) continue;
    uint8_t h[4];
    if (!rx_bytes(h, 3, deadline)) return -1;
    if (h[0] != 0x01 || h[1] != 0xE0 || h[2] != 0xFC) continue;
    int len, c;
    if (lng) {
      if (!rx_bytes(h, 4, deadline)) return -1;
      if (h[0] != 0xF4) continue;
      len = (h[1] | h[2] << 8) - 1;
      c = h[3];
    } else {
      c = rx_byte(deadline);
      len = n - 4;
    }
    if (c < 0) return -1;
    if (c != code || len < 0) continue;
    if (len < want) len = want;
    int keep = min(len, max);
    if (!rx_bytes(out, keep, deadline)) return -1;
    for (int i = keep; i < len; i++)
      if (rx_byte(deadline) < 0) return -1;
    return keep;
  }
}

static int fail(int r, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(detail, sizeof(detail), fmt, ap);
  va_end(ap);
  tuart_event("error: %s", detail);
  return r;
}

// ---------------------------------------------------------------- commands

static int link_check(uint32_t timeout_ms) {
  send_short(0x00, NULL, 0);
  uint8_t v = 0xff;
  if (recv(0x01, false, &v, 1, 0, timeout_ms) != 1) return fail(BK_ERR_TIMEOUT, "no answer to the link check");
  return v == 0 ? BK_OK : fail(BK_ERR_STATUS, "link check answered %02x", v);
}

static int link_retry(int tries) {
  int r = BK_ERR_TIMEOUT;
  for (int i = 0; i < tries && r != BK_OK; i++) r = link_check(100);
  return r;
}

// the CRC as the target computes it; on the BK7231T's bootloader the flash is then protected until a link check
static int crc_cmd(uint32_t addr, uint32_t end, uint32_t *raw) {
  uint8_t p[8], a[4];
  put_le32(p, addr);
  put_le32(p + 4, end);
  send_short(0x10, p, 8);
  if (recv(0x10, false, a, 4, 0, 300 + (end - addr) / 80) != 4)
    return fail(BK_ERR_TIMEOUT, "no answer to the CRC of 0x%x..0x%x", (unsigned)addr, (unsigned)end);
  *raw = get_le32(a);
  return BK_OK;
}

int bk_crc(uint32_t addr, uint32_t len, uint32_t *crc) {
  if (!linked) return BK_ERR_NOLINK;
  if (len < 1 || len > 0x1000000) return fail(BK_ERR_SIZE, "CRC length 1 .. 16 M");
  uint32_t raw = 0;
  int r = crc_cmd(addr, rom ? addr + len - 1 : addr + len, &raw);
  if (!rom && r == BK_OK) r = link_retry(3);    // lifts the bootloader's write protection again
  *crc = raw ^ 0xFFFFFFFFu;
  return r;
}

static int read4k(uint32_t addr, uint8_t *out) {
  static uint8_t ans[5 + BK_SECTOR];
  uint8_t p[4];
  put_le32(p, addr);
  send_long(0x09, p, 4);
  int n = recv(0x09, true, ans, sizeof(ans), 0, 200 + (uint32_t)(4200ull * 10 * 1000 / tuart_baud()));
  if (n < 0) return fail(BK_ERR_TIMEOUT, "no answer to the read of 0x%x", (unsigned)addr);
  if (n != (int)sizeof(ans)) return fail(BK_ERR_SIZE, "read of 0x%x: %d bytes", (unsigned)addr, n - 5);
  if (rom && ans[0]) return fail(BK_ERR_STATUS, "read of 0x%x: status %d", (unsigned)addr, ans[0]);
  if (rom && get_le32(ans + 1) != addr)
    return fail(BK_ERR_ECHO, "read of 0x%x answered for 0x%x", (unsigned)addr, (unsigned)get_le32(ans + 1));
  memcpy(out, ans + 5, BK_SECTOR);
  return BK_OK;
}

static bool guarded(uint32_t addr) { return (wrap ? addr % wrap : addr) < guard; }

int bk_write4k(uint32_t addr, const uint8_t *data) {
  if (!linked) return BK_ERR_NOLINK;
  if (addr % BK_SECTOR) return fail(BK_ERR_ALIGN, "write: 0x%x is not on a 4 k boundary", (unsigned)addr);
  if (guarded(addr)) return fail(BK_ERR_GUARD, "write at 0x%x: below 0x%x (bootloader)", (unsigned)addr, (unsigned)guard);
  blk_addr = 0xFFFFFFFF;
  uint8_t p[4], ans[8];
  put_le32(p, addr);
  send_long(0x07, p, 4, data, BK_SECTOR);
  int n = recv(0x07, true, ans, sizeof(ans), 0, 1000 + (uint32_t)(4200ull * 10 * 1000 / tuart_baud()));
  if (n < 0) return fail(BK_ERR_TIMEOUT, "no answer to the write of 0x%x", (unsigned)addr);
  if (rom && (n < 5 || ans[0])) return fail(BK_ERR_STATUS, "write of 0x%x: status %d", (unsigned)addr, ans[0]);
  if (rom && get_le32(ans + 1) != addr)
    return fail(BK_ERR_ECHO, "write of 0x%x answered for 0x%x", (unsigned)addr, (unsigned)get_le32(ans + 1));
  return BK_OK;
}

int bk_erase(uint32_t addr, bool block64) {
  if (!linked) return BK_ERR_NOLINK;
  uint32_t unit = block64 ? BK_BLOCK : BK_SECTOR;
  if (addr % unit) return fail(BK_ERR_ALIGN, "erase: 0x%x is not on a %u k boundary", (unsigned)addr, (unsigned)unit / 1024);
  if (guarded(addr)) return fail(BK_ERR_GUARD, "erase at 0x%x: below 0x%x (bootloader)", (unsigned)addr, (unsigned)guard);
  blk_addr = 0xFFFFFFFF;
  uint8_t p[5], ans[8];
  p[0] = block64 ? 0xD8 : 0x20;
  put_le32(p + 1, addr);
  send_long(0x0F, p, 5);
  int n = recv(0x0F, true, ans, sizeof(ans), 0, block64 ? 8000 : 3000);
  if (n < 0) return fail(BK_ERR_TIMEOUT, "no answer to the erase of 0x%x", (unsigned)addr);
  if (rom && (n < 6 || ans[0])) return fail(BK_ERR_STATUS, "erase of 0x%x: status %d", (unsigned)addr, ans[0]);
  if (rom && (ans[1] != p[0] || get_le32(ans + 2) != addr)) return fail(BK_ERR_ECHO, "erase of 0x%x: other answer", (unsigned)addr);
  return BK_OK;
}

static int read_reg(uint32_t addr, uint32_t *v) {
  if (!rom) return fail(BK_ERR_PROTO, "registers: BootROM only");
  uint8_t p[4], a[8];
  put_le32(p, addr);
  send_short(0x03, p, 4);
  if (recv(0x03, false, a, 8, 0, 300) != 8) return fail(BK_ERR_TIMEOUT, "no answer to the register read");
  if (get_le32(a) != addr) return fail(BK_ERR_ECHO, "register read answered for 0x%x", (unsigned)get_le32(a));
  *v = get_le32(a + 4);
  return BK_OK;
}

static int flash_id(uint8_t id[3]) {
  if (!rom) return fail(BK_ERR_PROTO, "flash id: BootROM only");
  uint8_t p[4], a[8];
  put_le32(p, 0x9F);
  send_long(0x0E, p, 4);
  if (recv(0x0E, true, a, sizeof(a), 5, 300) < 5) return fail(BK_ERR_TIMEOUT, "no answer to the flash id");
  if (a[0]) return fail(BK_ERR_STATUS, "flash id: status %d", a[0]);
  memcpy(id, a + 2, 3);
  return BK_OK;
}

static int sr_read(uint8_t op, uint8_t *v) {
  if (!rom) return fail(BK_ERR_PROTO, "flash status register: BootROM only");
  uint8_t a[4];
  send_long(0x0C, &op, 1);
  if (recv(0x0C, true, a, sizeof(a), 0, 300) != 3) return fail(BK_ERR_TIMEOUT, "no answer to the status read");
  if (a[0]) return fail(BK_ERR_STATUS, "status read: status %d", a[0]);
  if (a[1] != op) return fail(BK_ERR_ECHO, "status read answered for %02x", a[1]);
  *v = a[2];
  return BK_OK;
}

static int sr_write(uint8_t op, const uint8_t *v, int n) {
  if (!rom) return fail(BK_ERR_PROTO, "flash status register: BootROM only");
  if (n < 1 || n > 2) return fail(BK_ERR_SIZE, "status write: 1 or 2 bytes");
  uint8_t p[3] = {op, v[0], n > 1 ? v[1] : (uint8_t)0}, a[4];
  send_long(0x0D, p, 1 + n);
  if (recv(0x0D, true, a, sizeof(a), 0, 500) != 2 + n) return fail(BK_ERR_TIMEOUT, "no answer to the status write");
  if (a[0]) return fail(BK_ERR_STATUS, "status write: status %d", a[0]);
  if (memcmp(a + 1, p, 1 + n)) return fail(BK_ERR_ECHO, "status write answered differently");
  return BK_OK;
}

static void reboot() {
  const uint8_t p = 0xA5;
  send_short(0x0E, &p, 1);
  tuart_wait_tx(50);
  linked = false;
  blk_addr = 0xFFFFFFFF;
  tuart_set_baud(BK_LINK_BAUD);     // the target restarts at its reset speed
}

// ---------------------------------------------------------------- reading with CRC

static int read_block(uint32_t a, uint8_t *dst) {
  int r = BK_OK;
  for (int t = 0; t < 4; t++) {
    r = read4k(a, dst);
    if (r == BK_OK) {
      uint32_t c = 0;
      r = bk_crc(a, BK_SECTOR, &c);
      if (r == BK_OK && c == crc32z(0, dst, BK_SECTOR)) return BK_OK;
      if (r == BK_OK) r = fail(BK_ERR_CRC, "the block at 0x%x does not match the target's CRC", (unsigned)a);
    }
    delay(20);                                  // let a late answer pass, then start clean
  }
  return r;
}

int bk_read(uint32_t addr, uint8_t *out, int len) {
  if (!linked) return BK_ERR_NOLINK;
  while (len > 0) {
    uint32_t a = addr & ~(BK_SECTOR - 1);
    if (a != blk_addr) {
      blk_addr = 0xFFFFFFFF;
      int r = read_block(a, blk);
      if (r) return r;
      blk_addr = a;
    }
    int off = (int)(addr - a), n = min(len, (int)BK_SECTOR - off);
    memcpy(out, blk + off, n);
    out += n;
    addr += n;
    len -= n;
  }
  return BK_OK;
}

// ---------------------------------------------------------------- catch, link and speed

void bk_set_reset(void (*reset)()) { reset_fn = reset; }

bool bk_begin() {
  bk_end();
  if (!tuart_begin(BK_LINK_BAUD, 3 * BK_SECTOR, 0, "Beken download")) return false;
  on = true;
  linked = false;
  guard = BK_BOOT_END;
  wrap = 0x200000;
  blk_addr = 0xFFFFFFFF;
  return true;
}

void bk_end() {
  if (!on) return;
  on = linked = false;
  tuart_end();
}

bool bk_active() { return on; }
bool bk_linked() { return on && linked; }

// the 5-byte answer to the speed command comes after BK_BAUD_DELAY_MS at the new speed; then a link check
static int switch_baud(uint32_t baud) {
  uint8_t p[5], a[5];
  put_le32(p, baud);
  p[4] = BK_BAUD_DELAY_MS;
  send_short(0x0F, p, 5);
  tuart_wait_tx(50);
  delay(BK_BAUD_DELAY_MS / 2);
  tuart_set_baud(baud);
  rx_reset();
  int n = recv(0x0F, false, a, 5, 0, 300 + BK_BAUD_DELAY_MS);
  if (n != 5 || memcmp(a, p, 5)) fail(BK_ERR_ECHO, "no acknowledgement of %u baud", (unsigned)baud);
  return link_retry(3);
}

// after a failed speed change the target may be at either speed: back to 115200, through the target's own speed
// command if it still answers at the new one
static int back_to_link_baud() {
  if (link_retry(2) == BK_OK) {
    uint8_t p[5];
    put_le32(p, BK_LINK_BAUD);
    p[4] = BK_BAUD_DELAY_MS;
    send_short(0x0F, p, 5);
    tuart_wait_tx(50);
    delay(BK_BAUD_DELAY_MS / 2);
  }
  tuart_set_baud(BK_LINK_BAUD);
  delay(BK_BAUD_DELAY_MS);
  return link_retry(3);
}

// Catch the target at 115200 and find out which protocol it speaks.
static int catch_link(uint32_t catch_ms, BkLink *res) {
  static const uint8_t lc[5] = {0x01, 0xE0, 0xFC, 0x01, 0x00};
  static const uint8_t ans[8] = {0x04, 0x0E, 0x05, 0x01, 0xE0, 0xFC, 0x01, 0x00};
  linked = false;
  blk_addr = 0xFFFFFFFF;
  tuart_set_baud(BK_LINK_BAUD);

  // The BootROM listens for a link check only for a moment after reset: reset, then send link checks back to
  // back until the answer comes, and reset again after BK_CYCLE_MS. TX is an input while the target is off.
  char seen[20];
  int nseen = 0, match = 0;
  bool caught = false;
  uint32_t t0 = millis(), cycle = t0, on_us = micros();
  bool first = true;
  while (!caught && (int32_t)(millis() - t0) < (int32_t)catch_ms) {
    if (reset_fn && (first || millis() - cycle >= BK_CYCLE_MS)) {
      tuart_release_tx();
      tuart_event("target reset (catch)");
      reset_fn();
      tuart_attach_tx();
      tuart_flush_input();
      res->resets++;
      cycle = millis();
      on_us = micros();
      match = 0;
    }
    first = false;
    put(lc, sizeof(lc));
    tuart_wait_tx(5);
    size_t avail = tuart_available();
    while (avail && !caught) {
      uint8_t b[64];
      int got = tuart_read(b, (int)min(avail, sizeof(b)), 0);
      if (got <= 0) break;
      avail -= got;
      for (int i = 0; i < got && !caught; i++) {
        match = b[i] == ans[match] ? match + 1 : (b[i] == ans[0] ? 1 : 0);
        if (match == (int)sizeof(ans)) {
          caught = true;
        } else if (!match) {
          res->rx_before++;
          if (nseen < (int)sizeof(seen) - 1) seen[nseen++] = b[i] >= 0x20 && b[i] < 0x7f ? (char)b[i] : '.';
        }
      }
    }
  }
  seen[nseen] = 0;
  if (!caught) {
    float s = (millis() - t0) / 1000.0f;
    if (!res->rx_before)
      return fail(BK_ERR_TIMEOUT,
                  "no answer in %.1f s (%u resets), and nothing at all came back: check TX1 -> SWS wire, "
                  "RX1 <- GPIO 21, GND and the power of the target",
                  s, (unsigned)res->resets);
    return fail(BK_ERR_TIMEOUT,
                "no answer in %.1f s (%u resets), but the target sent %u bytes (\"%s\"): its own firmware runs. "
                "Is CEN on the reset wire or 3V3 on the power pins? Another chip on UART1?",
                s, (unsigned)res->resets, (unsigned)res->rx_before, seen);
  }
  res->answer_ms = (micros() - on_us) / 1000;
  tuart_wait_tx(50);
  delay(10);
  if (link_retry(3) != BK_OK) return fail(BK_ERR_TIMEOUT, "the target answered once, then no more");
  linked = true;

  // Which protocol: read the first block of the app, then ask for the CRC of 256 bytes of it. The BootROM's
  // range includes the end address (257 bytes), the BK7231T bootloader's does not.
  rom = false;                                  // no status / echo checks until the protocol is known
  int r = read4k(BK_PROBE_ADDR, aux);
  uint32_t raw = 0;
  if (r == BK_OK) r = crc_cmd(BK_PROBE_ADDR, BK_PROBE_ADDR + 256, &raw);
  if (r != BK_OK) {
    linked = false;
    return r;
  }
  raw ^= 0xFFFFFFFFu;
  if (raw == crc32z(0, aux, 257)) {
    rom = true;
  } else if (raw == crc32z(0, aux, 256)) {
    rom = false;
    link_retry(3);                              // lifts the write protection the CRC left behind
  } else {
    linked = false;
    return fail(BK_ERR_CRC, "the CRC of the probe block fits neither protocol (bad wiring, noise at 115200?)");
  }
  wrap = 0x200000;
  uint8_t id[3];
  if (rom && flash_id(id) == BK_OK && id[2] >= 17 && id[2] <= 25) wrap = 1u << id[2];
  res->rom = rom;
  res->baud = BK_LINK_BAUD;
  return BK_OK;
}

int bk_link(uint32_t catch_ms, uint32_t baud, BkLink *res) {
  memset(res, 0, sizeof(*res));
  detail[0] = 0;
  if (!on) return BK_ERR_NOLINK;
  int r = catch_link(catch_ms, res);
  if (r != BK_OK || baud == BK_LINK_BAUD) return r;

  // the speed: the change, a link check, and a 4 k read that must equal the one at 115200
  static uint8_t again[BK_SECTOR];
  r = switch_baud(baud);
  if (r == BK_OK) r = read4k(BK_PROBE_ADDR, again);
  if (r == BK_OK && memcmp(again, aux, BK_SECTOR)) r = fail(BK_ERR_CRC, "the test read differs at %u baud", (unsigned)baud);
  if (r == BK_OK) {
    res->baud = baud;
    blk_addr = 0xFFFFFFFF;
    return BK_OK;
  }
  char why[sizeof(detail)];
  snprintf(why, sizeof(why), "%u baud failed (%.150s)", (unsigned)baud, detail[0] ? detail : bk_err_text(r));
  res->fallback = true;
  if (back_to_link_baud() != BK_OK) {           // out of reach at either speed: catch it again at 115200
    uint32_t resets = res->resets;
    r = catch_link(catch_ms, res);
    res->resets += resets;
    res->fallback = true;
    if (r != BK_OK) return r;
  }
  snprintf(detail, sizeof(detail), "%s", why);
  blk_addr = 0xFFFFFFFF;
  return BK_OK;
}

// ---------------------------------------------------------------- text commands

static const char *const err_names[] = {"ok", "timeout", "status", "echo", "crc", "bootloader area",
                                        "alignment", "not linked (BKL first)", "not in this protocol", "size"};

const char *bk_detail() { return detail; }

const char *bk_err_text(int r) {
  static char t[sizeof(detail) + 40];
  const char *name = r >= 0 && r < (int)(sizeof(err_names) / sizeof(err_names[0])) ? err_names[r] : "error";
  if (detail[0] && r != BK_ERR_NOLINK) snprintf(t, sizeof(t), "%s: %s", name, detail);
  else snprintf(t, sizeof(t), "%s", name);
  return t;
}

static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  c |= 0x20;
  return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

static int unhex(const char *s, uint8_t *out, int max) {
  int n = 0;
  for (; s[0] && s[1]; s += 2) {
    int a = hexv(s[0]), b = hexv(s[1]);
    if (a < 0 || b < 0 || n >= max) return -1;
    out[n++] = (uint8_t)(a << 4 | b);
  }
  return s[0] ? -1 : n;
}

bool bk_command(int argc, char **argv, void (*out)(const char *line)) {
  static char o[2 * 512 + 16];
  const char *cmd = argv[0];
  uint32_t a1 = argc > 1 ? strtoul(argv[1], NULL, 0) : 0;
  uint32_t a2 = argc > 2 ? strtoul(argv[2], NULL, 0) : 0;
  int r = BK_OK;
  detail[0] = 0;
#define BK_FAIL(rr)                                  \
  do {                                               \
    snprintf(o, sizeof(o), "ERR %s", bk_err_text(rr)); \
    out(o);                                          \
    return true;                                     \
  } while (0)

  if (!strcmp(cmd, "BK")) {                     // BK: UART to the BK72xx download port; BK 0: leave
    if (argc > 1 && a1 == 0) {
      bk_end();
      out("OK");
    } else {
      if (!bk_begin()) {
        out("ERR UART driver");
        return true;
      }
      out("OK");
    }
    return true;
  }
  if (strncmp(cmd, "BK", 2)) return false;
  if (!on) {
    out("ERR not in BK mode (send BK first)");
    return true;
  }
  if (!strcmp(cmd, "BKL")) {                    // BKL <catch ms> <baud>: catch, link, protocol, speed
    BkLink l;
    uint32_t baud = argc > 2 ? a2 : 921600;
    if (baud < 9600 || baud > 3000000) {
      out("ERR baud 9600 .. 3000000");
      return true;
    }
    r = bk_link(argc > 1 ? a1 : 10000, baud, &l);
    if (r) BK_FAIL(r);
    snprintf(o, sizeof(o), "OK %s %u %u %u %d %u%s%s", l.rom ? "rom" : "bl", (unsigned)l.baud, (unsigned)l.resets,
             (unsigned)l.answer_ms, l.fallback ? 1 : 0, (unsigned)l.rx_before, l.fallback ? " " : "",
             l.fallback ? detail : "");
    out(o);
  } else if (!linked) {
    BK_FAIL(BK_ERR_NOLINK);
  } else if (!strcmp(cmd, "BKI")) {             // BKI: OK <chip id> <flash id> (BootROM; 0 000000 otherwise)
    uint32_t chip = 0;
    uint8_t id[3] = {0, 0, 0};
    if (rom) {
      if ((r = read_reg(0x800000, &chip)) != BK_OK) BK_FAIL(r);   // SCTRL_CHIP_ID
      if ((r = flash_id(id)) != BK_OK) BK_FAIL(r);
    }
    snprintf(o, sizeof(o), "OK %x %02x%02x%02x", (unsigned)chip, id[0], id[1], id[2]);
    out(o);
  } else if (!strcmp(cmd, "BKRR")) {            // BKRR <addr>: read a register (BootROM)
    uint32_t v = 0;
    if ((r = read_reg(a1, &v)) != BK_OK) BK_FAIL(r);
    snprintf(o, sizeof(o), "OK %08x", (unsigned)v);
    out(o);
  } else if (!strcmp(cmd, "BKSR")) {            // BKSR <opcode>: read a flash status register (0x05, 0x35, 0x15)
    uint8_t v = 0;
    if ((r = sr_read((uint8_t)a1, &v)) != BK_OK) BK_FAIL(r);
    snprintf(o, sizeof(o), "OK %02x", v);
    out(o);
  } else if (!strcmp(cmd, "BKSW")) {            // BKSW <opcode> <hex, 1 or 2 bytes>: write it
    uint8_t v[2];
    int n = argc > 2 ? unhex(argv[2], v, 2) : -1;
    if (n < 1) BK_FAIL(BK_ERR_SIZE);
    if ((r = sr_write((uint8_t)a1, v, n)) != BK_OK) BK_FAIL(r);
    out("OK");
  } else if (!strcmp(cmd, "BKC")) {             // BKC <addr> <len>: CRC-32 (zlib) of the range, by the target
    uint32_t c = 0;
    if ((r = bk_crc(a1, a2, &c)) != BK_OK) BK_FAIL(r);
    snprintf(o, sizeof(o), "OK %08x", (unsigned)c);
    out(o);
  } else if (!strcmp(cmd, "BKE")) {             // BKE <addr> [64]: erase the 4 k sector / 64 k block
    if ((r = bk_erase(a1, argc > 2 && a2 == 64)) != BK_OK) BK_FAIL(r);
    out("OK");
  } else if (!strcmp(cmd, "BKR")) {             // BKR <addr> <len>: read up to 512 bytes, hex
    static uint8_t d[512];
    if (a2 < 1 || a2 > sizeof(d)) BK_FAIL(BK_ERR_SIZE);
    if ((r = bk_read(a1, d, (int)a2)) != BK_OK) BK_FAIL(r);
    int k = snprintf(o, sizeof(o), "OK ");
    for (uint32_t i = 0; i < a2; i++) k += snprintf(o + k, sizeof(o) - k, "%02x", d[i]);
    out(o);
  } else if (!strcmp(cmd, "BKB")) {             // BKB <offset> <hex>: fill the 4 k staging buffer for BKW
    uint32_t off = min(a1, (uint32_t)BK_SECTOR);
    int n = argc > 2 ? unhex(argv[2], aux + off, (int)(BK_SECTOR - off)) : -1;
    if (n < 1) BK_FAIL(BK_ERR_SIZE);
    out("OK");
  } else if (!strcmp(cmd, "BKW")) {             // BKW <addr>: write the staging buffer to the erased sector
    if ((r = bk_write4k(a1, aux)) != BK_OK) BK_FAIL(r);
    out("OK");
  } else if (!strcmp(cmd, "BKG")) {             // BKG [first writable address [flash size]]: the bootloader guard
    if (argc > 1) guard = a1;
    if (argc > 2 && a2 >= 0x10000) wrap = a2;
    snprintf(o, sizeof(o), "OK %x %x", (unsigned)guard, (unsigned)wrap);
    out(o);
  } else if (!strcmp(cmd, "BKX")) {             // BKX: reboot into the application
    reboot();
    out("OK");
  } else {
    out("ERR unknown command");
  }
  return true;
#undef BK_FAIL
}
