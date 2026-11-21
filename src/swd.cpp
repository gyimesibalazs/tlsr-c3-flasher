// Arm Serial Wire Debug, bit-banged on the C3.
//
// SWDIO is the flasher's SWS wire: GPIO PIN_SWS_TX drives it through the 1 k resistor while the host has the
// line, and is switched to an input while the target drives it; GPIO PIN_SWS_RX reads it all the time.
// Timing as in CMSIS-DAP: the host changes SWDIO while SWCLK is low, the target samples on the rising edge;
// the host samples the target's bits just before the rising edge.
#include <Arduino.h>
#include "esp_cpu.h"
#include "soc/gpio_reg.h"
#include "config.h"
#include "swd.h"

static uint32_t half_cycles = 80;           // CPU cycles per half clock period
static uint32_t khz = SWD_KHZ_DEFAULT;
static int sel_ap = -1, sel_bank = -1;       // cached DP SELECT
static const int WAIT_RETRIES = 200;

static const uint32_t B_CLK = 1u << PIN_SWCLK, B_DRV = 1u << PIN_SWS_TX;

static inline void dly() {
  uint32_t t = esp_cpu_get_cycle_count();
  while ((uint32_t)(esp_cpu_get_cycle_count() - t) < half_cycles) {
  }
}
static inline void clk_lo() { REG_WRITE(GPIO_OUT_W1TC_REG, B_CLK); }
static inline void clk_hi() { REG_WRITE(GPIO_OUT_W1TS_REG, B_CLK); }
static inline void dio_drive() { REG_WRITE(GPIO_ENABLE_W1TS_REG, B_DRV); }
static inline void dio_release() { REG_WRITE(GPIO_ENABLE_W1TC_REG, B_DRV); }
static inline void dio_set(uint32_t b) { REG_WRITE(b ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, B_DRV); }
static inline uint32_t dio_get() { return (REG_READ(GPIO_IN_REG) >> PIN_SWS_RX) & 1; }

static inline void write_bit(uint32_t b) {
  dio_set(b);
  clk_lo();
  dly();
  clk_hi();
  dly();
}
static inline uint32_t read_bit() {
  clk_lo();
  dly();
  uint32_t b = dio_get();
  clk_hi();
  dly();
  return b;
}
static inline void cycle() {
  clk_lo();
  dly();
  clk_hi();
  dly();
}
static void write_bits(uint32_t v, int n) {
  for (int i = 0; i < n; i++) write_bit((v >> i) & 1);
}

void swd_set_khz(uint32_t k) {
  khz = k < 10 ? 10 : (k > 4000 ? 4000 : k);
  // a clock period is two delays plus the register writes (about 10 cycles), at 160 MHz
  uint32_t c = 160000 / khz / 2;
  half_cycles = c > 6 ? c - 5 : 1;
}
uint32_t swd_khz() { return khz; }

void swd_begin() {
  pinMode(PIN_SWS_RX, INPUT_PULLUP);         // SWDIO idles high
  pinMode(PIN_SWS_TX, OUTPUT);
  pinMode(PIN_SWCLK, OUTPUT);
  dio_set(1);
  clk_hi();
  dio_drive();
  sel_ap = sel_bank = -1;
  swd_set_khz(khz);
}

void swd_end() {
  pinMode(PIN_SWCLK, INPUT);
  pinMode(PIN_SWS_TX, INPUT);
  sel_ap = sel_bank = -1;
}

static void line_reset() {
  dio_drive();
  for (int i = 0; i < 56; i++) write_bit(1);
}
static void idle(int n) {
  dio_drive();
  for (int i = 0; i < n; i++) write_bit(0);
}

// one transfer; request: APnDP, RnW, A[3:2]
static int transfer(bool ap, bool read, uint8_t addr, uint32_t *data) {
  uint32_t a2 = (addr >> 2) & 1, a3 = (addr >> 3) & 1;
  uint32_t par = (ap + read + a2 + a3) & 1;
  uint32_t req = 1 | (ap << 1) | (read << 2) | (a2 << 3) | (a3 << 4) | (par << 5) | (0 << 6) | (1 << 7);
  dio_drive();
  write_bits(req, 8);
  dio_release();
  cycle();                                   // turnaround
  uint32_t ack = read_bit() | (read_bit() << 1) | (read_bit() << 2);
  if (ack == SWD_OK) {
    if (read) {
      uint32_t v = 0, p = 0;
      for (int i = 0; i < 32; i++) {
        uint32_t b = read_bit();
        v |= b << i;
        p ^= b;
      }
      uint32_t pb = read_bit();
      cycle();                               // turnaround
      dio_drive();
      idle(2);
      if (pb != p) return SWD_PARITY;
      if (data) *data = v;
    } else {
      cycle();                               // turnaround
      dio_drive();
      uint32_t v = *data, p = 0;
      for (int i = 0; i < 32; i++) {
        write_bit((v >> i) & 1);
        p ^= (v >> i) & 1;
      }
      write_bit(p);
      idle(2);
    }
    return SWD_OK;
  }
  if (ack == SWD_WAIT || ack == SWD_FAULT) {
    cycle();
    dio_drive();
    idle(2);
    return (int)ack;
  }
  // no (valid) answer: clock past a possible data phase, then take the line back
  for (int i = 0; i < 33; i++) cycle();
  cycle();
  dio_drive();
  idle(2);
  return SWD_NOACK;
}

static int xfer(bool ap, bool read, uint8_t addr, uint32_t *data) {
  int r = SWD_WAIT;
  for (int i = 0; i < WAIT_RETRIES && r == SWD_WAIT; i++) r = transfer(ap, read, addr, data);
  if (r == SWD_FAULT) {
    uint32_t clr = 0x1e;                     // ABORT: clear the sticky error flags
    transfer(false, false, 0x0, &clr);
  }
  return r;
}

int swd_dp_read(uint8_t addr, uint32_t *v) { return xfer(false, true, addr, v); }
int swd_dp_write(uint8_t addr, uint32_t v) {
  if (addr == 0x8) sel_ap = sel_bank = -1;
  return xfer(false, false, addr, &v);
}

static int ap_select(uint8_t ap, uint8_t addr) {
  int bank = (addr >> 4) & 0xf;
  if (ap == sel_ap && bank == sel_bank) return SWD_OK;
  uint32_t v = ((uint32_t)ap << 24) | ((uint32_t)bank << 4);
  int r = xfer(false, false, 0x8, &v);
  if (r == SWD_OK) {
    sel_ap = ap;
    sel_bank = bank;
  }
  return r;
}

// AP reads are posted: the value arrives with the next read, here from DP RDBUFF
int swd_ap_read(uint8_t ap, uint8_t addr, uint32_t *v) {
  int r = ap_select(ap, addr);
  if (r != SWD_OK) return r;
  uint32_t dummy;
  r = xfer(true, true, addr & 0xc, &dummy);
  if (r != SWD_OK) return r;
  return xfer(false, true, 0xc, v);
}

int swd_ap_write(uint8_t ap, uint8_t addr, uint32_t v) {
  int r = ap_select(ap, addr);
  if (r != SWD_OK) return r;
  return xfer(true, false, addr & 0xc, &v);
}

int swd_connect(uint32_t *idcode) {
  sel_ap = sel_bank = -1;
  // SWJ-DP: line reset, JTAG-to-SWD switch, line reset, idle
  line_reset();
  write_bits(0xe79e, 16);
  line_reset();
  idle(4);
  if (xfer(false, true, 0x0, idcode) == SWD_OK) return SWD_OK;
  // dormant state (SW-DP v2): selection alert sequence, then the SWD activation code
  dio_drive();
  for (int i = 0; i < 8; i++) write_bit(1);
  static const uint32_t alert[4] = {0x6209f392, 0x86852d95, 0xe3ddafe9, 0x19bc0ea2};
  for (uint32_t w : alert) write_bits(w, 32);
  idle(4);
  write_bits(0x1a, 8);
  line_reset();
  idle(4);
  return xfer(false, true, 0x0, idcode);
}

int swd_connect_multidrop(uint32_t targetsel, uint32_t *idcode) {
  sel_ap = sel_bank = -1;
  dio_drive();
  for (int i = 0; i < 8; i++) write_bit(1);
  static const uint32_t alert[4] = {0x6209f392, 0x86852d95, 0xe3ddafe9, 0x19bc0ea2};
  for (uint32_t w : alert) write_bits(w, 32);
  idle(4);
  write_bits(0x1a, 8);                       // SWD activation code
  line_reset();
  idle(4);
  // TARGETSEL (DP write 0xC): no target answers, so the ACK phase is just clocked through
  uint32_t par = 0;
  for (int i = 0; i < 32; i++) par ^= (targetsel >> i) & 1;
  write_bits(0x99, 8);
  dio_release();
  for (int i = 0; i < 5; i++) cycle();       // turnaround, 3 ACK bits, turnaround
  dio_drive();
  write_bits(targetsel, 32);
  write_bit(par);
  idle(2);
  return xfer(false, true, 0x0, idcode);
}

int swd_power_up() {
  int r = swd_dp_write(0x0, 0x1e);           // ABORT: clear sticky errors
  if (r != SWD_OK) return r;
  r = swd_dp_write(0x4, 0x50000000);         // CTRL/STAT: CSYSPWRUPREQ | CDBGPWRUPREQ
  if (r != SWD_OK) return r;
  for (int i = 0; i < 100; i++) {
    uint32_t st = 0;
    r = swd_dp_read(0x4, &st);
    if (r != SWD_OK) return r;
    if ((st & 0xa0000000) == 0xa0000000) return SWD_OK;
    delay(1);
  }
  return SWD_WAIT;
}

// MEM-AP: CSW 32-bit, auto increment, debug master, privileged
static const uint32_t CSW_WORD_INC = 0xa2000012;

int swd_mem_read(uint8_t ap, uint32_t addr, uint8_t *out, int len) {
  int r = swd_ap_write(ap, 0x00, CSW_WORD_INC);
  if (r != SWD_OK) return r;
  int words = len / 4;
  int i = 0;
  while (i < words) {
    // the address auto-increment wraps at 1 kbyte boundaries: set TAR for every such block
    uint32_t a = addr + 4 * i;
    int n = min(words - i, (int)((0x400 - (a & 0x3ff)) / 4));
    r = swd_ap_write(ap, 0x04, a);
    if (r != SWD_OK) return r;
    r = ap_select(ap, 0x0c);
    if (r != SWD_OK) return r;
    uint32_t v;
    r = xfer(true, true, 0x0c, &v);          // posted: starts the first read
    if (r != SWD_OK) return r;
    for (int k = 0; k < n; k++) {
      // the last word comes from RDBUFF, so the AP does not read one word past the block
      r = k + 1 < n ? xfer(true, true, 0x0c, &v) : xfer(false, true, 0x0c, &v);
      if (r != SWD_OK) return r;
      memcpy(out + 4 * (i + k), &v, 4);
    }
    i += n;
  }
  return SWD_OK;
}

int swd_mem_write(uint8_t ap, uint32_t addr, const uint8_t *data, int len) {
  int r = swd_ap_write(ap, 0x00, CSW_WORD_INC);
  if (r != SWD_OK) return r;
  int words = len / 4;
  int i = 0;
  while (i < words) {
    uint32_t a = addr + 4 * i;
    int n = min(words - i, (int)((0x400 - (a & 0x3ff)) / 4));
    r = swd_ap_write(ap, 0x04, a);
    if (r != SWD_OK) return r;
    for (int k = 0; k < n; k++) {
      uint32_t v;
      memcpy(&v, data + 4 * (i + k), 4);
      r = swd_ap_write(ap, 0x0c, v);
      if (r != SWD_OK) return r;
    }
    i += n;
  }
  uint32_t st;
  return swd_dp_read(0xc, &st);              // RDBUFF: waits until the last write is done
}

int swd_mem_write16(uint8_t ap, uint32_t addr, uint16_t v) {
  int r = swd_ap_write(ap, 0x00, 0xa2000001);        // CSW: 16-bit, no increment
  if (r != SWD_OK) return r;
  r = swd_ap_write(ap, 0x04, addr);
  if (r != SWD_OK) return r;
  r = swd_ap_write(ap, 0x0c, (uint32_t)v << (8 * (addr & 2)));   // the half-word goes on its byte lanes
  if (r != SWD_OK) return r;
  uint32_t st;
  return swd_dp_read(0xc, &st);
}

static const uint32_t DHCSR = 0xE000EDF0, DCRSR = 0xE000EDF4, DCRDR = 0xE000EDF8;

static int reg_ready() {
  for (int i = 0; i < 100; i++) {
    uint32_t v = 0;
    int r = swd_mem_read(0, DHCSR, (uint8_t *)&v, 4);
    if (r != SWD_OK) return r;
    if (v & (1 << 16)) return SWD_OK;        // S_REGRDY
  }
  return SWD_WAIT;
}

int swd_core_reg_write(uint8_t reg, uint32_t v) {
  int r = swd_mem_write(0, DCRDR, (const uint8_t *)&v, 4);
  if (r != SWD_OK) return r;
  uint32_t sel = reg | (1 << 16);            // REGWnR
  r = swd_mem_write(0, DCRSR, (const uint8_t *)&sel, 4);
  if (r != SWD_OK) return r;
  return reg_ready();
}

int swd_core_reg_read(uint8_t reg, uint32_t *v) {
  uint32_t sel = reg;
  int r = swd_mem_write(0, DCRSR, (const uint8_t *)&sel, 4);
  if (r != SWD_OK) return r;
  r = reg_ready();
  if (r != SWD_OK) return r;
  return swd_mem_read(0, DCRDR, (uint8_t *)v, 4);
}

const char *swd_ack_name(int ack) {
  switch (ack) {
    case SWD_OK: return "ok";
    case SWD_WAIT: return "wait (target busy)";
    case SWD_FAULT: return "fault";
    case SWD_PARITY: return "parity error";
    default: return "no answer";
  }
}
