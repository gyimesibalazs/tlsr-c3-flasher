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

static void jtag_pins_off();
void swd_begin() {
  jtag_pins_off();
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
  jtag_pins_off();
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

static bool use_jtag = false;               // memory / core access goes through a JTAG-DP (JTAG or cJTAG)
static int jtag_xfer(bool ap, bool read, uint8_t addr, uint32_t *data);

static int xfer(bool ap, bool read, uint8_t addr, uint32_t *data) {
  if (use_jtag) return jtag_xfer(ap, read, addr, data);
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
  use_jtag = false;
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
  use_jtag = false;
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
  if (use_jtag) {                            // the JTAG-DP returns every read itself: no pipelining here
    for (int i = 0; i < words; i++) {
      uint32_t a = addr + 4 * i, v;
      if (i == 0 || (a & 0x3ff) == 0) {
        r = swd_ap_write(ap, 0x04, a);
        if (r != SWD_OK) return r;
      }
      r = swd_ap_read(ap, 0x0c, &v);
      if (r != SWD_OK) return r;
      memcpy(out + 4 * i, &v, 4);
    }
    return SWD_OK;
  }
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

// ================================================================ JTAG: four wires or cJTAG (OScan1), JTAG-DP
// Four-wire JTAG: TCK on the SWCLK wire (idles low), TMS on PIN_JTAG_TMS, TDI on the SWS wire (GPIO 4 drives
// it through the resistor), TDO on PIN_JTAG_TDO. The target samples TMS / TDI on the rising edge of TCK and
// changes TDO on the falling edge; the host samples TDO just before the rising edge.
// cJTAG: activation as in OpenOCD's ftdi.c (cjtag_reset_online_activate); the TI ICEPick-C router sequence as
// in OpenOCD's tcl/target/ti/icepick.cfg (icepick_c_tapenable, port 0) used by its cc26x0 / cc26x2 configurations.

enum JState { J_RTI, J_PAUSE };
static JState jstate = J_RTI;
static int cur_ir = -1;                      // DAP instruction loaded (-1: unknown)
static const int ICEPICK_IR_LEN = 6, DAP_IR_LEN = 4;
static const uint8_t IR_ABORT = 0x8, IR_DPACC = 0xA, IR_APACC = 0xB, IR_IDCODE = 0xE;
static bool four_wire = false;
// position of the DAP in the chain: devices between it and TDO ("pre", their bits are shifted first) and
// between it and TDI ("post"), as instruction bits (all kept in BYPASS) and as data bits (one per device)
static int pre_ir = 0, post_ir = 0, pre_dr = 0, post_dr = 0;

static const uint32_t B_TMS = 1u << PIN_JTAG_TMS;
static inline uint32_t tdo_get() { return (REG_READ(GPIO_IN_REG) >> PIN_JTAG_TDO) & 1; }

static uint32_t jtag4(uint32_t tdi, uint32_t tms) {
  dio_set(tdi);
  REG_WRITE(tms ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, B_TMS);
  dly();
  uint32_t tdo = tdo_get();
  clk_hi();
  dly();
  clk_lo();
  return tdo;
}

static void jtag_pins_off() {
  four_wire = false;
  pinMode(PIN_JTAG_TMS, INPUT);
  pinMode(PIN_JTAG_TDO, INPUT);
}

static uint32_t oscan1(uint32_t tdi, uint32_t tms);
static inline uint32_t jbit(uint32_t tdi, uint32_t tms) { return four_wire ? jtag4(tdi, tms) : oscan1(tdi, tms); }

// one OScan1 bit cell: the inverted TDI, then TMS, then TDO driven by the target
static uint32_t oscan1(uint32_t tdi, uint32_t tms) {
  dio_drive();
  write_bit(!tdi);
  write_bit(tms);
  dio_release();
  return read_bit();                         // the next cell drives TMSC again before its falling edge
}

// TMSC edges while TCKC is high: 8 = reset, 6 = select (IEEE 1149.7 escape sequences)
static void escape(int edges) {
  dio_drive();
  dio_set(0);
  clk_lo();
  dly();
  clk_hi();
  dly();
  for (int i = 0; i < edges; i++) {
    dio_set(!(i & 1));
    dly();
    dly();
  }
  dio_set(0);
  clk_lo();
  dly();
}

static void goto_shift(bool ir) {
  if (jstate == J_PAUSE) {                   // Pause -> Exit2 -> Update
    jbit(0, 1);
    jbit(0, 1);
  }
  jbit(0, 1);                              // Select-DR
  if (ir) jbit(0, 1);                      // Select-IR
  jbit(0, 0);                              // Capture
  jbit(0, 0);                              // Shift
}

// shift n bits (LSB first); leaves Shift through Exit1 into Pause or (Update) Run-Test/Idle
static uint64_t shift(uint64_t out, int n, bool to_pause) {
  uint64_t in = 0;
  for (int i = 0; i < n; i++) in |= (uint64_t)jbit((out >> i) & 1, i == n - 1) << i;
  if (to_pause) {
    jbit(0, 0);                            // Exit1 -> Pause
    jstate = J_PAUSE;
  } else {
    jbit(0, 1);                            // Exit1 -> Update
    jbit(0, 0);                            // -> Run-Test/Idle
    jstate = J_RTI;
  }
  return in;
}

static void run_idle(int n) {
  if (jstate == J_PAUSE) {                   // Pause -> Exit2 -> Update -> Run-Test/Idle
    jbit(0, 1);
    jbit(0, 1);
    jbit(0, 0);
    jstate = J_RTI;
  }
  for (int i = 0; i < n; i++) jbit(0, 0);
}

// a scan of the router alone (before the DAP is in the chain)
static uint64_t jrc_ir(uint8_t ir, bool to_pause) {
  goto_shift(true);
  return shift(ir, ICEPICK_IR_LEN, to_pause);
}
static uint64_t jrc_dr(uint64_t v, int n, bool to_pause) {
  goto_shift(false);
  return shift(v, n, to_pause);
}
static void icepick_router(uint32_t rw, uint32_t block, uint32_t reg, uint32_t payload) {
  jrc_ir(0x02, true);                        // ROUTER
  jrc_dr(((rw & 1) << 31) | ((block & 7) << 28) | ((reg & 0xf) << 24) | (payload & 0xffffff), 32, true);
}

static uint64_t ones(int n) { return n >= 64 ? ~0ull : (1ull << n) - 1; }

// the DAP's instruction, every other device in BYPASS (all ones)
static void dap_ir(uint8_t ir) {
  if (cur_ir == ir) return;
  goto_shift(true);
  uint64_t v = ones(pre_ir) | ((uint64_t)ir << pre_ir) | (ones(post_ir) << (pre_ir + DAP_IR_LEN));
  shift(v, pre_ir + DAP_IR_LEN + post_ir, false);
  cur_ir = ir;
}

static int dp_scan(uint8_t ir, uint8_t addr, bool read, uint32_t wdata, uint32_t *prev) {
  dap_ir(ir);
  uint64_t out = (read ? 1 : 0) | (((uint64_t)(addr >> 2) & 3) << 1) | ((uint64_t)wdata << 3);
  goto_shift(false);
  uint64_t in = shift(out << pre_dr, pre_dr + 35 + post_dr, false) >> pre_dr;   // + one bypass bit per device
  run_idle(2);
  if (prev) *prev = (uint32_t)(in >> 3);
  uint32_t ack = in & 7;
  return ack == 2 ? SWD_OK : ack == 1 ? SWD_WAIT : SWD_NOACK;
}

static int dp_scan_retry(uint8_t ir, uint8_t addr, bool read, uint32_t wdata, uint32_t *prev) {
  int r = SWD_WAIT;
  for (int i = 0; i < WAIT_RETRIES && r == SWD_WAIT; i++) r = dp_scan(ir, addr, read, wdata, prev);
  return r;
}

// SWD semantics on top of the JTAG-DP: an AP read is posted (it returns the result of the scan before,
// exactly as SWD does, and DP RDBUFF gives its own result); a DP read returns its own value (one more scan)
static int jtag_xfer(bool ap, bool read, uint8_t addr, uint32_t *data) {
  if (!ap && addr == 0x0) {
    if (!read) {
      // the SWD sticky-clear bits of ABORT are reserved on JTAG: there CTRL/STAT clears them (write 1)
      if (*data & 0x1e) {
        int r = dp_scan_retry(IR_DPACC, 0x4, false, 0x50000032, NULL);
        if (r != SWD_OK) return r;
      }
      if (*data & 1) dp_scan(IR_ABORT, 0x0, false, 1, NULL);
      return SWD_OK;
    }
    dap_ir(IR_IDCODE);
    goto_shift(false);
    *data = (uint32_t)(shift(0, pre_dr + 32 + post_dr, false) >> pre_dr);
    return SWD_OK;
  }
  uint32_t prev = 0;
  int r = dp_scan_retry(ap ? IR_APACC : IR_DPACC, addr, read, read ? 0 : *data, &prev);
  if (r != SWD_OK || !read) return r;
  if (ap) {
    *data = prev;
    return SWD_OK;
  }
  r = dp_scan_retry(IR_DPACC, 0xC, true, 0, &prev);   // RDBUFF: the result of the read above
  if (r == SWD_OK) *data = prev;
  return r;
}

int ti_cjtag_connect(uint32_t *icepick_id, uint32_t *dap_id) {
  use_jtag = false;
  four_wire = false;
  sel_ap = sel_bank = -1;
  cur_ir = -1;
  // with the DAP enabled the chain is TDI -> ICEPick -> DAP -> TDO: the DAP gets the first bits
  pre_ir = pre_dr = 0;
  post_ir = ICEPICK_IR_LEN;
  post_dr = 1;
  // TAP reset escape, three clocks, selection escape, then OAC 1100 (OScan1), EC 1000, CP 0000 (LSB first)
  escape(8);
  dio_drive();
  for (int i = 0; i < 3; i++) write_bit(0);
  escape(6);
  static const uint8_t act[12] = {0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0};
  for (uint8_t b : act) write_bit(b);
  // OScan1 from here: Test-Logic-Reset, then Run-Test/Idle
  for (int i = 0; i < 6; i++) jbit(0, 1);
  jbit(0, 0);
  jstate = J_RTI;
  jrc_ir(0x04, false);                       // ICEPick IDCODE
  *icepick_id = (uint32_t)jrc_dr(0, 32, false);
  if (!(*icepick_id & 1) || *icepick_id == 0xffffffff) return SWD_NOACK;
  // icepick_c_tapenable, port 0 (the Cortex-M debug TAP)
  jrc_ir(0x07, true);                        // CONNECT
  jrc_dr(0x89, 8, true);
  icepick_router(1, 0x0, 0x1, 0x001000);
  icepick_router(1, 0x2, 0x0, 0x110048);
  icepick_router(1, 0x2, 0x0, 0x112048);
  icepick_router(1, 0x2, 0x0, 0x112148);
  jrc_ir(0x00, false);
  run_idle(10);
  use_jtag = true;
  cur_ir = -1;
  return jtag_xfer(false, true, 0x0, dap_id);
}

// ---------------------------------------------------------------- four-wire JTAG
void jtag_begin() {
  swd_begin();
  four_wire = true;
  use_jtag = false;
  pinMode(PIN_JTAG_TDO, INPUT_PULLUP);
  pinMode(PIN_JTAG_TMS, OUTPUT);
  REG_WRITE(GPIO_OUT_W1TS_REG, B_TMS);       // TMS high: the TAPs stay in Test-Logic-Reset
  dio_set(1);
  clk_lo();
}

// SWJ-DP parts that were left in SWD mode: the SWD-to-JTAG sequence on TMS (= SWDIO), then Test-Logic-Reset
// and Run-Test/Idle. On a plain JTAG chain this is only a walk through the TAP states ending in reset.
void jtag_reset() {
  for (int i = 0; i < 50; i++) jbit(1, 1);
  for (int i = 0; i < 16; i++) jbit(1, (0xE73C >> i) & 1);
  for (int i = 0; i < 6; i++) jbit(1, 1);
  jbit(1, 0);
  jstate = J_RTI;
  cur_ir = -1;
}

// TDO side first: ids[0] is the device next to TDO. A device without IDCODE shows its BYPASS bit (id 0).
int jtag_scan(uint32_t *ids, int max, int *count, int *ir_total) {
  jtag_reset();
  goto_shift(false);
  int n = 0;
  bool end = false;
  while (n < max) {
    if (!jbit(1, 0)) {
      ids[n++] = 0;                          // BYPASS (or TDO stuck low)
      continue;
    }
    uint32_t id = 1;
    for (int i = 1; i < 32; i++) id |= jbit(1, 0) << i;
    if (id == 0xffffffff) {                  // our own ones came through: end of the chain
      end = true;
      break;
    }
    ids[n++] = id;
  }
  jbit(1, 1);                                // Exit1
  jbit(0, 1);                                // Update
  jbit(0, 0);                                // Run-Test/Idle
  jstate = J_RTI;
  // total instruction length: flush the IR with zeros, then count the ones it takes until one comes out
  goto_shift(true);
  for (int i = 0; i < 512; i++) jbit(0, 0);
  int len = 0;
  while (len < 512 && !jbit(1, 0)) len++;
  jbit(1, 1);
  jbit(1, 1);
  jbit(0, 0);
  jstate = J_RTI;
  cur_ir = -1;
  *count = n;
  *ir_total = len;
  return end && len > 0 && len < 512 ? SWD_OK : SWD_NOACK;
}

void jtag_select(int pre_ir_bits, int post_ir_bits, int pre_devices, int post_devices) {
  pre_ir = pre_ir_bits;
  post_ir = post_ir_bits;
  pre_dr = pre_devices;
  post_dr = post_devices;
  cur_ir = -1;
}

int jtag_dp_connect(uint32_t *idcode) {
  if (pre_ir + DAP_IR_LEN + post_ir > 64 || pre_dr + 35 + post_dr > 64) return SWD_NOACK;
  sel_ap = sel_bank = -1;
  use_jtag = false;
  jtag_reset();
  use_jtag = true;
  int r = jtag_xfer(false, true, 0x0, idcode);
  if (r == SWD_OK && (!(*idcode & 1) || *idcode == 0xffffffff)) r = SWD_NOACK;
  return r;
}

// raw scans, n bits LSB first from out[], captured bits into in[]; ends in Run-Test/Idle
static void raw_scan(bool ir, const uint8_t *out, uint8_t *in, int n) {
  goto_shift(ir);
  for (int i = 0; i < n; i++) {
    uint32_t b = jbit((out[i / 8] >> (i % 8)) & 1, i == n - 1);
    if (b) in[i / 8] |= 1 << (i % 8);
    else in[i / 8] &= ~(1 << (i % 8));
  }
  jbit(0, 1);                                // Exit1 -> Update
  jbit(0, 0);                                // Run-Test/Idle
  jstate = J_RTI;
  if (ir) cur_ir = -1;
}
void jtag_ir_scan(const uint8_t *out, uint8_t *in, int n) { raw_scan(true, out, in, n); }
void jtag_dr_scan(const uint8_t *out, uint8_t *in, int n) { raw_scan(false, out, in, n); }

const char *swd_ack_name(int ack) {
  switch (ack) {
    case SWD_OK: return "ok";
    case SWD_WAIT: return "wait (target busy)";
    case SWD_FAULT: return "fault";
    case SWD_PARITY: return "parity error";
    default: return "no answer";
  }
}
