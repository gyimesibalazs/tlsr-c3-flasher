// GPIO-level simulation of a four-wire JTAG chain with an Arm JTAG-DP + MEM-AP, for swd.cpp's JTAG code:
// chain scan, DAP selection, memory writes / reads across 1 k boundaries, raw scans.
//   g++ -std=c++17 -I tests/jtag_sim/stub -I src -o /tmp/jtag_sim tests/jtag_sim/jtag_sim.cpp src/swd.cpp && /tmp/jtag_sim
#include <Arduino.h>
#include <soc/gpio_reg.h>
#include <deque>
#include <map>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include "config.h"
#include "swd.h"

static uint32_t out = 0, en = 0, cyc = 0;
uint32_t esp_cpu_get_cycle_count() { return cyc += 1000; }
void delay(uint32_t) {}
uint32_t millis() { static uint32_t t; return t += 1; }
void pinMode(int pin, int mode) { if (mode == OUTPUT || mode == OUTPUT_OPEN_DRAIN) en |= 1u << pin; else en &= ~(1u << pin); }

enum St { TLR, RTI, SELDR, CAPDR, SHDR, EX1DR, PADR, EX2DR, UPDR, SELIR, CAPIR, SHIR, EX1IR, PAIR, EX2IR, UPIR };
static St next(St s, int tms) {
  static const St t1[] = {TLR, SELDR, SELIR, EX1DR, EX1DR, UPDR, EX2DR, UPDR, SELDR, TLR, EX1IR, EX1IR, UPIR, EX2IR, UPIR, SELDR};
  static const St t0[] = {RTI, RTI, CAPDR, SHDR, SHDR, PADR, PADR, SHDR, RTI, CAPIR, SHIR, SHIR, PAIR, PAIR, SHIR, RTI};
  return tms ? t1[s] : t0[s];
}

struct Tap {
  int irlen; uint32_t idcode; bool dap;
  int instr = 0; std::deque<int> reg;
  // DAP state
  uint32_t ctrl = 0, select = 0, csw = 0, tar = 0, last = 0;
  std::map<uint32_t, uint32_t> *mem = nullptr;
  int idcode_ir() const { return dap ? 0xE : (1 << irlen) - 2; }
  int bypass_ir() const { return (1 << irlen) - 1; }
  void reset() { instr = idcode ? idcode_ir() : bypass_ir(); }
  void load(uint64_t v, int n) { reg.clear(); for (int i = 0; i < n; i++) reg.push_back((v >> i) & 1); }
  uint64_t value() const { uint64_t v = 0; for (size_t i = 0; i < reg.size(); i++) v |= (uint64_t)reg[i] << i; return v; }
  void capture_dr() {
    if (idcode && instr == idcode_ir()) return load(idcode, 32);
    if (dap && (instr == 0xA || instr == 0xB || instr == 0x8)) return load(2 | ((uint64_t)last << 3), 35);
    load(0, 1);
  }
  void update_dr() {
    if (!dap || (instr != 0xA && instr != 0xB)) return;
    uint64_t v = value();
    int rnw = v & 1, a = ((v >> 1) & 3) << 2; uint32_t data = (uint32_t)(v >> 3);
    if (instr == 0xA) {
      if (a == 0x4) { if (rnw) last = ctrl | ((ctrl & 0x50000000) << 1); else ctrl = data & ~0x32u; }
      else if (a == 0x8) { if (rnw) last = select; else select = data; }
      else if (a == 0xC) { /* RDBUFF: the result stays as it is */ }
      return;
    }
    uint32_t ap = (select & 0xF0) | a;
    if (ap == 0x00) { if (rnw) last = csw; else csw = data; }
    else if (ap == 0x04) { if (rnw) last = tar; else tar = data; }
    else if (ap == 0x0C) {
      if (rnw) last = (*mem)[tar]; else (*mem)[tar] = data;
      if (((csw >> 4) & 3) == 1) tar = (tar & ~0x3ffu) | ((tar + 4) & 0x3ff);   // wraps inside 1 k, like the real AP
    } else if (ap == 0xFC) { if (rnw) last = 0x24770011; }
  }
};

static std::vector<Tap> chain;             // [0] next to TDO
static St st = TLR;
static std::map<uint32_t, uint32_t> memory;

static int tms_line() { return (en >> PIN_JTAG_TMS & 1) ? (out >> PIN_JTAG_TMS & 1) : 1; }
static int tdi_line() { return (en >> PIN_SWS_TX & 1) ? (out >> PIN_SWS_TX & 1) : 1; }

static void rising() {
  int tms = tms_line(), tdi = tdi_line();
  if (st == CAPDR) for (auto &t : chain) t.capture_dr();
  if (st == CAPIR) for (auto &t : chain) t.load(1, t.irlen);
  if (st == SHDR || st == SHIR) {
    std::vector<int> lsb; for (auto &t : chain) lsb.push_back(t.reg.front());
    for (size_t k = 0; k < chain.size(); k++) {
      int in = k + 1 == chain.size() ? tdi : lsb[k + 1];
      chain[k].reg.pop_front(); chain[k].reg.push_back(in);
    }
  }
  St n = next(st, tms);
  if (n == UPDR) for (auto &t : chain) t.update_dr();
  if (n == UPIR) for (auto &t : chain) t.instr = (int)t.value();
  if (n == TLR) for (auto &t : chain) t.reset();
  st = n;
}

void sim_reg_write(int reg, uint32_t v) {
  uint32_t before = out;
  if (reg == GPIO_OUT_W1TS_REG) out |= v; else if (reg == GPIO_OUT_W1TC_REG) out &= ~v;
  else if (reg == GPIO_ENABLE_W1TS_REG) en |= v; else if (reg == GPIO_ENABLE_W1TC_REG) en &= ~v;
  if (!(before >> PIN_SWCLK & 1) && (out >> PIN_SWCLK & 1)) rising();
}
uint32_t sim_reg_read(int reg) {
  if (reg != GPIO_IN_REG) return 0;
  int tdo = (st == SHDR || st == SHIR) ? chain[0].reg.front() : 1;
  return (uint32_t)tdo << PIN_JTAG_TDO | (uint32_t)tdi_line() << PIN_SWS_RX;
}

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static void run(const char *name, std::vector<Tap> taps, int dap, int pre_ir, int post_ir) {
  chain = taps; st = TLR; memory.clear();
  for (auto &t : chain) { t.mem = &memory; t.reset(); }
  jtag_begin();
  uint32_t ids[32]; int n = 0, irt = 0;
  CHECK(jtag_scan(ids, 32, &n, &irt) == SWD_OK);
  int sum = 0; for (auto &t : chain) sum += t.irlen;
  CHECK(n == (int)chain.size()); CHECK(irt == sum);
  for (int k = 0; k < n && k < (int)chain.size(); k++) CHECK(ids[k] == chain[k].idcode);
  printf("%s: %d devices, IR %d:", name, n, irt); for (int k = 0; k < n; k++) printf(" %08x", ids[k]); printf("\n");
  jtag_select(pre_ir, post_ir, dap, (int)chain.size() - dap - 1);
  uint32_t id = 0;
  CHECK(jtag_dp_connect(&id) == SWD_OK); CHECK(id == chain[dap].idcode);
  CHECK(swd_power_up() == SWD_OK);
  uint8_t w[2048], r[2048];
  for (int i = 0; i < 2048; i++) w[i] = rand();
  CHECK(swd_mem_write(0, 0x200003c0, w, 2048) == SWD_OK);     // crosses 1 k boundaries
  memset(r, 0, sizeof r);
  CHECK(swd_mem_read(0, 0x200003c0, r, 2048) == SWD_OK);
  CHECK(!memcmp(w, r, 2048));
  uint32_t idr = 0;
  CHECK(swd_ap_read(0, 0xFC, &idr) == SWD_OK && idr == 0x24770011);
  // raw IR / DR scans: IDCODE of the first device through the whole chain
  uint8_t o[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, in[8];
  jtag_reset();
  memset(in, 0, 8);
  jtag_dr_scan(o, in, 32);
  CHECK((in[0] | in[1] << 8 | in[2] << 16 | (uint32_t)in[3] << 24) == (chain[0].idcode ? chain[0].idcode : 0xfffffffe));
}

int main() {
  Tap dap{4, 0x4BA00477, true}, bs{5, 0x16410041, false}, anon{3, 0, false}, other{7, 0x0000107f, false};
  run("STM32 (DAP, boundary scan)", {dap, bs}, 0, 0, 5);
  run("boundary scan, no-IDCODE device, DAP", {bs, anon, dap}, 2, 8, 0);
  run("other, DAP, boundary scan", {other, dap, bs}, 1, 7, 5);
  printf(fails ? "%d FAILURES\n" : "JTAG SIM OK\n", fails);
  return fails != 0;
}
