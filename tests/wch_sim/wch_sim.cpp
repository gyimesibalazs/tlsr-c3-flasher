// Simulated CH32V003 behind the single-wire debug interface, for wch.cpp and wchflash.cpp without hardware.
//
//   g++ -std=c++17 -Wall -Wextra -I tests/wch_sim/stub -I src -o /tmp/wch_sim tests/wch_sim/wch_sim.cpp
//       src/wch.cpp src/wchflash.cpp && /tmp/wch_sim      (the C++ checks)
//   /tmp/wch_sim --serve      (text protocol on stdin / stdout, used by test_wch_host.py)
//
// The real wch.cpp / wchflash.cpp run against stubs of the RMT driver. Every RMT transmission is turned into a
// line waveform; the simulated target decodes it with the windows of the QingKe V2 debug manual (T = 125 ns
// by default: 1 = low T..4T, 0 = low 6T..64T, high T..16T inside a packet, stop >= 18T, reset >= 256T low) and
// counts every violation. For reads it holds the line low for a 0 bit (8T from our edge, an assumption: the
// manual does not give it) and the receiver gets the merged waveform as RMT symbols, with a little jitter.
// Behind the line: the debug module (dmcontrol, dmstatus, abstract commands, autoexec, program buffer), a small
// RV32EC interpreter for the program buffer, the bus (flash, boot area, ESIG, option bytes, RAM, flash
// controller) and a timeline: power-on, the core starting, and the firmware taking the SWIO pin (UART TX)
// a given time after it starts running unless the core was halted first.
//
// Modelling choices that are not from a datasheet: the read-protected flash reads 0xE339E339 over debug, page
// erase / programming / mass erase are refused with WRPRTERR while read protection is on, programming RDPR =
// 0xA5 while protected erases the user flash, and the option bytes are reloaded at power-on and at a system
// reset (ndmreset or the PFIC reset).
#include <Arduino.h>
#include <deque>
#include <string>
#include <vector>
#include <cmath>
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "config.h"
#include "wch.h"
#include "wchflash.h"

// ================================================================ time and the RTOS / GPIO stubs

static uint64_t now_ns = 0;
static void chip_update();
static void advance(uint64_t ns) {
  now_ns += ns;
  chip_update();
}
uint32_t micros() { return (uint32_t)(now_ns / 1000); }
uint32_t millis() { return (uint32_t)(now_ns / 1000000); }
void delay(uint32_t ms) { advance((uint64_t)ms * 1000000); }
void pinMode(int, int) {}
void gpio_pullup_en(gpio_num_t) {}

struct SimQueue {
  std::deque<std::vector<uint8_t>> q;
  size_t item;
};
QueueHandle_t xQueueCreate(int, size_t item) {
  SimQueue *q = new SimQueue;
  q->item = item;
  return q;
}
void xQueueReset(QueueHandle_t q) { q->q.clear(); }
BaseType_t xQueueReceive(QueueHandle_t q, void *item, uint32_t ms) {
  if (q->q.empty()) {
    advance((uint64_t)ms * 1000000);
    return pdFALSE;
  }
  memcpy(item, q->q.front().data(), q->item);
  q->q.pop_front();
  return pdTRUE;
}
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken) {
  q->q.emplace_back((const uint8_t *)item, (const uint8_t *)item + q->item);
  *woken = pdFALSE;
  return pdTRUE;
}

// ================================================================ the chip

static const uint32_t SIM_CHIPID = 0x00330500;   // CH32V003 family id as minichlink masks it (0x00300500)
static uint32_t rng = 12345;
static uint32_t rnd() { return rng = rng * 1103515245 + 12345, (rng >> 16) & 0x7fff; }

struct Chip {
  // scenario
  bool present = true;
  bool power_pins = true;        // the target's VDD hangs on the power pins (else external power)
  int64_t take_pin_us = -1;      // the firmware turns SWIO into UART TX this long after it starts (-1 = never)
  uint64_t boot_ns = 200000;     // power-on until the core and the debug interface work
  double T = 125.0;              // debug clock period
  double zero_hold_T = 8;        // a 0 read from the target: line held low this many T from our falling edge
  int jitter_ticks = 2;          // receiver measurement noise, in 12.5 ns ticks
  // state
  bool powered = true, alive = true, running = true, halted = false, pin_taken = false;
  uint64_t t_on = 0, t_run = 0;
  bool outen = false;
  uint32_t shdw = 0, cfgr = 0;
  bool dmactive = false, haltreq = false, ndmreset = false, havereset = false, resumeack = false;
  uint32_t data0 = 0, data1 = 0, command = 0, autoexec = 0, progbuf[8] = {0}, cmderr = 0;
  uint64_t busy_until = 0;
  uint32_t x[16] = {0};
  uint8_t flash[16384], boot[1920], opt[64], ram[2048];
  // flash controller
  bool lock = true, flock = true, optwre = false, boot_lock = true, wrprterr = false, eop = false, boot_mode = false;
  int keyst = 0, mkeyst = 0, okeyst = 0, bkeyst = 0;
  uint32_t ctlr = 0, faddr = 0, buf[16];
  bool pend = false;
  uint32_t pend_addr = 0, pend_val = 0;
  uint64_t fbusy_until = 0;
  bool rdprt = false;            // loaded at reset from RDPR
  // statistics
  int violations = 0, packets = 0, resets = 0;
  std::string last_violation;
  uint64_t line_low_since = 0;   // the host holds the line low since (0 = not)
};
static Chip chip;

static void violation(const char *what) {
  chip.violations++;
  chip.last_violation = what;
}

static void load_options() { chip.rdprt = chip.opt[0] != 0xA5; }

static void lock_flash_controller() {
  chip.lock = chip.flock = chip.boot_lock = true;
  chip.optwre = false;
  chip.ctlr = 0;
  chip.keyst = chip.mkeyst = chip.okeyst = chip.bkeyst = 0;
  chip.pend = false;
}

static void system_reset() {        // everything but the debug module
  chip.resets++;
  lock_flash_controller();
  load_options();
  chip.pin_taken = false;
  chip.havereset = true;
  chip.t_run = now_ns;
  chip.halted = chip.haltreq;        // a halt request held through the reset stops the core at the reset vector
  chip.running = !chip.halted;
  chip.resumeack = false;
  memset(chip.x, 0, sizeof(chip.x));
}

static void dm_reset() {
  chip.dmactive = chip.haltreq = chip.ndmreset = false;
  chip.data0 = chip.data1 = chip.command = chip.autoexec = chip.cmderr = 0;
  memset(chip.progbuf, 0, sizeof(chip.progbuf));
  chip.outen = false;
  chip.shdw = chip.cfgr = 0;
}

static void power(bool on) {
  if (!chip.power_pins) return;      // externally powered: the power pins are not connected
  if (on && !chip.powered) {
    chip.powered = true;
    chip.alive = false;
    chip.t_on = now_ns;
  } else if (!on && chip.powered) {
    chip.powered = chip.alive = chip.running = chip.halted = chip.pin_taken = false;
    dm_reset();
    memset(chip.ram, 0x5a, sizeof(chip.ram));
  }
}

static void chip_update() {
  if (!chip.present || !chip.powered) return;
  if (!chip.alive && now_ns >= chip.t_on + chip.boot_ns) {
    chip.alive = true;
    dm_reset();
    system_reset();                  // haltreq is clear after power-on: the firmware starts
    chip.t_run = chip.t_on + chip.boot_ns;
  }
  if (chip.alive && chip.running && !chip.pin_taken && chip.take_pin_us >= 0 &&
      now_ns >= chip.t_run + (uint64_t)chip.take_pin_us * 1000)
    chip.pin_taken = true;
}

static void new_chip(const Chip &proto) {
  chip = proto;
  for (auto &b : chip.flash) b = (uint8_t)rnd();
  memset(chip.boot, 0xff, sizeof(chip.boot));
  for (int i = 0; i < 1920; i++) chip.boot[i] = (uint8_t)(i * 7);
  memset(chip.ram, 0, sizeof(chip.ram));
  memset(chip.opt, 0xff, sizeof(chip.opt));
  static const uint8_t factory[16] = {0xa5, 0x5a, 0x97, 0x68, 0x00, 0xff, 0x00, 0xff,
                                      0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00};
  memcpy(chip.opt, factory, 16);
  chip.t_on = now_ns;
  dm_reset();
  if (chip.powered) {
    chip.alive = true;
    system_reset();
  }
}

// ---------------------------------------------------------------- bus

static bool in_main(uint32_t a) { return a >= 0x08000000 && a < 0x08000000 + sizeof(chip.flash); }
static bool in_boot(uint32_t a) { return a >= 0x1FFFF000 && a < 0x1FFFF000 + sizeof(chip.boot); }
static bool in_opt(uint32_t a) { return a >= 0x1FFFF800 && a < 0x1FFFF840; }
static uint8_t *byte_at(uint32_t a) {
  if (a < sizeof(chip.flash)) a += 0x08000000;
  if (in_main(a)) return chip.flash + (a - 0x08000000);
  if (in_boot(a)) return chip.boot + (a - 0x1FFFF000);
  if (in_opt(a)) return chip.opt + (a - 0x1FFFF800);
  if (a >= 0x20000000 && a < 0x20000000 + sizeof(chip.ram)) return chip.ram + (a - 0x20000000);
  return NULL;
}

static uint32_t flash_reg_read(uint32_t off) {
  switch (off) {
    case 0x0C:
      return (now_ns < chip.fbusy_until ? 1 : 0) | chip.wrprterr << 4 | chip.eop << 5 | chip.boot_mode << 14 |
             chip.boot_lock << 15;
    case 0x10: return chip.ctlr | chip.lock << 7 | chip.optwre << 9 | chip.flock << 15;
    case 0x14: return chip.faddr;
    case 0x1C: return (chip.opt[0] != (uint8_t)~chip.opt[1]) | chip.rdprt << 1 | (uint32_t)chip.opt[2] << 2;
    case 0x20: return chip.opt[8] | chip.opt[10] << 8 | chip.opt[12] << 16 | (uint32_t)chip.opt[14] << 24;
  }
  return 0;
}

static bool key_step(int &st, uint32_t v) {
  if (st == 0 && v == 0x45670123) {
    st = 1;
    return false;
  }
  bool done = st == 1 && v == 0xCDEF89AB;
  st = 0;
  return done;
}

static void flash_reg_write(uint32_t off, uint32_t v) {
  const uint32_t PAGE_PG = 1u << 16, PAGE_ER = 1u << 17, BUF_LOAD = 1u << 18, BUF_RST = 1u << 19, STRT = 1u << 6,
                 MER = 1u << 2, OPTPG = 1u << 4, OPTER = 1u << 5, LOCK = 1u << 7, OPTWRE = 1u << 9;
  switch (off) {
    case 0x04:
      if (key_step(chip.keyst, v)) chip.lock = false;
      return;
    case 0x24:
      if (key_step(chip.mkeyst, v) && !chip.lock) chip.flock = false;
      return;
    case 0x08:
      if (key_step(chip.okeyst, v) && !chip.lock) chip.optwre = true;
      return;
    case 0x28:
      if (key_step(chip.bkeyst, v)) chip.boot_lock = false;
      return;
    case 0x0C:
      if (v & (1u << 4)) chip.wrprterr = false;
      if (v & (1u << 5)) chip.eop = false;
      if (!chip.boot_lock) chip.boot_mode = (v >> 14) & 1;
      return;
    case 0x14:
      chip.faddr = v;
      return;
    case 0x10:
      break;
    default:
      return;
  }
  if (chip.lock) return;
  if (v & LOCK) {
    lock_flash_controller();
    return;
  }
  if (!(v & OPTWRE)) chip.optwre = false;
  if (now_ns < chip.fbusy_until) violation("CTLR written while the flash controller is busy");
  chip.ctlr = v & (PAGE_PG | PAGE_ER | MER | OPTPG | OPTER | 3);
  uint32_t a = chip.faddr < sizeof(chip.flash) ? chip.faddr + 0x08000000 : chip.faddr;
  if ((v & BUF_RST) && (v & PAGE_PG)) {
    memset(chip.buf, 0xff, sizeof(chip.buf));
    chip.fbusy_until = now_ns + 1000;
  }
  if ((v & BUF_LOAD) && (v & PAGE_PG)) {
    if (!chip.pend) violation("BUF_LOAD without a word stored first");
    chip.buf[(chip.pend_addr & 63) / 4] = chip.pend_val;
    chip.pend = false;
    chip.fbusy_until = now_ns + 1000;
  }
  if (!(v & STRT)) return;
  bool refuse_main = chip.rdprt || chip.flock;
  if (v & PAGE_ER) {
    a &= ~63u;
    bool ok = (in_main(a) && !refuse_main) || (in_boot(a) && !chip.boot_lock && !chip.flock);
    if (ok) memset(byte_at(a), 0xff, in_boot(a) ? std::min<uint32_t>(64, 0x1FFFF000 + 1920 - a) : 64);
    else chip.wrprterr = true;
    chip.fbusy_until = now_ns + 2000000;
  } else if (v & MER) {
    if (chip.rdprt) chip.wrprterr = true;
    else memset(chip.flash, 0xff, sizeof(chip.flash));
    chip.fbusy_until = now_ns + 20000000;
  } else if (v & PAGE_PG) {
    a &= ~63u;
    bool ok = (in_main(a) && !refuse_main) || (in_boot(a) && !chip.boot_lock && !chip.flock);
    if (ok) {
      uint8_t *p = byte_at(a);
      int n = in_boot(a) ? std::min<uint32_t>(64, 0x1FFFF000 + 1920 - a) : 64;
      for (int i = 0; i < n; i++) p[i] &= (uint8_t)(chip.buf[i / 4] >> (8 * (i % 4)));
    } else {
      chip.wrprterr = true;
    }
    chip.fbusy_until = now_ns + 1500000;
  } else if (v & OPTER) {
    if (chip.optwre) memset(chip.opt, 0xff, sizeof(chip.opt));
    else chip.wrprterr = true;
    chip.fbusy_until = now_ns + 5000000;
  }
  chip.eop = true;
}

static bool bus_read(uint32_t a, int size, uint32_t &v) {
  if (a == 0xE00000F4 && size == 4) return v = chip.data0, true;
  if (a == 0xE00000F8 && size == 4) return v = chip.data1, true;
  if (a & (size - 1)) return false;
  if (a >= 0x40022000 && a < 0x40022030) return size == 4 ? (v = flash_reg_read(a - 0x40022000), true) : false;
  if (a >= 0x1FFFF7C0 && a < 0x1FFFF800) {
    uint32_t w = 0;
    uint32_t wa = a & ~3u;
    if (wa == 0x1FFFF7C4) w = SIM_CHIPID;
    else if (wa == 0x1FFFF7E0) w = 0x0010;            // FLACAP: 16 k
    else if (wa == 0x1FFFF7E8) w = 0x12345678;
    else if (wa == 0x1FFFF7EC) w = 0x9abcdef0;
    else if (wa == 0x1FFFF7F0) w = 0x0badcafe;
    v = (w >> (8 * (a & 3))) & (size == 4 ? 0xffffffff : size == 2 ? 0xffff : 0xff);
    return true;
  }
  uint8_t *p = byte_at(a);
  if (!p) return false;
  uint32_t fa = a < sizeof(chip.flash) ? a + 0x08000000 : a;
  if (in_main(fa) && chip.rdprt) {
    v = 0xE339E339 & (size == 4 ? 0xffffffff : size == 2 ? 0xffff : 0xff);
    return true;
  }
  v = 0;
  for (int i = 0; i < size; i++) v |= (uint32_t)p[i] << (8 * i);
  return true;
}

static bool bus_write(uint32_t a, int size, uint32_t v) {
  if (a == 0xE00000F4 && size == 4) return chip.data0 = v, true;
  if (a == 0xE00000F8 && size == 4) return chip.data1 = v, true;
  if (a & (size - 1)) return false;
  if (a >= 0x40022000 && a < 0x40022030) {
    if (size != 4) return false;
    flash_reg_write(a - 0x40022000, v);
    return true;
  }
  if (a == 0xE000E048 && size == 4) {
    if ((v >> 16) == 0xBEEF && (v & 0x80)) system_reset();
    return true;
  }
  uint32_t fa = a < sizeof(chip.flash) ? a + 0x08000000 : a;
  if (in_main(fa) || in_boot(fa)) {
    if (!(chip.ctlr & (1u << 16)) || chip.flock || size != 4) return false;   // only buffer loads
    chip.pend = true;
    chip.pend_addr = fa;
    chip.pend_val = v;
    return true;
  }
  if (in_opt(a)) {
    if (!(chip.ctlr & (1u << 4)) || !chip.optwre || size != 2) return false;
    uint8_t *p = byte_at(a);
    if (a == 0x1FFFF800 && (v & 0xff) == 0xA5 && chip.rdprt)
      memset(chip.flash, 0xff, sizeof(chip.flash));     // removing the read protection erases the user flash
    p[0] &= (uint8_t)v;
    p[1] &= (uint8_t)(v >> 8);
    chip.fbusy_until = now_ns + 1000000;
    chip.eop = true;
    return true;
  }
  uint8_t *p = byte_at(a);
  if (!p || !(a >= 0x20000000 && a < 0x20000800)) return false;
  for (int i = 0; i < size; i++) p[i] = (uint8_t)(v >> (8 * i));
  return true;
}

// ---------------------------------------------------------------- RV32EC subset for the program buffer

static int32_t sext(uint32_t v, int bits) { return (int32_t)(v << (32 - bits)) >> (32 - bits); }

static bool reg_ok(int r) { return r >= 0 && r < 16; }

// returns false on an exception
static bool run_progbuf() {
  uint8_t mem[34];
  for (int i = 0; i < 8; i++)
    for (int b = 0; b < 4; b++) mem[4 * i + b] = (uint8_t)(chip.progbuf[i] >> (8 * b));
  mem[32] = 0x02;                    // the implicit c.ebreak after a full buffer
  mem[33] = 0x90;
  uint32_t pc = 0;
  uint32_t *x = chip.x;
  for (int steps = 0; steps < 200000; steps++) {
    now_ns += 125;                   // 8 MHz core clock after reset
    if (pc > 32) return false;
    uint32_t h = mem[pc] | mem[pc + 1] << 8;
    x[0] = 0;
    if ((h & 3) != 3) {
      uint32_t f3 = h >> 13, q = h & 3;
      if (h == 0x9002) return true;  // c.ebreak
      if (q == 0 && (f3 == 2 || f3 == 6)) {   // c.lw / c.sw
        int r2 = 8 + ((h >> 2) & 7), r1 = 8 + ((h >> 7) & 7);
        uint32_t imm = ((h >> 10) & 7) << 3 | ((h >> 6) & 1) << 2 | ((h >> 5) & 1) << 6;
        if (f3 == 2) {
          uint32_t v;
          if (!bus_read(x[r1] + imm, 4, v)) return false;
          x[r2] = v;
        } else if (!bus_write(x[r1] + imm, 4, x[r2])) {
          return false;
        }
        pc += 2;
      } else if (q == 1 && (f3 == 0 || f3 == 2)) {   // c.addi (c.nop) / c.li
        int rd = (h >> 7) & 31;
        int32_t imm = sext(((h >> 12) & 1) << 5 | ((h >> 2) & 31), 6);
        if (!reg_ok(rd)) return false;
        x[rd] = f3 == 0 ? x[rd] + imm : (uint32_t)imm;
        pc += 2;
      } else if (q == 1 && f3 == 4 && ((h >> 10) & 3) == 2) {   // c.andi
        int rd = 8 + ((h >> 7) & 7);
        x[rd] &= (uint32_t)sext(((h >> 12) & 1) << 5 | ((h >> 2) & 31), 6);
        pc += 2;
      } else if (q == 1 && f3 == 5) {  // c.j
        uint32_t imm = ((h >> 12) & 1) << 11 | ((h >> 11) & 1) << 4 | ((h >> 9) & 3) << 8 | ((h >> 8) & 1) << 10 |
                       ((h >> 7) & 1) << 6 | ((h >> 6) & 1) << 7 | ((h >> 3) & 7) << 1 | ((h >> 2) & 1) << 5;
        pc += sext(imm, 12);
      } else if (q == 1 && (f3 == 6 || f3 == 7)) {   // c.beqz / c.bnez
        int r1 = 8 + ((h >> 7) & 7);
        uint32_t imm = ((h >> 12) & 1) << 8 | ((h >> 10) & 3) << 3 | ((h >> 5) & 3) << 6 | ((h >> 3) & 3) << 1 |
                       ((h >> 2) & 1) << 5;
        bool take = f3 == 6 ? x[r1] == 0 : x[r1] != 0;
        pc += take ? sext(imm, 9) : 2;
      } else if (q == 2 && f3 == 4 && ((h >> 2) & 31) != 0) {   // c.mv / c.add
        int rd = (h >> 7) & 31, rs = (h >> 2) & 31;
        if (!reg_ok(rd) || !reg_ok(rs)) return false;
        x[rd] = (h >> 12) & 1 ? x[rd] + x[rs] : x[rs];
        pc += 2;
      } else {
        return false;                // not in the subset
      }
      continue;
    }
    if (pc + 4 > 34) return false;
    uint32_t w = h | (uint32_t)(mem[pc + 2] | mem[pc + 3] << 8) << 16;
    uint32_t op = w & 0x7f, f3 = (w >> 12) & 7;
    int rd = (w >> 7) & 31, r1 = (w >> 15) & 31, r2 = (w >> 20) & 31;
    if (w == 0x00100073) return true;   // ebreak
    bool uses_r2 = op == 0x23 || op == 0x63, uses_rd = op != 0x23 && op != 0x63;
    if ((uses_rd && !reg_ok(rd)) || (op != 0x37 && !reg_ok(r1)) || (uses_r2 && !reg_ok(r2))) return false;
    if (op == 0x37) {
      x[rd] = w & 0xfffff000;
    } else if (op == 0x13 && (f3 == 0 || f3 == 4 || f3 == 6 || f3 == 7)) {
      int32_t imm = sext(w >> 20, 12);
      x[rd] = f3 == 0 ? x[r1] + imm : f3 == 4 ? x[r1] ^ imm : f3 == 6 ? x[r1] | imm : x[r1] & imm;
    } else if (op == 0x03 && f3 != 3 && f3 < 6) {
      int size = 1 << (f3 & 3);
      uint32_t v;
      if (!bus_read(x[r1] + sext(w >> 20, 12), size, v)) return false;
      if (f3 == 0) v = (uint32_t)sext(v, 8);
      if (f3 == 1) v = (uint32_t)sext(v, 16);
      x[rd] = v;
    } else if (op == 0x23 && f3 < 3) {
      int32_t imm = sext((w >> 25) << 5 | ((w >> 7) & 31), 12);
      if (!bus_write(x[r1] + imm, 1 << f3, x[r2])) return false;
    } else if (op == 0x63 && f3 < 2) {
      int32_t imm = sext(((w >> 31) & 1) << 12 | ((w >> 7) & 1) << 11 | ((w >> 25) & 63) << 5 | ((w >> 8) & 15) << 1, 13);
      bool take = f3 == 0 ? x[r1] == x[r2] : x[r1] != x[r2];
      if (take) {
        pc += imm;
        continue;
      }
    } else {
      return false;
    }
    pc += 4;
  }
  return false;                      // runaway loop
}

// ---------------------------------------------------------------- debug module

static bool busy() { return now_ns < chip.busy_until; }

static void exec_command(uint32_t cmd) {
  if (chip.cmderr) return;           // nothing runs until cmderr is cleared
  if (cmd >> 24) return void(chip.cmderr = 2);
  if (!chip.halted) return void(chip.cmderr = 4);
  if (cmd & (1u << 17)) {            // transfer
    uint32_t reg = cmd & 0xffff;
    if (((cmd >> 20) & 7) != 2 || reg < 0x1000 || reg > 0x100f) return void(chip.cmderr = 2);
    if (cmd & (1u << 16)) chip.x[reg - 0x1000] = chip.data0;
    else chip.data0 = chip.x[reg - 0x1000];
    chip.x[0] = 0;
  }
  if (cmd & (1u << 18)) {
    if (!run_progbuf()) chip.cmderr = 3;
  }
  chip.busy_until = now_ns + 2000;
}

static uint32_t dmi_read(uint8_t a) {
  uint32_t v = 0;
  switch (a) {
    case 0x04: v = chip.data0; break;
    case 0x05: v = chip.data1; break;
    case 0x10: v = chip.dmactive | chip.ndmreset << 1; break;
    case 0x11:
      v = 0x82 | (chip.halted ? 0x300 : 0) | (chip.running ? 0xc00 : 0) | (chip.resumeack ? 0x30000 : 0) |
          (chip.havereset ? 0xc0000 : 0);
      break;
    case 0x12: v = 0x002120f4; break;
    case 0x16: v = 0x08000002 | (busy() ? 1u << 12 : 0) | chip.cmderr << 8; break;
    case 0x18: v = chip.autoexec; break;
    case 0x40: v = chip.halted; break;
    case 0x7C: v = 0x00010003 | (chip.outen ? 0x400 : 0); break;
    case 0x7D: v = chip.cfgr; break;
    case 0x7E: v = chip.shdw; break;
    case 0x7F: v = SIM_CHIPID; break;
    default:
      if (a >= 0x20 && a < 0x28) v = chip.progbuf[a - 0x20];
  }
  if ((a == 0x04 || a == 0x05) && (chip.autoexec >> (a - 4)) & 1) {
    if (busy()) {
      if (!chip.cmderr) chip.cmderr = 1;
    } else {
      exec_command(chip.command);
    }
  }
  return v;
}

static void dmi_write(uint8_t a, uint32_t v) {
  if ((a == 0x04 || a == 0x05 || a == 0x17 || (a >= 0x20 && a < 0x28)) && busy()) {
    if (!chip.cmderr) chip.cmderr = 1;
    return;
  }
  switch (a) {
    case 0x04: chip.data0 = v; break;
    case 0x05: chip.data1 = v; break;
    case 0x10: {
      bool was_reset = chip.ndmreset, was_active = chip.dmactive;
      chip.dmactive = v & 1;
      if (!chip.dmactive) {          // debug module registers to their reset values (not the SDI configuration)
        chip.haltreq = chip.ndmreset = false;
        chip.data0 = chip.data1 = chip.command = chip.autoexec = chip.cmderr = 0;
        memset(chip.progbuf, 0, sizeof(chip.progbuf));
        return;
      }
      if (!was_active) return;       // the write that sets dmactive changes nothing else (debug spec 0.13)
      chip.haltreq = v >> 31;
      chip.ndmreset = (v >> 1) & 1;
      if ((v >> 28) & 1) chip.havereset = false;
      if (chip.ndmreset && !was_reset) {
        system_reset();
        chip.running = chip.halted = false;   // held in reset
      }
      if (!chip.ndmreset && was_reset) {
        chip.t_run = now_ns;
        chip.halted = chip.haltreq;
        chip.running = !chip.halted;
      }
      if (chip.haltreq && chip.running) {
        chip.running = false;
        chip.halted = true;
        chip.resumeack = false;
      }
      if (((v >> 30) & 1) && chip.halted && !chip.haltreq) {
        chip.halted = false;
        chip.running = true;
        chip.resumeack = true;
        chip.t_run = now_ns;
      }
      break;
    }
    case 0x16: chip.cmderr &= ~((v >> 8) & 7); break;
    case 0x17:
      chip.command = v;
      exec_command(v);
      return;
    case 0x18: chip.autoexec = v; break;
    case 0x7D:
      if ((v >> 16) == 0x5AA5) {
        chip.cfgr = v;
        chip.outen = (chip.shdw >> 10) & 1;
      }
      break;
    case 0x7E:
      if ((v >> 16) == 0x5AA5) chip.shdw = v;
      break;
    default:
      if (a >= 0x20 && a < 0x28) chip.progbuf[a - 0x20] = v;
  }
  if ((a == 0x04 || a == 0x05) && (chip.autoexec >> (a - 4)) & 1) exec_command(chip.command);
}

// ================================================================ the wire: RMT stubs

struct SimRmtChannel {
  bool tx;
  rmt_tx_done_callback_t txcb = NULL;
  rmt_rx_done_callback_t rxcb = NULL;
  void *user = NULL;
  rmt_symbol_word_t *rxbuf = NULL;
  size_t rxcap = 0;
  rmt_receive_config_t rxcfg{};
  bool armed = false;
};
struct SimRmtEncoder {
  int dummy;
};
static SimRmtChannel *rx_chan = NULL;
static int host_level = 1;           // what GPIO 4 drives between transmissions
static uint64_t sim_tx_latency_ns = 12000;   // driver / interrupt time before each transmission starts

esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *, rmt_channel_handle_t *ret) {
  *ret = new SimRmtChannel{true};
  return ESP_OK;
}
esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t *, rmt_channel_handle_t *ret) {
  *ret = new SimRmtChannel{false};
  rx_chan = *ret;
  return ESP_OK;
}
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *, rmt_encoder_handle_t *ret) {
  *ret = new SimRmtEncoder{0};
  return ESP_OK;
}
esp_err_t rmt_del_encoder(rmt_encoder_handle_t e) {
  delete e;
  return ESP_OK;
}
esp_err_t rmt_enable(rmt_channel_handle_t) { return ESP_OK; }
esp_err_t rmt_disable(rmt_channel_handle_t ch) {
  ch->armed = false;
  return ESP_OK;
}
esp_err_t rmt_del_channel(rmt_channel_handle_t ch) {
  if (ch == rx_chan) rx_chan = NULL;
  delete ch;
  return ESP_OK;
}
esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t ch, const rmt_tx_event_callbacks_t *c, void *u) {
  ch->txcb = c->on_trans_done;
  ch->user = u;
  return ESP_OK;
}
esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t ch, const rmt_rx_event_callbacks_t *c, void *u) {
  ch->rxcb = c->on_recv_done;
  ch->user = u;
  return ESP_OK;
}
esp_err_t rmt_receive(rmt_channel_handle_t ch, void *buf, size_t size, const rmt_receive_config_t *c) {
  ch->rxbuf = (rmt_symbol_word_t *)buf;
  ch->rxcap = size / sizeof(rmt_symbol_word_t);
  ch->rxcfg = *c;
  ch->armed = true;
  return ESP_OK;
}
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t, int) { return ESP_OK; }

static int line_level() {
  if (!chip.present) return host_level;
  if (!chip.powered) return 0;       // an unpowered chip clamps the line through its protection diode
  if (chip.pin_taken) return 1;      // UART TX idles high and is stronger than our 1 k
  return host_level;
}
int gpio_get_level(gpio_num_t) { return line_level(); }

// The target's view of one transmission: lows and highs in ns, decoded with the manual's windows.
struct Seg {
  double low, high;                  // high = the time until the next falling edge (or the end)
};

static int bit_of(double low, double T, bool &bad) {
  if (low > T && low < 4 * T) return 1;
  if (low > 6 * T && low < 64 * T) return 0;
  bad = true;
  return -1;
}

static void target_packet(const std::vector<Seg> &segs, std::vector<std::pair<double, double>> &extra_low) {
  const double T = chip.T;
  for (auto &s : segs)
    if (s.low >= 256 * T) {          // interface reset
      chip.outen = false;
      return;
    }
  if (segs.empty()) return;
  chip.packets++;
  char msg[160];
  bool bad = false;
  std::vector<int> bits;
  int nhead = std::min<int>(9, segs.size());
  for (int i = 0; i < nhead; i++) bits.push_back(bit_of(segs[i].low, T, bad));
  if (bad || nhead < 9) {
    snprintf(msg, sizeof(msg), "header bit outside the windows (lows %.0f ...)", segs[0].low);
    return violation(msg);
  }
  for (size_t i = 0; i + 1 < segs.size(); i++)
    if (segs[i].high <= T || (segs[i].high >= 16 * T && i + 1 < segs.size())) {
      snprintf(msg, sizeof(msg), "high of %.0f ns inside a packet (bit %zu)", segs[i].high, i);
      return violation(msg);
    }
  if (segs.back().high < 18 * T) return violation("no stop after the packet");
  if (bits[0] != 1) return violation("start bit 0");
  uint8_t addr = 0;
  for (int i = 1; i <= 7; i++) addr = addr << 1 | bits[i];
  if (bits[8] == 1) {                // host writes
    if (segs.size() != 41) {
      snprintf(msg, sizeof(msg), "write packet of %zu bits", segs.size());
      return violation(msg);
    }
    uint32_t v = 0;
    for (int i = 9; i < 41; i++) {
      int b = bit_of(segs[i].low, T, bad);
      if (bad) return violation("data bit outside the windows");
      v = v << 1 | b;
    }
    dmi_write(addr, v);
    return;
  }
  if (segs.size() != 41) {
    snprintf(msg, sizeof(msg), "read packet with %zu pulses", segs.size());
    return violation(msg);
  }
  bool answer = chip.outen;
  uint32_t v = dmi_read(addr);
  double t = 0;
  for (int i = 0; i < 9; i++) t += segs[i].low + segs[i].high;
  for (int i = 0; i < 32; i++) {
    const Seg &s = segs[9 + i];
    if (!(s.low > T && s.low < 4 * T)) return violation("read pulse outside the 1 window");
    bool zero = answer && !((v >> (31 - i)) & 1);
    if (zero) {
      double hold = chip.zero_hold_T * T;
      if (s.low + s.high < hold + T) return violation("read slot too short for the target's answer");
      extra_low.push_back({t, t + hold});
    }
    t += s.low + s.high;
  }
}

static void deliver_rx(const std::vector<std::pair<int, double>> &wave) {
  // wave: (level, ns) of the line during the transmission, ending with the idle high
  if (!rx_chan || !rx_chan->armed) return;
  const rmt_receive_config_t &c = rx_chan->rxcfg;
  // merge pieces shorter than the glitch filter into their neighbours
  std::vector<std::pair<int, double>> w;
  for (auto &p : wave) {
    if (!w.empty() && (w.back().first == p.first || p.second < c.signal_range_min_ns)) w.back().second += p.second;
    else w.push_back(p);
  }
  // start at the first falling edge
  size_t i = 0;
  while (i < w.size() && w[i].first == 1) i++;
  if (i >= w.size()) return;         // no edge: the receiver keeps waiting
  std::vector<rmt_symbol_word_t> out;
  for (; i < w.size(); i += 2) {
    rmt_symbol_word_t s;
    s.val = 0;
    auto tk = [&](double ns) {
      int t = (int)lround(ns / 12.5) + (int)(rnd() % (2 * chip.jitter_ticks + 1)) - chip.jitter_ticks;
      return (uint16_t)std::max(1, std::min(32767, t));
    };
    s.level0 = 0;
    s.duration0 = tk(w[i].second);
    s.level1 = 1;
    bool last = i + 1 >= w.size() || w[i + 1].second > c.signal_range_max_ns;
    s.duration1 = last ? 0 : tk(w[i + 1].second);
    out.push_back(s);
    if (last) break;
  }
  size_t n = std::min(out.size(), rx_chan->rxcap);
  memcpy(rx_chan->rxbuf, out.data(), n * sizeof(rmt_symbol_word_t));
  rx_chan->armed = false;
  rmt_rx_done_event_data_t ed{};
  ed.received_symbols = rx_chan->rxbuf;
  ed.num_symbols = n;
  rx_chan->rxcb(rx_chan, &ed, rx_chan->user);
}

esp_err_t rmt_transmit(rmt_channel_handle_t ch, rmt_encoder_handle_t, const void *payload, size_t bytes,
                       const rmt_transmit_config_t *c) {
  advance(sim_tx_latency_ns);
  const rmt_symbol_word_t *s = (const rmt_symbol_word_t *)payload;
  int n = (int)(bytes / sizeof(rmt_symbol_word_t));
  // the host's own waveform
  std::vector<std::pair<int, double>> hw;
  for (int i = 0; i < n; i++) {
    hw.push_back({(int)s[i].level0, s[i].duration0 * 12.5});
    hw.push_back({(int)s[i].level1, s[i].duration1 * 12.5});
  }
  double total = 0;
  for (auto &p : hw) total += p.second;
  bool reaches = chip.present && chip.powered && !chip.pin_taken;
  bool alive = reaches && chip.alive;
  // a low the host held before this transmission (power-off, line reset) counts as one long low
  if (host_level == 0 && alive && chip.line_low_since && (now_ns - chip.line_low_since) >= 256 * chip.T)
    chip.outen = false;
  std::vector<std::pair<double, double>> extra;
  if (alive) {
    std::vector<Seg> segs;
    for (size_t i = 0; i < hw.size(); i++) {
      if (hw[i].first == 0 && hw[i].second > 0) {
        double low = hw[i].second, high = 0;
        size_t j = i + 1;
        while (j < hw.size() && hw[j].first == 0) low += hw[j++].second;
        while (j < hw.size() && hw[j].first == 1) high += hw[j++].second;
        if (j >= hw.size() && c->flags.eot_level) high += 1e9;   // idle high afterwards
        segs.push_back({low, high});
        i = j - 1;
      }
    }
    if (!(segs.size() == 1 && segs[0].high < 1e8 && !c->flags.eot_level)) target_packet(segs, extra);
  }
  // the line as GPIO 5 sees it
  std::vector<std::pair<int, double>> wave;
  if (reaches || !chip.present) {
    double t = 0;
    for (auto &p : hw) {
      // split the host piece at the target's low intervals
      double a = t, b = t + p.second;
      double cur = a;
      for (auto &e : extra) {
        if (e.second <= cur || e.first >= b) continue;
        if (p.first == 1) {
          if (e.first > cur) wave.push_back({1, e.first - cur});
          double end = std::min(b, e.second);
          wave.push_back({0, end - std::max(cur, e.first)});
          cur = end;
        }
      }
      if (b > cur) wave.push_back({p.first, b - cur});
      t = b;
    }
    if (c->flags.eot_level) wave.push_back({1, 1e6});
  }
  advance((uint64_t)total);
  host_level = c->flags.eot_level;
  chip.line_low_since = host_level ? 0 : now_ns;
  if (!wave.empty()) deliver_rx(wave);
  if (ch->txcb) {
    rmt_tx_done_event_data_t ed{(size_t)n + 1};
    ch->txcb(ch, &ed, ch->user);
  }
  return ESP_OK;
}

// ================================================================ checks

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      failures++;                                         \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);         \
      printf(__VA_ARGS__);                                \
      printf("\n");                                       \
    }                                                     \
  } while (0)

static WchTiming deftim() {
  return WchTiming{WCH_ONE_LOW_NS, WCH_ZERO_LOW_NS, WCH_HIGH_NS, WCH_READ_SLOT_NS, WCH_READ_THRESHOLD_NS, WCH_STOP_NS};
}

// Encoding and decoding on their own, with receiver captures of a read of CPBR (0x00010403) and of 0xA5C3_0F01.
// The captures are synthetic: our pulses at 20 / 80 ticks with +-3 ticks of noise, the target's 0 bits as lows
// of 70..90 ticks (WCH's reference host and minichlink's CH32V003 programmer imply roughly 1 us), a few of them
// split by a short high right after our edge, as a slow target would produce.
static void test_encoding() {
  WchTiming t = deftim();
  rmt_symbol_word_t s[48];
  int n = wch_encode_write(0x7D, 0x5AA50400, t, s);
  CHECK(n == 41, "write is %d symbols", n);
  // start 1, address 1111101, write 1
  const int head[9] = {1, 1, 1, 1, 1, 1, 0, 1, 1};
  for (int i = 0; i < 9; i++)
    CHECK(s[i].duration0 == (head[i] ? 20 : 80) && s[i].level0 == 0 && s[i].level1 == 1, "header bit %d", i);
  for (int i = 0; i < 32; i++) {
    int b = (0x5AA50400u >> (31 - i)) & 1;
    CHECK(s[9 + i].duration0 == (b ? 20 : 80), "data bit %d", i);
  }
  CHECK(s[40].duration1 == 320, "stop %u ticks", (unsigned)s[40].duration1);
  CHECK(s[0].duration1 == 30, "high %u ticks", (unsigned)s[0].duration1);
  n = wch_encode_read(0x7C, t, s);
  CHECK(n == 41 && s[8].duration0 == 80, "read header");
  for (int i = 9; i < 41; i++) CHECK(s[i].duration0 == 20 && (i == 40 || s[i].duration1 == 100), "read slot %d", i);

  const uint32_t values[2] = {0x00010403, 0xA5C30F01};
  for (int k = 0; k < 2; k++) {
    rmt_symbol_word_t cap[60];
    int m = 0;
    uint32_t seed = 77 + k;
    auto noise = [&](int a) { seed = seed * 1103515245 + 12345; return a + (int)((seed >> 16) % 7) - 3; };
    rmt_symbol_word_t hd[48];
    wch_encode_read(0x7C, t, hd);
    for (int i = 0; i < 9; i++) {
      cap[m].val = 0;
      cap[m].level0 = 0;
      cap[m].duration0 = noise(hd[i].duration0);
      cap[m].level1 = 1;
      cap[m].duration1 = noise(hd[i].duration1);
      m++;
    }
    for (int i = 0; i < 32; i++) {
      bool one = (values[k] >> (31 - i)) & 1;
      int low = one ? noise(20) : 70 + (int)((seed >> 8) % 21);
      cap[m].val = 0;
      cap[m].level0 = 0;
      cap[m].level1 = 1;
      if (!one && i % 7 == 3) {      // split: our pulse, 9 ticks high, then the target's low
        cap[m].duration0 = 20;
        cap[m].duration1 = 9;
        m++;
        cap[m].val = 0;
        cap[m].level0 = 0;
        cap[m].level1 = 1;
        low -= 29;
      }
      cap[m].duration0 = low;
      cap[m].duration1 = i == 31 ? 0 : 120 - low;
      m++;
    }
    uint32_t v = 0;
    int r = wch_decode_read(cap, m, t, &v);
    CHECK(r == WCH_OK && v == values[k], "decode %08x: r=%d v=%08x (%s)", values[k], r, v, wch_last_detail());
  }
  // a capture that lost a pulse, and one where something else drives the line during our header
  rmt_symbol_word_t cap[48];
  wch_encode_read(0x11, t, cap);
  uint32_t v;
  CHECK(wch_decode_read(cap, 40, t, &v) == WCH_ERR_FRAME, "short capture accepted");
  cap[3].duration0 = 400;
  CHECK(wch_decode_read(cap, 41, t, &v) == WCH_ERR_FRAME, "distorted header accepted");
}

static void fresh(const Chip &proto) {
  new_chip(proto);
  wchf_set_power(power);
  wch_end();
  host_level = 1;
  CHECK(wch_begin(), "wch_begin");
}

static void test_attach() {
  // externally powered, firmware leaves SWIO alone
  Chip p;
  p.power_pins = false;
  fresh(p);
  WchAttach a;
  int r = wch_attach(NULL, 0, 50, &a);
  CHECK(r == WCH_OK && a.halted, "attach (external power): %d %s", r, wch_attach_reason(a));
  CHECK(a.cpbr == 0x00010403, "CPBR %08x", (unsigned)a.cpbr);
  CHECK(chip.halted, "core not halted");
  CHECK(chip.violations == 0, "timing violations: %d (%s)", chip.violations, chip.last_violation.c_str());

  // powered from the pins, firmware takes SWIO 1.5 ms after start: the catch must win
  p = Chip();
  p.take_pin_us = 1500;
  fresh(p);
  r = wch_attach(power, 300, 100, &a);
  CHECK(r == WCH_OK && a.halted && a.powered, "catch: %d %s", r, wch_attach_reason(a));
  CHECK(!chip.pin_taken, "the firmware took the pin");
  printf("catch: halted %u us after power-on, try %u (target alive after %u us)\n", (unsigned)a.first_us,
         (unsigned)a.tries, (unsigned)(p.boot_ns / 1000));
  CHECK(chip.violations == 0, "timing violations: %d (%s)", chip.violations, chip.last_violation.c_str());

  // the same without the power cycle: too late, the pin is UART TX
  p.take_pin_us = 1500;
  fresh(p);
  advance(5000000);
  r = wch_attach(NULL, 0, 20, &a);
  CHECK(r != WCH_OK && a.n_nopulse > 0 && strstr(wch_attach_reason(a), "never went low"), "late attach: %d %s", r,
        wch_attach_reason(a));
  printf("external power, pin taken: %s\n", wch_attach_reason(a));

  // a firmware that takes the pin after 100 us: faster than the catch (the first answer comes later)
  p.take_pin_us = 100;
  fresh(p);
  r = wch_attach(power, 300, 30, &a);
  CHECK(r != WCH_OK && strstr(wch_attach_reason(a), "not caught in time"), "too fast: %d %s", r, wch_attach_reason(a));

  // no chip on the wire
  p = Chip();
  p.present = false;
  fresh(p);
  r = wch_attach(power, 300, 20, &a);
  CHECK(r != WCH_OK && a.n_ones > 0 && strstr(wch_attach_reason(a), "nothing answers"), "absent: %s",
        wch_attach_reason(a));
  // powered off and not on the power pins: the line is clamped low
  p = Chip();
  p.power_pins = false;
  p.powered = false;
  fresh(p);
  r = wch_attach(NULL, 0, 20, &a);
  CHECK(r != WCH_OK && a.n_stucklow > 0, "unpowered: %s", wch_attach_reason(a));

  // the debug clock off by +-10 % (HSI not trimmed, temperature): the defaults still fit the windows
  for (double T : {112.5, 137.5}) {
    p = Chip();
    p.T = T;
    fresh(p);
    r = wch_attach(power, 300, 50, &a);
    CHECK(r == WCH_OK && chip.violations == 0, "T = %.1f ns: %d, %d violations (%s)", T, r, chip.violations,
          chip.last_violation.c_str());
  }
}

static void test_memory_and_flash() {
  Chip p;
  fresh(p);
  WchAttach a;
  CHECK(wch_attach(power, 300, 50, &a) == WCH_OK, "attach");
  CHECK(wchf_halt(true) == WCH_OK, "halt at reset");
  // RAM round trip, single and block
  uint8_t w[256], b[256];
  for (int i = 0; i < 256; i++) w[i] = (uint8_t)(i * 13 + 1);
  CHECK(wchf_mem_write(0x20000100, w, 256) == WCH_OK, "RAM write");
  CHECK(wchf_mem_read(0x20000100, b, 256) == WCH_OK && !memcmp(w, b, 256), "RAM read back");
  uint32_t v = 0;
  CHECK(wchf_read32(0x1FFFF7C4, &v) == WCH_OK && v == SIM_CHIPID, "CHIPID %08x", (unsigned)v);
  CHECK(wchf_read32(0x40000000, &v) == WCH_ERR_CMD && wchf_cmderr() == 3, "bus fault not reported");
  CHECK(wchf_read32(0x1FFFF7E0, &v) == WCH_OK && (v & 0xffff) == 16, "FLACAP %08x", (unsigned)v);
  // flash: erase a page, program it partly, read back
  CHECK(wchf_flash_erase(0x08000040) == WCH_OK, "page erase");
  CHECK(wchf_mem_read(0x08000040, b, 64) == WCH_OK, "read erased page");
  bool blank = true;
  for (int i = 0; i < 64; i++) blank &= b[i] == 0xff;
  CHECK(blank, "page not blank");
  CHECK(wchf_flash_write(0x08000040, w, 64) == WCH_OK, "program page");
  CHECK(wchf_mem_read(0x08000040, b, 64) == WCH_OK && !memcmp(b, w, 64), "page read back");
  CHECK(!memcmp(chip.flash + 0x40, w, 64), "flash content");
  CHECK(wchf_flash_erase(0x08000080) == WCH_OK && wchf_flash_write(0x08000088, w, 20) == WCH_OK, "partial page");
  CHECK(!memcmp(chip.flash + 0x88, w, 20) && chip.flash[0x80] == 0xff && chip.flash[0x9c] == 0xff, "partial content");
  CHECK(wchf_flash_write(0x08000070, w, 32) == WCH_ERR_ADDRESS, "page crossing accepted");
  // mass erase
  CHECK(wchf_flash_mass_erase() == WCH_OK, "mass erase");
  blank = true;
  for (auto c : chip.flash) blank &= c == 0xff;
  CHECK(blank, "mass erase left data");
  // boot area: needs its own key
  CHECK(wchf_flash_erase(0x1FFFF000) == WCH_OK && wchf_flash_write(0x1FFFF000, w, 64) == WCH_OK, "boot area");
  CHECK(!memcmp(chip.boot, w, 64), "boot area content");
  CHECK(wchf_flash_erase(0x1FFFF740) == WCH_OK && chip.boot[1919] == 0xff, "last boot page (64 bytes, 1920 = 30 x 64)");
  // option bytes
  uint8_t ob[16] = {0xa5, 0x5a, 0x97, 0x68, 0x12, 0xed, 0x34, 0xcb, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00};
  CHECK(wchf_options_write(ob) == WCH_OK && !memcmp(chip.opt, ob, 16), "options write");
  CHECK(chip.violations == 0, "timing violations: %d (%s)", chip.violations, chip.last_violation.c_str());
  // run: the firmware starts and (here) takes the pin
  chip.take_pin_us = 500;
  CHECK(wchf_reset_run() == WCH_OK && chip.running, "reset and run");
  advance(2000000);
  CHECK(chip.pin_taken, "firmware did not start");
}

static void test_protected() {
  Chip p;
  fresh(p);
  chip.opt[0] = 0x00;                // RDPR not 0xA5: read protected after the next reset
  chip.opt[1] = 0xff;
  WchAttach a;
  CHECK(wch_attach(power, 300, 50, &a) == WCH_OK, "attach");
  CHECK(chip.rdprt, "not protected");
  uint32_t obr = 0, v = 0;
  CHECK(wchf_read32(0x4002201C, &obr) == WCH_OK && (obr & 2), "OBR.RDPRT %08x", (unsigned)obr);
  CHECK(wchf_read32(0x08000000, &v) == WCH_OK && v == 0xE339E339, "protected read %08x", (unsigned)v);
  CHECK(wchf_flash_erase(0x08000000) == WCH_ERR_WRPRT, "erase allowed while protected");
  uint8_t keep = chip.flash[5];
  CHECK(chip.flash[5] == keep, "flash changed");
  // remove the protection: options with RDPR = 0xA5; the user flash is erased with it
  uint8_t ob[16] = {0xa5, 0x5a, 0x97, 0x68, 0x00, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00};
  CHECK(wchf_options_write(ob) == WCH_OK, "unprotect");
  bool blank = true;
  for (auto c : chip.flash) blank &= c == 0xff;
  CHECK(blank, "flash not erased by the unprotect");
  CHECK(wch_attach(power, 300, 50, &a) == WCH_OK && !chip.rdprt, "still protected after power cycle");
  CHECK(wchf_halt(false) == WCH_OK, "halt");   // also forgets the program buffer of the chip before the power cycle
  CHECK(wchf_read32(0x4002201C, &obr) == WCH_OK && !(obr & 2), "OBR after %08x", (unsigned)obr);
}

// ================================================================ serve: the text protocol for the Python tests

static void out_line(const char *s) {
  fputs(s, stdout);
  fputc('\n', stdout);
  fflush(stdout);
}

static void serve() {
  char ln[9000];
  Chip proto;
  fresh(proto);
  while (fgets(ln, sizeof(ln), stdin)) {
    char *argv[12] = {NULL};
    int argc = 0;
    for (char *tok = strtok(ln, " \t\r\n"); tok && argc < 12; tok = strtok(NULL, " \t\r\n")) argv[argc++] = tok;
    if (!argc) continue;
    if (!strcmp(argv[0], "SIM")) {
      // SIM new [present=0] [pins=0] [take=us] [protect=1] [off=1] [T=ns]: a new chip (random flash)
      // SIM peek <addr> <len>: hex straight from the model; SIM poke <addr> <hex>; SIM stat; SIM wait <ms>
      static char o[400];
      if (argc > 1 && !strcmp(argv[1], "new")) {
        Chip p;
        for (int i = 2; i < argc; i++) {
          char *eq = strchr(argv[i], '=');
          if (!eq) continue;
          *eq = 0;
          double v = atof(eq + 1);
          if (!strcmp(argv[i], "present")) p.present = v != 0;
          if (!strcmp(argv[i], "pins")) p.power_pins = v != 0;
          if (!strcmp(argv[i], "take")) p.take_pin_us = (int64_t)v;
          if (!strcmp(argv[i], "T")) p.T = v;
          if (!strcmp(argv[i], "off")) p.powered = v == 0;
        }
        bool protect = false;
        for (int i = 2; i < argc; i++)
          if (!strcmp(argv[i], "protect")) protect = true;
        new_chip(p);
        if (protect) {
          chip.opt[0] = 0x00;
          chip.opt[1] = 0xff;
          load_options();
        }
        // the firmware keeps its line state: only the chip changes
        out_line("OK");
      } else if (argc > 3 && !strcmp(argv[1], "peek")) {
        uint32_t a = strtoul(argv[2], NULL, 0), n = strtoul(argv[3], NULL, 0);
        std::string hex = "OK ";
        for (uint32_t i = 0; i < n; i++) {
          uint8_t *p = byte_at(a + i);
          char h[3];
          snprintf(h, sizeof(h), "%02x", p ? *p : 0);
          hex += h;
        }
        out_line(hex.c_str());
      } else if (argc > 3 && !strcmp(argv[1], "poke")) {
        uint32_t a = strtoul(argv[2], NULL, 0);
        for (size_t i = 0; argv[3][2 * i] && argv[3][2 * i + 1]; i++) {
          char h[3] = {argv[3][2 * i], argv[3][2 * i + 1], 0};
          uint8_t *p = byte_at(a + i);
          if (p) *p = (uint8_t)strtoul(h, NULL, 16);
        }
        out_line("OK");
      } else if (argc > 2 && !strcmp(argv[1], "wait")) {
        advance((uint64_t)(atof(argv[2]) * 1e6));
        out_line("OK");
      } else if (argc > 1 && !strcmp(argv[1], "stat")) {
        snprintf(o, sizeof(o), "OK violations=%d packets=%d halted=%d running=%d pin_taken=%d rdprt=%d last=%s",
                 chip.violations, chip.packets, chip.halted, chip.running, chip.pin_taken, chip.rdprt,
                 chip.last_violation.empty() ? "-" : chip.last_violation.c_str());
        out_line(o);
      } else {
        out_line("ERR SIM new|peek|poke|wait|stat");
      }
      continue;
    }
    if (!strcmp(argv[0], "V")) {
      out_line("OK tlsr-c3-flasher 0.22 simulated CH32V003");
      continue;
    }
    if (!strcmp(argv[0], "PWR")) {   // the host switches the power pins off at the end
      out_line("OK");
      continue;
    }
    if (!wchf_command(argc, argv, out_line)) out_line("ERR unknown command");
  }
}

int main(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "--serve")) {
    serve();
    return 0;
  }
  test_encoding();
  test_attach();
  test_memory_and_flash();
  test_protected();
  printf(failures ? "%d FAILED\n" : "all WCH simulation checks passed\n", failures);
  return failures ? 1 : 0;
}
