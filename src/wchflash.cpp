// WCH CH32V003 debug module and flash controller, over the single-wire interface of wch.cpp.
//
// Debug module (QingKe V2 debug manual, chapter 3; RISC-V debug spec 0.13): dmcontrol 0x10, dmstatus 0x11,
// hartinfo 0x12, abstractcs 0x16, command 0x17, abstractauto 0x18, data0 / data1 0x04 / 0x05 (also mapped
// at 0xE00000F4 / 0xE00000F8), progbuf0..7 0x20..0x27. Only register access is an abstract command here;
// memory is reached by small programs in the program buffer, run with "postexec" (command 0x00040000).
//
// Memory access uses the read / write program of WCH's own one-wire host (CH32V003_R_W_blk): data1 holds
// the address, its bit 0 selects write; the program moves one word between data0 and that address and adds
// 4 to data1. With abstractauto set, every access to data0 runs it again, so a block costs one register
// read or write per word. Flash pages are loaded with a second program (after minichlink's WriteWord):
// store the word, step the address, set BUF_LOAD and wait for BSY. The option bytes are written half-word
// by half-word with a third one. All three were checked with the RISC-V assembler (rv32ec).
//
// Flash controller at 0x40022000 (CH32V003 reference manual; ch32v003fun ch32v003hw.h; WCH flashop.c):
// KEYR +0x04, OBKEYR +0x08, STATR +0x0C (BSY 0, WRPRTERR 4, EOP 5, BOOT_MODE 14, BOOT_LOCK 15), CTLR +0x10
// (PER 1, MER 2, OPTPG 4, OPTER 5, STRT 6, LOCK 7, OPTWRE 9, FLOCK 15, PAGE_PG 16, PAGE_ER 17, BUF_LOAD 18,
// BUF_RST 19), ADDR +0x14, OBR +0x1C (OPTERR 0, RDPRT 1, USER 2..9), WPR +0x20, MODEKEYR +0x24,
// BOOT_MODEKEYR +0x28. Keys 0x45670123, 0xCDEF89AB.
//   page erase (64 B):  CTLR = PAGE_ER, ADDR = page, CTLR = PAGE_ER | STRT, wait BSY
//   page program:       CTLR = PAGE_PG, CTLR = PAGE_PG | BUF_RST, wait; per word: store it, CTLR = PAGE_PG |
//                       BUF_LOAD, wait; then ADDR = page, CTLR = PAGE_PG | STRT, wait
//   mass erase:         CTLR = MER, CTLR = MER | STRT, wait
//   option bytes:       OBKEYR keys (OPTWRE), CTLR = OPTER | OPTWRE (+ STRT), wait; CTLR = OPTPG | OPTWRE,
//                       half-word stores, wait after each
#include <Arduino.h>
#include "wch.h"
#include "wchflash.h"

// debug module registers
static const uint8_t DATA0 = 0x04, DATA1 = 0x05, DMCONTROL = 0x10, DMSTATUS = 0x11, ABSTRACTCS = 0x16,
                     COMMAND = 0x17, ABSTRACTAUTO = 0x18, PROGBUF0 = 0x20, CFGR = 0x7D, SHDWCFGR = 0x7E;
static const uint32_t CMD_EXEC = 0x00040000;          // postexec only: run the program buffer
static const uint32_t CMD_WRITE_GPR = 0x00231000;     // 32-bit, transfer, write: data0 -> x<n>

// flash controller
static const uint32_t FLASH_R = 0x40022000, KEYR = FLASH_R + 0x04, OBKEYR = FLASH_R + 0x08, STATR = FLASH_R + 0x0C,
                      CTLR = FLASH_R + 0x10, ADDR = FLASH_R + 0x14, MODEKEYR = FLASH_R + 0x24,
                      BOOT_MODEKEYR = FLASH_R + 0x28;
static const uint32_t KEY1 = 0x45670123, KEY2 = 0xCDEF89AB;
static const uint32_t SR_BSY = 1u << 0, SR_WRPRTERR = 1u << 4, SR_EOP = 1u << 5, SR_BOOT_MODE = 1u << 14,
                      SR_BOOT_LOCK = 1u << 15;
static const uint32_t CR_MER = 1u << 2, CR_OPTPG = 1u << 4, CR_OPTER = 1u << 5, CR_STRT = 1u << 6, CR_LOCK = 1u << 7,
                      CR_OPTWRE = 1u << 9, CR_FLOCK = 1u << 15, CR_PAGE_PG = 1u << 16, CR_PAGE_ER = 1u << 17,
                      CR_BUF_LOAD = 1u << 18, CR_BUF_RST = 1u << 19;

// program buffer contents
// read / write one word, data1 = address (| 1 = write), data1 += 4 (WCH CH32V003_R_W_blk; fills all 32 bytes,
// the debug module adds the ebreak)
static const uint32_t PROG_RW[8] = {0xe0000537, 0x0f450513, 0xf693414c, 0x8563ffc5,
                                    0x411000d5, 0xa019c290, 0xc1104290, 0xc14c0591};
//   lui a0,0xe0000 / addi a0,a0,244 / c.lw a1,4(a0) / andi a3,a1,-4 / beq a1,a3,read /
//   c.lw a2,0(a0) / c.sw a2,0(a3) / c.j inc / read: c.lw a2,0(a3) / c.sw a2,0(a0) / inc: c.addi a1,4 / c.sw a1,4(a0)
// flash buffer load: a0 = &data0, a4 = &STATR, a5 = PAGE_PG | BUF_LOAD (set once by abstract commands)
static const uint32_t PROG_FLASH[5] = {0x4110414c, 0x0591c190, 0xc35cc14c, 0x8a854314, 0x9002fef5};
//   c.lw a1,4(a0) / c.lw a2,0(a0) / c.sw a2,0(a1) / c.addi a1,4 / c.sw a1,4(a0) / c.sw a5,4(a4) /
//   1: c.lw a3,0(a4) / c.andi a3,1 / c.bnez a3,1b / c.ebreak
// one half-word: *(uint16_t *)data1 = data0
static const uint32_t PROG_HALF[3] = {0x4110414c, 0x00c59023, 0x00009002};
//   c.lw a1,4(a0) / c.lw a2,0(a0) / sh a2,0(a1) / c.ebreak

enum Prog { P_NONE, P_RW, P_FLASH, P_HALF };
static Prog loaded = P_NONE;
static bool gprs_set = false, main_unlocked = false, boot_unlocked = false;
static int last_cmderr = 0;
static void (*power_fn)(bool) = NULL;

void wchf_set_power(void (*power)(bool on)) { power_fn = power; }
int wchf_cmderr() { return last_cmderr; }

static void forget() {
  loaded = P_NONE;
  gprs_set = main_unlocked = boot_unlocked = false;
}

#define TRY(x)                \
  do {                        \
    int r_ = (x);             \
    if (r_ != WCH_OK) return r_; \
  } while (0)

// wait for the abstract command to finish; a cmderr is cleared and reported
static int abstract_wait() {
  uint32_t v = 0;
  for (int i = 0; i < 50; i++) {
    TRY(wch_dmi_read(ABSTRACTCS, &v));
    if (!(v & (1u << 12))) break;
  }
  if (v & (1u << 12)) return WCH_ERR_BUSY;
  int ce = (v >> 8) & 7;
  if (ce) {
    last_cmderr = ce;
    wch_dmi_write(ABSTRACTCS, 0x00000700);
    return WCH_ERR_CMD;
  }
  return WCH_OK;
}

static int write_gpr(int reg, uint32_t v) {
  TRY(wch_dmi_write(DATA0, v));
  TRY(wch_dmi_write(COMMAND, CMD_WRITE_GPR | reg));
  return abstract_wait();
}

static int load(Prog p) {
  if (loaded == p) return WCH_OK;
  TRY(wch_dmi_write(ABSTRACTAUTO, 0));
  const uint32_t *w = p == P_RW ? PROG_RW : p == P_FLASH ? PROG_FLASH : PROG_HALF;
  int n = p == P_RW ? 8 : p == P_FLASH ? 5 : 3;
  for (int i = 0; i < n; i++) TRY(wch_dmi_write(PROGBUF0 + i, w[i]));
  loaded = p;
  if (p != P_RW && !gprs_set) {
    // x10 = &data0 (the read / write program leaves the same value there), x14 = &STATR,
    // x15 = PAGE_PG | BUF_LOAD; none of the programs changes x14 / x15
    TRY(write_gpr(10, 0xE00000F4));
    TRY(write_gpr(14, STATR));
    TRY(write_gpr(15, CR_PAGE_PG | CR_BUF_LOAD));
    gprs_set = true;
  }
  return WCH_OK;
}

int wchf_read32(uint32_t addr, uint32_t *v) {
  TRY(load(P_RW));
  TRY(wch_dmi_write(DATA1, addr & ~3u));
  TRY(wch_dmi_write(COMMAND, CMD_EXEC));
  TRY(abstract_wait());
  return wch_dmi_read(DATA0, v);
}

int wchf_write32(uint32_t addr, uint32_t v) {
  TRY(load(P_RW));
  TRY(wch_dmi_write(DATA0, v));
  TRY(wch_dmi_write(DATA1, (addr & ~3u) | 1));
  TRY(wch_dmi_write(COMMAND, CMD_EXEC));
  return abstract_wait();
}

static uint32_t get32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static void put32(uint8_t *p, uint32_t v) {
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}

// As WCH's SingleLineReadMEMDirectBlock: the first word by one command, the rest by autoexec on data0
// (each read of data0 returns a word and fetches the next one). An access while the program still runs would
// set cmderr; it is checked once at the end, and the program takes a few microseconds against the ~60 us of
// a register read.
int wchf_mem_read(uint32_t addr, uint8_t *out, int len) {
  if ((addr | (uint32_t)len) & 3 || len < 4) return WCH_ERR_ADDRESS;
  int n = len / 4;
  uint32_t w;
  TRY(load(P_RW));
  TRY(wch_dmi_write(DATA1, addr));
  TRY(wch_dmi_write(COMMAND, CMD_EXEC));
  TRY(abstract_wait());
  if (n > 1) TRY(wch_dmi_write(ABSTRACTAUTO, 1));
  for (int i = 0; i < n - 1; i++) {
    int r = wch_dmi_read(DATA0, &w);
    if (r != WCH_OK) {
      wch_dmi_write(ABSTRACTAUTO, 0);
      return r;
    }
    put32(out + 4 * i, w);
  }
  if (n > 1) TRY(wch_dmi_write(ABSTRACTAUTO, 0));
  TRY(abstract_wait());
  TRY(wch_dmi_read(DATA0, &w));
  put32(out + 4 * (n - 1), w);
  return WCH_OK;
}

int wchf_mem_write(uint32_t addr, const uint8_t *data, int len) {
  if ((addr | (uint32_t)len) & 3 || len < 4) return WCH_ERR_ADDRESS;
  int n = len / 4;
  TRY(load(P_RW));
  TRY(wch_dmi_write(DATA0, get32(data)));
  TRY(wch_dmi_write(DATA1, addr | 1));
  TRY(wch_dmi_write(COMMAND, CMD_EXEC));
  TRY(abstract_wait());
  if (n > 1) {
    TRY(wch_dmi_write(ABSTRACTAUTO, 1));
    for (int i = 1; i < n; i++) TRY(wch_dmi_write(DATA0, get32(data + 4 * i)));
    TRY(wch_dmi_write(ABSTRACTAUTO, 0));
    TRY(abstract_wait());
  }
  return WCH_OK;
}

// ---------------------------------------------------------------- core control

static int enable_output() {
  TRY(wch_dmi_write(SHDWCFGR, 0x5AA50400));
  return wch_dmi_write(CFGR, 0x5AA50400);
}

int wchf_halt(bool reset) {
  forget();
  TRY(enable_output());
  TRY(wch_dmi_write(DMCONTROL, 0x80000001));
  TRY(wch_dmi_write(DMCONTROL, 0x80000001));
  if (reset) {
    TRY(wch_dmi_write(DMCONTROL, 0x80000003));   // ndmreset with the halt request held: stops at the reset vector
    TRY(wch_dmi_write(DMCONTROL, 0x80000001));
    TRY(enable_output());                         // in case the system reset reached the interface too
  }
  uint32_t v = 0;
  for (int i = 0; i < 100 && (v & 0x300) != 0x300; i++) TRY(wch_dmi_read(DMSTATUS, &v));
  if ((v & 0x300) != 0x300) return WCH_ERR_NOTHALTED;
  TRY(wch_dmi_write(DMCONTROL, reset ? 0x10000001 : 0x00000001));   // clear the request (and havereset)
  TRY(wch_dmi_write(ABSTRACTAUTO, 0));
  TRY(wch_dmi_write(ABSTRACTCS, 0x00000700));
  return WCH_OK;
}

int wchf_resume() {
  forget();
  TRY(wch_dmi_write(DMCONTROL, 0x00000001));
  TRY(wch_dmi_write(DMCONTROL, 0x40000001));
  return WCH_OK;
}

int wchf_reset_run() {
  forget();
  TRY(enable_output());
  TRY(wch_dmi_write(DMCONTROL, 0x80000001));
  TRY(wch_dmi_write(DMCONTROL, 0x80000003));
  TRY(wch_dmi_write(DMCONTROL, 0x80000001));   // out of reset, halted at the reset vector
  TRY(enable_output());
  uint32_t v = 0;
  for (int i = 0; i < 100 && (v & 0x300) != 0x300; i++) TRY(wch_dmi_read(DMSTATUS, &v));
  TRY(wch_dmi_write(DMCONTROL, 0x10000001));
  TRY(wch_dmi_write(DMCONTROL, 0x40000001));   // resume; from here on the firmware may take the pin
  return WCH_OK;
}

// ---------------------------------------------------------------- flash controller

static bool in_main(uint32_t a) { return a >= WCH_FLASH_BASE && a < WCH_FLASH_BASE + 0x40000; }
static bool in_boot(uint32_t a) { return a >= WCH_BOOT_BASE && a < WCH_BOOT_BASE + WCH_BOOT_SIZE; }

static int flash_wait(uint32_t timeout_ms) {
  uint32_t t0 = millis(), sr = 0;
  for (;;) {
    TRY(wchf_read32(STATR, &sr));
    if (!(sr & SR_BSY)) break;
    if (millis() - t0 > timeout_ms) return WCH_ERR_TIMEOUT;
  }
  if (sr & (SR_WRPRTERR | SR_EOP))             // write 1 to clear; the boot mode bit keeps its value
    TRY(wchf_write32(STATR, (sr & SR_BOOT_MODE) | SR_WRPRTERR | SR_EOP));
  return sr & SR_WRPRTERR ? WCH_ERR_WRPRT : WCH_OK;
}

int wchf_flash_unlock() {
  uint32_t cr = 0;
  TRY(wchf_read32(CTLR, &cr));
  if (cr & (CR_LOCK | CR_FLOCK)) {
    TRY(wchf_write32(KEYR, KEY1));
    TRY(wchf_write32(KEYR, KEY2));
    TRY(wchf_write32(MODEKEYR, KEY1));            // fast (64-byte) erase and programming
    TRY(wchf_write32(MODEKEYR, KEY2));
    TRY(wchf_read32(CTLR, &cr));
    if (cr & (CR_LOCK | CR_FLOCK)) return WCH_ERR_LOCKED;
  }
  main_unlocked = true;
  return WCH_OK;
}

static int unlock_for(uint32_t addr) {
  if (!main_unlocked) TRY(wchf_flash_unlock());
  if (in_boot(addr) && !boot_unlocked) {
    uint32_t sr = 0;
    TRY(wchf_read32(STATR, &sr));
    if (sr & SR_BOOT_LOCK) {
      TRY(wchf_write32(BOOT_MODEKEYR, KEY1));
      TRY(wchf_write32(BOOT_MODEKEYR, KEY2));
      TRY(wchf_read32(STATR, &sr));
      if (sr & SR_BOOT_LOCK) return WCH_ERR_LOCKED;
    }
    boot_unlocked = true;
  }
  return WCH_OK;
}

int wchf_flash_erase(uint32_t addr) {
  if (!in_main(addr) && !in_boot(addr)) return WCH_ERR_ADDRESS;
  addr &= ~(WCH_PAGE - 1);
  TRY(unlock_for(addr));
  TRY(flash_wait(10));
  TRY(wchf_write32(CTLR, CR_PAGE_ER));
  TRY(wchf_write32(ADDR, addr));
  TRY(wchf_write32(CTLR, CR_PAGE_ER | CR_STRT));
  int r = flash_wait(50);
  wchf_write32(CTLR, 0);
  return r;
}

int wchf_flash_mass_erase() {
  TRY(unlock_for(WCH_FLASH_BASE));
  TRY(flash_wait(10));
  TRY(wchf_write32(CTLR, CR_MER));
  TRY(wchf_write32(CTLR, CR_MER | CR_STRT));
  int r = flash_wait(500);
  wchf_write32(CTLR, 0);
  return r;
}

int wchf_flash_write(uint32_t addr, const uint8_t *data, int len) {
  if ((addr & 3) || len < 1 || (addr & (WCH_PAGE - 1)) + len > (int)WCH_PAGE) return WCH_ERR_ADDRESS;
  if (!in_main(addr) && !in_boot(addr)) return WCH_ERR_ADDRESS;
  uint8_t buf[WCH_PAGE];
  memset(buf, 0xff, sizeof(buf));
  memcpy(buf, data, len);
  int n = (len + 3) / 4;
  TRY(unlock_for(addr));
  TRY(flash_wait(10));
  TRY(wchf_write32(CTLR, CR_PAGE_PG));
  TRY(wchf_write32(CTLR, CR_PAGE_PG | CR_BUF_RST));
  TRY(flash_wait(10));
  TRY(load(P_FLASH));
  TRY(wch_dmi_write(DATA0, get32(buf)));
  TRY(wch_dmi_write(DATA1, addr));
  TRY(wch_dmi_write(COMMAND, CMD_EXEC));
  TRY(abstract_wait());
  if (n > 1) {
    TRY(wch_dmi_write(ABSTRACTAUTO, 1));
    for (int i = 1; i < n; i++) TRY(wch_dmi_write(DATA0, get32(buf + 4 * i)));
    TRY(wch_dmi_write(ABSTRACTAUTO, 0));
    TRY(abstract_wait());
  }
  TRY(wchf_write32(ADDR, addr & ~(WCH_PAGE - 1)));
  TRY(wchf_write32(CTLR, CR_PAGE_PG | CR_STRT));
  int r = flash_wait(20);
  wchf_write32(CTLR, 0);
  return r;
}

int wchf_options_write(const uint8_t ob[16]) {
  TRY(wchf_flash_unlock());
  TRY(wchf_write32(OBKEYR, KEY1));
  TRY(wchf_write32(OBKEYR, KEY2));
  uint32_t cr = 0;
  TRY(wchf_read32(CTLR, &cr));
  if (!(cr & CR_OPTWRE)) return WCH_ERR_LOCKED;
  TRY(flash_wait(10));
  TRY(wchf_write32(CTLR, CR_OPTER | CR_OPTWRE));
  TRY(wchf_write32(CTLR, CR_OPTER | CR_OPTWRE | CR_STRT));
  TRY(flash_wait(100));
  for (int i = 0; i < 8; i++) {
    TRY(wchf_write32(CTLR, CR_OPTPG | CR_OPTWRE));
    TRY(load(P_HALF));
    TRY(wch_dmi_write(DATA0, ob[2 * i] | ob[2 * i + 1] << 8));
    TRY(wch_dmi_write(DATA1, WCH_OPT_BASE + 2 * i));
    TRY(wch_dmi_write(COMMAND, CMD_EXEC));
    TRY(abstract_wait());
    TRY(flash_wait(50));
  }
  TRY(wchf_write32(CTLR, 0));                  // OPTPG off, OPTWRE cleared (option bytes locked again)
  uint8_t back[16];
  TRY(wchf_mem_read(WCH_OPT_BASE, back, 16));
  return memcmp(back, ob, 16) ? WCH_ERR_VERIFY : WCH_OK;
}

// ---------------------------------------------------------------- text commands

const char *wchf_err_text(int r) {
  static char out[200];
  if (r == WCH_ERR_CMD) {
    static const char *const names[8] = {"none", "busy", "not supported", "exception (bus fault?)", "core not halted",
                                         "bus error", "parity", "other"};
    snprintf(out, sizeof(out), "%s: cmderr %d, %s", wch_err_name(r), last_cmderr, names[last_cmderr & 7]);
  } else if (r == WCH_ERR_FRAME || r == WCH_ERR_NOPULSE || r == WCH_ERR_STUCKLOW) {
    snprintf(out, sizeof(out), "%s (%s)", wch_err_name(r), wch_last_detail());
  } else {
    snprintf(out, sizeof(out), "%s", wch_err_name(r));
  }
  return out;
}

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  c |= 0x20;
  return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

static int parse_hex(const char *s, uint8_t *out, int max) {
  int n = 0;
  while (s[0] && s[1]) {
    int a = hexval(s[0]), b = hexval(s[1]);
    if (a < 0 || b < 0 || n >= max) return -1;
    out[n++] = (uint8_t)(a << 4 | b);
    s += 2;
  }
  return s[0] ? -1 : n;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *d, int n) {
  crc = ~crc;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xedb88320UL & (0UL - (crc & 1)));
  }
  return ~crc;
}

bool wchf_command(int argc, char **argv, void (*out)(const char *)) {
  static char line[2 * 1024 + 16];
  static uint8_t buf[1024];
  if (argc < 1 || argv[0][0] != 'W') return false;
  const char *c = argv[0];
  uint32_t a1 = argc > 1 ? strtoul(argv[1], NULL, 0) : 0;
  uint32_t a2 = argc > 2 ? strtoul(argv[2], NULL, 0) : 0;
#define FAIL(r)                                                     \
  do {                                                              \
    snprintf(line, sizeof(line), "ERR wch %s", wchf_err_text(r));   \
    out(line);                                                      \
    return true;                                                    \
  } while (0)
#define CHECK(x)                \
  do {                          \
    int r_ = (x);               \
    if (r_ != WCH_OK) FAIL(r_); \
  } while (0)

  if (!strcmp(c, "WCH")) {                   // WCH [0]: line on (the caller tore the other modes down) / off
    if (argc > 1 && a1 == 0) {
      wch_end();
      out("OK");
      return true;
    }
    forget();
    if (!wch_active() && !wch_begin()) {
      out("ERR RMT setup failed");
      return true;
    }
    WchTiming t = wch_timing();
    snprintf(line, sizeof(line), "OK %u %u %u %u %u %u", (unsigned)t.one_low, (unsigned)t.zero_low,
             (unsigned)t.high, (unsigned)t.read_slot, (unsigned)t.read_threshold, (unsigned)t.stop);
    out(line);
    return true;
  }
  if (!wch_active()) {
    out("ERR not in WCH mode (send WCH first)");
    return true;
  }
  if (!strcmp(c, "WT")) {                    // WT [one zero high slot threshold stop] (ns)
    WchTiming t = wch_timing();
    if (argc >= 7) {
      WchTiming n = {a1, a2, (uint32_t)strtoul(argv[3], NULL, 0), (uint32_t)strtoul(argv[4], NULL, 0),
                     (uint32_t)strtoul(argv[5], NULL, 0), (uint32_t)strtoul(argv[6], NULL, 0)};
      if (n.one_low < 50 || n.zero_low < n.one_low * 2 || n.high < 50 || n.read_slot < n.one_low + 300 ||
          n.read_threshold <= n.one_low || n.stop < 2000 || n.stop > 100000 || n.zero_low > 100000 ||
          n.read_slot > 100000) {
        out("ERR timing: one >= 50, zero >= 2 x one, high >= 50, slot >= one + 300, one < threshold, stop 2000..100000");
        return true;
      }
      wch_set_timing(n);
      t = n;
    }
    snprintf(line, sizeof(line), "OK %u %u %u %u %u %u", (unsigned)t.one_low, (unsigned)t.zero_low,
             (unsigned)t.high, (unsigned)t.read_slot, (unsigned)t.read_threshold, (unsigned)t.stop);
    out(line);
  } else if (!strcmp(c, "WA")) {            // WA <off ms> <window ms>: (power cycle and) catch, core halted
    WchAttach a;
    if (a1 > 0 && !power_fn) {
      out("ERR no power pins");
      return true;
    }
    forget();
    int r = wch_attach(a1 > 0 ? power_fn : NULL, a1, a2 ? a2 : 100, &a);
    if (r == WCH_ERR_NOTHALTED) {
      snprintf(line, sizeof(line), "ERR wch the debug module answers (dmstatus %08x) but the core did not halt",
               (unsigned)a.dmstatus);
      out(line);
      return true;
    }
    if (r != WCH_OK) {
      snprintf(line, sizeof(line), "ERR wch no attach: %s", wch_attach_reason(a));
      out(line);
      return true;
    }
    snprintf(line, sizeof(line), "OK %08x %08x %u %u %d", (unsigned)a.dmstatus, (unsigned)a.cpbr, (unsigned)a.tries,
             (unsigned)a.first_us, a.powered ? 1 : 0);
    out(line);
  } else if (!strcmp(c, "WR")) {            // WR <dmi addr>: OK <value>
    uint32_t v = 0;
    CHECK(wch_dmi_read((uint8_t)a1, &v));
    snprintf(line, sizeof(line), "OK %08x", (unsigned)v);
    out(line);
  } else if (!strcmp(c, "WW")) {            // WW <dmi addr> <value>
    forget();                                // the caller may change anything
    CHECK(wch_dmi_write((uint8_t)a1, a2));
    out("OK");
  } else if (!strcmp(c, "WH")) {            // WH [reset]
    CHECK(wchf_halt(argc > 1 && !strcmp(argv[1], "reset")));
    out("OK");
  } else if (!strcmp(c, "WG")) {            // WG: resume without reset
    CHECK(wchf_resume());
    out("OK");
  } else if (!strcmp(c, "WRUN")) {          // WRUN: reset and run
    CHECK(wchf_reset_run());
    out("OK");
  } else if (!strcmp(c, "WM")) {            // WM <addr> <len>: memory read, hex (len multiple of 4, <= 1024)
    if ((a1 | a2) & 3 || a2 < 4 || a2 > sizeof(buf)) {
      out("ERR addr and len: multiples of 4, len <= 1024");
      return true;
    }
    CHECK(wchf_mem_read(a1, buf, (int)a2));
    static const char hx[] = "0123456789abcdef";
    int k = snprintf(line, sizeof(line), "OK ");
    for (uint32_t i = 0; i < a2; i++) {
      line[k++] = hx[buf[i] >> 4];
      line[k++] = hx[buf[i] & 15];
    }
    line[k] = 0;
    out(line);
  } else if (!strcmp(c, "WMW")) {           // WMW <addr> <hex>: memory write, whole words (RAM, registers)
    int n = argc > 2 ? parse_hex(argv[2], buf, sizeof(buf)) : -1;
    if (n < 4 || (n & 3) || (a1 & 3)) {
      out("ERR data: whole words at a word address");
      return true;
    }
    CHECK(wchf_mem_write(a1, buf, n));
    out("OK");
  } else if (!strcmp(c, "WFU")) {           // WFU: unlock the flash controller, OK <CTLR>
    uint32_t cr = 0;
    CHECK(wchf_flash_unlock());
    CHECK(wchf_read32(CTLR, &cr));
    snprintf(line, sizeof(line), "OK %08x", (unsigned)cr);
    out(line);
  } else if (!strcmp(c, "WFE")) {           // WFE <addr>: erase the 64-byte page
    CHECK(wchf_flash_erase(a1));
    out("OK");
  } else if (!strcmp(c, "WFX")) {           // WFX: mass erase of the user flash
    CHECK(wchf_flash_mass_erase());
    out("OK");
  } else if (!strcmp(c, "WFW")) {           // WFW <addr> <hex>: program inside one erased 64-byte page
    int n = argc > 2 ? parse_hex(argv[2], buf, WCH_PAGE) : -1;
    if (n < 1) {
      out("ERR data: 1..64 bytes inside one page");
      return true;
    }
    CHECK(wchf_flash_write(a1, buf, n));
    out("OK");
  } else if (!strcmp(c, "WFC")) {           // WFC <addr> <len>: CRC-32 of target memory
    if ((a1 | a2) & 3 || a2 < 4 || a2 > 0x40000) {
      out("ERR addr and len: multiples of 4, len <= 256 k");
      return true;
    }
    uint32_t crc = 0;
    for (uint32_t done = 0; done < a2;) {
      int cnt = (int)min((uint32_t)256, a2 - done);
      CHECK(wchf_mem_read(a1 + done, buf, cnt));
      crc = crc32_update(crc, buf, cnt);
      done += cnt;
    }
    snprintf(line, sizeof(line), "OK %08x", (unsigned)crc);
    out(line);
  } else if (!strcmp(c, "WOB")) {           // WOB <32 hex digits>: erase and write the 16 option bytes
    uint8_t ob[16];
    if (argc < 2 || parse_hex(argv[1], ob, 16) != 16) {
      out("ERR 16 bytes of hex");
      return true;
    }
    CHECK(wchf_options_write(ob));
    out("OK");
  } else {
    return false;
  }
  return true;
#undef FAIL
#undef CHECK
}
