// Bus-level simulation of a Puya PY32F002B-type chip (the MS32C001 of the Tuya dimmers) for swdflash.cpp's PY32
// routine: the real src/swdflash.cpp runs against stubs of the swd.h memory functions, which talk to a model of
// the flash controller (keys, page buffer with PG / PGSTRT, page and mass erase, BSY, WRPERR, timing registers that
// must match the factory bytes), the factory configuration and option bytes, RCC ICSCR, IWDG and the core's debug
// registers. Reserved addresses answer with a bus fault, as the datasheet says.
//   g++ -std=c++17 -Wall -Wextra -I tests/py32_sim/stub -I src -o /tmp/py32_sim tests/py32_sim/py32_sim.cpp src/swdflash.cpp && /tmp/py32_sim
//   /tmp/py32_sim --serve      (text protocol on stdin / stdout, used by test_py32_host.py)
#include <Arduino.h>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>
#include "swd.h"
#include "swdflash.h"

static uint32_t ms = 0;
uint32_t millis() { return ms += 1; }
void delay(uint32_t d) { ms += d; }

// ---------------------------------------------------------------- the chip
struct Chip {
  uint32_t flash_size = 18 * 1024;
  std::vector<uint8_t> flash = std::vector<uint8_t>(18 * 1024, 0xFF);
  uint8_t ram[1536];
  uint32_t idcode = 0x20220064;          // DBGMCU IDCODE
  uint32_t icscr = 0x00008000 | 0x1A5;   // HSI_FS = 4
  uint32_t factory[0x80 / 4];            // 0x1FFF0100..0x1FFF017F
  uint32_t optbytes[4] = {0x5FFF50AA, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
  uint32_t optr = 0x00001000;            // IWDG_SW = 1 (software watchdog)
  uint32_t cr = 0xC0000000, sr = 0, ts[9] = {0};
  int key_state = 0;                     // 0 locked, 1 first key seen, 2 unlocked, 3 locked until reset
  int busy = 0;
  std::vector<uint32_t> pbuf;            // page buffer while PG is set
  uint32_t pbuf_page = 0;
  bool pgstrt_early = false;
  uint32_t wrp_page = 0xFFFFFFFF;        // a write-protected page (WRPERR)
  // records
  int option_writes = 0, bad_timing_ops = 0, stray_flash_writes = 0, pg_errors = 0, iwdg_writes = 0;
  int erases = 0, programs = 0, mass_erases = 0, half_writes = 0;
  uint32_t dhcsr = 0, demcr = 0;
  bool halted = false;
  uint32_t iwdg_pr = 0, iwdg_rlr = 0xFFF;

  Chip() {
    memset(ram, 0, sizeof ram);
    for (auto &w : factory) w = 0xFFFFFFFF;
    factory[0] = 0x00009B5A;              // HSI trim word (0x1FFF0100)
    // timing for 24 MHz at 0x1FFF011C (index 7 .. 11), another set at 0x1FFF0130 (index 12 .. 16)
    const uint32_t t24[5] = {0x0C80A0B4, 0x01E00090, 0x0001D4C0, 0x0002BF20, 0x01180F00};
    const uint32_t t48[5] = {0x1900F168, 0x03C00120, 0x0003A980, 0x00057E40, 0x02301E00};
    for (int i = 0; i < 5; i++) factory[7 + i] = t24[i], factory[12 + i] = t48[i];
  }
  const uint32_t *timing_src() const { return (((icscr >> 13) & 7) == 5) ? &factory[12] : &factory[7]; }
  bool timing_ok() const {
    const uint32_t *p = timing_src();
    const uint32_t v[9] = {p[0] & 0x1FF, (p[0] >> 18) & 0x3FF, p[1] & 0x1FF, (p[1] >> 16) & 0xFFF,
                           (p[0] >> 9) & 0x1FF, p[2] & 0x3FFFF, p[3] & 0x3FFFF, p[4] & 0xFFFF, (p[4] >> 16) & 0x3FFF};
    for (int i = 0; i < 9; i++) if (ts[i] != v[i]) return false;
    return true;
  }
  void reset() {
    cr = 0xC0000000;
    sr = 0;
    key_state = 0;
    busy = 0;
    pbuf.clear();
    for (auto &t : ts) t = 0;            // not loaded: the routine has to set them
  }
  bool locked() const { return cr & 0x80000000u; }
  void start_op() {
    if (!timing_ok()) bad_timing_ops++;
    busy = 3;
    sr |= 1;                             // EOP (when the op ends; good enough for polling)
  }

  // returns false for a bus fault
  bool read(uint32_t a, uint32_t *v) {
    if (a >= 0x08000000 && a + 4 <= 0x08000000 + flash_size) { memcpy(v, &flash[a - 0x08000000], 4); return true; }
    if (a >= 0x20000000 && a + 4 <= 0x20000000 + sizeof ram) { memcpy(v, &ram[a - 0x20000000], 4); return true; }
    if (a >= 0x1FFF0080 && a < 0x1FFF0090) { *v = optbytes[(a - 0x1FFF0080) / 4]; return true; }
    if (a >= 0x1FFF0100 && a < 0x1FFF0180) { *v = factory[(a - 0x1FFF0100) / 4]; return true; }
    if (a >= 0x1FFF0000 && a < 0x1FFF0080) { *v = 0x12345678 + a; return true; }       // UID
    switch (a) {
      case 0x40015800: *v = idcode; return true;
      case 0x40021004: *v = icscr; return true;
      case 0x40022010:
        *v = sr | (busy > 0 ? 1u << 16 : 0);
        if (busy > 0) busy--;
        return true;
      case 0x40022014: *v = cr; return true;
      case 0x40022020: *v = optr; return true;
      case 0xE000ED00: *v = 0x410CC601; return true;
      case 0xE000EDF0: *v = dhcsr | (halted ? 1u << 17 : 0); return true;
      case 0xE000EDFC: *v = demcr; return true;
    }
    if (a >= 0x40022100 && a <= 0x40022120) { *v = ts[(a - 0x40022100) / 4]; return true; }
    return false;
  }

  bool write(uint32_t a, uint32_t v) {
    if (a >= 0x08000000 && a < 0x08000000 + flash_size) return flash_write(a, v);
    if (a >= 0x20000000 && a + 4 <= 0x20000000 + sizeof ram) { memcpy(&ram[a - 0x20000000], &v, 4); return true; }
    if (a >= 0x1FFF0000 && a < 0x1FFF0300) { option_writes++; return false; }
    if (a >= 0x40022100 && a <= 0x40022120) { if (!locked()) ts[(a - 0x40022100) / 4] = v; return true; }
    switch (a) {
      case 0x40003000: if (v == 0xAAAA) return true; iwdg_writes++; return true;
      case 0x40003004: iwdg_pr = v; iwdg_writes++; return true;
      case 0x40003008: iwdg_rlr = v; iwdg_writes++; return true;
      case 0x40022008:                   // KEYR
        if (key_state == 0 && v == 0x45670123) key_state = 1;
        else if (key_state == 1 && v == 0xCDEF89AB) { key_state = 2; cr &= ~0x80000000u; }
        else key_state = 3;              // wrong key: locked until reset
        return true;
      case 0x4002200C: option_writes++; return true;   // OPTKEYR
      case 0x40022020: option_writes++; return true;   // OPTR
      case 0x40022010: sr &= ~(v & 0x11); return true; // EOP, WRPERR: write 1 to clear
      case 0x40022014:
        if (locked()) { if (v & 0x80000000u) cr |= 0x80000000u; return true; }
        if (v & (1u << 17)) option_writes++;          // OPTSTRT
        if ((v & 1) && !(cr & 1)) { pbuf.clear(); pgstrt_early = false; }
        if ((v & (1u << 19)) && pbuf.size() < 31) pgstrt_early = true;
        cr = v;
        if (v & 0x80000000u) key_state = 0;
        return true;
      case 0xE000EDF0:
        dhcsr = v & 0xFFFF;
        if (v & 2) halted = true;
        else if ((v & 0xFFFF0000u) == 0xA05F0000u && !(v & 2)) halted = false;
        return true;
      case 0xE000EDFC: demcr = v; return true;
      case 0xE000ED0C:
        if ((v & 0xFFFF0004u) == 0x05FA0004u) {
          reset();
          halted = demcr & 1;
        }
        return true;
    }
    return false;
  }

  bool flash_write(uint32_t a, uint32_t v) {
    uint32_t off = a - 0x08000000, page = off & ~127u;
    if (locked() || !(cr & 7)) { stray_flash_writes++; return false; }
    if (cr & 4) {                                  // MER
      start_op();
      std::fill(flash.begin(), flash.end(), 0xFF);
      mass_erases++;
      return true;
    }
    if (cr & 2) {                                  // PER
      start_op();
      if (page == wrp_page) { sr |= 1u << 4; return true; }
      std::fill(flash.begin() + page, flash.begin() + page + 128, 0xFF);
      erases++;
      return true;
    }
    // PG: page buffer
    if (pbuf.empty()) pbuf_page = page;
    if (page != pbuf_page || off != page + 4 * pbuf.size()) { pg_errors++; return true; }
    pbuf.push_back(v);
    if (pbuf.size() == 32) {
      if (!(cr & (1u << 19)) || pgstrt_early) { pg_errors++; pbuf.clear(); return true; }
      start_op();
      if (page == wrp_page) { sr |= 1u << 4; pbuf.clear(); return true; }
      for (int i = 0; i < 32; i++) {
        uint32_t w;
        memcpy(&w, &flash[page + 4 * i], 4);
        w &= pbuf[i];
        memcpy(&flash[page + 4 * i], &w, 4);
      }
      programs++;
      pbuf.clear();
    }
    return true;
  }
};

static Chip chip;

// ---------------------------------------------------------------- swd.h stubs on the model
int swd_mem_read(uint8_t, uint32_t addr, uint8_t *out, int len) {
  for (int i = 0; i < len; i += 4) {
    uint32_t v;
    if (!chip.read(addr + i, &v)) return SWD_FAULT;
    memcpy(out + i, &v, 4);
  }
  return SWD_OK;
}
int swd_mem_write(uint8_t, uint32_t addr, const uint8_t *data, int len) {
  for (int i = 0; i < len; i += 4) {
    uint32_t v;
    memcpy(&v, data + i, 4);
    if (!chip.write(addr + i, v)) return SWD_FAULT;
  }
  return SWD_OK;
}
int swd_mem_write16(uint8_t, uint32_t, uint16_t) { chip.half_writes++; return SWD_FAULT; }
int swd_power_up() { return SWD_OK; }
int swd_ap_read(uint8_t ap, uint8_t addr, uint32_t *v) { *v = (ap == 0 && addr == 0xFC) ? 0x04770031 : 0; return SWD_OK; }
int swd_ap_write(uint8_t, uint8_t, uint32_t) { return SWD_OK; }
static uint32_t core_regs[20];
int swd_core_reg_read(uint8_t r, uint32_t *v) { *v = core_regs[r % 20]; return SWD_OK; }
int swd_core_reg_write(uint8_t r, uint32_t v) { core_regs[r % 20] = v; return SWD_OK; }
const char *swd_ack_name(int a) { return a == SWD_FAULT ? "FAULT" : a == SWD_OK ? "OK" : "other"; }

// ---------------------------------------------------------------- self tests
static int fails = 0;
static void check(bool ok, const char *what) {
  printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}

static uint32_t rnd() {
  static uint32_t x = 2463534242u;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  return x;
}

static void self_tests() {
  swdf_set_family(SWDF_PY32);
  check(swdf_halt(true) == SWD_OK && chip.halted, "halt at the reset vector");
  // erase and write the whole flash in page pieces, some pages in two halves
  std::vector<uint8_t> img(chip.flash_size);
  for (auto &b : img) b = rnd() & 0xFF;
  bool ok = true;
  for (uint32_t a = 0; a < chip.flash_size; a += 128) ok &= swdf_erase_page(0x08000000 + a) == SWD_OK;
  for (uint32_t a = 0; a < chip.flash_size; a += 128) {
    if ((a / 128) % 3 == 1) {
      ok &= swdf_write(0x08000000 + a, &img[a], 64) == SWD_OK;
      ok &= swdf_write(0x08000000 + a + 64, &img[a + 64], 64) == SWD_OK;
    } else {
      ok &= swdf_write(0x08000000 + a, &img[a], 128) == SWD_OK;
    }
  }
  check(ok && chip.flash == img, "erase and program 18 k in 128-byte pages (also half pages)");
  check(chip.bad_timing_ops == 0, "timing registers loaded from the factory bytes before every operation");
  check(chip.pg_errors == 0 && chip.stray_flash_writes == 0, "page buffer: 31 words, PGSTRT, the 32nd word");
  // a partial write in the middle of an erased page leaves the rest erased
  swdf_erase_page(0x08000100);
  uint8_t eight[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  swdf_write(0x08000110, eight, 8);
  bool rest = true;
  for (int i = 0; i < 128; i++) {
    uint8_t want = (i >= 0x10 && i < 0x18) ? eight[i - 0x10] : 0xFF;
    rest &= chip.flash[0x100 + i] == want;
  }
  check(rest, "a part of a page: the other bytes stay erased");
  check(swdf_write(0x08000170, eight, 32) == SWDF_ERR_ADDRESS, "a write across a page boundary is refused");
  // mass erase
  check(swdf_mass_erase() == SWD_OK && chip.mass_erases == 1 &&
            std::all_of(chip.flash.begin(), chip.flash.end(), [](uint8_t b) { return b == 0xFF; }),
        "mass erase");
  // the other HSI range uses the second timing set
  chip.icscr = (chip.icscr & ~0xE000u) | (5u << 13);
  int before = chip.bad_timing_ops;
  swdf_erase_page(0x08000000);
  check(chip.bad_timing_ops == before, "timing set chosen by HSI_FS (0x1FFF0130 for FS = 5)");
  chip.icscr = (chip.icscr & ~0xE000u) | (4u << 13);
  // write protection
  chip.wrp_page = 0x200;
  check(swdf_erase_page(0x08000200) == SWDF_ERR_PROGRAM, "write-protected page: WRPERR reported");
  chip.wrp_page = 0xFFFFFFFF;
  // missing factory timing
  uint32_t save = chip.factory[7];
  chip.factory[7] = 0xFFFFFFFF;
  check(swdf_erase_page(0x08000000) == SWDF_ERR_TIMING, "no factory timing: refused");
  chip.factory[7] = save;
  // wrong key: locked until reset
  swdf_halt(true);
  chip.write(0x40022008, 0x12345678);
  check(swdf_erase_page(0x08000000) == SWDF_ERR_LOCKED, "locked controller after a wrong key");
  swdf_halt(true);
  check(swdf_erase_page(0x08000000) == SWD_OK, "a reset unlocks it again");
  // a watchdog started by the option bytes gets the longest time-out
  chip.optr &= ~(1u << 12);
  swdf_erase_page(0x08000000);
  check(chip.iwdg_pr == 6 && chip.iwdg_rlr == 0xFFF, "hardware watchdog: 0xFFF / 256 set before the operation");
  chip.optr |= 1u << 12;
  check(chip.option_writes == 0, "option bytes, OPTKEYR, OPTR, OPTSTRT never written");
  check(chip.half_writes == 0, "no half-word writes");
}

// ---------------------------------------------------------------- serve: the firmware's SWD commands, as text
static uint32_t crc32_update(uint32_t crc, const uint8_t *d, int n) {
  crc = ~crc;
  while (n--) {
    crc ^= *d++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
  }
  return ~crc;
}

static std::string b64(const uint8_t *d, int n) {
  static const char *t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string s;
  for (int i = 0; i < n; i += 3) {
    uint32_t v = d[i] << 16 | (i + 1 < n ? d[i + 1] << 8 : 0) | (i + 2 < n ? d[i + 2] : 0);
    s += t[v >> 18 & 63];
    s += t[v >> 12 & 63];
    s += i + 1 < n ? t[v >> 6 & 63] : '=';
    s += i + 2 < n ? t[v & 63] : '=';
  }
  return s;
}

static int hexbytes(const char *h, uint8_t *out, int max) {
  int n = 0;
  while (h[0] && h[1] && n < max) {
    unsigned v;
    sscanf(h, "%2x", &v);
    out[n++] = (uint8_t)v;
    h += 2;
  }
  return n;
}

static void serve() {
  char line[8192];
  static uint8_t buf[4096];
  while (fgets(line, sizeof line, stdin)) {
    static char cmd[16], a[64], b[4096], c[4096];
    cmd[0] = a[0] = b[0] = c[0] = 0;
    int n = sscanf(line, "%15s %63s %4095s %4095s", cmd, a, b, c);
    (void)n;
    uint32_t a1 = strtoul(a, NULL, 0), a2 = strtoul(b, NULL, 0);
    auto swd = [](int r) { if (r != SWD_OK) { printf("ERR swd %s\n", swdf_err_name(r)); return false; } return true; };
    std::string s = cmd;
    if (s == "SC") printf("OK 0bc11477\n");
    else if (s == "SAR") { uint32_t v; swd_ap_read(a1, a2, &v); printf("OK %08x\n", v); }
    else if (s == "SM") {
      uint32_t len = strtoul(c, NULL, 0);
      if (swd(swd_mem_read(0, a2, buf, len))) printf("OK %s\n", b64(buf, len).c_str());
    } else if (s == "SMW") {
      int k = hexbytes(c, buf, sizeof buf);
      if (swd(swd_mem_write(0, a2, buf, k))) printf("OK\n");
    } else if (s == "SFAM") {
      if (std::string(a) == "py32") { swdf_set_family(SWDF_PY32); printf("OK\n"); }
      else printf("ERR unknown family\n");
    } else if (s == "SHALT") { if (swd(swdf_halt(std::string(a) == "reset"))) printf("OK\n"); }
    else if (s == "SRUN") { if (swd(swdf_run())) printf("OK\n"); }
    else if (s == "SFE") { if (swd(swdf_erase_page(a1))) printf("OK\n"); }
    else if (s == "SFX") { if (swd(swdf_mass_erase())) printf("OK\n"); }
    else if (s == "SFW") {
      int k = hexbytes(b, buf, sizeof buf);
      if (swd(swdf_write(a1, buf, k))) printf("OK\n");
    } else if (s == "SFC") {
      if (swd(swd_mem_read(0, a1, buf, a2))) printf("OK %08x\n", crc32_update(0, buf, a2));
    } else if (s == "SIM") {
      std::string what = a;
      if (what == "stats")
        printf("OK %d %d %d %d %d %d\n", chip.option_writes, chip.bad_timing_ops, chip.pg_errors, chip.stray_flash_writes,
               chip.mass_erases, chip.halted ? 1 : 0);
      else if (what == "idcode") { chip.idcode = a2; printf("OK\n"); }
      else printf("ERR sim\n");
    } else printf("ERR unknown command\n");
    fflush(stdout);
  }
}

int main(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "--serve")) {
    serve();
    return 0;
  }
  self_tests();
  printf("%s\n", fails ? "FAILED" : "all passed");
  return fails ? 1 : 0;
}
