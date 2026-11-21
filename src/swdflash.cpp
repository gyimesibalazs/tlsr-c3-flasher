// Flash programming over SWD. Register maps from the Silicon Labs device headers (efr32mg1p_msc.h,
// efr32mg21_msc.h, efr32mg22_msc.h, efr32mg22_cmu.h) and the Nordic nRF52 NVMC description.
#include <Arduino.h>
#include "swd.h"
#include "swdflash.h"

static SwdFlashFamily fam = SWDF_NONE;
static uint32_t msc_clk_bit = 0;             // EFR32 Series 2: MSC clock in CMU_CLKEN1 (0 = none)
static uint32_t st_page = 1024, st_bank2 = 0, l0_base = 0x40022000;

// Cortex-M debug registers
static const uint32_t DHCSR = 0xE000EDF0, DEMCR = 0xE000EDFC, AIRCR = 0xE000ED0C;

// EFR32 MSC
struct Msc { uint32_t base, writectrl, writecmd, addrb, wdata, status, lock; };
static const Msc MSC_S1 = {0x400E0000, 0x08, 0x0C, 0x10, 0x18, 0x1C, 0x40};
static const Msc MSC_S2 = {0x40030000, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x3C};
static const uint32_t MSC_S1_MASSLOCK = 0x54;
static const uint32_t MSC_UNLOCK = 0x1B71, MSC_MASS_UNLOCK = 0x631A;
static const uint32_t WREN = 1;
static const uint32_t CMD_LADDRIM = 1 << 0, CMD_ERASEPAGE = 1 << 1, CMD_WRITEEND = 1 << 2, CMD_WRITEONCE = 1 << 3,
                      CMD_ERASEMAIN0 = 1 << 8;
static const uint32_t ST_BUSY = 1 << 0, ST_LOCKED = 1 << 1, ST_INVADDR = 1 << 2, ST_WDATAREADY = 1 << 3;
static const uint32_t CMU_S_CLKEN1_SET = 0x40008000 + 0x1068;

// nRF52 NVMC and CTRL-AP
static const uint32_t NVMC_READY = 0x4001E400, NVMC_CONFIG = 0x4001E504, NVMC_ERASEPAGE = 0x4001E508;
static const uint8_t NRF_CTRL_AP = 1;

static int rd(uint32_t addr, uint32_t *v) { return swd_mem_read(0, addr, (uint8_t *)v, 4); }
static int wr(uint32_t addr, uint32_t v) { return swd_mem_write(0, addr, (const uint8_t *)&v, 4); }

#define TRY(x)                      \
  do {                              \
    int r_ = (x);                   \
    if (r_ != SWD_OK) return r_;    \
  } while (0)

void swdf_set_family(SwdFlashFamily f, uint32_t p1, uint32_t p2) {
  fam = f;
  msc_clk_bit = 0;
  st_bank2 = 0;
  if (f == SWDF_EFR32S2) {
    msc_clk_bit = p1;
  } else if (f == SWDF_STM32L0) {
    l0_base = p1 ? p1 : 0x40022000;
  } else {
    if (p1) st_page = p1;
    st_bank2 = p2;
  }
}

const char *swdf_err_name(int r) {
  switch (r) {
    case SWDF_ERR_TIMEOUT: return "flash controller timeout";
    case SWDF_ERR_LOCKED: return "flash locked";
    case SWDF_ERR_ADDRESS: return "invalid flash address";
    case SWDF_ERR_FAMILY: return "no flash family set (SFAM)";
    case SWDF_ERR_HALT: return "core does not halt";
    case SWDF_ERR_PROGRAM: return "flash programming error (write protected, read protected or not erased)";
    default: return swd_ack_name(r);
  }
}

int swdf_halt(bool reset) {
  TRY(wr(DHCSR, 0xA05F0003));                // DBGKEY | C_HALT | C_DEBUGEN
  if (reset) {
    TRY(wr(DEMCR, 1));                       // VC_CORERESET: stop at the reset vector
    TRY(wr(AIRCR, 0x05FA0004));              // SYSRESETREQ
    delay(20);
    TRY(swd_power_up());                     // the reset can drop the debug power request on some parts
  }
  for (int i = 0; i < 50; i++) {
    uint32_t v = 0;
    TRY(rd(DHCSR, &v));
    if (v & (1 << 17)) {                     // S_HALT
      if (reset) wr(DEMCR, 0);
      return SWD_OK;
    }
    delay(2);
  }
  return SWDF_ERR_HALT;
}

int swdf_run() {
  TRY(wr(DEMCR, 0));
  TRY(wr(DHCSR, 0xA05F0000));                // debug off, core runs
  return wr(AIRCR, 0x05FA0004);              // and starts from reset (the debug port answers on, the AP may not)
}

static int msc_wait(const Msc &m, uint32_t mask, uint32_t want, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    uint32_t st = 0;
    TRY(rd(m.base + m.status, &st));
    if ((st & mask) == want) return SWD_OK;
    if (millis() - t0 > timeout_ms) return SWDF_ERR_TIMEOUT;
  }
}

static int msc_check(const Msc &m) {
  uint32_t st = 0;
  TRY(rd(m.base + m.status, &st));
  if (st & ST_LOCKED) return SWDF_ERR_LOCKED;
  if (st & ST_INVADDR) return SWDF_ERR_ADDRESS;
  return SWD_OK;
}

static int msc_prepare(const Msc &m, uint32_t addr, bool s1) {
  if (msc_clk_bit) TRY(wr(CMU_S_CLKEN1_SET, 1u << msc_clk_bit));
  TRY(wr(m.base + m.lock, MSC_UNLOCK));
  TRY(wr(m.base + m.writectrl, WREN));
  TRY(wr(m.base + m.addrb, addr));
  if (s1) TRY(wr(m.base + m.writecmd, CMD_LADDRIM));
  return msc_check(m);
}

static int nvmc_wait(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    uint32_t r = 0;
    TRY(rd(NVMC_READY, &r));
    if (r & 1) return SWD_OK;
    if (millis() - t0 > timeout_ms) return SWDF_ERR_TIMEOUT;
  }
}

// ---------------------------------------------------------------- STM32
// Register maps from the reference manuals: RM0008 (F1), RM0091 (F0), RM0316 (F3), RM0090 (F4),
// RM0444 (G0), RM0440 (G4), RM0351 (L4), RM0434 (WB).
struct StFlash { uint32_t base, keyr, sr, cr, bsy, errors, lock; };
static const StFlash ST_V1 = {0x40022000, 0x04, 0x0C, 0x10, 1u << 0, (1u << 2) | (1u << 4), 1u << 7};
static const StFlash ST_F4 = {0x40023C00, 0x04, 0x0C, 0x10, 1u << 16, 0xF2, 1u << 31};
static const StFlash ST_V2 = {0x40022000, 0x08, 0x10, 0x14, 1u << 16, 0x3FA, 1u << 31};
// U5 (RM0456): the non-secure registers NSKEYR / NSSR / NSCR, quad-word programming
static const StFlash ST_U5 = {0x40022000, 0x08, 0x20, 0x28, 1u << 16, 0xFA, 1u << 31};
static const uint32_t ST_KEY1 = 0x45670123, ST_KEY2 = 0xCDEF89AB, ST_FLASH = 0x08000000;
static const uint32_t F4_PSIZE32 = 2u << 8;

static const StFlash &st() {
  return fam == SWDF_STM32F1 ? ST_V1 : fam == SWDF_STM32F4 ? ST_F4 : fam == SWDF_STM32U5 ? ST_U5 : ST_V2;
}

static int st_unlock(const StFlash &f) {
  uint32_t cr = 0;
  TRY(rd(f.base + f.cr, &cr));
  if (cr & f.lock) {
    TRY(wr(f.base + f.keyr, ST_KEY1));
    TRY(wr(f.base + f.keyr, ST_KEY2));
    TRY(rd(f.base + f.cr, &cr));
    if (cr & f.lock) return SWDF_ERR_LOCKED;
  }
  return wr(f.base + f.sr, f.errors | 1);    // clear old error flags and EOP (write 1 to clear)
}

static int st_wait(const StFlash &f, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    uint32_t sr = 0;
    TRY(rd(f.base + f.sr, &sr));
    if (!(sr & f.bsy)) return (sr & f.errors) ? (int)SWDF_ERR_PROGRAM : (int)SWD_OK;
    if (millis() - t0 > timeout_ms) return SWDF_ERR_TIMEOUT;
  }
}

// F4: 16 k x 4, 64 k, 128 k ...; the second bank of the 2 Mbyte parts has sector numbers 16 + n
static uint32_t f4_sector(uint32_t addr) {
  uint32_t off = addr - ST_FLASH, bank = 0;
  if (st_bank2 && addr >= st_bank2) {
    off = addr - st_bank2;
    bank = 16;
  }
  uint32_t n = off < 0x10000 ? off / 0x4000 : off < 0x20000 ? 4 : 5 + (off - 0x20000) / 0x20000;
  return bank + n;
}

static int st_erase(uint32_t addr) {
  const StFlash &f = st();
  TRY(st_unlock(f));
  uint32_t cr;
  if (fam == SWDF_STM32F1) {
    TRY(wr(f.base + f.cr, 1u << 1));         // PER
    TRY(wr(f.base + 0x14, addr));            // AR
    cr = (1u << 1) | (1u << 6);              // PER | STRT
  } else if (fam == SWDF_STM32F4) {
    cr = (1u << 1) | (f4_sector(addr) << 3) | F4_PSIZE32;   // SER | SNB | PSIZE
    TRY(wr(f.base + f.cr, cr));
    cr |= 1u << 16;                          // STRT
  } else {
    uint32_t bker = 0, pg = (addr - ST_FLASH) / st_page;
    if (st_bank2 && addr >= st_bank2) {
      bker = 1u << 11;
      pg = (addr - st_bank2) / st_page;
    }
    cr = (1u << 1) | (pg << 3) | bker;       // PER | PNB | BKER
    TRY(wr(f.base + f.cr, cr));
    cr |= 1u << 16;                          // STRT
  }
  TRY(wr(f.base + f.cr, cr));
  int r = st_wait(f, 4000);                  // an F4 128 k sector takes up to 2 s
  wr(f.base + f.cr, 0);
  return r;
}

static int st_write(uint32_t addr, const uint8_t *data, int len) {
  const StFlash &f = st();
  TRY(st_unlock(f));
  int r = SWD_OK;
  if (fam == SWDF_STM32F1) {
    TRY(wr(f.base + f.cr, 1u << 0));        // PG: half-word bus writes
    for (int i = 0; i < len && r == SWD_OK; i += 2) {
      uint16_t h = data[i] | (data[i + 1] << 8);
      r = swd_mem_write16(0, addr + i, h);
      if (r == SWD_OK) r = st_wait(f, 20);
    }
  } else if (fam == SWDF_STM32F4) {
    TRY(wr(f.base + f.cr, (1u << 0) | F4_PSIZE32));
    for (int i = 0; i < len && r == SWD_OK; i += 4) {
      r = swd_mem_write(0, addr + i, data + i, 4);
      if (r == SWD_OK) r = st_wait(f, 20);
    }
  } else if (fam == SWDF_STM32U5) {
    if (len % 16 || addr % 16) return SWDF_ERR_ADDRESS;  // quad words
    TRY(wr(f.base + f.cr, 1u << 0));        // PG
    for (int i = 0; i < len && r == SWD_OK; i += 16) {
      r = swd_mem_write(0, addr + i, data + i, 16);
      if (r == SWD_OK) r = st_wait(f, 20);
    }
  } else {
    if (len % 8 || addr % 8) return SWDF_ERR_ADDRESS;   // double words
    TRY(wr(f.base + f.cr, 1u << 0));        // PG
    for (int i = 0; i < len && r == SWD_OK; i += 8) {
      r = swd_mem_write(0, addr + i, data + i, 8);
      if (r == SWD_OK) r = st_wait(f, 20);
    }
  }
  wr(f.base + f.cr, 0);
  return r;
}

static int st_mass_erase() {
  const StFlash &f = st();
  TRY(st_unlock(f));
  uint32_t cr = fam == SWDF_STM32F1   ? (1u << 2)
                : fam == SWDF_STM32F4 ? (1u << 2) | (st_bank2 ? 1u << 15 : 0) | F4_PSIZE32
                                      : (1u << 2) | (st_bank2 ? 1u << 15 : 0);   // MER (+ second bank)
  TRY(wr(f.base + f.cr, cr));
  TRY(wr(f.base + f.cr, cr | (fam == SWDF_STM32F1 ? 1u << 6 : 1u << 16)));      // STRT
  int r = st_wait(f, 40000);
  wr(f.base + f.cr, 0);
  return r;
}

static bool is_stm32() {
  return fam == SWDF_STM32F1 || fam == SWDF_STM32F4 || fam == SWDF_STM32V2 || fam == SWDF_STM32U5;
}

// ---------------------------------------------------------------- STM32 L0 / L1 (RM0377 / RM0038: PECR)
static const uint32_t L0_PECR = 0x04, L0_PEKEYR = 0x0C, L0_PRGKEYR = 0x10, L0_SR = 0x18;
static const uint32_t L0_ERRORS = 0x33F00;   // WRPERR .. RDERR, NOTZEROERR, FWWERR

static int l0_unlock() {
  uint32_t pecr = 0;
  TRY(rd(l0_base + L0_PECR, &pecr));
  if (pecr & 1) {                            // PELOCK
    TRY(wr(l0_base + L0_PEKEYR, 0x89ABCDEF));
    TRY(wr(l0_base + L0_PEKEYR, 0x02030405));
  }
  TRY(rd(l0_base + L0_PECR, &pecr));
  if (pecr & 2) {                            // PRGLOCK
    TRY(wr(l0_base + L0_PRGKEYR, 0x8C9DAEBF));
    TRY(wr(l0_base + L0_PRGKEYR, 0x13141516));
  }
  TRY(rd(l0_base + L0_PECR, &pecr));
  if (pecr & 3) return SWDF_ERR_LOCKED;
  return wr(l0_base + L0_SR, L0_ERRORS);
}

static int l0_wait(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    uint32_t sr = 0;
    TRY(rd(l0_base + L0_SR, &sr));
    if (!(sr & 1)) return (sr & L0_ERRORS) ? (int)SWDF_ERR_PROGRAM : (int)SWD_OK;
    if (millis() - t0 > timeout_ms) return SWDF_ERR_TIMEOUT;
  }
}

static int l0_erase(uint32_t addr) {
  TRY(l0_unlock());
  TRY(wr(l0_base + L0_PECR, (1u << 9) | (1u << 3)));   // ERASE | PROG: writing a word erases the page
  int r = wr(addr, 0);
  if (r == SWD_OK) r = l0_wait(100);
  wr(l0_base + L0_PECR, 1);                  // PELOCK
  return r;
}

static int l0_write(uint32_t addr, const uint8_t *data, int len) {
  TRY(l0_unlock());
  int r = SWD_OK;
  for (int i = 0; i < len && r == SWD_OK; i += 4) {
    r = swd_mem_write(0, addr + i, data + i, 4);
    if (r == SWD_OK) r = l0_wait(20);
  }
  wr(l0_base + L0_PECR, 1);
  return r;
}

// ---------------------------------------------------------------- nRF5340 application core (NVMC secure)
static const uint32_t N53_NVMC = 0x50039000, N53_READY = N53_NVMC + 0x400, N53_CONFIG = N53_NVMC + 0x504;
static const uint8_t N53_CTRL_AP = 2;        // application core CTRL-AP

static int n53_wait(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    uint32_t r = 0;
    TRY(rd(N53_READY, &r));
    if (r & 1) return SWD_OK;
    if (millis() - t0 > timeout_ms) return SWDF_ERR_TIMEOUT;
  }
}

// ---------------------------------------------------------------- Microchip SAM D21 / D51 (NVMCTRL)
static const uint32_t NVMCTRL = 0x41004000, DSU = 0x41002000;

static int rd16(uint32_t addr, uint16_t *v) {
  uint32_t w = 0;
  TRY(rd(addr & ~3u, &w));
  *v = (uint16_t)(w >> (8 * (addr & 2)));
  return SWD_OK;
}

// SAM D21: CTRLA 0x00 (command), CTRLB 0x04, INTFLAG 0x14 (READY, ERROR), STATUS 0x18, ADDR 0x1C (half-words)
static int d21_cmd(uint8_t cmd) {
  TRY(swd_mem_write16(0, NVMCTRL + 0x00, 0xA500 | cmd));
  uint32_t t0 = millis();
  for (;;) {
    uint16_t f = 0;
    TRY(rd16(NVMCTRL + 0x14, &f));
    if (f & 1) {
      uint16_t st = 0;
      TRY(rd16(NVMCTRL + 0x18, &st));
      if ((f & 2) || (st & 0x1C)) {          // ERROR / PROGE, LOCKE, NVME
        swd_mem_write16(0, NVMCTRL + 0x18, 0x1C);
        return (st & 0x08) ? SWDF_ERR_LOCKED : SWDF_ERR_PROGRAM;
      }
      return SWD_OK;
    }
    if (millis() - t0 > 200) return SWDF_ERR_TIMEOUT;
  }
}

// SAM D51: CTRLA 0x00, CTRLB 0x04 (command), INTFLAG 0x10, STATUS 0x12 (READY), ADDR 0x14 (bytes)
static int d51_ready(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    uint16_t st = 0;
    TRY(rd16(NVMCTRL + 0x12, &st));
    if (st & 1) return SWD_OK;
    if (millis() - t0 > timeout_ms) return SWDF_ERR_TIMEOUT;
  }
}

static int d51_cmd(uint8_t cmd, uint32_t timeout_ms) {
  TRY(d51_ready(100));
  TRY(swd_mem_write16(0, NVMCTRL + 0x04, 0xA500 | cmd));
  TRY(d51_ready(timeout_ms));
  uint16_t f = 0;
  TRY(rd16(NVMCTRL + 0x10, &f));
  if (f & 0x4E) {                            // ADDRE, PROGE, LOCKE, NVME
    swd_mem_write16(0, NVMCTRL + 0x10, 0x4F);
    return (f & 0x08) ? SWDF_ERR_LOCKED : SWDF_ERR_PROGRAM;
  }
  return SWD_OK;
}

// DSU chip erase: also clears the security bit; works when the device is protected
static int samd_chip_erase() {
  TRY(swd_mem_write16(0, DSU + 0x00, 0x1F00));         // clear STATUSA (write 1 to clear)
  TRY(swd_mem_write16(0, DSU + 0x00, 0x0010));         // CTRL.CE
  uint32_t t0 = millis();
  for (;;) {
    uint32_t w = 0;
    TRY(rd(DSU, &w));
    if ((w >> 8) & 0x0C) return SWDF_ERR_PROGRAM;   // STATUSA.BERR / FAIL
    if ((w >> 8) & 1) return SWD_OK;         // STATUSA.DONE
    if (millis() - t0 > 10000) return SWDF_ERR_TIMEOUT;
    delay(10);
  }
}

int swdf_call(uint32_t pc, uint32_t sp, uint32_t lr, const uint32_t *args, int nargs, uint32_t timeout_ms,
              uint32_t *r0) {
  TRY(wr(DHCSR, 0xA05F000B));                // halted, interrupts masked while the function runs
  for (int i = 0; i < nargs && i < 4; i++) TRY(swd_core_reg_write(i, args[i]));
  TRY(swd_core_reg_write(13, sp));
  TRY(swd_core_reg_write(14, lr | 1));
  TRY(swd_core_reg_write(15, pc & ~1u));
  TRY(swd_core_reg_write(16, 0x01000000));   // xPSR: Thumb
  TRY(wr(DHCSR, 0xA05F0009));                // run, interrupts still masked
  uint32_t t0 = millis();
  for (;;) {
    uint32_t v = 0;
    TRY(rd(DHCSR, &v));
    if (v & (1 << 17)) break;                // halted at the breakpoint
    if (millis() - t0 > timeout_ms) {
      wr(DHCSR, 0xA05F000B);
      return SWDF_ERR_TIMEOUT;
    }
    delay(1);
  }
  return swd_core_reg_read(0, r0);
}

int swdf_erase_page(uint32_t addr) {
  if (is_stm32()) return st_erase(addr);
  if (fam == SWDF_STM32L0) return l0_erase(addr);
  if (fam == SWDF_NRF53) {                   // nRF53: a page is erased by writing 0xFFFFFFFF into it
    TRY(wr(N53_CONFIG, 2));
    int r = wr(addr, 0xFFFFFFFF);
    if (r == SWD_OK) r = n53_wait(200);
    wr(N53_CONFIG, 0);
    return r;
  }
  if (fam == SWDF_SAMD21) {
    TRY(wr(NVMCTRL + 0x1C, addr / 2));       // row containing addr
    return d21_cmd(0x02);                    // ER
  }
  if (fam == SWDF_SAMD51) {
    uint16_t ctrla = 0;
    TRY(rd16(NVMCTRL + 0x00, &ctrla));
    TRY(swd_mem_write16(0, NVMCTRL + 0x00, ctrla & ~0x30));   // WMODE = manual
    TRY(wr(NVMCTRL + 0x14, addr));
    return d51_cmd(0x01, 500);               // EB: erase the 8 k block
  }
  if (fam == SWDF_EFR32S1 || fam == SWDF_EFR32S2) {
    const Msc &m = fam == SWDF_EFR32S1 ? MSC_S1 : MSC_S2;
    TRY(msc_prepare(m, addr, fam == SWDF_EFR32S1));
    TRY(wr(m.base + m.writecmd, CMD_ERASEPAGE));
    int r = msc_wait(m, ST_BUSY, 0, 200);
    if (r != SWD_OK) return r;
    return msc_check(m);
  }
  if (fam == SWDF_NRF52) {
    TRY(wr(NVMC_CONFIG, 2));                 // erase enable
    TRY(wr(NVMC_ERASEPAGE, addr));
    int r = nvmc_wait(200);
    wr(NVMC_CONFIG, 0);
    return r;
  }
  return SWDF_ERR_FAMILY;
}

int swdf_write(uint32_t addr, const uint8_t *data, int len) {
  if (is_stm32()) return st_write(addr, data, len);
  if (fam == SWDF_STM32L0) return l0_write(addr, data, len);
  if (fam == SWDF_NRF53) {
    TRY(wr(N53_CONFIG, 1));
    int r = swd_mem_write(0, addr, data, len);
    if (r == SWD_OK) r = n53_wait(20);
    wr(N53_CONFIG, 0);
    return r;
  }
  if (fam == SWDF_SAMD21) {                  // one page: page buffer, then WP
    uint32_t ctrlb = 0;
    TRY(rd(NVMCTRL + 0x04, &ctrlb));
    TRY(wr(NVMCTRL + 0x04, ctrlb | 0x80));   // MANW: write only on the WP command
    TRY(d21_cmd(0x44));                      // PBC
    TRY(swd_mem_write(0, addr, data, len));
    TRY(wr(NVMCTRL + 0x1C, addr / 2));
    return d21_cmd(0x04);                    // WP
  }
  if (fam == SWDF_SAMD51) {
    uint16_t ctrla = 0;
    TRY(rd16(NVMCTRL + 0x00, &ctrla));
    TRY(swd_mem_write16(0, NVMCTRL + 0x00, ctrla & ~0x30));
    TRY(d51_cmd(0x15, 50));                  // PBC
    TRY(swd_mem_write(0, addr, data, len));
    TRY(wr(NVMCTRL + 0x14, addr));
    return d51_cmd(0x03, 50);                // WP
  }
  if (fam == SWDF_EFR32S1) {
    // one word at a time: load the address, WDATA, WRITEONCE, wait (as the Series 1 MSC_WriteWord does)
    const Msc &m = MSC_S1;
    for (int i = 0; i < len; i += 4) {
      TRY(msc_prepare(m, addr + i, true));
      uint32_t w;
      memcpy(&w, data + i, 4);
      TRY(msc_wait(m, ST_WDATAREADY, ST_WDATAREADY, 20));
      TRY(wr(m.base + m.wdata, w));
      TRY(wr(m.base + m.writecmd, CMD_WRITEONCE));
      TRY(msc_wait(m, ST_BUSY, 0, 20));
    }
    return msc_check(m);
  }
  if (fam == SWDF_EFR32S2) {
    // Series 2: the address auto-increments while WDATA is fed, WRITEEND closes the sequence
    const Msc &m = MSC_S2;
    TRY(msc_prepare(m, addr, false));
    for (int i = 0; i < len; i += 4) {
      uint32_t w;
      memcpy(&w, data + i, 4);
      TRY(msc_wait(m, ST_WDATAREADY, ST_WDATAREADY, 20));
      TRY(wr(m.base + m.wdata, w));
    }
    TRY(wr(m.base + m.writecmd, CMD_WRITEEND));
    TRY(msc_wait(m, ST_BUSY, 0, 50));
    return msc_check(m);
  }
  if (fam == SWDF_NRF52) {
    TRY(wr(NVMC_CONFIG, 1));                 // write enable; the bus stalls each word until it is written
    int r = swd_mem_write(0, addr, data, len);
    if (r == SWD_OK) r = nvmc_wait(20);
    wr(NVMC_CONFIG, 0);
    return r;
  }
  return SWDF_ERR_FAMILY;
}

int swdf_mass_erase() {
  if (is_stm32()) return st_mass_erase();
  if (fam == SWDF_SAMD21 || fam == SWDF_SAMD51) return samd_chip_erase();
  if (fam == SWDF_NRF53) {
    TRY(swd_ap_write(N53_CTRL_AP, 0x04, 1));
    uint32_t t0 = millis();
    for (;;) {
      uint32_t st = 1;
      TRY(swd_ap_read(N53_CTRL_AP, 0x08, &st));
      if (st == 0) break;
      if (millis() - t0 > 10000) return SWDF_ERR_TIMEOUT;
      delay(10);
    }
    return swd_ap_write(N53_CTRL_AP, 0x04, 0);
  }
  if (fam == SWDF_EFR32S1) {
    const Msc &m = MSC_S1;
    TRY(msc_prepare(m, 0, true));
    TRY(wr(m.base + MSC_S1_MASSLOCK, MSC_MASS_UNLOCK));
    TRY(wr(m.base + m.writecmd, CMD_ERASEMAIN0));
    int r = msc_wait(m, ST_BUSY, 0, 2000);
    wr(m.base + MSC_S1_MASSLOCK, 0);
    return r;
  }
  if (fam == SWDF_NRF52) {
    // CTRL-AP ERASEALL: works with APPROTECT on as well, and clears it
    TRY(swd_ap_write(NRF_CTRL_AP, 0x04, 1));
    uint32_t t0 = millis();
    for (;;) {
      uint32_t st = 1;
      TRY(swd_ap_read(NRF_CTRL_AP, 0x08, &st));
      if (st == 0) break;
      if (millis() - t0 > 5000) return SWDF_ERR_TIMEOUT;
      delay(10);
    }
    return swd_ap_write(NRF_CTRL_AP, 0x04, 0);
  }
  return SWDF_ERR_FAMILY;                    // Series 2: page by page from the host (no unlocked mass erase)
}
