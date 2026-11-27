// The LN882H side of hal.h: the clocks, UART0 and its pins, the QSPI flash controller (a DesignWare SSI), the flash
// cache, the restart, and main(). Only the registers the loader uses are defined here; addresses and bit meanings are
// from the chip's SDK (OpenLN882H, read for the facts only) and agree with what Lightning's own RAM loader binary
// does at its start. The loader takes nothing over from the ROM: it sets the clocks and UART0 itself, as Lightning's
// loader does, so that the UART divider is computed from a clock that is known.
#include <stdint.h>
#include <string.h>
#include "hal.h"
#include "loader.h"

#define REG(a) (*(volatile uint32_t *)(a))

// SYSC_CMP: clock dividers of the peripheral bus, clock gates, resets, pads and pin functions
#define CMP_DIV_UPDATE REG(0x40000000u)  // bit 0: take the APB0 divider, bit 5: take the RF interface divider
#define CMP_DIV_PARA REG(0x40000004u)    // bits 3..0: APB0 divider - 1, bit 4: RF interface divider - 1
#define CMP_CLK_GATE REG(0x40000010u)    // 1 = clock on
#define CMP_SW_RSTN REG(0x40000014u)     // 0 = held in reset
#define CMP_IO_EN REG(0x40000018u)       // bit 5: the flash pads to the QSPI controller
#define CMP_FUNC_EN REG(0x4000001Cu)     // bits 0..12: A0..A12 driven by the function chosen in FUNC_SEL
#define CMP_FUNC_SEL0 REG(0x40000020u)   // 6-bit function of A0 (bits 5..0), A1 (13..8), A2 (21..16), A3 (29..24)
#define GATE_QSPI (1u << 1)
#define GATE_GPIOA (1u << 4)
#define GATE_UART0 (1u << 10)
#define GATE_CACHE (1u << 22)
#define RSTN_UART0 (1u << 10)
#define FUNC_UART0_TX 2u
#define FUNC_UART0_RX 3u
#define PIN_A2 (1u << 2)
#define PIN_A3 (1u << 3)

// SYSC_AWO: the clock source, the PLL, the AHB divider, analog settings, the chip reset
#define AWO_DIV_UPDATE REG(0x40100000u)  // bit 0: take the AHB divider
#define AWO_DIV_PARA REG(0x40100004u)    // bits 3..0: AHB divider - 1
#define AWO_SW_RSTN REG(0x4010000Cu)     // bit 4: reset of the whole chip, low active
#define AWO_CLK_SEL REG(0x40100010u)     // bit 0: 1 = the PLL, 0 = the 40 MHz crystal
#define AWO_PMU_REG REG(0x40100014u)     // bits 9..8: band gap trim
#define AWO_PLL_CTRL REG(0x40100070u)    // bits 4..0: multiplier in halves (core = 40 MHz * m / 2), bit 15: PLL on
#define AWO_PLL_LOCK REG(0x40100074u)
#define AWO_VSET REG(0x4010009Cu)        // bits 14..12: LDO15_1 voltage (the PLL's supply)
#define PLL_ON (1u << 15)
#define XTAL_HZ 40000000u

// GPIOA
#define GPIOA_DDR REG(0x4000C018u)       // bits 15..0: 1 = output

// UART0: an STM32-like USART with FIFOs (the offsets and bits agree with the SDK's register description)
#define UART0 0x40002000u
#define U_CR1 REG(UART0 + 0x00)
#define U_CR2 REG(UART0 + 0x04)
#define U_CR3 REG(UART0 + 0x08)
#define U_BRR REG(UART0 + 0x0C)
#define U_ISR REG(UART0 + 0x1C)
#define U_ICR REG(UART0 + 0x20)
#define U_RDR REG(UART0 + 0x24)
#define U_TDR REG(UART0 + 0x28)
#define U_FIFO REG(UART0 + 0x2C)
#define CR1_UE (1u << 0)
#define CR1_RE (1u << 2)
#define CR1_TE (1u << 3)
#define CR1_OVER8 (1u << 15)             // 8x oversampling: BRR = 16 * APB0 clock / baud (the SDK's formula)
#define ISR_ERRORS 0x07u                 // PE, FE, NF
#define ISR_ORE 0x08u
#define ISR_RXNE 0x20u
#define ISR_TC 0x40u
#define ISR_TXFULL 0x1000000u
#define ICR_ERRORS 0x07u                 // PECF, FECF, NCF
#define ICR_TCCF 0x40u
#define ICR_ALL 0x120257u                // PECF, FECF, NCF, IDLECF, TCCF, CTSCF, CMCF, WUCF
#define FIFO_TX_LEVEL(v) (((v) >> 16) & 0x1F)

// the flash cache (XIP through the QSPI controller)
#define CACHE_EN REG(0x20200000u)        // bit 0 enable, bit 31 a line fill in progress

// the QSPI controller: DesignWare SSI, clocked from the AHB
#define QSPI 0x40200000u
#define Q_CTRLR0 REG(QSPI + 0x00)
#define Q_CTRLR1 REG(QSPI + 0x04)
#define Q_SSIENR REG(QSPI + 0x08)
#define Q_SER REG(QSPI + 0x10)
#define Q_BAUDR REG(QSPI + 0x14)
#define Q_SR REG(QSPI + 0x28)
#define Q_IMR REG(QSPI + 0x2C)
#define Q_DR REG(QSPI + 0x60)
#define Q_RX_SAMPLE_DLY REG(QSPI + 0xF0)
#define SR_BUSY 0x01u
#define SR_TFNF 0x02u
#define SR_TFE 0x04u
#define SR_RFNE 0x08u
#define CTRLR0_8BIT (7u << 16)           // standard SPI, Motorola, mode 0, 8-bit frames
#define TMOD_TX (1u << 8)
#define TMOD_EEPROM (3u << 8)            // the bytes written are sent, then CTRLR1 + 1 bytes are received
#define QSPI_SCK_HZ 10000000u            // the SPI clock for commands: the SDK's flash routines use AHB / 16 at 160 MHz
#define SPIN_LIMIT 1000000u

// the core
#define SYST_CSR REG(0xE000E010u)
#define SYST_RVR REG(0xE000E014u)
#define SYST_CVR REG(0xE000E018u)
#define NVIC_ICER0 REG(0xE000E180u)
#define NVIC_ICER1 REG(0xE000E184u)
#define NVIC_ICPR0 REG(0xE000E280u)
#define NVIC_ICPR1 REG(0xE000E284u)
#define SCB_AIRCR REG(0xE000ED0Cu)

extern uint32_t rom_vtor, rom_sp, rom_ctrl;   // saved by the start-up code
static uint32_t core_hz = 160000000u, apb_hz = XTAL_HZ;
static uint32_t rx_errors, ore_seen;

static void pause(uint32_t turns) {
  for (volatile uint32_t k = turns; k; k--) {
  }
}

// ---------------------------------------------------------------- start
// The clocks as Lightning's loader sets them (its SetSysClock): the band gap trim and the PLL's LDO first, then the
// PLL as the source with the multiplier 8 (core and AHB 160 MHz), the AHB undivided, APB0 / 4 = 40 MHz. The PLL is
// running when the ROM hands over (Lightning's loader switches to it without starting it); if its enable bit reads 0
// the crystal is kept with APB0 undivided, which also gives 40 MHz on APB0.
static void set_clocks(struct ln_clock *clk) {
  AWO_PMU_REG = (AWO_PMU_REG & ~(3u << 8)) | (1u << 8);
  AWO_VSET = (AWO_VSET & ~(7u << 12)) | (7u << 12);
  int pll = (AWO_PLL_CTRL & PLL_ON) != 0;
  if (pll) {
    AWO_CLK_SEL |= 1u;
    AWO_PLL_CTRL = (AWO_PLL_CTRL & ~0x1Fu) | 8u;
    CMP_DIV_PARA = (CMP_DIV_PARA & ~0xFu) | (4u - 1);
  } else {
    AWO_CLK_SEL &= ~1u;
    CMP_DIV_PARA &= ~0xFu;
  }
  AWO_DIV_PARA &= ~0xFu;
  CMP_DIV_PARA &= ~(1u << 4);             // the RF interface at 40 MHz, as Lightning's loader leaves it
  AWO_DIV_UPDATE |= 1u;
  CMP_DIV_UPDATE |= 1u;
  CMP_DIV_UPDATE |= 1u << 5;
  pause(100);
  clk->pll = (uint32_t)pll;
  clk->core_hz = core_hz = pll ? XTAL_HZ * 8 / 2 : XTAL_HZ;
  clk->apb_hz = apb_hz = XTAL_HZ;
}

// UART0 from its reset state: 8N1, 8x oversampling, no interrupts, no DMA, no flow control; then A2 / A3 given to it.
static void uart_init(void) {
  CMP_CLK_GATE |= GATE_GPIOA | GATE_UART0;
  pause(20);
  CMP_SW_RSTN &= ~RSTN_UART0;
  pause(20);
  CMP_SW_RSTN |= RSTN_UART0;
  pause(20);
  U_CR1 = CR1_OVER8;
  U_CR2 = 0;
  U_CR3 = 0;
  hal_uart_set_baud(115200);
  U_ICR = ICR_ALL;
  U_CR1 = CR1_OVER8 | CR1_RE | CR1_TE;
  U_CR1 = CR1_OVER8 | CR1_RE | CR1_TE | CR1_UE;
  CMP_FUNC_SEL0 = (CMP_FUNC_SEL0 & ~(0x3Fu << 16 | 0x3Fu << 24)) | FUNC_UART0_TX << 16 | FUNC_UART0_RX << 24;
  GPIOA_DDR &= ~(PIN_A2 | PIN_A3);       // the GPIO side an input, as the SDK sets a pin it gives to a function
  CMP_FUNC_EN |= PIN_A2 | PIN_A3;
  rx_errors = ore_seen = 0;
}

void hal_start(struct ln_regs *r, struct ln_clock *clk, const char *hello) {
  NVIC_ICER0 = NVIC_ICER1 = 0xFFFFFFFFu;  // no interrupt enabled or pending (PRIMASK is set as well)
  NVIC_ICPR0 = NVIC_ICPR1 = 0xFFFFFFFFu;
  SYST_CSR = 0;
  SYST_RVR = 0xFFFFFFu;
  SYST_CVR = 0;
  SYST_CSR = 5;                          // enabled, core clock, no interrupt

  memset(r, 0xFF, sizeof(*r));           // all ones: not read (that block's clock was off)
  r->clk_gate = CMP_CLK_GATE;
  r->io_en = CMP_IO_EN;
  r->func_en = CMP_FUNC_EN;
  r->func_sel0 = CMP_FUNC_SEL0;
  r->cmp_div = CMP_DIV_PARA;
  r->clk_sel = AWO_CLK_SEL;
  r->awo_div = AWO_DIV_PARA;
  r->pll_ctrl = AWO_PLL_CTRL;
  r->pll_lock = AWO_PLL_LOCK;
  r->pmu_reg = AWO_PMU_REG;
  r->vset = AWO_VSET;
  if (r->clk_gate & GATE_UART0) {
    r->uart_brr = U_BRR;
    r->uart_cr1 = U_CR1;
    r->uart_cr2 = U_CR2;
    r->uart_cr3 = U_CR3;
  }
  if (r->clk_gate & GATE_GPIOA) r->gpioa_ddr = GPIOA_DDR;
  r->vtor = rom_vtor;
  r->sp = rom_sp;
  r->ctrl = rom_ctrl;

  // as the ROM left UART0 (for the log only); then the clocks and UART0 of our own
  if ((r->clk_gate & GATE_UART0) && (r->uart_cr1 & (CR1_UE | CR1_TE)) == (CR1_UE | CR1_TE)) {
    hal_uart_write((const uint8_t *)hello, (uint32_t)strlen(hello));
    hal_uart_drain();
  }
  set_clocks(clk);
  uart_init();
}

// ---------------------------------------------------------------- UART0
uint32_t hal_uart_read(uint8_t *buf, uint32_t max) {
  uint32_t n = 0;
  while (n < max) {
    uint32_t isr = U_ISR;
    if (isr & ISR_ERRORS) {
      rx_errors++;
      U_ICR = ICR_ERRORS;
    }
    if ((isr & ISR_ORE) && !ore_seen) rx_errors++;
    ore_seen = isr & ISR_ORE;
    if (!(isr & ISR_RXNE)) break;
    buf[n++] = (uint8_t)U_RDR;
  }
  return n;
}

void hal_uart_write(const uint8_t *buf, uint32_t n) {
  U_ICR = ICR_TCCF;                      // TC from now on means: this data has left
  for (uint32_t i = 0; i < n; i++) {
    for (uint32_t k = 0; (U_ISR & ISR_TXFULL) && k < SPIN_LIMIT; k++) {
    }
    U_TDR = buf[i];
  }
}

void hal_uart_drain(void) {
  for (uint32_t k = 0; k < SPIN_LIMIT; k++)
    if (FIFO_TX_LEVEL(U_FIFO) == 0 && (U_ISR & ISR_TC)) return;
}

int hal_uart_set_baud(uint32_t baud) {
  // 8x oversampling: BRR = 16 * APB0 / baud (the SDK's hal_uart_baudrate_set, rounded here instead of truncated)
  uint32_t brr = (16u * apb_hz + baud / 2) / baud;
  if (brr < 16 || brr > 0x7FFFFu) return -1;
  U_BRR = brr;
  return 0;
}

uint32_t hal_uart_errors(void) {
  return rx_errors;
}

// ---------------------------------------------------------------- SysTick as a cycle counter
uint32_t hal_cycles(void) {
  return (0xFFFFFFu - SYST_CVR) & 0xFFFFFFu;
}

// ---------------------------------------------------------------- flash
void hal_flash_cache_off(struct ln_regs *r) {
  CMP_CLK_GATE |= GATE_CACHE | GATE_QSPI;
  pause(20);
  r->cache_en = CACHE_EN;
  for (uint32_t k = 0; (CACHE_EN & 0x80000000u) && k < SPIN_LIMIT; k++) {
  }
  CACHE_EN = CACHE_EN & 0x7FFFFFFEu;
  Q_SSIENR = 0;
}

void hal_flash_take(struct ln_regs *r) {
  CMP_CLK_GATE |= GATE_QSPI;
  CMP_IO_EN |= 1u << 5;
  r->qspi_ctrlr0 = Q_CTRLR0;
  r->qspi_baudr = Q_BAUDR;
  Q_SSIENR = 0;
  Q_IMR = 0;
  Q_SER = 0;
  Q_RX_SAMPLE_DLY = 1;
  uint32_t div = (core_hz / QSPI_SCK_HZ + 1) & ~1u;   // even, as the SSI wants it: 16 at 160 MHz, 4 at 40 MHz
  Q_BAUDR = div < 2 ? 2 : div;
}

static int wait_sr(uint32_t mask, uint32_t want) {
  for (uint32_t k = 0; k < SPIN_LIMIT; k++)
    if ((Q_SR & mask) == want) return 0;
  Q_SSIENR = 0;
  return LN_E_QSPI;
}

int hal_spi(const uint8_t *tx, uint32_t ntx, uint8_t *rx, uint32_t nrx) {
  Q_SSIENR = 0;
  Q_SER = 0;
  if (!nrx) {
    // transmit only: CS stays low while the FIFO has data, so it is kept fed (interrupts are off)
    Q_CTRLR0 = CTRLR0_8BIT | TMOD_TX;
    Q_SSIENR = 1;
    Q_SER = 1;
    for (uint32_t i = 0; i < ntx; i++) {
      if (wait_sr(SR_TFNF, SR_TFNF)) return LN_E_QSPI;
      Q_DR = tx[i];
    }
  } else {
    // EEPROM read: the command bytes (at most 5, the FIFO holds 16), then nrx bytes clocked in
    Q_CTRLR0 = CTRLR0_8BIT | TMOD_EEPROM;
    Q_CTRLR1 = nrx - 1;
    Q_SER = 1;
    Q_SSIENR = 1;
    for (uint32_t i = 0; i < ntx; i++) Q_DR = tx[i];
    if (wait_sr(SR_TFE, SR_TFE)) return LN_E_QSPI;
    for (uint32_t i = 0; i < nrx; i++) {
      if (wait_sr(SR_RFNE, SR_RFNE)) return LN_E_QSPI;
      rx[i] = (uint8_t)Q_DR;
    }
  }
  if (wait_sr(SR_TFE, SR_TFE) || wait_sr(SR_BUSY, 0)) return LN_E_QSPI;
  Q_SSIENR = 0;
  return 0;
}

void hal_spin(uint32_t us) {
  // at least 3 cycles a turn: n microseconds or more
  pause(us * (core_hz / 3000000u));
}

// ---------------------------------------------------------------- the rest
void hal_reboot(void) {
  AWO_SW_RSTN &= ~(1u << 4);
  AWO_SW_RSTN |= 1u << 4;
  hal_spin(1000);
  SCB_AIRCR = 0x05FA0004u;               // if the chip reset did not happen: the core's own
  for (;;) {
  }
}

// An exception (a fault, or an interrupt that should not come): one line on UART0, then stop.
void fault_report(const uint32_t *sp, uint32_t ipsr) {
  static const char hex[] = "0123456789abcdef";
  char line[] = "\r\nC3LN fault xx pc xxxxxxxx lr xxxxxxxx\r\n";
  line[13] = hex[(ipsr >> 4) & 15];
  line[14] = hex[ipsr & 15];
  for (int i = 0; i < 8; i++) {
    line[19 + i] = hex[(sp[6] >> (28 - 4 * i)) & 15];
    line[31 + i] = hex[(sp[5] >> (28 - 4 * i)) & 15];
  }
  hal_uart_write((const uint8_t *)line, sizeof(line) - 1);
  hal_uart_drain();
  for (;;) {
  }
}

int main(void) {
  loader_init();
  for (;;) loader_poll();
}

// ---------------------------------------------------------------- what the compiler may call
void *memset(void *d, int c, size_t n) {
  uint8_t *p = d;
  while (n--) *p++ = (uint8_t)c;
  return d;
}

void *memcpy(void *d, const void *s, size_t n) {
  uint8_t *p = d;
  const uint8_t *q = s;
  while (n--) *p++ = *q++;
  return d;
}

int memcmp(const void *a, const void *b, size_t n) {
  const uint8_t *p = a, *q = b;
  for (; n; n--, p++, q++)
    if (*p != *q) return *p - *q;
  return 0;
}

int strcmp(const char *a, const char *b) {
  while (*a && *a == *b) a++, b++;
  return (uint8_t)*a - (uint8_t)*b;
}

size_t strlen(const char *s) {
  size_t n = 0;
  while (s[n]) n++;
  return n;
}
