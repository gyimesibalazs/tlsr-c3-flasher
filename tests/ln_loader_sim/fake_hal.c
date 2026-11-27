// hal.h for the native build of the LN882H loader (loaders/ln882h/loader.c, flash.c): every call goes to a function
// that the Python test sets with sim_hooks (ctypes), where the UART bytes and the SPI NOR flash are modelled.
#include <stdint.h>
#include <string.h>
#include "hal.h"

typedef uint32_t (*read_fn)(uint8_t *buf, uint32_t max);
typedef void (*write_fn)(const uint8_t *buf, uint32_t n);
typedef int (*spi_fn)(const uint8_t *tx, uint32_t ntx, uint8_t *rx, uint32_t nrx);
typedef int (*baud_fn)(uint32_t baud);
typedef void (*event_fn)(int what);

static read_fn cb_read;
static write_fn cb_write;
static spi_fn cb_spi;
static baud_fn cb_baud;
static event_fn cb_event;
static uint32_t cycles, cycles_step = 208333;

void sim_hooks(read_fn r, write_fn w, spi_fn s, baud_fn b, event_fn e) {
  cb_read = r;
  cb_write = w;
  cb_spi = s;
  cb_baud = b;
  cb_event = e;
}

// what the ROM left as in the unicorn test (tests/test_ln_emu.py): crystal, APB0 40 MHz, UART0 at 115200
void hal_start(struct ln_regs *r, struct ln_clock *clk, const char *hello) {
  memset(r, 0xFF, sizeof(*r));
  r->uart_brr = 5555;
  r->uart_cr1 = 0x802D;
  r->uart_cr2 = 0;
  r->uart_cr3 = 0;
  r->clk_gate = 0x00000410;
  r->io_en = 0;
  r->func_en = 0x0C;
  r->func_sel0 = 0x03020000;
  r->cmp_div = 0;
  r->clk_sel = 0;
  r->awo_div = 0;
  r->pll_ctrl = 0x8008;
  r->pll_lock = 0x200;
  r->pmu_reg = 0;
  r->vset = 0;
  r->gpioa_ddr = 0;
  r->vtor = 0;
  r->sp = 0x20047F00;
  r->ctrl = 0;
  clk->core_hz = 160000000;
  clk->apb_hz = 40000000;
  clk->pll = 1;
  cb_write((const uint8_t *)hello, (uint32_t)strlen(hello));
  cb_event(0);
}

uint32_t hal_uart_read(uint8_t *buf, uint32_t max) { return cb_read(buf, max); }
void hal_uart_write(const uint8_t *buf, uint32_t n) { cb_write(buf, n); }
void hal_uart_drain(void) { cb_event(1); }
int hal_uart_set_baud(uint32_t baud) { return cb_baud(baud); }
uint32_t hal_uart_errors(void) { return 0; }

// each call 1.3 ms of a 160 MHz core by default: the 15 banner characters at 115200 between two calls, the wait line
// every 154 calls; the test may make time pass faster
void sim_cycles_step(uint32_t step) {
  cycles_step = step;
}

uint32_t hal_cycles(void) {
  cycles += cycles_step;
  return cycles & 0xFFFFFF;
}

void hal_flash_cache_off(struct ln_regs *r) {
  r->cache_en = 1;
  cb_event(2);
}

void hal_flash_take(struct ln_regs *r) {
  r->qspi_ctrlr0 = 0x00070000;
  r->qspi_baudr = 2;
  cb_event(3);
}

int hal_spi(const uint8_t *tx, uint32_t ntx, uint8_t *rx, uint32_t nrx) { return cb_spi(tx, ntx, rx, nrx); }
void hal_spin(uint32_t n) { (void)n; }

void hal_reboot(void) { cb_event(9); }
