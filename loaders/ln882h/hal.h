// What the loader needs from the chip. ln882h.c implements it on the LN882H's registers; the tests implement it on a
// model of an SPI NOR flash and a byte queue (tests/ln_loader_sim/fake_hal.c), so loader.c and flash.c build for both.
#pragma once
#include <stdint.h>

// ---- what the ROM left (read before anything is changed; for the info answer and the first-run diagnosis)
struct ln_regs {
  uint32_t uart_brr, uart_cr1, uart_cr2, uart_cr3;   // UART0
  uint32_t clk_gate, io_en;                  // SYSC_CMP: peripheral clock gates, pad enables (bit 5: flash pads)
  uint32_t func_en, func_sel0;               // SYSC_CMP: pin function enables, functions of A0..A3 (A2 TX, A3 RX)
  uint32_t cmp_div;                          // SYSC_CMP: APB0 divider - 1 (bits 3..0)
  uint32_t clk_sel, awo_div;                 // SYSC_AWO: clock source (bit 0: PLL), AHB divider - 1 (bits 3..0)
  uint32_t pll_ctrl, pll_lock;               // SYSC_AWO: PLL control (bits 4..0 multiplier, bit 15 on), lock
  uint32_t pmu_reg, vset;                    // SYSC_AWO: band gap trim, crystal settings; LDO voltages
  uint32_t gpioa_ddr;                        // GPIOA directions (1 = output)
  uint32_t cache_en;                         // flash cache: bit 0 on, bit 31 filling
  uint32_t qspi_ctrlr0, qspi_baudr;
  uint32_t vtor, sp;                         // the core as the ROM started the loader: VTOR, SP,
  uint32_t ctrl;                             // and IPSR << 16 | CONTROL << 8 | PRIMASK
};
#define LN_NREGS 22

// ---- the clocks the loader set
struct ln_clock {
  uint32_t core_hz, apb_hz;                  // core (= AHB, the QSPI controller), APB0 (UART0)
  uint32_t pll;                              // 1: from the PLL, 0: from the crystal (the PLL was off)
};

// ---- start: hal_start reads what the ROM left (*rom), says hello on UART0 as the ROM left it (for the log only: it
// may come garbled), sets the clocks (*clk) and sets up UART0 from scratch at 115200.
void hal_start(struct ln_regs *rom, struct ln_clock *clk, const char *hello);

// ---- UART0, polled
uint32_t hal_uart_read(uint8_t *buf, uint32_t max);      // what has come in, without waiting
void hal_uart_write(const uint8_t *buf, uint32_t n);    // waits for room in the transmit FIFO
void hal_uart_drain(void);                              // until the last stop bit has left
int hal_uart_set_baud(uint32_t baud);                   // 0 = switched; the caller drains first
uint32_t hal_uart_errors(void);                         // receive errors seen (framing, noise, overrun)

// ---- a free-running 24-bit cycle counter (SysTick at the core clock, counting up); the caller masks differences
uint32_t hal_cycles(void);

// ---- the flash: hal_flash_cache_off switches the flash cache off, hal_flash_take sets up the QSPI controller for
// plain SPI; both first switch on the clocks they need, and put the cache and QSPI values they found into *rom.
// hal_spi is one transaction with CS low: ntx bytes out (command, address, dummy bytes, data), then nrx bytes in.
void hal_flash_cache_off(struct ln_regs *rom);
void hal_flash_take(struct ln_regs *rom);
int hal_spi(const uint8_t *tx, uint32_t ntx, uint8_t *rx, uint32_t nrx);   // 0, or LN_E_QSPI on a time-out
void hal_spin(uint32_t n);                              // a short busy wait, about n microseconds or more

void hal_reboot(void);
