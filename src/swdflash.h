// Flash programming over SWD, run on the flasher: Silicon Labs EFR32 Series 1 / Series 2, Nordic nRF52 / nRF53,
// STM32, SAM D, Puya PY32F002B type.
#pragma once
#include <stdint.h>

enum SwdFlashFamily {
  SWDF_NONE = 0, SWDF_EFR32S1, SWDF_EFR32S2, SWDF_NRF52, SWDF_NRF53, SWDF_STM32F1, SWDF_STM32F4, SWDF_STM32V2,
  SWDF_STM32U5, SWDF_STM32L0, SWDF_SAMD21, SWDF_SAMD51, SWDF_PY32
};
// errors besides the SWD acknowledges of swd.h
enum { SWDF_ERR_TIMEOUT = -1, SWDF_ERR_LOCKED = -2, SWDF_ERR_ADDRESS = -3, SWDF_ERR_FAMILY = -4, SWDF_ERR_HALT = -5,
       SWDF_ERR_PROGRAM = -6, SWDF_ERR_TIMING = -7 };

// p1 / p2 per family:
//   EFR32 Series 2: p1 = bit of the MSC clock in CMU_CLKEN1 (16 or 17; 0 = none, xG21)
//   STM32 F1 / V2 / U5: p1 = erase page in bytes, p2 = start address of the second bank (0 = none)
//   STM32 F4: p2 = start address of the second bank;  STM32 L0/L1: p1 = flash register base
//   Puya PY32F002B type (MS32C001): no parameters; 128-byte pages
void swdf_set_family(SwdFlashFamily fam, uint32_t p1 = 0, uint32_t p2 = 0);
int swdf_halt(bool reset);               // stop the core (reset = stop it at the reset vector)
int swdf_erase_page(uint32_t addr);
int swdf_write(uint32_t addr, const uint8_t *data, int len);   // word aligned, within one page
int swdf_mass_erase();
int swdf_run();                          // leave debug halt and reset the target
// run a function on the halted core: registers set, returns at the breakpoint at lr; r0 = result
int swdf_call(uint32_t pc, uint32_t sp, uint32_t lr, const uint32_t *args, int nargs, uint32_t timeout_ms,
              uint32_t *r0);
const char *swdf_err_name(int r);
