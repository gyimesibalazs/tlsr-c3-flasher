// WCH CH32V003 over its single-wire debug interface: debug module (halt, resume, memory through the
// program buffer) and the flash controller (unlock, 64-byte page erase, fast page programming, mass erase,
// option bytes). Runs on the flasher; the line layer is wch.cpp.
#pragma once
#include <stdint.h>

// memory map of the CH32V003 (reference manual; minichlink chips.c)
#define WCH_FLASH_BASE 0x08000000u   // user flash (also seen at 0x00000000), 16 k on the V003
#define WCH_BOOT_BASE 0x1FFFF000u    // boot area, 1920 bytes
#define WCH_BOOT_SIZE 1920u
#define WCH_OPT_BASE 0x1FFFF800u     // option bytes: RDPR, USER, DATA0, DATA1, WRPR0..3 (each with its complement)
#define WCH_PAGE 64u                 // fast erase / programming page

void wchf_set_power(void (*power)(bool on));   // the power pins, for the catch at power-up (NULL = none)

int wchf_halt(bool reset);           // halt the core; reset = system reset with the halt request held
int wchf_resume();                   // leave the halt without a reset
int wchf_reset_run();                // system reset, then run (the firmware may take the SWIO pin again)
int wchf_read32(uint32_t addr, uint32_t *v);
int wchf_write32(uint32_t addr, uint32_t v);
int wchf_mem_read(uint32_t addr, uint8_t *out, int len);          // word aligned, len a multiple of 4
int wchf_mem_write(uint32_t addr, const uint8_t *data, int len);  // RAM and registers (not flash)

int wchf_flash_unlock();             // main flash (and the fast mode keys)
int wchf_flash_erase(uint32_t addr); // the 64-byte page at addr (user flash or boot area)
int wchf_flash_mass_erase();         // the whole user flash
int wchf_flash_write(uint32_t addr, const uint8_t *data, int len);   // inside one erased 64-byte page
int wchf_options_write(const uint8_t ob[16]);                          // erase the option page, write 16 bytes

int wchf_cmderr();                   // cmderr of the last failed abstract command (debug spec numbering)
const char *wchf_err_text(int r);    // error name with the detail of the line or the cmderr

// The W... text commands of the host protocol (see README). Returns false if the command is not one of them.
bool wchf_command(int argc, char **argv, void (*out)(const char *line));
