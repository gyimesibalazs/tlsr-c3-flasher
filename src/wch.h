// WCH single-wire debug interface (SDI, "SWIO") of the CH32V003: line layer on the RMT peripheral and the
// debug module interface (DMI) register access on top of it. See wch.cpp for the protocol.
#pragma once
#include <stdint.h>
#include "driver/rmt_types.h"

enum {
  WCH_OK = 0,
  WCH_ERR_DRIVER = 1,      // RMT channel could not be set up / transmit failed
  WCH_ERR_NOPULSE = 2,     // the line did not go low at all: driven high by the target, or the wire is open
  WCH_ERR_STUCKLOW = 3,    // the line stayed low: target unpowered, or the pin is driven low
  WCH_ERR_FRAME = 4,       // wrong number of pulses, or our own pulses came back distorted
  WCH_ERR_NOANSWER = 5,    // the line works but nothing answers (reads all ones)
  WCH_ERR_BUSY = 6,        // abstract command still busy
  WCH_ERR_CMD = 7,         // abstract command error (cmderr != 0), see wchf_cmderr()
  WCH_ERR_NOTHALTED = 8,   // the core did not halt
  WCH_ERR_TIMEOUT = 9,     // flash controller stayed busy
  WCH_ERR_WRPRT = 10,      // the flash controller reported a write protection error
  WCH_ERR_LOCKED = 11,     // the flash controller did not unlock
  WCH_ERR_ADDRESS = 12,    // address / length not allowed for this operation
  WCH_ERR_VERIFY = 13,     // option bytes read back differently
  WCH_ERR_NOTACTIVE = 14,  // WCH mode is not on
};

// Line timing in nanoseconds (defaults from config.h).
struct WchTiming {
  uint32_t one_low, zero_low, high, read_slot, read_threshold, stop;
};

bool wch_begin();                        // set up the RMT channels on the SWS wire (other modes torn down first)
void wch_end();                          // release the channels, both pins inputs
bool wch_active();
void wch_set_timing(const WchTiming &t);
WchTiming wch_timing();

int wch_dmi_write(uint8_t addr, uint32_t value);   // queued: returns once the packet is on its way
int wch_dmi_read(uint8_t addr, uint32_t *value);
void wch_line_reset();                   // low for WCH_LINE_RESET_US, then high
void wch_line_hold_low();                // keep the line low (while the target is switched off)
void wch_line_release();                 // back to the idle high level

// Attach: optional power cycle (power != NULL and off_ms > 0), then the handshake (enable the debug output,
// halt request, read dmstatus) repeated for up to window_ms. On success the core is halted.
struct WchAttach {
  uint32_t dmstatus, cpbr;
  uint32_t tries;
  uint32_t first_us;                     // time from power-on (or the start) to the first answer
  bool halted, powered;
  // what the failed tries saw
  uint32_t n_nopulse, n_stucklow, n_frame, n_ones, n_other;
};
int wch_attach(void (*power)(bool on), uint32_t off_ms, uint32_t window_ms, WchAttach *res);
// one line explaining a failed attach
const char *wch_attach_reason(const WchAttach &a);

const char *wch_err_name(int r);
// details of the last line error (pulse counts, durations)
const char *wch_last_detail();

// Pure encoding / decoding, also used by the tests. Durations in RMT ticks (12.5 ns).
int wch_encode_write(uint8_t addr, uint32_t value, const WchTiming &t, rmt_symbol_word_t *out);   // 41 symbols
int wch_encode_read(uint8_t addr, const WchTiming &t, rmt_symbol_word_t *out);                    // 41 symbols
// decode what the RX pin saw during a read packet (host pulses and the target's answer)
int wch_decode_read(const rmt_symbol_word_t *in, int n, const WchTiming &t, uint32_t *value);
