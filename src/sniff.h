// Two-line UART logger ("sniff"): listens to both directions of a UART link between two other parties (a Tuya Wi-Fi
// module and its MCU, say) without driving anything, into the UART log (uartlog.h) and, while a host watches, into
// the live ring (sniffring.h).
//
// Line A is the SWS wire (UART1 RX on GPIO 5; GPIO 4, the resistor side, an input), line B is PIN_SNIFF_B (UART0 RX,
// no TX pin; config.h). Log records: tag 2 = line A, tag 1 = line B, 3 = events; the session text is
// "sniff, A <baud>, B <baud>". A record's time is when its bytes were taken from the UART driver (see sniff.cpp for
// the resolution), so a request on one line and its answer on the other keep their order.
// Pin states while it runs (main.cpp sets the rest): reset wire, BOOT wire and power pins inputs, nothing driven.
#pragma once
#include <stdint.h>
#include "sniffring.h"

// live_mem: the live ring's memory (main.cpp lends a buffer of a mode that is off meanwhile), NULL: no live output
bool sniff_begin(uint32_t baud_a, uint32_t baud_b, uint8_t *live_mem, uint32_t live_size);
void sniff_end(const char *why);              // why: an event for the log, or NULL; the live ring is kept to drain
bool sniff_on();
uint32_t sniff_baud_a();
uint32_t sniff_baud_b();

void sniff_live_off();                        // the live output ends (the capture may go on)
bool sniff_live_on();
SniffRing &sniff_ring();

// takes what both lines received into the log (and the live ring); true when anything came
bool sniff_capture();
void sniff_event(const char *fmt, ...);       // into the log and the live output

// capture at power-up (NVS): both 0 = off
void sniff_boot_get(uint32_t *baud_a, uint32_t *baud_b);
void sniff_boot_set(uint32_t baud_a, uint32_t baud_b);
