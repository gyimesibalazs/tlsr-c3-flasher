// The target's UART: TX on GPIO 21 (PIN_BRIDGE_TX), RX on the SWS wire (PIN_SWS_RX). Every mode that talks to the
// target over a UART (the bridge, Beken) goes through here, so all of its traffic is in one place: each byte sent
// and received, speed changes and events are written to the UART log (uartlog.h). A new session (tuart_begin)
// starts a new log; the log of the last session stays readable after tuart_end.
// Not here: the Telink SWS mode's UART emulation, which drives the SWS line bit by bit (main.cpp).
#pragma once
#include <stddef.h>
#include <stdint.h>

bool tuart_begin(uint32_t baud, int rx_buf, int tx_buf, const char *session);
void tuart_end();
bool tuart_on();
uint32_t tuart_baud();

void tuart_write(const uint8_t *d, int n);
bool tuart_wait_tx(uint32_t ms);
size_t tuart_available();
int tuart_read(uint8_t *d, int max, uint32_t timeout_ms);   // the bytes read, 0 when none came in time
void tuart_flush_input();                                     // received bytes nobody asked for: logged, dropped
void tuart_set_baud(uint32_t baud);

// TX released (an input) while the target is unpowered, so that it does not feed the target; attached again after
void tuart_release_tx();
void tuart_attach_tx();

void tuart_event(const char *fmt, ...);
