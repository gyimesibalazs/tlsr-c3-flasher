// Wiring of the flasher (ESP32-C3 boards with the C3's own USB port). Avoid the strapping pins (2, 8, 9)
// and USB (18, 19); UART0 (20, 21) is free for the target because the host link is the USB port.
#pragma once

// SWS line: PIN_SWS_RX goes straight to the target's SWS pad,
// PIN_SWS_TX goes to the same pad through a 1 k ... 1.8 k resistor.
#define PIN_SWS_TX 4
#define PIN_SWS_RX 5

// Target reset (active low, open drain). Connect to the target's RST pad.
#define PIN_RST 6

// Optional target power (push-pull, strongest drive): connect the target's 3.3 V pad here instead of
// the 3V3 pin when the target has no reset pad, or when its firmware switches SWS off right after start.
// Every activation then switches the target off and on, and catches it while it starts.
// One pin gives about 40 mA at its limit; the pins of the list switch at the same instant, so several
// of them can be joined on the target's 3.3 V pad to share the current. Unused ones stay unconnected.
#define PWR_PINS {7, 10, 3, 1}
#define PWR_OFF_MS 300

// At power-up of the flasher: try to stop a target that got its power together with the flasher
// (3.3 V pad on the 3V3 pin, no reset wire). 0 = off.
#define AUTO_ACTIVATE_MS 3000

// Mode at power-up: 0 = UART emulation (pvvx's method), 1 = RMT (real pulse timing, faster).
#define SWS_MODE_DEFAULT 1

// UART mode: one SWire bit = 10 UART bits, so the SWire bit rate is baud / 10. Range 340000 ... 5000000.
#define SWS_BAUD_DEFAULT 921600

// RMT mode: SWire bit rate in bit/s. Range 30000 ... 2000000 (the chip's own default is about 960000).
#define SWS_RATE_DEFAULT 1000000

// ---- SWD (Arm Serial Wire Debug: EFR32, nRF52, STM32 ...)
// SWDIO is the SWS wire (GPIO 4 drives it through the resistor, GPIO 5 reads it); SWCLK has its own wire.
#define PIN_SWCLK 0
// SWD clock in kHz at power-up (SWCLK command); the bit-banged clock reaches about 2000.
#define SWD_KHZ_DEFAULT 1000

// ---- four-wire JTAG: TCK = SWCLK wire, TDI = SWS wire (through the resistor), TDO = GPIO 21, TMS its own wire
#define PIN_JTAG_TMS 20
#define PIN_JTAG_TDO 21

// ---- UART bridge
// RX is the SWS wire (GPIO 5, GPIO 4 is switched to an input), TX has its own wire. GPIO 21 also carries
// the C3's own boot messages for a moment after the flasher is plugged in.
#define PIN_BRIDGE_TX 21
// BOOT / strapping pin of the target, held at the level given to the BRIDGE command while the target starts
#define PIN_BOOT 0

// ---- SPI NOR flash, on the wires above: no new pin
#define PIN_SPI_CS PIN_JTAG_TMS   // chip select = the JTAG TMS wire (GPIO 20); the reset wire holds the board's
                                  // own processor in reset meanwhile, so that it keeps off the flash
#define PIN_SPI_CLK PIN_SWCLK     // clock = the SWCLK wire
#define PIN_SPI_DI PIN_SWS_TX     // flash DI (MOSI) = the SWS wire, driven through its resistor
#define PIN_SPI_DO PIN_BRIDGE_TX  // flash DO (MISO) = the UART bridge TX wire, an input here
#define SPI_KHZ_DEFAULT 4000

// ---- WCH CH32V003 single-wire debug (SDI, the "SWIO" pin), on the SWS wire: GPIO 4 drives it through the
// resistor (the line idles high through it), GPIO 5 reads it. The RMT peripheral times the pulses (12.5 ns).
// Bit timing of the debug interface (QingKe V2 debug manual, normal "2x" mode after reset, T = 125 ns):
//   1: low T..4T, high T..16T;  0: low 6T..64T, high T..16T;  stop: high for 18T or longer.
// The defaults sit inside those windows with margin on both sides (WCH's own reference host uses about
// 232 / 739 / 274 ns; the T values are those of the chip's 8 MHz debug clock).
#define WCH_ONE_LOW_NS 250        // low part of a 1 bit
#define WCH_ZERO_LOW_NS 1000      // low part of a 0 bit
#define WCH_HIGH_NS 375           // high part of every bit the host sends
#define WCH_READ_SLOT_NS 1500     // one bit read from the target: our short low pulse plus the target's answer
#define WCH_READ_THRESHOLD_NS 550 // a read bit is 0 when the line stayed low longer than this
#define WCH_STOP_NS 4000          // high after every packet (the stop condition is 18T = 2.25 us)
#define WCH_LINE_RESET_US 100     // low for this long resets the debug interface (256T = 32 us are needed)
// Catch at power-up: the power pins are switched off for WCH_OFF_MS, then the handshake is repeated from the
// first microsecond after power returns, for up to WCH_CATCH_MS, so that the core is halted before the
// firmware can turn the SWIO pin into something else (on the SOP8 CH32V003J4M6 pin 8 is also UART TX).
#define WCH_OFF_MS 300
#define WCH_CATCH_MS 100
