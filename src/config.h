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

// ---- UART bridge
// RX is the SWS wire (GPIO 5, GPIO 4 is switched to an input), TX has its own wire. GPIO 21 also carries
// the C3's own boot messages for a moment after the flasher is plugged in.
#define PIN_BRIDGE_TX 21
// BOOT / strapping pin of the target, held at the level given to the BRIDGE command while the target starts
#define PIN_BOOT 0

// ---- SPI NOR flash, on the wires above: no new pin
#define PIN_SPI_CS PIN_RST        // chip select = the reset wire
#define PIN_SPI_CLK PIN_SWCLK     // clock = the SWCLK wire
#define PIN_SPI_DI PIN_SWS_TX     // flash DI (MOSI) = the SWS wire, driven through its resistor
#define PIN_SPI_DO PIN_BRIDGE_TX  // flash DO (MISO) = the UART bridge TX wire, an input here
#define SPI_KHZ_DEFAULT 4000
