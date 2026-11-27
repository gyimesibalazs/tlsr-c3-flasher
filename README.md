# ESP32-C3 flasher: Telink SWS, Arm SWD, cJTAG, JTAG, SPI flash, WCH SWIO, Beken, LN882H and Realtek UART, ESP32 / ESP8266

An ESP32-C3 board with a few wires becomes a flasher for the chips found on small smart-home boards:

- Telink TLSR825x over its single-wire debug pin (SWS)
- Arm Cortex-M chips over SWD or four-wire JTAG, TI CC13xx / CC26xx over two-wire cJTAG
- 25-series SPI NOR flash chips
- WCH CH32V003 over its single-wire debug pin (SWIO)
- Beken BK7231N / BK7231T / BK7238, Lightning LN882H, Realtek RTL8710B and RTL8720C through their UART download modes
- ESP32 / ESP8266 through their ROM download mode, with esptool

It also works as a USB–UART bridge, as a logger that listens to both directions of a UART link between two other
chips, and it can be reached over Wi-Fi instead of USB. All modes share the same few wires; one mode is active at a
time. A Python script, `tlsr_c3.py`, drives it from the computer.

| Mode | Option | Targets | Tested on hardware |
|---|---|---|---|
| [Telink SWS](#telink-tlsr825x-over-sws) | (default) | TLSR8251 / 8253 / 8258 (Tuya ZT2S, ZTU, …) | yes: TLSR8258 on five kinds of boards |
| [Arm SWD](#arm-swd) | `--swd` | EFR32, nRF52 / nRF5340, STM32, SAM D / E, RP2040, GD32 / CH32 (F1 compatible), Puya PY32F002B / MS32C001 | yes: Puya MS32C001; the other chips no (simulated only) |
| [cJTAG](#ti-cc13xx--cc26xx-over-two-wire-cjtag) | `--cjtag` | TI CC13xx / CC26xx | no |
| [JTAG](#four-wire-jtag) | `--jtag` | chain scan; Arm chips behind a JTAG-DP | no (simulated only) |
| [SPI flash](#spi-nor-flash-eg-a-routers) | `--spi` | 25-series SPI NOR (routers, dongles, modules) | no (simulated only) |
| [WCH SWIO](#wch-ch32v003-over-swio) | `--wch` | WCH CH32V003 | no (simulated only) |
| [Beken UART](#beken-bk72xx-cb2s-cb3s-wb2s--over-uart) | `--bk` | BK7231N (Tuya CB2S, CB3S, …), BK7231T (WB2S, WB3S, …), BK7238 (Tuya T1) | yes: CB2S (BK7231N); BK7231T and BK7238 no |
| [ESP32 / ESP8266](#esp32--esp8266-over-uart) | `--esp` | ESP32, ESP32-S2 / S3 / C2 / C3 / C6 / H2, ESP8266 | no (simulated only) |
| [LN882H UART](#lightning-ln882h-over-uart) | `--ln` | Lightning LN882H (newer Tuya Wi-Fi / BLE modules) | yes: LN-02 module, both RAM loaders |
| [Realtek AmebaZ UART](#realtek-amebaz-rtl8710b-over-uart) | `--rtl` | RTL8710BN / BX (Tuya WR2, WR3, …) | no (simulated only) |
| [Realtek AmebaZ2 UART](#realtek-amebaz2-rtl8720c-over-uart) | `--rtl2` | RTL8720CF, RTL8710CX / CM (Tuya WBR2, WBR3, CR3L, …) | no (simulated only) |
| [UART bridge](#uart-bridge) | `bridge` | any 3.3 V UART, with a BOOT pin | yes, as part of the LN882H mode |
| [UART log](#uart-log) | `log` | what went over the target's UART in the last session | yes, in LN882H sessions |
| [Two-line UART logger](#two-line-uart-logger) | `sniff` | both directions of a UART link between two other chips (e.g. a Tuya module and its MCU) | no (simulated only) |
| [Wi-Fi link](#wi-fi-link) | `--host`, `wifi` | the flasher itself, over TCP instead of USB | no (simulated only) |

"Simulated only" means the mode was written from vendor documentation and reference tools and checked against
models of the chip, not on a real one. Expect problems on the first real chip and always keep a backup.

## Board, build and upload

Use an ESP32-C3 board whose USB connector goes straight to the C3 (ESP32-C3 Super Mini, Seeed XIAO ESP32C3, the
USB port of a DevKitM). Boards with a separate USB–UART chip (CH340, CP2102) cannot be used: the C3's UART0
(GPIO 20 / 21) belongs to the target.

Either flash a ready image, `c3-flasher-firmware-<version>.factory.bin` (bootloader, partition table and firmware in
one file), at address 0 with esptool or a web flasher:

    esptool --chip esp32c3 write-flash 0x0 c3-flasher-firmware-0.31.factory.bin

or build and upload with PlatformIO (Arduino core 3.x / ESP-IDF 5 is needed):

    pio run -e c3-usb -t upload

If the board does not show up as a serial port for the upload, hold its BOOT button while plugging it in.

On the computer, install pyserial (and esptool for the ESP mode):

    pip install pyserial
    pip install esptool        # only for --esp

Use firmware 0.31 with the current `tlsr_c3.py`. The script checks the firmware version and refuses a mode the
firmware is too old for:

| Needs firmware ≥ | |
|---|---|
| 0.18 | everything (Telink, SWD, cJTAG, `bridge`, `--esp`) |
| 0.19 / 0.21 / 0.22 | JTAG / SPI flash / WCH |
| 0.24 / 0.26 / 0.27 / 0.28 | Beken / LN882H / RTL8710B / RTL8720C |
| 0.29 / 0.30 / 0.31 | Wi-Fi link / `sniff` / `log` |

## Wiring

The wires are shared by all modes. The "SWS wire" is one wire to the target made of two C3 pins: GPIO 5 goes to
the target pin directly, GPIO 4 goes to the same pin through a 1 k … 1.8 k resistor. "Power pins" are GPIO 7, 10, 3
and 1, joined (see [Power](#power)). Pins are set in `src/config.h`.

Debug and SPI modes:

| C3 wire | Telink SWS | Arm SWD | cJTAG | JTAG | WCH SWIO | SPI flash |
|---|---|---|---|---|---|---|
| SWS wire (GPIO 5, GPIO 4 via 1 k) | SWS | SWDIO | TMSC | TDI | SWIO (PD1) | DI (pin 5) |
| GPIO 0 | – | SWCLK | TCKC | TCK | – | CLK (pin 6) |
| GPIO 21 | – | – | – | TDO | – | DO (pin 2) |
| GPIO 20 | – | – | – | TMS | – | /CS (pin 1) |
| GPIO 6 (reset wire) | RST | nRESET | nRESET | nRESET | – | RESET of the board's own processor |
| Power pins | 3.3 V | 3.3 V | 3.3 V | 3.3 V | VDD (switched for the catch) | only with `--power` |
| GND | GND | GND | GND | GND | VSS | GND (pin 4) |

UART modes (the target's TX goes to the SWS wire, where only GPIO 5 listens; GPIO 4 is then an input):

| C3 wire | Beken | ESP32 / ESP8266 | LN882H | RTL8710B | RTL8720C | UART bridge | Logger (`sniff`) |
|---|---|---|---|---|---|---|---|
| SWS wire | TX1 (P11) | U0TXD | TX0 (A2) | log TX (PA30), pulled low at restart | log TX (PA16) | target TX | line A |
| GPIO 0 (BOOT wire) | – | GPIO 0, held low | BOOT (A9), held low | – | PA00, held high | BOOT (`--boot`) | not connected |
| GPIO 21 | RX1 (P10) | U0RXD | RX0 (A3) | log RX (PA29) | log RX (PA15) | target RX | not connected |
| GPIO 20 | – | – | – | – | – | – | line B |
| GPIO 6 (reset wire) | CEN | EN (ESP8266: EN or RST) | CEN | CEN | CEN | RESET | not connected |
| Power pins | 3V3, switched for the catch | 3V3 | 3V3 | 3V3 | 3V3 | target power | not connected |
| GND | GND | GND | GND | GND | GND | GND | GND |

Never put 5 V on the target. Keep the data wires short (10 cm or less) for the faster settings. A pin is driven
only while its mode needs it; otherwise it is an input, so a loose wire does no harm.

### Power

The target can be powered from the C3's 3V3 pin, from its own supply, or from the power pins. The power pins can
switch the target off and on, which many modes use to catch the chip right after power-up (Telink boards without a
reset pad, firmware that disables the debug pin, chips whose bootloader listens only for a moment). One pin gives
about 40 mA at its limit, so join two or more (best all four) on the target's 3.3 V pad.

- Enough for: a bare chip board, a Telink or Beken module, an LN882H or ESP module while it is being flashed.
- Not enough for: relays and other loads, routers, and the Wi-Fi of an LN882H module (it restarts at every Wi-Fi
  start). Flash from the power pins, run from a real 3.3 V supply.
- Never connect the power pins to the 3V3 pin. Remove the batteries of a battery device first.
- The power pins stay inputs until the target is powered the first time. `power-off` switches them off and makes
  them inputs again.

The reset wire (GPIO 6) is open drain, active low; leave it unconnected if the board has no reset pad.

### Mains safety

Most mains-powered dimmers, plugs and relay modules have a non-isolated supply: their logic ground is connected to
the mains. **Disconnect both mains wires before you connect the flasher, and power the board from the flasher
only.** Using the flasher over Wi-Fi isolates the computer, not the C3: a C3 wired to a board on the mains is at
mains potential, together with its power bank and anything plugged into its USB port.

## Use

    python3 tlsr_c3.py -p /dev/ttyACM0 info        # Linux
    py tlsr_c3.py -p COM12 info                     # Windows

`-p` is the serial port (default `/dev/ttyACM0`); `--host NAME` instead of `-p` reaches the flasher over
[Wi-Fi](#wi-fi-link). Without a mode option the commands talk to a Telink chip; a mode option (`--swd`, `--bk`,
`--ln`, …) selects another target.

**Option order.** Global options (the port, the mode, `--run`, and every mode option such as `--ln-loader`,
`--unprotect`, `--yes-erase-all`) go **before** the command. Only the command's own options (`--twice`,
`--no-erase`, `--no-verify`, and those of `bridge`, `log`, `sniff`, `led`, `pins`) go after it:

    python3 tlsr_c3.py -p COM12 --ln --run wf 0 firmware.uf2       # right
    python3 tlsr_c3.py -p COM12 --ln wf 0 firmware.uf2 --run       # wrong: "unrecognized arguments: --run"

The commands are the same in every mode where they make sense:

| Command | What it does |
|---|---|
| `info` | identifies the chip and its flash |
| `rf ADDR SIZE FILE [--twice]` | reads flash into a file; `--twice` checks the read with a second pass or the chip's own CRC |
| `wf ADDR FILE [--no-erase] [--no-verify]` | writes a file, erasing what it needs, then verifies; modes that always erase or verify (Beken, ESP, Realtek, LN882H with Lightning's loader) refuse the option they cannot honour |
| `es ADDR SIZE` | erases the sectors that cover the range |
| `ea` | erases the whole flash (see each mode for what that includes) |
| `--run` (global) | restarts the target into its firmware at the end; without it the target stays stopped or in download mode |

Addresses and sizes may be decimal or `0x` hex. Always read a full backup before you write anything.

## Telink TLSR825x over SWS

Wiring: SWS to the SWS wire, GND, and either RST to the reset wire (board powered from the 3V3 pin) or the board's
3.3 V on the power pins (boards without a reset pad, and firmware that switches SWS off right after start). On the
power pins every command switches the board off for 300 ms and catches the chip while it starts.

    python3 tlsr_c3.py -p /dev/ttyACM0 selftest
    python3 tlsr_c3.py -p /dev/ttyACM0 info
    python3 tlsr_c3.py -p /dev/ttyACM0 rf 0 0x80000 stock.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 wf 0 new_firmware.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 es 0x2D000 0x49000
    python3 tlsr_c3.py -p /dev/ttyACM0 ea
    python3 tlsr_c3.py -p /dev/ttyACM0 --run info
    python3 tlsr_c3.py -p /dev/ttyACM0 --mode uart info

Used on these TLSR8258 boards: ZY-ZTH03Pro thermometer (512 k, reset pad), Zbeacon TS0001 relay module (512 k),
TS004F rotary knob (1 M), TS0052 dimmer module (1 M), TS011F plug with a ZT2S module (1 M); the last four without
a reset pad, on the power pins.

First use:

1. `selftest` with only the resistor fitted and no target: checks the TX – resistor – RX path.
2. `info`: prints the chip id (0x5562 for TLSR825x), the program counter and the flash JEDEC id. Every activation
   also writes a test pattern into the chip's RAM and reads it back; if that fails, nothing else is done. If the chip
   does not answer: check the resistor and GND, raise `-t`, try `--rate 500000` or `--mode uart`.
3. `rf 0 <flash size> stock.bin --twice`: full backup (0x80000 for 512 k, 0x100000 for 1 M).
4. Keep the backup safe, then `wf`. It erases the sectors it touches, writes and verifies.

Options:

- `-m rmt|uart`: how the line is driven. `rmt` (default) times the pulses exactly; `uart` is the fallback (pvvx's
  UART method, one UART byte per SWire bit).
- `--rate BPS`: RMT mode speed, 30000 … 2000000, default 1000000. The nearest speed the chip can do is used and
  printed. Reads work up to about 1.6 Mbit/s; the default leaves margin.
- `-b BAUD`: UART mode speed, 340000 … 5000000, default 921600 (92 kbit/s on the wire). Do not go higher; the
  activation check refuses a speed that reads wrong.
- `-t MS`: activation time after the reset pulse (default 100; try 300–1000 if the chip does not answer; 0 = no reset).
- `--burst N`: bytes read per request, 1 … 256 (default 64); `--burst 1` is the slowest and safest.
- `--wake`: after activation, switch the chip's clocks on and wake the flash from deep power-down (for a flash that
  reads all zeros / JEDEC 000000).
- `--run`: restart the CPU at the end. Without it the chip stays stopped after every command.

Speed at the defaults: a 512 k read takes about 9 s (20 s with `--twice`), a 182 k write with verify about 15 s.

When the flasher is plugged in, it tries for 3 seconds by itself to stop a target that is powered from the 3V3 pin
together with it. Whether that is early enough depends on the target's firmware; the power pins are the reliable way.

Cautions for Telink / Tuya firmware (check them in your own dump):

- The top sectors hold per-device data: the MAC address and the factory RF calibration (512 k flash: 0x76000 /
  0x77000; 1 M flash: 0xFE000 / 0xFF000). `ea` wipes them; `wf` erases only the sectors the file covers. To find
  them, search the backup for the device's IEEE address in reversed byte order.
- For pvvx's thermometer firmware: write the BLE firmware by wire first (smaller, address 0), then switch to Zigbee
  from his web flasher over BLE.

### Finding the pins of an unknown board

With the chip stopped, its GPIOs can be read and driven over SWS, which maps a board without reading its firmware:

    python3 tlsr_c3.py -p /dev/ttyACM0 pins --seconds 30            # 10 k pull-ups on every pin
    python3 tlsr_c3.py -p /dev/ttyACM0 pins --seconds 3 --pull down
    python3 tlsr_c3.py -p /dev/ttyACM0 led
    python3 tlsr_c3.py -p /dev/ttyACM0 led --only C4

- `pins` makes every pin (except PA7 = SWS) an input and prints each change with its time: press the button, turn
  the encoder, and read off the pins and their order. `--pull up|down|none` (default up). A pin that reads 1 with
  pull-ups and 0 with pull-downs is not connected; one that stays 1 with pull-downs is tied to the supply, often
  through an LED.
- `led` drives the pins one by one low and then high (`--hold` seconds each, default 1) and prints which one it is
  at: note the line when the LED lights. `--skip` lists pins that must stay inputs (default `A1,D4,D7`), `--only`
  tests just the given pins, three times.

## Arm SWD

Wiring: SWDIO to the SWS wire, SWCLK to GPIO 0, nRESET to the reset wire (optional), GND, power.

    python3 tlsr_c3.py -p /dev/ttyACM0 --swd info
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd rf 0 0x40000 backup.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd --run wf 0 firmware.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd es 0x3e000 0x2000
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd ea

`info` reads the debug port and the CPU type and, where the chip is known, its part, flash size and page size.
`--swd-khz` sets the clock (default 1000). The core is stopped at its reset vector for writing; `wf` erases only the
pages it touches, and verifies by CRCs computed on the flasher. `--twice` compares a CRC of the range.

| Chip | Page | `ea` |
|---|---|---|
| EFR32 Series 1 (MG1, MG12, MG13, MG14) | 2 k | mass erase |
| EFR32 Series 2 (xG21 … xG29) | 8 k | page by page |
| nRF52 | 4 k | ERASEALL (also clears APPROTECT) |
| nRF5340 application core | 4 k | ERASEALL |
| SAM D2x / L2x / C2x | 256 B rows | chip erase (also clears the security bit) |
| SAM D5x / E5x | 8 k blocks | chip erase |
| STM32 F0, F1, F3 (and GD32 / CH32 parts that report an F1 id, up to 512 k) | 1 k / 2 k | mass erase |
| STM32 F4 (needs 2.7–3.6 V) | 16 k / 64 k / 128 k sectors | mass erase |
| STM32 G0, G4, L4, WB, C0 | 2 k / 4 k | mass erase |
| STM32 L0, L1 | 128 / 256 B | page by page |
| STM32 U5 (TrustZone off) | 8 k | mass erase |
| RP2040 (external QSPI flash) | 4 k | – (give the range to `es`) |
| Puya PY32F002B, MS32C001 | 128 B | mass erase (main flash only) |

Notes:

- STM32 addresses may be given from 0 (taken as 0x08000000), RP2040 ones too (taken as 0x10000000).
- RP2040: `info` cannot read the size of the external flash, so give it to `rf`. `--run` restarts it with the watchdog.
- nRF5340: after `ea` the flasher writes UICR.APPROTECT and SECUREAPPROTECT to 0x50FA50FA, otherwise the chip locks
  its debug port again at the next reset. The firmware you write must keep it open as well (nrfx
  `nrf53_handle_approtect`).
- A debug-locked EFR32 and a read-protected STM32 (RDP, reported by `info`) cannot be read or written; unlocking is
  not implemented.
- Puya PY32F002B / MS32C001 (the SOP8 power-stage MCU of Tuya TS0052 dimmers: pin 1 VCC, 2 SWDIO, 3 SWCLK, 8 GND):
  `info` prints the chip id, the flash size and the option bytes. The option bytes are never written. An MS32C001
  reports 24 k of flash although its datasheet says 18 k; stay within 18 k. `info` and `rf` do not stop the core, so
  RAM can be read while the firmware runs. PY32F002A / F003 / F030 are not supported.

## TI CC13xx / CC26xx over two-wire cJTAG

Wiring: TMSC to the SWS wire, TCKC to GPIO 0, GND, optionally the reset wire.

    python3 tlsr_c3.py --cjtag info
    python3 tlsr_c3.py --cjtag rf 0 0x58000 backup.bin
    python3 tlsr_c3.py --cjtag --run wf 0 firmware.bin

Supports CC26x0, CC26x1, CC13x0 (4 k sectors) and CC26x2 / CC13x2 (8 k sectors); writing uses the chip's ROM
routines. What an image does not cover inside a sector is kept. The last sector holds the CCFG, which among others
enables the debug port: an image whose CCFG would disable it is refused, `ea` erases everything but that sector, and
`es` over it keeps the CCFG. Not supported: CC13x4 / CC26x4, CC2538.

## Four-wire JTAG

Wiring: TCK to GPIO 0, TMS to GPIO 20, TDI to the SWS wire, TDO to GPIO 21, nRESET to the reset wire, GND.

    python3 tlsr_c3.py --jtag info                     # scans the chain
    python3 tlsr_c3.py --jtag rf 0 0x10000 backup.bin  # an Arm chip: everything as over SWD

`info` lists the chain from the TDO side (IDCODE, maker, total instruction length). When it holds an Arm JTAG-DP,
the other devices are put in BYPASS and `rf`, `wf`, `es`, `ea` and `--run` work as over SWD for the chips in the SWD
table. The instruction lengths of the other devices must be known (the STM32 boundary-scan TAP is; a single unknown
device gets the rest). A chain without an Arm debug port is only listed. `--swd-khz` sets the clock.

## SPI NOR flash, e.g. a router's

| Flash pin | Flasher wire |
|---|---|
| /CS (1) | GPIO 20 |
| DO (2) | GPIO 21 |
| DI (5) | SWS wire |
| CLK (6) | GPIO 0 |
| GND (4) | GND |
| /WP (3), /HOLD (7) | to 3.3 V if the board does not pull them up |
| – | reset wire (GPIO 6) to the RESET of the board's own processor, if there is one |

    python3 tlsr_c3.py --spi info
    python3 tlsr_c3.py --spi rf 0 0 backup.bin --twice        # size 0 = the whole chip
    python3 tlsr_c3.py --spi --unprotect wf 0x50000 firmware.bin
    python3 tlsr_c3.py --spi --power info                     # a flash out of circuit, powered by the flasher

- In circuit, the reset wire holds the board's own processor in reset while the SPI mode is on (released at the end,
  also after an error or Ctrl-C), so it lets go of the flash. The board runs from its own supply.
- `--power` powers the flash from the power pins (switched off at the end): only for a flash out of circuit or a
  small module, never for a router.
- The size and erase unit come from the chip itself; chips over 16 Mbytes are supported. `wf` skips erase units that
  already hold the data and keeps what the file does not cover inside a unit.
- Block protection is shown by `info`; `wf`, `es` and `ea` refuse a protected chip unless `--unprotect` is given,
  which clears it and keeps the quad-enable bit.
- `--spi-khz` sets the clock (default 4000).

## WCH CH32V003 over SWIO

| CH32V003J4M6 (SOP8) pin | Flasher wire |
|---|---|
| 8: PD1 = SWIO (shared with PD4, PD5) | SWS wire |
| 4: VDD | power pins |
| 2: VSS | GND |

On other packages SWIO is PD1 alone: pin 18 of the TSSOP20 (F4P6), pin 7 of the SOP16 (A4M6), pin 15 of the QFN20
(F4U6); check the datasheet. The reset pin is not used.

    python3 tlsr_c3.py --wch info
    python3 tlsr_c3.py --wch rf 0 0x4000 backup.bin --twice
    python3 tlsr_c3.py --wch --run wf 0 firmware.bin
    python3 tlsr_c3.py --wch es 0x3c00 0x400
    python3 tlsr_c3.py --wch ea
    python3 tlsr_c3.py --wch --yes-erase-all --save-options options.bin unlock
    python3 tlsr_c3.py --wch --write-options wf 0x1FFFF800 options.bin
    python3 tlsr_c3.py --wch --external-power info

**The catch at power-up.** Once the firmware starts, the SWIO pin may be taken over (on the SOP8 it is also UART TX).
So every command switches the power pins off for `--wch-off` ms (300), then halts the core within the first
`--wch-window` ms (100) after power returns. The core stays halted until the end of the command or `--run`. With
`--external-power` the power is not switched; only a firmware that leaves SWIO alone can then be reached
(`--wch-window 10000` gives time to switch an external supply on by hand). `--wch-timing` changes the line timing
(ns: one,zero,high,slot,threshold,stop; default 250,1000,375,1500,550,4000).

What the commands do:

- `info`: chip id, flash size, unique id, read and write protection, the 16 option bytes decoded.
- Addresses: user flash at 0x08000000 (an address below 0x01000000 is taken from there), 16 k; boot area at
  0x1FFFF000 (1920 bytes); option bytes at 0x1FFFF800; RAM at 0x20000000. `rf` reads any of them.
- `wf` works in 64-byte pages: unchanged pages are skipped, uncovered bytes inside a page are kept, then verified.
- `es` erases the 64-byte pages of the range; `ea` the whole user flash (boot area and option bytes stay).
- Before `wf`, `es`, `ea` and `unlock` the option bytes are printed; `--save-options FILE` keeps a copy.
- `--write-options wf 0x1FFFF800 FILE` writes the 16 option bytes (complements are checked; a setting that switches
  read protection on is pointed out). They take effect at the next reset.

**Read protection.** On a protected chip `info` shows `READ PROTECTED` and `rf`, `wf`, `es`, `ea` refuse to start.
`--yes-erase-all unlock` removes the protection, and **the chip erases its whole user flash** in doing so: the
firmware of a protected chip cannot be read first. The flasher then power-cycles the chip and checks that the
protection is gone and the flash is blank.

## Beken BK72xx (CB2S, CB3S, WB2S …) over UART

| CB2S pad | Signal | Flasher wire |
|---|---|---|
| 7: TX1 (P11) | chip TX | SWS wire |
| 5: RX1 (P10) | chip RX | GPIO 21 |
| 10: CEN | reset | reset wire |
| 1: 3V3 | supply | power pins (all four), or the 3V3 pin |
| 3: GND | | GND |

Other modules have the same signals on other pads; see the module's datasheet. On boards like the Tuya FS-05R
dimmer, UART1 is also wired to another chip on the board: disconnect it (lift the module, cut the track, or keep
that chip unpowered), otherwise it fights the flasher's TX.

    python3 tlsr_c3.py --bk info
    python3 tlsr_c3.py --bk rf 0 0 backup.bin --twice          # size 0 = to the end of the flash
    python3 tlsr_c3.py --bk --run wf 0 firmware.uf2            # ESPHome / LibreTiny
    python3 tlsr_c3.py --bk wf 0x11000 OpenBK7231T_UA_<version>.bin
    python3 tlsr_c3.py --bk es 0x1D3000 0x8000
    python3 tlsr_c3.py --bk ea
    python3 tlsr_c3.py --bk --bk-baud 460800 --bk-seconds 20 info

**The catch.** The chip listens for a flash tool only for a moment after reset, so every command resets it (CEN and
the power pins) and sends link checks until it answers, retrying for `--bk-seconds` (10). If nothing answers, the
error says whether nothing came back (wiring, power) or the chip's own firmware talked (CEN or power not under the
flasher's control, or another chip on UART1). Without CEN and power wired, short CEN to GND for a moment while the
flasher tries. Then the speed is raised to `--bk-baud` (default 921600); if that fails, it falls back to 115200 and
says so.

What the commands do:

- `info`: protocol, speed, chip id, bootloader, flash id, size and protection bits, and the MAC address.
- `rf ADDR 0 FILE` reads to the end of the flash. Every 4 k block is checked against the chip's CRC; `--twice` adds
  a CRC of the whole range.
- `wf` clears the flash protection, skips sectors that already hold the data, keeps what the file does not cover in
  its first and last sector, and verifies with the chip's CRC.
- `es` erases the 4 k sectors (64 k blocks where possible) of the range.
- `ea` erases everything from 0x11000 to the end, including the RF calibration with the MAC address and the Tuya
  configuration: have a backup.
- `--run` restarts the chip into its firmware.
- `--bk-sr 1|2` gives the number of status register bytes of a flash chip the tool does not know (needed only when
  such a chip shows protection bits).

**The bootloader guard.** 0x0–0x11000 holds the bootloader. `wf` and `es` refuse to touch it unless
`--force-bootloader` is given (BK7231N only). On the BK7231N a wrong bootloader keeps the device from starting, but
UART flashing still works (it is in ROM). On the BK7231T the bootloader *is* the download code: writing it is always
refused, because a broken one leaves only SPI flashing. A file written at 0 that is longer than 0x11000 (a full
image or a backup) is written from 0x11000 on; the flasher says whether its bootloader part equals the chip's.

| File | Give it as |
|---|---|
| ESPHome / LibreTiny `firmware.uf2` | `wf 0 firmware.uf2` (the parts are placed by the file itself) |
| `image_bk7231n_app.0x011000.rbl` | `wf 0x11000 file` |
| OpenBeken `OpenBK7231N_QIO_<version>.bin` (full image) | `wf 0 file` |
| OpenBeken `OpenBK7231T_UA_<version>.bin` | `wf 0x11000 file` |
| a backup from `rf 0 0` | `wf 0 backup.bin` (everything but the bootloader) |
| `*_app.ota.rbl`, OpenBeken `.rbl`, `*.ug.bin` | refused: these are OTA update packages |

## ESP32 / ESP8266 over UART

Wiring: target TX0 to the SWS wire, RX0 to GPIO 21, its GPIO 0 to the BOOT wire (GPIO 0), EN (ESP8266: EN or RST)
to the reset wire, GND, 3V3 to the power pins. The strapping pins must allow the download mode as on any ESP board:
on a bare ESP8266 module GPIO 15 low, GPIO 2 high and EN high (10 k resistors on ESP-12); on an ESP32 GPIO 2 low or
open. This mode is for bare modules and devices (plugs, switches, sensors); a development board with its own USB–UART
chip is flashed through that.

    python3 tlsr_c3.py -p /dev/ttyACM0 --esp info                       # chip, MAC, flash id
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp rf 0 0 backup.bin --twice  # whole flash (esptool 5)
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp rf 0 0x400000 backup.bin   # esptool 4: give the size
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp wf 0 firmware.factory.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp es 0x9000 0x5000
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp ea

Needs esptool 4 or 5 on the computer. The flasher restarts the target with GPIO 0 held low (download mode), opens
the [UART bridge](#uart-bridge) and runs esptool through it; at the end the target is restarted into its firmware
(`--esp-stay` leaves it in download mode). esptool erases what it writes and checks the MD5; `rf --twice` reads twice
and compares.

- `--esp-baud`: esptool's working speed (default 921600). The bridge follows esptool's speed change by itself. A
  failed run is repeated at 460800, then 115200.
- `--esp-chip esp8266` (or esp32, esp32c3, …) skips the chip detection.
- The power pins carry an ESP8266 or a small ESP32 module while it is flashed, not its Wi-Fi peaks.

## Lightning LN882H over UART

Wiring: TX0 (A2) to the SWS wire, RX0 (A3) to GPIO 21, BOOT (A9) to the BOOT wire (GPIO 0), CEN to the reset wire,
3V3 to the power pins, GND. The pad names of a Tuya module are in its datasheet. The power pins are enough to flash
the module but not to run its Wi-Fi: run the new firmware from a real 3.3 V supply.

    python3 tlsr_c3.py -p /dev/ttyACM0 --ln info                        # flash id, size, status, MAC
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln rf 0 0 backup.bin --twice   # whole flash, then the chip's CRC of it
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln --run wf 0 firmware.uf2     # LibreTiny / ESPHome UF2
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln wf 0 backup.bin             # a full image from 0
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln es 0x1a0000 0x1000
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln ea
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln --ln-loader vendor info     # with Lightning's RAM loader

The flasher restarts the chip with BOOT held low, which starts the ROM's download mode, and loads a RAM loader into
the chip through the UART bridge. The speed is then raised to `--ln-baud` (default 1000000; on failure 460800, then
115200). `--ln-seconds` (default 5) is how long to wait for the ROM after the restart.

Two RAM loaders, chosen with `--ln-loader`:

- `own` (default): built into `tlsr_c3.py`, nothing to download. Reads 4 k per request, checks every transfer with a
  CRC, skips sectors that already hold the data, erases only sectors that are not blank, and verifies by the chip's
  CRC. `rf --twice` compares the chip's CRC of the range instead of reading twice. Read about 88 kB/s, write about
  53 kB/s. `--no-erase` writes only into blank sectors; `--no-verify` skips the final check.
- `vendor`: Lightning's own loader, as ltchiptool ships it. It is not part of this repository: download it once as
  `ln882h_ramcode.bin` next to `tlsr_c3.py` from
  <https://github.com/libretiny-eu/ltchiptool/raw/410874f66b60da1dd667dbba69f499acbf16eb03/ltchiptool/soc/ln882h/util/ramcode.bin>
  (37872 bytes, SHA-256 `6bd437c6f8366b9cca0fb8de0c80c70788516e3681c6c43d654512feb7a0c723`), give its path with
  `--ln-ramcode`, or have ltchiptool installed (its copy is found). Slower; verifies by reading back. `--no-erase` is
  not available, and it cannot write data that contains a text line `baudrate <digits>`.

With both loaders:

- A UF2 (LibreTiny / ESPHome, family LN882H) places its parts itself. A full image or a backup goes to 0.
- There is no bootloader guard: the download mode is in ROM, so a wrong write cannot lock you out.
- `ea` erases everything, including the boot loader and the partition table. The MAC address is in the flash's OTP
  area and stays.
- Before `wf`, `es` and `ea` the flash's block protection is cleared (other status bits are kept); `--bk-sr 1|2`
  gives the number of status bytes of an unknown flash chip that shows protection bits.
- `--run` restarts the chip with BOOT released; otherwise it stays in the loader.

If something fails, `python3 tlsr_c3.py log` shows what the chip said (see [UART log](#uart-log)); the own loader
prints how far its start got, and `--ln-loader vendor` remains as an alternative.

## Realtek AmebaZ (RTL8710B) over UART

Wiring: log TX (PA30) to the SWS wire, log RX (PA29) to GPIO 21, CEN to the reset wire, 3V3 to the power pins, GND.
On a WR module these are the TX / RX pads of the log UART (on the WR3: TXD2 / RXD2).

    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl info                        # chip, flash id and size, MAC, OTA slot
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl rf 0 0 backup.bin           # whole flash, checked by the chip's MD5
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl --run wf 0 firmware.uf2     # LibreTiny / ESPHome UF2
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl wf 0 backup.bin             # a full image from 0
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl es 0x1f0000 0x1000
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl ea

The chip enters its download mode when its log TX is low at reset: the flasher pulls the SWS wire low while it
restarts the chip, then releases it. Every read and write is checked against an MD5 the chip computes (2–3 s each;
`--no-verify` skips it for a write). The ROM erases what it writes, so `--no-erase` is not available. A UF2 is
written into the OTA slot the bootloader starts; if that is OTA2 and its address differs from the UF2's, it is
refused (write a full image instead). `--rtl-seconds` (default 5) is how long to wait for the ROM. `--run` restarts
the chip with its TX released; otherwise it stays in download mode.

## Realtek AmebaZ2 (RTL8720C) over UART

Wiring: log TX (PA16) to the SWS wire, log RX (PA15) to GPIO 21, PA00 to the BOOT wire (GPIO 0), CEN to the reset
wire, 3V3 to the power pins, GND. PA13 (RX0) must not be pulled low.

    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 info                        # chip, flash, MACs
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 rf 0 0 backup.bin           # whole flash, checked by the chip's SHA-256
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 --run wf 0 firmware.uf2     # LibreTiny / ESPHome UF2
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 wf 0 backup.bin             # a full image from 0
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 es 0x1f0000 0x1000

The chip enters its download mode when PA00 is **high** at reset; the BOOT wire holds it high while the flasher
restarts the chip. The speed is raised to `--rtl-baud` (default 1500000; on failure 921600, then 115200). Reads are
slow (a 2 MB flash takes a bit over a minute) and are checked against the chip's SHA-256, as are writes. The ROM
erases what it writes (no `--no-erase`) and has no erase command, so `es` and `ea` write 0xFF. A UF2 goes into OTA1;
a full image or a backup goes to 0. `--run` restarts the chip with PA00 released.

## UART bridge

    python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --boot hi
    ... any serial tool on the same port, at the same baud ...
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit

The flasher passes bytes between its USB port and the target's UART (target TX on the SWS wire, target RX on
GPIO 21), after restarting the target with the BOOT wire at the level given by `--boot` (`hi`, `lo`, or `z` = left
open, the default). It stays a bridge until `bridge-exit`.

- **The speed is fixed by the `bridge` command:** the C3's USB port does not pass on the speed your serial tool sets.
  The options below make the bridge follow a known tool's speed change.
- **Tools must not toggle DTR / RTS**: that resets the C3 itself (for esptool: `--before no-reset --after no-reset`).
- GPIO 21 carries the C3's own boot messages for a moment when the flasher is plugged in.

Options (one at a time):

- `--catch bk`: for ltchiptool and other Beken tools: resets the target every 1.5 s until a Beken ROM answers, so the
  tool links without a manual reset. Keep the tool at 115200. (The `--bk` mode does the same job on its own.)

      python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --catch bk
      ltchiptool flash read -d /dev/ttyACM0 -b 115200 bk72xx backup.bin
      python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit

- `--follow esp`: for running esptool yourself: start at 115200 with BOOT low and give esptool any `--baud`; the
  bridge switches when the target has accepted esptool's speed change. Start a fresh `bridge` for every esptool run.

      python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --boot lo --follow esp
      esptool --port /dev/ttyACM0 --baud 921600 --before no-reset --after no-reset flash-id
      python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit

- `--follow ln`: follows the LN882H RAM loaders' `baudrate N` line.
- `--follow ucfg`: follows the RTL8720C ROM console's `ucfg N` line.
- `--strap rtl`: restarts the target with its TX wire pulled low (the RTL8710B download strap).

Any program can also change the bridge's speed itself: after a one-second pause, send `~~~S<baud>` and a line feed in
one write. A one-second pause and `~~~X` ends the bridge (this is what `bridge-exit` does). Neither reaches the target.

## UART log

Every UART session of the flasher (`bridge`, `--esp`, `--bk`, `--ln`, `--rtl`, `--rtl2`, `sniff`) is recorded in the
C3's RAM: every byte in both directions with its time, and events such as resets and speed changes. Read it after a
failed run:

    python3 tlsr_c3.py log                      # writes uart_log.txt
    python3 tlsr_c3.py log run1.txt --bin run1.bin
    python3 tlsr_c3.py log --tuya               # decode Tuya MCU frames (a sniff session)
    python3 tlsr_c3.py log --clear

- Each session starts a new log; the last one stays until the next session, `log --clear`, or the flasher loses power.
- Leave the bridge (`bridge-exit`) before `log`. The `--esp` mode restarts the target for every esptool run and retry,
  so the log holds the last of them.
- The text has the time in seconds, `>` host to target, `<` target to host, `#` events (`A` / `B` for the two lines
  of `sniff`), then hex and ASCII, 16 bytes a line. Repeated lines and records are shown once with a count.
- **Size:** the log is compressed in about 110 k of RAM. The start of a session (catch, link, speed change) is always
  kept; after that, the oldest part is dropped, so a long session keeps its start and its end, and the text marks the
  gap. Repetitive console text fits about 1 MB, varied text about 270 k, random or encrypted data about 88 k. With
  Wi-Fi on the log gets less room. `log` prints the store's size and the free memory.

## Two-line UART logger

    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --tuya                    # live; Ctrl-C ends it
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --baud-b 115200 --seconds 60 --out sniff.txt
    python3 tlsr_c3.py --host c3-flasher.local sniff 9600 --tuya            # live over Wi-Fi
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --at-boot                 # capture from every power-up, no host
    python3 tlsr_c3.py --host c3-flasher.local log sniff.txt --tuya         # read that capture afterwards
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff --no-at-boot

Listens to both directions of a UART link between two other chips (for example a Tuya Wi-Fi module and the board's
MCU) without driving anything, and shows it live and records it in the [UART log](#uart-log).

| Flasher wire | Logger |
|---|---|
| SWS wire | line A: one direction, e.g. the MCU's TX |
| GPIO 20 | line B: the other direction, e.g. the module's TX |
| GND | the board's GND |
| GPIO 21, GPIO 0, GPIO 6, power pins | not connected |

Both lines must be 3.3 V logic (the C3 is not 5 V tolerant). The board keeps its own supply. Line B is GPIO 20,
not GPIO 21, because the C3 sends its boot messages on GPIO 21 at every power-up.

- `sniff BAUD [--baud-b N]`: line A's speed, and line B's when it differs (1200 … 5000000).
- Live output: the time, `A` or `B`, hex and ASCII, plus events (framing errors and breaks, which usually mean a
  wrong speed). `--out FILE` also writes it to a file; `--seconds N` ends after N seconds, otherwise Ctrl-C.
- The capture never waits for the computer: if the output cannot keep up (fast lines over weak Wi-Fi), the live output
  drops bytes and says how many (`# live output: N bytes dropped`). The UART log still gets everything.
- If the script was killed, `bridge-exit` ends the logger. Over Wi-Fi the live output ends when its TCP connection
  goes; the capture into the log goes on until the next command from any host.
- `--tuya` decodes the Tuya MCU serial protocol (`55 AA` frames: command names and data points) under each record.
- Timing: the order of a request on one line and the answer on the other is reliable when the answer starts at least
  about 1 ms plus two character times after the request (about 3 ms at 9600 baud).

**Capture at power-up.** `sniff BAUD --at-boot` stores the speeds in the C3. From then on, at every power-up, the
flasher skips its usual target activation and starts listening at once (with Wi-Fi started first, if set up). The
first command from a host stops it; read the capture with `log` / `log --tuya`. **The log is in RAM and is lost when
the C3 loses power**, and every new session replaces it, so read it first. `sniff --no-at-boot` switches this off.

**Mains-powered boards.** A module and MCU on a non-isolated board are at mains potential, and so is the C3 wired to
them. Use live capture only on a board powered from a safe low voltage, or over Wi-Fi with the C3 on a power bank,
nothing on its USB port, and nobody touching it. Otherwise use the capture at power-up: wire the C3 on its power bank
with the board disconnected from the mains, close everything, connect the mains and let it run; afterwards read the
log over Wi-Fi without touching anything, or disconnect the mains, take the C3's wires off while the power bank keeps
it running, and only then plug it into the computer.

## Wi-Fi link

    python3 tlsr_c3.py -p /dev/ttyACM0 wifi join "Home Net"        # join a network (password asked)
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi ap "C3 flasher"        # or run an access point (192.168.4.1)
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi key                    # set the key TCP clients must give
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi name c3-flasher        # mDNS name
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi                        # status
    export C3_FLASHER_KEY=...                                       # or --key
    python3 tlsr_c3.py --host c3-flasher.local --bk rf 0 0 backup.bin --twice
    python3 tlsr_c3.py --host 192.168.4.1 log
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi off

The flasher can join a network or run its own access point, and then takes every command on TCP port 7171, so the
computer need not be wired to the board. The setting is kept in the C3 and Wi-Fi starts with it at power-up.

- Set it up over USB: `wifi join` / `wifi ap` restart Wi-Fi, and a wrong password is fixed over USB again. The password
  is asked for (or taken from `C3_FLASHER_WIFI_PASSWORD`) and never shown. An access point password needs 8–63
  characters.
- `wifi name NAME` sets the mDNS name (default `c3-flasher`, so `c3-flasher.local`), used from the next Wi-Fi start.
  On its own access point the flasher is at 192.168.4.1.
- `--host NAME[:PORT]` replaces `-p` in every command, including `--esp` (esptool gets `socket://NAME:7171`), `bridge`,
  `log` and `sniff`. One TCP client is served at a time; a new connection replaces the old one. USB keeps working
  alongside, and a command from USB ends a bridge that a TCP client opened.
- **The key.** Without a key, anyone on the network can read and write the target through the flasher. `wifi key` sets
  one (1–64 printable characters, no spaces; empty removes it); give it with `C3_FLASHER_KEY` or `--key`. A connection
  without the right key is closed. While a bridge opened over TCP is on, the same computer may connect without the key
  (esptool cannot send it); the bridge ends by itself 60 s after the last TCP client left. The key travels in clear
  text: it keeps others out on a trusted network, it is no protection on an untrusted one.
- Over Wi-Fi every answer of a bridged target comes with the network's delay, which slows protocols with many small
  exchanges. Wi-Fi also takes memory from the [UART log](#uart-log); `wifi off` gives it back from the next session.
- Wi-Fi isolates the computer, not the C3: see [Mains safety](#mains-safety).

## Limitations / not implemented

- Telink: only TLSR825x (not the TLSR826x family); pvvx's RAM loader over the chip's UART is not used.
- SWD: no flash routine for STM32 F7 / H7 and F1 XL-density parts, the second bank of the bigger GD32F10x / F30x,
  GD32E23x (identified only); no unlocking of a debug-locked EFR32 or a read-protected STM32.
- cJTAG: no CC13x4 / CC26x4, no CC2538.
- JTAG: only Arm debug ports are used; RISC-V, EJTAG and boundary scan are not built on the raw scans.
- WCH: only the CH32V003. Not the V002 / V004 / V005 / V006 / V007, CH32X035, CH32V103 / V20x / V30x, CH5xx, nor the
  two-wire RVSWD of the bigger parts.
- Beken: BK7231N, BK7231T and BK7238 only (BK7231Q / U and BK7252 not); no eFuse / encryption key readout.
- Modes marked "simulated only" in the table at the top have not run on a real chip.

## Host protocol

For people who write their own host tools. One text line per command, answered by `OK [payload]` or `ERR reason`;
numbers are decimal or `0x` hex, data is hex. The same protocol runs over USB and over TCP port 7171.

General and Telink:

| Command | Meaning |
|---|---|
| `V` | firmware name and version, settings |
| `MODE uart\|rmt`, `BAUD n`, `RATE n`, `BURST n` | SWS line settings |
| `SELFTEST` | line test without a target |
| `PWR 1\|0\|z` | power pins on, off, or inputs |
| `RST` | reset pulse and power cycle |
| `A ms` | activate (reset, stop the CPU, set the speed) |
| `G addr len` / `S addr hex` | read / write registers or RAM |
| `FJ`, `FS` | flash JEDEC id, status |
| `FR addr len` / `FB addr len` | read flash, hex / base64 (`*` = all 0xff) |
| `FC addr len` | CRC-32 of a flash range |
| `FU`, `FE addr`, `FA`, `FW addr hex` | unprotect, erase a 4 k sector, erase all, program within one page |
| `RUN` | restart the CPU |
| `STAT` | timing since the last `STAT` |

SWD / cJTAG: `SWD`, `SWCLK khz`, `SC [reset|md sel]`, `TC` (cJTAG + ICEPick, `OK <icepick id> <dap id>`),
`SRST 0|1`, `SDR` / `SDW` / `SAR` / `SAW` (DP / AP registers), `SM` / `SMW` (memory), `SFAM name [p1 p2]` (select
the flash routine), `SHALT [reset]`, `SRUN`, `SCALL pc sp lr [r0..r3]`, `SFE addr`, `SFX`, `SFW addr hex`,
`SFC addr len`.

JTAG: `JTAG [khz]`, `JS` (chain scan: `OK <IR bits> <idcode> ...`, TDO side first, 0 = no IDCODE),
`JSEL pre_ir post_ir pre_dr post_dr` (DAP position), `JC` (connect the JTAG-DP, `OK <idcode>`), `JRST`,
`JIR bits hex` / `JDR bits hex` (raw scans, LSB first, answer the captured bits).

SPI flash: `SPI [khz]` (enter, board reset held low), `SPI 0` (leave, reset released), `PT hexout [nin]` (raw
transaction), `PJ` (JEDEC id), `PA 3|4` (address width), `PB addr len` (base64), `PC addr len` (CRC-32),
`PE addr [0x20|0x52|0xD8]`, `PX` (chip erase), `PW addr hex` (within one 256-byte page), `PS` (status 1).

WCH CH32V003: `WCH` (enter, `OK <timing>`), `WCH 0` (leave), `WT [one zero high slot threshold stop]` (timing in ns),
`WA off_ms window_ms` (power off for off_ms unless 0, then catch: `OK <dmstatus> <cpbr> <tries> <µs to the first
answer> <1 = power cycled>`), `WR addr` / `WW addr value` (debug module registers), `WH [reset]` (halt), `WG`
(resume), `WRUN` (reset and run), `WM addr len` (memory, whole words), `WMW addr hex` (RAM / registers, up to 512
bytes), `WFU` (unlock, `OK <CTLR>`), `WFE addr` (erase a 64-byte page), `WFX` (mass erase), `WFW addr hex` (program
inside one erased page), `WFC addr len` (CRC-32), `WOB hex` (erase and write the 16 option bytes).

Beken BK72xx: `BK` (enter; `BK 0` leaves), `BKL catch_ms baud` (catch, link, speed: `OK <rom|bl> <baud> <resets>
<ms to the answer> <1 = fell back to 115200> <bytes of the chip's firmware seen> [why]`), `BKI` (`OK <chip id>
<flash JEDEC id>`, `0 000000` on the BK7231T), `BKRR addr` (register), `BKSR opcode` / `BKSW opcode hex` (flash
status register), `BKC addr len` (CRC-32 computed by the chip), `BKE addr [64]` (erase a 4 k sector / 64 k block),
`BKR addr len` (read up to 512 bytes, hex), `BKB offset hex` + `BKW addr` (fill a 4 k buffer, write it to an erased
sector), `BKG [first writable address [flash size]]` (bootloader guard, 0x11000 after `BK`), `BKX` (reboot).

UART bridge: `BRIDGE <baud> [hi|lo|z] [bk|esp|ln|ucfg|rtl]` (BOOT level during the restart; `bk` catches a Beken ROM,
`esp` / `ln` / `ucfg` follow the tool's speed change, `rtl` holds the target's TX low across the restart; up to
5000000 baud). In the bridge: a one-second pause and `~~~S<baud>\n` sets the speed, a one-second pause and `~~~X`
leaves it. The bridge belongs to the source (USB or TCP) that opened it.

UART log: `LOG` (closes the block being filled, answers `OK log stored=<bytes> blocks= raw= dropped= tx= rx= heap=
heap_min=`; read it with `XR L 0 <stored>`), `LOG 0` (clear).

Two-line logger: `SNIFF <baud A> [<baud B>]` answers `OK sniff A <baud> B <baud>; ...`, then sends binary frames
`<tag:u8> <t_us:u32> <n:u16> <data>` (little endian; tag 1 = line B, 2 = line A, 3 = event text; t_us wraps) until a
one-second pause and `~~~X`, answered by `OK sniff closed` after the last frame. `SNIFFBOOT <baud A> [<baud B>]` /
`SNIFFBOOT 0` sets / clears the capture at power-up (without arguments: shows the setting).

Wi-Fi: `WIFI` (status: `OK wifi mode=off|ap|sta ssid=<hex> pass=none|set key=none|set name= port= ... client= heap=
largest=`, no secrets), `WIFI AP <ssid hex> <password hex>`, `WIFI STA <ssid hex> <password hex | ->`, `WIFI OFF`,
`WIFI KEY <key hex | ->`, `WIFI NAME <name>`; AP and STA restart Wi-Fi and drop the TCP client. Over TCP the first
line may be `AUTH <key>` (`OK auth`, or `ERR auth` and the connection is closed); with a key set it must be, except
from the address that opened a bridge while that bridge is on. A client that takes nothing for 8 s is dropped.

Binary streams for bulk data: `XR <src> <addr> <len>` answers `OK <len>`, then frames `D <n:u16> <data> <crc32>` of
up to 4 k (`E <n:u16> <text>` on an error, `Z` at the end). `XW <dst> <addr> <len> <unit> <skip>` answers `OK`, then
takes frames `<n:u16> <data> <crc32>` and answers each with `K` once written; up to three frames may be in flight.
After an error everything is dropped until the line has been quiet for 300 ms. Sources / destinations: `F` Telink
flash, `S` SWD flash (write) / memory (read), `M` SWD memory, `P` SPI flash, `C` CH32V003 memory (read) / flash or
RAM (write), `B` BK72xx flash (4 k aligned), `L` the UART log (read). `unit` is the boundary no single flash write
crosses; `skip 1` leaves out parts that are all 0xff (not on STM32 L0 / L1, whose erased flash reads 0x00).

## Files

- `src/main.cpp`, `src/config.h`, `platformio.ini`: the ESP32-C3 firmware: command parser, Telink SWS, pins.
- `src/swd.cpp` / `.h`: SWD, cJTAG and JTAG line handling and debug access.
- `src/swdflash.cpp` / `.h`: flash routines of the Arm and TI chips.
- `src/spiflash.cpp` / `.h`: SPI NOR flash.
- `src/wch.cpp` / `.h`, `src/wchflash.cpp` / `.h`: WCH SWIO line, debug module, flash and option bytes.
- `src/bk72xx.cpp` / `.h`: Beken download protocol.
- `src/tuart.cpp` / `.h`: the target UART shared by the bridge and the Beken mode.
- `src/lnscan.h`: the bridge's detection of a loader's speed-change line.
- `src/uartlog.cpp` / `.h`: the UART log.
- `src/sniff.cpp` / `.h`, `src/sniffring.h`: the two-line logger.
- `src/wifilink.cpp` / `.h`: the Wi-Fi link.
- `tlsr_c3.py`: the command-line tool (Python 3, pyserial), with the LN882H loader built in.
- `loaders/ln882h/`: the LN882H RAM loader and its build script.
- `tests/`: checks without hardware; see `tests/README.md`.

## For developers

- The protocols, register sequences and design choices are documented at the top of each source file; the LN882H
  loader's protocol is described at the top of `loaders/ln882h/loader.c`.
- After changing the LN882H loader, rebuild and embed it into `tlsr_c3.py` with PlatformIO's Arm toolchain
  (`~/.platformio/packages/toolchain-gccarmnoneeabi`, or `--cc`):

      python3 loaders/ln882h/build.py            # build, fill the boot header, embed into tlsr_c3.py
      python3 loaders/ln882h/build.py --check    # is the embedded one this build?

- The tests run without hardware; see `tests/README.md`.

## Credits

SWire emulation over a UART, the register sequences and the activation: pvvx
(<https://github.com/pvvx/TlsrComSwireWriter>, public domain; <https://github.com/pvvx/TLSRPGM>).

WCH SWIO:

- WCH (Nanjing Qinheng Microelectronics): the QingKe V2 debug manual and the one-wire host example for a CH32F103,
  both in <https://github.com/openwch/ch32v003> (`CH32V003_1Line_Base_on_CH32F103`): the line timing, the output
  enable, halt and resume, and the read / write program for the program buffer.
- ch32v003fun by Charles Lohr and contributors (MIT; <https://github.com/cnlohr/ch32v003fun>): minichlink and the
  funprog programmer core (`minichlink/funprog/funprog_rvswdio.h`), with its ESP32-S2 programmer
  (<https://github.com/cnlohr/esp32s2-cookbook>, `ch32v003programmer`) and the CH32V003 programmer in
  <https://github.com/cnlohr/rv003usb> (`rvswdio_programmer`): the flash buffer-load program (adapted), the
  power-off catch of its unbrick, the flash and option byte sequences, the chip identification and the factory
  option bytes, and measured pulse lengths for the timing.
- Ardulink by Mark Boldyrev (MIT-0; <https://gitlab.com/BlueSyncLine/arduino-ch32v003-swio>): another timing
  reference, and the series resistor on SWIO.

Beken BK72xx:

- ltchiptool by Kuba Szczodrzyński (MIT; <https://github.com/libretiny-eu/ltchiptool>, `ltchiptool/soc/bk72xx`, and
  its `uf2tool`): the flashing sequence (unprotect, erase, write, CRC), the RF calibration / MAC location, and the UF2
  format (blocks, tags, `OTA_PART_INFO`, `FAL_PTABLE`). LibreTiny (<https://github.com/libretiny-eu/libretiny>, MIT):
  the board flash layouts, the UF2 family ids, the build outputs and the note that the BK7231T has no ROM loader.
- bk7231tools by Khaled Nassar, Tom Clement and Kuba Szczodrzyński (MIT; <https://github.com/tuya-cloudcutter/bk7231tools>,
  `bk7231tools/serial`): the packet framing, the command codes and answers, linking with resets, the speed change,
  the CRC conventions of the two protocols, the BK7231T bootloader's address bypass and its protection after a CRC.
  Adapted from it: the bootloader CRC list, the chip id values and the table of status register sizes per flash id,
  with the protection mask.
- BK7231GUIFlashTool by openshwprojects (<https://github.com/openshwprojects/BK7231GUIFlashTool>; its repository has
  no license file, so no code was taken from it): read for the protocol only, which is reimplemented here: the flash
  id answer with the short length field, the speed change timing, the status register opcodes per flash maker
  (Macronix), the erase time-outs and the handling of OpenBeken's `_QIO_` / `_UA_` images.

Realtek AmebaZ: ltchiptool by Kuba Szczodrzyński (MIT; `ltchiptool/soc/ambz`): the download protocol (handshakes,
read, XMODEM with addresses, speed table), the RAM programs (`ambzcode.py`, taken over as `RtlCode`), the OTA slot
choice and the system data layout, and the chip ids. rtltool by pvvx (in ltchiptool, `ambz/util/rtltool.py`): the
sector erase and the packet checksum.

Realtek AmebaZ2: ltchiptool (MIT; `ltchiptool/soc/ambz2`, `soc/amb/efuse.py`): the console commands and their
answers, the flash set-up, `hashq`, the cancelled `fwd`, starting a RAM program through an unused console command,
the eFuse read program, the physical-to-logical eFuse conversion, the MAC locations, the strapping and the OTA1 choice.

Lightning LN882H:

- ltchiptool by Etienne Le Cousin and Kuba Szczodrzyński (MIT; <https://github.com/libretiny-eu/ltchiptool>,
  `ltchiptool/soc/ln882h`): the sequence (ROM link, RAM loader over YMODEM, speed change, `flash_read` with its CRC,
  `startaddr` + `upgrade`), the wiring and the RAM loader binary it ships (Lightning's, not included here).
- OpenLN882H SDK (<https://github.com/openshwprojects/OpenLN882H>, Shanghai Lightning Semiconductor's SDK; it has no
  license, so nothing of it is copied here: our loader is written from the facts read in it): the source of
  Lightning's RAM loader (`project/bootcode/ramcode_dl`): its commands, the YMODEM receiver, the erase of the received
  range that makes the host send whole sectors, the boot header layout, and what that loader sets up (the clocks in
  `SetSysClock`, `mcu/ln882h/system_ln882h.c` and `hal_clock.c`; UART0 on A2 / A3 with `hal_uart.c` and `hal_gpio.c`,
  the BRR formula; the flash cache switched off, the QSPI controller); the register addresses and bits of the clock
  tree (`reg_sysc_awo.h`, `reg_sysc_cmp.h`: clock source, PLL, AHB / APB0 dividers and their update bits, band gap and
  LDO settings), UART0, the pin functions, GPIOA, the QSPI controller (a DesignWare SSI), the flash cache, the clock
  gates, the resets, the pad enable and the chip reset (`mcu/driver_ln882h`); the transfer sequences of its flash
  driver and the flash commands it uses, the MAC address in the flash's security register at 0x100
  (`components/fs/nvds`), and how its build tools fill the boot header's length and CRCs (`tools/python_scripts`).
  LibreTiny: the UF2 family id.
- ltchiptool's copy of the SDK's boot header model (`ltchiptool/soc/ln882h/util/models/boot_header.py`): the same
  header fields; `loaders/ln882h/build.py` fills them in its own code and reproduces the header of Lightning's loader
  binary exactly (checked by `tests/test_ln_loader.py` when that binary is at hand). The start of that binary
  (disassembled, nothing taken from it but the order and values of its clock and UART set-up) confirmed that it runs
  the SDK's `SetSysClock` with the PLL, multiplier 8, APB0 / 4, and sets UART0 to 115200 with 8x oversampling.

## License

MIT, see `LICENSE`.
