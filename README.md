# ESP32-C3 flasher: Telink SWS, Arm SWD, cJTAG, JTAG, SPI flash, WCH SWIO, Beken, LN882H and Realtek UART, ESP32 / ESP8266

An ESP32-C3 board as a flasher for small boards' chips: Telink TLSR825x over its single-wire debug pin (SWS,
where it started), Arm Cortex-M chips over SWD or JTAG, TI CC13xx / CC26xx over two-wire cJTAG, 25-series
SPI NOR flash, the WCH CH32V003 over its single-wire debug pin (SWIO), Beken BK7231N / BK7231T Wi-Fi chips
over their UART download port, the Lightning LN882H and the Realtek RTL8710B and RTL8720C through their ROM download modes, and
ESP32 / ESP8266 chips through their ROM download mode with esptool. It also
works as a USB–UART bridge and as a two-line UART logger that listens to a link between two other chips (firmware
0.30), and the computer can reach it over Wi-Fi instead of USB (firmware 0.29).
All modes use the same few wires; one is active at a time.

**What is tested (2026-11-27).** The Telink mode has been used on real boards, the Arm SWD mode on one
chip: a Puya MS32C001 (PY32F002B type) in a Tuya dimmer, read and written several times (see
[Arm SWD](#arm-swd)), the Beken mode on a CB2S (BK7231N) in a Tuya FS-05R dimmer: full backup read and an
ESPHome firmware written (see [Beken](#beken-bk72xx-cb2s-cb3s-wb2s--over-uart)), and the LN882H mode on an LN-02 in
a CozyLife relay module: backup read with both loaders and an ESPHome firmware written with ours, which then ran
(see [LN882H](#lightning-ln882h-over-uart)); the UART bridge and the UART log were used for that too. The other modes are written from
the vendors' documentation and headers and checked without hardware (see `tests/`); expect to find bugs on the
first real chip, and keep a backup.

| Mode | Option | Targets | Tested |
|---|---|---|---|
| [Telink SWS](#telink-tlsr825x-over-sws) | (default) | TLSR8251 / 8253 / 8258 (Tuya "Z2", ZT2S, ZTU …) | on hardware: five kinds of boards |
| [Arm SWD](#arm-swd) | `--swd` | EFR32, nRF52 / nRF5340, STM32, SAM D / E, RP2040, GD32 / CH32 (F1 compatible), Puya PY32F002B / MS32C001 | on hardware: Puya MS32C001 (info, read, write); the other chips no |
| [cJTAG](#ti-cc13xx--cc26xx-over-two-wire-cjtag) | `--cjtag` | TI CC13xx / CC26xx | no |
| [JTAG](#four-wire-jtag) | `--jtag` | chain scan; Arm chips behind a JTAG-DP as over SWD | against a simulated chain |
| [SPI flash](#spi-nor-flash-eg-a-routers) | `--spi` | 25-series SPI NOR (routers, dongles, modules) | host logic against simulated chips |
| [WCH SWIO](#wch-ch32v003-over-swio) | `--wch` | WCH CH32V003 (SOP8 J4M6, TSSOP20 F4P6, …) | no; firmware and host against a simulated chip |
| [Beken UART](#beken-bk72xx-cb2s-cb3s-wb2s--over-uart) | `--bk` | BK7231N (Tuya CB2S, CB3S, …), BK7231T (WB2S, WB3S, …), BK7238 (Tuya T1) | on hardware: CB2S (BK7231N; read, write); BK7231T and BK7238 no |
| [ESP32 / ESP8266](#esp32--esp8266-over-uart) | `--esp` | ESP32, ESP32-S2 / S3 / C2 / C3 / C6 / H2, ESP8266 (with esptool) | no; the esptool runs checked against esptool 4.14 and 5.7 without hardware |
| [LN882H UART](#lightning-ln882h-over-uart) | `--ln` | Lightning LN882H (newer Tuya Wi-Fi / BLE modules) | on hardware (2026-11-27): an LN-02 (LN882H, TH25Q16 flash) in a CozyLife relay module. Lightning's loader: info, full read. Ours (protocol 2, after the start-up rebuild): info, full read (88 kB/s, the same bytes as Lightning's), an ESPHome UF2 written (53 kB/s, verified by the chip's CRC) that then ran |
| [Realtek AmebaZ UART](#realtek-amebaz-rtl8710b-over-uart) | `--rtl` | RTL8710BN / BX (Tuya WR2, WR3 …) | no; host and bridge against a model of the ROM |
| [Realtek AmebaZ2 UART](#realtek-amebaz2-rtl8720c-over-uart) | `--rtl2` | RTL8720CF, RTL8710CX / CM (Tuya WBR2, WBR3, CR3L …) | no; host and bridge against a model of the ROM console |
| [UART bridge](#uart-bridge) | `bridge` | any 3.3 V UART, with a BOOT pin | no |
| [Two-line UART logger](#two-line-uart-logger) | `sniff` | both directions of a 3.3 V UART link between two other parties (a Tuya module and its MCU), listened to without driving; Tuya MCU protocol decoded | no; the live output's ring natively, the host's parser and Tuya decoder against models, the firmware's capture only built |
| [Wi-Fi link](#wi-fi-link) | `--host`, `wifi` | the flasher itself, reached over TCP instead of USB (every mode above) | no; the host against a model of the flasher's command side, the firmware's Wi-Fi part only built |

## Board, build and upload

    pio run -e c3-usb -t upload      # USB connector wired straight to the C3 (Super Mini, XIAO)

Only boards that use the C3's own USB port are supported: UART0 (GPIO 20 / 21) belongs to the target
(UART bridge, SPI flash), so a board with a separate CH340 / CP2102 chip on it cannot be the host link.

Needs the Arduino core 3.x (ESP-IDF 5) for the RMT driver.

## Wiring

One table for every mode; the wires are shared:

| ESP32-C3 wire | SWS (Telink) | SWD (EFR32, nRF5x, STM32, …) | cJTAG (TI CC13xx / CC26xx) | JTAG | SPI flash | SWIO (WCH CH32V003) | Beken UART (BK7231N / T) | UART bridge | ESP32 / ESP8266 | LN882H UART | RTL8710B UART | RTL8720C UART | UART logger (sniff) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| GPIO 4 –1 k– GPIO 5 (the SWS wire) | SWS | SWDIO | TMSC | TDI | DI (flash pin 5) | SWIO (PD1; SOP8 pin 8) | TX1 (P11; CB2S pad 7), GPIO 4 an input | target TX → flasher RX (GPIO 4 is an input) | U0TXD (TX0) | TX0 (A2), GPIO 4 an input | log TX (PA30), pulled low through GPIO 4 across the restart | log TX (PA16), GPIO 4 an input | line A (GPIO 4 an input) |
| GPIO 0 | – | SWCLK | TCKC | TCK | CLK (pin 6) | – | – | BOOT (`--boot hi` / `lo`, open by default) | GPIO 0 (IO0), held low for the download mode | BOOT (A9), held low for the download mode | – | PA00, held high for the download mode | – (an input) |
| GPIO 21 | – | – | – | TDO | DO (pin 2) | – | RX1 (P10; CB2S pad 5) | flasher TX → target RX | U0RXD (RX0) | RX0 (A3) | log RX (PA29) | log RX (PA15) | – (an input; see below) |
| GPIO 20 | – | – | – | TMS | /CS (pin 1) | – | – | – | – | – | – | – | line B |
| GPIO 6 | RST | nRESET | nRESET | nRESET | RESET of the board's processor, held low | – (not used) | CEN (CB2S pad 10) | RESET | EN (ESP32), EN or RST (ESP8266) | CEN | CEN | CEN | – (an input) |
| GPIO 7 (+ 10, 3, 1) | target power | target power | target power | target power | target power with `--power` only | VDD (SOP8 pin 4), switched for the catch | 3V3 (CB2S pad 1), switched for the catch | target power | 3V3 (a module while it is flashed) | 3V3, switched for the restart | 3V3, switched for the restart | 3V3, switched for the restart | – (inputs: the board has its own supply) |
| GND | GND | GND | GND | GND | GND (pin 4) | VSS (SOP8 pin 2) | GND (CB2S pad 3) | GND | GND | GND | GND | GND | GND |

The SWS wire is one target pin: GPIO 5 goes to it directly, GPIO 4 through a 1 k … 1.8 k resistor.
In the SPI mode the flash's /WP (pin 3) and /HOLD (pin 7) must be high (3.3 V) if the board does not pull them
up, and the reset wire holds the board's own processor in reset, so that it keeps off the bus. In the UART bridge GPIO 0 is the
target's BOOT pin, which is the SWCLK / TCK wire of the other modes. The UART logger listens on the SWS wire and on
GPIO 20, not on the bridge's GPIO 21, which the C3 drives with its boot messages at every power-up (see
[there](#two-line-uart-logger)).

The [Wi-Fi link](#wi-fi-link) takes no wires: the C3's own antenna, and power from its USB connector (a power bank
or a USB charger is enough once Wi-Fi is set up).

Pins and the power-up defaults are set in `src/config.h`. Never put 5 V on the target. Keep the data
wires short (10 cm or less) for the faster settings.

**Power.** A target can get its 3.3 V from the C3's 3V3 pin, from its own supply, or from the power pins
GPIO 7, 10, 3 and 1 (`PWR_PINS`). The power pins switch with one register write, so the flasher can power
the target off and on: every Telink activation switches it off for 300 ms and catches it while it starts,
which is how boards without a reset pad (and firmware that switches SWS off right after start, for example
Tuya relay modules) are reached. One pin gives about 40 mA at its limit, so join two or more of them on the
target's 3.3 V pad; all four give enough for a bare chip board, not for relays or other loads, nor for a
router. Do not connect them to the 3V3 pin, and remove the batteries of a battery device first.

**Reset.** GPIO 6 is open drain, active low; leave it open if the board has no reset pad. In the SPI mode it
holds the board's own processor in reset instead (see there).

**Mains powered modules:** their supply is usually not isolated (logic ground = neutral). Disconnect both
mains wires before connecting the flasher, and power the board from the flasher only.

A pin is only an output while its mode needs it; otherwise it is an input, so a loose wire touching
something does not short it. The power pins stay inputs until the target is powered the first time;
`power-off` switches them off and back to inputs.

## Use

    pip install pyserial

`-p` gives the port (default `/dev/ttyACM0`); on Windows use `py tlsr_c3.py -p COM12 ...`. Without a mode
option the commands talk to a Telink chip; `--swd`, `--cjtag`, `--jtag`, `--spi`, `--wch` and `--bk` select the others. The
commands are the same in every mode where they make sense: `info`, `rf addr len file [--twice]` (read flash),
`wf addr file` (write and verify), `es addr len` (erase), `ea` (erase all), `--run` (restart the target at the
end). `--host NAME` instead of `-p` reaches the flasher over [Wi-Fi](#wi-fi-link) (firmware 0.29), with the same
commands.

## Telink TLSR825x over SWS

Used on these TLSR8258 boards:

| Board | Flash | Reset pad | Done with it |
|---|---|---|---|
| ZY-ZTH03Pro thermometer (Tuya) | 512 k GigaDevice | yes | backup, write, verify |
| Zbeacon TS0001 relay module | 512 k ZBIT | no (power pins) | backup, write, verify, sector erase |
| TS004F rotary knob (ZG-101ZD) | 1 M GigaDevice | no (power pins) | backup, write, pin and LED search |
| TS0052 dimmer module | 1 M | no (power pins) | backup, write, sector erase |
| TS011F plug with ZT2S module (two of them) | 1 M | no (power pins) | backup, sector erase, write |

At the default 1 Mbit/s (RMT mode) a 512 k read takes 9.3 s, a full backup with `--twice` 19.8 s, a 182 k
firmware write with verify 15.3 s. Read works up to 1.6 Mbit/s (`--rate 1500000`); the default leaves more
margin. UART mode works at 921600 baud; above that the first bit of each byte is read wrong (the activation
check refuses such a speed). Read first, write only after a backup.

The SWS line can be driven in two ways, selectable at run time on the same wiring:

| Mode | How | Speed setting | State |
|---|---|---|---|
| `rmt` (default) | The RMT peripheral generates and measures the pulses (12.5 ns steps) | `--rate` SWire bit rate, default 1 Mbit/s | tested up to 1.6 Mbit/s |
| `uart` | One UART byte per SWire bit, a port of pvvx's `TLSR825xComFlasher.py` | `-b` UART baud, SWire bit rate = baud / 10 | tested at 921600 baud (92 kbit/s); the fallback |

The firmware also tries an activation by itself for 3 seconds when the flasher is plugged in
(`AUTO_ACTIVATE_MS`), for a target that gets its power from the 3V3 pin together with the flasher. Whether
this is early enough depends on the target's firmware; the power pins are the reliable way. Boards with a
reset pad can be powered from the 3V3 pin and reset from GPIO 6.

### Commands

    python3 tlsr_c3.py -p /dev/ttyACM0 selftest
    python3 tlsr_c3.py -p /dev/ttyACM0 info
    python3 tlsr_c3.py -p /dev/ttyACM0 rf 0 0x80000 stock.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 wf 0 new_firmware.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 es 0x2D000 0x49000
    python3 tlsr_c3.py -p /dev/ttyACM0 ea
    python3 tlsr_c3.py -p /dev/ttyACM0 --run info

    python3 tlsr_c3.py -p /dev/ttyACM0 --mode uart info

Options:

- `-m rmt|uart` how the line is driven (default `rmt`).
- `--rate bps` RMT mode speed, 30000 … 2000000, default 1000000. The firmware picks the nearest divider
  the chip can do and prints the real bit rate (941 kbit/s for the default, 1.6 Mbit/s for 1500000).
- `-b baud` UART mode speed, 340000 … 5000000, default 921600 (92 kbit/s on the wire). Do not go higher.
- `-t ms` activation time after the reset pulse (default 100; try 300–1000 if the chip does not answer).
- `--burst n` bytes requested in one go; `--burst 1` is the slow, safest read (default 64, up to 256).
- `--run` restart the target's CPU at the end. Without it the chip stays stopped after every command.

`selftest` sends one END byte and checks what the RX pin sees; it works without a target and tells whether
the TX – resistor – RX path is right. `info` prints the chip id (0x5562 for TLSR825x), the program counter
and the flash JEDEC id; the flash size tells how much to read (0x80000 for 512 k, 0x100000 for 1 M).

### Finding the pins of an unknown board

With the chip stopped, its GPIO registers can be read and written over SWS. Two commands use this to map
a board without reading its firmware:

    python3 tlsr_c3.py -p /dev/ttyACM0 pins --seconds 30            # 10 k pull-ups on every pin
    python3 tlsr_c3.py -p /dev/ttyACM0 pins --seconds 3 --pull down
    python3 tlsr_c3.py -p /dev/ttyACM0 led
    python3 tlsr_c3.py -p /dev/ttyACM0 led --only C4

- `pins` makes every pin (except PA7 = SWS) an input and prints each change with its time, about 250
  polls per second: press the button, turn the encoder, close the switch input and read off the pins and
  their order. A pin that reads 1 with pull-ups and 0 with pull-downs is not connected to anything (free
  for a battery measurement, for example); one that stays 1 with pull-downs is tied to the supply, often
  through an LED.
- `led` drives the pins one by one low and then high for a second and prints which one it is at: note the
  line on the screen when the LED lights. `--skip` lists pins that must stay inputs (default `A1,D4,D7`),
  `--only` tests just the given pins three times.

The chip is stopped right after reset, with the clocks of its blocks off; `pins` and `led` switch them on
first (registers `0x60`–`0x65`), otherwise the pull resistors and the input enable of ports B and C, which
live in the analog registers, are not accepted.

### First use

1. `selftest` with only the resistor fitted, no target: checks the TX – resistor – RX path.
2. `info` — proves wiring, activation and the speed. Every activation also writes a test pattern into the
   chip's RAM and reads it back at the speed and burst in use; if that fails, nothing else is done.
   If the chip does not answer: check the resistor and GND, raise `-t`, lower `--rate 500000`, or try
   `--mode uart`.
3. `rf 0 <flash size> stock.bin --twice` — full backup; the second pass compares checksums.
4. Keep the backup safe, then `wf`. It erases the touched sectors, writes and verifies.

### The SWire protocol, and how the two modes map onto it

From Telink's SWire description (in pvvx's TLSRPGM repository) and the working tools:

- A bit is 5 time units: 0 = 1 low + 4 high, 1 = 4 low + 1 high. The line idles high.
- A byte is a command bit, 8 data bits MSB first, and one unit of low.
- A message is START (`0x5a`, command bit set), the address (3 bytes on TLSR825x, 2 on TLSR826x), RW_ID
  (bit 7 = read), the data, and END (`0xff`, command bit set).
- Read: the master sends one unit of low per byte. When the line has then been high for longer than that
  pulse, the slave sends 8 bits and one unit of low, timed by its own clock.
- Slave speed: bit rate = system clock / 5 / `reg 0xb2`. With the divider 5 the chip does about 960 kbit/s.
- `reg 0xb3` bit 7 is the FIFO mode: the address is not incremented, used to stream bytes through the SPI
  data register (`0x0c`; control `0x0d`) to the flash.
- Activation: reset pulse, then "stall CPU" (`0x0602 = 0x05`) repeated so that it lands before the firmware
  can disable SWS or go to sleep.

| | UART mode | RMT mode |
|---|---|---|
| Bit shape | `0x80` = low for 8 of 10 UART bit times, `0xfe` = low for 2 of 10 | exact 1 : 4 and 4 : 1 pulses |
| Speed match | the chip's divider is scanned until the read-back fits the UART frame | the current divider is read at a slow speed, the chip's bit period is measured, the clock is computed and the divider nearest to the requested rate is set |
| Read decode | the UART samples each slave bit at a fixed point, so the slave must run at baud / 10 | each bit is 1 when its low part is longer than its high part, any slave speed |
| Bulk read | start pulses sent one by one with a timed gap, up to 256 bytes per burst | up to 256 bytes per burst, 10 pulses per byte; the driver empties the channel memory in halves while receiving, a lost symbol fails the burst |
| Long writes | one UART stream | pieces of 9 bytes queued back to back, cut on byte boundaries |
| Echo check | yes, every write is read back | only in `selftest` |

### Flash access, checked against the chip's own flash code

Compared with `flash.c` / `spi_i.h` of pvvx's loader (Telink's reference routines) and `TlsrPgm.py`:

| Step | Reference | Here |
|---|---|---|
| Commands | 0x06 write enable, 0x02 page program, 0x03 read, 0x05 status, 0x20 sector erase (4 k), 0x60 chip erase, 0x01 write status | same |
| Page program | at most 256 bytes, inside one page | same; the host cuts on page boundaries |
| Read | command, address, a dummy byte, control `0x0a` (auto read), then the data register | same |
| Status | command, then a **dummy byte** to clock the answer in, then the data register | same. pvvx's SWire script reads without the dummy byte, so its "ready" check may be a constant |
| Unlock | write enable, write status 0, wait, status must read 0 | same, plus a check that the write-enable latch follows 0x06 / 0x04; without it nothing is erased or written |
| After a page program | poll the busy bit | 2 ms fixed wait, then poll |

Cautions that come from the layout of Telink / Tuya firmware (from memory of the Telink SDK, not confirmed
by the sources read: check them in your own dump):

- The top sectors hold per-device data: the MAC address and the factory RF calibration (512 k flash:
  0x76000 / 0x77000, 1 M flash: 0xFF000 / 0xFE000). `ea` wipes them. `wf` only erases the sectors the
  file covers, so a firmware written at address 0 leaves them alone.
- To find them, search the backup for the device's IEEE address in reversed byte order.
- pvvx's advice for these thermometers: write the BLE firmware by wire first (smaller, address 0), then
  switch to Zigbee from his web flasher over BLE.

## Arm SWD

    python3 tlsr_c3.py -p /dev/ttyACM0 --swd info
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd rf 0 0x40000 backup.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd wf 0 firmware.bin --run
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd es 0x3e000 0x2000
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd ea

`info` reads the IDCODE, the access ports, the CPU type and, for Silicon Labs EFR32 (Series 1 and 2) and
Nordic nRF52, the device information (part, flash size, page size). `--swd-khz` sets the clock (default 1000).
For a Puya PY32F002B-type chip it prints the DBGMCU IDCODE, the flash size (found from the first kbyte that answers
with a bus fault) and the option bytes.

The flash routines run on the flasher (`SFAM`, `SFE`, `SFW`, `SFX`, `SFC`): the core is stopped at its reset
vector, the touched pages are erased, the data goes in 512-byte pieces, and the verify compares CRCs computed
on the flasher. Register maps from the Silicon Labs device headers and Nordic's NVMC description:

| Chip | Flash controller | Page | Mass erase (`ea`) |
|---|---|---|---|
| EFR32 Series 1 (MG1, MG12, MG13, MG14) | MSC at 0x400E0000, word by word with WRITEONCE | 2 k (from DEVINFO) | ERASEMAIN0 |
| EFR32 Series 2 (xG21 … xG29) | MSC at 0x40030000, WDATA stream, WRITEEND; MSC clock bit 16 or 17 in CMU_CLKEN1 per family (none on xG21) | 8 k | page by page |
| nRF52 | NVMC at 0x4001E000 | 4 k | CTRL-AP ERASEALL (also clears APPROTECT) |
| nRF5340 application core | NVMC at 0x50039000; a page is erased by writing 0xFFFFFFFF into it | 4 k | CTRL-AP (AP 2) ERASEALL |
| SAM D2x / L2x / C2x | NVMCTRL at 0x41004000: page buffer, WP; ADDR in half-words | 64 B pages, 256 B rows | DSU chip erase (also clears the security bit) |
| SAM D5x / E5x | NVMCTRL at 0x41004000: page buffer, WP | 512 B pages, 8 k blocks | DSU chip erase |
| STM32 F0, F1, F3 | FLASH at 0x40022000, half-word programming | 1 k / 2 k | MER |
| STM32 F4 (F401 … F446, F42x/43x with two banks) | FLASH at 0x40023C00, 32-bit programming (needs 2.7–3.6 V) | 16 k / 64 k / 128 k sectors | MER (+ MER1) |
| STM32 G0, G4, L4, WB, C0 | FLASH at 0x40022000, double-word programming, second bank from the option bytes | 2 k / 4 k | MER1 (+ MER2) |
| STM32 L0, L1 | PECR at 0x40022000 / 0x40023C00, word programming; a page is erased by writing a word with ERASE + PROG | 128 / 256 B | page by page |
| STM32 U5 | non-secure registers (NSCR) at 0x40022000, quad-word programming, two banks | 8 k | MER1 + MER2 (TrustZone off only) |
| RP2040 | external QSPI flash, written by the boot ROM's own routines run on core 0 | 4 k | – (give the range to `es`) |
| Puya PY32F002B type: PY32F002B, MS32C001 (power stage of Tuya dimmers); **tested on an MS32C001** | FLASH at 0x40022000: page buffer (PG, 31 words, PGSTRT, the 32nd word); erase / program timing registers loaded from the factory bytes at 0x1FFF011C (0x1FFF0130 for HSI_FS = 5), as Puya's HAL does | 128 B | MER (main flash only) |

STM32 addresses may be given from 0 (taken as 0x08000000), RP2040 ones from 0 (taken as 0x10000000). The
RP2040 is reached through its multi-drop port (core 0); `info` cannot read the size of its external flash,
so `rf` needs it given. While its flash is written, core 1 is switched off; `--run` restarts the chip with the
watchdog. STM32 F7, H7 and the F1 XL-density parts have no flash routine yet. GD32 and CH32 parts that report an STM32F1 device ID are written with the F1 routine, up to 512 k
(the second bank of the bigger GD32F10x / F30x has its own registers, not handled); GD32E23x is only
identified (word programming). After a mass erase of an nRF5340 the flasher writes UICR.APPROTECT and
SECUREAPPROTECT to 0x50FA50FA, otherwise current silicon locks the debug port again at the next reset; the
firmware written there must keep it open as well (nrfx `nrf53_handle_approtect`).

A debug-locked EFR32 shows only its AAP and cannot be read or written: Series 1 opens with the AAP mass
erase, Series 2 through the Secure Engine; neither is implemented yet. A read-protected STM32 (RDP) is
reported by `info`; removing the protection erases the chip and is not implemented either.
A Puya chip is recognised by its DBGMCU IDCODE (0x20220064 is the PY32F002B, and the MS32C001 reports the same), or, with an unknown IDCODE, by its flash controller and the flash timing in its factory configuration
bytes; the PY32F002A / F003 / F030 (IDCODE 0x60001000) have another flash controller and are not handled. The
option bytes are never written. If the option bytes start the watchdog in hardware mode, it gets its longest
time-out (about 32 s) before each operation, as Puya's Keil algorithm does. The routine follows Puya's HAL and
Keil flash algorithm for the PY32F002B and is checked against a simulated chip only (`tests/py32_sim`).

**On hardware (2026-11-24): Puya MS32C001L12S6**, the SOP8 power-stage MCU of a Tuya TS0052 Zigbee dimmer
(pads DIO / CLK / 3.3 V / GND on the board; SOP8 pin 1 VCC, 2 SWDIO = PB6, 3 SWCLK = PA2, 8 GND):

- `info`: SW-DP IDCODE 0x0bc11477, AHB-AP IDR 0x04770031, Cortex-M0+ (CPUID 0x410cc601), DBGMCU IDCODE
  0x20220064, option bytes 0x2f55d0aa. The flash probe reports 24 kbytes, although the datasheet says 18: the chip
  is a PY32F002B inside, or the range above 18 k still answers; nothing above 18 k was written.
- `rf 0x08000000 0x4800 ... --twice`: the stock firmware (18432 bytes, readable, no read protection) read twice
  with identical results.
- `wf 0x08000000 ... --run` (6.4 to 7.2 kbytes, about ten times that day): erase, program through the page buffer,
  CRC verify and restart all worked, about 3 s for 7 k; the new firmware ran at once, and its RAM counters were
  read back with `rf 0x20000000 ...` while it ran.
- `info` and `rf` do not stop the core; the stock firmware leaves SWD on its pins.

The other SWD targets in the table are still untested on hardware.

## TI CC13xx / CC26xx over two-wire cJTAG

These chips have no SWD: their debug port is IEEE 1149.7 cJTAG on two pins (TMSC, TCKC) behind TI's ICEPick
router. The same two wires are used as for SWD: TMSC on the SWS wire (GPIO 4 – 1 k – GPIO 5), TCKC on
GPIO 0, plus GND and, if wanted, the reset pin.

    python3 tlsr_c3.py --cjtag info
    python3 tlsr_c3.py --cjtag rf 0 0x58000 backup.bin
    python3 tlsr_c3.py --cjtag wf 0 firmware.bin --run

The flasher brings the TAP online in the OScan1 format (escape sequences and activation code as in
OpenOCD's `ftdi.c`), enables the Cortex-M debug TAP behind the ICEPick (the router writes of OpenOCD's
`icepick_c_tapenable`, port 0) and then uses that TAP's JTAG-DP with the same memory / core layer as SWD.
The chip is identified from FCFG1 (ICEPick id, user id) and the flash size register: CC26x0, CC26x1, CC13x0
(4 k sectors), CC26x2 / CC13x2 (8 k sectors). Writing runs the boot ROM's own routines (HAPI table at
0x10000048: sector erase, program) on the halted core, sector by sector; what an image does not cover
inside a sector is read first and written back. The core is halted without a reset, because a system reset
drops the cJTAG link.

The last sector ends with the CCFG, which among others enables the debug port (`CCFG_TAP_DAP_0`,
`CPU_DAP_ENABLE` = 0xC5). An image whose CCFG would switch it off is refused, `ea` erases everything but that
sector, and `es` over the last sector keeps the CCFG. CC13x4 / CC26x4 (Cortex-M33, different ROM) and
CC2538 are not handled. Not tested on hardware: the OScan1 timing, the ICEPick sequence on these chips and
the ROM calls are all from the sources above.

## Four-wire JTAG

| JTAG | Flasher wire |
|---|---|
| TCK | SWCLK wire (GPIO 0) |
| TMS | GPIO 20 (its own wire) |
| TDI | SWS wire (GPIO 4 drives it through the 1 k resistor) |
| TDO | UART bridge TX wire (GPIO 21), an input here |
| nRESET | reset wire (GPIO 6), as for SWD |

    python3 tlsr_c3.py --jtag info                     # scans the chain
    python3 tlsr_c3.py --jtag rf 0 0x10000 backup.bin  # an Arm chip: everything as over SWD

`info` lists the chain from the TDO side (IDCODE, maker, total instruction length). When it holds an Arm
JTAG-DP (IDCODE `xBA0x477`, 4-bit IR), the flasher puts every other device in BYPASS and uses that debug port
with the same memory, core and flash code as SWD, so `rf`, `wf`, `es`, `ea` and `--run` work for the chips
in the SWD table. The other devices' instruction lengths must be known: the STM32 boundary-scan TAP (5 bits)
is, and a single unknown device gets the rest of the total. Before the scan the flasher sends the SWD-to-JTAG
sequence on TMS, so an SWJ-DP left in SWD mode answers too. A chain without an Arm DAP is only listed; `JIR`
and `JDR` do raw scans for anything else (RISC-V debug, EJTAG, boundary scan are not built on them yet).
Tested against a simulated chain (`tests/jtag_sim`), not on hardware yet.

## SPI NOR flash, e.g. a router's

The 25-series SPI flash of a router, a dongle or any other board:

| Flash pin | Flasher wire |
|---|---|
| /CS (1) | GPIO 20 (the JTAG TMS wire) |
| DO (2) | UART bridge TX wire (GPIO 21) |
| DI (5) | SWS wire (GPIO 4 drives it through the 1 k resistor) |
| CLK (6) | SWCLK wire (GPIO 0) |
| GND (4) | GND |
| /WP (3), /HOLD (7) | to 3.3 V if the board does not pull them up |
| – | reset wire (GPIO 6) to the RESET of the board's processor, if there is one |

    python3 tlsr_c3.py --spi info
    python3 tlsr_c3.py --spi rf 0 0 backup.bin --twice        # size 0 = the whole chip
    python3 tlsr_c3.py --spi wf 0x50000 firmware.bin --unprotect
    python3 tlsr_c3.py --spi --power info                     # a flash out of circuit, powered by the flasher

The size and the erase unit come from the chip's SFDP table (the id byte when it has none); chips over
16 Mbytes are used with the 4-byte opcodes. `wf` works erase unit by erase unit: a unit whose CRC already
matches is skipped, and what the image does not cover inside a unit is read first and kept, so a bootloader
or a calibration partition next to the written range stays. `rf --twice` checks the read with a second pass
computed on the flasher (CRC), and `wf` verifies the same way. Block protection (BP bits) is reported by
`info`; `--unprotect` clears it and keeps the quad-enable bit. Status registers 2 / 3 are only read on
Winbond and GigaDevice chips: on Macronix and ISSI the same command (0x35) switches the chip to QPI.

In circuit, the board's own processor drives the same pins. While the SPI mode is on, the reset wire holds it
in reset (open drain, released at the end, also after an error or Ctrl-C), so that its pins let go of the
flash. Without `--power` the board runs from its own supply; that is the way for a router, whose current the
power pins cannot give, and which would be powered backwards through the flash otherwise. `--power` switches
the power pins on after the reset is down and off at the end: for a flash out of circuit (adapter, socket) or
a small module. Clock 4 MHz by default (`--spi-khz`, the DI wire goes through 1 k); the read and write speeds
are printed (binary streams, see the host protocol). Not tested on hardware yet.

## WCH CH32V003 over SWIO

The CH32V003 has a one-wire debug interface, which WCH calls SDI and whose pin is SWIO (PD1). It goes on the
SWS wire, and the target's VDD on the power pins, so that the flasher can switch it off and catch it while it
starts:

| CH32V003J4M6 (SOP8) pin | Flasher wire |
|---|---|
| 8: PD1 = SWIO, bonded together with PD4 and PD5 (UART TX) | SWS wire: GPIO 5 directly, GPIO 4 through 1 k |
| 4: VDD | power pins GPIO 7 (+ 10, 3, 1) |
| 2: VSS | GND |
| 1 (PD6 / PA1, UART RX), 3 (PA2), 5 (PC1), 6 (PC2), 7 (PC4) | – |

In the other packages SWIO is PD1 alone: pin 18 of the TSSOP20 (F4P6); after the pinout drawings linked from
the ch32v003fun README, pin 7 of the SOP16 (A4M6) and pin 15 of the QFN20 (F4U6); check the datasheet before
soldering. The reset pin (PD7, not on the SOP8) is not used.

**These dimmer and switch boards are not isolated from the mains.** Disconnect both mains wires before
connecting the flasher, and power the board only from the flasher.

    python3 tlsr_c3.py --wch info
    python3 tlsr_c3.py --wch rf 0 0x4000 backup.bin --twice
    python3 tlsr_c3.py --wch wf 0 firmware.bin --run
    python3 tlsr_c3.py --wch es 0x3c00 0x400
    python3 tlsr_c3.py --wch ea
    python3 tlsr_c3.py --wch unlock --yes-erase-all --save-options options.bin
    python3 tlsr_c3.py --wch wf 0x1FFFF800 options.bin --write-options
    python3 tlsr_c3.py --wch --external-power info

**Why pin 8 needs the catch at power-up.** On the SOP8 one pad carries PD1, PD4 and PD5. Once the firmware
switches its UART on, PD5 drives the pad high as TX, and SWIO is gone: the flasher's pulses through 1 k do
not move it (`SWIO never went low`). After a reset, though, every pin is an input and PD1 is SWIO. So every
command starts the same way: SWIO is held low and the power pins are switched off for `--wch-off` ms (300),
then on again, and from the first microsecond on the flasher repeats, for up to `--wch-window` ms (100), two
halt requests, the debug output enable, two more halt requests and a read of the debug module status. The
first sensible status means the core is halted; it stays halted until the end of the command, or `--run`
restarts it. A halt request goes out about every 0.2 ms, so a firmware that takes the pin over sooner than
that after its start is not caught; the error then says so.

With `--external-power` the flasher does not switch the power: only a target whose firmware leaves SWIO
alone can then be reached. A long window (`--external-power --wch-window 10000`) leaves time to switch an
external supply on by hand while the flasher is trying. The power pins give about 40 mA each at their limit;
join two or more on VDD, as for the other modes.

**What the commands do.**

- `info`: the chip id (CHIPID at 0x1FFFF7C4 and the debug register 0x7f), the flash size (0x1FFFF7E0), the
  unique id, FLASH_OBR (read protection) and FLASH_WPR (write protection), and the 16 option bytes decoded.
- Addresses: the user flash at 0x08000000 (an address below 0x01000000 is taken from there; 0 = 0x08000000),
  16 k on the V003; the boot area at 0x1FFFF000, 1920 bytes (unlocked with its own key); the option bytes at
  0x1FFFF800; RAM at 0x20000000. `rf` reads any of them, and registers too.
- `wf` works in 64-byte pages: pages that already hold the data are skipped, what the image does not cover
  inside a page is read first and kept, the rest is erased and written with the fast page programming
  (16 words into the page buffer, then one program command), and verified by CRCs computed on the flasher.
- `es` erases the 64-byte pages of the range, `ea` the whole user flash (mass erase; the boot area and the
  option bytes stay).
- Before `wf`, `es`, `ea` and `unlock` the option bytes are printed; `--save-options FILE` keeps a copy.
- `wf 0x1FFFF800 file --write-options` writes the 16 option bytes (RDPR, USER, DATA0, DATA1, WRPR0..3, each
  followed by its complement): the complements are checked, and an RDPR other than 0xA5, which switches the
  read protection on, is pointed out first. They take effect at the next reset.
- `--run` resets the chip and lets it run; its firmware may take the pin again, the next command catches it.

**Read protection.** An RDPR other than 0xA5 protects the flash: `info` shows `READ PROTECTED`, the debug port
does not return the flash contents, and `rf`, `wf`, `es` and `ea` refuse to start. `unlock --yes-erase-all`
writes the option bytes with RDPR = 0xA5 (USER and DATA kept, write protection off); the chip then **erases
its whole user flash**. There is no way to read the firmware of a protected chip first. The flasher restarts
the target (a power cycle) to load the new option bytes, checks that the protection is gone and that the
flash is blank, and erases it if it is not.

**The protocol in brief.** From WCH's QingKe V2 debug manual (chapter 2, transport; chapter 3, debug
module) and WCH's one-wire host example for a CH32F103:

- The line idles high and every bit starts with a falling edge. A packet is a start bit 1, the 7-bit register
  address, 1 = host writes / 0 = host reads, and 32 data bits, MSB first; a high of 18T or more ends it.
  To read, the host sends a short low pulse per bit and the target keeps the line low for a 0.
- After reset the interface runs in "normal mode 2x" with the debug clock of the HSI divided by 3 (8 MHz,
  T = 125 ns); it needs the HSI running. A low of 256T or longer resets the interface.
- The target answers only after the output enable: 0x5AA50400 into SHDWCFGR (0x7E), then into CFGR (0x7D).
  CPBR (0x7C) then reads 0x00010403.
- The debug module follows the RISC-V debug specification 0.13: dmcontrol 0x10 (halt request 0x80000001,
  system reset 0x80000003, resume 0x40000001), dmstatus 0x11, abstractcs 0x16, command 0x17, abstractauto
  0x18, data0 / data1 0x04 / 0x05, eight program buffer words 0x20..0x27. Abstract commands reach the core
  registers only; memory goes through small programs in the program buffer. With abstractauto set every
  access to data0 runs the program again, so a block costs one register access per word.
- The flash controller at 0x40022000 is opened with the keys 0x45670123, 0xCDEF89AB in KEYR and in MODEKEYR
  (fast mode); BOOT_MODEKEYR opens the boot area, OBKEYR the option bytes. Page erase (64 bytes): CTLR =
  PAGE_ER, ADDR, CTLR = PAGE_ER | STRT. Programming: CTLR = PAGE_PG | BUF_RST, then per word a store into the
  page and CTLR = PAGE_PG | BUF_LOAD, then ADDR and CTLR = PAGE_PG | STRT; BSY in STATR after each step.
  Mass erase: MER, then MER | STRT. Option bytes: OPTER (+ STRT), then OPTPG and half-word stores.

| | Debug manual, normal mode (T = 125 ns) | Here (default; `--wch-timing one,zero,high,slot,threshold,stop`) |
|---|---|---|
| 1 | low T..4T (125–500 ns), high T..16T | low 250 ns, high 375 ns |
| 0 | low 6T..64T (0.75–8 µs), high T..16T | low 1000 ns, high 375 ns |
| stop | high 18T (2.25 µs) or longer | 4 µs |
| read bit | not given; the target holds the line low for a 0 | 250 ns pulse every 1.5 µs; a low longer than 550 ns is a 0 |
| interface reset | low 256T (32 µs) or longer | 100 µs |

The pulses come from the RMT peripheral (12.5 ns steps), as in the Telink mode: GPIO 4 drives the line through
the 1 k resistor, so the high level is a strong pull-up the target can pull down, and GPIO 5 measures the
length of every low pulse, ours and the target's. A packet is one RMT transmission that fits into the channel
memory, so no interrupt can stretch a bit in the middle of it. Bit-banging with interrupts off would meet the
timing as well, but the RMT measures the answer instead of sampling it at one instant. The defaults stay
inside the manual's windows for a debug clock between about 6 and 16 MHz; the simulation checks them at
±10 % of 8 MHz.

Written from the sources in the credits and checked against a simulated chip (`tests/wch_sim`,
`tests/test_wch_host.py`); nothing of it has run on a real CH32V003 yet.

## Beken BK72xx (CB2S, CB3S, WB2S …) over UART

The Beken BK7231N (Tuya CB2S, CB3S, CBU, …) and BK7231T (WB2S, WB3S, …) are flashed through their UART1 with
Beken's download protocol. The flasher speaks that protocol itself, as ltchiptool and bk7231tools do on a PC: it
catches the chip at power-up, links, raises the speed, and reads, erases and writes the flash, checking everything
with the CRC the chip computes. No other tool is needed (the [UART bridge](#uart-bridge) with `--catch bk` is the
older way, through ltchiptool).

**On hardware (2026-11-25): a CB2S (BK7231N) in a Tuya FS-05R Wi-Fi dimmer.** The flasher caught the chip at
power-up on its own, read the whole 2 MB flash (bk7231tools unpacked the bootloader, the application and the
encrypted Tuya storage from it without an error), and wrote an ESPHome (LibreTiny) `.uf2` with `wf 0 … --run`,
which booted and has been updated over the air since. Powered from the flasher's switched power pins alone, the
module also started Wi-Fi and joined the network. Not tried yet: a BK7231T, and the bootloader area
(`--force-bootloader`).

| CB2S pad | Signal | Flasher wire |
|---|---|---|
| 7: TX1 (P11) | the chip's TX | SWS wire: GPIO 5 directly (GPIO 4, behind the 1 k, is an input here) |
| 5: RX1 (P10) | the chip's RX | GPIO 21 |
| 10: CEN | reset, active low | GPIO 6 (reset wire, open drain) |
| 1: 3V3 | supply | power pins GPIO 7 (+ 10, 3, 1), joined; or the 3V3 pin |
| 3: GND | | GND |

Other modules have the same signals on other pads: look them up in the module's datasheet. With the power pins the
flasher switches the module off and on for every catch; join all four on 3V3, as the chip draws more than one pin
gives once its own firmware starts Wi-Fi. A module on the C3's 3V3 pin is restarted through CEN alone.

**Another chip on UART1.** On boards like the Tuya FS-05R dimmer the module's UART1 is also wired to another chip
on the board. Its TX fights the flasher's, and it may answer or print: disconnect it before flashing (lift the
module, cut or lift the track, or keep that chip unpowered). **These boards are not isolated from the mains**:
disconnect both mains wires and power the board only from the flasher.

    python3 tlsr_c3.py --bk info
    python3 tlsr_c3.py --bk rf 0 0 backup.bin --twice          # size 0 = to the end of the flash
    python3 tlsr_c3.py --bk wf 0 firmware.uf2 --run            # ESPHome / LibreTiny
    python3 tlsr_c3.py --bk wf 0x11000 OpenBK7231T_UA_<version>.bin
    python3 tlsr_c3.py --bk es 0x1D3000 0x8000
    python3 tlsr_c3.py --bk ea
    python3 tlsr_c3.py --bk --bk-baud 460800 --bk-seconds 20 info

**The catch.** The BK7231N's BootROM looks for a link check (`01 E0 FC 01 00`) on UART1 only for a moment after
reset, then starts the firmware in flash. So every command starts by pulling CEN low and switching the power pins
off for 300 ms (GPIO 21 is an input meanwhile, so that it does not feed the unpowered chip), and from the moment the
power is back the flasher sends link checks back to back (0.43 ms each at 115200 baud) until the answer
`04 0E 05 01 E0 FC 01 00` comes. Without an answer it resets again every 1.2 s, for `--bk-seconds` (10). If that
fails, the error says what came back: nothing at all (wiring, power), or bytes of the chip's own firmware (`info`
prints the start of them): then CEN and the power are not under the flasher's control, or another chip is on UART1.
Without CEN and power wired, short CEN to GND for a moment while the flasher tries.

**The protocol and the speed.** At 115200 baud the flasher reads the first 4 k of the application (0x11000) and
asks the chip for the CRC of 256 bytes of it: the BootROM's CRC includes the end address (257 bytes), the BK7231T's
bootloader's does not, which tells the two protocols apart. Then it sends the speed command (`--bk-baud`, default
921600): the chip answers 50 ms later at the new speed, a link check must pass, and the same 4 k read again must
equal the first. If any of it fails, the flasher returns to 115200 (with the chip's own speed command, or by
catching it again) and says so. A read answer carries no checksum, so every 4 k block of `rf` is compared with the
chip's CRC of it and read again (up to four times) when they differ. Expect about 30–40 s for 2 Mbytes at 921600
(estimated, not measured yet).

**The commands.**

- `info`: protocol and speed, the chip id (register 0x800000: 0x7231C is the BK7231N), the bootloader (by the CRC of
  its first bytes, in bk7231tools' list), the flash's JEDEC id, size and status register with its protection bits,
  and the MAC address from the RF calibration (BK7231N 0x1D0000, BK7231T 0x1E0000, BK7238 in the Tuya T1 layout
  0x1E3000).
- `rf addr size file`: `size 0` reads to the end of the flash. `--twice` adds the chip's CRC of the whole range.
- `wf addr file`: clears the flash protection, compares by CRC (per 64 k, then per 4 k) and skips sectors that
  already hold the data, keeps what the file does not cover inside its first and last sector (read first), erases
  64 k blocks where a whole one is covered and 4 k sectors elsewhere, writes in 4 k units and verifies with the
  chip's CRC of each part.
- `es addr size`: erases the 4 k sectors of the range (64 k blocks where possible).
- `ea`: erases everything from 0x11000 to the end; the bootloader stays. That includes the RF calibration with the
  MAC address and the Tuya configuration: have a backup.
- `--run`: the reboot command (`0E A5`): the chip starts the firmware in flash.

**The bootloader guard.** 0x0–0x11000 holds the bootloader. `wf` and `es` refuse anything there unless
`--force-bootloader` is given, and the flasher's firmware refuses it as well (`BKG`). On the BK7231N the download
code is in its ROM, so UART flashing keeps working, but a wrong bootloader keeps the device from starting until the
backup is written back. On the BK7231T the download code *is* that bootloader: writing it is refused even with
`--force-bootloader`; a broken one leaves only SPI flashing. A file written at 0 that is longer than 0x11000 (a full
image such as OpenBeken's `_QIO_` file, or a backup) is written from 0x11000 on; the flasher says whether its
bootloader part equals the chip's.

**Flash protection.** Before an erase or write the BP0–BP4 bits (and CMP, on flash chips with two status bytes) are
cleared, as ltchiptool does: the number of status bytes comes from bk7231tools' list of the flash chips found on
these modules (by JEDEC id); for a Macronix chip only BP0–BP3 are cleared, its bit 6 is the quad enable. A flash that
is not in the list is used if it shows no protection bits, otherwise `--bk-sr 1|2` gives its number of status
bytes. The protection is not set again afterwards, as with ltchiptool.

**Image formats.**

| File | From | Give it to `wf` as |
|---|---|---|
| `firmware.uf2` | ESPHome / LibreTiny build | `wf 0 firmware.uf2`: the parts meant for UART flashing are taken out with their offsets (the application at 0x11000 and its RBL header near the end of that partition); the OTA part is left out |
| `image_bk7231n_app.0x011000.rbl` | ESPHome / LibreTiny build directory | `wf 0x11000 file` |
| `OpenBK7231N_QIO_<version>.bin` | OpenBeken release | `wf 0 file`: a full image, written from 0x11000 (see the guard) |
| `OpenBK7231T_UA_<version>.bin` (a `_UA_` file) | OpenBeken release | `wf 0x11000 file` |
| `backup.bin` from `rf 0 0` | this flasher | `wf 0 backup.bin`: everything but the bootloader |
| `*_app.ota.rbl`, OpenBeken `.rbl`, `*.ug.bin` | OTA packages | refused: they are for updates over the air |

A UF2 is read as ltchiptool's uf2tool writes it (OTA format 2): every data block's address is relative to the
partition that the image's `OTA_PART_INFO` tag names for the "flasher" scheme, and the partitions' offsets come from
the `FAL_PTABLE` tag of the file's header block. The UF2 family must match the chip (BK7231N 0x7B3EF230, BK7231T
0x675A40B0).

**The protocol in brief.** From ltchiptool and bk7231tools (packets, linking, the flash and chip commands) and
OpenBeken's BK7231GUIFlashTool (its BK7231 flasher, for the details of the answers and the flash list):

- Command: `01 E0 FC <n> <code> <payload>` (n = 1 + payload), or long: `01 E0 FC FF F4 <len:u16> <code> <payload>`
  (len = 1 + payload). Answer: `04 0E <n> 01 E0 FC <code> <payload>` (n = 4 + payload), or long:
  `04 0E FF 01 E0 FC F4 <len:u16> <code> <payload>`. Little endian.
- Short commands: 0x00 link check (answer code 0x01, one byte 0); 0x03 read register (address → address, value);
  0x0F set baud rate (baud:u32, delay_ms:u8; the same 5 bytes come back after the delay, at the new speed; the host
  switches after half the delay); 0x10 CRC (start, end → CRC-32 with the zlib polynomial, started at 0xFFFFFFFF and
  not inverted at the end, so the zlib CRC is its complement); 0x0E reboot (0xA5, no answer).
- Long commands: 0x09 read 4 k (address → status, address, 4096 bytes); 0x07 write 4 k (address, 4096 bytes →
  status, address); 0x0F erase (0x20 = 4 k sector or 0xD8 = 64 k block, address → status, size, address);
  0x0C read a flash status register (opcode 0x05 / 0x35 / 0x15 → status, opcode, value); 0x0D write it (0x01 and
  one or two bytes → status, opcode, bytes); 0x0E flash id (0x9F as u32 → status, one byte, the three JEDEC bytes;
  some BootROM revisions give a length one short).

| | BK7231N | BK7231T |
|---|---|---|
| Download code | BootROM | the bootloader in flash (0x0–0x11000) |
| Chip id (register 0x800000) | 0x7231C | no register access |
| CRC range | end address included | end address excluded |
| Flash id, status registers | yes | no (the bootloader handles the protection) |
| Reading 0x0–0x11000 | directly | refused; the same bytes at the address plus the flash size (2 Mbytes) |
| After a CRC | – | the flash is write protected until the next link check (the flasher sends one) |
| RF calibration, MAC | 0x1D0000 | 0x1E0000 |
| Overwriting 0x0–0x11000 | the device does not start; UART flashing still works | UART flashing is gone |
| Leaves the download mode | only on reboot | also when the host stays silent too long |

Written from the sources in the credits and checked against a simulated BootROM / bootloader (`tests/bk_sim`,
`tests/test_bk_host.py`); nothing of it has run on a real chip yet.

## ESP32 / ESP8266 over UART

    python3 tlsr_c3.py -p /dev/ttyACM0 --esp info                       # chip, MAC, flash id
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp rf 0 0 backup.bin --twice  # whole flash (esptool 5)
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp rf 0 0x400000 backup.bin   # esptool 4: give the size
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp wf 0 firmware.factory.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp es 0x9000 0x5000           # erase a range (whole sectors)
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp ea                         # erase the whole flash

Needs esptool on the PC (`pip install esptool`, version 4 or 5). The flasher restarts the target (reset wire and
power pins) with its GPIO 0 held low by the BOOT wire, which starts the ROM's serial download mode, and opens the
[UART bridge](#uart-bridge); esptool then does the work through the bridge, with `--before no_reset --after
no_reset`, because a DTR / RTS reset would reset the C3 itself. At the end the bridge is closed and the target
restarted with GPIO 0 released, so it runs its firmware (`--esp-stay` leaves it in download mode). `wf`, `es` and
`ea` erase what they need and esptool checks the MD5 of what it wrote; `rf --twice` reads twice and compares.

The C3's USB port does not pass on the host's speed setting, so the bridge follows esptool instead (firmware
0.25): it starts at 115200, where esptool syncs with the ROM and loads its stub, and when esptool asks for its
working speed (`--esp-baud`, default 921600) with CHANGE_BAUDRATE, the bridge reads that command and switches its
own UART once the target has answered. If a run fails, it is repeated at 460800 and then 115200, with the target
restarted in download mode each time; the speed that worked is printed and kept for the next run (`rf --twice`).
Firmware older than 0.25 runs the bridge at `--esp-baud` from the start (default 115200 there) and the ROM finds
that speed from esptool's first sync. `--esp-chip esp8266` (or esp32, esp32c3 …) skips the detection.

Wiring (see [Wiring](#wiring)): the target's TX0 to the SWS wire, its RX0 to GPIO 21, its GPIO 0 to the BOOT
wire (the C3's GPIO 0, the SWCLK wire of the other modes), EN (ESP8266: EN or RST) to the reset wire, GND. The
strapping pins must allow the download mode as on any ESP board: on a bare ESP8266 module GPIO 15 low, GPIO 2
high and EN high (ESP-12: 10 k resistors), on an ESP32 GPIO 2 low or open. Power: the power pins carry an
ESP8266 or a small ESP32 module while it is flashed, not its Wi-Fi peaks. This mode is for bare modules and
devices (plugs, switches, sensors); a development board with its own USB–UART chip is flashed through that, as
its chip drives the same RX0 line.

## Lightning LN882H over UART

    python3 tlsr_c3.py -p /dev/ttyACM0 --ln info                        # ROM, flash id, size, status, MAC from the OTP
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln rf 0 0 backup.bin --twice   # whole flash, then the chip's CRC of it
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln wf 0 firmware.uf2 --run     # LibreTiny / ESPHome UF2
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln wf 0 backup.bin             # a full image from 0
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln es 0x1a0000 0x1000          # erase a range (whole sectors)
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln ea                          # erase the whole flash
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln --ln-loader vendor info     # with Lightning's RAM loader instead

Firmware 0.26. The flasher restarts the chip (reset wire and power pins) with its BOOT pin (GPIOA9) held low by the
BOOT wire, which starts the ROM's download mode on UART0, and opens the [UART bridge](#uart-bridge); the rest runs in
`tlsr_c3.py` through the bridge. The ROM takes text commands at 115200 and loads a RAM loader into the chip's RAM over
YMODEM (`download [rambin] [0x20000000] [size]`). `baudrate N` raises the loader's speed: the bridge reads the host's
bytes for that line and switches its own UART at the line's end (`src/lnscan.h`), so the host keeps the speed
(`--ln-baud`, default 1000000; when that fails, a new session at 460800, then 115200).

Two RAM loaders (`--ln-loader`):

- **`own`, the default: ours** (`loaders/ln882h`, 6 kbytes, built into `tlsr_c3.py`, nothing to download). It talks
  in binary frames with a CRC-32 both ways, reads 4 k per command, computes the CRC-32 of any range on the chip, erases
  only whole 4 k sectors or 64 k blocks it is asked for and checks them blank, writes pages without erasing (and
  refuses to write over bits that are not erased), reads every write back, and reads and writes the flash's status
  registers. Not working on a chip yet: the first try failed (see the checklist below).
- **`vendor`: Lightning's** (as ltchiptool ships it). Not part of this repository: download it once as
  `ln882h_ramcode.bin` next to `tlsr_c3.py`
  (<https://github.com/libretiny-eu/ltchiptool/raw/410874f66b60da1dd667dbba69f499acbf16eb03/ltchiptool/soc/ln882h/util/ramcode.bin>,
  37872 bytes, SHA-256 `6bd437c6…a0c723`), give it with `--ln-ramcode`, or have ltchiptool installed: its copy is
  found. It answers text commands (`flash_info`, `flash_uid`, `get_mac_in_flash_otp`, `flash_read` of up to 256 bytes
  in hex with a CRC-16, `ferase`, `ferase_all`, and `startaddr` + `upgrade`, after which it takes the data over YMODEM,
  erases the range and writes it). It rounds the length up to whole 4 k sectors but not the start, so the host always
  sends whole sectors (what is around the range in them is read first and sent along), reads the range back to verify
  (it has no CRC command), and refuses data that holds a line `baudrate <digits>` (the bridge would take it for the
  speed command); `--no-erase` is not available with it.

With our loader:

- **Write** works as on the Beken: sectors that already hold the data are skipped (a CRC per 64 k, then per 4 k
  sector), what the image does not cover inside its first and last sector is read first and kept, blank sectors are
  not erased, the others are erased (64 k blocks where a whole one is inside), then written in 4 k pieces; at the end
  the chip's CRC of every part is compared (`--no-verify` skips it). `--no-erase` writes only into blank sectors.
- **Protection.** Before `wf`, `es` and `ea` the status registers are read; block protection bits (BP0..BP4 and, with
  two status bytes, CMP) are cleared as in the Beken mode, with the same list of flash chips; the other bits, the quad
  enable among them, are written back as they were. A chip not in the list with protection bits set is refused;
  `--bk-sr 1|2` gives its number of status bytes. A write or erase the flash ignores is caught by the read-back.
- **Any data can be written**: a frame never holds CR or LF (they are escaped), so the bridge cannot find a speed
  command line in one.
- `rf --twice` compares the chip's CRC of the range with the data read, instead of a second read.

Both loaders:

- **A UF2** (LibreTiny / ESPHome, family LN882H) says itself where its parts go. A full image or backup goes to 0.
  There is no bootloader guard: the download mode is in the ROM, so a wrong write cannot lock the chip out.
- **`ea`** erases everything, also the boot loader and the partition table; the MAC address is in the flash's OTP
  and stays.
- At the end the bridge is closed; `--run` restarts the chip with BOOT released, otherwise it stays in the loader.

### Our loader (`loaders/ln882h`)

`loader.c` (protocol, commands), `flash.c` (SPI NOR commands), `hal.h` (what they need from the chip),
`ln882h.c` (the chip's registers, `main`), `start.S` (boot header, vector table, reset handler), `ln882h.ld`,
`build.py`. It runs from SRAM at 0x20000000 (the boot header first, the vector table at +0x100; image, zeroed data and
stack take 19 kbytes), Cortex-M4 Thumb-2, soft-float, interrupts off, everything polled.

Its start (protocol 2; the first chip showed that protocol 1, which kept the ROM's clock and UART divider, sent only
three garbled bytes and never answered at any speed):

1. All interrupts disabled and none pending, SysTick running as a cycle counter. The registers as the ROM left them
   are read (clock source, PLL, AHB and APB0 dividers, band gap / LDO settings, clock gates, pin functions of A0..A3,
   GPIOA directions, UART0's BRR / CR1 / CR2 / CR3; the info answer reports them, and `--ln info` prints what clock
   and bit rate they mean).
2. `C3LN 2 rom clock` is sent with UART0 as the ROM left it. Only for the log: it comes through when the ROM's divider
   still fits the clock at the hand-over, and garbled when it does not.
3. The clocks as Lightning's loader sets them (its `SetSysClock`, read in the SDK's `mcu/ln882h/system_ln882h.c` and
   `hal_clock.c` and seen in its binary): band gap trim 1 and the PLL's LDO (LDO15_1) at 7, then the PLL as the source
   with the multiplier 8 (core and AHB 160 MHz from the 40 MHz crystal), the AHB undivided, APB0 / 4 = 40 MHz. If the
   PLL's enable bit reads 0, the crystal is kept with APB0 undivided (core 40 MHz, APB0 still 40 MHz).
4. UART0 from scratch: its clock (and GPIOA's) on, a reset pulse, 8N1 with 8x oversampling, no interrupts, DMA or flow
   control, BRR = 16 x APB0 / baud (the SDK's formula; 5556 for 115200), enabled; then A2 / A3 given to it (functions
   2 / 3, GPIO side input, function enable). `baudrate N` computes the divider from the same 40 MHz.
5. The banner and the number of each init stage as the stage begins, each drained out of the UART before the stage
   runs, then `ready` or the error: `C3LN 2 init 1 2 3 4 5 ready` (1 clocks and UART, 2 flash cache off, 3 QSPI
   set-up, 4 JEDEC id, 5 status / unique id / MAC).
6. Until it has received a valid line (`version`, `baudrate`) or frame it repeats every 200 ms
   `C3LN 2 wait rx <bytes received> err <receive errors> ready` (or `error <code> at <stage>`): proof that it runs,
   that it sends at the speed the host listens at, and whether the host's bytes reach it.

For the flash it switches the cache off (clock gates of the cache and the QSPI controller switched on first), sets up
the QSPI controller for plain single-line SPI at 10 MHz (AHB / 16; / 4 on the crystal), wakes the flash from deep
power-down (`AB`), and uses the standard commands: `9F` JEDEC id, `4B` unique id, `05` / `35` / `15` status, `03` read,
`06` write enable (the latch is checked), `02` page program, `20` 4 k sector and `D8` 64 k block erase, `01` status
write, all waiting for WIP with a time-out, and `48` (security register 0x100) for the MAC, as Lightning's SDK keeps it
there.

A fault prints `C3LN fault <exception> pc <address> lr <address>`. All of it shows in the [UART log](#uart-log) of the
session.

The protocol (the full description is at the top of `loader.c`): text lines between frames, `version` (answered
`C3LN 2 ready`, or `C3LN 2 error <code> at <stage>`, so the host tells it from the ROM and from Lightning's
`RAMCODE`) and `baudrate <digits>` (no answer, switched after the line end); everything else is a frame
`C0 escaped(type, seq, status, length, data, CRC-32) C1`, little-endian, with C0 C1 DB CR LF inside sent as DB,
byte ^ 0x20. Commands: `01` info (stage, error, JEDEC id, size, status registers, unique id, MAC, the clocks set, a
core clock measured over the banner, bytes and errors received, the registers as the ROM left them), `02` read (up
to 4 k), `03` CRC-32 of a range, `04` erase (whole 4 k sectors), `05` write (up to 4 k, no erase), `06` status, `07`
status write, `08` reboot. An answer carries the type
with bit 7 set and a status; an error has a 32-bit detail (an address or a status register). Resynchronisation: C0
always starts a new frame, a frame the loader cannot read is answered at once with type FF and the reason, and a
flash change repeated with the same seq and CRC right after (its answer was lost) is answered again, not done again.

Rebuild after a change, with PlatformIO's Arm toolchain (`~/.platformio/packages/toolchain-gccarmnoneeabi`, or `--cc`):

    python3 loaders/ln882h/build.py            # build, fill the boot header, embed into tlsr_c3.py
    python3 loaders/ln882h/build.py --check    # is the embedded one this build?

### First run on a real chip

Our loader has not worked on a chip yet (protocol 1 failed on the first one; protocol 2 sets the clocks and UART0
itself). Lightning's loader is the reference for the first run:

1. `--ln --ln-loader vendor info`, then `--ln --ln-loader vendor rf 0 0 vendor.bin`: a backup, and proof that the
   wiring and the ROM's load path work.
2. `--ln info` with ours. What the output and the UART log (`python3 tlsr_c3.py log`) tell:
   - `C3LN 2 rom clock` readable right after the last YMODEM ACK: the ROM's UART divider still fit at the hand-over;
     garbled bytes there: it did not (that alone is no failure, the loader sets its own clocks next).
   - `C3LN 2 init 1 ...` readable: the clocks and UART0 set by the loader work. The numbers show how far the start got;
     a start that stops at a number hangs in that stage (2 cache, 3 QSPI, 4 / 5 flash commands).
   - `C3LN 2 wait rx 0 err 0 ...` repeated: it runs and is heard, but the host's bytes do not reach it (RX side: pin
     function of A3, UART0 receive); `rx N err M` with errors: its receiver is at another speed than its transmitter.
     The host stops there with that diagnosis (no other speeds are tried: they would not help).
   - Nothing readable at 115200: the host listens at other speeds (921600, 460800, 230400, 1843200, 57600, 28800,
     14400, 1000000, 2000000). A loader that runs sends its wait line every 200 ms there, so a speed where `C3LN`
     appears is the one its UART came up at (the clock is not what the SDK says: `--ln info` at that speed shows the
     registers); if it also answers, the session goes on at that speed.
   - Nothing at any speed, not even the hello: the loader stopped before or while setting the clocks (or the ROM did not
     start it).
   - When it answers: the `Clock:` line (what it set, the core clock measured against the UART), `ROM:` (the clock and
     bit rate the ROM's registers mean) and `Regs:` (all of them); the flash id, status and MAC should match what the
     vendor loader said.
3. `--ln rf 0 0 own.bin --twice` and compare: `cmp vendor.bin own.bin`.
4. Only then a write, best a small one at an unused place first (`--ln wf 0x1f0000 test.bin`, read it back), then the
   image.

When something fails, send the output of the command and of `python3 tlsr_c3.py log` (the UART log of the session:
the hello, the start line, wait lines, a fault line, every frame). `--ln-loader vendor` stays available for flashing
meanwhile.

Wiring (see [Wiring](#wiring)): TX0 (A2) to the SWS wire, RX0 (A3) to GPIO 21, BOOT (A9) to the BOOT wire (the C3's
GPIO 0), CEN to the reset wire, 3V3 to the power pins, GND. The pad names of a Tuya module are in its data sheet. The
power pins carry the chip in download mode, but not its Wi-Fi: on the first LN-02 (2026-11-27) ESPHome restarted at
every Wi-Fi start while powered from them, and ran from the C3 board's 3V3 pin. Flash from the power pins, run from
a real 3.3 V supply. Mains-powered boards are not isolated, so disconnect the mains first.

## Realtek AmebaZ (RTL8710B) over UART

    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl info                        # chip, flash id and size, MAC, OTA slot
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl rf 0 0 backup.bin           # whole flash, checked by the chip's MD5
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl wf 0 firmware.uf2 --run     # LibreTiny / ESPHome UF2
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl wf 0 backup.bin             # a full image from 0
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl es 0x1f0000 0x1000
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl ea

Firmware 0.27. For the RTL8710BN / BX of the Tuya WR modules (WR2, WR3, WR3E …). The chip starts its ROM's UART
download mode when its log TX (PA30) is low as it comes out of reset: the bridge's `rtl` mode pulls the SWS wire low
through GPIO 4's resistor while it restarts the chip (reset wire and power pins) and releases it 50 ms later. The ROM
talks on the log UART at 1500000 baud, and the rest runs in `tlsr_c3.py` through the bridge:

- While the ROM waits for a command it sends NAK now and then; an ACK quiets it. Reads are 1 k pieces, each answered
  with ACK, whole 4 k sectors; writes go over XMODEM with 1 k packets that carry their flash address and a sum
  checksum, and the ROM erases what it writes. The host always sends whole sectors (what is before the start and
  after the end in them is read and sent along). `es` / `ea` erase sector by sector.
- Chip type, flash id, the MAC address (eFuse) and an MD5 of a flash range come from small programs that the host
  loads into the chip's RAM (Thumb code pieces calling the ROM's functions, from ltchiptool). They print at 115200,
  so the host sends the ROM's speed command and then switches the bridge with its speed escape (a pause of a second
  and `~~~S<baud>`), and back to 1500000 when the ROM is in download mode again. Every connection runs one such
  program (for the flash size); every read and write is checked against the chip's MD5 of the range (2–3 s each,
  most of it the pauses before the escapes). `--no-verify` skips it for a write.
- **A UF2** (LibreTiny / ESPHome, family RTL8710B) is written into the OTA slot that the bootloader starts, chosen
  from the switch word in the system data at 0x9000, as ltchiptool does; if that is OTA2 and the chip's OTA2 address
  differs from the UF2's, it is refused (write a full image then). A full image or backup goes to 0.
- `--run` restarts the chip with its TX released at the end; otherwise it stays in download mode.

Wiring (see [Wiring](#wiring)): log TX (PA30) to the SWS wire, log RX (PA29) to GPIO 21, CEN to the reset wire,
3V3 to the power pins, GND. On a WR module these are the TX / RX pads of the log UART (the module's data sheet names
them; on the WR3 they are TXD2 / RXD2). Disconnect the mains first on a mains-powered device.

## Realtek AmebaZ2 (RTL8720C) over UART

    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 info                        # chip, flash, MACs (eFuse)
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 rf 0 0 backup.bin           # whole flash, checked by the chip's SHA-256
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 wf 0 firmware.uf2 --run     # LibreTiny / ESPHome UF2
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 wf 0 backup.bin             # a full image from 0
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 es 0x1f0000 0x1000

Firmware 0.28. For the RTL8720CF (Tuya WBR2, WBR3, CR3L …) and the RTL8710CX / CM. The ROM starts its download
console when PA00 is **high** at reset: the BOOT wire (the C3's GPIO 0) holds it high while the flasher restarts the
chip (reset wire and power pins). PA13 (RX0) must not be pulled low. The console runs on the log UART at 115200;
`ucfg <baud>` raises its speed (`--rtl-baud`, default 1500000; when that fails, a new session at 921600, then 115200),
and the bridge follows that line (`src/lnscan.h`). The rest runs in `tlsr_c3.py` through the bridge:

- Reads are text dumps (`DB` of the flash, mapped at 0x98000000 once `hashq` has set it up), 4 k per command, about
  five characters on the wire per byte: a 2 MB flash takes a bit over a minute at 1500000. The whole range is then
  checked against the SHA-256 the ROM computes (`hashq`, from where the last `fwd` started; an empty, cancelled `fwd`
  sets that).
- Writes go over XMODEM-1K (`fwd`), which erases what it writes; the host sends whole sectors (what is before the
  start and after the end in them is read and sent along) and checks the SHA-256 afterwards. The ROM has no erase
  command: `es` and `ea` write 0xFF.
- Chip type and the Wi-Fi / BT MAC addresses are in the eFuse, read by a small RAM program (ltchiptool's) that the
  host writes with `EW` and starts through an unused command of the ROM console, whose handler it points at the
  program. Only `info` does this.
- **A UF2** (LibreTiny / ESPHome, family RTL8720C) is written into OTA1, as ltchiptool does. A full image or backup
  goes to 0.
- `--run` restarts the chip with PA00 released at the end.

Wiring (see [Wiring](#wiring)): log TX (PA16) to the SWS wire, log RX (PA15) to GPIO 21, PA00 to the BOOT wire (GPIO
0), CEN to the reset wire, 3V3 to the power pins, GND. Disconnect the mains first on a mains-powered device.

## UART bridge

    python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --boot hi     # restarts the target with BOOT high
    ... any serial tool on the same port, at the same baud ...
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit

For a Beken BK72xx the [`--bk` mode](#beken-bk72xx-cb2s-cb3s-wb2s--over-uart) (firmware 0.24) is the preferred
way: the flasher speaks the download protocol itself, faster and with the bootloader guard. The bridge with
`--catch bk` stays for using ltchiptool or another PC tool.

`--catch bk` (firmware 0.23) is for a Beken BK72xx (CB2S, CB3S, WB2S ...), whose ROM answers a flash tool's
link check only right after a reset: while the host sends and the target has not answered yet (`04 0E ...`),
the flasher resets the target (power pins and RST) every 1.5 s. Wire the module's TX1 to the SWS wire, RX1 to
GPIO 21, CEN to GPIO 6 and 3V3 to the power pins, keep the baud at 115200 in the tool, for example:

    python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --catch bk
    ltchiptool flash read -d /dev/ttyACM0 -b 115200 bk72xx backup.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit

`--follow esp` (firmware 0.25) is for running esptool yourself through the bridge: start the bridge at 115200 with the BOOT pin low and give the tool any speed; the bridge
switches when esptool's CHANGE_BAUDRATE has been answered by the target. Each new tool run must start from a fresh
`bridge` command, as the stub left running stays at the raised speed:

    python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --boot lo --follow esp
    esptool --port /dev/ttyACM0 --baud 921600 --before no-reset --after no-reset flash-id
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit

`--follow ln` (firmware 0.26) is the same for the LN882H RAM loaders: the bridge switches at the end of a host line
`baudrate N` (it starts after CR / LF and holds printable characters only, see `src/lnscan.h`).

`--follow ucfg` (firmware 0.28) is the same for the Realtek AmebaZ2 ROM console's `ucfg <baud> …` line.

`--strap rtl` (firmware 0.27) restarts the target with its TX wire pulled low (the Realtek AmebaZ download strap).
For any protocol, the bridge's speed can also be set from the host: after a pause of a second, `~~~S<baud>` and a
line feed in one write (nothing of it reaches the target), the same way as the exit escape.

Otherwise the baud rate is fixed by the `bridge` command: the C3's USB port does not pass on the host's setting.
Tools must not toggle DTR / RTS, because those reset the C3 itself (`esptool --before no_reset --after
no_reset`). GPIO 21 carries the C3's own boot messages for a moment when the flasher is plugged in.

## UART log

Everything that goes over the target's UART in the bridge, `--esp` and `--bk` modes passes one place in the firmware
(`src/tuart.cpp`), which writes it to a log in the C3's RAM: every byte in both directions with its time (1 µs),
and the events: session start with its speed, target resets, speed changes (Beken, esptool's CHANGE_BAUDRATE),
stale bytes dropped, errors. Each UART session starts a new log; the log of the last one stays until the next
session, `log --clear` or the flasher is unplugged, so it can be read after a failed run:

    python3 tlsr_c3.py log                      # uart_log.txt
    python3 tlsr_c3.py log run1.txt --bin run1.bin
    python3 tlsr_c3.py log --clear

The bridge does not answer commands: leave it (`bridge-exit`) before `log`. The `--esp` mode restarts the target for
every esptool run and every retry at a lower speed, so the log holds the last of them.

Data in the same direction that follows within 2 ms of the last piece is added to the same record (a console's text
arrives a few bytes at a time), with the time of its first byte. A block takes 16 k of data; the record headers
(direction, time, length) are kept apart from the data, so text stays contiguous and compresses as text, and the
two parts are compressed separately with LZ4 (the block format, with a one-step lazy match search). The first 24 k
of compressed blocks of a session stay for good (the catch, the link, the speed change), the later ones go into an
88 k ring that drops its oldest blocks, so a long transfer keeps its start and its end; the text marks the dropped
part. The whole log takes about 150 k of the C3's RAM; `log` prints the free heap, and the least it has been since
the flasher started.

How much fits, measured with the firmware's code on real text arriving in 16 to 64 byte pieces with a pause after
each line: a Home Assistant log (repetitive lines) compresses about 10 times, so about 1.1 M of it fits; a Linux
kernel log (varied lines with timestamps) about 2.5 times, about 270 k. Erased flash and protocol frames compress
far better; random data, such as an encrypted or compressed image, not at all, and then the ring holds about 88 k
of it. The time each block takes to compress on the C3 is not measured yet.

The text has the time in seconds, `>` for host to target, `<` for target to host, `#` for events, then hex and
ASCII 16 bytes a line. A record made of one short pattern (the link checks while catching, erased flash) shows the
pattern once with its count, a run of equal lines is one line with its count, and a record repeated back to back is
shown once with its count. A log of the [two-line logger](#two-line-uart-logger) shows `A` and `B` instead;
`log --tuya` decodes the Tuya MCU protocol under the records.

## Two-line UART logger

    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --tuya                    # live; Ctrl-C ends it
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --baud-b 115200 --seconds 60 --out sniff.txt
    python3 tlsr_c3.py --host c3-flasher.local sniff 9600 --tuya            # live over Wi-Fi
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --at-boot                 # capture from every power-up, no host
    python3 tlsr_c3.py --host c3-flasher.local log sniff.txt --tuya         # read that capture afterwards
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff --no-at-boot

From firmware 0.30 the flasher listens to both directions of a UART link between two other parties, for example a
Tuya Wi-Fi module and the board's MCU, without driving anything: live to the computer, and into the
[UART log](#uart-log).

**Wiring.** Tapped onto the link in parallel; the board keeps its own supply.

| Flasher wire | Logger |
|---|---|
| SWS wire (GPIO 5 reads it; GPIO 4, through its resistor, is an input) | line A: one direction, e.g. the MCU's TX |
| GPIO 20 (the JTAG TMS / SPI CS wire) | line B: the other direction, e.g. the module's TX |
| GND | the board's GND |
| GPIO 21, GPIO 0, GPIO 6, the power pins | not connected; all of them inputs while the logger runs |

Line B is GPIO 20, not the bridge's GPIO 21: at every power-up and reset the C3's ROM and bootloader send their boot
messages on GPIO 21 (UART0 TX), which would drive the tapped line against its own driver and reach the chip at its
other end; GPIO 20 is an input from reset on. Both lines must be 3.3 V logic (the C3's pins are not 5 V tolerant);
the C3 holds them with its weak pull-up (about 45 k), so that an open wire reads idle instead of noise. While the
logger runs the reset wire, the BOOT wire, GPIO 21 and the power pins are inputs; when it ends, the flasher is back in
its normal state (the reset wire released, the SWS mode set up again).

**Live.** `sniff BAUD [--baud-b N]` prints every record as it comes: the time in seconds since the start, `A` or `B`,
then hex and ASCII 16 bytes a line, as `log` does; events (`# ...`) are the session start, framing errors and breaks
(wrong speed, or a line held low) and receive overflows, at most once a second per line. Ctrl-C or `--seconds` ends
it: once the host has sent nothing for a second and a half it sends the bridge's escape, the flasher sends what is
still waiting and answers `OK sniff closed`, and takes commands again. `--out FILE` writes the same lines to a file.
The capture never waits for the computer: when the computer takes the output slower than the lines bring it (fast
lines over a weak Wi-Fi link), the live output drops bytes and says how many (`# live output: N bytes dropped`, where
the gap is); the UART log still gets everything. If the host tool was killed, `bridge-exit` ends the logger too.

Over Wi-Fi the live output goes to the TCP connection that started it (the host tool keeps it open for the whole
run). When that connection is gone (closed, broken, or replaced by a new one) the live output ends at once, unlike
the bridge, which waits 60 s for the next tool: a client that connects next expects answers, not frames. The capture
into the log goes on until a command line comes from any host, USB or TCP (for example `log`, which then reads it). A
command from USB also ends a capture whose live output goes over Wi-Fi.

**Capture at power-up.** `sniff BAUD --at-boot` stores the speeds in the C3's flash (NVS). From then on, at every
power-up, the flasher leaves out its usual activation of a target (no reset pulse, no power switching) and listens
at once, with Wi-Fi started first if it is set up; the first command line from a host, over USB or TCP, stops it
(the script sends one as it connects), and `log` / `log --tuya` reads it. **The log is in RAM: it is lost when the
C3 loses power.** A new `sniff` (and every UART mode) starts a new log, so read the old one first. `sniff
--no-at-boot` switches the capture at power-up off.

**Mains-powered boards.** A Tuya module and its MCU on a board whose supply is not isolated are at mains potential,
and so is everything wired to them: the C3, its power bank, and a USB cable to the computer with the computer on it.
Live capture is for a board powered from a safe low voltage (a bench supply, batteries), or over Wi-Fi with the C3 on
a power bank and nobody touching it, nothing on its USB port. Otherwise use the capture at power-up: wire the C3 on
its power bank with the board disconnected from the mains, close everything up, connect the mains and let it run;
afterwards read the log over Wi-Fi without touching anything, or disconnect the board from the mains, then take the
C3's wires off while the power bank keeps it running, and only then plug it into the computer.

**Time resolution.** A record's time (1 µs clock) is when the flasher took its bytes from the UART driver. The driver
hands bytes over when its FIFO holds a threshold of them (every byte up to 57600 baud, 8 up to 230400, 32 above) or
after two character times of silence, and the loop takes them within about a millisecond (longer while a log block
is being compressed; the driver's 4 k buffer per line covers that). So the order of a request on one line and the
answer on the other is right when the answer starts later than about 1 ms plus two character times after the
request's last byte (about 3 ms at 9600 baud); what both lines bring within one pass of the loop is put A first.
Data of one line within 2 ms of its last piece joins that record, as in the UART log.

**Tuya decoding.** `--tuya` (`sniff` and `log`) decodes the Tuya MCU serial protocol under each record: frames `55 AA
<version> <command> <length> <data> <checksum>`, put together per line across records, with the command's name
(heartbeat, product info, working mode, network status, the resets, DP command, DP report, DP query, local time,
others as hex) and the data points (dpid, type raw / bool / value / string / enum / bitmap, length, value). Bytes
outside frames are shown and skipped; a frame with a bad checksum is shown and the search goes on after its `55 AA`:

```
    0.412337 B 55 aa 00 00 00 00 ff                             U......
               tuya v0 heartbeat (0x00)
    0.421904 A 55 aa 03 00 00 01 01 04                          U.......
               tuya v3 heartbeat (0x00), data 01
    1.412337 B 55 aa 00 08 00 00 07                             U......
               tuya v0 DP query (0x08)
    1.431871 A 55 aa 03 07 00 0d 01 01 00 01 01 02 02 00 04 00  U...............
               00 00 fa 1c                                      ....
               tuya v3 DP report (0x07): dp 1 bool len 1 = true; dp 2 value len 4 = 250
```

Which line is which depends on the wiring; the decoder does not need to know.

What runs where: the live output's ring and frames (`src/sniffring.h`) are built natively and checked with a host
that keeps up, a slow one and one that takes almost nothing (`tests/test_sniff.py`); the host's frame parser, the
display, the escape and `bridge-exit` against a fake flasher over a pseudo-terminal and over TCP; the Tuya decoder
on frames split across records, two in one record, bad checksums, garbage and every data point type. The capture
itself (`src/sniff.cpp`: two UART drivers with RX only, UART0 on GPIO 20) is built, not tried on a C3: open until
then are the actual time resolution, the loop's speed at high baud rates on two lines, the log store with Wi-Fi on,
and the live output's speed over USB and Wi-Fi.

## Wi-Fi link

    python3 tlsr_c3.py -p /dev/ttyACM0 wifi join "Home Net"        # station: joins a network (password asked)
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi ap "C3 flasher"        # or an access point of its own (192.168.4.1)
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi key                    # the key a TCP client must give (asked twice)
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi                        # status: mode, IP, name, port, key set or not
    export C3_FLASHER_KEY=...                                       # or --key
    python3 tlsr_c3.py --host c3-flasher.local --bk rf 0 0 backup.bin --twice
    python3 tlsr_c3.py --host 192.168.4.1 log
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi off

From firmware 0.29 the flasher can run Wi-Fi as a station (it joins a network) or as an access point of its own,
and takes the same commands on TCP port 7171 as over USB, so the computer need not be wired to the board. The
setting is kept in the C3's flash (NVS) and Wi-Fi starts with it at power-up. Do the set-up over USB: `wifi join` /
`wifi ap` restart Wi-Fi, which ends a TCP connection, and a mistake in the password is put right over USB again.
The SSID and the password go to the flasher in hex (spaces are fine), the password is asked for (or taken from
`C3_FLASHER_WIFI_PASSWORD` in scripts) and never shown, not by the flasher's status answer either (`password set`).
`wifi name NAME` sets the mDNS name (default `c3-flasher`, so `c3-flasher.local`; service `_c3flasher._tcp`), used
from the next start of Wi-Fi; on an access point the flasher is at 192.168.4.1. `wifi off` switches it off for good.

`--host NAME[:PORT]` (default port 7171) replaces `-p` in every command: Telink, SWD, SPI, WCH, Beken, `--esp`
(esptool gets `socket://NAME:7171` as its port), `--ln`, `--rtl`, `--rtl2`, `bridge`, `bridge-exit`, `log`, `sniff`. One TCP
client is served at a time; a new connection replaces the old one once it has been admitted. USB keeps working
alongside Wi-Fi (also for the set-up and as the way back in); a command line from USB ends a bridge that a TCP
client opened.

**The key.** Without a key anyone who can reach the flasher on the network can read and write the target through
it. `wifi key` sets one (1..64 printable characters, no spaces; an empty entry removes it); the host then gives it as
the first line of every connection (`--key`, or better the environment variable `C3_FLASHER_KEY`), and the flasher
takes that line out of the stream before anything else sees it: it never reaches the command parser, a bridged
target or the UART log, and it is never printed. A connection without the right key is closed. While a bridge
opened over TCP is on, connections from the same computer's address need no key, because esptool (or another
target tool given `socket://NAME:7171`) cannot send it; the bridge ends by itself when no TCP client has been
connected for 60 s. The key travels in clear text, as everything over this link does: it keeps out others on a
trusted network, it is no protection on an untrusted one. When the flasher has no key set, do not give one either:
the host sends it all the same, and the flasher takes it out of the stream only when it comes within the first
second of a connection (it always does, unless a packet is lost).

**Speed.** Not measured yet. The C3's USB port moves about 45 kbytes/s of base64 or more in the binary streams; a
TCP connection over Wi-Fi may be faster or slower depending on the signal, and the UART bridge over TCP adds the
network's delay to every answer of the target, which matters to protocols with many small exchanges.

**Memory.** Wi-Fi and its TCP/IP stack take their buffers from the C3's heap while they run, so starting Wi-Fi
gives the UART log's store back and the log then leaves 40 k of the largest free block to them (24 k without
Wi-Fi), on top of every smaller free block (firmware 0.30; 64 k in 0.29). Wi-Fi's fixed buffers are already taken by
then; the 40 k are for what it takes while it runs: frames in flight to and from the one TCP client (lwIP's window
and send buffer are about 5.7 k each, a Wi-Fi buffer 1.6 k) and bursts of received frames. The log of a session holds
less with Wi-Fi on: its store shrinks with the heap down to 21 k, the size of one block of the worst case, with the
kept start made smaller or left out so that the ring still holds such a block; below that there is no log (`log`
shows the store's size as `log store` and the lowest free heap since the start, the `wifi` status the free heap and
its largest block). How much the store gets with Wi-Fi on, and whether 40 k are enough under load, is not measured
yet: a lowest free heap under about 15 k in `log` after a session with Wi-Fi on would mean the 40 k are too few.
`wifi off` restores the normal share from
the next session.

**Safety.** Wi-Fi isolates the computer, not the C3. A C3 (and the power bank it hangs on) wired to a
mains-powered board whose supply is not isolated is at mains potential: do not touch it or its power bank, and plug
nothing into its USB port while the board is on the mains. Better still, power such a board from the flasher only,
with the mains wires disconnected, as in the [Wiring](#wiring) section.

What runs where: the host side is tested against a model of the flasher's command side (`tests/test_wifi_host.py`:
key accepted and refused, the `wifi` commands' hex encoding with the password absent from all output, a read stream
over TCP and a connection lost in the middle of one, `bridge-exit` and `--esp` over TCP). The firmware's Wi-Fi part
(`src/wifilink.cpp`) cannot be simulated without the ESP-IDF network stack; it is built, not tried on a C3: open until
then are the connection to real access points, mDNS, the heap left with Wi-Fi on, the speed, and how esptool,
ltchiptool and the LN / Realtek modes behave with the network's delays.

## Host protocol

One text line per command, answer `OK [payload]` or `ERR reason`; numbers decimal or `0x` hex, data hex.

`V`, `MODE uart|rmt`, `BAUD n`, `RATE n`, `BURST n`, `SELFTEST`, `RST` (reset pulse and power cycle),
`A ms` (activate), `G addr len` (read registers / RAM), `S addr hex` (write registers / RAM),
`FJ`, `FS`, `FR addr len` (hex), `FB addr len` (base64, `*` = all 0xff), `FC addr len` (CRC-32 of the range),
`FU`, `FE addr`, `FA`, `FW addr hex`, `RUN`.

SWD / cJTAG: `SWD`, `SWCLK khz`, `SC [reset|md sel]`, `TC` (cJTAG + ICEPick, `OK <icepick id> <dap id>`),
`SRST 0|1`, `SDR`/`SDW`/`SAR`/`SAW` (DP / AP registers), `SM`/`SMW` (memory), `SFAM name [p1 p2]`,
`SHALT [reset]`, `SRUN`, `SCALL pc sp lr [r0..r3]`, `SFE addr`, `SFX`, `SFW addr hex`, `SFC addr len`.

JTAG: `JTAG [khz]`, `JS` (chain scan: `OK <IR bits> <idcode> ...`, TDO side first, 0 = no IDCODE),
`JSEL pre_ir post_ir pre_dr post_dr` (DAP position), `JC` (JTAG-DP connect, `OK <idcode>`), `JRST`,
`JIR bits hex` / `JDR bits hex` (raw scans, LSB first, answer the captured bits).

SPI flash: `SPI [khz]` (reset held low), `SPI 0` (leave, reset released), `PT hexout [nin]` (raw transaction), `PJ` (JEDEC id), `PA 3|4` (address width),
`PB addr len` (base64), `PC addr len` (CRC-32), `PE addr [0x20|0x52|0xD8]`, `PX` (chip erase),
`PW addr hex` (within one 256-byte page), `PS` (status 1).

WCH CH32V003 over SWIO (firmware 0.22): `WCH` (the SWS wire to SWIO, `OK <timing>`), `WCH 0` (leave),
`WT [one zero high slot threshold stop]` (line timing in ns), `WA off_ms window_ms` (power off for off_ms if it
is not 0, then catch: `OK <dmstatus> <cpbr> <tries> <µs to the first answer> <1 = power cycled>`, or `ERR` with
the reason), `WR addr` / `WW addr value` (debug module registers), `WH [reset]` (halt), `WG` (resume),
`WRUN` (reset and run), `WM addr len` (memory, hex, whole words), `WMW addr hex` (RAM / registers, whole
words, up to 512 bytes per line), `WFU`
(unlock, `OK <CTLR>`), `WFE addr` (erase the 64-byte page), `WFX` (mass erase), `WFW addr hex` (program inside
one erased page), `WFC addr len` (CRC-32), `WOB hex` (erase and write the 16 option bytes).

Beken BK72xx (firmware 0.24): `BK` (UART1 at 115200 on GPIO 21 / the SWS wire, `BK 0` leaves), `BKL catch_ms baud`
(catch, link, protocol, speed: `OK <rom|bl> <baud> <resets> <ms to the answer> <1 = fell back to 115200>
<bytes of the chip's own firmware before> [why it fell back]`, or `ERR` with what was seen), `BKI` (`OK <chip id>
<flash JEDEC id>`, `0 000000` on the BK7231T), `BKRR addr` (register), `BKSR opcode` / `BKSW opcode hex` (flash
status register), `BKC addr len` (zlib CRC-32 computed by the chip), `BKE addr [64]` (erase the 4 k sector / 64 k
block), `BKR addr len` (read up to 512 bytes, hex), `BKB offset hex` + `BKW addr` (fill a 4 k buffer, write it to an
erased sector), `BKG [first writable address [flash size]]` (the bootloader guard, 0x11000 after `BK`), `BKX` (reboot).

UART bridge (firmware 0.15): `BRIDGE <baud> [hi|lo|z] [bk|esp|ln|ucfg|rtl]` (BOOT level while the target is restarted; `bk`
resets the target until a Beken ROM answers, `esp` follows esptool's CHANGE_BAUDRATE, firmware 0.25, `ln` the
LN882H loader's `baudrate N` line, firmware 0.26, `ucfg` the AmebaZ2 ROM's `ucfg N` line, firmware 0.28, `rtl` holds the target's TX low across the restart, firmware 0.27;
speeds up to 5000000); set its speed with a pause of a second and `~~~S<baud>\n` (firmware 0.27); leave it with
a pause of a second and `~~~X`; the bridge belongs to the source that opened it (USB or TCP). UART log (firmware 0.25): `LOG` (stores the block being filled, answers
`OK log stored=<bytes> blocks= raw=<bytes of records> dropped=<blocks> tx= rx= heap=<free> heap_min=`, then read it with `XR L 0 <stored>`),
`LOG 0` (clear).

Two-line UART logger (firmware 0.30): `SNIFF <baud A> [<baud B>]` (line A the SWS wire, line B GPIO 20, B's speed
A's when left out; 1200..5000000) answers `OK sniff A <baud> B <baud>; ...`, then sends binary frames `<tag:u8>
<t_us:u32> <n:u16> <data>` (little endian; tag 1 = line B, 2 = line A, 3 = event text; t_us the flasher's microsecond
clock, which wraps) until a pause of a second and `~~~X` from the same host, answered by `OK sniff closed` after the
last frame (a frame starts with a byte 1..3, a text line with a letter). The host's other bytes are ignored. The UART
log gets everything (tag 2 = A, 1 = B, session text `sniff, A <baud>, B <baud>`). A TCP host that is gone ends the
live output, not the capture; a command line from USB ends it, as does any command line once the live output has
ended. `SNIFFBOOT <baud A> [<baud B>]` / `SNIFFBOOT 0` sets / clears the capture at power-up (NVS; without arguments
the setting), answer `OK sniffboot A <a> B <b>` or `OK sniffboot off`; such a capture has no live output and ends
at the first command line.

Wi-Fi link (firmware 0.29): `WIFI` (status: `OK wifi mode=off|ap|sta ssid=<hex> pass=none|set key=none|set name=
port= [ip= rssi= | ip= stations= | ip=none state=connecting] client=<address|none> heap= largest=`, no secrets),
`WIFI AP <ssid hex> <password hex>` (8..63 characters), `WIFI STA <ssid hex> <password hex | ->`, `WIFI OFF`,
`WIFI KEY <key hex | ->`, `WIFI NAME <name>`; AP and STA restart Wi-Fi and drop a TCP client. Over TCP (port 7171)
the protocol is the same; the first line of a connection may be `AUTH <key>` (answer `OK auth`, or `ERR auth` and
the connection is closed); with a key set it must be, except from the address of the client that opened a bridge
while that bridge is on. Answers are gathered into whole lines before they are sent. A client that takes nothing
for 8 s, or whose connection fails, is dropped, and a stream to it stops at once; a client that vanished without
closing is noticed by TCP keep-alive within about half a minute.

Bulk data moves in binary streams (firmware 0.18), not in text answers: `XR <src> <addr> <len>` answers
`OK <len>` and then frames `D <n:u16> <data> <crc32>` of up to 4 k (`E <n:u16> <text>` on an error, `Z` at
the end); `XW <dst> <addr> <len> <unit> <skip>` answers `OK`, then takes frames `<n:u16> <data> <crc32>` and
answers each with `K` once it is written. The host keeps up to three frames on their way, so the next ones
wait in the C3's 16 k receive buffer while one is programmed. After an error the C3 drops everything until
the line has been quiet for 300 ms, so frames sent ahead never reach the command parser. Sources and
destinations: `F` Telink flash, `S` SWD flash (write) / memory (read), `M` SWD memory, `P` SPI flash, `C` CH32V003
memory (read) / flash in 64-byte pages or RAM (write), `B` BK72xx flash (read: every 4 k block checked against the
chip's CRC; write: whole erased 4 k sectors, address and length 4 k aligned), `L` the UART log (read); `unit`
is the boundary no single flash write crosses, `skip 1` leaves out parts that are all 0xff (not on STM32
L0 / L1, whose erased flash reads 0x00). The old text commands stay for single blocks. The second pass of
`rf --twice` and the verify after `wf` only compare CRCs (`FC`, `SFC`, `PC`, `BKC`).

## Not done

- Telink: pvvx's RAM loader (`TlsrComProg825x`) is not used: it talks over the chip's UART pins (PA0 / PB1),
  not over SWS, so it needs two more wires. The TLSR826x family (2-byte addresses) is not handled.
- SWD: STM32 F7 / H7 and the F1 XL-density parts have no flash routine; locked EFR32 (AAP / Secure Engine
  unlock) and STM32 read protection removal are not implemented.
- cJTAG: CC13x4 / CC26x4 and CC2538.
- JTAG: RISC-V debug, EJTAG and boundary scan on top of the raw scans.
- WCH: only the CH32V003. The V002 / V004 / V005 / V006 / V007 (256-byte pages, another boot area), CH32X035,
  CH32V103 / V20x / V30x and CH5xx are not handled (minichlink knows them), nor the two-wire RVSWD of the bigger
  parts. The fast 1x mode, the parity bit, the debug printf over SWIO and GDB are not used.
- WCH, open until a real chip has been on the wire: the timing margins (the length of the target's 0 when
  reading is assumed to be about 1 µs; the defaults and the 550 ns threshold rest on that), how soon after
  power-on the first halt request lands compared with a real firmware's start, whether the chip erases its
  flash at the moment RDPR = 0xA5 is written (as the simulation does) and whether a debug reset reloads the
  option bytes (the flasher power-cycles anyway), what a protected flash reads as over debug, and the
  independent watchdog of a firmware with IWDG_SW = 0 while the core is halted.

- Beken: BK7231N, BK7231T and BK7238 (the BK7238 has the BK7231N's BootROM protocol; only the calibration address and
  the UF2 family are its own, not tried on a chip). The BK7231Q / U and BK7252 protocols and the eFuse / encryption key
  readout are not handled. Open until a real chip has been on the wire: how soon after power-on the BootROM listens
  and whether the back-to-back link checks land in that window, whether 921600 (or more) is reliable over these
  wires, the 50 ms speed change delay, the CRC speed (timeouts assume at least 80 kbytes/s), the 64 k erase time
  (8 s allowed), the BK7231T bootloader's answers to refused reads and its time-out in download mode, and the
  power the module needs from the power pins.

## Files

- `src/main.cpp`, `src/config.h`, `platformio.ini`: the ESP32-C3 firmware, with the Telink SWS part and the
  command parser.
- `src/swd.cpp` / `.h`: SWD, cJTAG and JTAG line handling, DP / AP access, memory and core control.
- `src/swdflash.cpp` / `.h`: the flash routines of the Arm and TI chips.
- `src/spiflash.cpp` / `.h`: SPI NOR flash.
- `src/wch.cpp` / `.h`: the WCH single-wire debug line on the RMT peripheral, debug register access, the catch
  at power-up.
- `src/wchflash.cpp` / `.h`: CH32V003 debug module, memory access through the program buffer, flash controller,
  option bytes, and the W commands.
- `src/bk72xx.cpp` / `.h`: Beken BK72xx download protocol on UART1: catch at power-up, link, speed, flash read /
  erase / write / CRC, status registers, the bootloader guard, and the BK commands.
- `src/tuart.cpp` / `.h`: the target's UART (TX GPIO 21, RX the SWS wire), the one way the bridge and the Beken mode
  send, receive, change speed and drop stale bytes; every byte and event goes to the UART log there.
- `src/lnscan.h`: the bridge's reading of a loader's speed command line (LN882H `baudrate`, AmebaZ2 `ucfg`).
- `src/uartlog.cpp` / `.h`: the UART log: records, 16 k blocks with headers and data compressed apart (LZ4),
  the kept start and the ring.
- `src/sniff.cpp` / `.h`: the two-line UART logger: both lines' UART drivers (RX only), the capture into the UART
  log and the live ring, error events, the capture at power-up in NVS.
- `src/sniffring.h`: the logger's live output: frames in a ring that never makes the capture wait (builds natively).
- `src/wifilink.cpp` / `.h`: the Wi-Fi link: settings in NVS, access point or station, mDNS, the TCP server with
  its one client as a Stream, the key, and the WIFI command.
- `tlsr_c3.py`: the command line tool on the computer (Python 3, `pyserial`), with our LN882H loader built in.
- `loaders/ln882h/`: our LN882H RAM loader (C, start-up code, linker script) and `build.py`, which builds it with
  PlatformIO's Arm toolchain and embeds it into `tlsr_c3.py`.
- `tests/`: checks without hardware (stream protocol, SPI host logic, JTAG against a simulated chain, the WCH
  firmware and host against a simulated CH32V003, the BK72xx firmware and host against a simulated BootROM /
  bootloader, the UART log from the firmware's encoder to the host's text, the LN882H host and the bridge's speed
  following against a model of the ROM and both RAM loaders, ours built natively and its binary emulated, the
  RTL8710B host against a model of its ROM that also
  checks and runs the RAM programs, the RTL8720C host against a model of its ROM console, the host over TCP against
  a model of the flasher's command side, the logger's live ring natively and its host side and Tuya decoder against
  a fake flasher).

## Credits

SWire emulation over a UART, the register sequences and the activation: pvvx
(<https://github.com/pvvx/TlsrComSwireWriter>, public domain; <https://github.com/pvvx/TLSRPGM>).

WCH SWIO:

- WCH (Nanjing Qinheng Microelectronics): the QingKe V2 debug manual and the one-wire host example for a
  CH32F103, both in <https://github.com/openwch/ch32v003> (`CH32V003_1Line_Base_on_CH32F103`): the line
  timing, the output enable, halt and resume, and the read / write program for the program buffer.
- ch32v003fun by Charles Lohr and contributors (MIT; <https://github.com/cnlohr/ch32v003fun>): minichlink and
  the funprog programmer core (`minichlink/funprog/funprog_rvswdio.h`), with its ESP32-S2 programmer
  (<https://github.com/cnlohr/esp32s2-cookbook>, `ch32v003programmer`) and the CH32V003 programmer in
  <https://github.com/cnlohr/rv003usb> (`rvswdio_programmer`): the flash buffer-load program (adapted), the
  power-off catch of its unbrick, the flash and option byte sequences, the chip identification and the
  factory option bytes, and measured pulse lengths for the timing.
- Ardulink by Mark Boldyrev (MIT-0; <https://gitlab.com/BlueSyncLine/arduino-ch32v003-swio>): another
  timing reference, and the series resistor on SWIO.

Beken BK72xx:

- ltchiptool by Kuba Szczodrzyński (MIT; <https://github.com/libretiny-eu/ltchiptool>, `ltchiptool/soc/bk72xx`,
  and its `uf2tool`): the flashing sequence (unprotect, erase, write, CRC), the RF calibration / MAC location, and
  the UF2 format (blocks, tags, `OTA_PART_INFO`, `FAL_PTABLE`) read by `uf2_parts`. LibreTiny
  (<https://github.com/libretiny-eu/libretiny>, MIT): the board flash layouts, the UF2 family ids, the build outputs
  and the note that the BK7231T has no ROM loader.
- bk7231tools by Khaled Nassar, Tom Clement and Kuba Szczodrzyński (MIT;
  <https://github.com/tuya-cloudcutter/bk7231tools>, `bk7231tools/serial`): the packet framing, the command codes
  and answers, linking with resets, the speed change, the CRC conventions of the two protocols, the BK7231T
  bootloader's address bypass and its protection after a CRC. Adapted from it: the bootloader CRC list, the chip id
  values and the table of status register sizes per flash id, with the protection mask.
- BK7231GUIFlashTool by openshwprojects (<https://github.com/openshwprojects/BK7231GUIFlashTool>; its repository has
  no license file, so no code was taken from it): read for the protocol only, which is reimplemented here: the flash
  id answer with the short length field, the speed change timing, the status register opcodes per flash maker
  (Macronix), the erase time-outs and the handling of OpenBeken's `_QIO_` / `_UA_` images.

Realtek AmebaZ:

- ltchiptool by Kuba Szczodrzyński (MIT; `ltchiptool/soc/ambz`): the download protocol (handshakes, read, XMODEM with
  addresses, speed table), the RAM programs (`ambzcode.py`, taken over as `RtlCode`), the OTA slot choice and the
  system data layout, and the chip ids. rtltool by pvvx (in ltchiptool, `ambz/util/rtltool.py`): the sector erase
  and the packet checksum.

Realtek AmebaZ2: ltchiptool (MIT; `ltchiptool/soc/ambz2`, `soc/amb/efuse.py`): the console commands and their
answers, the flash set-up, `hashq`, the cancelled `fwd`, starting a RAM program through an unused console command,
the eFuse read program, the physical-to-logical eFuse conversion, the MAC locations, the strapping and the OTA1 choice.

Lightning LN882H:

- ltchiptool by Etienne Le Cousin and Kuba Szczodrzyński (MIT; <https://github.com/libretiny-eu/ltchiptool>,
  `ltchiptool/soc/ln882h`): the sequence (ROM link, RAM loader over YMODEM, speed change, `flash_read` with its CRC,
  `startaddr` + `upgrade`), the wiring and the RAM loader binary it ships (Lightning's, not included here).
- OpenLN882H SDK (<https://github.com/openshwprojects/OpenLN882H>, Shanghai Lightning Semiconductor's SDK; it has
  no license, so nothing of it is copied here: our loader is written from the facts read in it): the source of
  Lightning's RAM loader (`project/bootcode/ramcode_dl`): its commands, the YMODEM receiver, the erase of the received
  range that makes the host send whole sectors, the boot header layout, and what that loader sets up (the clocks in
  `SetSysClock`, `mcu/ln882h/system_ln882h.c` and `hal_clock.c`; UART0 on A2 / A3 with `hal_uart.c` and `hal_gpio.c`,
  the BRR formula; the flash cache switched off, the QSPI controller); the register addresses and bits of the clock
  tree (`reg_sysc_awo.h`, `reg_sysc_cmp.h`: clock source, PLL, AHB / APB0 dividers and their update bits, band gap and
  LDO settings), UART0, the pin functions, GPIOA, the QSPI controller (a DesignWare SSI), the flash cache, the clock
  gates, the resets, the pad enable and the chip reset (`mcu/driver_ln882h`); the
  transfer sequences of its flash driver and the flash commands it uses, the MAC address in the flash's security
  register at 0x100 (`components/fs/nvds`), and how its build tools fill the boot header's length and CRCs
  (`tools/python_scripts`). LibreTiny: the UF2 family id.
- ltchiptool's copy of the SDK's boot header model (`ltchiptool/soc/ln882h/util/models/boot_header.py`): the same
  header fields; `loaders/ln882h/build.py` fills them in its own code and reproduces the header of Lightning's loader
  binary exactly (checked by `tests/test_ln_loader.py` when that binary is at hand). The start of that binary
  (disassembled, nothing taken from it but the order and values of its clock and UART set-up) confirmed that it runs
  the SDK's `SetSysClock` with the PLL, multiplier 8, APB0 / 4, and sets UART0 to 115200 with 8x oversampling.

## License

MIT, see `LICENSE`.
