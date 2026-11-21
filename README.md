# ESP32-C3 as a Telink TLSR825x flasher

Reads, erases and writes the flash of a Telink TLSR825x chip (TLSR8251 / 8253 / 8258, Tuya "Z2") over its
single-wire debug pin (SWS), with an ESP32-C3 board instead of a USB-UART adapter.

**Status (2026-11-15):** used on three kinds of TLSR8258 boards:

| Board | Flash | Reset pad | Done with it |
|---|---|---|---|
| ZY-ZTH03Pro thermometer (Tuya) | 512 k GigaDevice | yes | backup, write, verify |
| Zbeacon TS0001 relay module | 512 k ZBIT | no (power pins) | backup, write, verify, sector erase |
| TS004F rotary knob (ZG-101ZD) | 1 M GigaDevice | no (power pins) | backup, write, pin and LED search |

At the default 1 Mbit/s (RMT mode) a 512 k read takes 9.3 s, a full backup with `--twice` 19.8 s, a 182 k
firmware write with verify 15.3 s. Read works up to 1.6 Mbit/s (`--rate 1500000`); the default leaves more
margin. UART mode works at 921600 baud; above that the first bit of each byte is read wrong (the activation
check refuses such a speed). Read first, write only after a backup.

The SWS line can be driven in two ways, selectable at run time on the same wiring:

| Mode | How | Speed setting | State |
|---|---|---|---|
| `rmt` (default) | The RMT peripheral generates and measures the pulses (12.5 ns steps) | `--rate` SWire bit rate, default 1 Mbit/s | tested up to 1.6 Mbit/s |
| `uart` | One UART byte per SWire bit, a port of pvvx's `TLSR825xComFlasher.py` | `-b` UART baud, SWire bit rate = baud / 10 | tested at 921600 baud (92 kbit/s); the fallback |

## Wiring

| ESP32-C3 | Target | Note |
|---|---|---|
| GPIO 5 (`PIN_SWS_RX`) | SWS | direct |
| GPIO 4 (`PIN_SWS_TX`) | SWS | through a 1 k … 1.8 k resistor |
| GPIO 6 (`PIN_RST`) | RST | open drain, active low; leave open if the board has no reset pad |
| 3V3 **or** GPIO 7 (+ 10, 3, 1) | 3.3 V / VBAT+ | see below; remove the batteries first |
| GND | GND | |

Pins and the power-up defaults are set in `src/config.h`. Never put 5 V on the target. Keep the SWS wire
short (10 cm or less) for the faster settings.

**Boards with a reset pad:** power from the 3V3 pin, reset from GPIO 6.

**Boards without a reset pad** (and boards whose firmware switches SWS off right after start, for example
Tuya relay modules): connect the target's 3.3 V pad to GPIO 7 instead of the 3V3 pin. Every activation
then switches the target off for 300 ms and catches it while it starts. One pin gives about 40 mA at its
limit; GPIO 7, 10, 3 and 1 (`PWR_PINS`) switch with one register write, so join two or more of them on the
target's 3.3 V pad to share the current. Do not connect these pins to the 3V3 pin. Enough for a bare
Telink board, not for relays or other loads.

The firmware also tries an activation by itself for 3 seconds when the flasher is plugged in
(`AUTO_ACTIVATE_MS`), for a target that gets its power from the 3V3 pin together with the flasher. Whether
this is early enough depends on the target's firmware; the power pins are the reliable way.

**Mains powered modules:** their supply is usually not isolated (logic ground = neutral). Disconnect both
mains wires before connecting the flasher, and power the board from the flasher only.

## Other targets: Arm SWD and a UART bridge (firmware 0.15)

The same board also talks Arm Serial Wire Debug and works as a USB–UART bridge. Only one of them is active at a
time, so they share the wires:

| ESP32-C3 | SWS (Telink) | SWD (EFR32, nRF52, STM32, …) | UART bridge |
|---|---|---|---|
| GPIO 4 –1 k– GPIO 5 (the SWS wire) | SWS | SWDIO | target TX → flasher RX (GPIO 4 is an input) |
| GPIO 0 | – | SWCLK | BOOT (`--boot hi` / `lo`, open by default) |
| GPIO 21 | – | – | flasher TX → target RX |
| GPIO 6 | RST | nRESET | RESET |
| GPIO 7 (+ 10, 3, 1) | target power | target power | target power |

A pin is only an output while its mode needs it; otherwise it is an input, so a loose wire touching
something does not short it. The power pins stay inputs until the target is powered the first time;
`power-off` switches them off and back to inputs.

SWD:

    python3 tlsr_c3.py -p /dev/ttyACM0 --swd info
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd rf 0 0x40000 backup.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd wf 0 firmware.bin --run
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd es 0x3e000 0x2000
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd ea

`info` reads the IDCODE, the access ports, the CPU type and, for Silicon Labs EFR32 (Series 1 and 2) and
Nordic nRF52, the device information (part, flash size, page size). `--swd-khz` sets the clock (default 1000).

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

STM32 addresses may be given from 0 (taken as 0x08000000), RP2040 ones from 0 (taken as 0x10000000). The
RP2040 is reached through its multi-drop port (core 0); `info` cannot read the size of its external flash,
so `rf` needs it given. While its flash is written, core 1 is switched off; `--run` restarts the chip with the
watchdog. STM32 F7, H7 and the F1 XL-density parts have no flash routine yet. GD32 and CH32 parts that report an STM32F1 device ID are written with the F1 routine.

A debug-locked EFR32 shows only its AAP and cannot be read or written: Series 1 opens with the AAP mass
erase, Series 2 through the Secure Engine; neither is implemented yet. A read-protected STM32 (RDP) is
reported by `info`; removing the protection erases the chip and is not implemented either.
Nothing of the SWD part is tested on hardware yet.

### TI CC13xx / CC26xx over two-wire cJTAG (firmware 0.16)

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

### SPI NOR flash, e.g. a router's (firmware 0.17, c3-usb boards only)

The 25-series SPI flash of a router, a dongle or any other board, on the flasher's existing wires:

| Flash pin | Flasher wire |
|---|---|
| /CS (1) | reset wire (GPIO 6) |
| DO (2) | UART bridge TX wire (GPIO 21) |
| DI (5) | SWS wire (GPIO 4 drives it through the 1 k resistor) |
| CLK (6) | SWCLK wire (GPIO 0) |
| GND (4) | GND |
| /WP (3), /HOLD (7) | to 3.3 V if the board does not pull them up |

    python3 tlsr_c3.py --spi info
    python3 tlsr_c3.py --spi rf 0 0 backup.bin --twice        # size 0 = the whole chip
    python3 tlsr_c3.py --spi wf 0x50000 firmware.bin --unprotect

The size and the erase unit come from the chip's SFDP table (the id byte when it has none); chips over
16 Mbytes are used with the 4-byte opcodes. `wf` works erase unit by erase unit: a unit whose CRC already
matches is skipped, and what the image does not cover inside a unit is read first and kept, so a bootloader
or a calibration partition next to the written range stays. `rf --twice` checks the read with a second pass
computed on the flasher (CRC), and `wf` verifies the same way. Block protection (BP bits) is reported by
`info`; `--unprotect` clears it and keeps the quad-enable bit. Status registers 2 / 3 are only read on
Winbond and GigaDevice chips: on Macronix and ISSI the same command (0x35) switches the chip to QPI.

In circuit, the board's own processor drives the same pins: keep it in reset (or its flash pins high
impedance) while the flasher works, and power the board from its own supply. The flasher does not switch
any power in SPI mode. Speed: about 40 kbytes/s reading (the USB link), clock 4 MHz by default
(`--spi-khz`, the DI wire goes through 1 k). Not tested on hardware yet.

UART bridge (c3-usb boards only; on c3-uart boards UART0 is the link to the computer):

    python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --boot hi     # restarts the target with BOOT high
    ... any serial tool on the same port, at the same baud ...
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit

The baud rate is fixed by the `bridge` command: the C3's USB port does not pass on the host's setting.
Tools must not toggle DTR / RTS, because those reset the C3 itself (`esptool --before no_reset --after
no_reset`). GPIO 21 carries the C3's own boot messages for a moment when the flasher is plugged in.

## Build and upload

    pio run -e c3-usb -t upload      # USB connector wired straight to the C3 (Super Mini, XIAO)
    pio run -e c3-uart -t upload     # board with a CH340 / CP2102 USB-UART chip

Needs the Arduino core 3.x (ESP-IDF 5) for the RMT driver.

## Use

    pip install pyserial
    python3 tlsr_c3.py -p /dev/ttyACM0 selftest
    python3 tlsr_c3.py -p /dev/ttyACM0 info
    python3 tlsr_c3.py -p /dev/ttyACM0 rf 0 0x80000 stock.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 wf 0 new_firmware.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 es 0x2D000 0x49000
    python3 tlsr_c3.py -p /dev/ttyACM0 ea
    python3 tlsr_c3.py -p /dev/ttyACM0 --run info

    python3 tlsr_c3.py -p /dev/ttyACM0 --mode uart info

On Windows use `py tlsr_c3.py -p COM12 ...`.

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

## Finding the pins of an unknown board

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

## First use

1. `selftest` with only the resistor fitted, no target: checks the TX – resistor – RX path.
2. `info` — proves wiring, activation and the speed. Every activation also writes a test pattern into the
   chip's RAM and reads it back at the speed and burst in use; if that fails, nothing else is done.
   If the chip does not answer: check the resistor and GND, raise `-t`, lower `--rate 500000`, or try
   `--mode uart`.
3. `rf 0 <flash size> stock.bin --twice` — full backup; the second pass compares checksums.
4. Keep the backup safe, then `wf`. It erases the touched sectors, writes and verifies.

## The protocol, and how the two modes map onto it

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

## Flash access, checked against the chip's own flash code

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

## Host protocol

One text line per command, answer `OK [payload]` or `ERR reason`; numbers decimal or `0x` hex, data hex.

`V`, `MODE uart|rmt`, `BAUD n`, `RATE n`, `BURST n`, `SELFTEST`, `RST` (reset pulse and power cycle),
`A ms` (activate), `G addr len` (read registers / RAM), `S addr hex` (write registers / RAM),
`FJ`, `FS`, `FR addr len` (hex), `FB addr len` (base64, `*` = all 0xff), `FC addr len` (CRC-32 of the range),
`FU`, `FE addr`, `FA`, `FW addr hex`, `RUN`.

SWD / cJTAG: `SWD`, `SWCLK khz`, `SC [reset|md sel]`, `TC` (cJTAG + ICEPick, `OK <icepick id> <dap id>`),
`SRST 0|1`, `SDR`/`SDW`/`SAR`/`SAW` (DP / AP registers), `SM`/`SMW` (memory), `SFAM name [p1 p2]`,
`SHALT [reset]`, `SRUN`, `SCALL pc sp lr [r0..r3]`, `SFE addr`, `SFX`, `SFW addr hex`, `SFC addr len`.

SPI flash: `SPI [khz]`, `PT hexout [nin]` (raw transaction), `PJ` (JEDEC id), `PA 3|4` (address width),
`PB addr len` (base64), `PC addr len` (CRC-32), `PE addr [0x20|0x52|0xD8]`, `PX` (chip erase),
`PW addr hex` (within one 256-byte page), `PS` (status 1).

The USB link is the slow part of a read (about 45 kbytes/s), so the host reads with `FB`, and the second
pass of `rf --twice` and the verify after `wf` only compare CRCs (`FC`).

## Not done

- pvvx's RAM loader (`TlsrComProg825x`) is not used: it talks over the chip's UART pins (PA0 / PB1), not
  over SWS, so it needs two more wires.
- Chips other than TLSR825x (the TLSR826x family uses 2-byte addresses).

## Files

- `src/main.cpp`, `src/config.h`, `platformio.ini`: the ESP32-C3 firmware.
- `tlsr_c3.py`: the command line tool on the computer (Python 3, `pyserial`).

## Credits

SWire emulation over a UART, the register sequences and the activation: pvvx
(<https://github.com/pvvx/TlsrComSwireWriter>, public domain; <https://github.com/pvvx/TLSRPGM>).

## License

MIT, see `LICENSE`.
