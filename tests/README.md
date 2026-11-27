# Tests without hardware

    python3 tests/test_streams.py     # host side of the XR / XW binary streams against a model of the firmware
    python3 tests/test_spi_host.py    # SPI flash host logic against a simulated Winbond / Macronix chip
    python3 tests/test_esp_host.py    # ESP mode: bridge, esptool arguments (4 and 5), restart, with fakes
    g++ -std=c++17 -I tests/jtag_sim/stub -I src -o /tmp/jtag_sim tests/jtag_sim/jtag_sim.cpp src/swd.cpp && /tmp/jtag_sim
    g++ -std=c++17 -Wall -Wextra -I tests/wch_sim/stub -I src -o /tmp/wch_sim tests/wch_sim/wch_sim.cpp src/wch.cpp src/wchflash.cpp && /tmp/wch_sim
    python3 tests/test_wch_host.py    # WCH host commands against the same simulation (builds it with g++)
    g++ -std=c++17 -Wall -Wextra -I tests/py32_sim/stub -I src -o /tmp/py32_sim tests/py32_sim/py32_sim.cpp src/swdflash.cpp && /tmp/py32_sim
    python3 tests/test_py32_host.py   # Puya PY32F002B / MS32C001 SWD host commands against py32_sim (builds it)
    g++ -std=c++17 -Wall -Wextra -I tests/bk_sim/stub -I src -o /tmp/bk_sim tests/bk_sim/bk_sim.cpp src/bk72xx.cpp src/tuart.cpp src/uartlog.cpp && /tmp/bk_sim
    python3 tests/test_bk_host.py     # Beken BK72xx host commands against bk_sim (builds it)
    python3 tests/test_uartlog.py     # UART log: src/uartlog.cpp built natively, its export decoded by tlsr_c3.py
    python3 tests/test_ln_host.py     # LN882H host, Lightning's loader, src/lnscan.h: model of bridge, ROM, loader
    python3 tests/test_ln_loader.py   # LN882H, our loader: build, header, loader.c natively, the host through it
    python3 tests/test_ln_emu.py      # our LN882H loader's binary in the unicorn emulator (pip install unicorn)
    python3 tests/test_rtl_host.py    # RTL8710B host against a model of the ROM (handshakes, read, XMODEM, RAM programs)
    python3 tests/test_rtl2_host.py   # RTL8720C host against a model of the ROM console (dumps, fwd, hashq, eFuse program)
    python3 tests/test_wifi_host.py   # host over TCP (--host, the key, wifi, a stream, bridge-exit, --esp) against a model
    python3 tests/test_sniff.py       # two-line UART logger: src/sniffring.h natively, the host's live output, Tuya decoder

`jtag_sim` builds the real `src/swd.cpp` for the PC and runs it against a GPIO-level simulation of a JTAG
chain (Arm JTAG-DP with a MEM-AP, an STM32 boundary-scan TAP, a device without IDCODE): chain scan, DAP
selection, memory writes and reads, raw scans.

`wch_sim` builds the real `src/wch.cpp` and `src/wchflash.cpp` against stubs of the RMT driver whose "wire" is a
simulated CH32V003: it decodes every packet with the timing windows of WCH's debug manual and counts violations,
answers reads by holding the line low for a 0, and behind that models the debug module, a small RV32EC
interpreter for the program buffer, the flash controller, the option bytes with read protection, and a firmware
that takes the SWIO pin a set time after it starts. Its own checks cover the bit encoding and decoding (with
synthetic receiver captures, jitter and split pulses), the catch at power-up, failed attaches and their reasons,
memory, flash, boot area and option bytes, and the protected chip. `test_wch_host.py` runs `tlsr_c3.py --wch`
(info, rf, wf, es, ea, unlock, option bytes, `--run`, failed attaches) against it through `wch_sim --serve`.

`py32_sim` builds the real `src/swdflash.cpp` against a bus-level model of a Puya PY32F002B-type chip (the
MS32C001 of the Tuya dimmers): flash keys (a wrong key locks until reset), the page buffer with PG / PGSTRT,
page and mass erase, BSY, WRPERR, timing registers that must equal the factory bytes for the selected HSI range,
a hardware-mode watchdog, and a fault on any write to the option bytes or the factory area. `test_py32_host.py`
runs `tlsr_c3.py`'s identification, write, verify, page erase and mass erase against it through `py32_sim --serve`.

`bk_sim` builds the real `src/bk72xx.cpp` against a stub of the ESP-IDF UART driver whose wire is simulated in time:
every byte takes ten bit times at the sender's speed, and a byte received at another speed arrives garbled or not at
all. Behind it is a BK7231N BootROM or a BK7231T bootloader: the UART is ready a moment after power-on, a link check
must arrive within a window or the chip's own firmware starts and prints its log; in download mode the frames are
parsed strictly (malformed frames and unknown commands are counted as violations), the speed command switches after
its delay, and the flash has status register protection that silently ignores erase and write, as a real flash does.
Options make the UART fail above a speed, corrupt one byte in n above 115200, leave CEN / power unwired, remove the
chip, or give the short flash id answer. Its own checks cover the catch, the speed change and the fallback, reads
with noise, erase / write / CRC, the bootloader guard, protection, reboot, the BK7231T protocol and the failed links.
`test_bk_host.py` runs `tlsr_c3.py --bk` (info, rf --twice, wf with partial sectors and unchanged sectors skipped,
the bootloader guard and full images, LibreTiny UF2 files, OTA packages refused, ea, --run, unknown and Macronix
flash chips, the BK7231T, failed links) against it through `bk_sim --serve`.

`test_ln_host.py` and `test_ln_loader.py` share `ln_sim.py`: a model of the flasher's bridge in its `ln` mode (the
speed following of `src/lnscan.h`; bytes sent while the bridge and the chip run at different speeds arrive garbled),
the LN882H ROM (version, YMODEM download into RAM) and either RAM loader. Lightning's is modelled after its SDK source;
ours is the real `loaders/ln882h/loader.c` and `flash.c`, built with gcc as a shared library against
`tests/ln_loader_sim/fake_hal.c` (ctypes) with the SPI NOR model of `ln_nor_model.py` behind it (write enable latch,
busy polls with a violation for any other command meanwhile, page program that only clears bits and wraps in its
page, block protection that silently ignores erase and write, a one-byte status write that clears the second byte).
`test_ln_loader.py` checks that the loader embedded in `tlsr_c3.py` equals a fresh build (skipped without
PlatformIO's Arm toolchain), the boot header, and the header code against Lightning's binary when it is at hand
(`$LN882H_RAMCODE`); then runs `tlsr_c3.py --ln` through it: info, rf, wf with partial sectors, the same image again
(nothing erased or written), blank sectors, `--no-erase`, data with a `baudrate` line, es, ea, protected flash
chips, a damaged frame in each direction (also around a write, which is not done twice), the speed fallback, a flash
that does not answer, a loader that does not start, a loader whose UART comes up at 921600 (found by the host
listening at other speeds through the bridge's speed escape), a loader that talks but hears nothing (its wait lines,
no speed probing), `--run`; frames sent straight to it (bad CRC, bad escape, a raw CR, unknown command, ranges,
unaligned erase, writing over data that is not erased, repeated requests); the start lines and the wait line every
200 ms until `version` came; and the host's diagnosis for each way a loader can stay silent.
`test_ln_emu.py` runs the built image itself in the unicorn emulator (Cortex-M4) against register models of the clock
tree (crystal, PLL, AHB / APB0 dividers with their update bits; a switch to a PLL that is off stops the core), UART0
(its bit rate computed from the modelled APB0 and its divider: bytes at a rate more than 3 % off the host's arrive
garbled or are lost with a framing error; they pass only with its clock on, out of reset, enabled, and A2 / A3 given
to it), the QSPI controller (DesignWare SSI, with violations for configuration written while enabled, data while
disabled, no slave selected, unread data, an SPI clock above 25 MHz), the flash cache, the clock gates and pad
enables, the chip reset, SysTick, the NVIC and VTOR. Three ROM hand-over states: a UART divider that does not fit the
clock (the hello garbled, then everything readable after the loader's own set-up), one that fits (the hello
readable), and the PLL off (the crystal kept). Then the wait line and receive errors, version, info (the clocks set,
the ROM's registers), reads, erase, a write across a page, CRCs, a protected flash, a damaged frame, the speed change
and the reboot. Timing is not modelled there. It is skipped without the unicorn package.

`test_wifi_host.py` runs `tlsr_c3.py` over TCP against a local server thread that models the flasher's command side
(V, AUTH with a key, WIFI, an XR stream, BRIDGE with the escape and the rule that the bridge owner's address needs no
key): connecting with and without the key and with a wrong one, `wifi ap` / `wifi join` / `wifi key` with the SSID
and password in hex and neither the password nor the key in any output, a read stream over TCP and a connection
that breaks in the middle of one, `bridge-exit` over TCP, and `--esp` handing `socket://…` to a fake esptool that
reaches the "target" without the key. The firmware's Wi-Fi part (`src/wifilink.cpp`) needs the ESP-IDF network
stack and is not simulated.

`test_sniff.py` builds `src/sniffring.h` (the two-line UART logger's live output) natively as `tests/sniff_sim` and
feeds it made-up captures of both lines (pieces of 1 to 1500 bytes, events, a clock that wraps) with a host that
keeps up, a slow one that takes 150 bytes per piece and frames cut anywhere, and one that takes a byte now and then;
`tlsr_c3.py`'s frame parser reads the output in random pieces, and the test checks that every frame is a piece in
order (long ones split at 1 k) and that every gap is reported before the next frame with its exact size. The Tuya MCU
decoder is checked on frames split across records, two in one record, a bad checksum, garbage between frames, a
length no frame has, and every data point type. The host's `sniff` runs against a fake flasher over a
pseudo-terminal (as the USB port) and over TCP through the command line (frames shown and decoded, the escape after
a pause, drops counted, `--out`, `--at-boot`, `--no-at-boot`); `bridge-exit` finds the closing line behind the frames;
`log --tuya` shows a logger session with A / B. `test_uartlog.py` also checks the small store of a session with Wi-Fi
on and the case of too little heap for any store. The capture itself (`src/sniff.cpp`) needs the C3's UART drivers
and is not simulated.

None of this replaces a test on a real target.
