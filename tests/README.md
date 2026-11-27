# Tests without hardware

Run from the repository root. Needs Python 3 with pyserial and `g++` / `gcc` for the native builds.
`test_ln_emu.py` needs `pip install unicorn` (skipped without it); the loader build check in `test_ln_loader.py`
needs PlatformIO's Arm toolchain (skipped without it).

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
    python3 tests/test_ln_emu.py      # our LN882H loader's binary in the unicorn emulator
    python3 tests/test_rtl_host.py    # RTL8710B host against a model of the ROM (handshakes, read, XMODEM, RAM programs)
    python3 tests/test_rtl2_host.py   # RTL8720C host against a model of the ROM console (dumps, fwd, hashq, eFuse program)
    python3 tests/test_wifi_host.py   # host over TCP (--host, the key, wifi, a stream, bridge-exit, --esp) against a model
    python3 tests/test_sniff.py       # two-line UART logger: src/sniffring.h natively, the host's live output, Tuya decoder

What the simulators are:

- `jtag_sim`: the real `src/swd.cpp` against a GPIO-level JTAG chain (Arm JTAG-DP with a MEM-AP, an STM32
  boundary-scan TAP, a device without IDCODE).
- `wch_sim`: the real `src/wch.cpp` and `src/wchflash.cpp` against stubs of the RMT driver whose wire is a simulated
  CH32V003: packet timing checked against WCH's debug manual, debug module, a small RV32EC interpreter for the program
  buffer, flash controller, option bytes with read protection, and a firmware that takes the SWIO pin after a set
  time. `test_wch_host.py` drives it through `wch_sim --serve`.
- `py32_sim`: the real `src/swdflash.cpp` against a bus-level PY32F002B-type chip (flash keys, page buffer, erase,
  timing registers that must match the factory bytes, hardware watchdog, faults on writes to protected areas).
  `test_py32_host.py` drives it through `py32_sim --serve`.
- `bk_sim`: the real `src/bk72xx.cpp` against a UART stub simulated in time (a byte at the wrong speed arrives
  garbled), with a BK7231N BootROM or a BK7231T bootloader behind it, strict frame parsing and flash protection that
  silently ignores erase and write. Options inject failures (speed limits, noise, missing CEN / power, no chip).
  `test_bk_host.py` drives it through `bk_sim --serve`.
- `ln_sim.py` (used by `test_ln_host.py` and `test_ln_loader.py`): the bridge's speed following, the LN882H ROM and
  either RAM loader. Ours is the real `loaders/ln882h/loader.c` and `flash.c`, built as a shared library against
  `tests/ln_loader_sim/fake_hal.c` with the SPI NOR model of `ln_nor_model.py`. `test_ln_loader.py` also checks that
  the loader embedded in `tlsr_c3.py` equals a fresh build, and the boot header against Lightning's binary when
  `$LN882H_RAMCODE` points to it.
- `test_ln_emu.py`: the built loader image in the unicorn emulator (Cortex-M4) against register models of the clock
  tree, UART0 (bit rate from the modelled clock), the QSPI controller and the flash cache, with three ROM hand-over
  states. Timing is not modelled.
- `test_wifi_host.py`: `tlsr_c3.py` over TCP against a local model of the flasher's command side (key, `wifi`
  commands with no secrets in the output, streams, broken connections, `bridge-exit`, `--esp` with `socket://`).
- `test_sniff.py`: `src/sniffring.h` built natively as `tests/sniff_sim`, fed with captures and hosts of different
  speeds (every gap must be reported with its size); the host's `sniff` against a fake flasher over a pseudo-terminal
  and over TCP; the Tuya decoder.

Not simulated: the firmware's Wi-Fi part (`src/wifilink.cpp`) and the logger's capture (`src/sniff.cpp`), which need
the ESP-IDF drivers. None of this replaces a test on a real target.
