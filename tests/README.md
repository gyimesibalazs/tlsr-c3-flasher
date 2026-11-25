# Tests without hardware

    python3 tests/test_streams.py     # host side of the XR / XW binary streams against a model of the firmware
    python3 tests/test_spi_host.py    # SPI flash host logic against a simulated Winbond / Macronix chip
    g++ -std=c++17 -I tests/jtag_sim/stub -I src -o /tmp/jtag_sim tests/jtag_sim/jtag_sim.cpp src/swd.cpp && /tmp/jtag_sim
    g++ -std=c++17 -Wall -Wextra -I tests/wch_sim/stub -I src -o /tmp/wch_sim tests/wch_sim/wch_sim.cpp src/wch.cpp src/wchflash.cpp && /tmp/wch_sim
    python3 tests/test_wch_host.py    # WCH host commands against the same simulation (builds it with g++)
    g++ -std=c++17 -Wall -Wextra -I tests/py32_sim/stub -I src -o /tmp/py32_sim tests/py32_sim/py32_sim.cpp src/swdflash.cpp && /tmp/py32_sim
    python3 tests/test_py32_host.py   # Puya PY32F002B / MS32C001 SWD host commands against py32_sim (builds it)
    g++ -std=c++17 -Wall -Wextra -I tests/bk_sim/stub -I src -o /tmp/bk_sim tests/bk_sim/bk_sim.cpp src/bk72xx.cpp && /tmp/bk_sim
    python3 tests/test_bk_host.py     # Beken BK72xx host commands against bk_sim (builds it)

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

None of this replaces a test on a real target.
