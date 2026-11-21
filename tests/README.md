# Tests without hardware

    python3 tests/test_streams.py     # host side of the XR / XW binary streams against a model of the firmware
    python3 tests/test_spi_host.py    # SPI flash host logic against a simulated Winbond / Macronix chip
    g++ -std=c++17 -I tests/jtag_sim/stub -I src -o /tmp/jtag_sim tests/jtag_sim/jtag_sim.cpp src/swd.cpp && /tmp/jtag_sim

`jtag_sim` builds the real `src/swd.cpp` for the PC and runs it against a GPIO-level simulation of a JTAG
chain (Arm JTAG-DP with a MEM-AP, an STM32 boundary-scan TAP, a device without IDCODE): chain scan, DAP
selection, memory writes and reads, raw scans. None of this replaces a test on a real target.
