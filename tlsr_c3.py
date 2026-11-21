#!/usr/bin/env python3
"""Host side of the ESP32-C3 Telink TLSR825x flasher.

Examples:
    python3 tlsr_c3.py -p /dev/ttyACM0 info
    python3 tlsr_c3.py -p /dev/ttyACM0 rf 0 0x100000 stock.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 wf 0 firmware.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 --rate 1500000 rf 0 0x80000 stock.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 --mode uart info
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd info                     # Arm SWD target (EFR32, nRF52, STM32)
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd rf 0 0x40000 efr32.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --boot hi        # UART bridge, BOOT pin high
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit
"""
import argparse
import base64
import os
import sys
import time
import zlib

import serial

SECTOR = 4096
PAGE = 256
READ_CHUNK = 1024
CRC_CHUNK = 4096


class FlasherError(RuntimeError):
    pass


class Flasher:
    def __init__(self, port, baud=460800):
        self.rxbuf = b''
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        # fixed and short: changing it per command reconfigures the port, which costs about 25 ms on Windows
        self.ser.timeout = 0.1
        # keep the control lines quiet, they reset some C3 boards
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        last = None
        for _ in range(5):
            try:
                self.version = self.cmd('V', timeout=1)
                return
            except FlasherError as e:
                last = e
                self.ser.reset_input_buffer()
                self.rxbuf = b''
        raise FlasherError('the flasher does not answer on %s (%s)' % (port, last))

    def readline(self, timeout=5):
        """Like serial.readline(), but takes everything that has arrived in one call: reading a long
        answer one byte at a time is slow on Windows."""
        deadline = time.time() + timeout
        while b'\n' not in self.rxbuf:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self.rxbuf += chunk
            elif time.time() >= deadline:
                break
        line, sep, rest = self.rxbuf.partition(b'\n')
        self.rxbuf = rest if sep else b''
        return line

    def cmd(self, text, timeout=5):
        self.ser.write(text.encode() + b'\n')
        answer = self.readline(timeout).decode(errors='replace').strip()
        if answer == 'OK':
            return ''
        if answer.startswith('OK '):
            return answer[3:]
        if answer.startswith('ERR'):
            raise FlasherError('%s -> %s' % (text.split()[0], answer))
        raise FlasherError('%s -> no or bad answer: %r' % (text.split()[0], answer))

    def activate(self, tact_ms):
        return self.cmd('A %d' % tact_ms, timeout=tact_ms / 1000 + 10)

    def read_flash(self, addr, size, progress=True):
        out = bytearray()
        while len(out) < size:
            n = min(READ_CHUNK, size - len(out))
            if progress:
                print('\rRead 0x%06x' % (addr + len(out)), end='', flush=True)
            for attempt in range(3):
                try:
                    answer = self.cmd('FB %d %d' % (addr + len(out), n))
                    data = b'\xff' * n if answer == '*' else base64.b64decode(answer, validate=True)
                    if len(data) != n:
                        raise FlasherError('short answer')
                    break
                except (FlasherError, ValueError):
                    if attempt == 2:
                        if progress:
                            print()
                        raise
            out += data
        if progress:
            print('\r' + ' ' * 20 + '\r', end='')
        return bytes(out)

    def compare_flash(self, addr, data, label='Compare'):
        """Check the flash against data by CRC, block by block; returns the address of the first bad block."""
        pos = 0
        while pos < len(data):
            n = min(CRC_CHUNK, len(data) - pos)
            print('\r%s 0x%06x' % (label, addr + pos), end='', flush=True)
            crc = int(self.cmd('FC %d %d' % (addr + pos, n), timeout=20), 16)
            if crc != zlib.crc32(data[pos:pos + n]):
                print()
                return addr + pos
            pos += n
        print('\r' + ' ' * 24 + '\r', end='')
        return None

    def write_flash(self, addr, data, erase=True):
        self.cmd('FU')
        erased = None
        if erase and addr % SECTOR:
            erased = addr - addr % SECTOR   # as in pvvx's tool: a partly covered first sector is kept
        pos = 0
        while pos < len(data):
            a = addr + pos
            n = min(PAGE - a % PAGE, len(data) - pos)
            page = data[pos:pos + n]
            sector = a - a % SECTOR
            if erase and sector != erased:
                print('\rErase 0x%06x' % sector, end='', flush=True)
                self.cmd('FE %d' % sector)
                erased = sector
            if page.count(0xff) != len(page):
                print('\rWrite 0x%06x' % a, end='', flush=True)
                self.cmd('FW %d %s' % (a, page.hex()))
            pos += n
        print('\r' + ' ' * 20 + '\r', end='')


def auto_int(x):
    return int(x, 0)


def watch_pins(f, seconds, pull):
    """TLSR825x: every pin (except PA7 = SWS) becomes a GPIO input, then the input registers are polled."""
    def ana_write(addr, value):
        f.cmd('S 0xb8 %02x%02x60' % (addr, value))   # analog register: address, data, start + write
        time.sleep(0.002)
        f.cmd('S 0xba 00')

    def ana_read(addr):
        f.cmd('S 0xb8 %02x0040' % addr)              # start, read
        time.sleep(0.002)
        value = bytes.fromhex(f.cmd('G 0xb9 1'))[0]
        f.cmd('S 0xba 00')
        return value

    # The chip was stopped right after reset: release the resets and switch the clocks of its
    # blocks on, as the SDK's start-up code does. Without it the analog registers (pull resistors,
    # input enable of ports B and C) cannot be written.
    f.cmd('S 0x60 000000ffffff')
    time.sleep(0.01)
    f.cmd('S 0x586 7f')                               # port A: GPIO function, PA7 stays SWS
    for reg in (0x58e, 0x596, 0x59e):                 # ports B, C, D: GPIO function
        f.cmd('S 0x%x ff' % reg)
    for reg in (0x582, 0x58a, 0x592, 0x59a):          # outputs off
        f.cmd('S 0x%x ff' % reg)
    pa_ie = bytes.fromhex(f.cmd('G 0x581 1'))[0]
    f.cmd('S 0x581 %02x' % (pa_ie | 0x7f))            # input enable, ports A and D
    f.cmd('S 0x599 ff')
    ana_write(0xbd, 0xff)                             # input enable, ports B and C
    ana_write(0xc0, 0xff)
    code = {'up': 0xff, 'down': 0xaa, 'none': 0x00}[pull]
    for reg in range(0x0e, 0x16):                     # two bits per pin: 3 = 10 k up, 2 = 100 k down
        if reg == 0x0f:
            ana_write(reg, (ana_read(reg) & 0xc0) | (code & 0x3f))   # PA7 untouched
        else:
            ana_write(reg, code)

    check = ana_read(0x0e)
    if check != code:
        print('Warning: the pull resistors were not accepted (register 0x0e reads %02x, wanted %02x).' % (check, code))

    def read_ports():
        return [bytes.fromhex(f.cmd('G 0x%x 1' % reg))[0] for reg in (0x580, 0x588, 0x590, 0x598)]

    def names(ports):
        return ' '.join('%s=%s' % (n, format(v, '08b')) for n, v in zip('ABCD', ports))

    last = read_ports()
    print('Pull: %s. Bits are pins 7..0 of each port (PA7 is the SWS line itself).' % pull)
    print('Start:  %s' % names(last))
    print('Watching for %.0f s: turn, press, switch...' % seconds)
    t0 = time.time()
    polls = 0
    while time.time() - t0 < seconds:
        now = read_ports()
        polls += 1
        if now != last:
            changed = []
            for port, a, b in zip('ABCD', last, now):
                for bit in range(8):
                    if (a ^ b) >> bit & 1 and not (port == 'A' and bit == 7):
                        changed.append('P%s%d %d->%d' % (port, bit, a >> bit & 1, b >> bit & 1))
            if changed:
                print('%6.2f s  %s' % (time.time() - t0, ', '.join(changed)))
            last = now
    print('End:    %s   (%d polls, %.0f per second)' % (names(last), polls, polls / seconds))


def find_led(f, skip, hold, only=''):
    """TLSR825x: every pin in turn becomes an output, low and then high, the others stay inputs."""
    f.cmd('S 0x60 000000ffffff')                      # clocks on, see watch_pins
    time.sleep(0.01)
    f.cmd('S 0x586 7f')                               # GPIO function everywhere, PA7 stays SWS
    for reg in (0x58e, 0x596, 0x59e):
        f.cmd('S 0x%x ff' % reg)
    for reg in (0x582, 0x58a, 0x592, 0x59a):          # all outputs off
        f.cmd('S 0x%x ff' % reg)
    skipped = {x.strip().upper() for x in skip.split(',') if x.strip()}
    skipped.add('A7')
    order = [(port, bit) for port in 'BCDA' for bit in range(8) if '%s%d' % (port, bit) not in skipped]
    wanted = [x.strip().upper() for x in only.split(',') if x.strip()]
    if wanted:
        order = [(x[0], int(x[1])) for x in wanted if x not in skipped] * 3
    print('Watch the LED. %d pins, %.0f s low and %.0f s high each (about %.0f s in all).' %
          (len(order), hold, hold, len(order) * 2 * hold))
    print('Note the line that is on the screen when the LED lights up.')
    for port, bit in order:
        base = 0x580 + 8 * 'ABCD'.index(port)
        out_reg, oen_reg = base + 3, base + 2
        for level in (0, 1):
            f.cmd('S 0x%x %02x' % (out_reg, (1 << bit) if level else 0))
            f.cmd('S 0x%x %02x' % (oen_reg, 0xff & ~(1 << bit)))   # 0 = output enabled
            print('  P%s%d %s' % (port, bit, 'HIGH' if level else 'low'), flush=True)
            time.sleep(hold)
        f.cmd('S 0x%x ff' % oen_reg)                  # back to input
        f.cmd('S 0x%x 00' % out_reg)
    print('Done, all pins are inputs again.')


# ---------------------------------------------------------------- SWD

# MEM-AP / Cortex-M registers
CPUID = 0xE000ED00
DHCSR = 0xE000EDF0
CORTEX = {0xC20: 'Cortex-M0', 0xC60: 'Cortex-M0+', 0xC23: 'Cortex-M3', 0xC24: 'Cortex-M4', 0xC27: 'Cortex-M7',
          0xD20: 'Cortex-M23', 0xD21: 'Cortex-M33', 0xD22: 'Cortex-M55'}
AP_IDR = {0x24770011: 'AHB-AP (Cortex-M3/M4)', 0x04770031: 'AHB-AP (Cortex-M0+)', 0x84770001: 'AHB-AP',
          0x14770015: 'AHB5-AP (Cortex-M33)', 0x74770001: 'AHB-AP (Cortex-M0)', 0x16E60001: 'Silabs AAP (debug locked?)',
          0x02880000: 'nRF CTRL-AP'}
EFR32_SERIES1_FAMILY = {16: 'EFR32MG1P', 17: 'EFR32MG1B', 18: 'EFR32MG1V', 19: 'EFR32BG1P', 25: 'EFR32FG1P',
                        28: 'EFR32MG12P', 31: 'EFR32BG12P', 37: 'EFR32FG12P', 40: 'EFR32MG13P', 43: 'EFR32BG13P',
                        46: 'EFR32FG13P', 49: 'EFR32MG14P', 52: 'EFR32BG14P', 55: 'EFR32FG14P'}


def swd_word(f, addr, ap=0):
    data = base64.b64decode(f.cmd('SM %d %d 4' % (ap, addr)))
    return int.from_bytes(data, 'little')


class SwdTarget:
    def __init__(self, name='', flash=None, page=None, family=None, base=0, sfam_args='', sectors=None, rp2040=False,
                 write_unit=None, erase_unit=None, ti=False):
        self.name, self.flash, self.page, self.family = name, flash, page, family
        self.write_unit = write_unit     # one SFW may not cross this (the SAM page buffer); None = any
        self.erase_unit = erase_unit or page
        self.base = base                 # address of the first flash byte
        self.sfam_args = sfam_args       # extra SFAM arguments (STM32 page size, second bank)
        self.sectors = sectors           # erase units as (start, size), for chips with uneven sectors
        self.rp2040 = rp2040
        self.ti = ti                     # TI CC13xx / CC26xx: written by the ROM routines, over cJTAG

    def erase_units(self, start, end):
        """(address, size) of every erase unit that overlaps start..end-1."""
        if self.sectors:
            return [(a, n) for a, n in self.sectors if a < end and a + n > start]
        u = self.erase_unit
        first = start - (start - self.base) % u
        return [(a, u) for a in range(first, end, u)]


# STM32: DBGMCU DEV_ID -> (name, writer, erase page, flash size register, dual bank rule)
#   writer: stm32f1 (F0 / F1 / F3 registers), stm32f4, stm32v2 (G0 / G4 / L4 / WB), None = identify only
STM32 = {
    0x440: ('STM32F05x', 'stm32f1', 1024, 0x1FFFF7CC, None), 0x444: ('STM32F03x', 'stm32f1', 1024, 0x1FFFF7CC, None),
    0x445: ('STM32F04x', 'stm32f1', 1024, 0x1FFFF7CC, None), 0x448: ('STM32F07x', 'stm32f1', 2048, 0x1FFFF7CC, None),
    0x442: ('STM32F09x', 'stm32f1', 2048, 0x1FFFF7CC, None),
    0x412: ('STM32F10x low density', 'stm32f1', 1024, 0x1FFFF7E0, None),
    0x410: ('STM32F10x medium density', 'stm32f1', 1024, 0x1FFFF7E0, None),
    0x414: ('STM32F10x high density', 'stm32f1', 2048, 0x1FFFF7E0, None),
    0x418: ('STM32F105/107', 'stm32f1', 2048, 0x1FFFF7E0, None),
    0x420: ('STM32F100 value line', 'stm32f1', 1024, 0x1FFFF7E0, None),
    0x428: ('STM32F100 value line HD', 'stm32f1', 2048, 0x1FFFF7E0, None),
    0x430: ('STM32F10x XL density (two flash controllers, not supported)', None, 2048, 0x1FFFF7E0, None),
    0x422: ('STM32F30x/F31x', 'stm32f1', 2048, 0x1FFFF7CC, None), 0x432: ('STM32F37x', 'stm32f1', 2048, 0x1FFFF7CC, None),
    0x438: ('STM32F303x6/8', 'stm32f1', 2048, 0x1FFFF7CC, None), 0x439: ('STM32F301/302x6/8', 'stm32f1', 2048, 0x1FFFF7CC, None),
    0x446: ('STM32F303xD/E', 'stm32f1', 2048, 0x1FFFF7CC, None),
    0x413: ('STM32F405/407/415/417', 'stm32f4', 0, 0x1FFF7A22, None),
    0x419: ('STM32F42x/43x', 'stm32f4', 0, 0x1FFF7A22, 'f4dual'),
    0x423: ('STM32F401xB/C', 'stm32f4', 0, 0x1FFF7A22, None), 0x433: ('STM32F401xD/E', 'stm32f4', 0, 0x1FFF7A22, None),
    0x431: ('STM32F411', 'stm32f4', 0, 0x1FFF7A22, None), 0x458: ('STM32F410', 'stm32f4', 0, 0x1FFF7A22, None),
    0x441: ('STM32F412', 'stm32f4', 0, 0x1FFF7A22, None), 0x421: ('STM32F446', 'stm32f4', 0, 0x1FFF7A22, None),
    0x463: ('STM32F413/423', 'stm32f4', 0, 0x1FFF7A22, None),
    0x466: ('STM32G03x/G04x', 'stm32v2', 2048, 0x1FFF75E0, None), 0x460: ('STM32G07x/G08x', 'stm32v2', 2048, 0x1FFF75E0, None),
    0x456: ('STM32G05x/G06x', 'stm32v2', 2048, 0x1FFF75E0, None),
    0x467: ('STM32G0Bx/G0Cx (dual bank not handled)', None, 2048, 0x1FFF75E0, None),
    0x468: ('STM32G431/441', 'stm32v2', 2048, 0x1FFF75E0, None),
    0x469: ('STM32G47x/48x', 'stm32v2', 2048, 0x1FFF75E0, 'g4dbank'),
    0x479: ('STM32G491/4A1', 'stm32v2', 2048, 0x1FFF75E0, None),
    0x435: ('STM32L43x/44x', 'stm32v2', 2048, 0x1FFF75E0, None), 0x462: ('STM32L45x/46x', 'stm32v2', 2048, 0x1FFF75E0, None),
    0x464: ('STM32L41x/42x', 'stm32v2', 2048, 0x1FFF75E0, None), 0x415: ('STM32L47x/48x', 'stm32v2', 2048, 0x1FFF75E0, 'half'),
    0x461: ('STM32L49x/4Ax', 'stm32v2', 2048, 0x1FFF75E0, 'l4dualbank'),
    0x495: ('STM32WB55', 'stm32v2', 4096, 0x1FFF75E0, None),
    0x443: ('STM32C01x', 'stm32v2', 2048, 0x1FFF75A0, None), 0x453: ('STM32C03x', 'stm32v2', 2048, 0x1FFF75A0, None),
    0x44C: ('STM32C05x', 'stm32v2', 2048, 0x1FFF75A0, None), 0x493: ('STM32C071', 'stm32v2', 2048, 0x1FFF75A0, None),
    0x44D: ('STM32C09x', 'stm32v2', 2048, 0x1FFF75A0, None),
    0x457: ('STM32L0 cat. 1', 'stm32l0', 128, 0x1FF8007C, 0x40022000),
    0x425: ('STM32L0 cat. 2', 'stm32l0', 128, 0x1FF8007C, 0x40022000),
    0x417: ('STM32L0 cat. 3', 'stm32l0', 128, 0x1FF8007C, 0x40022000),
    0x447: ('STM32L0 cat. 5', 'stm32l0', 128, 0x1FF8007C, 0x40022000),
    0x416: ('STM32L1 cat. 1', 'stm32l0', 256, 0x1FF8004C, 0x40023C00),
    0x429: ('STM32L1 cat. 2', 'stm32l0', 256, 0x1FF8004C, 0x40023C00),
    0x427: ('STM32L1 cat. 3', 'stm32l0', 256, 0x1FF800CC, 0x40023C00),
    0x436: ('STM32L1 cat. 4 / cat. 3', 'stm32l0', 256, 0x1FF800CC, 0x40023C00),
    0x437: ('STM32L1 cat. 5 / cat. 6', 'stm32l0', 256, 0x1FF800CC, 0x40023C00),
    0x455: ('STM32U535/545', 'stm32u5', 8192, 0x0BFA07A0, 'half'),
    0x482: ('STM32U575/585', 'stm32u5', 8192, 0x0BFA07A0, 'half'),
    0x481: ('STM32U59x/5Ax', 'stm32u5', 8192, 0x0BFA07A0, 'half'),
    0x476: ('STM32U5Fx/5Gx', 'stm32u5', 8192, 0x0BFA07A0, 'half'),
    0x450: ('STM32H74x/75x (not supported)', None, 0, 0x1FF1E880, None),
    0x449: ('STM32F74x/75x (not supported)', None, 0, 0x1FF0F442, None),
}
STM32_FLASH = 0x08000000


def stm32_sectors_f4(size, dual):
    """F4 sectors: 16 k x 4, 64 k, then 128 k; the 2 Mbyte parts repeat the map in the second bank."""
    def bank(base, n):
        out, a = [], base
        for sz in [0x4000] * 4 + [0x10000] + [0x20000] * 64:
            if a - base >= n:
                break
            out.append((a, sz))
            a += sz
        return out
    if dual:
        return bank(STM32_FLASH, size // 2) + bank(STM32_FLASH + size // 2, size // 2)
    return bank(STM32_FLASH, size)


# EFR32 Series 2: family number -> bit of the MSC clock in CMU_CLKEN1 (Simplicity SDK device headers; 0 = none)
EFR32S2_MSC_CLOCK = {21: 0, 22: 17, 23: 16, 24: 16, 25: 16, 26: 16, 27: 17, 28: 16, 29: 17}


def efr32_info(f, say, part):
    if part == 0xC24:                                  # Series 1 (Cortex-M4), DEVINFO at 0x0FE081B0
        part_reg, msize, meminfo = swd_word(f, 0x0FE081FC), swd_word(f, 0x0FE081F8), swd_word(f, 0x0FE081E4)
        fam = (part_reg >> 16) & 0xff
        if fam in EFR32_SERIES1_FAMILY:
            t = SwdTarget('%s%d' % (EFR32_SERIES1_FAMILY[fam], part_reg & 0xffff), (msize & 0xffff) * 1024,
                          1 << ((((meminfo >> 24) & 0xff) + 10) & 0xff), 'efr32s1')
            say('Chip:    %s (Silicon Labs Series 1), flash %d kbytes in %d-byte pages, RAM %d kbytes' % (
                t.name, t.flash // 1024, t.page, msize >> 16))
            return t
    if part == 0xD21:                                  # Series 2 (Cortex-M33), DEVINFO at 0x0FE08000
        part_reg, msize, meminfo = swd_word(f, 0x0FE08010), swd_word(f, 0x0FE08018), swd_word(f, 0x0FE08014)
        letters = {0: 'FG', 1: 'MG', 2: 'BG'}
        famnum = (part_reg >> 16) & 0x3f
        if famnum in EFR32S2_MSC_CLOCK:
            letter = letters.get((part_reg >> 24) & 0x3f, '?%d' % ((part_reg >> 24) & 0x3f))
            t = SwdTarget('EFR32%s%d (device %d)' % (letter, famnum, part_reg & 0xffff), (msize & 0xffff) * 1024,
                          1 << (((meminfo & 0xff) + 10) & 0xff), 'efr32s2', sfam_args=' %d' % EFR32S2_MSC_CLOCK[famnum])
            say('Chip:    %s (Silicon Labs Series 2), flash %d kbytes in %d-byte pages, RAM %d kbytes' % (
                t.name, t.flash // 1024, t.page, (msize >> 16) & 0x7ff))
            return t
    return None


def nrf_info(f, say, part):
    if part in (0xC24, 0xC20, 0xC60):                  # nRF52 (Cortex-M4 / M0): FICR at 0x10000000
        info_part = swd_word(f, 0x10000100)
        if 0x52000 <= info_part <= 0x52fff:
            t = SwdTarget('nRF%x' % info_part, swd_word(f, 0x10000110) * 1024, swd_word(f, 0x10000010), 'nrf52')
            say('Chip:    %s, flash %d kbytes in %d-byte pages' % (t.name, t.flash // 1024, t.page))
            return t
    if part == 0xD21:                                  # nRF5340 application core: FICR at 0x00FF0000
        if swd_word(f, 0x00FF020C) == 0x5340:
            t = SwdTarget('nRF5340 (application core)', swd_word(f, 0x00FF021C) * 1024, swd_word(f, 0x00FF0220),
                          'nrf53')
            say('Chip:    %s, flash %d kbytes in %d-byte pages (the network core is not handled)' % (
                t.name, t.flash // 1024, t.page))
            return t
    return None


def samd_info(f, say, part):
    did = swd_word(f, 0x41002018)                      # DSU DID
    proc = did >> 28
    if proc not in (1, 6):                             # 1: Cortex-M0+ (D2x / L2x / C2x), 6: Cortex-M4 (D5x / E5x)
        return None
    param = swd_word(f, 0x41004008)                    # NVMCTRL PARAM: number of pages and page size
    pages, page = param & 0xffff, 8 << ((param >> 16) & 7)
    if not pages:
        return None
    if proc == 1:
        t = SwdTarget('SAM D2x/L2x/C2x (DID 0x%08x)' % did, pages * page, page, 'samd21',
                      write_unit=page, erase_unit=4 * page)           # erased by rows of 4 pages
    else:
        t = SwdTarget('SAM D5x/E5x (DID 0x%08x)' % did, pages * page, page, 'samd51',
                      write_unit=page, erase_unit=16 * page)          # erased by 8 k blocks
    say('Chip:    %s, flash %d kbytes in %d-byte pages' % (t.name, t.flash // 1024, page))
    return t


def stm32_info(f, say, part=None):
    dev = 0
    for reg in (0xE0042000, 0x40015800, 0xE0044000):  # DBGMCU_IDCODE: M3/M4/M7/M33, M0/M0+, U5
        try:
            v = swd_word(f, reg)
        except FlasherError:
            continue
        if v & 0xfff in STM32:
            dev = v & 0xfff
            break
    if not dev:
        return None
    name, writer, page, size_reg, dual = STM32[dev]
    try:
        word = swd_word(f, size_reg & ~3)
        kb = (word >> (8 * (size_reg & 2))) & 0xffff
    except FlasherError:
        kb = 0
    size = kb * 1024 or None
    bank2, sectors, sfam = 0, None, ''
    if writer == 'stm32f4':
        sectors = stm32_sectors_f4(size or 0, dual == 'f4dual' and kb == 2048)
        bank2 = STM32_FLASH + size // 2 if dual == 'f4dual' and kb == 2048 else 0
        sfam = ' 0 %d' % bank2
    elif writer == 'stm32v2':
        optr = swd_word(f, 0x40022020)
        if dual == 'g4dbank':                          # DBANK: two banks of 2 k pages, else one bank of 4 k pages
            if optr & (1 << 22):
                bank2 = STM32_FLASH + size // 2
            else:
                page = 4096
        elif dual == 'l4dualbank' and optr & (1 << 21) or dual == 'half':
            bank2 = STM32_FLASH + size // 2
        sfam = ' %d %d' % (page, bank2)
    elif writer == 'stm32f1':
        sfam = ' %d' % page
    elif writer == 'stm32u5':
        if swd_word(f, 0x40022040) & (1 << 31):
            say('         TrustZone is on (TZEN): the secure flash cannot be written through the non-secure registers')
            writer = None
        bank2 = STM32_FLASH + size // 2
        sfam = ' %d %d' % (page, bank2)
    elif writer == 'stm32l0':
        sfam = ' %d' % dual                            # the flash register base (0x40022000 / 0x40023C00)
    label = name + (' (DEV_ID 0x%03x; GD32 / CH32 compatibles report the same)' if writer == 'stm32f1' else
                    ' (DEV_ID 0x%03x)') % dev
    t = SwdTarget(label, size, page or None, writer, STM32_FLASH, sfam, sectors)
    say('Chip:    %s, flash %s%s' % (t.name, '%d kbytes' % kb if kb else 'size unknown',
                                     ', second bank at 0x%08x' % bank2 if bank2 else ''))
    if writer is None:
        say('         identification only: this family has no flash routine yet')
    # read protection
    try:
        if writer == 'stm32f1':
            rdp = swd_word(f, 0x4002201C) & 2                         # FLASH_OBR.RDPRT
        elif writer == 'stm32f4':
            rdp = ((swd_word(f, 0x40023C14) >> 8) & 0xff) != 0xAA     # FLASH_OPTCR.RDP
        elif writer == 'stm32v2':
            rdp = (swd_word(f, 0x40022020) & 0xff) != 0xAA            # FLASH_OPTR.RDP
        elif writer == 'stm32u5':
            rdp = (swd_word(f, 0x40022040) & 0xff) != 0xAA
        elif writer == 'stm32l0':
            rdp = (swd_word(f, dual + 0x1C) & 0xff) != 0xAA          # OPTR / OBR
        else:
            rdp = 0
        if rdp:
            say('         READ PROTECTED (RDP): the flash cannot be read; removing it erases the chip')
    except FlasherError:
        pass
    return t


RP2040_TARGETSEL = 0x01002927      # core 0
RP2040_CHIP_ID = 0x40000000


def swd_connect_any(f):
    """Plain SWD first, then the RP2040's multi-drop port. Returns (idcode, multidrop).
    With f.cjtag: cJTAG through the TI ICEPick router instead, idcode is then the JTAG-DP's."""
    if getattr(f, 'cjtag', False):
        icepick, dap = (int(x, 16) for x in f.cmd('TC').split())
        f.icepick = icepick
        return dap, False
    try:
        return int(f.cmd('SC'), 16), False
    except FlasherError as first:
        try:
            return int(f.cmd('SC md %d' % RP2040_TARGETSEL), 16), True
        except FlasherError:
            raise first


def swd_info(f, quiet=False):
    """Identify an Arm SWD target without changing it. family: the SFAM name for flash writing, or None."""
    say = (lambda *a: None) if quiet else print
    idcode, multidrop = swd_connect_any(f)
    if getattr(f, 'cjtag', False):
        say('ICEPick: 0x%08x' % f.icepick)
    say('IDCODE:  0x%08x (designer 0x%03x, part 0x%04x, version %d)' % (
        idcode, (idcode >> 1) & 0x7ff, (idcode >> 12) & 0xffff, idcode >> 28))
    idrs = []
    for ap in range(4):
        try:
            idr = int(f.cmd('SAR %d 0xfc' % ap), 16)
        except FlasherError:
            break
        if idr == 0:
            break
        idrs.append(idr)
        say('AP %d:    IDR 0x%08x %s' % (ap, idr, AP_IDR.get(idr, '')))
    if not idrs or idrs[0] == 0x16E60001:
        say('The memory AP is not reachable: the chip is probably debug locked (only a mass erase opens it).')
        return SwdTarget()
    cpuid = swd_word(f, CPUID)
    part = (cpuid >> 4) & 0xfff
    say('CPU:     %s (CPUID 0x%08x)' % (CORTEX.get(part, 'unknown'), cpuid))
    say('Core:    %s' % ('halted' if swd_word(f, DHCSR) & (1 << 17) else 'running'))
    if multidrop:
        chip = swd_word(f, RP2040_CHIP_ID)
        if (chip & 0x0fffffff) == 0x0002927:
            t = SwdTarget('RP2040 (revision %d)' % (chip >> 28), None, 4096, 'rp2040', 0x10000000, rp2040=True)
            say('Chip:    %s, external QSPI flash at 0x10000000 (size not read: give the size to rf)' % t.name)
            return t
    if getattr(f, 'cjtag', False):
        t = ti_info(f, say)
        if t is not None:
            return t
    # each probe reads addresses that do not exist on other chips: a bus fault there just means "not this one"
    for probe in (efr32_info, nrf_info, stm32_info, samd_info):
        try:
            t = probe(f, say, part)
        except FlasherError:
            t = None
        if t is not None:
            return t
    say('Chip:    not recognised; reading works, writing needs a known chip')
    return SwdTarget()


# ---- TI CC13xx / CC26xx (cJTAG): identified from FCFG1, written by the boot ROM's flash routines (HAPI)
TI_ICEPICK = {0x0b99a02f: ('CC26x0', 0x1000), 0x0b9bd02f: ('CC26x1', 0x1000), 0x0b9be02f: ('CC13x0', 0x1000),
              0x0bb4102f: ('CC26x2 / CC13x2', 0x2000)}
TI_FCFG1_ICEPICK_ID, TI_FCFG1_USER_ID, TI_FLASH_SIZE = 0x50001318, 0x50001294, 0x4003002C
TI_HAPI = 0x10000048                    # ROM table: [3] sector erase (addr), [5] program (buf, addr, count)
TI_BKPT, TI_BUF, TI_STACK = 0x20000000, 0x20000100, 0x20003000
TI_DAP_ENABLE = 0xC5                    # CCFG_TAP_DAP_0.CPU_DAP_ENABLE: any other value locks the debug port


def ti_info(f, say):
    ice = swd_word(f, TI_FCFG1_ICEPICK_ID)
    known = TI_ICEPICK.get(ice & 0x0fffffff)
    if known is None:
        say('Chip:    TI, ICEPick id 0x%08x in FCFG1 is not known' % ice)
        return None
    name, sector = known
    if ice & 0x0fffffff == 0x0bb4102f:
        name = 'CC13x2' if swd_word(f, TI_FCFG1_USER_ID) & 0x00800000 else 'CC26x2'
    elif ice & 0x0fffffff == 0x0b99a02f and swd_word(f, TI_FCFG1_USER_ID) & 0x00800000:
        name = 'CC13x0'
    size = (swd_word(f, TI_FLASH_SIZE) & 0xff) * sector
    t = SwdTarget('%s (revision %d)' % (name, ice >> 28), size, sector, 'ti', ti=True)
    say('Chip:    %s, %d k flash in %d k sectors, the last one holds the CCFG' % (t.name, size // 1024, sector // 1024))
    return t


def ti_ccfg_check(t, last_sector):
    """The CCFG sits at the end of the last sector: refuse an image that would lock the debug port."""
    off = t.page - 0x1C + 2             # CCFG_TAP_DAP_0, bits 23:16
    if last_sector[off] != TI_DAP_ENABLE:
        raise FlasherError('the CCFG in this image disables the debug port (CPU_DAP_ENABLE 0x%02x, not 0x%02x): '
                           'the chip could not be written over cJTAG again' % (last_sector[off], TI_DAP_ENABLE))


def ti_write(f, t, addr, data):
    """Whole sectors through the ROM: what the image does not cover is read back first and kept."""
    first = addr - addr % t.page
    end = addr + len(data)
    end += -end % t.page
    if end > t.flash:
        raise FlasherError('0x%x bytes at 0x%x do not fit into %d kbytes of flash' % (len(data), addr, t.flash // 1024))
    old = bytearray(swd_read(f, first, end - first, progress=False))
    old[addr - first:addr - first + len(data)] = data
    if end == t.flash:
        ti_ccfg_check(t, old[-t.page:])
    f.cmd('SHALT')                       # no reset: a system reset drops the cJTAG connection
    f.cmd('SMW 0 %d %s' % (TI_BKPT, (0xBE00BE00).to_bytes(4, 'little').hex()))   # bkpt; bkpt
    erase = swd_word(f, TI_HAPI + 3 * 4)
    program = swd_word(f, TI_HAPI + 5 * 4)

    def call(fn, *args):
        r = int(f.cmd('SCALL %d %d %d %s' % (fn, TI_STACK, TI_BKPT, ' '.join(str(a) for a in args)), timeout=25), 16)
        if r:
            raise FlasherError('ROM flash routine failed with status 0x%x at 0x%08x' % (r, args[-2] if len(args) > 1
                                                                                        else args[0]))
    for a in range(first, end, t.page):
        sector = bytes(old[a - first:a - first + t.page])
        print('\rErase/write 0x%08x' % a, end='', flush=True)
        call(erase, a)
        if sector.count(0xff) == len(sector):
            continue
        for k in range(0, t.page, 0x1000):            # 4 k through the RAM buffer at a time
            part = sector[k:k + 0x1000]
            if part.count(0xff) == len(part):
                continue
            for j in range(0, len(part), SWD_WRITE_CHUNK):
                f.cmd('SMW 0 %d %s' % (TI_BUF + j, part[j:j + SWD_WRITE_CHUNK].hex()))
            call(program, TI_BUF, a + k, len(part))
    print()


SWD_WRITE_CHUNK = 512          # bytes per SFW line: the flasher's command line holds about 1400 characters


# ---- RP2040: the flash is written by the boot ROM's own routines, run on core 0
RP_BKPT, RP_BUF, RP_STACK = 0x20000000, 0x20001000, 0x20040000
RP_SECTOR, RP_BLOCK = 4096, 65536


def rp2040_rom_funcs(f):
    table = swd_word(f, 0x14) & 0xffff                  # pointer to the ROM function table
    funcs, a = {}, table
    while True:
        w = swd_word(f, a & ~3)
        entry = (w >> (8 * (a & 2))) & 0xffff
        w2 = swd_word(f, (a + 2) & ~3)
        ptr = (w2 >> (8 * ((a + 2) & 2))) & 0xffff
        if entry == 0:
            break
        funcs[chr(entry & 0xff) + chr(entry >> 8)] = ptr
        a += 4
    need = ('IF', 'EX', 'RE', 'RP', 'FC', 'CX')
    missing = [k for k in need if k not in funcs]
    if missing:
        raise FlasherError('RP2040 boot ROM functions not found: %s' % ', '.join(missing))
    return funcs


def rp2040_write(f, offset, data):
    """offset: from the start of the flash; data: whole 4 k sectors."""
    f.cmd('SHALT')
    # core 1 off while the flash is out of XIP mode (it may run from flash); the watchdog reset of --run
    # or a power cycle brings it back
    f.cmd('SMW 0 %d %s' % (0x40010004, (1 << 16).to_bytes(4, 'little').hex()))    # PSM_FRCE_OFF.proc1
    f.cmd('SMW 0 %d %s' % (RP_BKPT, (0xBE00BE00).to_bytes(4, 'little').hex()))   # bkpt; bkpt
    fn = rp2040_rom_funcs(f)

    def call(name, *args, timeout=20):
        return int(f.cmd('SCALL %d %d %d %s' % (fn[name], RP_STACK, RP_BKPT, ' '.join(str(a) for a in args)),
                         timeout=timeout), 16)
    call('IF')
    call('EX')
    step = 0x8000                                       # 32 k through the RAM buffer at a time
    for pos in range(0, len(data), step):
        part = data[pos:pos + step]
        print('\rErase/write 0x%08x' % (0x10000000 + offset + pos), end='', flush=True)
        call('RE', offset + pos, len(part), RP_BLOCK, 0xD8)
        for k in range(0, len(part), SWD_WRITE_CHUNK):
            f.cmd('SMW 0 %d %s' % (RP_BUF + k, part[k:k + SWD_WRITE_CHUNK].hex()))
        call('RP', offset + pos, RP_BUF, len(part))
    call('FC')
    call('CX')
    print()


def rp2040_run(f):
    f.cmd('SMW 0 %d %s' % (DHCSR, (0xA05F0000).to_bytes(4, 'little').hex()))
    try:
        f.cmd('SMW 0 %d %s' % (0x40058000, (0x80000000).to_bytes(4, 'little').hex()))   # watchdog: reset now
    except FlasherError:
        pass                                            # the chip resets before it answers


def swd_write(f, t, addr, data, erase=True):
    if t.rp2040:
        offset = addr - 0x10000000 if addr >= 0x10000000 else addr
        # whole sectors: keep what is already there around the new data
        first = offset - offset % RP_SECTOR
        end = offset + len(data)
        end += -end % RP_SECTOR
        old = bytearray(swd_read(f, 0x10000000 + first, end - first, progress=False))
        old[offset - first:offset - first + len(data)] = data
        rp2040_write(f, first, bytes(old))
        return
    if t.ti:
        ti_write(f, t, addr, data)
        return
    if t.family is None:
        raise FlasherError('writing needs a recognised chip (EFR32 Series 1 / 2, nRF52)')
    if addr % 4 or len(data) % 4:
        data += b'\xff' * (-len(data) % 4)
        if addr % 4:
            raise FlasherError('the address must be a multiple of 4')
    if t.flash and addr + len(data) > t.flash:
        raise FlasherError('0x%x bytes at 0x%x do not fit into %d kbytes of flash' % (len(data), addr, t.flash // 1024))
    if t.family == 'stm32v2' and (addr % 8 or len(data) % 8):
        if addr % 8:
            raise FlasherError('STM32 G0/G4/L4/WB: the address must be a multiple of 8')
        data += b'\xff' * (-len(data) % 8)
    f.cmd('SFAM ' + t.family + t.sfam_args)
    f.cmd('SHALT reset')
    if erase:
        for a, n in t.erase_units(addr, addr + len(data)):
            print('\rErase 0x%08x' % a, end='', flush=True)
            f.cmd('SFE %d' % a, timeout=10)
        print()
    unit = min(SWD_WRITE_CHUNK, t.write_unit or SWD_WRITE_CHUNK)
    if t.family == 'stm32u5':
        if addr % 16:
            raise FlasherError('STM32 U5: the address must be a multiple of 16')
        data += b'\xff' * (-len(data) % 16)
    pos = 0
    while pos < len(data):
        a = addr + pos
        n = min(unit - a % unit, len(data) - pos)
        chunk = data[pos:pos + n]
        if chunk.count(0xff) != len(chunk):
            print('\rWrite 0x%08x' % a, end='', flush=True)
            f.cmd('SFW %d %s' % (a, chunk.hex()), timeout=20)
        pos += n
    print()


def swd_verify(f, addr, data):
    pos = 0
    while pos < len(data):
        n = min(CRC_CHUNK, len(data) - pos)
        print('\rVerify 0x%08x' % (addr + pos), end='', flush=True)
        crc = int(f.cmd('SFC %d %d' % (addr + pos, n), timeout=20), 16)
        if crc != zlib.crc32(data[pos:pos + n]):
            print()
            return addr + pos
        pos += n
    print('\r' + ' ' * 24 + '\r', end='')
    return None


def swd_read(f, addr, size, progress=True):
    out = bytearray()
    while len(out) < size:
        n = min(READ_CHUNK, size - len(out))
        if progress:
            print('\rRead 0x%08x' % (addr + len(out)), end='', flush=True)
        for attempt in range(3):
            try:
                answer = f.cmd('SM 0 %d %d' % (addr + len(out), n))
                data = b'\xff' * n if answer == '*' else base64.b64decode(answer, validate=True)
                if len(data) != n:
                    raise FlasherError('short answer')
                break
            except (FlasherError, ValueError):
                if attempt == 2:
                    raise
        out += data
    if progress:
        print('\r' + ' ' * 24 + '\r', end='')
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description='ESP32-C3 Telink TLSR825x SWire flasher')
    ap.add_argument('-p', '--port', default='/dev/ttyACM0')
    ap.add_argument('--host-baud', type=auto_int, default=460800, help='only for boards with a USB-UART chip')
    ap.add_argument('-t', '--tact', type=auto_int, default=100,
                    help='activation time in ms after the reset pulse (0 = no reset, default 100)')
    ap.add_argument('-m', '--mode', choices=['uart', 'rmt'], default='rmt',
                    help='how the SWS line is driven: rmt = pulse timing (default), uart = UART emulation')
    ap.add_argument('-b', '--baud', type=auto_int, default=0,
                    help='uart mode: UART baud, the SWire bit rate is a tenth of it (default 921600)')
    ap.add_argument('--rate', type=auto_int, default=0,
                    help='rmt mode: SWire bit rate in bit/s (default 1000000)')
    ap.add_argument('--burst', type=auto_int, default=0,
                    help='bytes read per burst, 1 = slow and safe (default 64)')
    ap.add_argument('-r', '--run', action='store_true', help='restart the target CPU at the end')
    ap.add_argument('--swd', action='store_true',
                    help='Arm SWD target (EFR32, nRF52, STM32): SWDIO on the SWS wire, SWCLK on GPIO 0; '
                         'EFR32 Series 1/2, nRF52, STM32 F0/F1/F3/F4/G0/G4/L4/WB, RP2040; info, rf, wf, es, ea')
    ap.add_argument('--swd-khz', type=auto_int, default=1000, help='SWD clock in kHz (default 1000)')
    ap.add_argument('--cjtag', action='store_true',
                    help='TI CC13xx / CC26xx over two-wire cJTAG (TMSC on the SWS wire, TCKC on GPIO 0) '
                         'instead of SWD: info, rf, wf, es, ea')
    ap.add_argument('--wake', action='store_true',
                    help='after activation switch the clocks on and wake the flash from deep power-down '
                         '(for a flash that reads all zeros / JEDEC 000000)')
    sub = ap.add_subparsers(dest='op', required=True)
    sub.add_parser('info', help='chip id, program counter, flash id')
    sub.add_parser('selftest', help='line test without talking to the target (TX - resistor - RX)')
    p = sub.add_parser('rf', help='read flash to a file')
    p.add_argument('address', type=auto_int)
    p.add_argument('size', type=auto_int)
    p.add_argument('filename')
    p.add_argument('--twice', action='store_true', help='read twice and compare')
    p = sub.add_parser('wf', help='write a file to flash, erasing the sectors, then verify')
    p.add_argument('address', type=auto_int)
    p.add_argument('filename')
    p.add_argument('--no-erase', action='store_true')
    p.add_argument('--no-verify', action='store_true')
    p = sub.add_parser('es', help='erase flash sectors')
    p.add_argument('address', type=auto_int)
    p.add_argument('size', type=auto_int)
    sub.add_parser('ea', help='erase the whole flash')
    p = sub.add_parser('bridge', help='make the flasher a UART bridge to the target (RX = SWS wire, TX = GPIO 21), '
                                      'then use another tool on the same port')
    p.add_argument('baud', type=auto_int)
    p.add_argument('--boot', choices=['hi', 'lo', 'z'], default='z',
                   help='level of the BOOT pin (GPIO 0) while the target is restarted (default: left open)')
    sub.add_parser('bridge-exit', help='leave the UART bridge')
    sub.add_parser('power-off', help='switch the target power pins off and make them inputs')
    p = sub.add_parser('led', help='drive every free pin low and then high for a moment, one after the other, '
                                   'to find the pin of an LED on an unknown board')
    p.add_argument('--skip', default='A1,D4,D7',
                   help='pins that must stay inputs, e.g. buttons and encoder lines (default: A1,D4,D7)')
    p.add_argument('--hold', type=float, default=1.0, help='seconds per level (default 1)')
    p.add_argument('--only', default='', help='test just these pins, e.g. C4 (repeats 3 times)')
    p = sub.add_parser('pins', help='watch the GPIO inputs of the stopped chip and print every change '
                                    '(to find buttons, encoders and switch inputs on an unknown board)')
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--pull', choices=['up', 'down', 'none'], default='up',
                   help='pull resistor on every pin while watching (default: 10 k pull-up)')
    args = ap.parse_args()

    t0 = time.time()
    if args.op == 'bridge-exit':
        # the flasher is in the bridge and does not answer commands: send the escape after a pause
        with serial.Serial(args.port, args.host_baud, timeout=2, dsrdtr=False, rtscts=False) as s:
            time.sleep(1.2)
            s.write(b'~~~X')
            answer = s.read_until(b'\n', 200)
        print(answer.decode(errors='replace').strip() or 'no answer (was the bridge open?)')
        return
    try:
        f = Flasher(args.port, args.host_baud)
        print('Flasher:', f.version)
        try:
            fw = tuple(int(x) for x in f.version.split()[1].split('.'))
        except (IndexError, ValueError):
            fw = (0, 0)
        if fw < (0, 11):
            raise FlasherError('this script needs flasher firmware 0.11 or newer on the C3')
        if args.cjtag:
            args.swd = True
            f.cjtag = True
            if fw < (0, 16):
                raise FlasherError('cJTAG needs flasher firmware 0.16 or newer on the C3')
        if args.op in ('bridge', 'power-off') or args.swd:
            if fw < (0, 15):
                raise FlasherError('this needs flasher firmware 0.15 or newer on the C3')
        if args.op == 'power-off':
            f.cmd('PWR 0')
            time.sleep(0.2)
            f.cmd('PWR z')
            print('Target power off, power pins are inputs.')
            return
        if args.op == 'bridge':
            print(f.cmd('BRIDGE %d %s' % (args.baud, args.boot)))
            f.ser.close()
            print('The port is free for the target tool now. Do not let it toggle DTR / RTS (they reset the C3);')
            print('for example esptool --before no_reset --after no_reset. Leave with: bridge-exit')
            return
        if args.swd:
            f.cmd('SWD')
            f.cmd('SWCLK %d' % args.swd_khz)
            t = None
            if args.op == 'info':
                t = swd_info(f)
            elif args.op == 'wf':
                with open(args.filename, 'rb') as fh:
                    data = fh.read()
                if not data:
                    raise FlasherError('empty file')
                data += b'\xff' * (-len(data) % 4)
                t = swd_info(f, quiet=True)
                if t.base and args.address < t.base:
                    args.address += t.base                 # STM32 0x08000000, RP2040 0x10000000
                print('Target:  %s' % (t.name or 'unknown'))
                print('Write 0x%08x..0x%08x' % (args.address, args.address + len(data)))
                swd_write(f, t, args.address, data, erase=not args.no_erase)
                if not args.no_verify:
                    bad = swd_verify(f, args.address, data)
                    if bad is not None:
                        raise FlasherError('verify failed in the block at 0x%08x' % bad)
                    print('Verified.')
            elif args.op in ('es', 'ea'):
                t = swd_info(f, quiet=True)
                if t.family is None:
                    raise FlasherError('erasing needs a recognised chip')
                if t.rp2040:
                    if args.op == 'ea':
                        raise FlasherError('RP2040: give the range with es (the flash size is not known)')
                    start = args.address - 0x10000000 if args.address >= 0x10000000 else args.address
                    start -= start % RP_SECTOR
                    end = args.address + max(args.size, 1) - (0x10000000 if args.address >= 0x10000000 else 0)
                    end += -end % RP_SECTOR
                    rp2040_write(f, start, b'\xff' * (end - start))
                elif t.ti:
                    # the last sector holds the CCFG (debug port enable): ea keeps it, es keeps its contents
                    if args.op == 'ea':
                        print('Erasing everything but the last sector (CCFG)...')
                        start, end = 0, t.flash - t.page
                    else:
                        start, end = args.address, args.address + max(args.size, 1)
                        start -= start % t.page
                        end += -end % t.page
                    last = None
                    if end > t.flash - t.page:
                        last = bytearray(swd_read(f, t.flash - t.page, t.page, progress=False))
                        keep = 0x58                 # the CCFG at the end of the sector stays
                        last[:-keep] = b'\xff' * (t.page - keep)
                    if start < min(end, t.flash - t.page):
                        ti_write(f, t, start, b'\xff' * (min(end, t.flash - t.page) - start))
                    if last is not None:
                        ti_write(f, t, t.flash - t.page, bytes(last))
                else:
                    f.cmd('SFAM ' + t.family + t.sfam_args)
                    f.cmd('SHALT reset')
                    if args.op == 'ea' and t.family in ('efr32s1', 'nrf52', 'nrf53', 'stm32f1', 'stm32f4', 'stm32v2',
                                                        'stm32u5', 'samd21', 'samd51'):
                        print('Mass erase %s...' % t.name)
                        f.cmd('SFX', timeout=60)
                    else:
                        addr = args.address + (t.base if args.address < t.base else 0) if args.op == 'es' else t.base
                        end = addr + max(args.size, 1) if args.op == 'es' else t.base + t.flash
                        for a, n in t.erase_units(addr, end):
                            print('\rErase 0x%08x' % a, end='', flush=True)
                            f.cmd('SFE %d' % a, timeout=10)
                        print()
            elif args.op == 'rf':
                os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
                t1 = time.time()
                swd_connect_any(f)
                data = swd_read(f, args.address, args.size)
                print('Read in %.1f s' % (time.time() - t1))
                if args.twice:
                    print('Second pass...')
                    again = swd_read(f, args.address, args.size)
                    if again != data:
                        bad = next(i for i in range(len(data)) if data[i] != again[i])
                        with open(args.filename + '.pass1', 'wb') as fh:
                            fh.write(data)
                        raise FlasherError('the second read differs at 0x%08x; the first read was kept as %s.pass1'
                                           % (args.address + bad, args.filename))
                    print('Both reads are identical.')
                with open(args.filename, 'wb') as fh:
                    fh.write(data)
                print('Saved %d bytes to %s' % (len(data), args.filename))
            else:
                raise FlasherError('over SWD: info, rf, wf, es, ea')
            if args.run:
                if t is not None and t.rp2040:
                    rp2040_run(f)
                elif t is not None and t.ti:
                    try:
                        f.cmd('SRUN')
                    except FlasherError:
                        pass                    # the system reset drops the cJTAG link before the answer
                else:
                    f.cmd('SRUN')
                print('Target restarted.')
            print('Done in %.1f s' % (time.time() - t0))
            return
        # always set the speeds: the flasher keeps the last values until it is powered off
        f.cmd('BAUD %d' % (args.baud or 921600))
        f.cmd('RATE %d' % (args.rate or 1000000))
        f.cmd('MODE ' + args.mode)
        # always set it: the flasher keeps the last value until it is powered off
        f.cmd('BURST %d' % (args.burst or 64))
        if args.op == 'selftest':
            print('Line:   ', f.cmd('SELFTEST'))
            print('Done in %.1f s' % (time.time() - t0))
            return
        print('Target: ', f.activate(args.tact))
        if args.wake:
            # A chip stopped before its boot code ran can have the flash interface clocks off, and a
            # firmware may have left the flash in deep power-down (0xB9), where it ignores everything
            # but 0xAB (release). Clocks on as the SDK start-up does, then 0xAB, then >= 30 us.
            f.cmd('S 0x60 000000ffffff')
            time.sleep(0.01)
            f.cmd('S 0x0d 00')                        # flash CS low
            f.cmd('S 0x0c ab')                        # release from deep power-down
            f.cmd('S 0x0d 01')                        # CS high
            time.sleep(0.01)
            print('Wake:    clocks on, flash 0xAB sent; JEDEC now %s' % f.cmd('FJ'))

        if args.op == 'info':
            # 0x7e: chip id (0x5562 = TLSR825x), 0x6bc: program counter
            cid = bytes.fromhex(f.cmd('G 0x7e 2'))
            pc = bytes.fromhex(f.cmd('G 0x6bc 4'))
            print('Chip id: 0x%04x' % int.from_bytes(cid, 'little'))
            print('PC:      0x%06x' % int.from_bytes(pc, 'little'))
            jedec = bytes.fromhex(f.cmd('FJ'))
            size = (1 << jedec[2]) >> 10 if 10 <= jedec[2] <= 24 else 0
            print('Flash:   JEDEC %s, %d kbytes, status 0x%s' % (jedec.hex(), size, f.cmd('FS')))
        elif args.op == 'rf':
            # create the folder first, so that a long read is not lost on a missing directory
            os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
            f.cmd('STAT')
            t1 = time.time()
            data = f.read_flash(args.address, args.size)
            print('Read in %.1f s (flasher: %s)' % (time.time() - t1, f.cmd('STAT')))
            t1 = time.time()
            if args.twice:
                print('Second pass...')
                # the flasher reads everything again and only sends checksums: no data over the slow USB link
                bad = f.compare_flash(args.address, data, 'Check')
                if bad is not None:
                    with open(args.filename + '.pass1', 'wb') as fh:
                        fh.write(data)
                    raise FlasherError('the second read differs in the block at 0x%06x; the first read was kept '
                                       'as %s.pass1, do not trust it' % (bad, args.filename))
                print('Both reads are identical. Check took %.1f s (flasher: %s)' % (time.time() - t1, f.cmd('STAT')))
            with open(args.filename, 'wb') as fh:
                fh.write(data)
            print('Saved %d bytes to %s' % (len(data), args.filename))
        elif args.op == 'wf':
            with open(args.filename, 'rb') as fh:
                data = fh.read()
            if not data:
                raise FlasherError('empty file')
            print('Write 0x%06x..0x%06x' % (args.address, args.address + len(data)))
            f.write_flash(args.address, data, erase=not args.no_erase)
            if not args.no_verify:
                print('Verify...')
                bad = f.compare_flash(args.address, data, 'Verify')
                if bad is not None:
                    raise FlasherError('verify failed in the block at 0x%06x' % bad)
                print('Verified.')
        elif args.op == 'es':
            f.cmd('FU')
            first = args.address - args.address % SECTOR
            last = args.address + max(args.size, 1) - 1
            for a in range(first, last + 1, SECTOR):
                print('\rErase 0x%06x' % a, end='', flush=True)
                f.cmd('FE %d' % a)
            print()
        elif args.op == 'led':
            find_led(f, args.skip, args.hold, args.only)
        elif args.op == 'pins':
            watch_pins(f, args.seconds, args.pull)
        elif args.op == 'ea':
            f.cmd('FU')
            print('Erase all...')
            f.cmd('FA', timeout=60)
        if args.run:
            f.cmd('RUN')
            print('Target restarted.')
    except (FlasherError, serial.SerialException) as e:
        print('\nError:', e)
        sys.exit(1)
    print('Done in %.1f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
