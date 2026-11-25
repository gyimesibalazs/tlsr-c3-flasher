#!/usr/bin/env python3
"""Host side of the ESP32-C3 Telink TLSR825x flasher.

Examples:
    python3 tlsr_c3.py -p /dev/ttyACM0 info
    python3 tlsr_c3.py -p /dev/ttyACM0 rf 0 0x100000 stock.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 wf 0 firmware.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 --rate 1500000 rf 0 0x80000 stock.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 --mode uart info
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd info                     # Arm SWD target (EFR32, nRF52, STM32, PY32)
    python3 tlsr_c3.py -p /dev/ttyACM0 --swd rf 0 0x40000 efr32.bin --twice
    python3 tlsr_c3.py -p /dev/ttyACM0 --wch info                     # WCH CH32V003 over SWIO, caught at power-up
    python3 tlsr_c3.py -p /dev/ttyACM0 --bk info                      # Beken BK7231N / T over its UART download port
    python3 tlsr_c3.py -p /dev/ttyACM0 --bk rf 0 0 backup.bin --twice # whole flash
    python3 tlsr_c3.py -p /dev/ttyACM0 --bk wf 0 firmware.uf2 --run   # LibreTiny / ESPHome UF2
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
        last = None
        for _ in range(8):
            # an empty line stops a stream an interrupted client left running (firmware 0.20) and is
            # otherwise ignored; then drop whatever that stream had still sent
            self.ser.write(b'\n')
            time.sleep(0.3)
            self.ser.reset_input_buffer()
            self.rxbuf = b''
            try:
                self.version = self.cmd('V', timeout=1)
                return
            except FlasherError as e:
                last = e
        raise FlasherError('the flasher does not answer on %s (%s); if it was left in the UART bridge, '
                           'run bridge-exit, otherwise unplug it and plug it in again' % (port, last))

    def stop(self):
        """After Ctrl-C: tell the flasher to stop a stream it may still be sending."""
        try:
            self.ser.write(b'\n')
            self.ser.flush()
        except serial.SerialException:
            pass

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

    # ---- binary streams (firmware 0.18): a whole range per command, 4 k frames with a CRC-32 each
    def _read_exact(self, n, timeout=10):
        deadline = time.time() + timeout
        while len(self.rxbuf) < n:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self.rxbuf += chunk
                deadline = time.time() + timeout
            elif time.time() >= deadline:
                raise FlasherError('the flasher stopped sending (%d of %d bytes)' % (len(self.rxbuf), n))
        out, self.rxbuf = self.rxbuf[:n], self.rxbuf[n:]
        return out

    def _stream_error(self):
        n = int.from_bytes(self._read_exact(2), 'little')
        return self._read_exact(n).decode(errors='replace')

    def stream_read(self, src, addr, size, label='Read', progress=True):
        """XR: src F = Telink flash, S = SWD memory, P = SPI flash."""
        self.cmd('XR %s %d %d' % (src, addr, size))
        out = bytearray()
        t0 = time.time()
        while True:
            kind = self._read_exact(1)
            if kind == b'Z':
                self._read_exact(2)                    # the end frame carries a length too (0)
                break
            if kind == b'E':
                raise FlasherError('read at 0x%08x: %s' % (addr + len(out), self._stream_error()))
            if kind != b'D':
                raise FlasherError('bad frame %r in the read stream' % kind)
            n = int.from_bytes(self._read_exact(2), 'little')
            data = self._read_exact(n)
            if zlib.crc32(data) != int.from_bytes(self._read_exact(4), 'little'):
                raise FlasherError('CRC error in the read stream at 0x%08x' % (addr + len(out)))
            out += data
            if progress:
                dt = max(time.time() - t0, 1e-3)
                print('\r%s 0x%08x  %4.0f kB/s' % (label, addr + len(out), len(out) / dt / 1000), end='', flush=True)
        if progress:
            print('\r' + ' ' * 40 + '\r', end='')
        if len(out) != size:
            raise FlasherError('short read stream: %d of %d bytes' % (len(out), size))
        self.last_rate = len(out) / max(time.time() - t0, 1e-3)
        return bytes(out)

    def stream_write(self, dst, addr, data, unit=256, skip=True, label='Write', progress=True, window=3):
        """XW: dst F = Telink flash, P = SPI flash, S = SWD flash, M = SWD memory. The flash must be erased.
        Frames end on 4 k address boundaries; up to `window` frames are on their way while one is written."""
        frames, pos = [], 0
        while pos < len(data):
            n = min(4096 - (addr + pos) % 4096, len(data) - pos)
            frames.append(data[pos:pos + n])
            pos += n
        self.cmd('XW %s %d %d %d %d' % (dst, addr, len(data), unit, 1 if skip else 0))
        sent = acked = done = 0
        t0 = time.time()
        while acked < len(frames):
            while sent < len(frames) and sent - acked < window:
                fr = frames[sent]
                self.ser.write(len(fr).to_bytes(2, 'little') + fr + zlib.crc32(fr).to_bytes(4, 'little'))
                sent += 1
            kind = self._read_exact(1, timeout=30)
            if kind == b'E':
                why = self._stream_error()
                raise FlasherError('write at 0x%08x: %s' % (addr + done, why))
            if kind != b'K':
                raise FlasherError('bad answer %r in the write stream' % kind)
            done += len(frames[acked])
            acked += 1
            if progress:
                dt = max(time.time() - t0, 1e-3)
                print('\r%s 0x%08x  %4.0f kB/s' % (label, addr + done, done / dt / 1000), end='', flush=True)
        if progress:
            print('\r' + ' ' * 40 + '\r', end='')
        self.last_rate = len(data) / max(time.time() - t0, 1e-3)

    def read_flash(self, addr, size, progress=True):
        return self.stream_read('F', addr, size, progress=progress)

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
        if erase:
            # as in pvvx's tool: a partly covered first sector is not erased, only written
            first = addr if addr % SECTOR == 0 else addr - addr % SECTOR + SECTOR
            for sector in range(first, addr + len(data), SECTOR):
                print('\rErase 0x%06x' % sector, end='', flush=True)
                self.cmd('FE %d' % sector)
            print('\r' + ' ' * 20 + '\r', end='')
        self.stream_write('F', addr, data, unit=PAGE, skip=True)


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
        # DEVINFO_TypeDef: INFO, PART, MEMINFO, MSIZE (efr32mg2x_devinfo.h)
        part_reg, meminfo, msize = swd_word(f, 0x0FE08004), swd_word(f, 0x0FE08008), swd_word(f, 0x0FE0800C)
        letters = {0: 'FG', 1: 'MG', 2: 'BG', 3: 'ZG', 5: 'PG', 8: 'SG'}
        famnum = (part_reg >> 16) & 0x3f
        if famnum in EFR32S2_MSC_CLOCK:
            letter = letters.get((part_reg >> 24) & 0x3f, '?%d' % ((part_reg >> 24) & 0x3f))
            t = SwdTarget('EFR32%s%d (device %d)' % (letter, famnum, part_reg & 0xffff), (msize & 0xffff) * 1024,
                          1 << (((meminfo & 0xff) + 10) & 0xff), 'efr32s2', sfam_args=' %d' % EFR32S2_MSC_CLOCK[famnum],
                          base=0x08000000 if famnum >= 23 else 0)   # FLASH_BASE: xG21 / xG22 at 0
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


# Puya PY32F002B type, also the MS32C001 (a PY32F002B-compatible part): DBGMCU IDCODE at 0x40015800 (the
# 002B reads 0x20220064, the other PY32F0 parts 0x60001000, which have another flash controller); recognised from
# the flash controller at 0x40022000 and the flash timing in the factory configuration bytes at 0x1FFF011C.
PUYA_DEV = {0x064: 'PY32F002B'}
PY32_FLASH, PY32_PAGE = 0x08000000, 128


def puya_info(f, say, part):
    if part != 0xC60:                                   # Cortex-M0+
        return None
    idcode = swd_word(f, 0x40015800)
    if idcode & 0xfff in STM32 or idcode == 0x60001000:
        return None
    timing = [swd_word(f, 0x1FFF011C + 4 * i) for i in range(5)]
    if timing[0] in (0, 0xffffffff) or timing[4] in (0, 0xffffffff):
        return None
    swd_word(f, 0x40022100)                             # FLASH TS0: only the Puya controller has it
    cr = swd_word(f, 0x40022014)
    # flash size: the first 1 k step that is not readable (the reserved space answers with a bus fault)
    kb = 0
    for k in range(1, 65):
        try:
            swd_word(f, PY32_FLASH + 1024 * k - 4)
        except FlasherError:
            break
        kb = k
    if kb == 64:                                        # no fault at all: size not known
        kb = 0
    name = PUYA_DEV.get(idcode & 0xfff)
    label = ('Puya %s (DBGMCU IDCODE 0x%08x)' % (name, idcode) if name else
             'Puya PY32F002B type, e.g. MS32C001 (DBGMCU IDCODE 0x%08x not in the list)' % idcode)
    t = SwdTarget(label, kb * 1024 or None, PY32_PAGE, 'py32', PY32_FLASH, write_unit=PY32_PAGE,
                  erase_unit=PY32_PAGE)
    say('Chip:    %s, flash %s in %d-byte pages%s' % (
        t.name, '%d kbytes' % kb if kb else 'size unknown', PY32_PAGE, '' if cr & (1 << 31) else ' (flash unlocked)'))
    say('         option bytes 0x%08x (not changed by this tool)' % swd_word(f, 0x1FFF0080))
    return t


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
    if dev == 0x410 and v >> 16 == 0x1909:
        name, writer = 'GD32E23x (word programming, not handled)', None
    elif dev == 0x410 and v >> 16 in (0x1303, 0x1704):
        name = 'GD32F1x0' if v >> 16 == 0x1303 else 'GD32F3x0'
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


# ---- four-wire JTAG: the chain is scanned, the Arm DAP found in it, then the SWD code above runs through it
JEP106 = {0x23B: 'Arm', 0x020: 'STMicroelectronics', 0x017: 'Texas Instruments', 0x01F: 'Atmel / Microchip',
          0x015: 'NXP', 0x00E: 'Freescale / NXP', 0x049: 'Xilinx', 0x06E: 'Altera / Intel', 0x0C8: 'GigaDevice',
          0x029: 'Microchip', 0x093: 'Silicon Labs', 0x244: 'Nordic', 0x51E: 'Espressif', 0x489: 'SiFive',
          0x612: 'Andes', 0x1F0: 'Lattice', 0x0BF: 'Broadcom', 0x31E: 'Cypress', 0x03A: 'Intel'}


def jtag_ir_len(idcode):
    """Instruction length of the devices whose length is known from the IDCODE, else None."""
    if idcode & 0xfff == 0x477 and (idcode >> 12) & 0xff00 == 0xba00:
        return 4                                      # Arm JTAG-DP (Cortex-M / -A debug port)
    if idcode & 0xfff == 0x041:
        return 5                                      # ST boundary-scan TAP of the STM32
    return None


def jtag_chain(f, say):
    """Scan the chain and select its Arm DAP. Returns the device list (TDO side first) and the DAP index."""
    answer = f.cmd('JS').split()
    ir_total, ids = int(answer[0]), [int(x, 16) for x in answer[1:]]
    say('Chain:   %d device(s), %d instruction bits in all (TDO side first)' % (len(ids), ir_total))
    lens = [jtag_ir_len(i) if i else None for i in ids]
    unknown = [k for k, n in enumerate(lens) if n is None]
    if len(unknown) == 1:
        lens[unknown[0]] = ir_total - sum(n for n in lens if n)
    for k, i in enumerate(ids):
        if i:
            say('  %d:     0x%08x  %s, part 0x%04x, version %d, IR %s' % (
                k, i, JEP106.get((i >> 1) & 0x7ff, 'maker 0x%03x' % ((i >> 1) & 0x7ff)), (i >> 12) & 0xffff,
                i >> 28, lens[k] if lens[k] else '?'))
        else:
            say('  %d:     no IDCODE (BYPASS only), IR %s' % (k, lens[k] if lens[k] else '?'))
    daps = [k for k, i in enumerate(ids) if i and jtag_ir_len(i) == 4]
    if not daps:
        return ids, None
    if None in lens or sum(lens) != ir_total:
        raise FlasherError('the instruction lengths of the other devices in the chain are not known '
                           '(%d bits in all): cannot reach the Arm DAP' % ir_total)
    k = daps[0]
    f.cmd('JSEL %d %d %d %d' % (sum(lens[:k]), sum(lens[k + 1:]), k, len(ids) - k - 1))
    return ids, k


def swd_connect_any(f):
    """Plain SWD first, then the RP2040's multi-drop port. Returns (idcode, multidrop).
    With f.cjtag: cJTAG through the TI ICEPick router instead, idcode is then the JTAG-DP's."""
    if getattr(f, 'cjtag', False):
        icepick, dap = (int(x, 16) for x in f.cmd('TC').split())
        f.icepick = icepick
        return dap, False
    if getattr(f, 'jtag', False):
        return int(f.cmd('JC'), 16), False
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
    for probe in (efr32_info, nrf_info, stm32_info, puya_info, samd_info):
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
            f.stream_write('M', TI_BUF, part, unit=4096, skip=False, progress=False)
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
        f.stream_write('M', RP_BUF, part, unit=4096, skip=False, progress=False)
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
        raise FlasherError('writing needs a recognised chip (see the README, Arm SWD)')
    if addr % 4 or len(data) % 4:
        data += b'\xff' * (-len(data) % 4)
        if addr % 4:
            raise FlasherError('the address must be a multiple of 4')
    if t.flash and addr - t.base + len(data) > t.flash:      # addresses are absolute (base: STM32, PY32)
        raise FlasherError('0x%x bytes at 0x%x do not fit into %d kbytes of flash' % (len(data), addr, t.flash // 1024))
    if t.family == 'stm32f1' and addr + len(data) > STM32_FLASH + 0x80000:
        raise FlasherError('above 512 k: GD32F10x / F30x bank 1 has its own registers, not handled yet')
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
    unit = min(SWD_WRITE_CHUNK, t.write_unit or SWD_WRITE_CHUNK)   # one flash call never crosses this
    if t.family == 'stm32u5':
        if addr % 16:
            raise FlasherError('STM32 U5: the address must be a multiple of 16')
        data += b'\xff' * (-len(data) % 16)
    # STM32 L0 / L1 flash reads 0x00 when erased: nothing may be skipped there
    f.stream_write('S', addr, data, unit=unit, skip=t.family != 'stm32l0')


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
    return f.stream_read('S', addr, size, progress=progress)


# ---- SPI NOR flash (25-series) on the flasher's wires: CS = GPIO 20, CLK = SWCLK, DI = SWS, DO = GPIO 21,
#      the reset wire holds the board's processor in reset
SPI_VENDORS = {0xEF: 'Winbond', 0xC8: 'GigaDevice', 0xC2: 'Macronix', 0x20: 'Micron / XMC', 0x1C: 'EON',
               0xBF: 'SST / Microchip', 0x9D: 'ISSI', 0x68: 'Boya', 0x5E: 'Zbit', 0x85: 'Puya', 0x0B: 'XTX',
               0xA1: 'Fudan', 0x01: 'Spansion / Cypress / Infineon', 0x1F: 'Adesto / Atmel / Renesas', 0xC4: 'GT',
               0xCD: 'Paragon', 0x8C: 'ESMT', 0x4A: 'ESMT', 0xD8: 'Silicon Kaiser', 0xBA: 'Zetta'}


class SpiFlash:
    def __init__(self, jedec, size, erase=(0x1000, 0x20), sfdp=False):
        self.jedec, self.size, self.sfdp = jedec, size, sfdp
        self.erase_size, self.erase_op = erase


def spi_tx(f, out, nin=0):
    answer = f.cmd('PT %s %d' % (bytes(out).hex(), nin))
    return bytes.fromhex(answer) if nin else b''


SPI_SR23_VENDORS = (0xEF, 0xC8)   # Winbond, GigaDevice: 0x35 / 0x15 read status 2 / 3


def spi_status(f, t):
    """Status registers 1..3; 2 and 3 only where 0x35 / 0x15 read them (0xff otherwise): on Macronix and ISSI
    0x35 enters QPI mode, after which the chip no longer answers on one wire."""
    sr1 = spi_tx(f, [0x05], 1)[0]
    if t.jedec[0] not in SPI_SR23_VENDORS:
        return sr1, 0xff, 0xff
    return sr1, spi_tx(f, [0x35], 1)[0], spi_tx(f, [0x15], 1)[0]


def spi_sfdp(f):
    """(size in bytes, (erase size, opcode)) from the SFDP basic table, or None."""
    head = spi_tx(f, [0x5A, 0, 0, 0, 0], 16)
    if head[:4] != b'SFDP':
        return None
    dwords, ptr = head[8 + 3], int.from_bytes(head[8 + 4:8 + 7], 'little')
    if head[8] != 0x00 or head[8 + 7] != 0xFF or dwords < 9:
        return None
    t = spi_tx(f, [0x5A, ptr >> 16 & 0xff, ptr >> 8 & 0xff, ptr & 0xff, 0], min(dwords, 16) * 4)
    dw = [int.from_bytes(t[i:i + 4], 'little') for i in range(0, len(t), 4)]
    density = dw[1]
    bits = (1 << (density & 0x7fffffff)) if density & 0x80000000 else density + 1
    types = [((dw[7] >> 16 * i) & 0xff, (dw[7] >> (16 * i + 8)) & 0xff) for i in range(2)]
    types += [((dw[8] >> 16 * i) & 0xff, (dw[8] >> (16 * i + 8)) & 0xff) for i in range(2)]
    types = sorted((1 << n, op) for n, op in types if n and op in (0x20, 0x52, 0xD8))
    erase = types[0] if types else ((0x1000, 0x20) if dw[0] & 3 == 1 else (0x10000, 0xD8))
    return bits // 8, erase


def spi_info(f, quiet=False):
    say = (lambda *a: None) if quiet else print
    jedec = bytes.fromhex(f.cmd('PJ'))
    say('JEDEC:   %s (%s)' % (jedec.hex(), SPI_VENDORS.get(jedec[0], 'unknown maker')))
    if jedec in (b'\x00\x00\x00', b'\xff\xff\xff'):
        raise FlasherError('no answer from the flash: check the wires, its power, WP / HOLD (both high), and that the '
                           'board\'s own processor is held in reset (it drives the same pins)')
    sf = spi_sfdp(f)
    if sf:
        size, erase = sf
    else:
        if not 0x10 <= jedec[2] <= 0x22:
            raise FlasherError('no SFDP, and the size byte 0x%02x of the id does not give a size' % jedec[2])
        size, erase = 1 << jedec[2], (0x1000, 0x20)
    t = SpiFlash(jedec, size, erase, bool(sf))
    f.cmd('PA %d' % (4 if size > 0x1000000 else 3))
    say('Size:    %d kbytes (%s), erased in %d k units (opcode 0x%02X)%s' % (
        size // 1024, 'SFDP' if sf else 'from the id', erase[0] // 1024, erase[1],
        ', 4-byte addresses' if size > 0x1000000 else ''))
    sr = spi_status(f, t)
    say('Status:  SR1 %02x' % sr[0] + ('  SR2 %02x  SR3 %02x' % sr[1:] if t.jedec[0] in SPI_SR23_VENDORS else ''))
    if spi_protected(t, sr):
        say('         block protection is set: writing needs --unprotect')
    return t


def spi_protected(t, sr):
    sr1, sr2, _ = sr
    if sr1 & 0x3C:                                  # BP0..BP3
        return True
    return t.jedec[0] in (0xEF, 0xC8) and sr1 & 0x40   # Winbond SEC / GigaDevice BP4


def spi_unprotect(f, t):
    sr1, sr2, _ = spi_status(f, t)
    if not spi_protected(t, (sr1, sr2, 0)):
        return
    clear = 0x3C | (0x40 if t.jedec[0] in (0xEF, 0xC8) else 0)
    new1 = sr1 & ~clear & 0xff
    spi_tx(f, [0x06])
    if t.jedec[0] in (0xEF, 0xC8) and sr2 != 0xff:
        # two-byte write keeps status register 2 (QE must stay as it is); Winbond CMP (SR2 bit 6) is cleared too,
        # because with CMP set, BP = 0 protects the whole chip
        spi_tx(f, [0x01, new1, sr2 & ~0x40 & 0xff])
    else:
        spi_tx(f, [0x01, new1])
    for _ in range(100):
        if not int(f.cmd('PS'), 16) & 1:
            break
        time.sleep(0.01)
    sr = spi_status(f, t)
    if spi_protected(t, sr):
        raise FlasherError('the protection bits did not clear (SR1 %02x): the WP pin is probably low with SRP set' % sr[0])
    print('Block protection cleared (SR1 %02x -> %02x).' % (sr1, sr[0]))


def spi_read(f, addr, size, progress=True):
    return f.stream_read('P', addr, size, progress=progress)


def spi_crc(f, addr, size):
    return int(f.cmd('PC %d %d' % (addr, size), timeout=10 + size / 200000), 16)


def spi_write(f, t, addr, data, erase=True):
    """Erase unit by erase unit: units whose CRC already matches are skipped, what the image does not cover
    inside a unit is read first and kept; each run of changed units is erased, then streamed in one go."""
    if addr + len(data) > t.size:
        raise FlasherError('0x%x bytes at 0x%x do not fit into %d kbytes' % (len(data), addr, t.size // 1024))
    u = t.erase_size
    first = addr - addr % u
    end = addr + len(data)
    end += -end % u
    runs, skipped = [], 0                 # [start, bytes] of consecutive units that need writing
    for a in range(first, end, u):
        print('\rCompare 0x%08x' % a, end='', flush=True)
        lo, hi = max(a, addr), min(a + u, addr + len(data))
        part = data[lo - addr:hi - addr]
        if lo > a or hi < a + u:
            want = bytearray(spi_read(f, a, u, progress=False))
            want[lo - a:hi - a] = part
            want = bytes(want)
        else:
            want = part
        if spi_crc(f, a, u) == zlib.crc32(want):
            skipped += 1
        elif runs and runs[-1][0] + len(runs[-1][1]) == a:
            runs[-1][1] += want
        else:
            runs.append([a, bytearray(want)])
    print('\r' + ' ' * 28 + '\r', end='')
    for start, buf in runs:
        if erase:
            for a in range(start, start + len(buf), u):
                print('\rErase 0x%08x' % a, end='', flush=True)
                f.cmd('PE %d %d' % (a, t.erase_op), timeout=10)
            print('\r' + ' ' * 28 + '\r', end='')
        f.stream_write('P', start, bytes(buf))
    if skipped:
        print('%d of %d erase units already held the data.' % (skipped, (end - first) // u))

def spi_verify(f, addr, data):
    step = 0x10000
    for pos in range(0, len(data), step):
        n = min(step, len(data) - pos)
        print('\rVerify 0x%08x' % (addr + pos), end='', flush=True)
        if spi_crc(f, addr + pos, n) != zlib.crc32(data[pos:pos + n]):
            print()
            return addr + pos
    print('\r' + ' ' * 24 + '\r', end='')
    return None


def spi_main(f, args):
    """SPI mode: the board's processor is held in reset meanwhile; with --power the flasher also powers the
    board (after the reset is down) and switches it off at the end."""
    print('SPI:     %s kHz, target held in reset' % f.cmd('SPI %d' % args.spi_khz))
    try:
        if args.power:
            f.cmd('PWR 1')
            time.sleep(0.1)                   # the flash needs a moment after power-up
            print('Power:   on')
        spi_ops(f, args)
    finally:
        # also after Ctrl-C or an error: stop a stream that may still run, then power off and release
        f.stop()
        time.sleep(0.3)
        f.ser.reset_input_buffer()
        f.rxbuf = b''
        try:
            if args.power:
                f.cmd('PWR 0')
                time.sleep(0.2)
                f.cmd('PWR z')
            f.cmd('SPI 0')
        except FlasherError:
            pass


def spi_ops(f, args):
    t = spi_info(f, quiet=args.op != 'info')
    if args.op == 'info':
        return
    if args.op == 'rf':
        size = args.size or t.size - args.address
        os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
        t1 = time.time()
        data = spi_read(f, args.address, size)
        print('Read in %.1f s (%.0f kB/s)' % (time.time() - t1, f.last_rate / 1000))
        if args.twice:
            bad = spi_verify(f, args.address, data)
            if bad is not None:
                with open(args.filename + '.pass1', 'wb') as fh:
                    fh.write(data)
                raise FlasherError('the flash reads differently the second time in the block at 0x%08x; '
                                   'the read was kept as %s.pass1' % (bad, args.filename))
            print('Second pass (CRC on the flasher) matches.')
        with open(args.filename, 'wb') as fh:
            fh.write(data)
        print('Saved %d bytes to %s' % (len(data), args.filename))
        return
    if args.op not in ('wf', 'es', 'ea'):
        raise FlasherError('SPI flash: info, rf, wf, es, ea')
    if spi_protected(t, spi_status(f, t)):
        if not args.unprotect:
            raise FlasherError('the flash has block protection set: add --unprotect to clear it first')
        spi_unprotect(f, t)
    if args.op == 'wf':
        with open(args.filename, 'rb') as fh:
            data = fh.read()
        if not data:
            raise FlasherError('empty file')
        print('Write 0x%08x..0x%08x' % (args.address, args.address + len(data)))
        spi_write(f, t, args.address, data, erase=not args.no_erase)
        if not args.no_verify:
            bad = spi_verify(f, args.address, data)
            if bad is not None:
                raise FlasherError('verify failed in the block at 0x%08x' % bad)
            print('Verified.')
    elif args.op == 'es':
        u = t.erase_size
        start = args.address - args.address % u
        end = args.address + max(args.size, 1)
        end += -end % u
        blank = zlib.crc32(b'\xff' * u)
        for a in range(start, min(end, t.size), u):
            if spi_crc(f, a, u) != blank:
                print('\rErase 0x%08x' % a, end='', flush=True)
                f.cmd('PE %d %d' % (a, t.erase_op), timeout=10)
        print()
    else:
        print('Chip erase (up to a few minutes on big chips)...')
        f.cmd('PX', timeout=420)


# ---- WCH CH32V003 over its single-wire debug interface (SDI, the SWIO pin): SWIO on the SWS wire, target power
#      on the power pins so that it can be caught at power-up. The flash routines run on the flasher (W commands).
WCH_FLASH, WCH_BOOT, WCH_BOOT_SIZE, WCH_OPT, WCH_RAM, WCH_PAGE = 0x08000000, 0x1FFFF000, 1920, 0x1FFFF800, 0x20000000, 64
WCH_FACTORY_OPTIONS = bytes([0xa5, 0x5a, 0x97, 0x68, 0x00, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00])
WCH_CPBR = 0x00010403          # what WCH's own host expects once the debug output is on


class WchTarget:
    def __init__(self, flash=16384, rdprt=False, wpr=0xffffffff, options=WCH_FACTORY_OPTIONS, name=''):
        self.flash, self.rdprt, self.wpr, self.options, self.name = flash, rdprt, wpr, options, name


def wch_mem(f, addr, size):
    """Any range of target memory (word reads on the flasher)."""
    start = addr - addr % 4
    end = addr + size
    end += -end % 4
    out = bytearray()
    for a in range(start, end, 1024):
        out += bytes.fromhex(f.cmd('WM %d %d' % (a, min(1024, end - a))))
    return bytes(out[addr - start:addr - start + size])


def wch_word(f, addr):
    return int.from_bytes(wch_mem(f, addr, 4), 'little')


def wch_options_text(ob):
    """The 16 option bytes as text: each byte is followed by its complement."""
    def pair(i):
        b, nb = ob[2 * i], ob[2 * i + 1]
        return '%02x/%02x%s' % (b, nb, '' if nb == b ^ 0xff else ' (complement wrong)')
    user = ob[2]
    lines = ['RDPR  %s  %s' % (pair(0), 'read protection off' if ob[0] == 0xa5 else 'READ PROTECTED'),
             'USER  %s  IWDG_SW %d, STANDBY_RST %d, RST_MODE %d, START_MODE %d' % (
                 pair(1), user & 1, user >> 2 & 1, user >> 3 & 3, user >> 5 & 1),
             'DATA  %s  %s' % (pair(2), pair(3)),
             'WRPR  %s  %s  %s  %s%s' % (pair(4), pair(5), pair(6), pair(7),
                                         '' if all(ob[8 + 2 * i] == 0xff for i in range(4)) else
                                         '  (some pages write protected)')]
    return lines


def wch_attach(f, args, say=print):
    off = 0 if args.external_power else args.wch_off
    try:
        answer = f.cmd('WA %d %d' % (off, args.wch_window), timeout=(off + args.wch_window) / 1000 + 10)
    except FlasherError as e:
        hint = ('\n  With external power the flasher cannot restart the target. If its firmware uses the SWIO pin '
                '(pin 8 of the SOP8 is also UART TX), power it from the power pins and leave out --external-power.'
                if args.external_power else
                '\n  Check: GND to pin 2, the power pins to VDD (pin 4), SWIO (pin 8) on GPIO 5 and through 1 k on '
                'GPIO 4; a longer --wch-off if the board has big capacitors; --wch-window for a slow start.')
        raise FlasherError(str(e) + hint)
    dmstatus, cpbr, tries, first_us, powered = answer.split()
    tries, first_us = int(tries), int(first_us)
    if powered == '1':
        say('Attach:  core halted; first answer %.2f ms after power-on (try %d), dmstatus %s' % (
            first_us / 1000, tries, dmstatus))
    else:
        say('Attach:  core halted at try %d (external power: no catch at power-up), dmstatus %s' % (tries, dmstatus))
    if int(cpbr, 16) != WCH_CPBR:
        say('         CPBR 0x%s, WCH\'s reference host expects 0x%08x' % (cpbr, WCH_CPBR))


def wch_info(f, say=print):
    dmi_id = int(f.cmd('WR 0x7f'), 16)
    chipid = wch_word(f, 0x1FFFF7C4)
    flacap = wch_word(f, 0x1FFFF7E0) & 0xffff
    uid = [wch_word(f, a) for a in (0x1FFFF7E8, 0x1FFFF7EC, 0x1FFFF7F0)]
    obr = wch_word(f, 0x4002201C)
    wpr = wch_word(f, 0x40022020)
    ob = wch_mem(f, WCH_OPT, 16)
    v003 = (dmi_id & 0xfff00f00) == 0x00300500 or (chipid & 0xfff00f00) == 0x00300500
    name = 'CH32V003' if v003 else 'not a CH32V003 (only the V003 is handled)'
    flash = flacap * 1024 if 0 < flacap <= 256 else 16384
    say('Chip:    %s, CHIPID 0x%08x (debug register 0x7f: 0x%08x), flash %d kbytes' % (name, chipid, dmi_id,
                                                                                         flash // 1024))
    say('UID:     %08x %08x %08x' % tuple(uid))
    t = WchTarget(flash, bool(obr & 2), wpr, ob, name)
    say('Flash:   OBR 0x%08x (%s%s), WPR 0x%08x%s' % (
        obr, 'READ PROTECTED' if t.rdprt else 'not read protected', ', option byte error' if obr & 1 else '',
        wpr, '' if wpr == 0xffffffff else ' (write protected pages)'))
    for line in wch_options_text(ob):
        say('Options: ' + line)
    if not v003:
        raise FlasherError('this chip id is not a CH32V003; refusing to go on')
    return t


def wch_options_dump(f, t, args):
    """Shown before anything is written: the option bytes as they are, and where to keep them."""
    print('Options: %s (RDPR USER DATA0 DATA1 WRPR0..3, each with its complement)' % t.options.hex())
    if args.save_options:
        with open(args.save_options, 'wb') as fh:
            fh.write(t.options)
        print('         saved to %s' % args.save_options)
    else:
        print('         (--save-options FILE keeps a copy; wf 0x1FFFF800 FILE --write-options puts them back)')


def wch_area(t, addr, size):
    """(name, start, end) of the memory area that holds addr..addr+size-1, or None."""
    for name, start, n in (('flash', WCH_FLASH, t.flash), ('boot', WCH_BOOT, WCH_BOOT_SIZE), ('options', WCH_OPT, 64),
                           ('ram', WCH_RAM, 2048)):
        if start <= addr and addr + size <= start + n:
            return name, start, start + n
    return None


def wch_addr(addr):
    return addr + WCH_FLASH if addr < 0x01000000 else addr      # 0x00000000 is an alias of the user flash


def wch_read(f, addr, size, progress=True):
    start = addr - addr % 4
    end = addr + size
    end += -end % 4
    data = f.stream_read('C', start, end - start, progress=progress)
    return data[addr - start:addr - start + size]


def wch_write_flash(f, t, addr, data):
    """Page by page (64 bytes): what the image does not cover inside a page is read first and kept, pages that
    already hold the data are left alone, the others are erased and written in runs."""
    first = addr - addr % WCH_PAGE
    end = addr + len(data)
    end += -end % WCH_PAGE
    old = bytearray(wch_read(f, first, end - first, progress=False))
    want = bytearray(old)
    want[addr - first:addr - first + len(data)] = data
    runs, same = [], 0
    for a in range(first, end, WCH_PAGE):
        page = bytes(want[a - first:a - first + WCH_PAGE])
        if page == old[a - first:a - first + WCH_PAGE]:
            same += 1
        elif runs and runs[-1][0] + len(runs[-1][1]) == a:
            runs[-1][1] += page
        else:
            runs.append([a, bytearray(page)])
    for start, buf in runs:
        for a in range(start, start + len(buf), WCH_PAGE):
            print('\rErase 0x%08x' % a, end='', flush=True)
            f.cmd('WFE %d' % a, timeout=5)
        print('\r' + ' ' * 28 + '\r', end='')
        f.stream_write('C', start, bytes(buf), unit=WCH_PAGE, skip=True)
    if same:
        print('%d of %d pages already held the data.' % (same, (end - first) // WCH_PAGE))


def wch_verify(f, addr, data):
    start = addr - addr % 4
    pre = wch_mem(f, start, addr - start) if addr > start else b''
    body = pre + data
    body_end = start + len(body)
    tail = -body_end % 4
    if tail:
        body += wch_mem(f, body_end, tail)
    for pos in range(0, len(body), 4096):
        n = min(4096, len(body) - pos)
        print('\rVerify 0x%08x' % (start + pos), end='', flush=True)
        if int(f.cmd('WFC %d %d' % (start + pos, n), timeout=20), 16) != zlib.crc32(body[pos:pos + n]):
            print()
            return start + pos
    print('\r' + ' ' * 24 + '\r', end='')
    return None


def wch_options_check(ob):
    for i in range(8):
        if ob[2 * i + 1] != ob[2 * i] ^ 0xff:
            raise FlasherError('option byte %d: 0x%02x is not the complement of 0x%02x' % (i, ob[2 * i + 1], ob[2 * i]))


def wch_unlock(f, t, args):
    """Remove the read protection: the option bytes get RDPR = 0xA5 (and no write protection); the chip erases
    its whole user flash when this happens."""
    if not t.rdprt and t.wpr == 0xffffffff:
        print('Neither read nor write protected: nothing to unlock.')
        return
    if not args.yes_erase_all:
        raise FlasherError('unlock removes the read protection, and the chip then ERASES ITS WHOLE USER FLASH; '
                           'nothing of the firmware can be read before. Add --yes-erase-all to go on.')
    new = bytearray(t.options)
    new[0:2] = b'\xa5\x5a'
    for i in (1, 2, 3):                               # USER, DATA0, DATA1 stay unless their complement is broken
        if new[2 * i + 1] != new[2 * i] ^ 0xff:
            new[2 * i:2 * i + 2] = WCH_FACTORY_OPTIONS[2 * i:2 * i + 2]
    new[8:16] = WCH_FACTORY_OPTIONS[8:16]             # no write protection
    print('Options: %s -> %s' % (t.options.hex(), bytes(new).hex()))
    f.cmd('WH reset', timeout=5)
    f.cmd('WOB ' + bytes(new).hex(), timeout=20)
    # the option bytes are loaded at reset: a power cycle when the flasher powers the target
    print('Option bytes written; restarting the target to load them...')
    wch_attach(f, args)
    f.cmd('WH reset', timeout=5)
    t2 = wch_info(f, say=lambda *a: None)
    if t2.rdprt:
        raise FlasherError('still read protected after the restart; power the target off and on, then run info')
    blank = zlib.crc32(b'\xff' * t2.flash)
    if int(f.cmd('WFC %d %d' % (WCH_FLASH, t2.flash), timeout=30), 16) != blank:
        print('The user flash is not blank after the unlock; erasing it.')
        f.cmd('WFX', timeout=10)
    print('Read protection removed, user flash erased.')


def wch_main(f, args):
    print('WCH:     SWIO timing (ns) one/zero/high/slot/threshold/stop %s' % f.cmd('WCH').replace(' ', '/'))
    if args.wch_timing:
        print('         set to %s' % f.cmd('WT ' + ' '.join(args.wch_timing.split(','))).replace(' ', '/'))
    wch_attach(f, args)
    t = wch_info(f, say=print if args.op in ('info', 'unlock') else (lambda *a: None))
    if args.op == 'info':
        pass
    elif args.op == 'rf':
        addr = wch_addr(args.address)
        area = wch_area(t, addr, args.size)
        if area and area[0] == 'flash' and t.rdprt:
            raise FlasherError('the flash is read protected: the debug port does not return its contents '
                               '(unlock removes the protection and erases the flash)')
        os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
        t1 = time.time()
        data = wch_read(f, addr, args.size)
        print('Read in %.1f s (%.0f kB/s)' % (time.time() - t1, len(data) / max(time.time() - t1, 1e-3) / 1000))
        if args.twice:
            bad = wch_verify(f, addr, data)
            if bad is not None:
                with open(args.filename + '.pass1', 'wb') as fh:
                    fh.write(data)
                raise FlasherError('the second pass differs in the block at 0x%08x; the first read was kept as '
                                   '%s.pass1' % (bad, args.filename))
            print('Second pass (CRC on the flasher) matches.')
        with open(args.filename, 'wb') as fh:
            fh.write(data)
        print('Saved %d bytes to %s' % (len(data), args.filename))
    elif args.op in ('wf', 'es', 'ea', 'unlock'):
        wch_options_dump(f, t, args)
        if args.op == 'unlock':
            wch_unlock(f, t, args)
        else:
            if t.rdprt:
                raise FlasherError('the chip is read protected: the flash cannot be erased or written page by page. '
                                   '"unlock --yes-erase-all" removes the protection and erases everything')
            f.cmd('WH reset', timeout=5)                # stopped at the reset vector, peripherals reset
            if args.op == 'ea':
                print('Mass erase of the user flash...')
                f.cmd('WFX', timeout=10)
            elif args.op == 'es':
                addr = wch_addr(args.address)
                area = wch_area(t, addr, max(args.size, 1))
                if not area or area[0] not in ('flash', 'boot'):
                    raise FlasherError('es: user flash 0x%08x..0x%08x or boot area 0x%08x..0x%08x only' % (
                        WCH_FLASH, WCH_FLASH + t.flash, WCH_BOOT, WCH_BOOT + WCH_BOOT_SIZE))
                start = addr - addr % WCH_PAGE
                for a in range(start, addr + max(args.size, 1), WCH_PAGE):
                    print('\rErase 0x%08x' % a, end='', flush=True)
                    f.cmd('WFE %d' % a, timeout=5)
                print()
            else:
                with open(args.filename, 'rb') as fh:
                    data = fh.read()
                if not data:
                    raise FlasherError('empty file')
                addr = wch_addr(args.address)
                area = wch_area(t, addr, len(data))
                if area is None:
                    raise FlasherError('0x%x bytes at 0x%08x do not fit into one area: user flash 0x%08x (%d k), '
                                       'boot 0x%08x (%d bytes), option bytes 0x%08x, RAM 0x%08x' % (
                                           len(data), addr, WCH_FLASH, t.flash // 1024, WCH_BOOT, WCH_BOOT_SIZE,
                                           WCH_OPT, WCH_RAM))
                if area[0] == 'options':
                    if addr != WCH_OPT or len(data) != 16:
                        raise FlasherError('option bytes: exactly 16 bytes at 0x%08x' % WCH_OPT)
                    wch_options_check(data)
                    for line in wch_options_text(data):
                        print('New:     ' + line)
                    if data[0] != 0xa5:
                        print('These option bytes switch the READ PROTECTION ON: afterwards the flash can only be '
                              'opened again by erasing it.')
                    if not args.write_options:
                        raise FlasherError('writing the option bytes needs --write-options')
                    f.cmd('WOB ' + data.hex(), timeout=20)
                    print('Option bytes written; they take effect at the next reset or power cycle.')
                    return t
                print('Write 0x%08x..0x%08x (%s)' % (addr, addr + len(data), area[0]))
                if area[0] == 'ram':
                    f.stream_write('C', addr, data + b'\x00' * (-len(data) % 4), unit=WCH_PAGE, skip=False)
                else:
                    wch_write_flash(f, t, addr, data)
                if not args.no_verify:
                    bad = wch_verify(f, addr, data)
                    if bad is not None:
                        raise FlasherError('verify failed in the block at 0x%08x' % bad)
                    print('Verified.')
    else:
        raise FlasherError('over SWIO: info, rf, wf, es, ea, unlock')
    if args.run:
        f.cmd('WRUN', timeout=5)
        print('Target restarted (its firmware may take the SWIO pin again; the next command catches it at power-up).')
    return t


# ---- Beken BK72xx (BK7231N, BK7231T) over the UART download protocol: TX1 on the SWS wire, RX1 on GPIO 21, CEN on
#      the reset wire, 3V3 on the power pins. The protocol itself runs on the flasher (BK commands, src/bk72xx.cpp).
BK_BOOT_END = 0x11000           # the bootloader in flash, on BK7231N and BK7231T
BK_SECTOR, BK_BLOCK = 0x1000, 0x10000
BK_CHIPS = {0x7231C: 'BK7231N', 0x7231A: 'BK7231T', 0x7231: 'BK7231Q', 0x7238: 'BK7238', 0x7252: 'BK7252'}
# CRC-32 of the first 256 bytes of the bootloader (257 on the BootROM protocol, whose CRC range includes the end
# address), and the chip ID register value: from bk7231tools (MIT, tuya-cloudcutter/bk7231tools, serial/base/enums.py)
BK_BOOTLOADERS = {
    0xE14191BA: 'BK7231N 1.0.1', 0x9B49BF9C: 'BK7231N 3.0.1', 0xF42F8C32: 'BK7238 (T1) 2.0.0',
    0x6BEB0924: 'BK7238 1.0.14', 0xF0231EF6: 'BK7231Q', 0xFF5A3EAC: 'BK7231Q (Tuya SDK)',
    0xC1ECA871: 'BK7231T (BK7231S 1.0.1)', 0x4B31E44D: 'BK7231T (BK7231S 1.0.3)', 0xBA54C1B8: 'BK7231T (BK7231S 1.0.5)',
    0xE5CBC953: 'BK7231T (BK7231S 1.0.6)', 0x2739019F: 'BK7231U 1.0.6', 0x39F9B50C: 'BK7252 0.1.3',
    0xE3A27C26: 'BK7252 (SDK)'}
# Number of status register bytes of the flash chips found on BK72xx modules, by JEDEC id: from bk7231tools
# (MIT, serial/cmd_hl_flash.py, FLASH_SR_SIZE). The protection bits cleared before an erase or write are the same as
# there: BP0..BP4 (bits 2..6) and, with two bytes, CMP (bit 14).
BK_FLASH_SR = {
    '0b4014': 2, '0b4015': 2, '0b4016': 2, '0b4017': 2, '0b6017': 2, '0e4016': 2, '1c3113': 1, '1c4116': 1,
    '1c7015': 1, '1c7016': 1, '204016': 2, '514013': 1, '514014': 1, '5e4014': 1, '854215': 1, '852015': 2,
    '856013': 2, '856014': 2, '856016': 2, '856017': 2, 'c22314': 2, 'c22315': 2, 'c84013': 1, 'c84014': 2,
    'c84015': 2, 'c84016': 1, 'c86515': 2, 'c86516': 2, 'c86517': 2, 'cd6014': 2, 'e04013': 1, 'e04014': 1,
    'eb6015': 2, 'ef4016': 2, 'ef4018': 2}
BK_FLASH_VENDORS = {0x0B: 'XTX', 0x1C: 'ESMT / EON', 0x20: 'XMC', 0x51: 'GigaDevice (MD)', 0x5E: 'XTX / Zbit',
                    0x68: 'Boya', 0x85: 'Puya', 0xA1: 'Fudan', 0xC2: 'Macronix', 0xC4: 'Giantec', 0xC8: 'GigaDevice',
                    0xCD: 'TH', 0xE0: 'Boya', 0xEB: 'TH', 0xEF: 'Winbond'}
# RF calibration with the MAC address (TLV) at these addresses, as ltchiptool reads it
BK_CALIBRATION = {'BK7231N': 0x1D0000, 'BK7231T': 0x1E0000}
UF2_FAMILIES = {0x7B3EF230: 'BK7231N', 0x675A40B0: 'BK7231T', 0xAFE81D49: 'BK7231Q'}


class BkTarget:
    def __init__(self, rom=True, baud=115200, chip_id=0, flash_id='', size=0x200000):
        self.rom, self.baud, self.chip_id, self.flash_id, self.size = rom, baud, chip_id, flash_id, size
        self.chip = BK_CHIPS.get(chip_id, '') if rom else 'BK7231T'
        self.bootloader = ''
        self.sr = None                # (value, bytes, mask) of the flash status registers, BootROM only


def bk_spans(t, addr, size):
    """The BK7231T bootloader refuses its own area, but the same flash is reached at the address plus the flash
    size (the flash ignores the upper address bits): (address on the wire, logical address, length) pieces."""
    out = []
    while size > 0:
        if not t.rom and addr < BK_BOOT_END:
            n = min(size, BK_BOOT_END - addr)
            out.append((addr + t.size, addr, n))
        else:
            n = size
            out.append((addr, addr, n))
        addr += n
        size -= n
    return out


def bk_crc(f, t, addr, size):
    """zlib CRC-32 of a flash range, computed by the target."""
    crc = None
    for wire, _, n in bk_spans(t, addr, size):
        part = int(f.cmd('BKC %d %d' % (wire, n), timeout=10 + n / 50000), 16)
        crc = part if crc is None else zlib_crc32_combine(crc, part, n)
    return crc


def zlib_crc32_combine(crc1, crc2, len2):
    """CRC-32 of A + B from crc(A), crc(B) and len(B) (zlib's crc32_combine)."""
    def gf2_times(mat, vec):
        s, i = 0, 0
        while vec:
            if vec & 1:
                s ^= mat[i]
            vec >>= 1
            i += 1
        return s

    def gf2_square(mat):
        return [gf2_times(mat, mat[n]) for n in range(32)]
    if len2 <= 0:
        return crc1
    odd = [0xEDB88320] + [1 << n for n in range(31)]
    even = gf2_square(odd)
    odd = gf2_square(even)
    while True:
        even = gf2_square(odd)
        if len2 & 1:
            crc1 = gf2_times(even, crc1)
        len2 >>= 1
        if not len2:
            break
        odd = gf2_square(even)
        if len2 & 1:
            crc1 = gf2_times(odd, crc1)
        len2 >>= 1
        if not len2:
            break
    return crc1 ^ crc2


def bk_read(f, t, addr, size, progress=True):
    out = b''
    for wire, _, n in bk_spans(t, addr, size):
        out += f.stream_read('B', wire, n, progress=progress)
    return out


def bk_attach(f, args):
    print('BK72xx:  catching the target at power-up (up to %d s; reset by RST = CEN and the power pins)...'
          % args.bk_seconds)
    f.cmd('BK')
    a = f.cmd('BKL %d %d' % (args.bk_seconds * 1000, args.bk_baud), timeout=2 * args.bk_seconds + 40).split(' ', 6)
    rom, baud, resets, ms, fallback = a[0] == 'rom', int(a[1]), int(a[2]), int(a[3]), a[4] == '1'
    print('Link:    %s protocol at %d baud, %d reset%s, answer %d ms after the last one%s' % (
        'BootROM' if rom else 'bootloader', baud, resets, '' if resets == 1 else 's', ms,
        ', %s bytes of its own firmware before' % a[5] if a[5] != '0' else ''))
    if fallback:
        print('         %d baud did not work (%s); using %d' % (args.bk_baud, a[6] if len(a) > 6 else '?', baud))
    return rom, baud


def bk_sr_layout(t, args):
    """(status bytes, protection mask) of the flash, or None if the flash is not known."""
    vendor = int(t.flash_id[:2], 16)
    if vendor == 0xC2:
        return 1, 0x3C          # Macronix: bit 6 is QE, BP0..BP3 only; the second byte is a configuration register
    n = args.bk_sr or BK_FLASH_SR.get(t.flash_id)
    if not n:
        return None
    return n, 0x407C if n == 2 else 0x7C


def bk_status(f, n):
    sr = int(f.cmd('BKSR 0x05'), 16)
    if n == 2:
        sr |= int(f.cmd('BKSR 0x35'), 16) << 8
    return sr


def bk_info(f, args, say=print):
    rom, baud = bk_attach(f, args)
    chip_id, fid = f.cmd('BKI').split()
    t = BkTarget(rom, baud, int(chip_id, 16), fid)
    if rom:
        t.size = 1 << int(fid[4:6], 16) if 0x11 <= int(fid[4:6], 16) <= 0x19 else 0x200000
        say('Chip:    %s (chip id 0x%x)' % (t.chip or 'unknown', t.chip_id))
    else:
        say('Chip:    bootloader protocol, no chip id: BK7231T (or another chip whose download code is in its '
            'flash); flash size taken as 2 Mbytes')
    crc = int(f.cmd('BKC 0 %d' % (257 if rom else 256)), 16)
    t.bootloader = BK_BOOTLOADERS.get(crc, '')
    say('Boot:    %s (CRC %08x of its first bytes)' % (t.bootloader or 'bootloader not in the list', crc))
    if rom:
        lay = bk_sr_layout(t, args)
        vendor = BK_FLASH_VENDORS.get(int(fid[:2], 16), 'unknown maker')
        if lay:
            sr = bk_status(f, lay[0])
            t.sr = (sr, lay[0], lay[1])
            prot = 'protected, cleared before erase / write' if sr & lay[1] else 'not protected'
            say('Flash:   JEDEC %s (%s), %d kbytes, status %0*x: %s' % (fid, vendor, t.size // 1024, 2 * lay[0],
                                                                         sr, prot))
        else:
            sr = int(f.cmd('BKSR 0x05'), 16)
            t.sr = (sr, 1, 0x7C)
            say('Flash:   JEDEC %s (%s), %d kbytes, status 1 = %02x (%s); this flash is not in the list: '
                '--bk-sr 1|2 gives its number of status bytes' % (fid, vendor, t.size // 1024, sr,
                                                                  'protected' if sr & 0x7C else 'no BP bits set'))
    cal = BK_CALIBRATION.get(t.chip)
    if cal and cal + BK_SECTOR <= t.size:
        tlv = bk_read(f, t, cal, 0x40, progress=False)
        if tlv[0x1C:0x24] == b'\x02\x11\x11\x11\x06\x00\x00\x00':
            say('MAC:     %s (in the RF calibration at 0x%x)' % (':'.join('%02x' % b for b in tlv[0x24:0x2A]), cal))
    return t


def bk_unprotect(f, t, args):
    if not t.rom:
        return                  # the BK7231T bootloader has no status register commands and unlocks by itself
    lay = bk_sr_layout(t, args)
    sr = bk_status(f, lay[0] if lay else 1)
    if lay is None:
        if sr & 0x7C:
            raise FlasherError('flash %s is not in the list and its status register 0x%02x has protection bits set; '
                               'give its number of status bytes with --bk-sr 1|2' % (t.flash_id, sr))
        print('Flash %s: not in the list; status 1 has no BP bits set, going on' % t.flash_id)
        return
    n, mask = lay
    if not sr & mask:
        return
    new = sr & ~mask
    print('Flash:   status %0*x -> %0*x (protection off)' % (2 * n, sr, 2 * n, new))
    f.cmd('BKSW 1 %s' % new.to_bytes(n, 'little').hex())
    back = bk_status(f, n)
    if back & mask:
        raise FlasherError('the flash is still protected (status %0*x)' % (2 * n, back))


def bk_guard(t, args, start, end, what):
    """Refuse anything in the bootloader area unless --force-bootloader (never on a BK7231T)."""
    if start >= BK_BOOT_END:
        return
    if not t.rom:
        raise FlasherError('%s would touch 0x%x..0x%x: on a BK7231T the bootloader IS the UART download code; '
                           'overwriting it bricks UART flashing (SPI only). Refused.' % (what, start, BK_BOOT_END))
    if not args.force_bootloader:
        raise FlasherError('%s would touch the bootloader area 0x0..0x%x. Refused; --force-bootloader allows it. '
                           '(For an application image use the address 0x%x.)' % (what, BK_BOOT_END, BK_BOOT_END))
    print('WARNING: %s touches the bootloader at 0x0..0x%x (--force-bootloader). On a BK7231N the BootROM keeps '
          'UART flashing possible, but a wrong bootloader keeps the device from starting.' % (what, BK_BOOT_END))


def bk_erase(f, t, start, end):
    """Erase [start, end), 4 k aligned: 64 k blocks where a whole one is inside, 4 k sectors elsewhere."""
    a = start
    while a < end:
        if a % BK_BLOCK == 0 and a + BK_BLOCK <= end:
            print('\rErase 0x%06x (64 k)' % a, end='', flush=True)
            f.cmd('BKE %d 64' % bk_spans(t, a, 1)[0][0], timeout=15)
            a += BK_BLOCK
        else:
            print('\rErase 0x%06x      ' % a, end='', flush=True)
            f.cmd('BKE %d' % bk_spans(t, a, 1)[0][0], timeout=6)
            a += BK_SECTOR
    print('\r' + ' ' * 28 + '\r', end='')


def bk_write(f, t, addr, data):
    """Sector by sector: sectors whose CRC already matches are skipped, what the image does not cover inside a
    sector is read first and kept; each run of changed sectors is erased, then streamed in one go."""
    if addr + len(data) > t.size:
        raise FlasherError('0x%x bytes at 0x%x do not fit into %d kbytes' % (len(data), addr, t.size // 1024))
    first = addr - addr % BK_SECTOR
    end = addr + len(data)
    end += -end % BK_SECTOR
    want = bytearray()
    if first < addr:
        want += bk_read(f, t, first, BK_SECTOR, progress=False)[:addr - first]
    want += data
    if len(want) < end - first:
        want += bk_read(f, t, end - BK_SECTOR, BK_SECTOR, progress=False)[len(want) - (end - BK_SECTOR - first):]
    runs, skipped = [], 0
    chunk = first
    while chunk < end:                             # a CRC per 64 k block first, per 4 k inside one that differs
        cend = min(end, chunk - chunk % BK_BLOCK + BK_BLOCK)
        print('\rCompare 0x%06x' % chunk, end='', flush=True)
        if bk_crc(f, t, chunk, cend - chunk) == zlib.crc32(want[chunk - first:cend - first]):
            skipped += (cend - chunk) // BK_SECTOR
        else:
            for a in range(chunk, cend, BK_SECTOR):
                part = bytes(want[a - first:a - first + BK_SECTOR])
                if bk_crc(f, t, a, BK_SECTOR) == zlib.crc32(part):
                    skipped += 1
                elif runs and runs[-1][0] + len(runs[-1][1]) == a:
                    runs[-1][1] += part
                else:
                    runs.append([a, bytearray(part)])
        chunk = cend
    print('\r' + ' ' * 28 + '\r', end='')
    for start, buf in runs:
        bk_erase(f, t, start, start + len(buf))
        for wire, logical, n in bk_spans(t, start, len(buf)):
            f.stream_write('B', wire, bytes(buf[logical - start:logical - start + n]), unit=BK_SECTOR, skip=True)
    if skipped:
        print('%d of %d sectors already held the data.' % (skipped, (end - first) // BK_SECTOR))


def bk_verify(f, t, addr, data):
    """The target's CRC of the whole range; on a mismatch, block by block to find the first bad one."""
    if bk_crc(f, t, addr, len(data)) == zlib.crc32(data):
        return None
    for pos in range(0, len(data), BK_SECTOR):
        n = min(BK_SECTOR - (addr + pos) % BK_SECTOR, len(data) - pos)
        if bk_crc(f, t, addr + pos, n) != zlib.crc32(data[pos:pos + n]):
            return addr + pos
    return addr


def uf2_parts(raw, chip=''):
    """A LibreTiny / ESPHome UF2 (ltchiptool's uf2tool, OTA format 2): the parts written over UART (the "flasher"
    OTA scheme) as [(flash offset, data)]. A block's address is relative to the partition that the last
    OTA_PART_INFO tag names for that scheme; the partitions' offsets are in the FAL_PTABLE tag of the header."""
    if len(raw) % 512 or not raw:
        raise FlasherError('UF2: the size is not a multiple of 512')
    gtags, part, family, parts, ptable = {}, None, None, [], {}
    for i in range(0, len(raw), 512):
        b = raw[i:i + 512]
        m1, m2, flags, addr, length, seq, _, fam = (int.from_bytes(b[k:k + 4], 'little') for k in range(0, 32, 4))
        if m1 != 0x0A324655 or m2 != 0x9E5D5157 or int.from_bytes(b[508:], 'little') != 0x0AB16F30 or length > 476:
            raise FlasherError('UF2: bad block %d' % (i // 512))
        if seq != i // 512:
            raise FlasherError('UF2: block %d has the sequence number %d' % (i // 512, seq))
        if flags & 0x2000:
            family = fam
        tags = {}
        if flags & 0x8000:
            tb = b[32 + length:508]
            k = 0
            while k + 4 <= len(tb) and tb[k]:
                n = tb[k]
                tags[int.from_bytes(tb[k + 1:k + 4], 'little')] = tb[k + 4:k + n]
                k += n + (-n % 4)
        if not length:
            gtags.update(tags)
            if 0x8288ED in tags:            # FAL_PTABLE: magic, name[16], flash name[16], offset, length, padding
                pt = tags[0x8288ED]
                for k in range(0, len(pt) - 47, 48):
                    if int.from_bytes(pt[k:k + 4], 'little') == 0x45503130:
                        name = pt[k + 4:k + 20].split(b'\0')[0].decode()
                        ptable[name] = (int.from_bytes(pt[k + 36:k + 40], 'little'),
                                        int.from_bytes(pt[k + 40:k + 44], 'little'))
            continue
        if 0xC0EE0C in tags:                # OTA_PART_INFO: 6 nibbles (partition index per scheme), names
            info = tags[0xC0EE0C]
            names = [x.decode() for x in info[3:].split(b'\0') if x]
            index = int(info[0:3].hex()[3])     # scheme 3: FLASHER_SINGLE
            part = names[index - 1] if index else None
        if not part:
            continue
        if part not in ptable:
            raise FlasherError('UF2: partition "%s" is not in the partition table of the file' % part)
        start, plen = ptable[part]
        if addr + length > plen:
            raise FlasherError('UF2: data beyond the end of partition "%s"' % part)
        offs = start + addr
        if parts and parts[-1][0] + len(parts[-1][1]) == offs:
            parts[-1][1] += b[32:32 + length]
        else:
            parts.append([offs, bytearray(b[32:32 + length])])
    if 0x6C8492 not in gtags:
        raise FlasherError('UF2: not the OTA format 2 of LibreTiny (ltchiptool 3.x or older wrote it); use the '
                           '.0x011000.rbl image of the build instead')
    if not parts:
        raise FlasherError('UF2: nothing in it is meant for flashing over UART')
    fam = UF2_FAMILIES.get(family, '0x%08x' % (family or 0))
    if chip and fam != chip:
        raise FlasherError('UF2: this image is for %s, the chip is %s' % (fam, chip))
    board = gtags.get(0xCA25C8, b'').decode(errors='replace')
    fw = gtags.get(0x00DE43, b'').decode(errors='replace')
    ver = gtags.get(0x9FC7BC, b'').decode(errors='replace')
    print('UF2:     %s %s, board %s, family %s: %s' % (fw, ver, board, fam, ', '.join(
        '0x%x bytes at 0x%06x' % (len(d), o) for o, d in parts)))
    return [(o, bytes(d)) for o, d in parts]


def bk_image(f, t, args):
    """The file of wf as [(flash address, data)], with the bootloader area handled."""
    with open(args.filename, 'rb') as fh:
        raw = fh.read()
    if not raw:
        raise FlasherError('empty file')
    if raw[:4] == b'UF2\n':
        parts = uf2_parts(raw, t.chip if t.chip in ('BK7231N', 'BK7231T') else '')
        if args.address not in (0, parts[0][0]):
            raise FlasherError('a UF2 says itself where it goes: give the address 0')
        return parts
    if raw[:4] == b'RBL\0':
        raise FlasherError('this is an OTA package (it starts with its RBL header), not for flashing over UART: give '
                           'the .uf2, the ..._app.0x011000.rbl (at 0x11000) or OpenBeken\'s _UA_ / _QIO_ .bin')
    if raw[:4] == b'\x55\xaa\x55\xaa':
        raise FlasherError('this is a Tuya OTA package (UG), not for flashing over UART')
    addr = args.address
    if addr == 0 and len(raw) > BK_BOOT_END and not args.force_bootloader:
        # a full image: OpenBeken's _QIO_ file or a backup. Its bootloader part is left out.
        chip_crc = bk_crc(f, t, 0, BK_BOOT_END)
        same = chip_crc == zlib.crc32(raw[:BK_BOOT_END])
        print('Full image from 0: its first 0x%x bytes (bootloader) are %s; written from 0x%x on. '
              '--force-bootloader writes them too.' % (BK_BOOT_END, 'the same as on the chip' if same else
                                                       'DIFFERENT from the chip\'s, which stays', BK_BOOT_END))
        return [(BK_BOOT_END, raw[BK_BOOT_END:])]
    if '_UA_' in os.path.basename(args.filename) and addr != BK_BOOT_END:
        print('Note: OpenBeken\'s _UA_ images go to 0x%x.' % BK_BOOT_END)
    return [(addr, raw)]


def bk_main(f, args):
    try:
        bk_ops(f, args)
    finally:
        f.stop()                # after Ctrl-C or an error: stop a stream that may still run
        time.sleep(0.3)
        f.ser.reset_input_buffer()
        f.rxbuf = b''


def bk_ops(f, args):
    if args.op not in ('info', 'rf', 'wf', 'es', 'ea'):
        raise FlasherError('BK72xx: info, rf, wf, es, ea')
    t = bk_info(f, args, say=print if args.op == 'info' else (lambda *a: None))
    if args.op == 'info':
        pass
    elif args.op == 'rf':
        size = args.size or t.size - args.address
        if args.address + size > t.size:
            raise FlasherError('0x%x bytes at 0x%x: beyond the flash (%d kbytes)' % (size, args.address, t.size // 1024))
        os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
        t1 = time.time()
        data = bk_read(f, t, args.address, size)
        print('Read 0x%x bytes in %.1f s (%.0f kB/s); every 4 k block checked against the target\'s CRC' % (
            size, time.time() - t1, size / max(time.time() - t1, 1e-3) / 1000))
        if args.twice:
            bad = bk_verify(f, t, args.address, data)
            if bad is not None:
                with open(args.filename + '.pass1', 'wb') as fh:
                    fh.write(data)
                raise FlasherError('the target\'s CRC of the range differs in the block at 0x%06x; the read was kept '
                                   'as %s.pass1' % (bad, args.filename))
            print('Second pass (CRC of the whole range on the target) matches.')
        with open(args.filename, 'wb') as fh:
            fh.write(data)
        print('Saved %d bytes to %s' % (len(data), args.filename))
    elif args.op == 'wf':
        parts = bk_image(f, t, args)
        for addr, data in parts:
            bk_guard(t, args, addr, addr + len(data), 'writing 0x%06x..0x%06x' % (addr, addr + len(data)))
        if args.force_bootloader:
            f.cmd('BKG 0')
        bk_unprotect(f, t, args)
        for addr, data in parts:
            print('Write 0x%06x..0x%06x' % (addr, addr + len(data)))
            bk_write(f, t, addr, data)
        if not args.no_verify:
            for addr, data in parts:
                bad = bk_verify(f, t, addr, data)
                if bad is not None:
                    raise FlasherError('verify failed in the block at 0x%06x' % bad)
            print('Verified (CRC on the target).')
    elif args.op == 'es':
        start = args.address - args.address % BK_SECTOR
        end = args.address + max(args.size, 1)
        end = min(end + (-end % BK_SECTOR), t.size)
        bk_guard(t, args, start, end, 'erasing 0x%06x..0x%06x' % (start, end))
        if args.force_bootloader:
            f.cmd('BKG 0')
        bk_unprotect(f, t, args)
        bk_erase(f, t, start, end)
        print('Erased 0x%06x..0x%06x' % (start, end))
    elif args.op == 'ea':
        cal = BK_CALIBRATION.get(t.chip)
        print('Erasing 0x%06x..0x%06x: everything but the bootloader%s.' % (
            BK_BOOT_END, t.size, ', also the RF calibration and MAC address at 0x%x' % cal if cal else ''))
        bk_unprotect(f, t, args)
        bk_erase(f, t, BK_BOOT_END, t.size)
    else:
        raise FlasherError('BK72xx: info, rf, wf, es, ea')
    if args.run:
        f.cmd('BKX')
        print('Target restarted (reboot command).')
    return t


def main():
    ap = argparse.ArgumentParser(description='ESP32-C3 Telink TLSR825x SWire flasher')
    ap.add_argument('-p', '--port', default='/dev/ttyACM0')
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
    ap.add_argument('--swd-khz', type=auto_int, default=1000, help='SWD / JTAG clock in kHz (default 1000)')
    ap.add_argument('--jtag', action='store_true',
                    help='four-wire JTAG (TCK on GPIO 0, TMS on GPIO 20, TDI on the SWS wire, TDO on GPIO 21): '
                         'info scans the chain; an Arm DAP in it is used like SWD: info, rf, wf, es, ea')
    ap.add_argument('--cjtag', action='store_true',
                    help='TI CC13xx / CC26xx over two-wire cJTAG (TMSC on the SWS wire, TCKC on GPIO 0) '
                         'instead of SWD: info, rf, wf, es, ea')
    ap.add_argument('--spi', action='store_true',
                    help='SPI NOR flash (25-series): CS on GPIO 20, CLK on GPIO 0, DI on the '
                         'SWS wire, DO on GPIO 21; info, rf (size 0 = to the end), wf, es, ea')
    ap.add_argument('--spi-khz', type=auto_int, default=4000, help='SPI clock in kHz (default 4000)')
    ap.add_argument('--power', action='store_true',
                    help='SPI flash: power the board from the power pins (only for a flash out of circuit or a small '
                         'module, not for a router), switched off at the end')
    ap.add_argument('--unprotect', action='store_true',
                    help='SPI flash: clear the block protection bits before wf / es / ea')
    ap.add_argument('--wch', action='store_true',
                    help='WCH CH32V003 over its single-wire debug pin (SWIO on the SWS wire, power on the power '
                         'pins): info, rf, wf, es, ea, unlock')
    ap.add_argument('--wch-off', type=auto_int, default=300,
                    help='WCH: power-off time in ms before the catch at power-up (default 300)')
    ap.add_argument('--wch-window', type=auto_int, default=100,
                    help='WCH: how long to keep trying after power-on, ms (default 100)')
    ap.add_argument('--external-power', action='store_true',
                    help='WCH: the target has its own supply; no power cycle, so no catch at power-up')
    ap.add_argument('--wch-timing', default='',
                    help='WCH: SWIO timing in ns: one,zero,high,slot,threshold,stop (default 250,1000,375,1500,550,4000)')
    ap.add_argument('--save-options', default='', help='WCH: save the 16 option bytes to this file first')
    ap.add_argument('--write-options', action='store_true',
                    help='WCH: allow wf to the option bytes (0x1FFFF800, 16 bytes)')
    ap.add_argument('--yes-erase-all', action='store_true',
                    help='WCH: confirm unlock (removing the read protection erases the whole user flash)')
    ap.add_argument('--bk', action='store_true',
                    help='Beken BK72xx (BK7231N, BK7231T: CB2S, CB3S, WB2S ...) over its UART download port: TX1 on '
                         'the SWS wire, RX1 on GPIO 21, CEN on the reset wire, 3V3 on the power pins')
    ap.add_argument('--bk-baud', type=auto_int, default=921600,
                    help='BK72xx: speed after the link at 115200 (default 921600; 115200 when it fails)')
    ap.add_argument('--bk-seconds', type=auto_int, default=10,
                    help='BK72xx: how long to keep resetting and sending link checks (default 10)')
    ap.add_argument('--bk-sr', type=int, choices=[1, 2], default=0,
                    help='BK72xx: number of status register bytes of a flash chip that is not in the list')
    ap.add_argument('--force-bootloader', action='store_true',
                    help='BK72xx: allow wf / es in the bootloader area 0x0..0x11000 (BK7231N only)')
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
    sub.add_parser('unlock', help='WCH: remove the read protection (ERASES the user flash; needs --yes-erase-all)')
    p = sub.add_parser('bridge', help='make the flasher a UART bridge to the target (RX = SWS wire, TX = GPIO 21), '
                                      'then use another tool on the same port')
    p.add_argument('baud', type=auto_int)
    p.add_argument('--boot', choices=['hi', 'lo', 'z'], default='z',
                   help='level of the BOOT pin (GPIO 0) while the target is restarted (default: left open)')
    p.add_argument('--catch', choices=['bk'],
                   help='bk: reset the target (power pins and RST) every 1.5 s while the host sends and no Beken '
                        'BK72xx ROM answer (04 0E) has come yet, so that a flash tool links without a manual reset')
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
        with serial.Serial(args.port, 115200, timeout=2, dsrdtr=False, rtscts=False) as s:
            time.sleep(1.2)
            s.write(b'~~~X')
            answer = s.read_until(b'\n', 200)
        print(answer.decode(errors='replace').strip() or 'no answer (was the bridge open?)')
        return
    f = None
    try:
        f = Flasher(args.port)
        print('Flasher:', f.version)
        try:
            fw = tuple(int(x) for x in f.version.split()[1].split('.'))
        except (IndexError, ValueError):
            fw = (0, 0)
        if fw < (0, 18):
            raise FlasherError('this script needs flasher firmware 0.18 or newer on the C3 (binary streams)')
        if args.wch:
            if fw < (0, 22):
                raise FlasherError('WCH mode needs flasher firmware 0.22 or newer on the C3')
            wch_main(f, args)
            print('Done in %.1f s' % (time.time() - t0))
            return
        if args.op == 'unlock':
            raise FlasherError('unlock is a WCH command (--wch)')
        if args.bk:
            if fw < (0, 24):
                raise FlasherError('BK72xx mode needs flasher firmware 0.24 or newer on the C3')
            bk_main(f, args)
            print('Done in %.1f s' % (time.time() - t0))
            return
        if args.spi:
            if fw < (0, 21):
                raise FlasherError('SPI flash needs flasher firmware 0.21 or newer on the C3 (chip select moved '
                                   'to GPIO 20, the reset wire holds the board in reset)')
            spi_main(f, args)
            print('Done in %.1f s' % (time.time() - t0))
            return
        if args.jtag:
            if fw < (0, 19):
                raise FlasherError('JTAG needs flasher firmware 0.19 or newer on the C3')
            print('JTAG:    %s kHz' % f.cmd('JTAG %d' % args.swd_khz))
            ids, dap = jtag_chain(f, print if args.op == 'info' else (lambda *a: None))
            if dap is None:
                if args.op == 'info':
                    print('No Arm debug port in the chain: only the scan above (JIR / JDR for raw scans).')
                    return
                raise FlasherError('no Arm debug port (JTAG-DP) in the chain')
            if args.op == 'info':
                print('Arm DAP: device %d' % dap)
            args.swd = True
            f.jtag = True
        if args.cjtag:
            args.swd = True
            f.cjtag = True
            if fw < (0, 16):
                raise FlasherError('cJTAG needs flasher firmware 0.16 or newer on the C3')
        if args.op in ('bridge', 'power-off') or args.swd:
            if fw < (0, 15):
                raise FlasherError('this needs flasher firmware 0.15 or newer on the C3')
        if args.op == 'bridge' and args.catch and fw < (0, 23):
            raise FlasherError('--catch needs flasher firmware 0.23 or newer on the C3')
        if args.op == 'power-off':
            f.cmd('PWR 0')
            time.sleep(0.2)
            f.cmd('PWR z')
            print('Target power off, power pins are inputs.')
            return
        if args.op == 'bridge':
            print(f.cmd('BRIDGE %d %s%s' % (args.baud, args.boot, ' ' + args.catch if args.catch else '')))
            f.ser.close()
            print('The port is free for the target tool now. Do not let it toggle DTR / RTS (they reset the C3);')
            print('for example esptool --before no_reset --after no_reset. Leave with: bridge-exit')
            return
        if args.swd:
            if not args.jtag:
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
                                                        'stm32u5', 'samd21', 'samd51', 'py32'):
                        print('Mass erase %s...' % t.name)
                        f.cmd('SFX', timeout=60)
                        if t.family == 'nrf53':
                            # an erased UICR.APPROTECT / SECUREAPPROTECT means protected on current nRF5340
                            # silicon: keep the debug port open after the next reset
                            for a in (0x00FF8000, 0x00FF801C):
                                f.cmd('SFW %d %s' % (a, (0x50FA50FA).to_bytes(4, 'little').hex()))
                            print('UICR APPROTECT / SECUREAPPROTECT set to unprotected (0x50FA50FA).')
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
                print('Read in %.1f s (%.0f kB/s)' % (time.time() - t1, f.last_rate / 1000))
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
    except KeyboardInterrupt:
        if f is not None:
            f.stop()
        print('\nInterrupted.')
        sys.exit(130)
    print('Done in %.1f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
