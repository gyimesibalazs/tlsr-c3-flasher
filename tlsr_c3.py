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
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp info                     # ESP32 / ESP8266 ROM download mode (esptool)
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp rf 0 0 backup.bin        # whole flash (esptool 5; give the size on 4)
    python3 tlsr_c3.py -p /dev/ttyACM0 --esp wf 0 firmware.bin
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln info                      # Lightning LN882H over its UART download mode
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln rf 0 0 backup.bin --twice # whole flash
    python3 tlsr_c3.py -p /dev/ttyACM0 --ln wf 0 firmware.uf2 --run   # LibreTiny / ESPHome UF2
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl info                     # Realtek RTL8710BN over its UART download mode
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl rf 0 0 backup.bin        # whole flash, checked by the chip's MD5
    python3 tlsr_c3.py -p /dev/ttyACM0 --rtl2 info                    # Realtek RTL8720CF over its UART download mode
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge 115200 --boot hi        # UART bridge, BOOT pin high
    python3 tlsr_c3.py -p /dev/ttyACM0 bridge-exit
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi join MyNetwork           # Wi-Fi link (firmware 0.29): join a network
    python3 tlsr_c3.py -p /dev/ttyACM0 wifi key                      # set the key a TCP client must give
    python3 tlsr_c3.py --host c3-flasher.local --bk info              # any command over TCP instead of USB
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --tuya             # two-line UART logger (firmware 0.30), live
    python3 tlsr_c3.py -p /dev/ttyACM0 sniff 9600 --at-boot           # capture from every power-up, read with log
"""
import argparse
import base64
import getpass
import hashlib
import itertools
import os
import re
import select
import socket
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


# ================================================================ the link to the flasher: USB or TCP (Wi-Fi)
# Over Wi-Fi (firmware 0.29) the flasher takes the same commands on a TCP port; its address is given as
# socket://NAME:PORT wherever a serial port goes, so esptool can use it as it is. With a key set on the flasher every
# connection starts with "AUTH <key>" (answer "OK auth"); while a bridge opened over TCP is on, connections from the
# same computer need no key (esptool cannot send it).

WIFI_PORT = 7171


def is_tcp(port):
    return port.startswith('socket://')


def tcp_url(host):
    """--host NAME[:PORT] -> socket://NAME:PORT"""
    name, sep, port = host.rpartition(':')
    if not sep or not port.isdigit():
        name, port = host, str(WIFI_PORT)
    if not name:
        raise FlasherError('--host: NAME or NAME:PORT')
    return 'socket://%s:%s' % (name, port)


class TcpPort:
    """A TCP connection with the parts of serial.Serial this script uses. pyserial's own socket:// port reports at
    most one byte in in_waiting, so every read(in_waiting) would take a single byte; this one buffers what came."""
    def __init__(self, url, timeout=0.1, connect_timeout=5.0):
        self.name = url
        host, _, port = url[len('socket://'):].rpartition(':')
        try:
            self.sock = socket.create_connection((host, int(port)), timeout=connect_timeout)
        except (OSError, ValueError) as e:
            raise FlasherError('cannot reach the flasher at %s:%s (%s)' % (host, port, e))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(30)                   # sendall: a flasher that takes nothing for this long is gone
        self.timeout = timeout
        self.buf = bytearray()
        self.is_open = True
        self.closed_why = None                     # the flasher closed the connection: the error to raise

    def _pull(self, wait):
        """Takes what has come within wait seconds; False when nothing came."""
        if self.closed_why or not select.select([self.sock], [], [], max(wait, 0))[0]:
            return False
        try:
            chunk = self.sock.recv(65536)
        except OSError as e:
            raise serial.SerialException('the connection to the flasher failed: %s' % e)
        if not chunk:
            # what was received before is still read; its last words may say why (e.g. "ERR auth: ...")
            last = bytes(self.buf).decode(errors='replace').strip().splitlines()[-1:]
            self.closed_why = 'the flasher closed the connection' + (
                ' (%s)' % last[0] if last and last[0].startswith('ERR') else '')
            return False
        self.buf += chunk
        return True

    def _check_closed(self):
        if self.closed_why and not self.buf:
            raise serial.SerialException(self.closed_why)

    @property
    def in_waiting(self):
        self._pull(0)
        self._check_closed()
        return len(self.buf)

    def read(self, size=1):
        deadline = time.time() + (self.timeout or 0)
        while len(self.buf) < size and self._pull(deadline - time.time()):
            pass
        self._check_closed()
        out = bytes(self.buf[:size])
        del self.buf[:size]
        return out

    def read_until(self, expected=b'\n', size=None):
        deadline = time.time() + (self.timeout or 0)
        while expected not in self.buf and (size is None or len(self.buf) < size):
            if not self._pull(deadline - time.time()):
                break
        self._check_closed()
        end = self.buf.find(expected)
        n = end + len(expected) if end >= 0 else len(self.buf)
        if size is not None:
            n = min(n, size)
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def write(self, data):
        try:
            self.sock.sendall(data)
        except OSError as e:
            raise serial.SerialException('the connection to the flasher failed: %s' % e)
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        while self._pull(0):
            pass
        self.buf.clear()

    def close(self):
        if self.is_open:
            self.is_open = False
            self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def tcp_open(url, key=None, timeout=0.1):
    """Connects and gives the key first when there is one. The key is never printed."""
    port = TcpPort(url, timeout=timeout)
    if key:
        port.write(b'AUTH ' + key.encode() + b'\n')
        saved, port.timeout = port.timeout, 5.0
        try:
            answer = port.read_until(b'\n').decode(errors='replace').strip()
        except serial.SerialException:
            answer = ''
        port.timeout = saved
        if answer != 'OK auth':
            port.close()
            if answer.startswith('ERR auth'):
                raise FlasherError('the flasher at %s did not take the key' % url[len('socket://'):])
            raise FlasherError('the flasher at %s did not answer the key (%r)' % (url[len('socket://'):], answer))
    return port


class Flasher:
    def __init__(self, port, baud=460800, key=None):
        self.rxbuf = b''
        tcp = is_tcp(port)
        if tcp:
            self.ser = tcp_open(port, key)
        else:
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
            try:
                self.ser.write(b'\n')
                time.sleep(0.3)
                self.ser.reset_input_buffer()
                self.rxbuf = b''
                self.version = self.cmd('V', timeout=1)
                return
            except (FlasherError, serial.SerialException) as e:
                last = e
                if not tcp and isinstance(e, serial.SerialException):
                    raise
                if tcp and 'ERR auth' in str(e):
                    self.ser.close()
                    raise FlasherError('the flasher at %s wants its key: --key or C3_FLASHER_KEY'
                                       % port[len('socket://'):])
                if tcp and isinstance(e, serial.SerialException):
                    self.ser.close()
                    raise
        self.ser.close()
        raise FlasherError('the flasher does not answer on %s (%s); if it was left in the UART bridge, '
                           'run bridge-exit (it also ends the UART logger), otherwise unplug it and plug it in again' % (port, last))

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
# RF calibration with the MAC address (TLV) at these addresses, as ltchiptool reads it; BK7238: the Tuya T1 layout
# (LibreTiny boards/_base/beken-7238-tuya.json)
BK_CALIBRATION = {'BK7231N': 0x1D0000, 'BK7231T': 0x1E0000, 'BK7238': 0x1E3000}
UF2_FAMILIES = {0x7B3EF230: 'BK7231N', 0x675A40B0: 'BK7231T', 0xAFE81D49: 'BK7231Q', 0x159AC324: 'BK7238',
                0xA38090A8: 'LN882H', 0x22E0D6FC: 'RTL8710B', 0xE08F7564: 'RTL8720C'}


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


def uf2_parts(raw, chip='', scheme=3):
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
            index = int(info[0:3].hex()[scheme])  # 3: FLASHER_SINGLE, 4 / 5: FLASHER_DUAL_1 / 2
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
        parts = uf2_parts(raw, t.chip if t.chip in ('BK7231N', 'BK7231T', 'BK7238') else '')
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


# ================================================================ UART log
# The flasher keeps what went over the target's UART in the last UART session (bridge, --esp, --bk) in RAM, in
# blocks compressed with LZ4 (see src/uartlog.h for the format): the start of the session for good, the end in a
# ring. `log` reads it (LOG, then XR L) and writes it as text.

ULOG_DIR = {1: '>', 2: '<', 3: '#'}      # host -> target, target -> host, event
SNIFF_DIR = {1: 'B', 2: 'A', 3: '#'}     # the two-line logger (sniff): line B, line A, event


def lz4_block_decompress(src, raw_len):
    out = bytearray()
    i = 0
    while i < len(src):
        tok = src[i]
        i += 1
        lit = tok >> 4
        if lit == 15:
            while True:
                b = src[i]
                i += 1
                lit += b
                if b != 255:
                    break
        out += src[i:i + lit]
        i += lit
        if i >= len(src):
            break
        off = src[i] | src[i + 1] << 8
        i += 2
        m = tok & 15
        if m == 15:
            while True:
                b = src[i]
                i += 1
                m += b
                if b != 255:
                    break
        m += 4
        if off == 0 or off > len(out):
            raise FlasherError('UART log: bad LZ4 offset')
        for _ in range(m):                    # byte by byte: a match may overlap what it copies
            out.append(out[-off])
    if len(out) != raw_len:
        raise FlasherError('UART log: block unpacked to %d bytes instead of %d' % (len(out), raw_len))
    return bytes(out)


def _varint(b, i):
    v = shift = 0
    while True:
        c = b[i]
        i += 1
        v |= (c & 0x7F) << shift
        shift += 7
        if not c & 0x80:
            return v, i


def ulog_parse(data):
    """The stored blocks -> a list of (t_us, tag, payload), with ('gap', blocks) where the ring dropped blocks.
    Times count from the first block."""
    out = []
    i = 0
    base = prev_t0 = prev_seq = None
    u16 = lambda at: int.from_bytes(data[at:at + 2], 'little')
    while i + 20 <= len(data):
        if data[i:i + 2] != b'UL' or data[i + 3] != 2:
            raise FlasherError('UART log: no block header (version 2) at %d' % i)
        flags, pay_len, pay_stored = data[i + 2], u16(i + 4), u16(i + 6)
        seq, t0 = int.from_bytes(data[i + 8:i + 12], 'little'), int.from_bytes(data[i + 12:i + 16], 'little')
        rec_len, rec_stored = u16(i + 16), u16(i + 18)
        p = i + 20
        recs = data[p:p + rec_stored]
        pay = data[p + rec_stored:p + rec_stored + pay_stored]
        i = p + rec_stored + pay_stored
        recs = recs if flags & 2 else lz4_block_decompress(recs, rec_len)
        pay = pay if flags & 1 else lz4_block_decompress(pay, pay_len)
        if base is None:
            base = 0
        else:
            if seq != prev_seq + 1:
                out.append(('gap', seq - prev_seq - 1))
            base += (t0 - prev_t0) & 0xFFFFFFFF
        prev_t0, prev_seq = t0, seq
        t = base
        j = k = 0
        while j < len(recs):
            tag = recs[j]
            dt, j = _varint(recs, j + 1)
            ln, j = _varint(recs, j)
            t += dt
            out.append((t, tag, pay[k:k + ln]))
            k += ln
        if k != len(pay):
            raise FlasherError('UART log: block %d has %d bytes of data, its records %d' % (seq, len(pay), k))
    return out


def _period(payload):
    """The length of a short pattern the payload repeats at least 3 times exactly, else 0."""
    for p in range(1, 65):
        if len(payload) >= 3 * p and len(payload) % p == 0 and payload == payload[:p] * (len(payload) // p):
            return p
    return 0


def _hex_lines(head, payload):
    """16 bytes a line; a run of lines equal to the one before is one line saying how many (as hexdump's *)."""
    lines, prev, same = [], None, 0
    for off in range(0, len(payload), 16):
        chunk = payload[off:off + 16]
        if chunk == prev and len(chunk) == 16:
            same += 1
            continue
        if same:
            lines.append('%14s (the same line %d more times)' % ('', same))
            same = 0
        text = ''.join(chr(c) if 0x20 <= c < 0x7F else '.' for c in chunk)
        lines.append('%s %-47s  %s' % (head if not lines else ' ' * 14, chunk.hex(' '), text))
        prev = chunk
    if same:
        lines.append('%14s (the same line %d more times)' % ('', same))
    return lines


def _record_lines(t, tag, payload, dirs=ULOG_DIR):
    """One record: time in seconds, direction, hex and ASCII 16 bytes a line (an event as its text). A record made of
    one short pattern (link checks sent back to back) shows the pattern once with its count."""
    head = '%12.6f %s' % (t / 1e6, dirs.get(tag, '?'))
    if tag == 3:
        return ['%s %s' % (head, payload.decode(errors='replace'))]
    p = _period(payload)
    if p:
        return _hex_lines(head, payload[:p]) + ['%14s (this %d-byte pattern %d times in a row)' % ('', p, len(payload) // p)]
    return _hex_lines(head, payload)


def ulog_dirs(entries):
    """The direction letters of a log: A / B for a session of the two-line logger, else > / <."""
    for e in entries:
        if e[0] != 'gap' and e[1] == 3:
            return SNIFF_DIR if e[2].startswith(b'sniff') else ULOG_DIR
    return ULOG_DIR


def ulog_text(entries, annotate=None):
    """Readable lines (see _record_lines); a record repeated back to back is shown once with its count.
    annotate(tag, payload): extra lines after each data record (the Tuya decoder); records are not merged then."""
    dirs = ulog_dirs(entries)
    lines = []
    k = 0
    while k < len(entries):
        e = entries[k]
        if e[0] == 'gap':
            lines.append('          ... %d blocks dropped by the ring ...' % e[1])
            if annotate:
                annotate(None, None)                 # the decoders start over
            k += 1
            continue
        t, tag, payload = e
        rep = 1
        while not annotate and k + rep < len(entries) and entries[k + rep][0] != 'gap' and \
                entries[k + rep][1:] == (tag, payload) and tag != 3:
            rep += 1
        lines += _record_lines(t, tag, payload, dirs)
        if annotate and tag != 3:
            lines += ['%14s %s' % ('', x) for x in annotate(tag, payload)]
        if rep > 1:
            lines.append('%14s (the same %d more times, until %.6f)' % ('', rep - 1, entries[k + rep - 1][0] / 1e6))
        k += rep
    return '\n'.join(lines) + '\n'


def log_main(f, args):
    if args.clear:
        f.cmd('LOG 0')
        print('UART log cleared.')
        return
    answer = f.cmd('LOG')
    st = dict(kv.split('=') for kv in answer.split()[2:])
    stored = int(st['stored'])
    print('UART log: %s blocks, %s bytes of records in %d bytes stored, %s blocks dropped by the ring; '
          '%s bytes to the target, %s from it' % (st['blocks'], st['raw'], stored, st['dropped'], st['tx'], st['rx']))
    if 'heap' in st:
        print('Flasher RAM: %s bytes of heap free, %s at the least since it started%s' % (
            st['heap'], st['heap_min'], '; log store %d k' % (int(st['cap']) // 1024) if 'cap' in st else ''))
    if not stored:
        print('The log is empty (no UART session since the flasher started, or LOG 0).')
        return
    data = f.stream_read('L', 0, stored, label='Log')
    if args.bin:
        with open(args.bin, 'wb') as out:
            out.write(data)
    entries = ulog_parse(data)
    with open(args.filename, 'w', encoding='utf-8') as out:
        out.write(ulog_text(entries, TuyaLines() if getattr(args, 'tuya', False) else None))
    print('Written: %s (%d records)' % (args.filename, sum(1 for e in entries if e[0] != 'gap')))


# ================================================================ Tuya MCU serial protocol (sniff --tuya, log --tuya)
# Between a Tuya Wi-Fi module and the board's MCU: frames 55 AA <version> <command> <length:u16 BE> <data> <checksum:
# the sum of all bytes before it, mod 256>, the same in both directions. Data points (commands 06, 07, 22): dpid,
# type, length:u16 BE, value.

TUYA_COMMANDS = {0x00: 'heartbeat', 0x01: 'product info', 0x02: 'working mode', 0x03: 'network status',
                 0x04: 'reset (Wi-Fi)', 0x05: 'reset (select mode)', 0x06: 'DP command', 0x07: 'DP report',
                 0x08: 'DP query', 0x1C: 'local time', 0x22: 'DP report (record)'}
TUYA_DP_TYPES = {0: 'raw', 1: 'bool', 2: 'value', 3: 'string', 4: 'enum', 5: 'bitmap'}
TUYA_MAX_LEN = 1500        # a longer length field is taken for garbage, not waited for


def tuya_dp_value(typ, v):
    if typ == 1 and len(v) == 1:
        return 'true' if v[0] else 'false'
    if typ == 2 and len(v) == 4:
        u = int.from_bytes(v, 'big')
        return '%d' % u if u < 0x80000000 else '%d (%d signed)' % (u, u - (1 << 32))
    if typ == 3:
        return repr(v.decode('utf-8', 'replace'))
    if typ in (4, 5) and 1 <= len(v) <= 4:
        u = int.from_bytes(v, 'big')
        return '%d' % u if typ == 4 else '0x%0*x' % (2 * len(v), u)
    return v.hex(' ') or '(empty)'


def tuya_dps(d):
    """The data points of a DP command / report as text, or None when the data is not a list of them."""
    parts, i = [], 0
    while i < len(d):
        if i + 4 > len(d):
            return None
        dpid, typ, n = d[i], d[i + 1], int.from_bytes(d[i + 2:i + 4], 'big')
        v = d[i + 4:i + 4 + n]
        if len(v) < n:
            return None
        parts.append('dp %d %s len %d = %s' % (dpid, TUYA_DP_TYPES.get(typ, 'type %d' % typ), n, tuya_dp_value(typ, v)))
        i += 4 + n
    return parts


def tuya_frame_text(frame):
    ver, cmd, data = frame[2], frame[3], frame[6:-1]
    name = TUYA_COMMANDS.get(cmd)
    text = 'tuya v%d %s' % (ver, '%s (0x%02x)' % (name, cmd) if name else 'command 0x%02x' % cmd)
    dps = tuya_dps(data) if cmd in (0x06, 0x07, 0x22) and data else None
    if dps:
        return '%s: %s' % (text, '; '.join(dps))
    if cmd == 0x01 and data:
        return '%s: %r' % (text, data.decode('utf-8', 'replace'))
    if cmd == 0x1C and len(data) == 8 and data[0]:
        return '%s: 20%02d-%02d-%02d %02d:%02d:%02d, weekday %d' % ((text,) + tuple(data[1:8]))
    return '%s%s' % (text, ', data ' + data.hex(' ') if data else '')


class TuyaDecoder:
    """Frames of one line, reassembled across records; bytes outside frames are reported and skipped, a frame with a
    bad checksum is shown and the search goes on right after its 55 AA."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while self.buf:
            i = self.buf.find(b'\x55\xaa')
            if i < 0:
                i = len(self.buf) - (1 if self.buf[-1] == 0x55 else 0)   # a last 55 may start a frame
            if i > 0:
                out.append('tuya: %d bytes outside frames: %s' % (i, bytes(self.buf[:min(i, 32)]).hex(' ') +
                                                                    (' ...' if i > 32 else '')))
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            n = int.from_bytes(self.buf[4:6], 'big')
            if n > TUYA_MAX_LEN:
                out.append('tuya: 55 aa with a length of %d: not a frame' % n)
                del self.buf[:2]
                continue
            if len(self.buf) < 7 + n:
                break
            frame = bytes(self.buf[:7 + n])
            want = sum(frame[:-1]) & 0xFF
            if frame[-1] != want:
                out.append('tuya: bad checksum %02x (should be %02x): %s' % (frame[-1], want, frame.hex(' ')))
                del self.buf[:2]
                continue
            out.append(tuya_frame_text(frame))
            del self.buf[:7 + n]
        return out


class TuyaLines:
    """A decoder per line, as ulog_text's annotate: (tag, payload) -> lines; (None, None): start over (a gap)."""
    def __init__(self):
        self.dec = {}

    def __call__(self, tag, payload):
        if tag is None:
            self.dec = {}
            return []
        return self.dec.setdefault(tag, TuyaDecoder()).feed(payload)


# ================================================================ two-line UART logger (firmware 0.30)
# SNIFF <baud A> [<baud B>]: the flasher listens to line A (the SWS wire) and line B (GPIO 20) without driving
# anything. Answer "OK sniff ...", then binary frames <tag:u8> <t_us:u32 LE> <n:u16 LE> <data> (tag 1 = line B, 2 =
# line A, 3 = event text) until the escape (a pause of a second, then ~~~X), answered by "OK sniff closed" after the
# last frame; a frame starts with 1..3, a text line with a letter. Everything also goes into the UART log (log).
# SNIFFBOOT <a> <b> | 0: capture from the flasher's power-up without a host, until a host's first command.

SNIFF_ESCAPE = b'~~~X'


class SniffParser:
    """Live output -> items: ('frame', tag, t_us, data) or ('line', text)."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while self.buf:
            if self.buf[0] in (1, 2, 3):
                if len(self.buf) < 7:
                    break
                n = int.from_bytes(self.buf[5:7], 'little')
                if len(self.buf) < 7 + n:
                    break
                out.append(('frame', self.buf[0], int.from_bytes(self.buf[1:5], 'little'), bytes(self.buf[7:7 + n])))
                del self.buf[:7 + n]
            else:
                end = self.buf.find(b'\n')
                if end < 0:
                    break
                out.append(('line', bytes(self.buf[:end]).decode(errors='replace').strip()))
                del self.buf[:end + 1]
        return out


class UsClock:
    """The flasher's 32-bit microsecond times -> microseconds since the first one (across the wrap)."""
    def __init__(self):
        self.prev = None
        self.total = 0

    def __call__(self, t):
        if self.prev is not None:
            self.total += (t - self.prev) & 0xFFFFFFFF
        self.prev = t
        return self.total


class SniffView:
    """Live frames -> printed lines (as the log's text, A / B), optionally decoded; counts what came."""
    def __init__(self, tuya=False, out=None, say=print):
        self.clock = UsClock()
        self.tuya = TuyaLines() if tuya else None
        self.out, self.say = out, say
        self.bytes = {1: 0, 2: 0}
        self.dropped = 0

    def frame(self, tag, t_us, data):
        lines = _record_lines(self.clock(t_us), tag, data, SNIFF_DIR)
        if tag in self.bytes:
            self.bytes[tag] += len(data)
            if self.tuya:
                lines += ['%14s %s' % ('', x) for x in self.tuya(tag, data)]
        else:
            m = re.match(rb'live output: (\d+) bytes dropped', data)
            if m:
                self.dropped += int(m.group(1))
        for x in lines:
            self.say(x)
            if self.out:
                self.out.write(x + '\n')


def sniff_live(f, baud_a, baud_b, seconds=0, view=None):
    """SNIFF, show the frames until `seconds` have passed or Ctrl-C, then the escape; returns the closing answer."""
    view = view or SniffView()
    answer = f.cmd('SNIFF %d %d' % (baud_a, baud_b))
    t_cmd = time.time()
    view.say('Listening: %s' % answer.split(';')[0])
    view.say('Ctrl-C ends it%s.' % (' (or %g s)' % seconds if seconds else ''))
    parser = SniffParser()
    pending, f.rxbuf = f.rxbuf, b''
    closed = None

    def take(data):
        nonlocal closed
        for item in parser.feed(data):
            if item[0] == 'frame':
                view.frame(*item[1:])
            elif item[1].startswith('OK sniff closed'):
                closed = item[1]
            else:
                view.say('# flasher: %s' % item[1])

    try:
        take(pending)
        while not seconds or time.time() - t_cmd < seconds:
            take(f.ser.read(max(1, f.ser.in_waiting)))
    except KeyboardInterrupt:
        pass
    # the escape needs a second without bytes from the host before it, as the flasher sees it: Wi-Fi may hold the
    # escape back longer than it held the answer, so half a second more
    while time.time() - t_cmd < 1.5:
        take(f.ser.read(max(1, f.ser.in_waiting)))
    f.ser.write(SNIFF_ESCAPE)
    deadline = time.time() + 10
    while closed is None and time.time() < deadline:
        take(f.ser.read(max(1, f.ser.in_waiting)))
    if closed is None:
        raise FlasherError('the flasher did not close the logger (no "OK sniff closed"); try bridge-exit')
    f.rxbuf = bytes(parser.buf)
    return closed


def sniff_main(f, args):
    if args.no_at_boot:
        f.cmd('SNIFFBOOT 0')
        print('Capture at power-up: off.')
        return
    if not args.sniff_baud:
        raise FlasherError('sniff: give the speed of line A (and --baud-b for line B when it differs)')
    a, b = args.sniff_baud, args.baud_b or args.sniff_baud
    if args.at_boot:
        print('Capture at power-up: %s' % f.cmd('SNIFFBOOT %d %d' % (a, b)).replace('sniffboot ', ''))
        print('From the next power-up the flasher listens on its own (power bank, nothing on its USB port). The first')
        print('command of a host stops it (this script too); then read the log with: log [--tuya]. The log is in RAM:')
        print('it is lost when the flasher loses power. Off again with: sniff --no-at-boot')
        return
    out = open(args.out, 'w', encoding='utf-8') if args.out else None
    try:
        view = SniffView(tuya=args.tuya, out=out)
        sniff_live(f, a, b, args.seconds, view)
    finally:
        if out:
            out.close()
    print('Closed. Line A %d bytes, line B %d bytes%s; all of it is in the UART log (log [--tuya]).' % (
        view.bytes[2], view.bytes[1], ', %d bytes dropped from the live output (the host was too slow)' %
        view.dropped if view.dropped else ''))


# ================================================================ ESP32 / ESP8266 over the UART bridge
# The flasher restarts the target with its GPIO 0 (the BOOT wire) held low, which starts the ROM's serial download
# mode, and esptool talks to the ROM through the bridge, never resetting the target itself. The C3's USB port ignores
# the host's speed setting, so the bridge cannot see esptool change its port's speed. From firmware 0.25 the bridge
# follows esptool instead ("BRIDGE 115200 lo esp"): esptool connects at 115200, loads its stub and asks for the
# speed of --esp-baud with CHANGE_BAUDRATE; the bridge reads that command and switches its own UART once the target
# has answered. Older firmware: the bridge runs at --esp-baud from the start and the ROM finds that speed from
# esptool's first sync. When a run fails, it is repeated at the next lower speed (ESP_SPEEDS), with the target
# restarted in download mode. At the end the bridge is closed and the target restarted with GPIO 0 released, so it
# runs its firmware.

ESP_SPEEDS = (921600, 460800, 115200)
ESP_ROM_BAUD = 115200

def _exit_answer(s):
    """After the escape: the bridge's or the logger's closing line; target bytes or logger frames may come first."""
    data = b''
    deadline = time.time() + 3
    while time.time() < deadline:
        data += s.read(max(1, s.in_waiting))
        m = re.search(rb'OK (?:bridge|sniff) closed[^\n]*\n', data)
        if m:
            return m.group(0).decode().strip()
    return data.split(b'\n')[0][:200].decode(errors='replace').strip()


def bridge_exit(port, key=None):
    """Leave the bridge or the two-line logger: neither answers commands, so send the escape after a pause."""
    if is_tcp(port):
        with tcp_open(port, key, timeout=0.2) as s:
            time.sleep(1.2)
            s.reset_input_buffer()                 # the target's bytes that waited for this connection
            s.write(b'~~~X')
            return _exit_answer(s)
    with serial.Serial(port, 115200, timeout=0.2, dsrdtr=False, rtscts=False) as s:
        time.sleep(1.2)
        s.write(b'~~~X')
        return _exit_answer(s)


def esptool_commands(op_args, major):
    """The esptool runs for one of our operations: a list of argument lists (rf --twice reads twice)."""
    d = '-' if major >= 5 else '_'
    op = op_args.op
    if op == 'info':
        return [['flash' + d + 'id']]
    if op == 'rf':
        if op_args.size == 0 and major < 5:
            raise FlasherError('rf with size 0 (the whole flash) needs esptool 5; give the size, e.g. 0x400000')
        size = 'ALL' if op_args.size == 0 else hex(op_args.size)
        runs = [['read' + d + 'flash', hex(op_args.address), size, op_args.filename]]
        if op_args.twice:
            runs.append(['read' + d + 'flash', hex(op_args.address), size, op_args.filename + '.2'])
        return runs
    if op == 'wf':
        # esptool erases what it writes and checks the MD5 of the result
        return [['write' + d + 'flash', hex(op_args.address), op_args.filename]]
    if op == 'es':
        start = op_args.address - op_args.address % SECTOR
        end = op_args.address + max(op_args.size, 1)
        end += -end % SECTOR
        return [['erase' + d + 'region', hex(start), hex(end - start)]]
    if op == 'ea':
        return [['erase' + d + 'flash']]
    raise FlasherError('%s is not available with --esp (info, rf, wf, es, ea)' % op)


def esptool_argv(port, baud, chip, major, run):
    d = '-' if major >= 5 else '_'
    argv = ['--port', port, '--baud', str(baud), '--before', 'no' + d + 'reset', '--after', 'no' + d + 'reset']
    if chip:
        argv += ['--chip', chip]
    return argv + run


def run_esptool(esptool, argv):
    print('esptool', ' '.join(argv))
    try:
        esptool.main(argv)
    except SystemExit as e:
        if e.code not in (None, 0):
            raise FlasherError('esptool stopped with exit code %s' % e.code)
    except Exception as e:  # esptool's own errors: FatalError, serial errors
        raise FlasherError('esptool: %s' % e)


def esp_speeds(first):
    """The speeds to try: the asked one, then the lower ones of ESP_SPEEDS."""
    return [first] + [b for b in ESP_SPEEDS if b < first]


def esp_main(f, args, follow=True):
    """follow: the firmware follows esptool's speed change (0.25 and newer)."""
    try:
        import esptool
    except ImportError:
        raise FlasherError('the ESP mode runs esptool: pip install esptool')
    try:
        major = int(str(esptool.__version__).split('.')[0])
    except (AttributeError, ValueError):
        major = 4
    runs = esptool_commands(args, major)
    key = getattr(args, 'key', None)
    baud = args.esp_baud or (ESP_SPEEDS[0] if follow else ESP_ROM_BAUD)
    speeds = esp_speeds(baud)
    error = None
    g = f
    for run in runs:
        while True:
            baud = speeds[0]
            if g is None:
                g = Flasher(args.port, key=key)
            # Restart the target with GPIO 0 low: ROM download mode. Every run starts from the ROM, as a stub left
            # running by the run before stays at its raised speed. Over Wi-Fi esptool gets the flasher's socket://
            # address and needs no key: the bridge was opened from this computer.
            if follow:
                print(g.cmd('BRIDGE %d lo esp' % ESP_ROM_BAUD))
            else:
                print(g.cmd('BRIDGE %d lo' % baud))
            g.ser.close()
            g = None
            time.sleep(0.1)
            try:
                run_esptool(esptool, esptool_argv(args.port, baud, args.esp_chip, major, run))
                error = None
            except FlasherError as e:
                error = e
            answer = bridge_exit(args.port, key)
            if not answer.startswith('OK'):
                print('Leaving the bridge: %s' % (answer or 'no answer'))
            if error is None or len(speeds) == 1:
                break
            print('%s at %d baud; trying again at %d baud' % (error, baud, speeds[1]))
            speeds = speeds[1:]
        if error is not None:
            break
    if error is None:
        print('esptool ran at %d baud.' % speeds[0])
    if error is None and args.op == 'rf' and args.twice:
        with open(args.filename, 'rb') as a, open(args.filename + '.2', 'rb') as b:
            same = a.read() == b.read()
        os.remove(args.filename + '.2')
        if not same:
            error = FlasherError('the two reads differ')
        else:
            print('Both reads are the same.')
    if not args.esp_stay:
        g = Flasher(args.port, key=key)
        try:
            g.cmd('RST')
        finally:
            g.ser.close()
        print('Target restarted with GPIO 0 released.')
    if error is not None:
        raise error


# ================================================================ Lightning LN882H over UART
# The flasher is a UART bridge (firmware 0.26: "BRIDGE 115200 lo ln") with the BOOT wire (GPIO 0) on the chip's BOOT pin
# (GPIOA9) held low, so the chip starts its ROM download mode. The ROM takes text commands at 115200 and loads a RAM
# loader ("ramcode", Lightning's, as ltchiptool ships it) over YMODEM. The loader reads and erases the flash on text
# commands and writes it from a YMODEM transfer; "baudrate N" raises its speed, which the bridge follows.

LN_ROM_BAUD = 115200
LN_SPEEDS = (1000000, 460800, 115200)
LN_RAM_ADDR = 0x20000000
LN_READ_CHUNK = 256                      # the loader's flash_read takes at most 256 bytes
LN_RAMCODE_NAME = 'ln882h_ramcode.bin'
LN_RAMCODE_SHA256 = '6bd437c6f8366b9cca0fb8de0c80c70788516e3681c6c43d654512feb7a0c723'
LN_RAMCODE_URL = ('https://github.com/libretiny-eu/ltchiptool/raw/410874f66b60da1dd667dbba69f499acbf16eb03/'
                  'ltchiptool/soc/ln882h/util/ramcode.bin')
# a line the bridge would take for the loader's speed change (src/lnscan.h); never sent inside a YMODEM transfer
LN_BAUD_LINE = re.compile(rb'[\r\n]baudrate [0-9]+[\r\n]')
LN_ROM_VERSION = re.compile(rb'[ -~]{11}/[0-9:]{8}')    # the ROM's build date, e.g. "Mar 14 2021/00:23:32"
YM_SOH, YM_EOT, YM_ACK, YM_NAK, YM_CAN, YM_C = 0x01, 0x04, 0x06, 0x15, 0x18, 0x43
BRIDGE_PAUSE = 1.2                       # the bridge takes its escape only after a pause of a second


def crc16_xmodem(data, crc=0):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1 ^ 0x1021) if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc


def ln_ramcode(path=''):
    """The LN882H RAM loader: --ln-ramcode, else ln882h_ramcode.bin next to this script, else the copy in an installed
    ltchiptool."""
    candidates = [path] if path else [os.path.join(os.path.dirname(os.path.abspath(__file__)), LN_RAMCODE_NAME)]
    if not path:
        try:
            import importlib.util
            spec = importlib.util.find_spec('ltchiptool')
            if spec and spec.origin:
                candidates.append(os.path.join(os.path.dirname(spec.origin), 'soc', 'ln882h', 'util', 'ramcode.bin'))
        except (ImportError, ValueError):
            pass
    for c in candidates:
        if os.path.isfile(c):
            with open(c, 'rb') as fh:
                data = fh.read()
            if hashlib.sha256(data).hexdigest() != LN_RAMCODE_SHA256:
                print('Note: %s is not the RAM loader build this script was checked against' % c)
            if LN_BAUD_LINE.search(data):
                raise FlasherError('%s holds a "baudrate N" line, which the bridge would take for a speed change' % c)
            return data
    raise FlasherError('the LN882H RAM loader is missing: download it from\n  %s\nas %s next to tlsr_c3.py (or give '
                       '--ln-ramcode, or pip install ltchiptool)' % (LN_RAMCODE_URL, LN_RAMCODE_NAME))


class LnLink:
    """The LN882H's ROM, then its RAM loader, through the flasher's UART bridge (ser is the flasher's port)."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = b''
        self.ramcode = False

    def write(self, data):
        self.ser.write(data)

    def _more(self):
        chunk = self.ser.read(max(1, self.ser.in_waiting))
        self.buf += chunk
        return bool(chunk)

    def wait_for(self, pattern, timeout):
        """Reads until the regex pattern is in what came; returns the match (the buffer keeps what follows it)."""
        deadline = time.time() + timeout
        while True:
            m = pattern.search(self.buf)
            if m:
                self.buf = self.buf[m.end():]
                return m
            if not self._more() and time.time() >= deadline:
                return None

    def quiet(self, gap=0.15, limit=2.0):
        """Reads until nothing came for gap seconds; returns it and empties the buffer."""
        deadline = time.time() + limit
        last = time.time()
        while time.time() < deadline:
            if self._more():
                last = time.time()
            elif time.time() - last >= gap:
                break
        out, self.buf = self.buf, b''
        return out

    def command(self, text, expect, timeout=2.0, end=b'\r\n'):
        """Sends a command line; returns the match of expect (a regex) in the answer, None when it did not come."""
        self.buf = b''
        self.write(text.encode() + end)
        return self.wait_for(expect, timeout)

    def link(self, timeout):
        """version until the ROM (its build date) or the loader (RAMCODE) answers; returns the answer."""
        deadline = time.time() + timeout
        while True:
            m = self.command('version', re.compile(LN_ROM_VERSION.pattern + rb'|RAMCODE'), timeout=0.4)
            if m:
                self.ramcode = m.group(0) == b'RAMCODE'
                return m.group(0).decode()
            if time.time() >= deadline:
                return None

    def ymodem_send(self, name, data, first_ack=10.0, ack=10.0, progress=None):
        """YMODEM with 128-byte packets and CRC-16, the receiver starting with 'C'. first_ack: the wait for the
        answer to the header packet (the loader erases the whole range before it answers)."""
        def packet(seq, payload):
            payload = payload.ljust(128, b'\x1a' if seq else b'\0')
            return bytes([YM_SOH, seq & 0xFF, 0xFF - (seq & 0xFF)]) + payload + crc16_xmodem(payload).to_bytes(2, 'big')

        def answer(timeout, wanted=(YM_ACK, YM_NAK, YM_CAN)):
            deadline = time.time() + timeout
            while True:
                for i, b in enumerate(self.buf):
                    if b in wanted:
                        self.buf = self.buf[i + 1:]
                        return b
                self.buf = b''
                if not self._more() and time.time() >= deadline:
                    return None

        def send(seq, payload, timeout):
            for _ in range(10):
                self.write(packet(seq, payload))
                a = answer(timeout)
                if a == YM_ACK:
                    return
                if a == YM_CAN:
                    raise FlasherError('YMODEM: the receiver cancelled at packet %d' % seq)
            raise FlasherError('YMODEM: no ACK for packet %d' % seq)

        if answer(5.0, (YM_C,)) is None:
            raise FlasherError('YMODEM: the receiver did not start (no "C")')
        send(0, name.encode() + b'\0' + str(len(data)).encode() + b'\0', first_ack)
        answer(2.0, (YM_C,))                         # the receiver asks for the data
        for k, pos in enumerate(range(0, len(data), 128)):
            send(k + 1, data[pos:pos + 128], ack)
            if progress:
                progress(min(pos + 128, len(data)))
        for _ in range(10):                          # EOT: the loader answers the first with NAK, the second with ACK
            self.write(bytes([YM_EOT]))
            if answer(1.0) == YM_ACK:
                break
        else:
            raise FlasherError('YMODEM: no ACK for EOT')
        answer(1.0, (YM_C,))
        self.write(packet(0, b''))                   # the empty header: end of the batch
        answer(1.0, (YM_ACK,))

    def set_baud(self, baud):
        self.buf = b''
        self.write(b'baudrate %d\r' % baud)          # only CR: the bridge switches at the line end (src/lnscan.h)
        time.sleep(0.05)
        self.quiet(0.05, 0.3)

    def flash_read(self, addr, n):
        """n <= 256 bytes; the loader answers in hex with a CRC-16 (XMODEM) of the data at the end."""
        text = 'flash_read 0x%X 0x%X' % (addr, n)
        for attempt in range(3):
            self.buf = b''
            self.write(text.encode() + b'\r\n')
            if self.wait_for(re.compile(re.escape(text.encode())), 1.0) is None:
                continue
            want = 2 * (n + 2)
            deadline = time.time() + 1.0
            hexd = b''
            while True:
                hexd = re.sub(rb'\s', b'', self.buf)
                if len(hexd) >= want or (not self._more() and time.time() >= deadline):
                    break
            if len(hexd) < want or not re.fullmatch(rb'[0-9A-Fa-f]+', hexd[:want]):
                continue
            raw = bytes.fromhex(hexd[:want].decode())
            if crc16_xmodem(raw[:n]) == int.from_bytes(raw[n:], 'big'):
                self.buf = b''
                return raw[:n]
        raise FlasherError('LN882H: flash_read at 0x%x: no valid answer' % addr)


class LnTarget:
    def __init__(self):
        self.rom = ''
        self.flash_id = 0
        self.size = 0
        self.uid = ''
        self.mac = ''
        self.baud = LN_ROM_BAUD


def ln_leave_bridge(f):
    time.sleep(BRIDGE_PAUSE)
    f.ser.reset_input_buffer()
    f.rxbuf = b''
    f.ser.write(b'~~~X')
    return f.readline(3).decode(errors='replace').strip()


def ln_connect(f, args, baud, say=print):
    """Bridge on, ROM linked, loader loaded and linked, speed raised to baud: (LnLink, LnTarget)."""
    say(f.cmd('BRIDGE %d lo ln' % LN_ROM_BAUD))
    link = LnLink(f.ser)
    t = LnTarget()
    answer = link.link(args.ln_seconds)
    if answer is None:
        raise FlasherError('LN882H: no answer from the ROM. Check TX0 (A2) on the SWS wire, RX0 (A3) on GPIO 21, BOOT '
                           '(A9) on GPIO 0, CEN on the reset wire, 3V3 on the power pins')
    if not link.ramcode:
        t.rom = answer
        ramcode = ln_ramcode(args.ln_ramcode)
        say('ROM:     %s; loading the RAM loader (%d bytes)' % (answer, len(ramcode)))
        link.buf = b''
        link.write(b'download [rambin] [0x%X] [%d]\r\n' % (LN_RAM_ADDR, len(ramcode)))
        link.ymodem_send('ramcode.bin', ramcode, first_ack=5.0, ack=3.0)
        if link.link(8.0) != 'RAMCODE':
            raise FlasherError('LN882H: the RAM loader does not answer after loading')
    if baud != LN_ROM_BAUD:
        link.set_baud(baud)
        if link.link(1.5) != 'RAMCODE':
            raise FlasherError('LN882H: no answer at %d baud' % baud)
    t.baud = baud
    m = link.command('flash_info', re.compile(rb'id:0x([0-9A-Fa-f]+),flash size:(\d+)M'))
    if not m:
        raise FlasherError('LN882H: no answer to flash_info')
    t.flash_id = int(m.group(1), 16)
    t.size = 1 << (t.flash_id & 0xFF) if 16 <= (t.flash_id & 0xFF) <= 25 else int(m.group(2)) << 20
    m = link.command('flash_uid', re.compile(rb'flash uid:0x([0-9A-Fa-f]+)'))
    t.uid = m.group(1).decode() if m else ''
    m = link.command('get_mac_in_flash_otp', re.compile(rb'(?<![0-9A-Fa-f:])(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}(?![0-9A-Fa-f:])'), timeout=0.5)
    t.mac = m.group(0).decode().upper() if m else ''
    link.quiet(0.05, 0.3)
    return link, t


def ln_read(link, addr, size, progress=True):
    out = bytearray()
    t0 = time.time()
    for pos in range(addr, addr + size, LN_READ_CHUNK):
        out += link.flash_read(pos, min(LN_READ_CHUNK, addr + size - pos))
        if progress and (len(out) % 0x10000 == 0 or len(out) == size):
            el = max(time.time() - t0, 1e-3)
            print('\r  read 0x%06x / 0x%06x (%.0f kB/s)' % (len(out), size, len(out) / el / 1000), end='', flush=True)
    if progress:
        print()
    return bytes(out)


def ln_image(args, t):
    with open(args.filename, 'rb') as fh:
        raw = fh.read()
    if not raw:
        raise FlasherError('empty file')
    if raw[:4] == b'UF2\n':
        parts = uf2_parts(raw, 'LN882H')
        if args.address not in (0, parts[0][0]):
            raise FlasherError('a UF2 says itself where it goes: give the address 0')
        return parts
    return [(args.address, raw)]


def ln_write(link, t, addr, data):
    """The loader erases the range itself when the YMODEM header comes and writes the data. It rounds the length up
    to 4 k but not the start: from an unaligned start it erases into the sector after the end. So the range sent is
    whole sectors: what is on the chip before the start and after the end in them is read first and sent along."""
    end = addr + len(data)
    if end > t.size:
        raise FlasherError('0x%x bytes at 0x%x: beyond the flash (%d kbytes)' % (len(data), addr, t.size // 1024))
    head, tail = addr % SECTOR, -end % SECTOR
    if head:
        data = ln_read(link, addr - head, head, progress=False) + data
        addr -= head
    if tail:
        data = data + ln_read(link, end, tail, progress=False)
    if LN_BAUD_LINE.search(data):
        raise FlasherError('the data holds a line "baudrate <digits>", which the bridge would take for the loader\'s '
                           'speed change in the middle of the transfer (src/lnscan.h); write it in two parts around it')
    m = link.command('startaddr 0x%X' % addr, re.compile(rb'pppp|ffff'))
    if not m or m.group(0) != b'pppp':
        raise FlasherError('LN882H: startaddr 0x%x %s' % (addr, 'not answered' if not m else 'refused'))
    link.buf = b''
    link.write(b'upgrade\r')                          # only CR: the loader is in YMODEM after it
    t0 = time.time()

    def progress(n):
        if n % 0x10000 < 128 or n == len(data):
            el = max(time.time() - t0, 1e-3)
            print('\r  write 0x%06x / 0x%06x (%.0f kB/s)' % (n, len(data), n / el / 1000), end='', flush=True)

    link.ymodem_send('image.bin', data, first_ack=10 + len(data) / 20000, ack=10.0, progress=progress)
    print()
    if link.link(3.0) != 'RAMCODE':
        raise FlasherError('LN882H: the loader does not answer after the write')


def ln_main(f, args):
    if args.op not in ('info', 'rf', 'wf', 'es', 'ea'):
        raise FlasherError('LN882H: info, rf, wf, es, ea')
    speeds = [args.ln_baud] if args.ln_baud else list(LN_SPEEDS)
    error = None
    for baud in speeds:
        try:
            link, t = ln_connect(f, args, baud)
            error = None
        except FlasherError as e:
            error = e
            answer = ln_leave_bridge(f)
            if not answer.startswith('OK'):
                print('Leaving the bridge: %s' % (answer or 'no answer'))
            if baud == speeds[-1] or 'no answer from the ROM' in str(e) or 'RAM loader is missing' in str(e):
                raise
            print('%s; trying again at %d baud' % (e, speeds[speeds.index(baud) + 1]))
            continue
        break
    try:
        ln_ops(link, t, args)
    finally:
        answer = ln_leave_bridge(f)
        if not answer.startswith('OK'):
            print('Leaving the bridge: %s' % (answer or 'no answer'))
    if args.run:
        f.cmd('RST')
        print('Target restarted with BOOT released.')


def ln_ops(link, t, args):
    if args.op == 'info':
        if t.rom:
            print('ROM:     %s' % t.rom)
        print('Chip:    LN882H, RAM loader at %d baud' % t.baud)
        print('Flash:   id 0x%06X, %d kbytes%s' % (t.flash_id, t.size // 1024, ', uid ' + t.uid if t.uid else ''))
        if t.mac:
            print('MAC:     %s (OTP)' % t.mac)
    elif args.op == 'rf':
        size = args.size or t.size - args.address
        if args.address + size > t.size:
            raise FlasherError('0x%x bytes at 0x%x: beyond the flash (%d kbytes)' % (size, args.address, t.size // 1024))
        os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
        t1 = time.time()
        data = ln_read(link, args.address, size)
        print('Read 0x%x bytes in %.1f s at %d baud; every 256 bytes checked against the loader\'s CRC' % (
            size, time.time() - t1, t.baud))
        if args.twice:
            if ln_read(link, args.address, size) != data:
                with open(args.filename + '.pass1', 'wb') as fh:
                    fh.write(data)
                raise FlasherError('the two reads differ; the first was kept as %s.pass1' % args.filename)
            print('Second read is the same.')
        with open(args.filename, 'wb') as fh:
            fh.write(data)
        print('Saved %d bytes to %s' % (len(data), args.filename))
    elif args.op == 'wf':
        if args.no_erase:
            raise FlasherError('LN882H: the loader always erases what it writes (no --no-erase)')
        parts = ln_image(args, t)
        for addr, data in parts:
            print('Write 0x%06x..0x%06x' % (addr, addr + len(data)))
            ln_write(link, t, addr, data)
        if not args.no_verify:
            for addr, data in parts:
                back = ln_read(link, addr, len(data), progress=False)
                if back != data:
                    bad = next(i for i in range(len(data)) if back[i] != data[i])
                    raise FlasherError('verify failed at 0x%06x' % (addr + bad))
            print('Verified (read back).')
    elif args.op == 'es':
        start = args.address - args.address % SECTOR
        end = args.address + max(args.size, 1)
        end = min(end + (-end % SECTOR), t.size)
        m = link.command('ferase 0x%X 0x%X' % (start, end - start), re.compile(rb'pppp|ffff'),
                         timeout=5 + (end - start) / 20000)
        if not m or m.group(0) != b'pppp':
            raise FlasherError('LN882H: ferase failed (%s)' % ('no answer' if not m else 'refused'))
        print('Erased 0x%06x..0x%06x' % (start, end))
    elif args.op == 'ea':
        print('Erasing the whole flash (also the boot loader and the partition table; the ROM download mode stays)')
        m = link.command('ferase_all', re.compile(rb'pppp|ffff'), timeout=120)
        if not m or m.group(0) != b'pppp':
            raise FlasherError('LN882H: ferase_all failed (%s)' % ('no answer' if not m else 'refused'))
        print('Erased.')


# ================================================================ Realtek AmebaZ (RTL8710B) over UART
# The flasher is a UART bridge (firmware 0.27: "BRIDGE 1500000 z rtl") that pulls the chip's log TX (PA30, on the SWS
# wire) low across the restart, so the ROM starts its UART download mode on the log UART at 1500000 baud. The ROM
# speaks single-byte commands; while idle it sends NAK now and then ("loud" handshake), an ACK from the host quiets
# it. Flash is read in 1 k pieces each answered with ACK, and written over XMODEM (1 k packets that carry their
# address, with a sum checksum); the ROM erases what it writes. Chip id, flash id, MAC and an MD5 of the flash come
# from small programs run in the chip's RAM (from ltchiptool's AmbZCode), which print their result on the log UART
# at 115200; the bridge changes its speed for that with its speed escape. After the program the ROM is back in
# download mode at 1500000.

RTL_ROM_BAUD = 1500000
RTL_DIAG_BAUD = 115200
RTL_BAUD_TABLE = (110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 128000,
                  153600, 230400, 380400, 460800, 500000, 921600, 1000000, 1382400, 1444400, 1500000, 1843200,
                  2000000, 2100000, 2764800, 3000000, 3250000, 3692300, 3750000, 4000000, 6000000)
RTL_FLASH = 0x08000000
RTL_RAM = 0x10002000
RTL_DATA = 0x10003000
RTL_SYSTEM = 0x9000                      # system data: OTA2 address, OTA switch, flash and log UART settings
RTL_MARK = b'AmbZTool_Marker!'
RTL_ROM_BANNER = b'UARTIMG_Download'
RTL_CHIPS = {0xE0: 'RTL8710BL', 0xFF: 'RTL8710BN', 0xFE: 'RTL8710BU', 0xF6: 'RTL8710BX', 0xFB: 'RTL8710L0',
             0xFD: 'RTL8711BN', 0xFC: 'RTL8711BU'}
RTL_CMD_BAUD, RTL_CMD_XMODEM, RTL_CMD_ERASE, RTL_CMD_READ, RTL_CMD_STATUS = 0x05, 0x07, 0x17, 0x19, 0x21
NAK1, ACK1, CAN1, STX1, EOT1 = b'\x15', b'\x06', b'\x18', b'\x02', b'\x04'


def _le32(v):
    return (v & 0xFFFFFFFF).to_bytes(4, 'little')


class RtlCode:
    """Thumb code for the AmebaZ ROM, calling its functions: from ltchiptool (MIT, Kuba Szczodrzyński,
    ltchiptool/soc/ambz/util/ambzcode.py); the comments there have the assembly."""

    @staticmethod
    def greeting(delay_ms, text):
        return (b'\x05\x48\x03\x4b\x98\x47\x06\xa0\x04\x49\x02\x4b\x98\x47\x0f\xe0\x6d\x34\x00\x00\x49\xec\x00\x00'
                + _le32(delay_ms) + _le32(len(text)) + text.ljust(16, b'\0'))

    @staticmethod
    def download_mode():
        return (b'\x04\x4b\x18\x68\x04\x49\x08\x40\x18\x60\x02\x20\x00\x4b\x98\x47\x01\x09\x00\x00'
                b'\x10\x02\x00\x40\xff\xff\xff\xef')

    @staticmethod
    def flash_id(offset):
        return b'\x9f\x20\x03\x21\x02\x4a\x01\x4b\x98\x47\x03\xe0\x65\x74\x00\x00' + _le32(RTL_DATA + offset)

    @staticmethod
    def chip_id(offset):
        return (b'\x03\x48\xf8\x21\x04\x4a\x07\x23\x02\x4c\xa0\x47\x06\xe0\x00\x00\xae\x26\x00\x00\x65\x6d\x00\x00'
                + _le32(RTL_DATA + offset))

    @staticmethod
    def efuse_logical_map(offset):
        return b'\x02\x48\x01\x4b\x98\x47\x03\xe0\x91\x70\x00\x00' + _le32(RTL_DATA + offset)

    @staticmethod
    def md5(address, length, offset):
        return (b'\x0c\x48\x06\x4b\x98\x47\x0b\x48\x07\x49\x08\x4a\x04\x4b\x98\x47\x07\x48\x08\x49\x03\x4b\x98\x47'
                b'\x0e\xe0\x00\x00\xf5\x1d\x01\x00\x25\x1e\x01\x00\xc9\x1e\x01\x00'
                + _le32(address) + _le32(length) + _le32(RTL_DATA + offset) + _le32(RTL_DATA + offset + 16))

    @staticmethod
    def print_data(length, address=RTL_DATA):
        return b'\x03\x48\x04\x49\x01\x4b\x98\x47\x06\xe0\x00\x00\x49\xec\x00\x00' + _le32(address) + _le32(length)


class RtlLink:
    def __init__(self, f):
        self.f = f
        self.ser = f.ser
        self.buf = b''
        self.baud = RTL_ROM_BAUD

    def write(self, data):
        self.ser.write(data)

    def _more(self):
        chunk = self.ser.read(max(1, self.ser.in_waiting))
        self.buf += chunk
        return bool(chunk)

    def flush(self):
        self.ser.reset_input_buffer()
        self.buf = b''

    def read(self, n, timeout=1.0):
        deadline = time.time() + timeout
        while len(self.buf) < n:
            if not self._more() and time.time() >= deadline:
                break
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def gather(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            self._more()
        out, self.buf = self.buf, b''
        return out

    def loud(self):
        """Loud handshake: the ROM sends NAKs while it waits for a command."""
        self.flush()
        self.write(bytes([RTL_CMD_STATUS]))
        got = self.read(6, 1.0)
        if len(got) < 6 or got[-1:] != NAK1:
            raise FlasherError('RTL8710B: no NAK from the ROM\'s download mode (got %s)' % (got.hex() or 'nothing'))

    def link(self, timeout):
        """Waits for the ROM's NAKs; on silence it tries to bring the ROM back (CAN, XMODEM handshake, CAN)."""
        deadline = time.time() + timeout
        seen = b''
        while time.time() < deadline:
            seen = (seen + self.gather(0.3))[-64:]
            if seen.endswith(NAK1 * 4):
                self.loud()
                return True
            self.write(bytes([0x18, RTL_CMD_XMODEM, 0x18]))
        return False

    def quiet(self, limit=10.0):
        """Quiet handshake: ACK until the NAKs stop, so that a read is not mixed with them."""
        deadline = time.time() + limit
        while time.time() < deadline:
            self.flush()
            self.write(ACK1)
            if not self.gather(0.1):
                return
        raise FlasherError('RTL8710B: the ROM does not go quiet')

    def bridge_speed(self, baud):
        """The bridge's speed escape (firmware 0.27): a pause of a second, then ~~~S<baud>."""
        time.sleep(BRIDGE_PAUSE)
        self.write(b'~~~S%d\n' % baud)
        time.sleep(0.05)
        self.baud = baud

    def set_speed(self, baud):
        """The ROM's speed command, answered with ACK at the old speed; then the bridge follows."""
        self.flush()
        self.write(bytes([RTL_CMD_BAUD, RTL_BAUD_TABLE.index(baud)]))
        if self.read(1, 1.0) != ACK1:
            raise FlasherError('RTL8710B: no ACK for the speed change to %d' % baud)
        self.bridge_speed(baud)
        self.loud()

    def flash_read(self, addr, size, progress=None):
        """Whole 4 k sectors: addr and size multiples of 4096. 64 k per read command, every 1 k answered with ACK."""
        out = bytearray()
        while len(out) < size:
            n = min(0x10000, size - len(out))
            for attempt in range(3):
                try:
                    out += self._read_run(addr + len(out), n)
                    break
                except FlasherError:
                    if attempt == 2:
                        raise
                    time.sleep(0.2)
                    self.write(ACK1 * (n // 1024 + 1))   # let the ROM finish the command
                    time.sleep(0.2)
            if progress:
                progress(len(out))
        return bytes(out)

    def _read_run(self, addr, n):
        self.quiet()
        self.write(bytes([RTL_CMD_READ]) + addr.to_bytes(3, 'little') + (n // 4096).to_bytes(2, 'little'))
        data = bytearray()
        for _ in range(n // 1024):
            piece = self.read(1024, 2.0)
            if len(piece) != 1024:
                raise FlasherError('RTL8710B: read at 0x%x stopped after %d bytes' % (addr, len(data) + len(piece)))
            data += piece
            self.write(ACK1)
        self.write(ACK1)
        return bytes(data)

    def xmodem_write(self, addr, data, ram=False, progress=None):
        """XMODEM with 1 k packets that start with their 4-byte address and end with a sum checksum."""
        self.loud()
        self.flush()
        self.write(bytes([RTL_CMD_XMODEM]))
        if self.read(1, 1.0) != ACK1:
            raise FlasherError('RTL8710B: no ACK for the XMODEM handshake')
        seq = 1
        for pos in range(0, len(data), 1024):
            body = _le32(addr + pos) + data[pos:pos + 1024].ljust(1024, b'\xff')
            pkt = STX1 + bytes([seq, 0xFF - seq]) + body + bytes([sum(body) & 0xFF])
            for _ in range(10):
                self.flush()
                self.write(pkt)
                a = self.read(1, 3.0)
                if a == ACK1:
                    break
                if a == CAN1:
                    raise FlasherError('RTL8710B: the ROM cancelled the transfer at 0x%x' % (addr + pos))
            else:
                raise FlasherError('RTL8710B: no ACK for the packet at 0x%x' % (addr + pos))
            seq = (seq + 1) & 0xFF
            if progress:
                progress(pos + 1024)
        self.flush()
        self.write(EOT1)
        if ram:
            return                              # the ROM jumps into the program; its ACK does not come
        if self.read(1, 3.0) != ACK1:
            raise FlasherError('RTL8710B: no ACK for EOT')
        self.loud()

    def run_code(self, code, timeout=8.0):
        """Runs code in RAM and returns what it printed (between the marker and the ROM's banner)."""
        self.set_speed(RTL_DIAG_BAUD)
        code = RtlCode.greeting(400, RTL_MARK) + code + RtlCode.download_mode()
        table = [0, 0x1000219A + 1, 0x100021EE + 1, 0x100020F4 + 1, 0x100021EE + 1, 0x08000540 + 1]
        table[0] = (RTL_RAM + len(table) * 4) | 1
        self.xmodem_write(RTL_RAM, b''.join(_le32(x) for x in table) + code, ram=True)
        deadline = time.time() + timeout
        resp = b''
        while time.time() < deadline and RTL_ROM_BANNER not in resp:
            self._more()
            resp, self.buf = resp + self.buf, b''
        self.bridge_speed(RTL_ROM_BAUD)         # the ROM is back in download mode at its own speed
        if not self.link(5.0):
            raise FlasherError('RTL8710B: the ROM did not come back after the RAM program')
        if RTL_MARK not in resp:
            raise FlasherError('RTL8710B: the RAM program did not answer (%r)' % resp[:80])
        resp = resp.partition(RTL_MARK)[2]
        return resp.partition(RTL_ROM_BANNER)[0]

    def md5(self, addr, size):
        return self.run_code(RtlCode.md5(RTL_FLASH | addr, size, 0) + RtlCode.print_data(16))[:16]


class RtlTarget:
    def __init__(self):
        self.chip_id = 0
        self.chip = ''
        self.flash_id = b''
        self.size = 0x200000
        self.mac = ''


def rtl_connect(f, args):
    print(f.cmd('BRIDGE %d z rtl' % RTL_ROM_BAUD))
    link = RtlLink(f)
    if not link.link(args.rtl_seconds):
        raise FlasherError('RTL8710B: no answer from the ROM. Check the log TX (PA30) on the SWS wire, the log RX '
                           '(PA29) on GPIO 21, CEN on the reset wire, 3V3 on the power pins')
    t = RtlTarget()
    data = link.run_code(RtlCode.chip_id(0) + RtlCode.flash_id(1) + RtlCode.efuse_logical_map(16) +
                         RtlCode.print_data(16 + 512))
    if len(data) < 16 + 512:
        raise FlasherError('RTL8710B: short answer from the RAM program (%d bytes)' % len(data))
    t.chip_id = data[0]
    t.chip = RTL_CHIPS.get(t.chip_id, 'unknown 0x%02X' % t.chip_id)
    t.flash_id = data[1:4]
    if 0x14 <= t.flash_id[2] <= 0x19:
        t.size = 1 << t.flash_id[2]
    t.mac = ':'.join('%02X' % b for b in data[16 + 0x11A:16 + 0x120])
    return link, t


def rtl_read(link, t, addr, size, progress=True):
    """Any range: whole sectors are read, the MD5 of them is checked by the chip."""
    start = addr - addr % SECTOR
    end = addr + size
    end += -end % SECTOR
    t0 = time.time()

    def show(n):
        if progress:
            el = max(time.time() - t0, 1e-3)
            print('\r  read 0x%06x / 0x%06x (%.0f kB/s)' % (n, end - start, n / el / 1000), end='', flush=True)

    data = link.flash_read(start, end - start, show)
    if progress:
        print()
    if link.md5(start, end - start) != hashlib.md5(data).digest():
        raise FlasherError('RTL8710B: the MD5 of 0x%x..0x%x on the chip differs from what was read' % (start, end))
    return data[addr - start:addr - start + size]


def rtl_write(link, t, addr, data, verify=True):
    """The ROM erases what it writes; the range sent is whole sectors (the rest of the first and last ones is read
    first and sent along). Checked by the chip's MD5 of the range."""
    end = addr + len(data)
    if end > t.size:
        raise FlasherError('0x%x bytes at 0x%x: beyond the flash (%d kbytes)' % (len(data), addr, t.size // 1024))
    head, tail = addr % SECTOR, -end % SECTOR
    if head:
        data = link.flash_read(addr - head, SECTOR)[:head] + data
        addr -= head
    if tail:
        data = data + link.flash_read(end + tail - SECTOR, SECTOR)[SECTOR - tail:]
    t0 = time.time()

    def show(n):
        if n % 0x10000 == 0 or n >= len(data):
            el = max(time.time() - t0, 1e-3)
            print('\r  write 0x%06x / 0x%06x (%.0f kB/s)' % (min(n, len(data)), len(data), n / el / 1000), end='',
                  flush=True)

    link.xmodem_write(RTL_FLASH | addr, data, progress=show)
    print()
    if verify and link.md5(addr, len(data)) != hashlib.md5(data).digest():
        raise FlasherError('RTL8710B: verify failed: the MD5 of 0x%x..0x%x on the chip differs' % (addr, addr + len(data)))


def rtl_erase(link, start, end):
    for a in range(start, end, SECTOR):
        link.loud()
        link.flush()
        link.write(bytes([RTL_CMD_ERASE]) + a.to_bytes(3, 'little') + (1).to_bytes(2, 'little'))
        if link.read(1, 3.0) != ACK1:
            raise FlasherError('RTL8710B: no ACK for the erase of 0x%x' % a)


def rtl_image(link, t, args):
    with open(args.filename, 'rb') as fh:
        raw = fh.read()
    if not raw:
        raise FlasherError('empty file')
    if raw[:4] != b'UF2\n':
        return [(args.address, raw)]
    if args.address != 0:
        raise FlasherError('a UF2 says itself where it goes: give the address 0')
    # the image goes into the OTA slot the bootloader starts: OTA1 unless the switch word has an odd number of 0 bits
    sysdata = link.flash_read(RTL_SYSTEM, SECTOR)
    ota2_addr, switch = int.from_bytes(sysdata[0:4], 'little'), int.from_bytes(sysdata[4:8], 'little')
    slot = 1 + bin(switch ^ 0xFFFFFFFF).count('1') % 2
    parts = uf2_parts(raw, 'RTL8710B', scheme=3 + slot)
    print('OTA:     slot %d is the one that starts (switch 0x%08x, OTA2 at 0x%x)' % (slot, switch, ota2_addr & 0xFFFFFF))
    if slot == 2 and parts and (ota2_addr & 0xFFFFFF) != parts[0][0]:
        raise FlasherError('the chip starts OTA2 at 0x%x, the UF2 puts it at 0x%x; write a full image instead' % (
            ota2_addr & 0xFFFFFF, parts[0][0]))
    return parts


def rtl_main(f, args):
    if args.op not in ('info', 'rf', 'wf', 'es', 'ea'):
        raise FlasherError('RTL8710B: info, rf, wf, es, ea')
    try:
        link, t = rtl_connect(f, args)
        rtl_ops(link, t, args)
    finally:
        answer = ln_leave_bridge(f)
        if not answer.startswith('OK'):
            print('Leaving the bridge: %s' % (answer or 'no answer'))
    if args.run:
        f.cmd('RST')
        print('Target restarted with its TX released.')


def rtl_ops(link, t, args):
    if args.op == 'info':
        print('Chip:    %s (eFuse 0xF8 = 0x%02X)' % (t.chip, t.chip_id))
        print('Flash:   id %s, %d kbytes' % (t.flash_id.hex().upper(), t.size // 1024))
        print('MAC:     %s (eFuse)' % t.mac)
        sysdata = link.flash_read(RTL_SYSTEM, SECTOR)
        ota2, switch = int.from_bytes(sysdata[0:4], 'little'), int.from_bytes(sysdata[4:8], 'little')
        print('OTA:     OTA2 at 0x%x, switch 0x%08x: slot %d starts' % (
            ota2 & 0xFFFFFF, switch, 1 + bin(switch ^ 0xFFFFFFFF).count('1') % 2))
    elif args.op == 'rf':
        size = args.size or t.size - args.address
        if args.address + size > t.size:
            raise FlasherError('0x%x bytes at 0x%x: beyond the flash (%d kbytes)' % (size, args.address, t.size // 1024))
        os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
        t1 = time.time()
        data = rtl_read(link, t, args.address, size)
        print('Read 0x%x bytes in %.1f s; checked against the MD5 computed by the chip' % (size, time.time() - t1))
        if args.twice:
            if rtl_read(link, t, args.address, size) != data:
                with open(args.filename + '.pass1', 'wb') as fh:
                    fh.write(data)
                raise FlasherError('the two reads differ; the first was kept as %s.pass1' % args.filename)
            print('Second read is the same.')
        with open(args.filename, 'wb') as fh:
            fh.write(data)
        print('Saved %d bytes to %s' % (len(data), args.filename))
    elif args.op == 'wf':
        if args.no_erase:
            raise FlasherError('RTL8710B: the ROM always erases what it writes (no --no-erase)')
        for addr, data in rtl_image(link, t, args):
            print('Write 0x%06x..0x%06x' % (addr, addr + len(data)))
            rtl_write(link, t, addr, data, verify=not args.no_verify)
        if not args.no_verify:
            print('Verified (MD5 on the chip).')
    elif args.op == 'es':
        start = args.address - args.address % SECTOR
        end = args.address + max(args.size, 1)
        end = min(end + (-end % SECTOR), t.size)
        rtl_erase(link, start, end)
        print('Erased 0x%06x..0x%06x' % (start, end))
    elif args.op == 'ea':
        print('Erasing 0x0..0x%x sector by sector (also the boot loader and the system data; the ROM download '
              'mode stays)' % t.size)
        rtl_erase(link, 0, t.size)
        print('Erased.')


# ================================================================ Realtek AmebaZ2 (RTL8720C) over UART
# The flasher is a UART bridge (firmware 0.28: "BRIDGE 115200 hi ucfg") with the BOOT wire (GPIO 0) high on the chip's
# PA00, so its ROM starts the download console on the log UART (PA16 TX, PA15 RX) at 115200. The console takes text
# commands: ping, ucfg <baud> (speed; the bridge follows the line, src/lnscan.h), DW / DB (dump words / bytes as text;
# the flash is mapped at 0x98000000 once it is set up), EW (write words), fwd <flash settings> <offset> (an XMODEM
# transfer written to the flash, erasing what it writes; it also sets where hashq starts), hashq <length> (SHA-256).
# Chip type and MAC come from the eFuse, read by a small RAM program (ltchiptool's) started through an unused ROM
# console command whose handler is pointed at it.

RTL2_BAUD = 115200
RTL2_SPEEDS = (1500000, 921600, 115200)
RTL2_FLASH_MAP = 0x98000000
RTL2_CODE = 0x10037000
RTL2_DATA = 0x10038000
RTL2_USED = ('ping', 'disc', 'ucfg', 'DW', 'DB', 'EW', 'EB', 'WDTRST', 'hashq', 'fwd', 'fwdram')
RTL2_CHIPS = {0xFE: 'RTL87x0CF', 0xFD: 'RTL87x0CM'}
RTL2_UCFG_LINE = re.compile(rb'[\r\n]ucfg [0-9]{1,9}(?: [ -~]*)?[\r\n]')
# eFuse read program (ltchiptool, MIT, ltchiptool/soc/ambz2/util/ambz2code.py): 512 bytes into RTL2_DATA
RTL2_EFUSE_CODE = (b'\x38\xb5\x00\x25\x07\x48\x29\x00\x09\x4a\x2a\x44\xaa\x23\x13\x70\x00\x23\x05\x4c\x08\x34\x24\x68'
                   b'\xa0\x47\x04\x4b\x6d\x1c\x9d\x42\xf0\xd1\x38\xbd\x00\x00\x30\x33\x00\x05\x00\x00\x00\x02\x00\x00'
                   b'\x00\x80\x03\x10')


def efuse_logical(phys):
    """Realtek eFuse physical map to logical map (ltchiptool, MIT, ltchiptool/soc/amb/efuse.py)."""
    logical = bytearray(b'\xff' * 512)
    pos = 0

    def take():
        nonlocal pos
        b = phys[pos] if pos < len(phys) else 0xFF
        pos += 1
        return b

    def copy(cell, word_map):
        a = 8 * cell
        for i in range(4):
            if not word_map & (1 << i):
                logical[a] = take()
                logical[a + 1] = take()
            a += 2

    while pos < len(phys):
        h = take()
        if h == 0xFF:
            break
        if h & 0x1F != 0xF:
            copy(h >> 4, h & 0xF)
            continue
        e = take()
        if e & 0xF == 0xF:
            continue
        if e & 0x80:
            for bit in (1, 2, 4, 8):
                if not e & bit:
                    take()
                    take()
            continue
        copy((h >> 5) | ((e & 0xF0) >> 1), e & 0xF)
    return bytes(logical)


class Rtl2Link:
    def __init__(self, ser):
        self.ser = ser
        self.buf = b''
        self.cfg = '0 1'                     # flash speed (0: single IO) and pin mode

    def write(self, data):
        self.ser.write(data)

    def _more(self):
        chunk = self.ser.read(max(1, self.ser.in_waiting))
        self.buf += chunk
        return bool(chunk)

    def flush(self):
        self.ser.reset_input_buffer()
        self.buf = b''

    def read(self, n, timeout):
        deadline = time.time() + timeout
        while len(self.buf) < n:
            if not self._more() and time.time() >= deadline:
                break
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def command(self, text):
        self.flush()
        self.write(text.encode() + b'\n')

    def lines(self, timeout):
        """Yields text lines until nothing came for timeout seconds."""
        deadline = time.time() + timeout
        while True:
            while b'\n' in self.buf:
                line, _, self.buf = self.buf.partition(b'\n')
                line = line.strip()
                if line:
                    deadline = time.time() + timeout
                    yield line.decode(errors='replace')
            if not self._more() and time.time() >= deadline:
                return

    def ping(self):
        self.command('ping')
        got = self.read(4, 0.5)
        tail = self.read(64, 0.05)
        return got == b'ping' and b'$8710c' not in tail

    def link(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.ping():
                return True
            if b'$8710c' in self.buf:
                break
        return False

    def dump_words(self, addr, count):
        self.command('DW %X %d' % (addr, count))
        words = []
        for line in self.lines(1.0):
            f = line.split()
            if f[0].rstrip(':') == '[Addr]':
                continue
            if int(f[0].rstrip(':'), 16) != addr + 4 * len(words):
                raise FlasherError('RTL8720C: DW answered for another address: %s' % line)
            words += [int(x, 16) for x in f[1:5]]
            if len(words) >= count:
                return words[:count]
        raise FlasherError('RTL8720C: DW %X: short answer' % addr)

    def dump_bytes(self, addr, count, timeout=3.0):
        self.command('DB %X %d' % (addr, count))
        out = bytearray()
        for line in self.lines(timeout):
            f = line.split()
            if f[0] == '[Addr]':
                continue
            if int(f[0].rstrip(':'), 16) != addr + len(out) or len(f) < 17:
                raise FlasherError('RTL8720C: DB: unexpected line %r' % line)
            out += bytes(int(x, 16) for x in f[1:17])
            if len(out) >= count:
                return bytes(out[:count])
        raise FlasherError('RTL8720C: DB %X: short answer (%d of %d bytes)' % (addr, len(out), count))

    def write_words(self, addr, data):
        data = data + b'\0' * (-len(data) % 4)
        words = ['%X' % int.from_bytes(data[i:i + 4], 'little') for i in range(0, len(data), 4)]
        for i in range(0, len(words), 7):              # EW takes up to about 80 characters
            chunk = words[i:i + 7]
            self.command('EW %X %s' % (addr + 4 * i, ' '.join(chunk)))
            got = list(itertools.islice(self.lines(0.5), len(chunk)))
            if len(got) < len(chunk):
                raise FlasherError('RTL8720C: EW %X: no answer' % (addr + 4 * i))

    def set_speed(self, baud):
        self.command('ucfg %d 0 0' % baud)            # the bridge switches at the line end, OK comes at the new speed
        self.read(2, 1.0)                              # "OK"; a part of it may be lost in the switch
        if not self.link(2.0):
            raise FlasherError('RTL8720C: no answer at %d baud' % baud)

    def flash_init(self):
        mode = (self.dump_words(0x40000038, 1)[0] >> 5) & 3
        self.write_words(0x40002800, (0x7EFFFFFF).to_bytes(4, 'little'))
        self.cfg = '0 %d' % mode
        self.hashq(0)                                  # sets the flash up in the ROM
        return mode

    def hashq(self, length):
        self.command('hashq %d %s' % (length, self.cfg))
        got = self.read(6 + 32, 5 + length / 150000)
        if not got.startswith(b'hashs ') or len(got) < 38:
            raise FlasherError('RTL8720C: hashq: unexpected answer %r' % got[:40])
        return got[6:38]

    def _xmodem(self, data, progress=None):
        """XMODEM-1K as the receiver asks: NAK = sum checksum, 'C' = CRC-16."""
        start = self.read(1, 4.0)
        if start not in (b'\x15', b'C'):
            raise FlasherError('RTL8720C: XMODEM did not start (got %r)' % start)
        crc = start == b'C'
        seq = 1
        for pos in range(0, len(data), 1024):
            body = data[pos:pos + 1024].ljust(1024, b'\xff')
            tail = crc16_xmodem(body).to_bytes(2, 'big') if crc else bytes([sum(body) & 0xFF])
            pkt = b'\x02' + bytes([seq, 0xFF - seq]) + body + tail
            for _ in range(10):
                self.flush()
                self.write(pkt)
                a = self.read(1, 3.0)
                if a == b'\x06':
                    break
                if a == b'\x18':
                    raise FlasherError('RTL8720C: the ROM cancelled the transfer at +0x%x' % pos)
            else:
                raise FlasherError('RTL8720C: no ACK for the packet at +0x%x' % pos)
            seq = (seq + 1) & 0xFF
            if progress:
                progress(pos + 1024)
        for _ in range(5):
            self.flush()
            self.write(b'\x04')
            if self.read(1, 3.0) == b'\x06':
                return
        raise FlasherError('RTL8720C: no ACK for EOT')

    def fwd(self, offset, data, progress=None):
        """Writes data (whole sectors) to the flash at offset; afterwards hashq starts there."""
        self.command('fwd %s %x' % (self.cfg, offset))
        self._xmodem(data, progress)
        if not self.link(3.0):
            raise FlasherError('RTL8720C: no answer after the write')

    def hash_from(self, offset):
        """Sets where hashq starts: fwd with an empty, cancelled transfer."""
        self.command('fwd %s %x' % (self.cfg, offset))
        if self.read(1, 4.0) not in (b'\x15', b'C'):
            raise FlasherError('RTL8720C: fwd did not start')
        self.write(b'\x18\x18')
        got = self.read(3, 2.0)
        if got != b'\x18ER':
            raise FlasherError('RTL8720C: unexpected answer to the cancelled fwd: %r' % got)
        if not self.link(3.0):
            raise FlasherError('RTL8720C: no answer after the cancelled fwd')

    def run_code(self, code, addr=RTL2_CODE):
        """Loads code with EW and runs it through an unused ROM console command (its handler pointed at it)."""
        self.write_words(addr, code)
        table = self.dump_words(0x1002F054, 1)[0]
        target = None
        for entry in range(table, table + 8 * 12, 12):
            name_ptr = self.dump_words(entry, 1)[0]
            if name_ptr == 0:
                break
            name = self.dump_bytes(name_ptr, 16).partition(b'\0')[0]
            if not name or not name.isascii() or name.decode() in RTL2_USED:
                continue
            target = entry + 4, name.decode()
        if target is None:
            raise FlasherError('RTL8720C: no unused ROM console command to start a RAM program with')
        self.write_words(target[0], ((addr | 1)).to_bytes(4, 'little'))
        self.command(target[1])
        time.sleep(0.2)
        if not self.link(3.0):
            raise FlasherError('RTL8720C: no answer after the RAM program')


class Rtl2Target:
    def __init__(self):
        self.chip = ''
        self.chip_id = 0
        self.mode = 1
        self.size = 0x200000
        self.mac = self.mac_bt = ''
        self.version = 0
        self.baud = RTL2_BAUD


def rtl2_connect(f, args, baud):
    print(f.cmd('BRIDGE %d hi ucfg' % RTL2_BAUD))
    link = Rtl2Link(f.ser)
    time.sleep(0.2)
    if not link.link(args.rtl_seconds):
        if b'$8710c' in link.buf:
            raise FlasherError('RTL8720C: the ROM console answered, not the download mode: is PA00 on the BOOT wire '
                               '(GPIO 0, held high)?')
        raise FlasherError('RTL8720C: no answer from the ROM. Check the log TX (PA16) on the SWS wire, the log RX '
                           '(PA15) on GPIO 21, PA00 on GPIO 0 (held high), CEN on the reset wire, 3V3 on the power '
                           'pins; PA13 (RX0) must not be pulled low')
    t = Rtl2Target()
    if baud != RTL2_BAUD:
        link.set_speed(baud)
    t.baud = baud
    t.mode = link.flash_init()
    t.size = 0x200000 if t.mode == 1 else 0x400000
    t.version = (link.dump_words(0x400001F0, 1)[0] >> 4) & 0xF
    return link, t


def rtl2_efuse(link, t):
    link.run_code(RTL2_EFUSE_CODE)
    phys = link.dump_bytes(RTL2_DATA, 512)
    logi = efuse_logical(phys)
    t.chip_id = phys[0x1F8]
    t.chip = RTL2_CHIPS.get(t.chip_id, 'unknown 0x%02X' % t.chip_id)
    t.mac = ':'.join('%02X' % b for b in logi[0x11A:0x120])
    t.mac_bt = ':'.join('%02X' % b for b in logi[0x190:0x196])


def rtl2_read(link, t, addr, size, progress=True):
    out = bytearray()
    t0 = time.time()
    chunk = 4096
    for pos in range(addr, addr + size, chunk):
        n = min(chunk, addr + size - pos)
        for attempt in range(3):
            try:
                out += link.dump_bytes(RTL2_FLASH_MAP | pos, n, timeout=1.0 + 2e5 / t.baud)
                break
            except (FlasherError, ValueError):
                if attempt == 2:
                    raise
                link.read(1 << 20, 0.5)               # let the rest of the dump pass
        if progress and (len(out) % 0x10000 == 0 or len(out) == size):
            el = max(time.time() - t0, 1e-3)
            print('\r  read 0x%06x / 0x%06x (%.0f kB/s)' % (len(out), size, len(out) / el / 1000), end='', flush=True)
    if progress:
        print()
    link.hash_from(addr)
    if link.hashq(size) != hashlib.sha256(out).digest():
        raise FlasherError('RTL8720C: the SHA-256 of 0x%x..0x%x on the chip differs from what was read' % (
            addr, addr + size))
    return bytes(out)


def rtl2_write(link, t, addr, data, verify=True):
    """fwd erases what it writes; whole sectors are sent (the rest of the first and last ones read first)."""
    end = addr + len(data)
    if end > t.size:
        raise FlasherError('0x%x bytes at 0x%x: beyond the flash (%d kbytes)' % (len(data), addr, t.size // 1024))
    head, tail = addr % SECTOR, -end % SECTOR
    if head:
        data = link.dump_bytes(RTL2_FLASH_MAP | (addr - head), head) + data
        addr -= head
    if tail:
        data = data + link.dump_bytes(RTL2_FLASH_MAP | end, tail)
    if RTL2_UCFG_LINE.search(data):
        raise FlasherError('the data holds a line "ucfg <digits>", which the bridge would take for the ROM\'s speed '
                           'change in the middle of the transfer (src/lnscan.h); write it in two parts around it')
    t0 = time.time()

    def show(n):
        if n % 0x10000 == 0 or n >= len(data):
            el = max(time.time() - t0, 1e-3)
            print('\r  write 0x%06x / 0x%06x (%.0f kB/s)' % (min(n, len(data)), len(data), n / el / 1000), end='',
                  flush=True)

    link.fwd(addr, data, show)
    print()
    if verify and link.hashq(len(data)) != hashlib.sha256(data).digest():
        raise FlasherError('RTL8720C: verify failed: the SHA-256 of 0x%x..0x%x on the chip differs' % (
            addr, addr + len(data)))


def rtl2_image(args):
    with open(args.filename, 'rb') as fh:
        raw = fh.read()
    if not raw:
        raise FlasherError('empty file')
    if raw[:4] != b'UF2\n':
        return [(args.address, raw)]
    if args.address != 0:
        raise FlasherError('a UF2 says itself where it goes: give the address 0')
    return uf2_parts(raw, 'RTL8720C', scheme=4)     # always OTA1, as ltchiptool does


def rtl2_main(f, args):
    if args.op not in ('info', 'rf', 'wf', 'es', 'ea'):
        raise FlasherError('RTL8720C: info, rf, wf, es, ea')
    speeds = [args.rtl_baud] if args.rtl_baud else list(RTL2_SPEEDS)
    link = t = None
    for baud in speeds:
        try:
            link, t = rtl2_connect(f, args, baud)
            break
        except FlasherError as e:
            answer = ln_leave_bridge(f)
            if not answer.startswith('OK'):
                print('Leaving the bridge: %s' % (answer or 'no answer'))
            if baud == speeds[-1] or 'no answer from the ROM' in str(e) or 'ROM console' in str(e):
                raise
            print('%s; trying again at %d baud' % (e, speeds[speeds.index(baud) + 1]))
    try:
        rtl2_ops(link, t, args)
    finally:
        answer = ln_leave_bridge(f)
        if not answer.startswith('OK'):
            print('Leaving the bridge: %s' % (answer or 'no answer'))
    if args.run:
        f.cmd('RST')
        print('Target restarted with PA00 released.')


def rtl2_ops(link, t, args):
    if args.op == 'info':
        rtl2_efuse(link, t)
        print('Chip:    %s (eFuse 0x1F8 = 0x%02X), ROM %s' % (t.chip, t.chip_id, 'v3.0' if t.version > 2 else 'v2.1'))
        print('Flash:   %d kbytes (%s pins), console at %d baud' % (
            t.size // 1024, 'RTL8720CF' if t.mode == 1 else 'RTL8720CX / CM', t.baud))
        print('MAC:     %s (Wi-Fi), %s (BT) (eFuse)' % (t.mac, t.mac_bt))
    elif args.op == 'rf':
        size = args.size or t.size - args.address
        if args.address + size > t.size:
            raise FlasherError('0x%x bytes at 0x%x: beyond the flash (%d kbytes)' % (size, args.address, t.size // 1024))
        os.makedirs(os.path.dirname(os.path.abspath(args.filename)), exist_ok=True)
        t1 = time.time()
        data = rtl2_read(link, t, args.address, size)
        print('Read 0x%x bytes in %.1f s at %d baud; checked against the SHA-256 computed by the chip' % (
            size, time.time() - t1, t.baud))
        if args.twice:
            if rtl2_read(link, t, args.address, size) != data:
                with open(args.filename + '.pass1', 'wb') as fh:
                    fh.write(data)
                raise FlasherError('the two reads differ; the first was kept as %s.pass1' % args.filename)
            print('Second read is the same.')
        with open(args.filename, 'wb') as fh:
            fh.write(data)
        print('Saved %d bytes to %s' % (len(data), args.filename))
    elif args.op == 'wf':
        if args.no_erase:
            raise FlasherError('RTL8720C: the ROM always erases what it writes (no --no-erase)')
        for addr, data in rtl2_image(args):
            print('Write 0x%06x..0x%06x' % (addr, addr + len(data)))
            rtl2_write(link, t, addr, data, verify=not args.no_verify)
        if not args.no_verify:
            print('Verified (SHA-256 on the chip).')
    elif args.op in ('es', 'ea'):
        if args.op == 'es':
            start = args.address - args.address % SECTOR
            end = args.address + max(args.size, 1)
            end = min(end + (-end % SECTOR), t.size)
        else:
            start, end = 0, t.size
            print('Erasing the whole flash (also the boot loader and the partition table; the ROM download mode stays)')
        # the ROM has no erase command: the range is written with 0xFF, which fwd erases first
        rtl2_write(link, t, start, b'\xff' * (end - start))
        print('Erased 0x%06x..0x%06x' % (start, end))


# ================================================================ Wi-Fi settings (firmware 0.29)

def wifi_status(answer):
    """'wifi mode=sta ssid=<hex> pass=set ...' -> dict, the SSID decoded"""
    st = dict(kv.split('=', 1) for kv in answer.split()[1:] if '=' in kv)
    try:
        st['ssid'] = '' if st.get('ssid', '-') == '-' else bytes.fromhex(st['ssid']).decode('utf-8', 'replace')
    except ValueError:
        pass
    return st


def wifi_show(st):
    mode = {'ap': 'access point', 'sta': 'station', 'off': 'off'}.get(st.get('mode'), st.get('mode'))
    if st.get('mode') == 'off':
        print('Wi-Fi:   off%s' % (' (last network "%s")' % st['ssid'] if st.get('ssid') else ''))
    else:
        where = 'IP %s' % st.get('ip', 'none')
        if st.get('state'):
            where += ' (%s)' % st['state']
        if st.get('rssi'):
            where += ', RSSI %s dBm' % st['rssi']
        if st.get('stations'):
            where += ', %s stations' % st['stations']
        print('Wi-Fi:   %s "%s", %s' % (mode, st.get('ssid', ''), where))
    print('Name:    %s (%s.local), port %s' % (st.get('name'), st.get('name'), st.get('port')))
    print('Secrets: password %s, key %s' % (st.get('pass'), st.get('key')))
    if st.get('key') == 'none' and st.get('mode') != 'off':
        print('         without a key anyone on the network can use the flasher (wifi key sets one)')
    print('Client:  %s; heap %s bytes free, largest block %s' % (st.get('client'), st.get('heap'), st.get('largest')))


def ask_secret(env, prompt, confirm=False):
    """A password or key: from the environment variable env, or typed (never echoed, never printed)."""
    value = os.environ.get(env)
    if value is not None:
        return value
    value = getpass.getpass(prompt)
    if confirm and value and getpass.getpass('Again: ') != value:
        raise FlasherError('the two entries differ')
    return value


def wifi_hex(text):
    return text.encode('utf-8').hex() if text else '-'


def wifi_main(f, args):
    over_tcp = is_tcp(args.port)
    action, value = args.action, args.value
    if action in ('ap', 'join', 'name') and not value:
        raise FlasherError('wifi %s: give the %s' % (action, 'name' if action == 'name' else 'SSID'))
    if action in ('status', 'off', 'key') and value:
        raise FlasherError('wifi %s takes no value (passwords and keys are asked for)' % action)
    if action == 'status':
        wifi_show(wifi_status(f.cmd('WIFI')))
        return
    if action == 'off':
        print(f.cmd('WIFI OFF'))
        return
    if action == 'name':
        if not re.fullmatch(r'[A-Za-z0-9-]{1,31}', value):
            raise FlasherError('the name: 1..31 letters, digits, -')
        f.cmd('WIFI NAME %s' % value)
        print('Name set to %s; used from the next start of Wi-Fi (wifi ap / join, or a restart).' % value)
        return
    if action == 'key':
        key = ask_secret('C3_FLASHER_KEY', 'Key for TCP clients (empty: no key): ', confirm=True)
        if key and not re.fullmatch(r'[!-~]{1,64}', key):
            raise FlasherError('the key: 1..64 printable characters without spaces')
        f.cmd('WIFI KEY %s' % wifi_hex(key))
        print('Key %s.' % ('set' if key else 'removed: anyone on the network can use the flasher'))
        if key:
            print('Give it with --key or C3_FLASHER_KEY when connecting with --host.')
        return
    ssid = value
    if not 1 <= len(ssid.encode('utf-8')) <= 32:
        raise FlasherError('the SSID: 1..32 bytes')
    if action == 'ap':
        password = ask_secret('C3_FLASHER_WIFI_PASSWORD', 'Password for the access point (8..63 characters): ',
                              confirm=True)
        if not 8 <= len(password.encode('utf-8')) <= 63:
            raise FlasherError('an access point needs a password of 8..63 characters')
    else:
        password = ask_secret('C3_FLASHER_WIFI_PASSWORD', 'Password of "%s" (empty for an open network): ' % ssid)
        if password and not 8 <= len(password.encode('utf-8')) <= 63:
            raise FlasherError('a network password has 8..63 characters')
    f.cmd('WIFI %s %s %s' % ('AP' if action == 'ap' else 'STA', wifi_hex(ssid), wifi_hex(password)))
    password = None
    if over_tcp:
        print('Wi-Fi restarts as %s "%s"; this connection ends. Reconnect with --host (the address may change).'
              % ('an access point' if action == 'ap' else 'a station on', ssid))
        return
    st = wifi_status(f.cmd('WIFI'))
    deadline = time.time() + 15
    while action == 'join' and st.get('ip', 'none') == 'none' and time.time() < deadline:
        time.sleep(1)
        st = wifi_status(f.cmd('WIFI'))
    wifi_show(st)
    if action == 'join' and st.get('ip', 'none') == 'none':
        print('Not connected yet: it keeps trying. Check the SSID and the password; wifi shows the state.')


def main():
    ap = argparse.ArgumentParser(description='ESP32-C3 Telink TLSR825x SWire flasher')
    ap.add_argument('-p', '--port', default='/dev/ttyACM0')
    ap.add_argument('--host', default='',
                    help='reach the flasher over Wi-Fi instead of USB: NAME[:PORT] (default port %d; firmware 0.29), '
                         'e.g. c3-flasher.local or 192.168.4.1' % WIFI_PORT)
    ap.add_argument('--key', default='',
                    help='the key the flasher asks of TCP clients (better: the environment variable C3_FLASHER_KEY)')
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
    ap.add_argument('--esp', action='store_true',
                    help='ESP32 / ESP8266 in its ROM download mode, through esptool: target TX on the SWS wire, RX '
                         'on GPIO 21, GPIO 0 (IO0) on the BOOT wire (GPIO 0), EN / RST on the reset wire; info, rf, '
                         'wf, es, ea')
    ap.add_argument('--esp-baud', type=auto_int, default=0,
                    help='ESP: speed of esptool after its stub is loaded (default 921600; firmware older than 0.25: '
                         'the bridge speed from the start, default 115200); a failed run is repeated at 460800, '
                         'then 115200')
    ap.add_argument('--esp-chip', default='',
                    help='ESP: chip type for esptool (esp32, esp8266, esp32c3, ...; default: detected)')
    ap.add_argument('--esp-stay', action='store_true',
                    help='ESP: leave the target in download mode at the end (no restart)')
    ap.add_argument('--ln', action='store_true',
                    help='Lightning LN882H over its UART download mode: TX0 (A2) on the SWS wire, RX0 (A3) on GPIO 21, '
                         'BOOT (A9) on GPIO 0, CEN on the reset wire, 3V3 on the power pins; info, rf, wf, es, ea')
    ap.add_argument('--ln-baud', type=auto_int, default=0,
                    help='LN882H: speed of the RAM loader (default 1000000; when that fails 460800, then 115200)')
    ap.add_argument('--ln-ramcode', default='',
                    help='LN882H: the RAM loader file (default: %s next to this script, or ltchiptool\'s)' %
                         LN_RAMCODE_NAME)
    ap.add_argument('--ln-seconds', type=auto_int, default=5,
                    help='LN882H: how long to wait for the ROM after the restart (default 5)')
    ap.add_argument('--rtl', action='store_true',
                    help='Realtek AmebaZ (RTL8710BN, RTL8710BX ...: Tuya WR modules) over its UART download mode: log '
                         'TX (PA30) on the SWS wire, log RX (PA29) on GPIO 21, CEN on the reset wire, 3V3 on the '
                         'power pins; info, rf, wf, es, ea')
    ap.add_argument('--rtl2', action='store_true',
                    help='Realtek AmebaZ2 (RTL8720CF, RTL8710CX ...: Tuya WBR / CR modules) over its UART download mode: '
                         'log TX (PA16) on the SWS wire, log RX (PA15) on GPIO 21, PA00 on GPIO 0 (held high), CEN '
                         'on the reset wire, 3V3 on the power pins; info, rf, wf, es, ea')
    ap.add_argument('--rtl-baud', type=auto_int, default=0,
                    help='RTL8720C: console speed after the link at 115200 (default 1500000; when that fails 921600, '
                         'then 115200)')
    ap.add_argument('--rtl-seconds', type=auto_int, default=5,
                    help='RTL8710B / RTL8720C: how long to wait for the ROM after the restart (default 5)')
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
    p.add_argument('--follow', choices=['esp', 'ln', 'ucfg'],
                   help='esp: follow esptool\'s speed change (CHANGE_BAUDRATE): start the bridge at 115200 and give '
                        'esptool any --baud; the bridge switches when the target has answered (firmware 0.25). '
                        'ln: follow the LN882H RAM loader\'s "baudrate N" line (firmware 0.26); ucfg: the Realtek '
                        'AmebaZ2 ROM\'s "ucfg N ..." line (firmware 0.28)')
    p.add_argument('--strap', choices=['rtl'],
                   help='rtl: restart the target with its TX wire (the SWS wire) pulled low, the download strap of a '
                        'Realtek AmebaZ (firmware 0.27)')
    sub.add_parser('bridge-exit', help='leave the UART bridge')
    p = sub.add_parser('log', help='save the UART log of the last UART session (bridge, --esp, --bk) as text: '
                                   'every byte to and from the target with its time, speed changes, resets')
    p.add_argument('filename', nargs='?', default='uart_log.txt')
    p.add_argument('--bin', default='', help='also save the stored blocks as they are')
    p.add_argument('--clear', action='store_true', help='clear the log instead')
    p.add_argument('--tuya', action='store_true', help='decode the Tuya MCU serial protocol (55 AA frames) under the '
                                                       'records')
    p = sub.add_parser('sniff', help='two-line UART logger (firmware 0.30): listen to both directions of a UART link '
                                     'between two other parties without driving anything, line A on the SWS wire '
                                     '(GPIO 5), line B on GPIO 20; live, and into the UART log (log)')
    p.add_argument('sniff_baud', metavar='BAUD', nargs='?', type=auto_int, default=0,
                   help='speed of line A (and of line B without --baud-b)')
    p.add_argument('--baud-b', type=auto_int, default=0, help='speed of line B when it differs')
    p.add_argument('--seconds', type=float, default=0, help='end after this many seconds (default: Ctrl-C)')
    p.add_argument('--out', default='', help='also write the lines to this file')
    p.add_argument('--tuya', action='store_true', help='decode the Tuya MCU serial protocol (55 AA frames)')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--at-boot', action='store_true',
                   help='do not listen now: capture from every power-up of the flasher, without a host, until a '
                        'host\'s first command (read it with log)')
    g.add_argument('--no-at-boot', action='store_true', help='switch the capture at power-up off')
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
    p = sub.add_parser('wifi', help='Wi-Fi link (firmware 0.29): status; off; ap SSID (an access point of its own); '
                                    'join SSID (a network); key (the key TCP clients must give; empty: none); '
                                    'name NAME (mDNS name). Passwords and the key are asked for, or taken from '
                                    'C3_FLASHER_WIFI_PASSWORD / C3_FLASHER_KEY')
    p.add_argument('action', nargs='?', choices=['status', 'off', 'ap', 'join', 'key', 'name'], default='status')
    p.add_argument('value', nargs='?', default=None, help='the SSID (ap, join) or the name (name)')
    args = ap.parse_args()

    t0 = time.time()
    args.key = args.key or os.environ.get('C3_FLASHER_KEY', '')
    try:
        if args.host:
            args.port = tcp_url(args.host)
    except FlasherError as e:
        print('Error:', e)
        sys.exit(1)
    if args.op == 'bridge-exit':
        try:
            print(bridge_exit(args.port, args.key) or 'no answer (was the bridge or the logger open? A logger whose '
                  'TCP client has gone ends with any command, e.g. log)')
        except (FlasherError, serial.SerialException) as e:
            print('Error:', e)
            sys.exit(1)
        return
    f = None
    try:
        f = Flasher(args.port, key=args.key)
        print('Flasher:', f.version)
        try:
            fw = tuple(int(x) for x in f.version.split()[1].split('.'))
        except (IndexError, ValueError):
            fw = (0, 0)
        if fw < (0, 18):
            raise FlasherError('this script needs flasher firmware 0.18 or newer on the C3 (binary streams)')
        if args.op == 'wifi':
            if fw < (0, 29):
                raise FlasherError('the Wi-Fi link needs flasher firmware 0.29 or newer on the C3')
            wifi_main(f, args)
            return
        if args.op == 'sniff':
            if fw < (0, 30):
                raise FlasherError('the two-line UART logger needs flasher firmware 0.30 or newer on the C3')
            sniff_main(f, args)
            return
        if args.op == 'log':
            if fw < (0, 25):
                raise FlasherError('the UART log needs flasher firmware 0.25 or newer on the C3')
            log_main(f, args)
            return
        if args.wch:
            if fw < (0, 22):
                raise FlasherError('WCH mode needs flasher firmware 0.22 or newer on the C3')
            wch_main(f, args)
            print('Done in %.1f s' % (time.time() - t0))
            return
        if args.op == 'unlock':
            raise FlasherError('unlock is a WCH command (--wch)')
        if args.esp:
            if fw < (0, 15):
                raise FlasherError('the ESP mode needs flasher firmware 0.15 or newer on the C3 (UART bridge)')
            esp_main(f, args, follow=fw >= (0, 25))
            print('Done in %.1f s' % (time.time() - t0))
            return
        if args.rtl2:
            if fw < (0, 28):
                raise FlasherError('RTL8720C mode needs flasher firmware 0.28 or newer on the C3')
            rtl2_main(f, args)
            print('Done in %.1f s' % (time.time() - t0))
            return
        if args.rtl:
            if fw < (0, 27):
                raise FlasherError('RTL8710B mode needs flasher firmware 0.27 or newer on the C3')
            rtl_main(f, args)
            print('Done in %.1f s' % (time.time() - t0))
            return
        if args.ln:
            if fw < (0, 26):
                raise FlasherError('LN882H mode needs flasher firmware 0.26 or newer on the C3')
            ln_main(f, args)
            print('Done in %.1f s' % (time.time() - t0))
            return
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
        if args.op == 'bridge' and sum(bool(x) for x in (args.catch, args.follow, args.strap)) > 1:
            raise FlasherError('--catch, --follow and --strap do not go together')
        if args.op == 'bridge' and args.strap and fw < (0, 27):
            raise FlasherError('--strap needs flasher firmware 0.27 or newer on the C3')
        need = {'esp': (0, 25), 'ln': (0, 26), 'ucfg': (0, 28)}
        if args.op == 'bridge' and args.follow and fw < need[args.follow]:
            raise FlasherError('--follow %s needs flasher firmware %d.%d or newer on the C3' % (
                (args.follow,) + need[args.follow]))
        if args.op == 'bridge' and args.catch and fw < (0, 23):
            raise FlasherError('--catch needs flasher firmware 0.23 or newer on the C3')
        if args.op == 'power-off':
            f.cmd('PWR 0')
            time.sleep(0.2)
            f.cmd('PWR z')
            print('Target power off, power pins are inputs.')
            return
        if args.op == 'bridge':
            mode = args.catch or args.follow or args.strap
            print(f.cmd('BRIDGE %d %s%s' % (args.baud, args.boot, ' ' + mode if mode else '')))
            f.ser.close()
            if is_tcp(args.port):
                print('The target tool connects to %s now (from this computer no key is needed while the bridge is'
                      % args.port)
                print('open). With no connection for 60 s the bridge ends by itself. Leave with: --host %s bridge-exit'
                      % args.host)
                return
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
