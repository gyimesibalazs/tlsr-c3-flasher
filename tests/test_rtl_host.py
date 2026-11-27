"""Realtek AmebaZ mode (--rtl): tlsr_c3.py against a model of the flasher's bridge ("BRIDGE 1500000 z rtl", the speed
escape "~~~S<baud>", the exit escape) and of the RTL8710B ROM download mode: NAKs while idle (loud), quiet after an
ACK, the speed command (answered at the old speed), the 1 k flash read with an ACK per piece, XMODEM with 1 k packets
that carry their address and a sum checksum (flash: erased and written; RAM: run after EOT), sector erase. The RAM
program is checked piece by piece against ltchiptool's code pieces (their fields: addresses, lengths), run on the
model (chip id, flash id, eFuse logical map, MD5 of a flash range) and its output printed at 115200 between the
marker and the ROM's banner. Bytes sent while the bridge and the chip run at different speeds arrive garbled."""
import hashlib, os, sys, tempfile, time, types
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tlsr_c3 as T

T.BRIDGE_PAUSE = 0
C = T.RtlCode


class Sim:
    def __init__(self, size=0x200000, strap_wired=True, chip_id=0xFF, flash_id=b'\xc8\x40\x15'):
        self.flash = bytearray(os.urandom(size))
        self.flash[0x9000:0x9008] = (0x080D0000).to_bytes(4, 'little') + b'\xff\xff\xff\xff'
        self.size, self.strap_wired, self.chip_id, self.flash_id = size, strap_wired, chip_id, flash_id
        self.efuse = bytearray(b'\xff' * 512)
        self.efuse[0x11A:0x120] = bytes.fromhex('00e04c112233')
        self.out = bytearray()
        self.mode = 'cmd'
        self.cmdline = b''
        self.target = 'off'
        self.bridge_baud = self.target_baud = 0
        self.inbuf = b''
        self.loud = True
        self.last_nak = 0
        self.violations, self.log = [], []
        self.erased = []
        self.xm = None

    # ---- the port
    def write(self, data):
        data = bytes(data)
        if self.mode == 'bridge':
            if data == b'~~~X':
                self.mode = 'cmd'
                self.out += b'OK bridge closed\n'
                return len(data)
            if data.startswith(b'~~~S') and data.endswith(b'\n'):
                self.bridge_baud = int(data[4:-1])
                self.log.append('bridge speed %d' % self.bridge_baud)
                return len(data)
            for b in data:
                self.target_in(self.garble(b))
            return len(data)
        for b in data:
            if b == 10:
                self.flasher_cmd(self.cmdline.decode().strip())
                self.cmdline = b''
            else:
                self.cmdline += bytes([b])
        return len(data)

    def read(self, n=1):
        if self.mode == 'bridge' and self.target == 'rom' and self.loud and not self.out and self.xm is None \
                and time.time() - self.last_nak > 0.01:
            self.last_nak = time.time()
            self.to_host(T.NAK1)
        d = bytes(self.out[:n])
        del self.out[:n]
        return d

    @property
    def in_waiting(self):
        return len(self.out)

    def reset_input_buffer(self):
        self.out.clear()

    # ---- flasher
    def flasher_cmd(self, line):
        self.log.append(line)
        if line.startswith('BRIDGE'):
            a = line.split()
            self.bridge_baud = int(a[1])
            self.mode = 'bridge'
            self.out += b'OK bridge %s baud, boot open\n' % a[1].encode()
            strap = len(a) > 3 and a[3] == 'rtl' and self.strap_wired
            self.target = 'rom' if strap else 'app'
            self.target_baud = 1500000 if strap else 115200
            self.loud = True
            self.inbuf = b''
            self.xm = None
        elif line == 'RST':
            self.target = 'app'
            self.out += b'OK\n'
        else:
            self.out += b'ERR unknown command\n'

    def garble(self, b):
        return b if self.bridge_baud == self.target_baud else 0xFF

    def to_host(self, data):
        if self.mode == 'bridge':
            self.out += bytes(self.garble(b) for b in data)

    # ---- the ROM
    def target_in(self, b):
        if self.target != 'rom':
            return
        self.inbuf += bytes([b])
        while self.inbuf and self.step():
            pass

    def step(self):
        """Consumes one complete command or packet from inbuf; False when more bytes are needed."""
        buf = self.inbuf
        if self.xm is not None:
            if buf[0] == 0x04:                       # EOT
                self.inbuf = buf[1:]
                self.xm_end()
                return True
            if buf[0] != 0x02:
                self.violations.append('XMODEM: 0x%02x outside a packet' % buf[0])
                self.inbuf = buf[1:]
                return True
            if len(buf) < 3 + 1028 + 1:
                return False
            pkt, self.inbuf = buf[:1032], buf[1032:]
            seq, nseq, body, ck = pkt[1], pkt[2], pkt[3:1031], pkt[1031]
            if seq != 255 - nseq or sum(body) & 0xFF != ck or seq != self.xm['seq']:
                self.to_host(T.NAK1)
                return True
            addr = int.from_bytes(body[:4], 'little')
            self.xm['chunks'].append((addr, body[4:]))
            self.xm['seq'] = (seq + 1) & 0xFF
            self.to_host(T.ACK1)
            return True
        c = buf[0]
        if c == 0x06:                                # ACK: quiet
            self.loud = False
            self.inbuf = buf[1:]
        elif c == 0x18:                              # CAN
            self.inbuf = buf[1:]
        elif c == T.RTL_CMD_STATUS:
            self.inbuf = buf[1:]
            self.loud = True
            self.to_host(b'\x00')
        elif c == T.RTL_CMD_BAUD:
            if len(buf) < 2:
                return False
            self.inbuf = buf[2:]
            self.to_host(T.ACK1)
            self.target_baud = T.RTL_BAUD_TABLE[buf[1]]
            self.log.append('rom speed %d' % self.target_baud)
        elif c == T.RTL_CMD_XMODEM:
            self.inbuf = buf[1:]
            self.xm = dict(seq=1, chunks=[])
            self.to_host(T.ACK1)
        elif c == T.RTL_CMD_READ:
            if len(buf) < 6:
                return False
            self.inbuf = buf[6:]
            addr, n = int.from_bytes(buf[1:4], 'little'), int.from_bytes(buf[4:6], 'little') * 4096
            if self.loud:
                self.violations.append('read while loud')
            self.reading = (addr, n)
            for k in range(0, n, 1024):              # the model sends at once; the host ACKs each piece
                self.out += bytes(self.flash[addr + k:addr + k + 1024])
            self.ack_due = n // 1024 + 1
        elif c == T.RTL_CMD_ERASE:
            if len(buf) < 6:
                return False
            self.inbuf = buf[6:]
            addr, cnt = int.from_bytes(buf[1:4], 'little'), int.from_bytes(buf[4:6], 'little')
            if addr % 4096:
                self.violations.append('erase not aligned')
            self.flash[addr:addr + cnt * 4096] = b'\xff' * (cnt * 4096)
            self.erased.append(addr)
            self.to_host(T.ACK1)
        else:
            self.violations.append('unknown command 0x%02x' % c)
            self.inbuf = buf[1:]
        return True

    def xm_end(self):
        x, self.xm = self.xm, None
        chunks = x['chunks']
        if not chunks:
            self.violations.append('empty XMODEM')
            return
        base = chunks[0][0]
        if base >> 24 == 0x08:
            for addr, d in chunks:
                a = addr & 0xFFFFFF
                if a % 4096 == 0:                    # the ROM erases each sector it starts writing
                    self.flash[a:a + 4096] = b'\xff' * 4096
                for i, v in enumerate(d):
                    self.flash[a + i] &= v
            self.to_host(T.ACK1)
            self.target_baud = 1500000
            self.log.append('flash write 0x%x %d' % (base & 0xFFFFFF, 1024 * len(chunks)))
        elif base == T.RTL_RAM:
            ram = b''.join(d for _, d in chunks)
            self.run_ram(ram)
        else:
            self.violations.append('XMODEM to 0x%x' % base)

    # ---- the RAM program, piece by piece
    def run_ram(self, ram):
        entry = int.from_bytes(ram[:4], 'little')
        if entry != (T.RTL_RAM + 24) | 1:
            self.violations.append('entry 0x%x' % entry)
            return
        code = ram[24:]
        data = bytearray(1024)
        printed = b''
        pos = 0
        self.target_baud = 115200                    # the ROM hands the log UART back at 115200
        while True:
            if pos >= len(code):
                self.violations.append('the program does not end with download_mode')
                return
            rest = code[pos:]
            if rest.startswith(C.greeting(400, T.RTL_MARK)[:24]):
                n = int.from_bytes(rest[28:32], 'little')
                printed += rest[32:32 + n]
                pos += 48
            elif rest.startswith(C.chip_id(0)[:24]):
                off = int.from_bytes(rest[24:28], 'little') - T.RTL_DATA
                data[off] = self.chip_id
                pos += 28
            elif rest.startswith(C.flash_id(0)[:16]):
                off = int.from_bytes(rest[16:20], 'little') - T.RTL_DATA
                data[off:off + 3] = self.flash_id
                pos += 20
            elif rest.startswith(C.efuse_logical_map(0)[:12]):
                off = int.from_bytes(rest[12:16], 'little') - T.RTL_DATA
                data[off:off + 512] = self.efuse
                pos += 16
            elif rest.startswith(C.md5(0, 0, 0)[:40]):
                addr, n = int.from_bytes(rest[40:44], 'little'), int.from_bytes(rest[44:48], 'little')
                off = int.from_bytes(rest[48:52], 'little') - T.RTL_DATA
                if addr >> 24 != 0x08:
                    self.violations.append('md5 of 0x%x' % addr)
                a = addr & 0xFFFFFF
                data[off:off + 16] = hashlib.md5(bytes(self.flash[a:a + n])).digest()
                pos += 56
            elif rest.startswith(C.print_data(0)[:16]):
                addr, n = int.from_bytes(rest[16:20], 'little'), int.from_bytes(rest[20:24], 'little')
                printed += bytes(data[addr - T.RTL_DATA:addr - T.RTL_DATA + n])
                pos += 24
            elif rest.startswith(C.download_mode()):
                break                                # back to the ROM; the rest is the packet's padding
            else:
                self.violations.append('unknown code piece at +%d: %s' % (pos, rest[:16].hex()))
                return
        self.to_host(b'\x00garbage' + printed + b'\r\n' + T.RTL_ROM_BANNER + b'\r\n')
        self.target_baud = 1500000                   # download mode again
        self.loud = True
        self.log.append('ram program')


def flasher(sim):
    f = object.__new__(T.Flasher)
    f.ser = sim
    f.rxbuf = b''
    f.version = 'tlsr-c3 0.27'
    return f


def args(op, **kw):
    a = types.SimpleNamespace(op=op, rtl_seconds=2, run=False, address=0, size=0, filename='', twice=False,
                              no_verify=False, no_erase=False)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def run(sim, a):
    T.rtl_main(flasher(sim), a)
    assert not sim.violations, sim.violations
    assert sim.mode == 'cmd', 'the bridge was left open'


tmp = tempfile.mkdtemp()

# ---- info: chip id, flash id and size, MAC from the eFuse map, OTA slot
sim = Sim()
run(sim, args('info'))
assert sim.log[0] == 'BRIDGE 1500000 z rtl', sim.log
assert 'rom speed 115200' in sim.log and 'bridge speed 115200' in sim.log and 'bridge speed 1500000' in sim.log
print('info: ok')

# ---- rf of an unaligned range, checked by the chip's MD5; whole flash
sim = Sim()
out = os.path.join(tmp, 'part.bin')
run(sim, args('rf', address=0x1234, size=0x5678, filename=out, twice=True))
assert open(out, 'rb').read() == bytes(sim.flash[0x1234:0x1234 + 0x5678])
sim = Sim(size=0x100000, flash_id=b'\xc8\x40\x14')
run(sim, args('rf', filename=out))
assert open(out, 'rb').read() == bytes(sim.flash)
print('rf: ok')

# ---- wf at an unaligned address: the rest of the first and last sectors stays; MD5 verified
sim = Sim()
before = bytes(sim.flash)
img = os.urandom(0x3333)
fn = os.path.join(tmp, 'img.bin')
open(fn, 'wb').write(img)
run(sim, args('wf', address=0x20100, filename=fn))
assert bytes(sim.flash[0x20100:0x20100 + len(img)]) == img
assert bytes(sim.flash[:0x20100]) == before[:0x20100]
assert bytes(sim.flash[0x20100 + len(img):]) == before[0x20100 + len(img):]
print('wf: ok')

# ---- es, ea
sim = Sim()
run(sim, args('es', address=0x30010, size=0x1000))
assert sim.erased == [0x30000, 0x31000] and all(b == 0xff for b in sim.flash[0x30000:0x32000])
sim = Sim(size=0x100000, flash_id=b'\xc8\x40\x14')
run(sim, args('ea'))
assert all(b == 0xff for b in sim.flash)
print('es, ea: ok')

# ---- no strap: the chip runs its firmware and never NAKs; the bridge is left
sim = Sim(strap_wired=False)
try:
    T.rtl_main(flasher(sim), args('info'))
    raise AssertionError('no error')
except T.FlasherError as e:
    assert 'no answer from the ROM' in str(e), e
assert sim.mode == 'cmd'
print('no download mode: ok')

# ---- --run
sim = Sim()
run(sim, args('info', run=True))
assert sim.log[-1] == 'RST' and sim.target == 'app'
print('--run: ok')
print('all RTL8710B host tests passed')
