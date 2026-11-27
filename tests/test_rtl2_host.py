"""Realtek AmebaZ2 mode (--rtl2): tlsr_c3.py against a model of the flasher's bridge ("BRIDGE 115200 hi ucfg": PA00
high, the "ucfg N" line followed as src/lnscan.h does, the exit escape) and of the RTL8720C ROM's download console:
ping, ucfg (OK at the new speed), DW / DB text dumps (the flash at 0x98000000 only after hashq has set it up), EW,
fwd (XMODEM-1K, the CRC or the checksum variant; written sector by sector with an erase at each sector start; a
cancelled transfer answers CAN "ER"), hashq (SHA-256 from where fwd started), and the ROM console command table
through which a RAM program is started: the eFuse read program is recognised and fills the eFuse image. Bytes sent
while the bridge and the chip run at different speeds arrive garbled."""
import hashlib, os, sys, tempfile, types
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tlsr_c3 as T
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lnscan_model import LnScan

T.BRIDGE_PAUSE = 0
TABLE = 0x1002F100
NAMES = [b'ping', b'DW', b'DB', b'EW', b'oldcmd', b'fwd', b'hashq']


def efuse_image(chip_id=0xFE, mac=bytes.fromhex('00e04c8720cf'), bt=bytes.fromhex('00e04c8720d0')):
    phys = bytearray(b'\xff' * 512)
    # cell 35 (0x118..0x11F) and cell 50 (0x190..0x197), extended headers, all four words present
    rec = bytes([0x6F, 0x40]) + b'\xff\xff' + mac + bytes([0x4F, 0x60]) + bt + b'\xff\xff'
    phys[:len(rec)] = rec
    phys[0x1F8] = chip_id
    return bytes(phys)


class Sim:
    def __init__(self, strap_wired=True, crc=True, mode=1, fail_above=0):
        self.size = 0x200000 if mode == 1 else 0x400000
        self.flash = bytearray(os.urandom(self.size))
        self.strap_wired, self.crc, self.fail_above = strap_wired, crc, fail_above
        self.mem = {0x40000038: mode << 5, 0x400001F0: 0x30, 0x1002F054: TABLE}
        for i, name in enumerate(NAMES):
            self.mem[TABLE + 12 * i] = 0x1002F300 + 16 * i
            self.mem[TABLE + 12 * i + 4] = 0x1234 + i
            self.put_bytes(0x1002F300 + 16 * i, name.ljust(16, b'\0'))
        self.mem[TABLE + 12 * len(NAMES)] = 0
        self.out = bytearray()
        self.mode = 'cmd'
        self.cmdline = b''
        self.target = 'off'
        self.bridge_baud = self.target_baud = 115200
        self.line = b''
        self.xm = None
        self.flash_ready = False
        self.hash_off = 0
        self.violations, self.log = [], []

    def put_bytes(self, addr, data):
        for i in range(0, len(data), 4):
            self.mem[addr + i] = int.from_bytes(data[i:i + 4].ljust(4, b'\0'), 'little')

    def get_byte(self, addr):
        if addr >> 24 == 0x98:
            if not self.flash_ready:
                self.violations.append('flash read before it was set up')
            return self.flash[addr & 0xFFFFFF]
        return (self.mem.get(addr & ~3, 0) >> (8 * (addr & 3))) & 0xFF

    # ---- the port
    def write(self, data):
        data = bytes(data)
        if self.mode == 'bridge':
            if data == b'~~~X':
                self.mode = 'cmd'
                self.out += b'OK bridge closed\n'
                return len(data)
            for b in data:
                baud = self.scan.feed(b)
                g = self.garble(b)                   # the line end still goes at the old speed
                if baud:                             # the bridge switches before the ROM answers at the new one
                    self.bridge_baud = baud
                self.target_in(g)
            return len(data)
        for b in data:
            if b == 10:
                self.flasher_cmd(self.cmdline.decode().strip())
                self.cmdline = b''
            else:
                self.cmdline += bytes([b])
        return len(data)

    def read(self, n=1):
        d = bytes(self.out[:n])
        del self.out[:n]
        return d

    @property
    def in_waiting(self):
        return len(self.out)

    def reset_input_buffer(self):
        self.out.clear()

    def flasher_cmd(self, line):
        self.log.append(line)
        if line.startswith('BRIDGE'):
            a = line.split()
            self.bridge_baud = int(a[1])
            self.mode = 'bridge'
            self.scan = LnScan(b'ucfg ', True)
            self.out += b'OK bridge %s baud, boot high\n' % a[1].encode()
            self.target = 'download' if a[2] == 'hi' and self.strap_wired else 'app'
            self.target_baud = 115200
            self.flash_ready = False
            self.xm = None
            self.line = b''
            if self.target == 'app':
                self.to_host(b'\r\n== Rtl8710c IoT Platform ==\r\n')
        elif line == 'RST':
            self.target = 'app'
            self.out += b'OK\n'
        else:
            self.out += b'ERR unknown command\n'

    def garble(self, b):
        ok = self.bridge_baud == self.target_baud and not (self.fail_above and self.target_baud > self.fail_above)
        return b if ok else 0xFF

    def to_host(self, data):
        if self.mode == 'bridge':
            self.out += bytes(self.garble(b) for b in data)

    # ---- the ROM console
    def target_in(self, b):
        if self.target != 'download':
            return
        if self.xm is not None:
            self.xm_in(b)
            return
        if b == 10:
            line, self.line = self.line.decode(errors='replace').strip(), b''
            if line:
                self.execute(line)
        elif b != 13:
            self.line += bytes([b])

    def execute(self, line):
        a = line.split()
        self.log.append('rom: ' + ' '.join(a[:3]))
        if line == 'ping':
            self.to_host(b'ping')
        elif a[0] == 'ucfg':
            self.target_baud = int(a[1])
            self.to_host(b'OK')
        elif a[0] == 'DW':
            addr, n = int(a[1], 16), int(a[2])
            for i in range(0, n, 4):
                words = [self.mem.get(addr + 4 * (i + k), 0) for k in range(min(4, n - i))]
                self.to_host(b'%08X:    %s\r\n' % (addr + 4 * i, b'    '.join(b'%08X' % w for w in words)))
        elif a[0] == 'DB':
            addr, n = int(a[1], 16), int(a[2])
            self.to_host(b'[Addr]   .0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .A .B .C .D .E .F\r\n')
            for i in range(0, n, 16):
                row = bytes(self.get_byte(addr + i + k) for k in range(16))
                self.to_host(b'%08X: %s   %s\r\n' % (addr + i, b' '.join(b'%02X' % x for x in row),
                                                       bytes(x if 32 <= x < 127 else 46 for x in row)))
        elif a[0] == 'EW':
            addr = int(a[1], 16)
            for k, w in enumerate(a[2:]):
                self.mem[addr + 4 * k] = int(w, 16)
                self.to_host(b'[%08X] = %08X\r\n' % (addr + 4 * k, int(w, 16)))
        elif a[0] == 'hashq':
            n = int(a[1])
            if a[2:] != ['0', '1']:
                self.violations.append('hashq flash settings %r' % a[2:])
            self.flash_ready = True
            self.to_host(b'hashs ' + hashlib.sha256(bytes(self.flash[self.hash_off:self.hash_off + n])).digest())
        elif a[0] == 'fwd':
            if a[1:3] != ['0', '1']:
                self.violations.append('fwd flash settings %r' % a[1:3])
            self.hash_off = int(a[3], 16)
            self.xm = dict(off=self.hash_off, seq=1, buf=b'', data=bytearray())
            self.to_host(b'C' if self.crc else b'\x15')
        else:
            for i, name in enumerate(NAMES):
                if a[0] == name.decode():
                    if name.decode() in T.RTL2_USED:
                        self.to_host(b'\r\n')
                        return
                    handler = self.mem[TABLE + 12 * i + 4]
                    if handler != (T.RTL2_CODE | 1):
                        self.violations.append('%s run with handler 0x%x' % (line, handler))
                        return
                    code = b''.join(self.mem.get(T.RTL2_CODE + 4 * k, 0).to_bytes(4, 'little')
                                    for k in range(len(T.RTL2_EFUSE_CODE) // 4))
                    if code != T.RTL2_EFUSE_CODE:
                        self.violations.append('unknown RAM program')
                        return
                    self.put_bytes(T.RTL2_DATA, efuse_image())
                    self.log.append('efuse program')
                    return
            self.to_host(b'\r\nCommand NOT found.\r\n')

    def xm_in(self, b):
        x = self.xm
        x['buf'] += bytes([b])
        buf = x['buf']
        if buf[0] == 0x18:
            if len(buf) < 2:
                return
            self.xm = None
            self.to_host(b'\x18ER')
            return
        if buf[0] == 0x04:
            self.xm = None
            self.to_host(b'\x06')
            off = x['off']
            for k in range(0, len(x['data']), 1024):
                a = off + k
                if a % 4096 == 0:
                    self.flash[a:a + 4096] = b'\xff' * 4096
                for i, v in enumerate(x['data'][k:k + 1024]):
                    self.flash[a + i] &= v
            self.log.append('fwd 0x%x %d' % (off, len(x['data'])))
            return
        need = 3 + 1024 + (2 if self.crc else 1)
        if buf[0] != 0x02:
            self.violations.append('XMODEM: 0x%02x outside a packet' % buf[0])
            x['buf'] = b''
            return
        if len(buf) < need:
            return
        x['buf'] = b''
        seq, body, tail = buf[1], buf[3:1027], buf[1027:need]
        good = tail == (T.crc16_xmodem(body).to_bytes(2, 'big') if self.crc else bytes([sum(body) & 0xFF]))
        if buf[2] != 255 - seq or not good or seq != x['seq']:
            self.to_host(b'\x15')
            return
        x['data'] += body
        x['seq'] = (seq + 1) & 0xFF
        self.to_host(b'\x06')


def flasher(sim):
    f = object.__new__(T.Flasher)
    f.ser = sim
    f.rxbuf = b''
    f.version = 'tlsr-c3 0.28'
    return f


def args(op, **kw):
    a = types.SimpleNamespace(op=op, rtl_seconds=1, rtl_baud=0, run=False, address=0, size=0, filename='',
                              twice=False, no_verify=False, no_erase=False)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def run(sim, a):
    T.rtl2_main(flasher(sim), a)
    assert not sim.violations, sim.violations
    assert sim.mode == 'cmd', 'the bridge was left open'


if __name__ == '__main__':
    tmp = tempfile.mkdtemp()

    # the eFuse conversion on its own
    logi = T.efuse_logical(efuse_image())
    assert logi[0x11A:0x120] == bytes.fromhex('00e04c8720cf') and logi[0x190:0x196] == bytes.fromhex('00e04c8720d0')

    # info: speed raised to 1500000 by the bridge following the ucfg line, the eFuse program through an unused command
    sim = Sim()
    run(sim, args('info'))
    assert sim.log[0] == 'BRIDGE 115200 hi ucfg' and sim.bridge_baud == sim.target_baud == 1500000, sim.log
    assert 'efuse program' in sim.log and 'rom: EW 1002F134 10037001' in sim.log, sim.log  # oldcmd's handler
    print('info: ok')

    # rf of an unaligned range, checked by the chip's SHA-256
    sim = Sim(crc=False)
    out = os.path.join(tmp, 'part.bin')
    run(sim, args('rf', address=0x1234, size=0x5678, filename=out, twice=True))
    assert open(out, 'rb').read() == bytes(sim.flash[0x1234:0x1234 + 0x5678])
    print('rf: ok')

    # wf at an unaligned address (CRC and checksum XMODEM): the rest of the first and last sectors stays
    for crc in (True, False):
        sim = Sim(crc=crc)
        before = bytes(sim.flash)
        img = os.urandom(0x2345)
        fn = os.path.join(tmp, 'img.bin')
        open(fn, 'wb').write(img)
        run(sim, args('wf', address=0x10100, filename=fn))
        assert bytes(sim.flash[0x10100:0x10100 + len(img)]) == img
        assert bytes(sim.flash[:0x10100]) == before[:0x10100]
        assert bytes(sim.flash[0x10100 + len(img):]) == before[0x10100 + len(img):]
    print('wf: ok')

    # es: the range written with 0xFF
    sim = Sim()
    before = bytes(sim.flash)
    run(sim, args('es', address=0x20010, size=0x1000))
    assert all(b == 0xff for b in sim.flash[0x20000:0x22000])
    assert sim.flash[:0x20000] == before[:0x20000] and sim.flash[0x22000:] == before[0x22000:]
    print('es: ok')

    # the chip's UART fails above 921600: a new session at 921600
    sim = Sim(fail_above=921600)
    run(sim, args('info'))
    assert sim.log.count('BRIDGE 115200 hi ucfg') == 2 and sim.target_baud == 921600, sim.log
    print('fallback to 921600: ok')

    # PA00 not wired: the firmware runs, no answer
    sim = Sim(strap_wired=False)
    try:
        T.rtl2_main(flasher(sim), args('info'))
        raise AssertionError('no error')
    except T.FlasherError as e:
        assert 'no answer from the ROM' in str(e), e
    assert sim.mode == 'cmd'
    print('no download mode: ok')

    # --run
    sim = Sim()
    run(sim, args('info', run=True))
    assert sim.log[-1] == 'RST' and sim.target == 'app'
    print('--run: ok')
    print('all RTL8720C host tests passed')
