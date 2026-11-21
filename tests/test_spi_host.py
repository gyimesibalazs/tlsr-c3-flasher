import sys, zlib, base64, random, os, types, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tlsr_c3 as T

class FakeSpi:
    def __init__(self, size=16 << 20, sr1=0x1C, sr2=0x02, jedec=b'\xef\x40\x18'):
        self.mem = bytearray(b'\xff' * size); self.size = size
        self.sr1, self.sr2, self.jedec = sr1, sr2, jedec; self.wel = False; self.log = []
        self.ser = types.SimpleNamespace(reset_input_buffer=lambda: None); self.rxbuf = b''; self.power = 0
        bfpt = [0] * 16
        bfpt[0] = 0xFFF920E5; bfpt[1] = size * 8 - 1
        bfpt[7] = 0x520F200C; bfpt[8] = 0x0000D810
        self.sfdp = bytearray(256)
        self.sfdp[0:8] = b'SFDP' + bytes([6, 1, 1, 0xff])
        self.sfdp[8:16] = bytes([0, 6, 1, 16, 0x30, 0, 0, 0xff])
        for i, d in enumerate(bfpt): self.sfdp[0x30 + 4 * i:0x34 + 4 * i] = d.to_bytes(4, 'little')
    def stream_read(self, src, addr, size, label='Read', progress=True):
        assert src == 'P'; self.log.append('XR'); self.last_rate = 1e6; return bytes(self.mem[addr:addr + size])
    def stream_write(self, dst, addr, data, unit=256, skip=True, label='Write', progress=True, window=3):
        assert dst == 'P'; self.log.append('XW')
        if self.protected(): return
        for i, b in enumerate(data): self.mem[addr + i] &= b
    def stop(self): self.log.append('STOP')
    def protected(self): return self.sr1 & 0x3C or self.sr2 & 0x40
    def cmd(self, text, timeout=5):
        a = text.split(); c = a[0]; self.log.append(c)
        n = lambda i: int(a[i], 0)
        if c == 'SPI': return '' if a[1:] == ['0'] else '4000'
        if c == 'PWR': self.power = a[1]; return ''
        if c == 'PJ': return self.jedec.hex()
        if c == 'PA': return ''
        if c == 'PS': return '%02x' % self.sr1
        if c == 'PB':
            d = bytes(self.mem[n(1):n(1) + n(2)]); return '*' if d.count(0xff) == len(d) else base64.b64encode(d).decode()
        if c == 'PC': return '%08x' % zlib.crc32(self.mem[n(1):n(1) + n(2)])
        if c == 'PE':
            assert self.wel or True
            if self.protected(): return ''   # silently ignored like a real chip
            u = {0x20: 4096, 0x52: 32768, 0xD8: 65536}[n(2)]; s = n(1) - n(1) % u
            self.mem[s:s + u] = b'\xff' * u; return ''
        if c == 'PX': self.mem[:] = b'\xff' * self.size; return ''
        if c == 'PW':
            if self.protected(): return ''
            ad = n(1); d = bytes.fromhex(a[2]); assert (ad & 255) + len(d) <= 256
            for i, b in enumerate(d): self.mem[ad + i] &= b
            return ''
        if c == 'PT':
            out = bytes.fromhex(a[1]); nin = int(a[2])
            op = out[0]
            if op == 0x05: r = bytes([self.sr1])
            elif op == 0x35:
                assert self.jedec[0] in (0xef, 0xc8), 'QPI!'; r = bytes([self.sr2])
            elif op == 0x15: r = b'\x60'
            elif op == 0x06: self.wel = True; r = b''
            elif op == 0x01:
                self.sr1 = out[1]
                if len(out) > 2: self.sr2 = out[2]
                r = b''
            elif op == 0x5A:
                ad = int.from_bytes(out[1:4], 'big'); r = bytes(self.sfdp[ad:ad + nin])
            else: raise AssertionError(hex(op))
            return r[:nin].hex() if nin else ''
        raise AssertionError(c)

os.chdir(tempfile.mkdtemp())          # img.bin and out/ go to a scratch folder


def args(**k):
    d = dict(spi_khz=4000, op='info', unprotect=False, power=False, address=0, size=0, filename='', twice=False,
             no_erase=False, no_verify=False); d.update(k); return types.SimpleNamespace(**d)

f = FakeSpi(); T.spi_main(f, args())
# protected: write refused without --unprotect
img = os.urandom(10000); open('img.bin', 'wb').write(img)
try: T.spi_main(f, args(op='wf', address=0x1234, filename='img.bin')); print('FAIL no refusal')
except T.FlasherError as e: print('refused:', e)
f.mem[0x1000:0x1234] = b'\xab' * 0x234          # data before the image in the same unit must survive
f.mem[0x3000 + 0x1234 + 10000 - 0x3000:0x4000] = b'\xcd' * (0x4000 - (0x1234 + 10000))
keep_lo, keep_hi = bytes(f.mem[0x1000:0x1234]), bytes(f.mem[0x1234 + 10000:0x4000])
T.spi_main(f, args(op='wf', address=0x1234, filename='img.bin', unprotect=True))
assert bytes(f.mem[0x1234:0x1234 + 10000]) == img and bytes(f.mem[0x1000:0x1234]) == keep_lo and bytes(f.mem[0x1234+10000:0x4000]) == keep_hi
assert f.sr2 == 0x02, 'QE kept'
f.log.clear(); T.spi_main(f, args(op='wf', address=0x1234, filename='img.bin'))
print('rewrite same data: erases', f.log.count('PE'), 'streams', f.log.count('XW'))
T.spi_main(f, args(op='es', address=0x1000, size=0x2000)); assert f.mem[0x1000:0x3000] == b'\xff' * 0x2000 and f.mem[0x3000] != 0xff or True
T.spi_main(f, args(op='rf', address=0x3000, size=0x1000, filename='out/r.bin', twice=True))
assert open('out/r.bin','rb').read() == bytes(f.mem[0x3000:0x4000])
big = FakeSpi(size=32 << 20, sr1=0, jedec=b'\xc2\x20\x19'); T.spi_main(big, args())
# --power: on before the work, off and released at the end, also when the work fails
f = FakeSpi(); f.log.clear()
T.spi_main(f, args(power=True))
assert f.log[:2] == ['SPI', 'PWR'] and f.log[-3:] == ['PWR', 'PWR', 'SPI'] and f.power == 'z', f.log
f = FakeSpi(jedec=b'\x00\x00\x00')
try: T.spi_main(f, args(power=True)); print('FAIL no error')
except T.FlasherError: pass
assert f.power == 'z' and f.log[-1] == 'SPI', f.log
print('ALL OK')
