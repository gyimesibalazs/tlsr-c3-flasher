"""LN882H mode (--ln) with Lightning's RAM loader (--ln-loader vendor): tlsr_c3.py against the model of the flasher's
bridge and the chip in ln_sim.py (the bridge's speed following, the ROM, the vendor loader as its SDK source behaves).
Our own loader is tested in test_ln_loader.py. Also builds src/lnscan.h natively and checks it on its own."""
import os, subprocess, sys, tempfile, types
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ln_sim import T, Sim, flasher, ROOT
from lnscan_model import LnScan


tmp = tempfile.mkdtemp()
RAMCODE = os.path.join(tmp, 'ramcode.bin')
RAMCODE_DATA = bytes(range(256)) * 147 + b'\x01\x02'      # 37634 bytes, not a multiple of 128
with open(RAMCODE, 'wb') as fh:
    fh.write(RAMCODE_DATA)


def args(op, **kw):
    a = types.SimpleNamespace(op=op, ln_baud=0, ln_loader='vendor', ln_ramcode=RAMCODE, ln_seconds=1, run=False,
                              address=0, size=0, filename='', twice=False, no_verify=False, no_erase=False, bk_sr=0)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def run(sim, a):
    T.ln_main(flasher(sim), a)
    assert not sim.violations, sim.violations
    assert sim.mode == 'cmd', 'the bridge was left open'


# ---- src/lnscan.h natively, against the Python model, on the same streams
cases = [b'baudrate 1000000\r', b'version\r\nbaudrate 460800\n', b'xbaudrate 9600\r', b'baudrate 12x\r',
         b'\x01baudrate 921600\r', b'\rbaudrate 921600\r', b'baudrate 100\r', b'baudrate 99999999999\r',
         b'baudrate  115200\r', b'baudrate 115200', b'BAUDRATE 115200\r', b'\x00\xffbaudrate 2000000\n',
         b'baudrate 3000000\r\nbaudrate 115200\r', b'baudrate 115200 0 1\r']
UCFG = [b'ucfg 1500000 0 1\n', b'ucfg 921600\n', b'ucfg 1500000x 0 1\n', b'ping\nucfg 2000000 4 1\r',
        b'\x02ucfg 1500000 0 1\n', b'ucfg  1500000\n', b'baudrate 115200\n']
cases = [(c, 0) for c in cases] + [(c, 1) for c in UCFG]
src = os.path.join(tmp, 'lnscan_test.cpp')
with open(src, 'w') as fh:
    fh.write('#include "lnscan.h"\n#include <stdio.h>\n#include <string.h>\nint main() {\n'
             '  static const char *cases[] = {%s};\n'
             '  static const int lens[] = {%s};\n'
             '  static const int ucfg[] = {%s};\n'
             '  for (int k = 0; k < %d; k++) { LineBaudScan s = ucfg[k] ? LineBaudScan("ucfg ", true) :'
             ' LineBaudScan(); for (int i = 0; i < lens[k]; i++) {'
             ' unsigned b = s.feed((uint8_t)cases[k][i]); if (b) printf("%%d:%%d:%%u ", k, i, b); } }\n'
             '  printf("\\n"); return 0; }\n' % (
                 ', '.join('"%s"' % ''.join('\\x%02x' % x for x in c) for c, _ in cases),
                 ', '.join(str(len(c)) for c, _ in cases), ', '.join(str(u) for _, u in cases), len(cases)))
exe = os.path.join(tmp, 'lnscan_test')
subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', os.path.join(ROOT, 'src'), '-o', exe, src],
               check=True)
native = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.split()
model = []
for k, (c, u) in enumerate(cases):
    s = LnScan(b'ucfg ', True) if u else LnScan()
    for i, b in enumerate(c):
        v = s.feed(b)
        if v:
            model.append('%d:%d:%d' % (k, i, v))
assert native == model, (native, model)
assert model == ['0:16:1000000', '1:24:460800', '5:16:921600', '12:16:3000000', '12:33:115200',
                 '14:16:1500000', '15:11:921600', '17:21:2000000'], model
print('lnscan.h: %d cases, native = model' % len(cases))

# ---- info: ROM, RAM loader over YMODEM, speed raised to 1000000 by the bridge following the line
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('info'))
assert sim.log[0] == 'BRIDGE 115200 lo ln', sim.log
assert 'ymodem ram %d bytes' % len(RAMCODE_DATA) in sim.log
assert sim.bridge_baud == sim.target_baud == 1000000
assert 'ramcode: baudrate 1000000' in sim.log
print('info: ok')

# ---- rf of a range and the whole flash, --twice; a bad CRC once is read again
sim = Sim(ramcode=RAMCODE_DATA, bad_crc_once=True)
out = os.path.join(tmp, 'part.bin')
run(sim, args('rf', address=0x1234, size=0x3456, filename=out, twice=True))
assert open(out, 'rb').read() == bytes(sim.flash[0x1234:0x1234 + 0x3456])
sim = Sim(size=0x100000, ramcode=RAMCODE_DATA)
run(sim, args('rf', address=0, size=0, filename=out))
assert open(out, 'rb').read() == bytes(sim.flash)
print('rf: ok')

# ---- wf: unaligned start and end; what is around the range in its first and last sectors stays
sim = Sim(ramcode=RAMCODE_DATA)
before = bytes(sim.flash)
img = os.urandom(0x2345)
fn = os.path.join(tmp, 'img.bin')
with open(fn, 'wb') as fh:
    fh.write(img)
run(sim, args('wf', address=0x10100, filename=fn))
assert bytes(sim.flash[0x10100:0x10100 + len(img)]) == img
assert bytes(sim.flash[:0x10100]) == before[:0x10100]
assert bytes(sim.flash[0x10100 + len(img):]) == before[0x10100 + len(img):]
print('wf: ok')

# ---- es, ea
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('es', address=0x5010, size=0x10))
assert all(b == 0xff for b in sim.flash[0x5000:0x6000])
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('ea'))
assert all(b == 0xff for b in sim.flash)
print('es, ea: ok')

# ---- the chip's UART fails above 460800: a new session at 460800
sim = Sim(ramcode=RAMCODE_DATA, fail_above=460800)
run(sim, args('info'))
assert sim.log.count('BRIDGE 115200 lo ln') == 2 and sim.target_baud == 460800, sim.log
print('fallback to 460800: ok')

# ---- BOOT not wired: the chip runs its firmware, the ROM never answers; the bridge is left
sim = Sim(ramcode=RAMCODE_DATA, boot_wired=False)
try:
    T.ln_main(flasher(sim), args('info'))
    raise AssertionError('no error')
except T.FlasherError as e:
    assert 'no answer from the ROM' in str(e), e
assert sim.mode == 'cmd' and sim.log.count('BRIDGE 115200 lo ln') == 1
print('no ROM: ok')

# ---- data with a "baudrate N" line is refused before anything is written
sim = Sim(ramcode=RAMCODE_DATA)
with open(fn, 'wb') as fh:
    fh.write(b'\x00' * 100 + b'\nbaudrate 9600\r\n' + b'\x00' * 100)
before = bytes(sim.flash)
try:
    run(sim, args('wf', address=0x20000, filename=fn))
    raise AssertionError('no error')
except T.FlasherError as e:
    assert 'baudrate' in str(e), e
assert bytes(sim.flash) == before and sim.mode == 'cmd'
print('baudrate line in the data refused: ok')

# ---- --run: restart with BOOT released
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('info', run=True))
assert sim.log[-1] == 'RST' and sim.target == 'app'
print('--run: ok')
print('all LN882H host tests passed')
