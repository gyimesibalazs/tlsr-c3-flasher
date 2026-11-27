"""LN882H mode with our RAM loader (loaders/ln882h, --ln-loader own, the default).

1. The build: the loader embedded in tlsr_c3.py equals a fresh build (needs PlatformIO's Arm toolchain, skipped with
   a message without it); its boot header is complete; build.py's header code reproduces the header of Lightning's
   loader binary when that is at hand ($LN882H_RAMCODE, ln882h_ramcode.bin next to tlsr_c3.py, or ltchiptool's).
2. The loader's protocol and flash logic (loader.c, flash.c built natively against tests/ln_loader_sim/fake_hal.c and
   the SPI NOR model of ln_nor_model.py) driven by tlsr_c3.py's real --ln code through the model of the bridge and
   the ROM in ln_sim.py: info, rf (a range, the whole flash, --twice), wf with an unaligned start and end (what is
   around it kept), the same image again (nothing erased or written), blank sectors (not erased), --no-erase, data
   with a "baudrate" line, es, ea, a protected flash (cleared, QE kept; refused for an unknown flash), a damaged
   frame in each direction, the speed fallback, a flash that does not answer, a loader that does not start, --run.
3. Frames sent straight to the native loader: bad CRC, bad escape, a raw CR in a frame, unknown command, a range
   beyond the flash, an unaligned erase, a write over data that is not erased, a repeated write; and the host's
   messages for a fault line, a restarted loader and a loader that does not answer after the load."""
import hashlib, importlib.util, os, sys, tempfile, types, zlib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ln_sim import T, Sim, Nor, NativeLoader, flasher, check_violations, LOADER_DIR, ROOT

spec = importlib.util.spec_from_file_location('ln_build', os.path.join(LOADER_DIR, 'build.py'))
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)
tmp = tempfile.mkdtemp()

# ---- 1. the build and the boot header
cc = build.compiler()
if cc:
    image, top = build.build(cc)
    assert image == T.LN_LOADER, 'tlsr_c3.py holds another build of the loader: run loaders/ln882h/build.py'
    print('build: the embedded loader equals a fresh build (%d bytes, RAM up to 0x%x)' % (len(image), top))
else:
    print('build: SKIPPED, arm-none-eabi-gcc not found (PlatformIO\'s toolchain or $LN882H_CC)')
h = build.header_fields(T.LN_LOADER)
assert h['load'] == 0x20000000 and h['vectors'] == 0x20000100 and h['crc_offset'] == 0x20 and h['crp'] == 0, h
assert h['length_ok'] and h['crc_ok'] and h['header_crc_ok'], h
assert build.fill_header(T.LN_LOADER) == T.LN_LOADER
assert not T.LN_BAUD_LINE.search(T.LN_LOADER)
vendor = None
for c in [os.environ.get('LN882H_RAMCODE', ''), os.path.join(ROOT, T.LN_RAMCODE_NAME)]:
    if c and os.path.isfile(c):
        vendor = open(c, 'rb').read()
        break
else:
    try:
        spec = importlib.util.find_spec('ltchiptool')
        c = spec and os.path.join(os.path.dirname(spec.origin), 'soc', 'ln882h', 'util', 'ramcode.bin')
        if c and os.path.isfile(c):
            vendor = open(c, 'rb').read()
    except (ImportError, ValueError):
        pass
if vendor and hashlib.sha256(vendor).hexdigest() == T.LN_RAMCODE_SHA256:
    v = build.header_fields(vendor)
    assert v['length_ok'] and v['crc_ok'] and v['header_crc_ok'], v
    blank = bytearray(vendor)
    blank[4:6] = b'\xff\xff'
    blank[8:12] = blank[20:24] = b'\xff' * 4
    assert build.fill_header(bytes(blank)) == vendor
    print('header: our header code gives the vendor loader\'s header (length, both CRCs) from a blanked one')
else:
    print('header: vendor loader binary not at hand ($LN882H_RAMCODE): its header check SKIPPED; ours is complete')


# ---- 2. tlsr_c3.py --ln through the bridge model against the native loader
def args(op, **kw):
    a = types.SimpleNamespace(op=op, ln_baud=0, ln_loader='own', ln_ramcode='', ln_seconds=1, run=False, address=0,
                              size=0, filename='', twice=False, no_verify=False, no_erase=False, bk_sr=0)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def run(sim, a):
    T.ln_main(flasher(sim), a)
    check_violations(sim)
    assert sim.mode == 'cmd', 'the bridge was left open'


def fails(sim, a, text):
    try:
        T.ln_main(flasher(sim), a)
    except T.FlasherError as e:
        assert text in str(e), e
        assert sim.mode == 'cmd', 'the bridge was left open'
        return str(e)
    raise AssertionError('no error')


def image_file(data):
    fn = os.path.join(tmp, 'img.bin')
    with open(fn, 'wb') as fh:
        fh.write(data)
    return fn


sim = Sim()
run(sim, args('info'))
assert sim.log[0] == 'BRIDGE 115200 lo ln', sim.log
assert 'ymodem ram %d bytes' % len(T.LN_LOADER) in sim.log
assert sim.bridge_baud == sim.target_baud == 1000000 and 'own: baud 1000000' in sim.log
# start, banner drained, stage 2 drained, cache off, stage 3 drained, QSPI set-up: each stage out before it runs
assert sim.native.events[:6] == [0, 1, 1, 2, 1, 3], sim.native.events
print('info: ok')

sim = Sim()
out = os.path.join(tmp, 'part.bin')
run(sim, args('rf', address=0x1234, size=0x3456, filename=out, twice=True))
assert open(out, 'rb').read() == bytes(sim.flash[0x1234:0x1234 + 0x3456])
sim = Sim(nor=Nor(size=0x100000, jedec=0xEB6014))
run(sim, args('rf', filename=out))
assert open(out, 'rb').read() == bytes(sim.nor.mem)
print('rf: ok')

sim = Sim()
before = bytes(sim.nor.mem)
img = os.urandom(0x12345)
run(sim, args('wf', address=0x10100, filename=image_file(img)))
end = 0x10100 + len(img)
assert bytes(sim.nor.mem[0x10100:end]) == img
assert bytes(sim.nor.mem[:0x10100]) == before[:0x10100] and bytes(sim.nor.mem[end:]) == before[end:]
assert sim.nor.counts['erase64k'] == 1 and sim.nor.counts['erase4k'] == 3, sim.nor.counts   # 0x20000 is whole
counts = dict(sim.nor.counts)
run(sim, args('wf', address=0x10100, filename=image_file(img)))
assert sim.nor.counts['program'] == counts['program'] and sim.nor.counts['erase4k'] == counts['erase4k'] and \
    sim.nor.counts['erase64k'] == counts['erase64k'], (counts, sim.nor.counts)
print('wf (unaligned, neighbours kept; the same image again: nothing erased or written): ok')

sim = Sim()
sim.nor.mem[0x40000:0x43000] = b'\xff' * 0x3000
img = os.urandom(0x2000) + b'\xff' * 0x1000                          # the last sector stays blank
run(sim, args('wf', address=0x40000, filename=image_file(img)))
assert bytes(sim.nor.mem[0x40000:0x43000]) == img and sim.nor.counts['erase4k'] == 0, sim.nor.counts
sim.nor.mem[0x50000:0x51000] = b'\xff' * 0x1000
sim.nor.mem[0x50800] = 0x00
fails(sim, args('wf', address=0x50000, filename=image_file(b'\x5a' * 0x1000), no_erase=True), 'not erased')
run(sim, args('wf', address=0x50000, filename=image_file(b'\x5a' * 0x800), no_erase=True))
assert bytes(sim.nor.mem[0x50000:0x50800]) == b'\x5a' * 0x800
print('wf (blank sectors not erased, --no-erase): ok')

sim = Sim()
data = b'\x00' * 100 + b'\nbaudrate 9600\r\n' + b'\x0d\x0a\xc0\xc1\xdb' * 20 + b'\x00' * 100
run(sim, args('wf', address=0x20010, filename=image_file(data)))
assert bytes(sim.nor.mem[0x20010:0x20010 + len(data)]) == data and sim.bridge_baud == 1000000
print('data with a "baudrate" line and the frame bytes written (the frames cannot hold such a line): ok')

sim = Sim()
run(sim, args('es', address=0x5010, size=0x10))
assert all(b == 0xff for b in sim.nor.mem[0x5000:0x6000]) and sim.nor.counts['erase4k'] == 1
sim = Sim()
run(sim, args('ea'))
assert all(b == 0xff for b in sim.nor.mem) and sim.nor.counts['erase64k'] == 32
print('es, ea: ok')

# protected: BP bits and CMP set; a flash in the list is cleared with both status bytes, QE (0x0200) stays
sim = Sim(nor=Nor(sr=0x423C))
img = os.urandom(0x1800)
run(sim, args('wf', address=0x3000, filename=image_file(img)))
assert bytes(sim.nor.mem[0x3000:0x4800]) == img and (sim.nor.sr1, sim.nor.sr2) == (0x00, 0x02), (sim.nor.sr1,
                                                                                                  sim.nor.sr2)
sim = Sim(nor=Nor(jedec=0x123415, sr=0x001C))
before = bytes(sim.nor.mem)
fails(sim, args('es', address=0, size=0x1000), 'not in the list')
assert bytes(sim.nor.mem) == before
sim = Sim(nor=Nor(jedec=0x123415, sr=0x001C))
run(sim, args('es', address=0, size=0x1000, bk_sr=1))
assert sim.nor.mem[:0x1000] == b'\xff' * 0x1000
print('protected flash (cleared, QE kept; unknown flash refused, then --bk-sr): ok')

# a damaged request and a damaged answer, also the answer of a write: sent again, the write not done twice
# frames of a write of 3 sectors at 0x7000: info, CRC of the range, 3 sector CRCs, 3 erases, 3 writes, the verify
for corrupt in [dict(corrupt_in=3), dict(corrupt_out=3), dict(corrupt_out=7), dict(corrupt_in=10),
                dict(corrupt_out=10)]:
    sim = Sim(**corrupt)
    img = os.urandom(0x3000)
    run(sim, args('wf', address=0x7000, filename=image_file(img)))
    assert bytes(sim.nor.mem[0x7000:0xa000]) == img
    assert any(x.startswith('corrupted') for x in sim.log), corrupt
    assert sim.nor.counts['program'] == 48, sim.nor.counts
print('a damaged frame each way (recovered, nothing done twice): ok')

sim = Sim(fail_above=460800)
run(sim, args('info'))
assert sim.log.count('BRIDGE 115200 lo ln') == 2 and sim.target_baud == 460800, sim.log
print('fallback to 460800: ok')

sim = Sim(nor=Nor(jedec=0xFFFFFF))
run(sim, args('info'))
e = fails(sim, args('rf', filename=out), 'could not set up the flash')
assert 'stage 4, error 13' in e, e
print('no flash (init stops at stage 4, error 13): ok')

sim = Sim(silent=True)
e = fails(sim, args('info'), 'our RAM loader does not answer')
assert '--ln-loader vendor' in e and 'tlsr_c3.py log' in e and sim.log.count('BRIDGE 115200 lo ln') == 1, (e, sim.log)
print('a loader that does not start: one session, a clear message: ok')

sim = Sim(own_baud=921600)
run(sim, args('info'))
assert sim.log.count('BRIDGE 115200 lo ln') == 1 and sim.target_baud == 921600, sim.log
print('a loader whose UART comes up at 921600: found by listening there, worked with at that speed: ok')

sim = Sim(deaf=True)
e = fails(sim, args('info'), 'nothing the host sent reached it')
assert 'wait rx 0 err 0 ready' in e and sim.log.count('BRIDGE 115200 lo ln') == 1, (e, sim.log)
print('a loader that talks but hears nothing: its wait lines, no speed probing, a clear message: ok')

sim = Sim()
run(sim, args('info', run=True))
assert sim.log[-1] == 'RST' and sim.target == 'app'
print('--run: ok')


# ---- 3. frames straight to the native loader
def frame(cmd, data=b'', seq=7):
    return T.ln_frame(bytes([cmd, seq, 0]) + len(data).to_bytes(2, 'little') + data)


def ask(sim, raw):
    """Bytes to the started loader; its frames back as (type, seq, status, data)."""
    sim.out.clear()
    for b in raw:
        sim.native.feed(b)
    sim.native.poll()
    got, buf = [], bytes(sim.out)
    while T.LN_FR_END in buf:
        s, e = buf.index(T.LN_FR_START), buf.index(T.LN_FR_END)
        body = T.ln_unescape(buf[s + 1:e])
        assert zlib.crc32(body[:-4]) == int.from_bytes(body[-4:], 'little')
        got.append((body[0], body[1], body[2], body[5:-4]))
        buf = buf[e + 1:]
    return got


sim = Sim()
T.ln_connect(flasher(sim), args('info'), 115200, say=lambda *a: None)
assert sim.target == 'own'
det = lambda d: int.from_bytes(d, 'little')
bad = bytearray(frame(T.LN_INFO))
bad[3] ^= 1
assert ask(sim, bad)[0][:3] == (0xFF, 7, 1)                                         # CRC
assert ask(sim, b'\xc0\x01\x07\x00\x00\x00\xdb\x41\x00\x00\x00\xc1')[0][:3] == (0xFF, 7, 2)   # escape
assert ask(sim, b'\xc0\x01\x07\x00\r')[0][:3] == (0xFF, 7, 2)                        # raw CR: the frame ends
assert ask(sim, b'\xc0\x01\x07')[:1] == [] and ask(sim, frame(T.LN_STATUS))[0][:3] == (0x86, 7, 0)  # C0 restarts
assert ask(sim, frame(0x33))[0][:3] == (0xB3, 7, 3)
assert ask(sim, frame(T.LN_READ, (0x1FFFF0).to_bytes(4, 'little') + (0x20).to_bytes(4, 'little')))[0][2] == 5
assert ask(sim, frame(T.LN_READ, bytes(4) + (4097).to_bytes(4, 'little')))[0][2] == 4
assert ask(sim, frame(T.LN_ERASE, (0x1100).to_bytes(4, 'little') + (0x1000).to_bytes(4, 'little')))[0][2] == 6
sim.nor.mem[0x8000:0x8010] = b'\xff' * 8 + b'\x0f' * 8
before = bytes(sim.nor.mem)
r = ask(sim, frame(T.LN_WRITE, (0x8000).to_bytes(4, 'little') + b'\x33' * 16))[0]
assert r[2] == 7 and det(r[3]) == 0x8008 and bytes(sim.nor.mem) == before, r
w = frame(T.LN_WRITE, (0x8000).to_bytes(4, 'little') + b'\x33' * 8, seq=9)
n0 = sim.nor.counts['program']
assert ask(sim, w)[0][:3] == (0x85, 9, 0) and ask(sim, w)[0][:3] == (0x85, 9, 0)    # the repeat is not done again
assert sim.nor.counts['program'] == n0 + 1 and sim.nor.mem[0x8000:0x8008] == b'\x33' * 8
assert ask(sim, frame(T.LN_STATUS, seq=10))[0][2] == 0
assert ask(sim, w)[0][:3] == (0x85, 9, 0) and sim.nor.counts['program'] == n0 + 2   # after another request: new
c = ask(sim, frame(T.LN_CRC, (0x1234).to_bytes(4, 'little') + (0x23456).to_bytes(4, 'little')))[0]
assert det(c[3]) == zlib.crc32(bytes(sim.nor.mem[0x1234:0x1234 + 0x23456]))
sim.out.clear()
for b in b'version\r\n':
    sim.native.feed(b)
sim.native.poll()
assert bytes(sim.out) == b'C3LN 2 ready\r\n', bytes(sim.out)
check_violations(sim)
print('frames straight to the loader (errors, resync, repeat, CRC, version): ok')

# the start lines and the wait line: until a valid line or frame came, every 200 ms (154 polls of the fake's clock)
sim = Sim()
sim.native = T_native = NativeLoader(sim, sim.nor)
T_native.lib.sim_cycles_step(208333)
sim.mode, sim.target = 'bridge', 'own'
T_native.start()
assert bytes(sim.out).startswith(b'\r\nC3LN 2 rom clock\r\n\r\nC3LN 2 init 1 2 3 4 5 ready\r\n'), bytes(sim.out)
sim.out.clear()
for _ in range(160):
    T_native.lib.loader_poll()
assert bytes(sim.out) == b'\r\nC3LN 2 wait rx 0 err 0 ready\r\n', bytes(sim.out)
sim.out.clear()
for b in b'hello\r':                                  # not a valid line: still waiting, the bytes counted
    T_native.feed(b)
for _ in range(160):
    T_native.lib.loader_poll()
assert bytes(sim.out) == b'\r\nC3LN 2 wait rx 6 err 0 ready\r\n', bytes(sim.out)
sim.out.clear()
for b in b'version\r':
    T_native.feed(b)
for _ in range(400):
    T_native.lib.loader_poll()
assert bytes(sim.out) == b'C3LN 2 ready\r\n', bytes(sim.out)
print('start lines, the wait line every 200 ms until "version" came: ok')


class Canned:
    """A port that answers anything with the given bytes."""
    def __init__(self, data):
        self.data = bytearray(data)
        self.in_waiting = 0

    def write(self, d):
        self.in_waiting = len(self.data)

    def read(self, n=1):
        d = bytes(self.data[:n])
        del self.data[:n]
        self.in_waiting = len(self.data)
        return d


for text, want in [(b'\r\nC3LN fault 03 pc 20000abc lr 20000def\r\n', 'C3LN fault 03 pc 20000abc'),
                   (b'\r\nC3LN 2 init 1 2 3 4 5 ready\r\n', 'started again')]:
    try:
        T.LnLink(Canned(text)).call(T.LN_STATUS, timeout=0.2, tries=1)
        raise AssertionError('no error')
    except T.FlasherError as e:
        assert want in str(e), e
print('a fault line or a new start banner during a command: reported: ok')
for seen, want in [(b'\r\nC3LN 2 init 1 2 3', 'stopped in init stage 3 (QSPI set-up)'),
                   (b'\r\nC3LN 2 init 1 2 3 4 5 ready\r\n', 'then fell silent'),
                   (b'\r\nC3LN 2 init 1\r\nC3LN fault 03 pc 20000123 lr 20000456\r\n', 'exception'),
                   (b'\r\nC3LN 2 rom clock\r\n\xff\x13', 'stopped there, or UART0 now runs at another speed'),
                   (b'\r\nC3LN 2 init 1 2 3 4 5 ready\r\n\r\nC3LN 2 wait rx 0 err 0 ready\r\n',
                    'nothing the host sent reached it'),
                   (b'\r\nC3LN 2 wait rx 27 err 9 ready\r\n', '27 bytes with 9 receive errors'),
                   (b'\r\nC3LN 2 wait rx 9 err 0 error 13 at 4\r\n', 'no valid "version" line'),
                   (b'\xff\x00\xff', 'nothing came'), (b'garbage', "came only 7 bytes of something else")]:
    e = T.ln_own_silent(types.SimpleNamespace(seen=seen))
    assert want in e and 'tlsr_c3.py log' in e and '--ln-loader vendor' in e, e
print('the messages for a loader that does not answer: ok')
print('all LN882H loader tests passed')
