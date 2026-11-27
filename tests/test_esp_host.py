"""ESP32 / ESP8266 mode (--esp): the sequence around esptool and the esptool arguments, with a fake flasher and a
fake esptool module (no hardware, no esptool needed)."""
import os, sys, types, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tlsr_c3 as T

log = []


class FakeFlasher:
    def __init__(self, port='COM9', baud=0, key=None):
        self.ser = types.SimpleNamespace(close=lambda: log.append('close'))
    def cmd(self, text, timeout=5):
        log.append(text)
        return 'bridge %s' % text.split()[1] if text.startswith('BRIDGE') else ''


def fake_esptool(version, fail=0, write_files=None):
    """fail: the number of runs that fail first (True: all)"""
    m = types.ModuleType('esptool')
    m.__version__ = version
    left = [fail]
    def main(argv):
        log.append(('esptool',) + tuple(argv))
        if left[0] is True or left[0] > 0:
            if left[0] is not True:
                left[0] -= 1
            raise RuntimeError('Failed to connect')
        if write_files is not None and argv[-4].startswith('read'):
            with open(argv[-1], 'wb') as out:
                out.write(write_files.pop(0))
    m.main = main
    return m


def args(op, **kw):
    a = types.SimpleNamespace(op=op, port='COM9', esp_baud=0, esp_chip='', esp_stay=False,
                              address=0, size=0, filename='', twice=False, no_verify=False, no_erase=False)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def run(a, esptool, follow=True):
    log.clear()
    sys.modules['esptool'] = esptool
    T.Flasher = FakeFlasher
    T.bridge_exit = lambda port, key=None: (log.append('bridge-exit'), 'OK bridge closed')[1]
    T.esp_main(FakeFlasher(), a, follow=follow)
    return list(log)


def esptool_calls(seq):
    return [c[1:] for c in seq if isinstance(c, tuple)]


# info, esptool 5: download mode first with the bridge at 115200 following esptool's speed change to 921600,
# esptool without resets, bridge closed, then a plain restart
seq = run(args('info'), fake_esptool('5.1.0'))
assert seq[0] == 'BRIDGE 115200 lo esp' and seq[1] == 'close', seq
assert esptool_calls(seq) == [('--port', 'COM9', '--baud', '921600', '--before', 'no-reset', '--after', 'no-reset',
                               'flash-id')], seq
assert seq[-3:] == ['bridge-exit', 'RST', 'close'], seq

# firmware older than 0.25: the bridge runs at the esptool speed from the start, 115200 by default
seq = run(args('info'), fake_esptool('5.1.0'), follow=False)
assert seq[0] == 'BRIDGE 115200 lo' and esptool_calls(seq)[0][3] == '115200', seq

# esptool 4 spells with underscores; chip and speed are passed on
seq = run(args('wf', address=0x1000, filename='fw.bin', esp_baud=460800, esp_chip='esp8266'), fake_esptool('4.8.1'),
          follow=False)
assert seq[0] == 'BRIDGE 460800 lo', seq
assert esptool_calls(seq) == [('--port', 'COM9', '--baud', '460800', '--before', 'no_reset', '--after', 'no_reset',
                               '--chip', 'esp8266', 'write_flash', '0x1000', 'fw.bin')], seq

# es is widened to whole sectors; ea; --esp-stay leaves the target in download mode
seq = run(args('es', address=0x1234, size=0x10), fake_esptool('5.0.0'))
assert esptool_calls(seq)[0][-3:] == ('erase-region', '0x1000', '0x1000'), seq
seq = run(args('ea', esp_stay=True), fake_esptool('5.0.0'))
assert esptool_calls(seq)[0][-1] == 'erase-flash' and seq[-1] == 'bridge-exit', seq

# rf: size 0 is ALL on esptool 5, refused on esptool 4 before anything is touched
seq = run(args('rf', size=0, filename='x.bin'), fake_esptool('5.0.0'))
assert esptool_calls(seq)[0][-4:] == ('read-flash', '0x0', 'ALL', 'x.bin'), seq
log.clear()
try:
    run(args('rf', size=0, filename='x.bin'), fake_esptool('4.8.1'))
    raise AssertionError('esptool 4 rf of size 0 must fail')
except T.FlasherError as e:
    assert 'esptool 5' in str(e) and log == [], (e, log)

# rf --twice compares the two reads and keeps one file
d = tempfile.mkdtemp()
fn = os.path.join(d, 'b.bin')
seq = run(args('rf', size=0x100, filename=fn, twice=True), fake_esptool('5.0.0', write_files=[b'a' * 256, b'a' * 256]))
assert len(esptool_calls(seq)) == 2 and os.path.exists(fn) and not os.path.exists(fn + '.2'), seq
try:
    run(args('rf', size=0x100, filename=fn, twice=True), fake_esptool('5.0.0', write_files=[b'a' * 256, b'b' * 256]))
    raise AssertionError('different reads must fail')
except T.FlasherError as e:
    assert 'differ' in str(e)

# a failed run is repeated at the next lower speed, the target restarted in download mode each time; the later runs
# stay at the speed that worked
seq = run(args('rf', size=0x100, filename=fn, twice=True), fake_esptool('5.0.0', fail=1, write_files=[b'a' * 256] * 2))
calls = esptool_calls(seq)
assert [c[3] for c in calls] == ['921600', '460800', '460800'], calls
assert seq.count('BRIDGE 115200 lo esp') == 3 and seq.count('bridge-exit') == 3, seq

# an esptool failure at every speed still closes the bridge and restarts the target, then reports the error
try:
    seq = run(args('info'), fake_esptool('5.0.0', fail=True))
    raise AssertionError('must fail')
except T.FlasherError as e:
    assert 'Failed to connect' in str(e) and log[-3:] == ['bridge-exit', 'RST', 'close'], (e, log)
    assert [c[3] for c in esptool_calls(log)] == ['921600', '460800', '115200'], log

# operations that do not exist in this mode
for op in ('selftest', 'pins'):
    try:
        run(args(op), fake_esptool('5.0.0'))
        raise AssertionError(op)
    except T.FlasherError:
        pass

print('esp host: all checks passed')

# --no-erase / --no-verify are refused: esptool always erases and verifies
for flag in ('no_erase', 'no_verify'):
    try:
        run(args('wf', filename='x.bin', **{flag: True}), fake_esptool('5.1.0'))
        raise AssertionError('accepted --%s' % flag.replace('_', '-'))
    except T.FlasherError as e:
        assert 'no --no-erase' in str(e), e
print('--no-erase / --no-verify refused: ok')
