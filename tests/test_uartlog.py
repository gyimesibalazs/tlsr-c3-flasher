"""UART log (firmware 0.25): src/uartlog.cpp built natively (tests/uartlog_sim) on a made-up session, its export
decoded by tlsr_c3.py. A short session must come back record for record; a long one (a catch and 2 MB read) must
keep its start for good, its end in the ring, and say where blocks were dropped. Needs g++."""
import os, sys, subprocess, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import tlsr_c3 as T

d = tempfile.mkdtemp()
exe = os.path.join(d, 'uartlog_sim')
subprocess.check_call(['g++', '-std=c++17', '-O2', '-o', exe, os.path.join(HERE, 'uartlog_sim', 'uartlog_sim.cpp'),
                       os.path.join(HERE, '..', 'src', 'uartlog.cpp')])


def sim(scenario):
    bin_, rec = os.path.join(d, scenario + '.bin'), os.path.join(d, scenario + '.txt')
    stats = subprocess.check_output([exe, scenario, bin_, rec]).decode()
    st = dict(kv.split('=') for kv in stats.split())
    recs = []
    t_first = None
    for line in open(rec):
        t, tag, hx = (line.split() + [''])[:3]
        t = int(t)
        t_first = t if t_first is None else t_first
        recs.append(((t - t_first) & 0xFFFFFFFF, int(tag), bytes.fromhex(hx)))
    return open(bin_, 'rb').read(), recs, st


def records(entries):
    return [e for e in entries if e[0] != 'gap']


# short: everything fits; a record that crosses a block end comes back in pieces with the same time, so the bytes
# are compared per direction, the events with their times
data, recs, st = sim('small')
entries = T.ulog_parse(data)
assert not any(e[0] == 'gap' for e in entries)
for tag in (1, 2):
    assert b''.join(p for t, g, p in entries if g == tag) == b''.join(p for t, g, p in recs if g == tag), tag
assert [(t, p) for t, g, p in entries if g == 3] == [(t, p) for t, g, p in recs if g == 3]
assert int(st['dropped']) == 0 and int(st['raw']) > 0
text = T.ulog_text(entries)
assert 'this 5-byte pattern 30 times in a row' in text and '# baud 921600' in text, text[:2000]
assert 'this 1-byte pattern' in text             # the erased flash

# long: 2 MB read; the start (catch, link, speed change) is kept, the end is in the ring, a gap between
data, recs, st = sim('long')
entries = T.ulog_parse(data)
gaps = [e for e in entries if e[0] == 'gap']
assert len(gaps) == 1 and gaps[0][1] == int(st['dropped']) > 0, (gaps, st)
events = [p for t, g, p in records(entries) if g == 3]
assert events[:2] == [b'Beken download, 115200 baud', b'target reset (catch)'] and events[-1] == b'end', events
assert b'baud 921600' in events
assert len(data) <= 112 * 1024 and int(st['stored']) == len(data)
# the end of the session is complete: what came from the target after the gap is the end of all it sent, with the
# same times
tail = records(entries[entries.index(gaps[0]) + 1:])
rx_tail = b''.join(p for t, g, p in tail if g == 2)
rx_all = b''.join(p for t, g, p in recs if g == 2)
assert len(rx_tail) > 50000 and rx_all.endswith(rx_tail[-50000:])
assert tail[-1][0] == recs[-1][0] and tail[-1][2] == b'end'
# Wi-Fi on: a store of 26 k; its first part shrinks so that the ring holds a block of the worst case. The start and
# the end of the session are still there.
long_st, long_data = st, data
data, recs, st = sim('long-wifi')
entries = T.ulog_parse(data)
assert int(st['cap']) == 26 * 1024 and len(data) <= 26 * 1024 and int(st['stored']) == len(data), st
assert records(entries)[0][2] == b'Beken download, 115200 baud' and records(entries)[-1][2] == b'end'
gaps = [e for e in entries if e[0] == 'gap']
assert len(gaps) == 1 and gaps[0][1] == int(st['dropped']) > 0, (gaps, st)
tail = records(entries[entries.index(gaps[0]) + 1:])
rx_tail = b''.join(p for t, g, p in tail if g == 2)
assert len(rx_tail) > 4000 and rx_all.endswith(rx_tail), len(rx_tail)
# less heap than the smallest store: no log, every block counted as dropped
data, recs, st = sim('long-nomem')
assert data == b'' and int(st['cap']) == 0 and int(st['dropped']) == int(st['blocks']) > 0, st
st, data = long_st, long_data

print('uartlog: all checks passed (long session: %s blocks, %s dropped, %s bytes of records, %d bytes stored)'
      % (st['blocks'], st['dropped'], st['raw'], len(data)))
