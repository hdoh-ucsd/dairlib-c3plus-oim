#!/usr/bin/env python3
"""Minimal UDPM LCM listener: prints object-state poses (no lcm module needed).
Usage: watch_object_state.py <port> <dur_s>"""
import socket, struct, sys, time

port = int(sys.argv[1])
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
GRP = '239.255.76.67'

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', port))
mreq = struct.pack('=4sl', socket.inet_aton(GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
sock.settimeout(0.5)

def decode_object_state(data):
    o = 8  # fingerprint
    utime = struct.unpack_from('>q', data, o)[0]; o += 8
    n = struct.unpack_from('>i', data, o)[0]; o += 4
    name = data[o:o+n-1].decode(); o += n
    npos = struct.unpack_from('>i', data, o)[0]; o += 4
    nvel = struct.unpack_from('>i', data, o)[0]; o += 4
    for _ in range(npos):
        n = struct.unpack_from('>i', data, o)[0]; o += 4 + n
    pos = struct.unpack_from('>%dd' % npos, data, o)
    return utime, name, pos

last = {}
t0 = time.time()
while time.time() - t0 < dur:
    try:
        pkt, _ = sock.recvfrom(65535)
    except socket.timeout:
        continue
    if len(pkt) < 8 or pkt[:4] != b'\x4c\x43\x30\x32':  # LC02 short message
        continue
    o = 8
    end = pkt.index(b'\x00', o)
    chan = pkt[o:end].decode(errors='replace')
    if not chan.startswith('OBJECT'):
        continue
    payload = pkt[end+1:]
    try:
        utime, name, pos = decode_object_state(payload)
    except Exception:
        continue
    t = utime * 1e-6
    if t - last.get(chan, -1) >= 0.5:
        last[chan] = t
        print('t=%7.2f %s %s q=%s' % (t, chan, name,
              ' '.join('%.4f' % p for p in pos)), flush=True)
