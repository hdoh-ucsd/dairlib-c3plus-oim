"""LCM wire decoding and isolated multicast diagnostics."""
import os
import struct
import time
import uuid

class Cur:
    def __init__(self, b):
        self.b, self.i = b, 8

    def i32(self):
        v = struct.unpack_from('>i', self.b, self.i)[0]; self.i += 4; return v

    def i64(self):
        v = struct.unpack_from('>q', self.b, self.i)[0]; self.i += 8; return v

    def s(self):
        n = self.i32()
        v = self.b[self.i:self.i + n - 1].decode(); self.i += n; return v

    def dbl(self, n=1):
        v = struct.unpack_from('>%dd' % n, self.b, self.i); self.i += 8 * n
        return list(v)

def decode_object_state(b):
    c = Cur(b)
    utime = c.i64(); name = c.s()
    npos = c.i32(); nvel = c.i32()
    [c.s() for _ in range(npos)]
    pos = c.dbl(npos)
    return utime, name, pos

def decode_robot_output(b):
    c = Cur(b)
    utime = c.i64()
    npos = c.i32(); nvel = c.i32(); neff = c.i32()
    [c.s() for _ in range(npos)]
    pos = c.dbl(npos)
    [c.s() for _ in range(nvel)]
    vel = c.dbl(nvel)
    eff = []
    if neff > 0:
        [c.s() for _ in range(neff)]
        eff = c.dbl(neff)
    return utime, pos, vel, eff


def multicast_url(port):
    return f"udpm://239.255.76.67:{port}?ttl=0"


def check_multicast():
    from pydrake.lcm import DrakeLcm
    lc = DrakeLcm(multicast_url(42000 + os.getpid() % 20000))
    channel = "DAIRLIB_DEPENDENCY_CHECK_" + uuid.uuid4().hex
    received = []
    lc.Subscribe(channel, lambda data: received.append(data))
    deadline = time.monotonic() + 2
    while not received and time.monotonic() < deadline:
        lc.Publish(channel, b"dependency-check")
        lc.HandleSubscriptions(timeout_millis=100)
    if not received:
        raise RuntimeError("LCM multicast loopback failed; check the container multicast route")
    return "publish/subscribe round trip passed"
