"""Minimal Feetech STS3215 bus driver over a USB adapter (Waveshare Bus Servo Adapter, jumper on B)."""
import glob, time, serial

ADDR_ID, ADDR_TORQUE, ADDR_ACC, ADDR_GOAL, ADDR_SPEED = 5, 40, 41, 42, 46
ADDR_LOCK, ADDR_POS, ADDR_LOAD, ADDR_VOLT, ADDR_TEMP, ADDR_MOVING = 55, 56, 60, 62, 63, 66


def default_port():
    ports = glob.glob('/dev/cu.usbmodem5A*') + glob.glob('/dev/cu.usbserial*') + glob.glob('/dev/cu.wchusbserial*')
    if not ports: raise SystemExit('Geen servo-adapter gevonden (USB)')
    return ports[0]


class Bus:
    def __init__(self, port=None, baud=1_000_000):
        self.s = serial.Serial(port or default_port(), baud, timeout=.02)

    def _tx(self, sid, instr, params=b''):
        body = bytes([sid, len(params)+2, instr])+bytes(params)
        pkt = b'\xff\xff'+body+bytes([~sum(body) & 0xff])
        self.s.reset_input_buffer(); self.s.write(pkt); self.s.flush()

    def _rx(self, n_params):
        need = 6+n_params; data = b''; t = time.time()
        while len(data) < need and time.time()-t < .05:
            data += self.s.read(need-len(data))
        i = data.find(b'\xff\xff')
        if i < 0 or len(data) < i+need: return None
        pkt = data[i:i+need]
        if (~sum(pkt[2:-1]) & 0xff) != pkt[-1]: return None
        return pkt[5:5+n_params]

    def ping(self, sid):
        self._tx(sid, 1); return self._rx(0) is not None

    def read(self, sid, addr, n):
        self._tx(sid, 2, [addr, n]); return self._rx(n)

    def write(self, sid, addr, data):
        self._tx(sid, 3, [addr, *data]); self._rx(0)

    def sync_write(self, addr, per_id):
        """One SYNC WRITE packet (no replies): per_id = {sid: [bytes...]} all of equal length."""
        n = len(next(iter(per_id.values())))
        params = [addr, n] + [b for sid, d in per_id.items() for b in (sid, *d)]
        self._tx(0xFE, 0x83, params)

    def read16(self, sid, addr):
        r = self.read(sid, addr, 2)
        if r is None: return None
        v = r[0] | r[1] << 8
        return -(v & 0x7fff) if v & 0x8000 else v

    def position(self, sid): return self.read16(sid, ADDR_POS)
    def torque(self, sid, on): self.write(sid, ADDR_TORQUE, [1 if on else 0])

    def move(self, sid, pos, speed=1500, acc=50):
        pos = max(0, min(4095, int(pos)))
        self.write(sid, ADDR_ACC, [acc])
        self.write(sid, ADDR_GOAL, [pos & 0xff, pos >> 8, 0, 0, speed & 0xff, speed >> 8])

    def status(self, sid):
        v = self.read(sid, ADDR_VOLT, 2)
        return dict(pos=self.position(sid), volt=v[0]/10 if v else None, temp=v[1] if v else None)

    def scan(self, ids=range(0, 31)):
        return [i for i in ids if self.ping(i)]


if __name__ == '__main__':
    b = Bus(); ids = b.scan(); print('servo IDs:', ids)
    for i in ids: print(i, b.status(i))
