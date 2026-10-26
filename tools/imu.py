"""MPU6050 (or MPU6500/9250 clone) on the Pi's I2C bus: body tilt for the twin and the cockpit.

Wiring on the Pi 4: VCC -> 3.3 V (pin 1), GND -> pin 6, SDA -> GPIO2 (pin 3), SCL -> GPIO3 (pin 5).
Enable I2C once: sudo raspi-config nonint do_i2c 0   (then /dev/i2c-1 exists).

No extra Python packages: plain /dev/i2c-1 with the I2C_SLAVE ioctl.
Angles are in the SENSOR frame. Which sensor axis is the robot's forward/left is measured once by tilting
the robot by hand (tools/imu_align.py -> calibration/imu_map.json, never guess directions). With that map the
IMU also gives gravity and the turn rate in the BODY frame (+x = front = L1/L6, +y = left = L1/L2/L3, +z = up),
which is what the walking policy (tools/policy_walk.py) uses.

    python3 tools/imu.py          # print roll/pitch at 10 Hz to check the wiring
"""
import fcntl, json, math, os, struct, threading, time
from pathlib import Path

MAP_FILE = Path(__file__).resolve().parents[1]/'calibration/imu_map.json'


def load_map():
    """3x3 rows: body axes in sensor coordinates (body = R @ sensor), or None when not measured yet."""
    try: return json.loads(MAP_FILE.read_text())['sensor_to_body']
    except (OSError, ValueError, KeyError): return None


def mat_vec(R, v): return [sum(R[i][k]*v[k] for k in range(3)) for i in range(3)]


class GravityFilter:
    """Complementary filter on accelerometer (g) + gyro (rad/s), both body frame -> unit gravity vector.
    The gyro turns the estimate (dg/dt = -w x g), the accelerometer pulls it back over ~0.5 s and is trusted
    less while it does not read ~1 g (steps, impacts, free fall). Also used by the simulator tests."""

    def __init__(self, k=.98):
        self.k = k; self.g = None

    def update(self, acc, gyro, dt):
        n = math.sqrt(sum(a*a for a in acc))
        g_acc = [-a/n for a in acc] if n > 1e-6 else [0., 0., -1.]   # at rest the sensor reads +1 g upwards
        if self.g is None: self.g = g_acc; return self.g
        g, w = self.g, gyro
        g = [g[0] - dt*(w[1]*g[2] - w[2]*g[1]), g[1] - dt*(w[2]*g[0] - w[0]*g[2]), g[2] - dt*(w[0]*g[1] - w[1]*g[0])]
        trust = (1 - self.k)*max(0., 1 - abs(n - 1.)/.5)
        g = [(1 - trust)*a + trust*b for a, b in zip(g, g_acc)]
        m = math.sqrt(sum(a*a for a in g)); self.g = [a/m for a in g]
        return self.g

I2C_SLAVE = 0x0703
WHO_AM_I = {0x68: 'MPU6050', 0x70: 'MPU6500', 0x71: 'MPU9250', 0x72: 'MPU6050 kloon', 0x73: 'MPU9255', 0x98: 'MPU6050 kloon'}


class IMU:
    def __init__(self, bus=1, addr=None):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        for a in ([addr] if addr else [0x68, 0x69]):
            try:
                fcntl.ioctl(self.fd, I2C_SLAVE, a); who = self._read(0x75, 1)[0]
                if who in WHO_AM_I: self.addr, self.model = a, WHO_AM_I[who]; break
            except OSError: pass
        else:
            os.close(self.fd); raise OSError('geen MPU6050 gevonden op 0x68/0x69')
        self._write(0x6B, 0x01)        # wake up, clock from gyro X
        self._write(0x1A, 0x03)        # digital low-pass ~44 Hz
        self._write(0x1B, 0x00)        # gyro +-250 deg/s
        self._write(0x1C, 0x00)        # accel +-2 g
        self.roll = self.pitch = 0.; self.rate = [0., 0., 0.]; self.acc = [0., 0., 1.]; self.temp = 0.
        self.bias = [0., 0., 0.]; self.hz = 0.; self.ok = False; self.lock = threading.Lock()
        self.R = load_map(); self.filt = GravityFilter(); self.gravity = None; self.gyro_body = None; self.acc_body = None; self.t = 0.

    def _write(self, reg, val): os.write(self.fd, bytes([reg, val]))
    def _read(self, reg, n): os.write(self.fd, bytes([reg])); return os.read(self.fd, n)

    def raw(self):
        ax, ay, az, t, gx, gy, gz = struct.unpack('>7h', self._read(0x3B, 14))
        return [ax/16384., ay/16384., az/16384.], [gx/131., gy/131., gz/131.], t/340. + 36.53

    def calibrate(self, secs=1.):
        """Gyro offset while the robot stands still."""
        s, n, end = [0., 0., 0.], 0, time.time() + secs
        while time.time() < end:
            _, g, _ = self.raw(); s = [a + b for a, b in zip(s, g)]; n += 1; time.sleep(.005)
        self.bias = [v/max(1, n) for v in s]

    def run(self, hz=100):
        """Complementary filter: gyro for fast motion, accelerometer to stay level over time."""
        self.calibrate(); t0 = time.time(); last = t0; count = 0
        a, _, _ = self.raw()
        self.roll, self.pitch = math.degrees(math.atan2(a[1], a[2])), math.degrees(math.atan2(-a[0], math.hypot(a[1], a[2])))
        while True:
            try:
                a, g, temp = self.raw()
            except OSError:
                self.ok = False; time.sleep(.5); continue
            now = time.time(); dt = now - last; last = now
            g = [x - b for x, b in zip(g, self.bias)]
            ar, ap = math.degrees(math.atan2(a[1], a[2])), math.degrees(math.atan2(-a[0], math.hypot(a[1], a[2])))
            k = .98
            body = None
            if self.R:
                wb = [math.radians(x) for x in mat_vec(self.R, g)]; ab = mat_vec(self.R, a)
                body = (self.filt.update(ab, wb, min(dt, .05)), wb, ab)
            with self.lock:
                self.roll = k*(self.roll + g[0]*dt) + (1 - k)*ar
                self.pitch = k*(self.pitch + g[1]*dt) + (1 - k)*ap
                self.rate, self.acc, self.temp, self.ok, self.t = g, a, temp, True, now
                if body: self.gravity, self.gyro_body, self.acc_body = body
            count += 1
            if now - t0 >= 1: self.hz = count/(now - t0); t0, count = now, 0
            time.sleep(max(0, 1/hz - (time.time() - now)))

    def state(self):
        with self.lock:
            s = dict(model=self.model, ok=self.ok, roll=round(self.roll, 2), pitch=round(self.pitch, 2),
                     rate=[round(v, 1) for v in self.rate], g=round(math.sqrt(sum(v*v for v in self.acc)), 3),
                     acc=[round(v, 4) for v in self.acc], temp=round(self.temp, 1), hz=round(self.hz), aligned=bool(self.R))
            if self.gravity:
                s.update(gravity=[round(v, 5) for v in self.gravity], gyro_body=[round(v, 4) for v in self.gyro_body],
                         acc_body=[round(v, 4) for v in self.acc_body],
                         age=round(time.time() - self.t, 3))
            return s


def start():
    """IMU running in a thread, or None when there is no I2C bus / no sensor (Mac, lvdell, Pi without IMU)."""
    if not os.path.exists('/dev/i2c-1'): return None
    try: imu = IMU()
    except OSError: return None
    threading.Thread(target=imu.run, daemon=True).start()
    return imu


if __name__ == '__main__':
    imu = start()
    if not imu: raise SystemExit('Geen IMU: staat I2C aan (raspi-config) en zit de MPU6050 op SDA=GPIO2 / SCL=GPIO3?')
    print(f'{imu.model} op 0x{imu.addr:02x}. Hou de robot 1 s stil voor de gyro-kalibratie…')
    time.sleep(1.5)
    while True:
        s = imu.state(); print(f"roll {s['roll']:7.2f}°  pitch {s['pitch']:7.2f}°  draai {s['rate'][2]:6.1f}°/s  {s['g']:.2f} g  {s['temp']} °C  {s['hz']} Hz", flush=True)
        time.sleep(.1)
