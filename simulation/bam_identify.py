"""Measure a real STS3215 on a pendulum rig and fit a BAM M6 model (macOS/Linux).

Rig: servo clamped with a horizontal shaft, a stiff arm on the horn, a known
mass at the tip. The arm hangs straight down at angle 0. Servo ID 1 by default.

    python simulation/bam_identify.py                  # full run: record, process, fit, plot
    python simulation/bam_identify.py --dry-run        # test the pipeline without hardware
    python simulation/bam_identify.py --fit-only       # re-fit existing recordings

Through the ESP32-C3 bus firmware (firmware/sts3215_bus, "bridge" mode):
    python simulation/bam_identify.py --esp --id 2 --mass-g 38 --length 0.22 --arm-mass-g 0 --record-only
Record every weight like that, then fit all of them together:
    python simulation/bam_identify.py --fit-only

Talks the Feetech STS protocol directly over pyserial; no pypot fork needed.
EEPROM is never unlocked, so the P gain changes are not written to flash.
"""
from pathlib import Path
import argparse, datetime, glob, json, math, os, subprocess, sys, time
import numpy as np
from bam.trajectory import trajectories

HERE = Path(__file__).resolve().parent
STEPS = 4096/(2*math.pi)  # encoder steps per radian
KPS = [4, 8, 16, 32]
TRAJECTORIES = ['sin_time_square', 'sin_sin', 'up_and_down', 'lift_and_drop', 'nothing']
# STS3215 registers
P_GAIN, D_GAIN, I_GAIN, OFFSET, MODE, TORQUE, GOAL, TORQUE_LIMIT, POSITION = 21, 22, 23, 31, 33, 40, 42, 48, 56
STALL_LOAD = 950  # 0.1 % units: 95 % load for 0.3 s aborts the recording


class STS:
    def __init__(self, port, servo_id, baud=1_000_000, esp=False):
        import serial
        self.id = servo_id
        self.port = serial.Serial(port, 115200 if esp else baud, timeout=.02)
        if esp:
            # ESP32-C3 firmware: switch it to transparent bridge mode.
            self.port.write(b'~~~\nstop\n'); time.sleep(.3)
            self.port.reset_input_buffer(); self.port.write(b'bridge\n'); time.sleep(.3)
            reply = self.port.read(4096)
            if b'bridge on' not in reply: raise SystemExit('ESP32 firmware antwoordt niet met "bridge on"')
            self.port.reset_input_buffer()

    def close(self):
        try: self.torque(False)
        except IOError as e: print('  torque uit mislukt:', e)
        finally: self.port.write(b'~~~'); self.port.close()

    def _packet(self, instruction, params):
        body = [self.id, len(params)+2, instruction, *params]
        return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])

    def _reply(self):
        head = self.port.read(5)
        if len(head) < 5 or head[:2] != b'\xff\xff': raise IOError('No reply from servo')
        rest = self.port.read(head[3]-1)
        if len(rest) < head[3]-1: raise IOError('Short reply from servo')
        # Status bits: 0x01 voltage, 0x04 temperature, 0x20 overload. A short supply
        # dip (0x01) is logged, not fatal; anything else stops the run.
        self.flags = getattr(self, 'flags', 0) | head[4]
        if head[4] & ~0x21: raise IOError(f'Servo error flags 0x{head[4]:02x}')
        return rest[:-1]

    def write(self, address, values):
        self.port.reset_input_buffer()
        self.port.write(self._packet(3, [address, *values])); self._reply()

    def read(self, address, count):
        self.port.reset_input_buffer()
        self.port.write(self._packet(2, [address, count])); return self._reply()

    def ping(self):
        try: self.read(3, 2); return True
        except IOError: return False

    def torque(self, on): self.write(TORQUE, [1 if on else 0])

    def goal(self, steps):
        steps = int(np.clip(round(steps), 0, 4095)); self.write(GOAL, [steps & 0xFF, steps >> 8])

    def state(self):
        """Raw position steps, speed steps/s (signed), load, volts, temperature."""
        r = self.read(POSITION, 8)
        def signed(lo, hi, bit=15):
            v = lo | hi << 8
            return -(v & ~(1 << bit)) if v & (1 << bit) else v
        return signed(r[0], r[1]), signed(r[2], r[3]), signed(r[4], r[5], 10), r[6]/10, r[7]

    def gains(self, kp):
        self.write(MODE, [0]); self.write(P_GAIN, [kp]); self.write(D_GAIN, [0]); self.write(I_GAIN, [0])

    def torque_limit(self, permille):
        self.write(TORQUE_LIMIT, [permille & 0xFF, permille >> 8])

    def center_here(self):
        """Shift the encoder zero in RAM (EEPROM stays locked) so the current
        position reads 2048; +-170 deg around it then never crosses 4095 -> 0."""
        pos = self.state()[0]
        r = self.read(OFFSET, 2); raw = r[0] | r[1] << 8
        offset = (-(raw & 0x7FF) if raw & 0x800 else raw & 0x7FF) + pos-2048
        offset = (offset+2047) % 4096-2047
        v = abs(offset) | (0x800 if offset < 0 else 0)
        self.write(OFFSET, [v & 0xFF, v >> 8]); time.sleep(.05)
        return self.state()[0]


class FakeSTS:
    """Pendulum stand-in for --dry-run; a crude P controller, not a real servo."""
    def __init__(self, *_):
        self.q, self.dq, self.target, self.on, self.kp, self.t = 2048., 0., 2048., False, 32, time.time()
    def ping(self): return True
    def torque(self, on): self.on = on
    def goal(self, steps): self.target = steps
    def gains(self, kp): self.kp = kp
    def torque_limit(self, permille):
        self.write(TORQUE_LIMIT, [permille & 0xFF, permille >> 8])

    def center_here(self): return int(self.q)
    def torque_limit(self, permille): pass
    def close(self): pass
    def state(self):
        now = time.time(); dt = min(now-self.t, .01); self.t = now
        q = (self.q-2048)/STEPS
        tau = -.5*9.81*.1*math.sin(q)-.05*self.dq
        if self.on: tau += np.clip(self.kp*.08*(self.target-self.q)/STEPS, -2.9, 2.9)
        self.dq += tau/(.5*.01)*dt; self.q += self.dq*STEPS*dt
        return int(self.q), int(self.dq*STEPS), 0, 12.1, 30


def find_port():
    ports = sorted(glob.glob('/dev/cu.usbmodem*')+glob.glob('/dev/cu.usbserial*')+glob.glob('/dev/cu.wchusbserial*')
                   + glob.glob('/dev/ttyACM*')+glob.glob('/dev/ttyUSB*'))
    if not ports: raise SystemExit('Geen USB-adapter gevonden. Sluit de servo-adapter aan of geef --port op.')
    if len(ports) > 1: print('Meerdere poorten gevonden, ik gebruik', ports[0], '(kies een andere met --port)')
    return ports[0]


def ask(prompt, value):
    if value is not None: return value
    while True:
        try: return float(input(prompt).replace(',', '.'))
        except ValueError: print('Geef een getal, bv. 0.5')


def move_to(servo, zero, angle, seconds=1.):
    q = (servo.state()[0]-zero)/STEPS
    for s in np.linspace(0, 1, max(2, int(seconds/.02))):
        servo.goal(zero+STEPS*(q+(angle-q)*s)); time.sleep(.02)


def probe(servo, zero, margin_deg=10., reach_deg=125.):
    """Find the free range on both sides with gentle, torque-limited moves.
    Returns (lo, hi) in rad relative to hanging, already minus the margin."""
    servo.gains(32); servo.torque_limit(300); servo.torque(True)
    limits = []
    try:
        for side in (+1, -1):
            reached = 0.
            for deg in np.arange(1, reach_deg+1, 1.):
                servo.goal(zero+STEPS*side*math.radians(deg)); time.sleep(.02)
                q = (servo.state()[0]-zero)/STEPS
                reached = max(reached, side*q)
                if deg-math.degrees(side*q) > 10:  # the arm stopped following: something is in the way
                    print(f'  kant {side:+d}: geblokkeerd rond {math.degrees(reached):.0f} graden')
                    break
            else:
                print(f'  kant {side:+d}: vrij tot {reach_deg:.0f} graden')
            limits.append(side*max(0., reached-math.radians(margin_deg)))
            move_to(servo, zero, 0., 2.)
    finally:
        servo.torque_limit(1000)
    return limits[1], limits[0]


def limited(trajectory, lo, hi):
    """Scale a BAM trajectory so every goal stays inside [lo, hi] (rad)."""
    ts = np.arange(0, trajectory.duration, .001)
    goals = np.array([trajectory(t)[0] for t in ts])
    free = min(hi, -lo)
    if not all(trajectory(t)[1] for t in ts):
        # Torque is released: the arm swings freely to the other side too.
        scale = free/max(np.abs(goals).max(), 1e-9)
    else:
        scale = min(hi/goals.max() if goals.max() > 0 else np.inf, lo/goals.min() if goals.min() < 0 else np.inf)
    scale = float(min(1., scale))
    class Limited:
        duration = trajectory.duration
        def __call__(self, t):
            g, enable = trajectory(t); return g*scale, enable
    return Limited(), scale


def record(servo, zero, name, kp, rig, logdir, tag='', limits=None):
    trajectory, scale = limited(trajectories[name], *limits) if limits else (trajectories[name], 1.)
    goal0, enable = trajectory(0)
    servo.gains(kp)
    if enable: servo.torque(True); move_to(servo, zero, goal0)
    else: servo.torque(False)
    time.sleep(.5)
    data = dict(**rig, kp=kp, motor='sts3215', trajectory=name, trajectory_scale=scale,
                range_rad=list(limits) if limits else None, entries=[])
    start = time.time(); torque_on = enable; volts = []; stall = None
    while (t := time.time()-start) < trajectory.duration:
        goal, enable = trajectory(t)
        if enable != torque_on: servo.torque(enable); torque_on = enable
        if enable: servo.goal(zero+STEPS*goal)
        t0 = time.time()-start
        pos, speed, load, v, temp = servo.state()
        t1 = time.time()-start
        volts.append(v)
        # Stall guard: a blocked arm must never be pushed at full torque.
        stall = None if abs(load) < STALL_LOAD else (stall or time.time())
        if stall and time.time()-stall > .3:
            servo.torque(False)
            raise SystemExit(f'STALL: {load/10:.0f} % last tijdens {name}, koppel uit. Zit de arm ergens tegen?')
        data['entries'].append(dict(position=(pos-zero)/STEPS, speed=speed/STEPS, load=float(load), input_volts=v,
                                    temp=float(temp), timestamp=(t0+t1)/2, goal_position=float(goal), torque_enable=enable))
    servo.torque(True); move_to(servo, zero, 0., 1.5); servo.torque(False)
    data['vin'] = float(np.mean(volts))
    data['servo_flags'] = getattr(servo, 'flags', 0); servo.flags = 0
    if data['servo_flags'] & 1: print('  let op: spanningsdip gemeld door de servo tijdens deze opname')
    rate = len(data['entries'])/trajectory.duration
    stamp = datetime.datetime.now().strftime('%Y-%m-%d_%Hh%Mm%S')
    (logdir/f'{stamp}_{tag}_kp{kp}_{name}.json').write_text(json.dumps(data))
    print(f'  opgeslagen: {rate:.0f} metingen/s, {data["vin"]:.1f} V')
    if rate < 150: print('  LET OP: minder dan 150 metingen/s; gebruik een snellere USB-adapter of kortere kabel.')


def bam(*args):
    print('$ python -m bam.'+' '.join(args), flush=True)
    subprocess.run([sys.executable, '-m', 'bam.'+args[0], *args[1:]], check=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port'); ap.add_argument('--id', type=int, default=1)
    ap.add_argument('--mass', type=float, help='gewicht aan het uiteinde, kg')
    ap.add_argument('--mass-g', type=float, help='gewicht aan het uiteinde, gram')
    ap.add_argument('--arm-mass', type=float, help='massa van de arm zelf, kg')
    ap.add_argument('--arm-mass-g', type=float, help='massa van de arm zelf, gram')
    ap.add_argument('--esp', action='store_true', help='via de ESP32-C3 firmware (bridge mode)')
    ap.add_argument('--record-only', action='store_true', help='enkel opnemen, niet fitten')
    ap.add_argument('--kps', type=int, nargs='+', default=KPS, help='welke Kp-waarden (standaard 4 8 16 32)')
    ap.add_argument('--only', nargs='+', choices=TRAJECTORIES, help='enkel deze bewegingen opnemen')
    ap.add_argument('--probe', action='store_true', help='eerst de vrije ruimte aftasten en de bewegingen daarbinnen schalen')
    ap.add_argument('--length', type=float, help='as tot gewicht, m')
    ap.add_argument('--data', type=Path, default=HERE.parent/'bam_data'/'identify')
    ap.add_argument('--output', type=Path, default=HERE/'bam_sts3215_12v_measured_m6.json')
    ap.add_argument('--trials', type=int, default=3000)
    ap.add_argument('--dry-run', action='store_true', help='geen hardware; nep-servo om de pijplijn te testen')
    ap.add_argument('--fit-only', action='store_true', help='niet opnemen, enkel bestaande data fitten')
    args = ap.parse_args()
    if args.mass_g is not None: args.mass = args.mass_g/1000
    if args.arm_mass_g is not None: args.arm_mass = args.arm_mass_g/1000
    raw, processed = args.data/'raw', args.data/'processed'
    if args.dry_run: raw, processed = args.data/'dry_raw', args.data/'dry_processed'
    raw.mkdir(parents=True, exist_ok=True); processed.mkdir(parents=True, exist_ok=True)

    if not args.fit_only:
        if args.dry_run: servo = FakeSTS(); rig = dict(mass=.5, arm_mass=.01, length=.1)
        else:
            port = args.port or find_port()
            servo = STS(port, args.id, esp=args.esp)
            if not servo.ping(): raise SystemExit(f'Servo ID {args.id} antwoordt niet op {port}. Voeding aan? Juiste ID?')
            print(f'Servo ID {args.id} gevonden op {port}.')
            rig = dict(mass=ask('Gewicht aan het uiteinde (kg): ', args.mass),
                       arm_mass=ask('Massa van de arm zelf (kg): ', args.arm_mass),
                       length=ask('Afstand as -> gewicht (m): ', args.length))
            servo.torque(False)
            print('Koppel uit: de arm zakt vanzelf naar beneden. Wachten tot hij stil hangt...')
            last, still, t0 = None, time.time(), time.time()
            while time.time()-still < 2 and time.time()-t0 < 20:
                pos = servo.state()[0]
                if last is not None and abs(pos-last) > 3: still = time.time()
                last = pos; time.sleep(.05)
        zero = servo.center_here()
        print(f'Nulpunt: {zero} steps. Start {len(KPS)*len(TRAJECTORIES)} opnames van 6 s. Blijf uit de buurt van de arm!')
        tag = f'{rig["mass"]*1000:g}g'
        limits = None
        if args.probe:
            print('Vrije ruimte aftasten (30 % koppel, traag)...', flush=True)
            limits = probe(servo, zero)
            print(f'Bereik: {math.degrees(limits[0]):.0f} tot {math.degrees(limits[1]):+.0f} graden (10 graden marge)', flush=True)
            if min(-limits[0], limits[1]) < math.radians(15) and max(-limits[0], limits[1]) < math.radians(30):
                servo.close(); raise SystemExit('Te weinig vrije ruimte voor een zinvolle meting.')
        try:
            for kp in args.kps:
                for name in (args.only or TRAJECTORIES):
                    print(f'Kp {kp}, {name.replace("_", " ")}', flush=True)
                    record(servo, zero, name, kp, rig, raw, tag, limits)
                    time.sleep(1.)
                if not args.dry_run and kp != args.kps[-1]:
                    _, _, _, v, temp = servo.state(); print(f'  servo {temp} C, {v:.1f} V')
                    if temp > 50: print('  Servo is warm; even 60 s afkoelen...'); time.sleep(60)
            servo.gains(32)
        finally:
            servo.close()
        if args.record_only:
            print(f'Opnames voor {tag} klaar in {raw}'); return

    for f in processed.glob('*.json'): f.unlink()
    bam('process', '--raw', str(raw), '--logdir', str(processed), '--dt', '0.005')
    output = args.output.with_name('dry_run_m6.json') if args.dry_run else args.output
    bam('fit', '--actuator', 'sts3215', '--model', 'm6', '--logdir', str(processed), '--method', 'cmaes',
        '--trials', str(100 if args.dry_run else args.trials), '--workers', str(os.cpu_count() or 1), '--output', str(output))
    params = json.loads(output.read_text()); params.setdefault('model', 'm6'); params.setdefault('actuator', 'sts3215')
    output.write_text(json.dumps(params, indent=2)+'\n')
    print(f'\nKlaar! Model: {output}\nBekijk de fit:  python -m bam.plot --actuator sts3215 --sim --logdir {processed} --params {output}')
    print(f'Gebruik het:   mjpython simulation/stunts.py --parameters {output}')


if __name__ == '__main__': main()
