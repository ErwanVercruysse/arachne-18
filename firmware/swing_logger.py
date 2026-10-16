"""Swing one STS3215 +-45 deg via the ESP32-C3 bus firmware and save every sample.

    python firmware/swing_logger.py --mass-g 60              # 30 s test, servo ID 2
    python firmware/swing_logger.py --mass-g 120 --seconds 60
    python firmware/swing_logger.py --mass-g 60 --lift         # hang down, 0 -> 90 -> 0 deg

Start with the arm hanging still: that position becomes the centre (0 rad).
Output folder bam_data/swing/<date>_<mass>g/ holds:
  samples.csv   every sample: time, goal, position, speed, load, current, volts, temp
  swings.csv    one row per swing: time to reach, end error, peak load and current
  bam_log.json  the same samples in BAM log format (usable by bam.process / bam.fit)
  summary.json  test settings and averages
  plot.png      position/goal, load and current over time
"""
from pathlib import Path
import argparse, csv, datetime, glob, json, math, time
import numpy as np
import serial

ROOT = Path(__file__).resolve().parents[1]
STEP = 2*math.pi/4096  # rad per encoder step


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mass-g', type=float, required=True, help='gewicht aan de arm, gram')
    ap.add_argument('--id', type=int, default=2)
    ap.add_argument('--deg', type=int, help='default 45 (swing) or 90 (lift)')
    ap.add_argument('--lift', action='store_true', help='let the arm hang down, then lift 0 -> deg -> 0')
    ap.add_argument('--between', type=int, nargs=2, metavar=('FROM', 'TO'), help='raw encoder positions measured by hand')
    ap.add_argument('--here', action='store_true', help='with --lift: start from the current position, no hanging')
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--length', type=float, default=.22, help='as tot gewicht, m')
    ap.add_argument('--arm-mass-g', type=float, default=0., help='massa van de arm zelf, gram')
    ap.add_argument('--port')
    args = ap.parse_args()
    args.deg = args.deg or (90 if args.lift else 45)
    mode = 'between' if args.between else 'lift' if args.lift else 'swing'
    if args.between: args.deg = round((args.between[1]-args.between[0])*360/4096)
    port = args.port or sorted(glob.glob('/dev/cu.usbmodem*'))[0]
    stamp = datetime.datetime.now().strftime('%Y-%m-%d_%Hh%Mm%S')
    out = ROOT/'bam_data'/'swing'/f'{stamp}_{args.mass_g:g}g_{mode}{args.deg}'
    out.mkdir(parents=True, exist_ok=True)

    s = serial.Serial(port, 115200, timeout=.1)
    s.write(b'stop\n'); time.sleep(.5); s.reset_input_buffer()
    if args.between: s.write(f'between {args.id} {args.between[0]} {args.between[1]}\n'.encode())
    else: s.write(f'{mode}{"here" if args.lift and args.here else ""} {args.id} {args.deg}\n'.encode())
    samples, swings, notes = [], [], []
    start = time.time()+60; buf = b''  # clock starts once the servo reports it is moving
    print(f'Opnemen {args.seconds:g} s met {args.mass_g:g} g, servo {args.id}, {mode} {args.deg} graden ...')
    try:
        while time.time()-start < args.seconds:
            buf += s.read(4096)
            *lines, buf = buf.split(b'\n')
            for raw in lines:
                line = raw.decode(errors='replace').strip()
                f = line.split(',')
                if f[0] == 'D' and len(f) == 9:
                    samples.append([int(f[1]), int(f[2]), int(f[3]), int(f[4]), int(f[5]), int(f[6]), float(f[7]), int(f[8])])
                elif f[0] == 'S' and len(f) == 8:
                    swings.append(f[1:]); print(f'  zwaai {f[1]}: {f[2]} ms, piek last {f[4]} %, piek stroom {f[5]} mA, {f[7]} C')
                elif line:
                    notes.append(line); print(' ', line)
                    if 'around position' in line: start = time.time()
                    if 'does not answer' in line or 'STALL' in line: start = 0
    finally:
        s.write(b'stop\n'); time.sleep(.3); s.close()
    if not samples: raise SystemExit('Geen metingen ontvangen; draait de sts3215_bus firmware?')

    a = np.array(samples, dtype=float)
    a = a[a[:, 6] > 0]  # drop the rare corrupted read (0 V)
    t = (a[:, 0]-a[0, 0])/1000
    around = [n for n in notes if 'around position' in n]
    center = int(around[0].split()[-1]) if around else int(a[0, 2])
    with open(out/'samples.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['time_s', 'goal_deg', 'position_deg', 'speed_deg_s', 'load_pct', 'current_mA', 'volts', 'temp_C'])
        for i, r in enumerate(a):
            w.writerow([f'{t[i]:.4f}', f'{(r[1]-center)*360/4096:.2f}', f'{(r[2]-center)*360/4096:.2f}',
                        f'{r[3]*360/4096:.1f}', f'{r[4]/10:.1f}', f'{r[5]*6.5:.0f}', r[6], int(r[7])])
    with open(out/'swings.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['side', 'time_to_reach_ms', 'end_error_steps', 'peak_load_pct', 'peak_current_mA', 'volts', 'temp_C'])
        w.writerows(swings)
    bam = dict(mass=args.mass_g/1000, arm_mass=args.arm_mass_g/1000, length=args.length, kp=32, motor='sts3215',
               vin=float(a[:, 6].mean()), trajectory=f'{mode}_{args.deg}deg', entries=[
                   dict(timestamp=float(t[i]), position=float((r[2]-center)*STEP), speed=float(r[3]*STEP),
                        load=float(r[4]), input_volts=float(r[6]), temp=float(r[7]),
                        goal_position=float((r[1]-center)*STEP), torque_enable=True) for i, r in enumerate(a)])
    (out/'bam_log.json').write_text(json.dumps(bam))
    sw = np.array([[float(x) for x in r[1:5]] for r in swings]) if swings else np.zeros((0, 4))
    summary = dict(mass_g=args.mass_g, servo_id=args.id, mode=mode, amplitude_deg=args.deg, length_m=args.length,
                   arm_mass_g=args.arm_mass_g, duration_s=float(t[-1]), samples=len(a),
                   sample_rate_hz=round(len(a)/max(t[-1], 1e-9), 1), swings=len(swings),
                   mean_time_to_reach_ms=float(sw[:, 0].mean()) if len(sw) else None,
                   mean_peak_load_pct=float(sw[:, 2].mean()) if len(sw) else None,
                   max_peak_load_pct=float(sw[:, 2].max()) if len(sw) else None,
                   mean_peak_current_mA=float(sw[:, 3].mean()) if len(sw) else None,
                   max_current_mA=float(np.abs(a[:, 5]).max()*6.5), volts_min=float(a[:, 6].min()),
                   temp_start_C=int(a[0, 7]), temp_end_C=int(a[-1, 7]), centre_steps=center)
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        ax[0].plot(t, (a[:, 1]-center)*360/4096, label='doel'); ax[0].plot(t, (a[:, 2]-center)*360/4096, label='positie')
        ax[0].set_ylabel('graden'); ax[0].legend(loc='upper right')
        ax[1].plot(t, a[:, 4]/10, color='C2'); ax[1].set_ylabel('last %')
        ax[2].plot(t, a[:, 5]*6.5, color='C3'); ax[2].set_ylabel('stroom mA'); ax[2].set_xlabel('tijd s')
        fig.suptitle(f'STS3215 ID {args.id}, {args.mass_g:g} g, {mode} {args.deg} graden')
        fig.tight_layout(); fig.savefig(out/'plot.png', dpi=110)
    except ImportError:
        pass
    print(json.dumps(summary, indent=2))
    print('Opgeslagen in', out)


if __name__ == '__main__': main()
