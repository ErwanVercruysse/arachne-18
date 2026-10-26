"""Measure how the MPU6050 sits in the robot by tilting it by hand -> calibration/imu_map.json.

Needs live_twin.py running on the Pi (it reads the IMU). Three poses, each held still for 2 s:
  1. flat on a LEVEL table (standing or lying, doesn't matter): this pose also takes out the accelerometer's
     own offset (up to ~3 deg on an MPU6050), so whatever the robot sees here counts as level from now on
  2. the FRONT lifted ~20-30 degrees (front = legs L1 and L6), the back stays on the table
  3. the LEFT side lifted ~20-30 degrees (left = legs L1, L2, L3), the right side stays on the table
Pose 2 gives the forward axis, pose 3 checks it (a mirrored or swapped result is refused).
Restart the twin afterwards (sudo systemctl restart arachne-twin) so it loads the map.

    .venv/bin/python tools/imu_align.py
"""
import json, math, os, time, urllib.request
from pathlib import Path

URL = os.environ.get('ARACHNE_URL', 'http://localhost:5870')
OUT = Path(__file__).resolve().parents[1]/'calibration/imu_map.json'


def mean_acc(secs=2.):
    s, n, end = [0., 0., 0.], 0, time.time() + secs
    while time.time() < end:
        imu = json.loads(urllib.request.urlopen(URL + '/sense', timeout=2).read()).get('imu')
        if not imu or not imu.get('ok'): raise SystemExit('geen IMU in /sense: draait live_twin.py op de Pi en zit de MPU6050 erop?')
        s = [a + b for a, b in zip(s, imu['acc'])]; n += 1; time.sleep(.05)
    return [a/n for a in s]


def unit(v): n = math.sqrt(sum(a*a for a in v)); return [a/n for a in v]
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def cross(a, b): return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]


def main():
    input('1/3  Zet de robot plat op een waterpasse tafel en laat hem los. Enter… ')
    up = unit(mean_acc())                                  # at rest the accelerometer reads +1 g along body +z
    input('2/3  Til de VOORKANT (poten L1 en L6) 20-30° op, achterkant blijft liggen. Hou stil. Enter… ')
    a = unit(mean_acc())
    # front up: the measured 'up' leans towards body +x
    fwd = [x - dot(a, up)*u for x, u in zip(a, up)]
    tilt = math.degrees(math.asin(min(1., math.sqrt(dot(fwd, fwd)))))
    if tilt < 10: raise SystemExit(f'maar {tilt:.0f}° gekanteld, til hem verder op (20-30°) en begin opnieuw')
    x = unit(fwd); z = up; y = cross(z, x)
    input('3/3  Leg hem plat en til nu de LINKERKANT (poten L1, L2, L3) 20-30° op. Hou stil. Enter… ')
    b = unit(mean_acc())
    side = math.degrees(math.asin(max(-1., min(1., dot(b, y))))); along = math.degrees(math.asin(max(-1., min(1., dot(b, x)))))
    print(f'controle: {side:+.0f}° naar links, {along:+.0f}° naar voren (verwacht: ~+20..30 en ~0)')
    if side < 10 or abs(along) > .5*side:
        raise SystemExit('klopt niet: was het echt de linkerkant (L1 L2 L3)? Niets opgeslagen, begin opnieuw.')
    R = [x, y, z]                                          # rows: body axes in sensor coordinates
    OUT.write_text(json.dumps(dict(sensor_to_body=[[round(v, 5) for v in r] for r in R], created=time.strftime('%Y-%m-%dT%H:%M:%S'),
                                   note='body = R @ sensor. +x front (L1/L6), +y left (L1-L3), +z up. Measured with tools/imu_align.py.'),
                              indent=2) + '\n')
    print(f'opgeslagen in {OUT}. Herstart de twin: sudo systemctl restart arachne-twin')


if __name__ == '__main__': main()
