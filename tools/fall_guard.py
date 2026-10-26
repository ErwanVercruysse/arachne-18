"""Notice a fall and land in the least destructive way. Runs inside live_twin.py (it owns the IMU and the bus).

Detection (100 Hz, the MPU6050 in live_twin):
  free fall  - the accelerometer reads less than FREE_G for FREE_T (off a table, dropped): needs no alignment;
  tipping    - more than TIP_DEG of tilt (needs calibration/imu_map.json).
Reaction (chosen with training/fall_test.py, physics + servo model of the browser simulation):
  free fall -> REACT['free'], tipping -> REACT['tip']; see REACTIONS for what each one does.
Afterwards every moving script is stopped and servo writes are refused for LOCK_S, so nothing fights the landing.
Stand up again with A / 'Opstaan' once it lies still.
"""
import math, time
from pathlib import Path

import numpy as np

FREE_G, FREE_T = .4, .04        # below 0.4 g for 40 ms = falling (a 75 cm table gives ~390 ms of free fall)
TIP_DEG = 35.
IMPACT_G = 2.0
LOCK_S = 3.
STOP_FILES = ('/tmp/arachne_pad_stop',)
HOLD = None                      # 'pose' value: keep the current goal

# name: (pose, torque limit register 48 in 0.1 %; 0 = torque off)
REACTIONS = {
    'stiff': (HOLD, 1000),       # what happened before: keep holding the pose at full torque
    'limp': (HOLD, 0),           # torque off (only the gear friction holds)
    'brace': ('brace', 350),     # feet down and wide, soft servos: the legs take the landing like springs
    'brace_firm': ('brace', 600),
    'brace_low': ('brace_low', 350),  # feet further down: more travel to absorb the landing
    'brace_low_firm': ('brace_low', 500),
    'tuck': ('tuck', 350),       # legs up and soft: land on the belly
}
# training/fall_test.py, 75 cm table, 12 random falls each (results/fall_table_75cm.json): holding stiff puts
# 3.2-4.6 Nm through the gears (above the 2.9 Nm stall); 'brace' keeps them at 1.7 Nm and has the lowest
# average chassis impact (nose first 443 N vs 657 N stiff, sideways 643 N vs 988 N). Tipping over on the floor
# is a short fall: 'limp' loads the gears least.
REACT = {'free': 'brace', 'tip': 'limp'}


def pose(name, joints):
    """Joint angles [18] (model rad, within the measured ranges) for a reaction pose."""
    from policy_walk import ik, gait                                  # noqa: E402 (keeps live_twin's import light)
    lo, hi = joints.real
    if name in ('brace', 'brace_low'):
        r, z = (.154, -.125) if name == 'brace' else (.13, -.155)       # from the coxa axis / below the hips
        feet = np.concatenate([gait.COXA + r*gait.RADIAL, np.full((6, 1), z)], 1)
        return ik(feet, lo, hi)
    if name == 'tuck':                                                  # femurs up, feet above the belly
        return np.clip(np.tile([0., math.radians(60), 0.], 6), lo, hi)
    raise ValueError(name)


class Detector:
    """Feed it the accelerometer magnitude (g) and the tilt (deg, or None) at a steady rate."""

    def __init__(self):
        self.t_free = 0.; self.state = 'ok'; self.since = 0.; self.still = 0.

    def update(self, acc_g, tilt_deg, dt):
        """Returns 'free' or 'tip' once when a fall starts, 'impact' once on landing, else None."""
        now = time.time()
        if self.state == 'ok':
            self.t_free = self.t_free + dt if acc_g < FREE_G else 0.
            if self.t_free >= FREE_T: self.state, self.since = 'falling', now; return 'free'
            if tilt_deg is not None and tilt_deg > TIP_DEG: self.state, self.since = 'tipped', now; return 'tip'
            return None
        if self.state == 'falling' and acc_g > IMPACT_G: self.state, self.since = 'landed', now; return 'impact'
        # back to watching once it has been still (about 1 g) for a second, after the lock
        self.still = self.still + dt if abs(acc_g - 1) < .1 else 0.
        if now - self.since > LOCK_S and self.still > 1.: self.state = 'ok'; self.t_free = 0.
        return None

    def locked(self): return self.state != 'ok' and time.time() - self.since < LOCK_S


def apply_sim(w, reaction):
    """The reaction on tools/sim_world.SimWorld (used by training/fall_test.py)."""
    what, limit = REACTIONS[reaction]
    if what is not HOLD:
        q = pose(what, _joints())
        w.goal_q = q.copy(); w.ticks_goal = w._ticks(q); w.speed_reg[:] = 0
    if limit: w.limit_reg[:] = limit; w.torque[:] = True
    else: w.torque[:] = False


def apply_bus(reaction, sync_write, goals):
    """The reaction on the real robot. sync_write(addr, {joint: [bytes]}) per register, goals({joint: ticks})."""
    what, limit = REACTIONS[reaction]
    j = _joints()
    if limit: sync_write(48, {n: [limit & 0xff, limit >> 8] for n in j.cal})
    else: sync_write(40, {n: [0] for n in j.cal})
    if what is not HOLD and limit: goals(j.ticks_of(pose(what, j)))
    for f in STOP_FILES: Path(f).touch()


_J = []
def _joints():
    if not _J:
        from policy_walk import Joints
        _J.append(Joints())
    return _J[0]
