# STS3215 12 V: measured BAM M6 model

**The simulator now uses `simulation/bam_sts3215_12v_measured_m6.json`**, fitted to 80 pendulum recordings of one of
the robot's own 12 V servos (`bam_data/identify/raw`, recorded 2026-09-27 through the ESP32-C3 bridge): a 52 g, 22 cm
arm with 38 / 173 / 263 / 383 g at the tip, position gains Kp 4 / 8 / 16 / 32, BAM's five test trajectories, about
1 kHz sampling at 11.8-11.9 V. Details and numbers: [`simulation/bam_12v_measured_report.json`](../simulation/bam_12v_measured_report.json).

| Mean position error when the model replays the recordings | Datasheet approximation | Measured |
|---|---:|---:|
| All 80 recordings | 10.7° | **6.5°** |
| Kp 16, held out of the fit | 11.6° | **6.8°** |
| Lift and drop | 17.8° | 8.3° |
| Fast sine sweeps (hardest) | 22.5° | 15.0° |
| Light 38 g load | 14.6° | 5.1° |

How it was made: `bam.fit` (M6, CMA-ES, 4 workers x 3000 trials) on the Kp 4, 8 and 32 recordings, with Kp 16 kept out
to check that it predicts runs it never saw. The recordings never reach the servo's speed limit, so `max_velocity`
was set back to the datasheet 4.72 rad/s; `command_delay` (5 ms) comes from the original BAM identification, which
BAM 1.0.2 does not fit; `q_offset` is the rig's zero and is 0 for the robot. Implied stall torque before friction is
2.9 Nm and no-load speed 4.8 rad/s, both in line with the datasheet.

A fit on all 80 recordings scored slightly better (6.3°) but was rejected: it implied a ~40 Nm stall torque, because
pendulum loads below ~1 Nm do not pin down the winding resistance. Torque near stall, heat, voltage sag and gear
backlash are still not identified; a heavier-load or stall test would complete the model.

Refit after new recordings: `python simulation/bam_identify.py --fit-only` (it now also removes the hand-written
note fields that some recordings carry and BAM cannot batch).

## Earlier: provisional datasheet approximation

`simulation/bam_sts3215_12v_approx_m6.json` remains available (`--parameters` / `actuator_params=`) for comparison.

The upstream BAM project provides an identified **STS3215 7.4 V** model. This project uses a separately labeled **12 V approximation**, authorized for this prototype. It is not an identification of the user's 12 V hardware.

## Provenance

The original M6 parameter file is preserved as `simulation/bam_sts3215_7p4v_m6_original.json`, from [Rhoban/BAM](https://github.com/Rhoban/bam), commit `e9a619d56da5236206f4de6ceec2c1ee1b497b5c`. Its SHA-256 is `74d5068bd4453046fd5e33a11bfcc69a7f121740409f6818706a789b000df5e9`. Runtime dependency: PyPI `better-actuator-models==1.0.2`.

The parameter source snapshot and PyPI release do not expose exactly the same runtime API. This project's runner explicitly initializes the Feetech firmware target-smoothing state on reset, because the installed release normally initializes it when loading a measurement log.

## What was fitted

The official Feetech ST-3215-C018 12 V datasheet specifies **30 kg·cm (2.941995 Nm) stall output torque** and **0.222 s/60° no-load speed** at 12 V. `fit_bam_12v.py` solves for effective `kt` and `R` so the M6 net output matches those points with inherited friction.

| Parameter | Approximation |
|---|---:|
| Supply voltage | 12 V; runtime can override it |
| Effective motor constant `kt` | 2.3572085648 |
| Effective resistance `R` | 6.2986515716 |
| Firmware speed limit | 4.7171060865 rad/s |
| Net stall output after modeled friction | 2.941995 Nm |
| PWM limit | 0.97, inherited |

The M6 friction parameters, apparent rotor inertia, proportional-control scale and approximately 4.98 ms command delay are inherited from the 7.4 V identification. The original test-rig angle offset is set to zero. The runner interpolates delayed target history at the 1 ms simulation timestep. Voltage sag is disabled.

Reproduce the fit:

```sh
python simulation/fit_bam_12v.py
python simulation/validate.py
```

The output files are `bam_sts3215_12v_approx_m6.json` and `bam_12v_fit_report.json`. This modifies the tracked parameter/report files; inspect the diff before committing.

## Interpretation and limits

`kt` and `R` are **effective model coefficients**, not measured electrical properties. The datasheet's terminal resistance and current figures do not produce a consistent ideal DC motor model when combined directly with every torque/speed figure; this fit therefore targets the two mechanical points explicitly.

Do not use the model's inferred electrical current for wire sizing, pack sizing or battery-life estimates. The datasheet separately specifies 2.7 A stall current per servo. Thermal behavior, voltage sag, gearbox backlash and overload shutdown are not modeled reliably. Stall output torque is not a continuous load rating.

The 12 V approximation needs measured data before it can support quantitative hardware predictions. A useful next step is an actual BAM identification on the 12 V servos with the intended supply, firmware settings and load. Use `--parameters` to load a replacement model.

Sources: [BAM actuator catalogue](https://bam.readthedocs.io/en/latest/usage/actuators.html), [BAM MuJoCo integration](https://bam.readthedocs.io/en/latest/usage/mujoco_cpu.html), [Feetech 12 V datasheet](https://cdn.robotshop.com/media/F/Fit/RB-Fit-155/pdf/feetech_12v_30kg_cm_magnetic_encoding_servo_sts321_specification_pdf.pdf).
