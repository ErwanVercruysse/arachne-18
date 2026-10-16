# ESP32-C3 servo bus firmware

Drives the STS3215 servos over a single wire from an ESP32-C3 Super Mini.

## Wiring

- GPIO21 → 560 Ω resistor → signal line of the servos (daisy-chained)
- GND shared between the ESP32, the servos and the 12 V supply
- Servos on 12 V; the ESP32 on USB

UART1 TX and RX both run on GPIO21 at 1 Mbaud. The firmware ignores its own
echo and only drives the line while sending.

## Sketches

| Folder | Purpose |
|---|---|
| `sts3215_bus/` | Main firmware: multiple servos, text commands over USB |
| `sts3215_sweep/` | First test: one servo (ID 1) spinning continuously |

## Build and upload

All code is in `bus.cpp`; the `.ino` is an empty stub.

```sh
cd firmware/sts3215_bus
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc -u -p /dev/cu.usbmodem1101 .
```

## Commands (serial monitor, 115200, one per line)

| Command | Action |
|---|---|
| `scan` | List servo IDs 1–30 and their positions; flags duplicate IDs |
| `id <old> <new>` | Change an ID permanently. **Connect only that servo.** |
| `demo <a> <b>` | Servo a clockwise, b anticlockwise; swap every 3 turns |
| `swing <id> [deg]` | Position mode, ±deg around the current position; streams measurements |
| `between <id> <a> <b>` | Swing between two raw positions (read with `scan` after moving the arm by hand) |
| `stop` | All servos: speed 0, torque off |
| `cw <+1\|-1>` | Set which speed sign is clockwise |

New servos all ship as ID 1. Set IDs one servo at a time. The robot uses IDs
1–18; see `simulation/joint_map.json`.

Current servos: ID 1 and ID 2 (set 2026-09-27).

## Swing test with logging

Hang the arm still, then from the repo root:

```sh
python firmware/swing_logger.py --mass-g 60 --seconds 30
```

Everything is saved in `bam_data/swing/<date>_<mass>g/`: `samples.csv` (~1 kHz),
`swings.csv`, `bam_log.json` (BAM format), `summary.json` and `plot.png`.
The firmware shifts the encoder zero in RAM so the start position reads 2048;
a swing never crosses the 4095 → 0 wrap. A power cycle resets it.

To get the direction right, turn the arm by hand (torque off) from start to end
and read both positions with `scan`. For servo ID 2 on the test rig: start 3957,
end 2879 (−95°):

```sh
python firmware/swing_logger.py --mass-g 38 --between 3957 2879 --seconds 30
```

The firmware stops with `STALL` and torque off if the load stays above 95 % for
0.3 s. If the servo drops off the bus while current is low, check the 12 V
connectors first: loose connectors caused repeated brown-outs on 2026-09-27.
