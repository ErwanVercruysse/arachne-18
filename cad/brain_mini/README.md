# ARACHNE-18 mini brain board (v1.0)

Small 4-layer servo/IMU board for the hexapod that works **with the existing Raspberry Pi 4**: no CM5, no PI LINK.
The Pi talks to the ESP32-S3 over **one USB-C (board) <-> USB-A (Pi) cable** and gets its 5 V from the board.
It is the big board (`cad/brain_pcb`) without the CM5 part: same buck, 12 V entry, servo buses and IMU circuits.

| Block | Part | What it does |
|---|---|---|
| Real-time | **ESP32-S3-WROOM-1-N16R8** | drives both servo buses, reads the IMU, native USB to the Pi (`/dev/ttyACM0`) |
| IMU | **MPU-6050** (I2C on the ESP32, INT on IO6) | near the middle of the board |
| GPIO | 2x13 header | 19 free GPIOs + I2C + 3V3/5V/GND |
| Servo buses | 2x (74AHCT1G126 + 74LVC1G34) | Feetech STS half-duplex, 5 V levels, 1 Mbaud |
| Legs | 6x Molex 5268-03A (2.50 mm) | bus A = L1 L2 L3 (right), bus B = L4 L5 L6 (left) |
| Power | 2 big solder pads, P-MOSFET reverse protection, TVS, 2x 330 uF | 12 V straight to the servos |
| 5 V | TI **TPS54560B** buck, 5 V 5 A | Pi 4 (via the **5V PI** pads) + ESP32 + bus drivers |

Board: **92 x 60 mm**, 1.6 mm, 4 layers (F signals / In1 GND / In2 12 V / B signals + 5 V band to the Pi pads),
R6 corners, 4x M3 (3.2 mm) at (±41.5, ±25.5). Mounts on acrylic plate **v4**
(`cad/electronics_plate/make_electronics_plate_v4.py`): M3 x 12 -> PCB -> 8 mm spacer -> plate -> nut.

## Connections

* **USB-C "USB > PI 4"**: USB-C to USB-A cable to any Pi 4 USB port. Flash the ESP32 from the Pi
  (`arduino-cli` / `esptool` over `/dev/ttyACM0`) or from the Mac with the same cable. The Pi's 5 V on this cable
  only feeds the ESP32 side (through SS14 D7), so the ESP32 can be flashed without the 12 V battery.
* **5V PI pads** (top edge, next to the buck): 5 V / 3 A out for the Pi 4. Solder an 18-20 AWG pair to the Pi's
  GPIO pin 2 or 4 (+5 V) and pin 6 (GND), or to a cut USB-C lead plugged into the Pi power socket.
* **12V IN pads** (bottom-left): battery, 14-16 AWG; zip-tie slots on plate v4 as strain relief.
* **Servo (each leg):** `1 = GND, 2 = +12 V, 3 = DATA` (silkscreen G V S). Check against your cable first!

## ESP32-S3 pins

| GPIO | function | GPIO | function |
|---|---|---|---|
| 17 / 18 / 16 | bus A TX / RX / DE (UART1, DE = RTS) | 4 / 5 / 6 | IMU SDA / SCL / INT |
| 10 / 11 / 9 | bus B TX / RX / DE (UART2) | 1 | 12 V sense (100k/15k: V = adc x 7.67) |
| 19 / 20 | USB D- / D+ (to the Pi) | 2 | blue LED |
| 0 / EN | BOOT / RESET buttons | | |

**GPIO header J14 (2x13, 2.54 mm, middle of the board, every pin labelled on the silkscreen):** all 19 free
GPIOs + I2C + power. Odd pins left, even right:

| pin | | pin | |
|---|---|---|---|
| 1 | 3V3 | 2 | 5V |
| 3 | GND | 4 | GND |
| 5 | SDA (IO4, IMU bus) | 6 | SCL (IO5) |
| 7 | IO7 | 8 | IO15 |
| 9 | IO8 | 10 | IO3 * |
| 11 | IO46 * | 12 | IO12 |
| 13 | IO13 | 14 | IO14 |
| 15 | IO21 | 16 | IO47 |
| 17 | IO48 | 18 | IO45 * |
| 19 | IO38 | 20 | IO39 |
| 21 | IO40 | 22 | IO41 |
| 23 | IO42 | 24 | TX43 (UART0) |
| 25 | RX44 (UART0) | 26 | GND |

\* strapping pins: don't pull them up/down at boot. IO35-37 are used by the octal PSRAM (N16R8).

## Files

| File | |
|---|---|
| `arachne_mini.kicad_pro / .kicad_sch / .kicad_pcb` | open in KiCad 10 |
| `fab/` | gerbers zip, BOM + CPL with LCSC numbers for JLCPCB, schematic PDF, renders |
| `tools/design.py` | **single source of truth**: every part, net and position |
| `tools/build_all.sh` | regenerates everything (libraries, schematic, board, Freerouting + A* finisher, checks, fab) |

## Order (JLCPCB)

4 layers, 1.6 mm, upload `fab/arachne_mini_gerbers_JLCPCB.zip`, then BOM + CPL for assembly (LCSC numbers filled in).
The ESP32 antenna overhangs the top edge by ~6 mm on purpose.

## Before the first power-up

1. 12 V on the pads without Pi or servos: green LED, measure 5.0 V on the 5V PI pads and 3.3 V.
2. Connect the Pi (5V PI pads + USB cable), flash the ESP32.
3. Servos one leg at a time (torque on one servo at a time, 30 %).
