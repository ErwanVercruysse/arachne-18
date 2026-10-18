# ARACHNE-18 mini brain board (v1.0)

Small 4-layer servo/IMU board for the hexapod that works **with the existing Raspberry Pi 4**: no CM5, no PI LINK.
The Pi talks to the ESP32-S3 over **one USB-C (board) <-> USB-A (Pi) cable** and gets its 5 V from the board.
It is the big board (`cad/brain_pcb`) without the CM5 part: same buck, 12 V entry, servo buses and IMU circuits.

| Block | Part | What it does |
|---|---|---|
| Real-time | **ESP32-S3-WROOM-1-N16R8** | drives both servo buses, reads the IMU, native USB to the Pi (`/dev/ttyACM0`) |
| IMU | **MPU-6050** (I2C on the ESP32, INT on IO6) | near the middle of the board |
| Sensors | G-V-S strip + 2x Qwiic | 19 free GPIOs, each with GND + 3.3 V, plus I2C |
| Servo buses | 2x (74AHCT1G126 + 74LVC1G34) | Feetech STS half-duplex, 5 V levels, 1 Mbaud |
| Legs | 6x Molex 5268-03A (2.50 mm) | bus A = L1 L2 L3 (right), bus B = L4 L5 L6 (left) |
| Power | 2 big solder pads, P-MOSFET reverse protection, TVS, 2x 330 uF | 12 V straight to the servos |
| 5 V | TI **TPS54560B** buck, 5 V 5 A | Pi 4 (via the **5V PI** pads) + ESP32 + bus drivers |

Board: **92 x 70 mm**, 1.6 mm, 4 layers (F signals / In1 GND / In2 12 V / B signals + 5 V band to the Pi pads),
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

**Sensor strip (bottom edge):** every free GPIO has its own **G - V - S** column (2.54 mm), like an Arduino sensor
shield, so a standard 3-wire sensor cable (GND, VCC, signal) plugs straight on.
* **V = 3.3 V.** The ESP32 pins are NOT 5 V tolerant: use 3.3 V sensors, or a level shifter for 5 V ones.
* Columns from left to right: IO7, IO15, IO8, IO3\*, IO46\*, IO12, IO13, IO14, IO21, IO47, IO48, IO45\*, IO38, IO39,
  IO40, IO41, IO42, TX43, RX44. The name is printed above every column.
* Next to the strip: **2x Qwiic / STEMMA QT** (JST-SH 4-pin, 3.3 V I2C, the same bus as the MPU-6050 on IO4/IO5).

\* strapping pins: don't pull them up/down at boot. IO35-37 are used by the octal PSRAM (N16R8).

**Servo connectors:** Molex 5268-type headers for the **5264** plugs (2.5 mm) on the Feetech STS3215 cables.

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
