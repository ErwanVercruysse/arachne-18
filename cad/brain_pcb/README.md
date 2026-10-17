# ARACHNE-18 brain board (v1.0)

> **Build variant ordered first: Pi 4 + this board, no CM5.** The CM5 costs ~€150-160 in Oct 2026 and is out of
> stock everywhere, so the 20 CM5-only parts are marked DNP (`DNP` in `tools/design.py`; footprints stay, so a
> CM4/CM5 can be added later). The existing Raspberry Pi 4 connects with a straight 10-way ribbon on **PI LINK**
> (gets 5 V from the board, UART to the ESP32, GPIO4 = ESP reset) or with a USB cable to the ESP32 USB-C.
> BOM/CPL in `fab/` already include the LCSC numbers (stock checked 2026-10-17).


One 4-layer PCB that replaces the Raspberry Pi 4, both Waveshare bus-servo adapters and the LM2596:

| Block | Part | What it does |
|---|---|---|
| Computer | Raspberry Pi **Compute Module 5** (wireless, e.g. CM5104032) on 2x Hirose/Amphenol 100-pin | Linux, WiFi 5 + Bluetooth 5, ~2-3x a Pi 4 |
| Real-time | **ESP32-S3-WROOM-1-N16R8** | drives both servo buses, reads the IMU |
| IMU | **MPU-6050** (I2C on the ESP32, INT on IO6) | sits near the robot centre |
| Servo buses | 2x (74AHCT1G126 + 74LVC1G34) | Feetech STS half-duplex, 5 V levels, 1 Mbaud |
| Legs | 6x Molex 5268-03A (2.50 mm, mates with the 5264 plug) | bus A = L1 L2 L3, bus B = L4 L5 L6 |
| Power | 2 big solder pads, P-MOSFET reverse protection, TVS, 2x 330 uF | 12 V straight to the servos |
| 5 V | TI **TPS54560B** buck, 5 V 5 A (TI datasheet design, 400 kHz) | CM5 + ESP32 + fan |

Board: 140 x 70 mm, 1.6 mm, 4 layers (F signals / In1 GND plane / In2 12 V plane / B signals + 5 V bus).
The outline follows the acrylic plate, 1 mm inside it, with notches where the corner coxa brackets swing.
Four M3 holes sit on the chassis inserts (CAD 64 x 25, hand-measured 65.1 x 25.35: the 4.2 mm holes fit both).

**Checks (2026-10-17):** DRC 0 errors / 0 unconnected, ERC 0, schematic <-> PCB parity 0 issues.

## Files

| File | |
|---|---|
| `arachne_brain.kicad_pro / .kicad_sch / .kicad_pcb` | open in KiCad 10 |
| `arachne.kicad_sym`, `arachne.pretty/`, `3d/` | project-local libraries (self-contained) |
| `fab/` | gerbers zip, BOM + CPL for JLCPCB, schematic PDF, renders |
| `tools/design.py` | **single source of truth**: every part, net and position |
| `tools/build_all.sh` | regenerates libraries, schematic, board, routing (Freerouting + own A* finisher), checks, fab files (~10 min) |

## Connectors

**Servo (each leg):** `1 = GND, 2 = +12 V, 3 = DATA` (silkscreen G V S). Check this against your cable before the first plug-in!

**PI LINK (2x5, 2.54 mm)** = the first 10 pins of a Raspberry Pi header, so a straight 10-way ribbon fits a Pi 4
if you run the board *without* a CM5:

| pin | signal | pin | signal |
|---|---|---|---|
| 1 | - | 2 | +5 V |
| 3 | CM5 SDA (GPIO2) | 4 | +5 V |
| 5 | CM5 SCL (GPIO3) | 6 | GND |
| 7 | ESP32 EN (reset) | 8 | ESP32 RX (= Pi TXD) |
| 9 | GND | 10 | ESP32 TX (= Pi RXD) |

**FAN:** JST-SH 4-pin, same as the Pi 5 active cooler (5 V, PWM, GND, tacho).

**USB-C "CM5":** flash the eMMC (`rpiboot`): hold RPIBOOT, power on, plug USB into the Mac. 12 V must be connected.

**USB-C "ESP32":** when a cable is plugged in, the ESP32 USB goes to that connector (arduino-cli from the Mac).
Without cable the ESP32 USB is wired to the CM5 (USB3-0 port, USB2 pair) and shows up as `/dev/ttyACM*` on the Pi,
so the existing Python tools keep working with a USB-CDC bridge firmware.

## ESP32-S3 pins

| GPIO | function | GPIO | function |
|---|---|---|---|
| 17 / 18 / 16 | bus A TX / RX / DE (UART1, RS485 half-duplex, DE = RTS) | 4 / 5 / 6 | IMU SDA / SCL / INT |
| 10 / 11 / 9 | bus B TX / RX / DE (UART2) | 1 | 12 V sense (100k/15k: V = adc x 7.67) |
| 43 / 44 | UART0 TX / RX to CM5 GPIO15 / GPIO14 | 2 | blue LED |
| 19 / 20 | USB D- / D+ | 0 / EN | BOOT / RESET (buttons + CM5 GPIO23 / GPIO22) |
| 12 13 14 21 47 48 | free (not routed) | | |

The DE (output-enable) of each bus driver is active high, like the DE pin of a MAX485, so ESP-IDF's
`UART_MODE_RS485_HALF_DUPLEX` can drive it as the RTS pin. If the firmware drives DE as a plain GPIO instead:
high while sending, low to listen (a 10k pull-down keeps the bus free while the ESP32 boots).

## Never back-power the CM5

The CM5 datasheet forbids any voltage on its pins while it is off. So: ESP32 -> CM5 UART goes through a
74LVC1G34 powered from the CM5's own 3.3 V, the CM5 controls ESP reset/boot only through 2N7002s, and the USB
switch is disabled when neither the CM5 nor a USB cable is present.

## Order notes (JLCPCB)

* 4 layers, 1.6 mm, JLC04161H-7628 stack-up is fine; min via 0.45/0.2 mm, min track/space 0.127 mm.
* The ESP32 antenna overhangs the top edge by ~6 mm on purpose (Espressif's best-RF placement).
* The CM5 board-to-board connectors (2x DF40C-100DS-0.4V / Amphenol 10164227-1001A1RLF) are fine-pitch: let JLC place them.
* Fill in the LCSC numbers in the BOM in the JLC BOM tool (MPNs are listed); the MPU-6050 is sometimes out of stock.
* Mount: M3 x 25 through PCB -> 10 mm spacer -> acrylic plate v3 -> chassis insert.

## Before the first power-up

1. Check the servo plug pin order (G V S) against a real STS3215 cable; check the 5264-type plug fits the 5268 header.
2. First power-up **without** CM5 and servos: 12 V on the pads, measure 5.0 V (green LED) and 3.3 V.
3. Flash the ESP32 over its USB-C, then fit the CM5 and flash its eMMC (`rpiboot`, RPIBOOT button).
4. Then servos, one leg at a time (same rules as before: torque on one servo at a time, 30 %).

## Known limits

* 12 V from the MOSFET into the 12 V plane goes through 5 + 8 vias (0.6/0.3 mm, ~1.5-2 A each): fine for walking
  (3-6 A), marginal if all 18 servos stall at once. For jumps, solder a short 1.5 mm2 wire from the Q1 source pad to C1+.
* JLC's standard 4-layer inner copper is 0.5 oz; the 12 V plane is wide enough (~1 mOhm), but order 1 oz inner if offered.
* The CM5 3D model is a simple stand-in (tools/cm5_wrl.py); the connector model is Amphenol's.
