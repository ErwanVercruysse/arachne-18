"""ARACHNE-18 brain MINI board (Pi 4 over USB, no CM5): single source of truth for parts, nets and placement.

Board coordinates in mm, origin = board centre, +x = bus-A end (legs L1-L3),
+y = down in KiCad (bottom edge = 12 V pads / USB-C), top edge y = -30 = antenna edge.
The Raspberry Pi 4 talks to the ESP32-S3 over one USB-C (board) <-> USB-A (Pi) cable and gets 5 V from the
'5V PI' solder pads (buck output).

Each part: ref, symbol (lib:name), footprint (lib:name), value, pin->net, (x, y, rot), extra.
Pins not listed are no-connect.
"""
import json, os

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD_W, BOARD_H, CORNER_R = 92.0, 70.0, 6.0
BOARD_Y = (-30.0, 40.0)       # top (antenna) edge, bottom (sensor strip + USB-C) edge
MOUNT = (41.5, 25.5)          # M3 holes in the corners (fixed to the acrylic plate v4 with spacers)
ESP = (19.0, -23.25)          # ESP32-S3 module origin: antenna overhangs the top edge by ~6 mm

parts = []
DNP = set()                         # parts JLC does not place (hand-solder / optional)
HAND = "hand-solder (not placed by JLC): "


def P(ref, sym, fp, value, pins, at, mpn='', lcsc='', group='', dnp=False, side='F'):
    if dnp: DNP.add(ref)
    parts.append(dict(ref=ref, sym=sym, fp=fp, value=value, pins={str(k): v for k, v in pins.items()},
                      at=at, mpn=mpn, lcsc=lcsc, group=group, dnp=dnp, side=side))


R0603 = 'Resistor_SMD:R_0603_1608Metric'
C0603 = 'Capacitor_SMD:C_0603_1608Metric'
C0805 = 'Capacitor_SMD:C_0805_2012Metric'
C1206 = 'Capacitor_SMD:C_1206_3216Metric'
C1210 = 'Capacitor_SMD:C_1210_3225Metric'
LED0603 = 'LED_SMD:LED_0603_1608Metric'
SOT23 = 'Package_TO_SOT_SMD:SOT-23'
SOT235 = 'Package_TO_SOT_SMD:SOT-23-5'
BTN = 'Button_Switch_SMD:SW_SPST_TL3342'
USBC = 'Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12'


def R(ref, val, a, b, at, g, mpn=''):
    P(ref, 'Device:R', R0603, val, {1: a, 2: b}, at, mpn=mpn or f'0603 {val} 1%', group=g)


def C(ref, val, a, b, at, g, fp=C0603, mpn=''):
    size = fp.split(':')[1].split('_')[1]            # 0603 / 0805 / 1206 / 1210
    kind = 'X7R' if val.endswith(('n', 'p')) or val == '100n' else 'X5R'
    P(ref, 'Device:C', fp, val, {1: a, 2: b}, at, mpn=mpn or f'{size} {val} {kind} ceramic', group=g)



# block offsets: the buck and power-entry blocks keep the proven layout of the big board, just moved
BUCK, ENTRY = (15.0, 3.0), (16.5, -5.0)


def b(x, y, rot=0):
    return (x + BUCK[0], y + BUCK[1], rot)


def e(x, y, rot=0):
    return (x + ENTRY[0], y + ENTRY[1], rot)


# ---------------------------------------------------------------- power entry (12 V servo rail)
G = 'Voeding 12 V'
P('J1', 'Connector_Generic:Conn_01x02', 'arachne:SolderWire_2pad_P7mm_D2.2mm', '12V IN',
  {1: 'GND', 2: 'VIN_RAW'}, e(-44, 30), mpn='solder pads for 14-16 AWG', group=G)
P('Q1', 'arachne:Q_PMOS_GDS', 'Package_TO_SOT_SMD:TO-252-2', 'AOD4185',
  {1: 'Q1_G', 2: 'VIN_RAW', 3: 'V12'}, e(-27.5, 29.5, 180), mpn='AOD4185 (P-MOSFET 40 V 40 A, reverse polarity)', group=G)
R('R1', '10k', 'Q1_G', 'GND', e(-19.8, 25.4, 90), G)
P('D2', 'Device:D_Zener', 'Diode_SMD:D_SOD-123', 'BZT52C15', {1: 'V12', 2: 'Q1_G'}, e(-19.8, 30.0, 90),
  mpn='BZT52C15 gate clamp (optional: AOD4185 Vgs max 20 V)', group=G)
P('D1', 'Device:D_TVS', 'Diode_SMD:D_SMB', 'SMBJ15CA', {1: 'V12', 2: 'GND'}, e(-23.5, 15, 270),
  mpn='SMBJ15CA TVS (optional)', group=G)
P('C1', 'Device:C_Polarized', 'Capacitor_SMD:CP_Elec_8x10.5', '330u 25V', {1: 'V12', 2: 'GND'}, e(-41.5, 15, 270),
  mpn='330uF 25V 8x10.5 alu, STRONGLY recommended for the servo current peaks', group=G)
P('C2', 'Device:C_Polarized', 'Capacitor_SMD:CP_Elec_8x10.5', '330u 25V', {1: 'V12', 2: 'GND'}, e(-31.5, 15, 270),
  mpn='330uF 25V 8x10.5 alu, STRONGLY recommended for the servo current peaks', group=G)
C('C3', '10u 25V', 'V12', 'GND', e(-23.5, 7.5, 90), G, fp=C0805)
R('R2', '100k', 'V12', 'VSENSE', (25.5, 12.5, 90), G)
R('R3', '15k', 'VSENSE', 'GND', (27.1, 12.5, 90), G)
C('C4', '100n', 'VSENSE', 'GND', (28.7, 12.5, 90), G)

# ---------------------------------------------------------------- 5 V / 5 A buck (TI TPS54560B reference design)
G = 'Buck 12 V -> 5 V 5 A'
P('U1', 'Regulator_Switching:TPS54560BDDA', 'Package_SO:Texas_R-PDSO-G8_EP2.95x4.9mm_Mask2.4x3.1mm_ThermalVias',
  'TPS54560B', {1: 'BOOT', 2: 'V12', 3: 'BUCK_EN', 4: 'BUCK_RT', 5: 'BUCK_FB', 6: 'BUCK_COMP', 7: 'GND', 8: 'SW', 9: 'GND'},
  b(-33, -13, 0), mpn='TPS54560BDDAR', group=G)
C('C5', '10u 25V', 'V12', 'GND', b(-40.3, -13.6, 180), G, fp=C0805)
C('C6', '10u 25V', 'V12', 'GND', b(-40.3, -19.4, 180), G, fp=C0805)
C('C7', '100n', 'V12', 'GND', b(-39.7, -16.4, 180), G)
R('R4', '470k', 'V12', 'BUCK_EN', b(-38.6, -6.5, 90), G)
R('R5', '100k', 'BUCK_EN', 'GND', b(-37.0, -6.5, 90), G)
R('R6', '220k', 'BUCK_RT', 'GND', b(-35.0, -7.5, 90), G)
C('C8', '100n', 'BOOT', 'SW', b(-33.4, -17.2, 0), G)
P('L1', 'Device:L', 'Inductor_SMD:L_Bourns_SRP1038C_10.0x10.0mm', '6.8u', {1: 'SW', 2: 'V5'},
  b(-35, -26.5, 180), mpn='Bourns SRP1038C-6R8M (6.8 uH, >= 8 A)', group=G)
P('D4', 'Device:D_Schottky', 'Diode_SMD:D_SMA', 'SS54', {1: 'SW', 2: 'GND'}, b(-24, -20, 90),
  mpn='SS54 (5 A 40 V Schottky, JLC basic)', group=G)
C('C9', '47u 10V', 'V5', 'GND', b(-44.2, -30.5, 180), G, fp=C1206)
C('C10', '47u 10V', 'V5', 'GND', b(-44.2, -26.8, 180), G, fp=C1206)
C('C11', '47u 10V', 'V5', 'GND', b(-44.2, -23.1, 180), G, fp=C1206)
R('R7', '27k', 'V5', 'BUCK_FB', b(-27, -11.5, 90), G)
R('R8', '5k1', 'BUCK_FB', 'GND', b(-25.4, -11.5, 90), G)
R('R9', '15k', 'BUCK_COMP', 'COMP_RC', b(-27.8, -7.6, 0), G)
C('C12', '4n7', 'COMP_RC', 'GND', b(-24.0, -7.6, 0), G)
C('C13', '47p', 'BUCK_COMP', 'GND', b(-23.8, -11.5, 90), G)
P('D5', 'Device:LED', LED0603, 'LED 5V', {1: 'GND', 2: 'LED5_A'}, b(-27.8, -4.6, 180), mpn='LED 0603 red (JLC basic)', group=G)
R('R10', '2k2', 'V5', 'LED5_A', b(-24.0, -4.6, 180), G)
P('J13', 'Connector_Generic:Conn_01x02', 'arachne:SolderWire_2pad_P5mm_D1.6mm_5V', '5V PI',
  {1: 'GND', 2: 'V5'}, (-1.0, -25.5, 0), mpn='solder pads: 5 V 3 A to the Pi 4 (GPIO pin 2/4 + 6, or a USB-C lead)', group=G)

# ---------------------------------------------------------------- 3.3 V for ESP32 + IMU
G = '3,3 V (ESP32 + IMU)'
P('D6', 'Diode:SS14', 'Diode_SMD:D_SMA', 'SS14', {1: 'V5_ESP', 2: 'V5'}, (10.0, 17.5, 0), mpn='SS14', group=G)
P('D7', 'Diode:SS14', 'Diode_SMD:D_SMA', 'SS14', {1: 'V5_ESP', 2: 'VBUS_ESP'}, (10.0, 21.5, 0), mpn='SS14', group=G)
P('U2', 'Regulator_Linear:AMS1117-3.3', 'Package_TO_SOT_SMD:SOT-223-3_TabPin2', 'AMS1117-3.3',
  {1: 'GND', 2: '3V3', 3: 'V5_ESP'}, (19.5, 4.0, 0), mpn='AMS1117-3.3', group=G)
C('C14', '10u', 'V5_ESP', 'GND', (13.5, 3.5, 90), G, fp=C0805)
C('C15', '22u', '3V3', 'GND', (20.8, 9.2, 0), G, fp=C0805)
C('C16', '100n', '3V3', 'GND', (17.1, 9.2, 0), G)

# ---------------------------------------------------------------- free ESP32-S3 GPIOs -> GPIO header J14
# (IO35-37 are taken by the octal PSRAM of the N16R8; IO3, IO45, IO46 are strapping pins: no pull at boot)
GPIO_PINS = {'IO7': 7, 'IO15': 8, 'IO8': 12, 'IO3': 15, 'IO46': 16, 'IO12': 20, 'IO13': 21, 'IO14': 22, 'IO21': 23,
             'IO47': 24, 'IO48': 25, 'IO45': 26, 'IO38': 31, 'IO39': 32, 'IO40': 33, 'IO41': 34, 'IO42': 35,
             'IO44': 36, 'IO43': 37}
# sensor strip along the bottom edge: one G-V-S column per GPIO (like an Arduino sensor shield), so a standard
# 3-wire sensor cable (GND, VCC, signal, 2.54 mm) plugs straight on. V = 3.3 V (ESP32 pins are NOT 5 V tolerant).
STRIP = ['IO7', 'IO15', 'IO8', 'IO3', 'IO46', 'IO12', 'IO13', 'IO14', 'IO21', 'IO47', 'IO48', 'IO45', 'IO38', 'IO39',
         'IO40', 'IO41', 'IO42', 'IO43', 'IO44']
STRIP_LABEL = {'IO43': 'TX43', 'IO44': 'RX44'}
STRIP_X0, STRIP_P = -24.5, 2.54          # x of the first column, pitch
STRIP_Y = (32.4, 34.94, 37.48)           # rows S, V, G

# ---------------------------------------------------------------- ESP32-S3
G = 'ESP32-S3 (servo-bus + IMU + USB naar Pi)'
P('U3', 'RF_Module:ESP32-S3-WROOM-1', 'RF_Module:ESP32-S3-WROOM-1', 'ESP32-S3-WROOM-1-N16R8',
  {1: 'GND', 40: 'GND', 41: 'GND', 2: '3V3', 3: 'ESP_EN', 4: 'IMU_SDA', 5: 'IMU_SCL', 6: 'IMU_INT',
   9: 'DE_A', 10: 'TX_A', 11: 'RX_A', 17: 'DE_B', 18: 'TX_B', 19: 'RX_B',
   13: 'ESP_DM', 14: 'ESP_DP', 27: 'ESP_IO0', 39: 'VSENSE', 38: 'LED_ESP',
   **{n: g for g, n in GPIO_PINS.items()}},
  (ESP[0], ESP[1], 0), mpn='ESP32-S3-WROOM-1-N16R8', group=G)
C('C17', '22u', '3V3', 'GND', (9.6, -6.5, 90), G, fp=C0805)
C('C18', '100n', '3V3', 'GND', (11.4, -6.5, 90), G)
R('R15', '1k', 'LED_ESP', 'LED_ESP_A', (13.2, -6.5, 90), G)
P('D8', 'Device:LED', LED0603, 'LED ESP', {1: 'GND', 2: 'LED_ESP_A'}, (15.0, -6.5, 270), mpn='LED 0603 red (JLC basic)', group=G)
R('R11', '10k', '3V3', 'ESP_EN', (18.4, -6.5, 90), G)
C('C19', '1u', 'ESP_EN', 'GND', (20.2, -6.5, 90), G)
R('R12', '10k', '3V3', 'ESP_IO0', (22.0, -6.5, 90), G)
P('SW1', 'Switch:SW_Push', 'Button_Switch_SMD:SW_Push_1P1T_NO_CK_KMR2', 'RESET', {1: 'ESP_EN', 2: 'GND'}, (26.6, -6.2, 0), mpn='CK KMR221G (4.2x2.8 tact); the S3 flashes over native USB without it', group=G)
P('SW2', 'Switch:SW_Push', 'Button_Switch_SMD:SW_Push_1P1T_NO_CK_KMR2', 'BOOT', {1: 'ESP_IO0', 2: 'GND'}, (26.6, -2.0, 0), mpn='CK KMR221G (4.2x2.8 tact); the S3 flashes over native USB without it', group=G)

# ---------------------------------------------------------------- servo bus A (legs 1-3) and B (legs 4-6)
for bus, sx, legs, refs in (('A', 1, ('L1', 'L2', 'L3'), (4, 5, 16, 17, 18, 19, 20, 21, 2, 3, 4)),
                            ('B', -1, ('L4', 'L5', 'L6'), (6, 7, 20, 21, 22, 23, 22, 23, 5, 6, 7))):
    G = f'Servo-bus {bus} ({", ".join(legs)})'
    u_tx, u_rx, r_ser, r_pu, r_de, r_rx, c_tx, c_rx, *jr = refs
    bx = sx * 33.7
    P(f'U{u_tx}', '74xGxx:74AHCT1G126', SOT235, '74AHCT1G126',
      {1: f'DE_{bus}', 2: f'TX_{bus}', 3: 'GND', 4: f'BUS{bus}_DRV', 5: 'V5'}, (bx, -8, 90 if sx > 0 else 270),
      mpn='74AHCT1G126GW (TX driver, 5 V out)', group=G)
    P(f'U{u_rx}', '74xGxx:74LVC1G34', SOT235, '74LVC1G34',
      {2: f'BUS{bus}_RXIN', 3: 'GND', 4: f'RX_{bus}', 5: '3V3'}, (bx, 7, 90 if sx > 0 else 270),
      mpn='74LVC1G34GW (RX buffer, 5 V tolerant)', group=G)
    R(f'R{r_ser}', '47', f'BUS{bus}_DRV', f'BUS_{bus}', (bx, -3.5, 90), G)
    R(f'R{r_pu}', '10k', f'BUS_{bus}', 'V5', (bx - sx * 2, -3.5, 90), G)
    R(f'R{r_de}', '10k', f'DE_{bus}', 'GND', (bx, -12.5, 90), G)
    R(f'R{r_rx}', '1k', f'BUS_{bus}', f'BUS{bus}_RXIN', (bx, 2.5, 90), G)
    C(f'C{c_tx}', '100n', 'V5', 'GND', (bx - sx * 3, -8, 90), G)
    C(f'C{c_rx}', '100n', '3V3', 'GND', (bx - sx * 3, 7, 90), G)
    for j, leg, y in zip(jr, legs, (-13.5, 0, 13.5)):
        P(f'J{j}', 'Connector_Generic:Conn_01x03', 'arachne:Molex_5267-03A_1x03_P2.50mm_Vertical', f'{leg} servo',
          {1: 'GND', 2: 'V12', 3: f'BUS_{bus}'}, (sx * 40.5, y if sx > 0 else -y, 90 if sx > 0 else 270),
          mpn='Molex 22-03-5035 / 5267-03A Mini-SPOX (mates with the 5264 plug on Feetech cables), LCSC C45581067', group=G)

# ---------------------------------------------------------------- IMU (near the middle of the robot)
G = 'IMU LSM6DS3TR-C'
IM = (0.0, 0.0)                     # exact robot centre (mount holes are symmetric around 0,0)
P('U8', 'Sensor_Motion:LSM6DS3', 'Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y', 'LSM6DS3TR-C',
  {1: 'GND', 2: 'GND', 3: 'GND', 4: 'IMU_INT', 5: '3V3', 6: 'GND', 7: 'GND', 8: '3V3', 12: '3V3', 13: 'IMU_SCL', 14: 'IMU_SDA'},
  (IM[0], IM[1], 0), mpn='LSM6DS3TR-C (6-axis IMU, I2C 0x6A)', group=G)
for ref, val, dx, dy in (('C24', '100n', -2.8, 2.2), ('C26', '100n', 2.8, -2.2)):
    C(ref, val, '3V3', 'GND', (IM[0] + dx, IM[1] + dy, 90), G)
R('R24', '4k7', 'IMU_SDA', '3V3', (IM[0] + 5.5, IM[1] - 2, 90), G)
R('R25', '4k7', 'IMU_SCL', '3V3', (IM[0] + 7.1, IM[1] - 2, 90), G)

# ---------------------------------------------------------------- USB-C: ESP32 native USB -> Raspberry Pi 4 (C to A cable)
G = 'USB-C naar Pi 4'
P('J8', 'Connector:USB_C_Receptacle_USB2.0_16P', USBC, 'USB-C PI',
  {'A1': 'GND', 'A12': 'GND', 'B1': 'GND', 'B12': 'GND', 'SH': 'GND', 'A4': 'VBUS_ESP', 'A9': 'VBUS_ESP',
   'B4': 'VBUS_ESP', 'B9': 'VBUS_ESP', 'A5': 'CC1_E', 'B5': 'CC2_E', 'A6': 'ESP_DP', 'B6': 'ESP_DP',
   'A7': 'ESP_DM', 'B7': 'ESP_DM'}, (31.5, 36.2, 0), mpn='HRO TYPE-C-31-M-12', group=G)
R('R27', '5k1', 'CC1_E', 'GND', (27.0, 28.5, 90), G)
R('R28', '5k1', 'CC2_E', 'GND', (36.0, 28.5, 90), G)
P('U10', 'Power_Protection:USBLC6-2SC6', 'Package_TO_SOT_SMD:SOT-23-6', 'USBLC6-2SC6',
  {1: 'ESP_DP', 6: 'ESP_DP', 3: 'ESP_DM', 4: 'ESP_DM', 2: 'GND', 5: 'VBUS_ESP'}, (31.5, 27.5, 0), mpn='USBLC6-2SC6 ESD (optional)', group=G)
C('C29', '1u', 'VBUS_ESP', 'GND', (27.0, 25.0, 90), G)

# ---------------------------------------------------------------- sensor strip (S / V / G rows) + 2x Qwiic I2C
G = 'Sensor-strook (G V S per GPIO) + I2C'
for ref, row, nets_ in (('J14', 0, STRIP), ('J15', 1, ['3V3'] * len(STRIP)), ('J16', 2, ['GND'] * len(STRIP))):
    P(ref, 'Connector_Generic:Conn_01x19', 'Connector_PinHeader_2.54mm:PinHeader_1x19_P2.54mm_Vertical',
      ('SENSOR ' + 'SVG'[row]), {i + 1: n for i, n in enumerate(nets_)}, (STRIP_X0, STRIP_Y[row], 90),
      mpn='1x19 pin header 2.54 mm (row ' + 'SVG'[row] + ' of the sensor strip)', group=G)
for ref, x in (('J17', -40.0), ('J18', -31.0)):
    P(ref, 'Connector_Generic_MountingPin:Conn_01x04_MountingPin', 'Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal',
      'I2C', {1: 'GND', 2: '3V3', 3: 'IMU_SDA', 4: 'IMU_SCL', 'MP': 'GND'}, (x, 35.5, 180), mpn='JST SM04B-SRSS-TB (Qwiic / STEMMA QT, optional)', group=G)

# ---------------------------------------------------------------- mounting
for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1)), 1):
    P(f'H{i}', 'Mechanical:MountingHole', 'arachne:MountingHole_3.2mm_M3', 'M3', {},
      (sx * MOUNT[0], sy * MOUNT[1], 0), group='Bevestiging')

NETCLASS = {
    'POWER12': (['VIN_RAW', 'V12'], 1.5, 0.2),
    'POWER5': (['V5', 'SW'], 0.6, 0.2),
    'PWR': (['3V3', 'V5_ESP', 'VBUS_ESP', 'GND', 'BUS_A', 'BUS_B'], 0.35, 0.15),
    'USB': (['ESP_DP', 'ESP_DM'], 0.25, 0.15),
}



def nets():
    s = set()
    for p in parts:
        s |= set(p['pins'].values())
    return sorted(s)


def outline(inset=0.0, n=10):
    """rounded rectangle (closed), optionally inset"""
    import math
    x0, y0, x1, y1 = -BOARD_W / 2 + inset, BOARD_Y[0] + inset, BOARD_W / 2 - inset, BOARD_Y[1] - inset
    r = CORNER_R - inset; pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n); pts.append((round(cx + r * math.cos(a), 4), round(cy + r * math.sin(a), 4)))
    return pts + pts[:1]


if __name__ == '__main__':
    json.dump({'edge': outline(), 'inner': outline(0.3)}, open(os.path.join(HERE, 'outline.json'), 'w'))
    print('outline.json written')
