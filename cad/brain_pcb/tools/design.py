"""ARACHNE-18 brain board: single source of truth for parts, nets and placement.

Board coordinates in mm, origin = board centre, +x = bus-A end (legs L1-L3),
+y = down in KiCad (bottom edge = USB / power side), top edge y = -33 = antenna edge.

Each part: ref, symbol (lib:name), footprint (lib:name), value, pin->net, (x, y, rot), extra.
Pins not listed are no-connect.
"""
import json, os

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD_W, BOARD_H, CORNER_R = 140.0, 70.0, 6.0
# chassis M3 insert holes: CAD (64, 25) vs hand-measured (65.1, 25.35) -> hole between them, 4.2 mm wide
MOUNT = (64.55, 25.18)

# Raspberry Pi CM5: footprint origin = a module mounting hole; rotated 180 so the antenna end
# (footprint +y) faces the top board edge.
CM5_ORIGIN = (16.5, -31.0)

parts = []


def P(ref, sym, fp, value, pins, at, mpn='', lcsc='', group='', dnp=False, side='F'):
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


# ---------------------------------------------------------------- power entry (12 V servo rail)
G = 'Voeding 12 V'
P('J1', 'Connector_Generic:Conn_01x02', 'arachne:SolderWire_2pad_P7mm_D2.2mm', '12V IN',
  {1: 'GND', 2: 'VIN_RAW'}, (-44, 30, 0), mpn='solder pads for 14-16 AWG', group=G)
P('Q1', 'arachne:Q_PMOS_GDS', 'Package_TO_SOT_SMD:TO-252-2', 'AOD4185',
  {1: 'Q1_G', 2: 'VIN_RAW', 3: 'V12'}, (-27.5, 29.5, 180), mpn='AOD4185 (P-MOSFET 40 V 40 A, reverse polarity)', group=G)
R('R1', '10k', 'Q1_G', 'GND', (-19.8, 25.4, 90), G)
P('D2', 'Device:D_Zener', 'Diode_SMD:D_SOD-123', 'BZT52C15', {1: 'V12', 2: 'Q1_G'}, (-19.8, 30.0, 90),
  mpn='BZT52C15 (gate clamp)', group=G)
P('D1', 'Device:D_TVS', 'Diode_SMD:D_SMB', 'SMBJ15CA', {1: 'V12', 2: 'GND'}, (-23.5, 15, 270),
  mpn='SMBJ15CA', group=G)
P('C1', 'Device:C_Polarized', 'Capacitor_SMD:CP_Elec_8x10.5', '330u 25V', {1: 'V12', 2: 'GND'}, (-41.5, 15, 270),
  mpn='330uF 25V 8x10.5 alu', group=G)
P('C2', 'Device:C_Polarized', 'Capacitor_SMD:CP_Elec_8x10.5', '330u 25V', {1: 'V12', 2: 'GND'}, (-31.5, 15, 270),
  mpn='330uF 25V 8x10.5 alu', group=G)
C('C3', '10u 25V', 'V12', 'GND', (-23.5, 7.5, 90), G, fp=C1206)
R('R2', '100k', 'V12', 'VSENSE', (27, 9.5, 90), G)
R('R3', '15k', 'VSENSE', 'GND', (28.6, 9.5, 90), G)
C('C4', '100n', 'VSENSE', 'GND', (30.2, 9.5, 90), G)

# ---------------------------------------------------------------- 5 V / 5 A buck (TI TPS54560B reference design)
G = 'Buck 12 V -> 5 V 5 A'
P('U1', 'Regulator_Switching:TPS54560BDDA', 'Package_SO:Texas_R-PDSO-G8_EP2.95x4.9mm_Mask2.4x3.1mm_ThermalVias',
  'TPS54560B', {1: 'BOOT', 2: 'V12', 3: 'BUCK_EN', 4: 'BUCK_RT', 5: 'BUCK_FB', 6: 'BUCK_COMP', 7: 'GND', 8: 'SW', 9: 'GND'},
  (-33, -13, 0), mpn='TPS54560BDDAR', group=G)
C('C5', '10u 25V', 'V12', 'GND', (-40.3, -13.6, 180), G, fp=C1206)
C('C6', '10u 25V', 'V12', 'GND', (-40.3, -19.4, 180), G, fp=C1206)
C('C7', '100n', 'V12', 'GND', (-39.7, -16.4, 180), G)
R('R4', '442k', 'V12', 'BUCK_EN', (-38.6, -6.5, 90), G)
R('R5', '90.9k', 'BUCK_EN', 'GND', (-37.0, -6.5, 90), G)
R('R6', '243k', 'BUCK_RT', 'GND', (-35.0, -7.5, 90), G)
C('C8', '100n', 'BOOT', 'SW', (-33.4, -17.2, 0), G)
P('L1', 'Device:L', 'Inductor_SMD:L_Bourns_SRP1038C_10.0x10.0mm', '6.8u', {1: 'SW', 2: 'V5'},
  (-35, -26.5, 180), mpn='Bourns SRP1038C-6R8M (6.8 uH, >= 8 A)', group=G)
P('D4', 'Device:D_Schottky', 'Diode_SMD:D_SMC', 'B560C', {1: 'SW', 2: 'GND'}, (-24, -20, 90),
  mpn='B560C (5 A 60 V Schottky)', group=G)
C('C9', '47u 10V', 'V5', 'GND', (-44.2, -30.5, 180), G, fp=C1210)
C('C10', '47u 10V', 'V5', 'GND', (-44.2, -26.8, 180), G, fp=C1210)
C('C11', '47u 10V', 'V5', 'GND', (-44.2, -23.1, 180), G, fp=C1210)
R('R7', '53.6k', 'V5', 'BUCK_FB', (-27, -11.5, 90), G)
R('R8', '10.2k', 'BUCK_FB', 'GND', (-25.4, -11.5, 90), G)
R('R9', '16.9k', 'BUCK_COMP', 'COMP_RC', (-27.8, -7.6, 0), G)
C('C12', '4n7', 'COMP_RC', 'GND', (-24.0, -7.6, 0), G)
C('C13', '47p', 'BUCK_COMP', 'GND', (-23.8, -11.5, 90), G)
P('D5', 'Device:LED', LED0603, 'groen 5V', {1: 'GND', 2: 'LED5_A'}, (-27.8, -4.6, 180), mpn='LED 0603 green', group=G)
R('R10', '2k2', 'V5', 'LED5_A', (-24.0, -4.6, 180), G)

# ---------------------------------------------------------------- 3.3 V for ESP32 + IMU
G = '3,3 V (ESP32 + IMU)'
P('D6', 'Diode:SS14', 'Diode_SMD:D_SMA', 'SS14', {1: 'V5_ESP', 2: 'V5'}, (26.5, 23.0, 0), mpn='SS14', group=G)
P('D7', 'Diode:SS14', 'Diode_SMD:D_SMA', 'SS14', {1: 'V5_ESP', 2: 'VBUS_ESP'}, (26.5, 27.0, 0), mpn='SS14', group=G)
P('U2', 'Regulator_Linear:AMS1117-3.3', 'Package_TO_SOT_SMD:SOT-223-3_TabPin2', 'AMS1117-3.3',
  {1: 'GND', 2: '3V3', 3: 'V5_ESP'}, (47, -6, 0), mpn='AMS1117-3.3', group=G)
C('C14', '10u', 'V5_ESP', 'GND', (41, -6.5, 90), G, fp=C0805)
C('C15', '22u', '3V3', 'GND', (48.3, -0.8, 0), G, fp=C0805)
C('C16', '100n', '3V3', 'GND', (44.6, -0.8, 0), G)

# ---------------------------------------------------------------- ESP32-S3
G = 'ESP32-S3 (servo-bus + IMU)'
P('U3', 'RF_Module:ESP32-S3-WROOM-1', 'RF_Module:ESP32-S3-WROOM-1', 'ESP32-S3-WROOM-1-N16R8',
  {1: 'GND', 40: 'GND', 41: 'GND', 2: '3V3', 3: 'ESP_EN', 4: 'IMU_SDA', 5: 'IMU_SCL', 6: 'IMU_INT',
   9: 'DE_A', 10: 'TX_A', 11: 'RX_A', 17: 'DE_B', 18: 'TX_B', 19: 'RX_B',
   13: 'ESP_DM', 14: 'ESP_DP', 27: 'ESP_IO0', 37: 'ESP_TX', 36: 'ESP_RX', 39: 'VSENSE', 38: 'LED_ESP',
},
  (34.5, -28.25, 0), mpn='ESP32-S3-WROOM-1-N16R8', group=G)
C('C17', '22u', '3V3', 'GND', (25.5, -9.0, 90), G, fp=C0805)
C('C18', '100n', '3V3', 'GND', (27.5, -9.0, 90), G)
R('R11', '10k', '3V3', 'ESP_EN', (45.2, -11.6, 0), G)
C('C19', '1u', 'ESP_EN', 'GND', (48.5, -11.6, 0), G)
R('R12', '10k', '3V3', 'ESP_IO0', (45.2, -13.4, 0), G)
P('SW1', 'Switch:SW_Push', 'Button_Switch_SMD:SW_Push_1P1T_NO_CK_KMR2', 'RESET', {1: 'ESP_EN', 2: 'GND'}, (50.3, -19.0, 0), mpn='CK KMR221G (4.2x2.8 tact)', group=G)
P('SW2', 'Switch:SW_Push', 'Button_Switch_SMD:SW_Push_1P1T_NO_CK_KMR2', 'BOOT', {1: 'ESP_IO0', 2: 'GND'}, (50.3, -15.2, 0), mpn='CK KMR221G (4.2x2.8 tact)', group=G)
P('Q2', 'Transistor_FET:2N7002', SOT23, '2N7002', {1: 'CM_ESP_EN', 2: 'GND', 3: 'ESP_EN'}, (23.5, -3, 90), group=G, mpn='2N7002')
R('R13', '100k', 'CM_ESP_EN', 'GND', (23.5, 0.5, 90), G)
P('Q3', 'Transistor_FET:2N7002', SOT23, '2N7002', {1: 'CM_ESP_BOOT', 2: 'GND', 3: 'ESP_IO0'}, (27, -3, 90), group=G, mpn='2N7002')
R('R14', '100k', 'CM_ESP_BOOT', 'GND', (27, 0.5, 90), G)
P('D8', 'Device:LED', LED0603, 'blauw ESP', {1: 'GND', 2: 'LED_ESP_A'}, (34.5, -11.2, 0), mpn='LED 0603 blue', group=G)
R('R15', '1k', 'LED_ESP', 'LED_ESP_A', (31.0, -11.2, 0), G)

# ---------------------------------------------------------------- servo bus A (legs 1-3) and B (legs 4-6)
for bus, sx, legs, refs in (('A', 1, ('L1', 'L2', 'L3'), (4, 5, 16, 17, 18, 19, 20, 21, 2, 3, 4)),
                            ('B', -1, ('L4', 'L5', 'L6'), (6, 7, 20, 21, 22, 23, 22, 23, 5, 6, 7))):
    G = f'Servo-bus {bus} ({", ".join(legs)})'
    u_tx, u_rx, r_ser, r_pu, r_de, r_rx, c_tx, c_rx, *jr = refs
    bx = sx * 57.5
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
        P(f'J{j}', 'Connector_Generic:Conn_01x03', 'arachne:Molex_5268-03A_1x03_P2.50mm_Vertical', f'{leg} servo',
          {1: 'GND', 2: 'V12', 3: f'BUS_{bus}'}, (sx * 64.3, y if sx > 0 else -y, 90 if sx > 0 else 270),
          mpn='Molex 5268-03A (5264 mating, Feetech bus)', group=G)

# ---------------------------------------------------------------- IMU
G = 'IMU MPU-6050'
P('U8', 'Sensor_Motion:MPU-6050', 'Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm', 'MPU-6050',
  {1: 'GND', 8: '3V3', 9: 'GND', 10: 'IMU_REG', 11: 'GND', 12: 'IMU_INT', 13: '3V3', 18: 'GND',
   20: 'IMU_CP', 23: 'IMU_SCL', 24: 'IMU_SDA'}, (34, 2.5, 0), mpn='MPU-6050', group=G)
C('C24', '100n', 'IMU_REG', 'GND', (30.5, 4.5, 90), G)
C('C25', '2n2', 'IMU_CP', 'GND', (37.5, 4.5, 90), G)
C('C26', '100n', '3V3', 'GND', (37.5, 0.5, 90), G)
C('C27', '10n', '3V3', 'GND', (30.5, 0.5, 90), G)
R('R24', '4k7', 'IMU_SDA', '3V3', (39.5, 0.5, 90), G)
R('R25', '4k7', 'IMU_SCL', '3V3', (41.1, 0.5, 90), G)

# ---------------------------------------------------------------- ESP <-> CM5 UART (no back-powering of CM5)
G = 'UART ESP32 <-> CM5'
P('U9', '74xGxx:74LVC1G34', SOT235, '74LVC1G34', {2: 'ESP_TX', 3: 'GND', 4: 'CM_RXD', 5: 'CM_3V3'},
  (23.5, 5, 90), mpn='74LVC1G34GW (Ioff: safe when CM5 is off)', group=G)
C('C28', '100n', 'CM_3V3', 'GND', (26.5, 5, 90), G)
R('R26', '1k', 'CM_TXD', 'ESP_RX', (23.5, 9.5, 90), G)

# ---------------------------------------------------------------- USB-C for the ESP32 (+ switch to CM5 USB host)
G = 'USB-C ESP32'
P('J8', 'Connector:USB_C_Receptacle_USB2.0_16P', USBC, 'USB-C ESP32',
  {'A1': 'GND', 'A12': 'GND', 'B1': 'GND', 'B12': 'GND', 'SH': 'GND', 'A4': 'VBUS_ESP', 'A9': 'VBUS_ESP',
   'B4': 'VBUS_ESP', 'B9': 'VBUS_ESP', 'A5': 'CC1_E', 'B5': 'CC2_E', 'A6': 'UC_DP', 'B6': 'UC_DP',
   'A7': 'UC_DM', 'B7': 'UC_DM'}, (38, 31.2, 0), mpn='HRO TYPE-C-31-M-12', group=G)
R('R27', '5k1', 'CC1_E', 'GND', (32.5, 23.5, 90), G)
R('R28', '5k1', 'CC2_E', 'GND', (43.5, 23.5, 90), G)
P('U10', 'Power_Protection:USBLC6-2SC6', 'Package_TO_SOT_SMD:SOT-23-6', 'USBLC6-2SC6',
  {1: 'UC_DP', 6: 'UC_DP', 3: 'UC_DM', 4: 'UC_DM', 2: 'GND', 5: 'VBUS_ESP'}, (38, 22.5, 0), mpn='USBLC6-2SC6', group=G)
C('C29', '1u', 'VBUS_ESP', 'GND', (44.5, 19, 90), G)
R('R29', '10k', 'VBUS_ESP', 'USB_SEL', (31.5, 18, 90), G)
R('R30', '20k', 'USB_SEL', 'GND', (29.9, 18, 90), G)
P('U11', 'Interface_USB:FSUSB42MUX', 'Package_SO:MSOP-10_3x3mm_P0.5mm', 'FSUSB42MUX',
  {1: '3V3', 2: 'USB_SEL', 3: 'ESP_DP', 4: 'ESP_DM', 5: 'GND', 6: 'CM_U3_DM', 7: 'CM_U3_DP', 8: 'UC_DM', 9: 'UC_DP',
   10: 'USB_OE_N'}, (38, 15, 0), mpn='FSUSB42MUX (SEL=0: CM5, SEL=1: USB-C)', group=G)
C('C30', '100n', '3V3', 'GND', (42.5, 15, 90), G)
R('R31', '10k', '3V3', 'USB_OE_N', (33.5, 15, 90), G)
P('Q4', 'Transistor_FET:2N7002', SOT23, '2N7002', {1: 'CM_3V3', 2: 'GND', 3: 'USB_OE_N'}, (34, 11.2, 0), mpn='2N7002', group=G)
P('Q5', 'Transistor_FET:2N7002', SOT23, '2N7002', {1: 'USB_SEL', 2: 'GND', 3: 'USB_OE_N'}, (40, 11.2, 0), mpn='2N7002', group=G)
R('R32', '100k', 'CM_3V3', 'GND', (30.5, 13.5, 90), G)

# ---------------------------------------------------------------- USB-C for CM5 (rpiboot / eMMC flashing)
G = 'USB-C CM5 (eMMC flashen)'
P('J9', 'Connector:USB_C_Receptacle_USB2.0_16P', USBC, 'USB-C CM5',
  {'A1': 'GND', 'A12': 'GND', 'B1': 'GND', 'B12': 'GND', 'SH': 'GND', 'A4': 'VBUS_CM', 'B9': 'VBUS_CM', 'A9': 'VBUS_CM', 'B4': 'VBUS_CM',
   'A5': 'CC1_C', 'B5': 'CC2_C', 'A6': 'CM_USB_P', 'B6': 'CM_USB_P',
   'A7': 'CM_USB_N', 'B7': 'CM_USB_N'}, (-12, 31.2, 0), mpn='HRO TYPE-C-31-M-12', group=G)
R('R33', '5k1', 'CC1_C', 'GND', (-17.5, 23.6, 90), G)
R('R34', '5k1', 'CC2_C', 'GND', (-6.5, 23.6, 90), G)
P('U12', 'Power_Protection:USBLC6-2SC6', 'Package_TO_SOT_SMD:SOT-23-6', 'USBLC6-2SC6',
  {1: 'CM_USB_P', 6: 'CM_USB_P', 3: 'CM_USB_N', 4: 'CM_USB_N', 2: 'GND', 5: 'VBUS_CM'}, (-12, 23.2, 0),
  mpn='USBLC6-2SC6', group=G)

# ---------------------------------------------------------------- CM5
G = 'Raspberry Pi CM5'
cm5_names = {int(k): v for k, v in json.load(open(os.path.join(HERE, 'cm5_pins.json'))).items()}
cm5 = {}
for n, name in cm5_names.items():
    if name == 'GND':
        cm5[n] = 'GND'
    elif name == '5V':
        cm5[n] = 'V5'
    elif name in ('CM5_3.3V', 'GPIO_VREF'):
        cm5[n] = 'CM_3V3'
cm5.update({55: 'CM_TXD', 51: 'CM_RXD', 58: 'CM_SDA', 56: 'CM_SCL', 46: 'CM_ESP_EN', 47: 'CM_ESP_BOOT',
            19: 'CM_FAN_PWM', 16: 'CM_FAN_TACH', 21: 'CM_LED_ACT', 92: 'CM_PWR_BUT', 93: 'CM_RPIBOOT',
            103: 'CM_USB_N', 105: 'CM_USB_P', 134: 'CM_U3_DP', 136: 'CM_U3_DM'})
assert cm5_names[55] == 'GPIO14' and cm5_names[51] == 'GPIO15' and cm5_names[58] == 'GPIO2'
assert cm5_names[46] == 'GPIO22' and cm5_names[47] == 'GPIO23' and cm5_names[134] == 'USB3-0-DP'
P('M1', 'arachne:CM5', 'arachne:Raspberry-Pi-5-Compute-Module', 'CM5 (wireless)', cm5, (CM5_ORIGIN[0], CM5_ORIGIN[1], 180),
  mpn='CM5104032 (4 GB, 32 GB eMMC, WiFi/BT) + 2x DF40C-100DS-0.4V', group=G)
C('C31', '47u 10V', 'V5', 'GND', (22.5, -15.0, 270), G, fp=C1210)
C('C32', '47u 10V', 'V5', 'GND', (22.5, -20.5, 90), G, fp=C1210)
C('C33', '100n', 'CM_3V3', 'GND', (22.5, -11.0, 90), G)
P('D9', 'Device:LED', LED0603, 'groen ACT', {1: 'CM_LED_ACT', 2: 'LED_ACT_A'}, (3.5, 23.3, 180), mpn='LED 0603 green', group=G)
R('R35', '470', 'CM_3V3', 'LED_ACT_A', (-0.2, 23.3, 180), G)
P('SW3', 'Switch:SW_Push', BTN, 'RPIBOOT', {1: 'CM_RPIBOOT', 2: 'GND'}, (-1.0, 30.5, 0), mpn='TL3342 / 5.2x5.2 tact', group=G)
P('SW4', 'Switch:SW_Push', BTN, 'AAN/UIT', {1: 'CM_PWR_BUT', 2: 'GND'}, (8.5, 30.5, 0), mpn='TL3342 / 5.2x5.2 tact', group=G)
P('J10', 'Connector_Generic_MountingPin:Conn_01x04_MountingPin', 'Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal',
  'FAN', {1: 'V5', 2: 'CM_FAN_PWM', 3: 'GND', 4: 'CM_FAN_TACH', 'MP': 'GND'}, (17.5, 30.5, 0), mpn='JST SM04B-SRSS-TB (Pi 5 fan)', group=G)

# ---------------------------------------------------------------- headers
G = 'Headers'
P('J11', 'Connector_Generic:Conn_02x05_Odd_Even', 'Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical', 'PI LINK',
  {2: 'V5', 3: 'CM_SDA', 4: 'V5', 5: 'CM_SCL', 6: 'GND', 7: 'ESP_EN', 8: 'ESP_RX', 9: 'GND', 10: 'ESP_TX'},
  (48.73, 8.62, 0), mpn='2x5 pin header 2.54 (= Pi header pins 1-10)', group=G)

# ---------------------------------------------------------------- mounting
for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1)), 1):
    P(f'H{i}', 'Mechanical:MountingHole', 'arachne:MountingHole_4.2mm_M3_loose', 'M3', {},
      (sx * MOUNT[0], sy * MOUNT[1], 0), group='Bevestiging')

NETCLASS = {
    'POWER12': (['VIN_RAW', 'V12'], 1.5, 0.2),
    'POWER5': (['V5', 'SW'], 0.6, 0.2),
    'PWR': (['3V3', 'CM_3V3', 'V5_ESP', 'VBUS_ESP', 'VBUS_CM', 'GND', 'BUS_A', 'BUS_B'], 0.35, 0.15),
    'USB': (['ESP_DP', 'ESP_DM', 'UC_DP', 'UC_DM', 'CM_U3_DP', 'CM_U3_DM', 'CM_USB_P', 'CM_USB_N'], 0.25, 0.15),
}


# Pi-4 build (2026-10-17): the CM5 is unaffordable/out of stock, so these CM5-only parts are not fitted
# (footprints stay on the board, so a CM4/CM5 can still be added later). The Pi 4 connects via PI LINK or ESP USB-C.
DNP = {'M1', 'U9', 'C28', 'R26', 'Q2', 'R13', 'Q3', 'R14', 'J9', 'U12', 'R33', 'R34', 'SW3', 'SW4', 'J10',
       'D9', 'R35', 'Q4', 'R32', 'C33'}
for _p in parts:
    _p['dnp'] = _p['ref'] in DNP


def nets():
    s = set()
    for p in parts:
        s |= set(p['pins'].values())
    return sorted(s)
