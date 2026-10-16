// ESP32-C3 Super Mini -> Feetech STS3215 bus over ONE wire (half-duplex UART).
//
// Wiring: GPIO21 --[560 ohm]-- servo signal line (all servos daisy-chained),
// shared GND, servos on 12 V. UART1 TX and RX are both routed to GPIO21.
// Every byte we send is received back (echo) and discarded before the
// servo's reply is parsed, so our own transmission is ignored.
//
// USB serial (115200), one command per line:
//   scan                 list servo IDs on the bus (1..30)
//   id <old> <new>       change a servo ID (permanent, EEPROM)
//   demo <a> <b>         a clockwise, b anticlockwise, swap every 3 turns
//   stop                 all servos: speed 0, torque off
//   cw <sign>            which speed sign is clockwise (+1 or -1)
//   swing <id> [deg]     position mode: +-deg around the current position (default 45)
//   lift <id> [deg]      torque off, let the arm hang down, then 0 -> deg -> 0 ... (default 90;
//                        negative deg lifts the other way)
//   lifthere <id> [deg]  same, but the current position is the start (no hanging first)
//   between <id> <a> <b> swing between two raw encoder positions (measured by hand), no offset
//   bridge               transparent USB <-> servo bus bridge for host tools (bam_identify.py);
//                        whole packets are forwarded, echo removed. Leave with the line ~~~
// Stall guard: load above 95 % for 300 ms switches torque off and stops.
//                        and print load, current and temperature per swing

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_reg.h"

constexpr int BUS_PIN = 21;
constexpr uint32_t BAUD = 1000000;  // STS3215 factory default
constexpr uint8_t BROADCAST = 0xFE;

// STS3215 registers
constexpr uint8_t REG_ID = 5, REG_OFFSET = 31, REG_MODE = 33, REG_TORQUE = 40, REG_ACCEL = 41,
                  REG_GOAL = 42, REG_SPEED = 46, REG_LOCK = 55, REG_POSITION = 56,
                  REG_LOAD = 60, REG_CURRENT = 69;

// ---------------------------------------------------------------- one-wire bus
// Direction is switched with the raw output-enable register. The IDF helper
// gpio_set_direction() would re-route the pin to plain GPIO and cut UART TX off.
void busTransmitMode() { REG_WRITE(GPIO_ENABLE_W1TS_REG, 1UL << BUS_PIN); }
void busReceiveMode()  { REG_WRITE(GPIO_ENABLE_W1TC_REG, 1UL << BUS_PIN); }

void busBegin() {
  Serial1.begin(BAUD, SERIAL_8N1, BUS_PIN, BUS_PIN);
  // Take the pin away from its IO_MUX default (U0TXD on the C3) and wire both
  // UART1 signals to it through the GPIO matrix.
  esp_rom_gpio_pad_select_gpio(BUS_PIN);
  gpio_set_direction((gpio_num_t)BUS_PIN, GPIO_MODE_INPUT);   // input buffer on, output off
  gpio_set_pull_mode((gpio_num_t)BUS_PIN, GPIO_PULLUP_ONLY);  // idle high when nobody drives
  esp_rom_gpio_connect_in_signal(BUS_PIN, U1RXD_IN_IDX, false);
  // UART1 TX drives the pin, but output-enable comes from GPIO_ENABLE (OEN_SEL=1),
  // so we decide when the ESP drives the line and when the servo may answer.
  REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * BUS_PIN, U1TXD_OUT_IDX | (1UL << GPIO_FUNC0_OEN_SEL_S));
  busReceiveMode();
}

// Send a packet, then swallow our own echo. Returns false if the echo is wrong,
// which means the line is not getting through (wiring, resistor, baud rate).
bool busSend(const uint8_t *data, size_t len) {
  while (Serial1.available()) Serial1.read();  // drop stale bytes
  busTransmitMode();
  Serial1.write(data, len);
  Serial1.flush();       // blocks until the last stop bit has left the pin
  busReceiveMode();      // release the line immediately so the servo can answer
  uint32_t t0 = micros();
  size_t got = 0;
  bool ok = true;
  while (got < len && micros() - t0 < 2000) {
    if (Serial1.available()) { if (Serial1.read() != data[got]) ok = false; got++; }
  }
  return ok && got == len;
}

enum { RX_TIMEOUT = -1, RX_CORRUPT = -2 };

// Read a status packet: FF FF ID LEN ERR PARAMS... CHK. Returns param count,
// RX_TIMEOUT (nobody answered) or RX_CORRUPT (bad checksum: e.g. two servos
// with the same ID answering at once).
int busReceive(uint8_t id, uint8_t *params, size_t maxParams, uint32_t timeoutUs = 3000) {
  uint8_t buf[64];
  size_t n = 0;
  bool sawBytes = false;
  uint32_t t0 = micros();
  while (micros() - t0 < timeoutUs && n < sizeof(buf)) {
    if (!Serial1.available()) continue;
    buf[n++] = Serial1.read();
    sawBytes = true;
    if (n == 1 && buf[0] != 0xFF) n = 0;               // resync on header
    if (n == 2 && buf[1] != 0xFF) n = 0;
    if (n >= 4 && n == (size_t)buf[3] + 4) {           // complete packet
      uint8_t sum = 0;
      for (size_t i = 2; i < n - 1; i++) sum += buf[i];
      if ((uint8_t)~sum != buf[n - 1] || buf[2] != id) return RX_CORRUPT;
      size_t count = buf[3] - 2;
      if (count > maxParams) count = maxParams;
      memcpy(params, buf + 5, count);
      return count;
    }
  }
  return sawBytes ? RX_CORRUPT : RX_TIMEOUT;
}

size_t packet(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t count, uint8_t *out) {
  out[0] = out[1] = 0xFF; out[2] = id; out[3] = count + 2; out[4] = instr;
  uint8_t sum = id + count + 2 + instr;
  for (uint8_t i = 0; i < count; i++) { out[5 + i] = params[i]; sum += params[i]; }
  out[5 + count] = ~sum;
  return 6 + count;
}

bool writeReg(uint8_t id, uint8_t reg, const uint8_t *data, uint8_t count) {
  uint8_t params[16], pkt[24];
  params[0] = reg; memcpy(params + 1, data, count);
  size_t len = packet(id, 0x03, params, count + 1, pkt);
  if (!busSend(pkt, len)) return false;
  if (id == BROADCAST) return true;  // broadcast gets no reply
  uint8_t reply[4];
  return busReceive(id, reply, sizeof(reply)) >= 0;
}

bool writeByte(uint8_t id, uint8_t reg, uint8_t value) { return writeReg(id, reg, &value, 1); }

int ping(uint8_t id) {
  uint8_t pkt[8], reply[4];
  size_t len = packet(id, 0x01, nullptr, 0, pkt);
  if (!busSend(pkt, len)) return RX_TIMEOUT;
  return busReceive(id, reply, sizeof(reply), 2000);
}

int readPosition(uint8_t id) {
  uint8_t params[2] = {REG_POSITION, 2}, pkt[16], reply[4];
  size_t len = packet(id, 0x02, params, 2, pkt);
  if (!busSend(pkt, len) || busReceive(id, reply, sizeof(reply)) != 2) return -1;
  return (reply[0] | (reply[1] << 8)) & 0x0FFF;
}

// Wheel-mode speed: magnitude in steps/s (4096 steps = 1 turn), bit 15 = reverse.
bool setSpeed(uint8_t id, int stepsPerSecond) {
  uint16_t v = abs(stepsPerSecond);
  if (stepsPerSecond < 0) v |= 0x8000;
  uint8_t d[2] = {(uint8_t)v, (uint8_t)(v >> 8)};
  return writeReg(id, REG_SPEED, d, 2);
}

// Wheel mode is written to RAM only (EEPROM stays locked): after a power cycle
// the servo is back in normal position mode.
bool wheelStart(uint8_t id) {
  return writeByte(id, REG_MODE, 1) && writeByte(id, REG_ACCEL, 20) && writeByte(id, REG_TORQUE, 1);
}

bool changeId(uint8_t from, uint8_t to) {
  if (ping(to) != RX_TIMEOUT) { Serial.printf("ID %d is already in use\n", to); return false; }
  bool ok = writeByte(from, REG_LOCK, 0);   // unlock EEPROM
  ok = ok && writeByte(from, REG_ID, to);   // servo answers this write with its OLD id
  delay(20);
  ok = ok && writeByte(to, REG_LOCK, 1);    // lock again under the new id
  return ok && ping(to) >= 0;
}

// ---------------------------------------------------------------- demo
int cwSign = +1;       // speed sign that turns clockwise; set with "cw -1" if wrong
int demoSpeed = 3000;  // ~264 deg/s
struct Demo { bool on = false; uint8_t a = 1, b = 2; int dir = +1; long turned = 0; int last = -1; } demo;
struct Swing { bool on = false, lift = false; uint8_t id = 2; int center = 2048, amp = 512, side = +1; uint32_t t0 = 0;
               int peakLoad = 0, peakCurrent = 0; } swing;

void demoApply() {
  setSpeed(demo.a, cwSign * demo.dir * demoSpeed);
  setSpeed(demo.b, -cwSign * demo.dir * demoSpeed);
  Serial.printf("servo %d %s, servo %d %s\n", demo.a, demo.dir > 0 ? "clockwise" : "anticlockwise",
                demo.b, demo.dir > 0 ? "anticlockwise" : "clockwise");
}

void demoStart(uint8_t a, uint8_t b) {
  if (!wheelStart(a) || !wheelStart(b)) { Serial.println("a servo did not answer; run scan"); return; }
  demo.on = true; demo.a = a; demo.b = b; demo.dir = +1; demo.turned = 0; demo.last = readPosition(a);
  demoApply();
}

void demoStep() {
  // Count servo a's rotation by unwrapping its 0..4095 encoder.
  int pos = readPosition(demo.a);
  if (pos < 0) return;
  if (demo.last >= 0) {
    int d = pos - demo.last;
    if (d > 2048) d -= 4096;
    if (d < -2048) d += 4096;
    demo.turned += abs(d);
  }
  demo.last = pos;
  if (demo.turned >= 3L * 4096) {  // three full turns: swap both directions
    demo.turned = 0; demo.dir = -demo.dir;
    demoApply();
  }
}

void stopAll() {
  demo.on = false;
  swing.on = false;
  setSpeed(BROADCAST, 0);
  writeByte(BROADCAST, REG_TORQUE, 0);
  Serial.println("stopped, torque off");
}

// ---------------------------------------------------------------- swing (position mode)



bool setGoal(uint8_t id, int pos, int speed) {
  pos = constrain(pos, 0, 4095);
  uint8_t d[6] = {(uint8_t)pos, (uint8_t)(pos >> 8), 0, 0, (uint8_t)speed, (uint8_t)(speed >> 8)};
  return writeReg(id, REG_GOAL, d, 6);
}

// swing: centre +- amp.  lift: centre (hanging down) and centre + amp.
int swingGoal() {
  if (swing.lift) return swing.center + (swing.side > 0 ? swing.amp : 0);
  return swing.center + swing.side * swing.amp;
}

void swingGo() {
  setGoal(swing.id, swingGoal(), 1500);  // ~130 deg/s
  swing.t0 = millis(); swing.peakLoad = swing.peakCurrent = 0;
}

void swingStart(uint8_t id, int deg, bool lift = false, bool hang = true) {
  demo.on = false; swing.on = false;
  if (lift && hang) {
    // Let the arm fall and settle: gravity finds "down" for us.
    writeByte(id, REG_TORQUE, 0);
    Serial.println("torque off, waiting for the arm to hang still");
    int last = -1; uint32_t still = millis(), t0 = millis();
    while (millis() - still < 1500 && millis() - t0 < 15000) {
      int p = readPosition(id);
      if (p >= 0 && last >= 0 && abs(p - last) > 3) still = millis();
      if (p >= 0) last = p;
      delay(50);
    }
  }
  int pos = -1;
  for (int tries = 0; tries < 10 && pos < 0; tries++) { pos = readPosition(id); if (pos < 0) delay(10); }
  if (pos < 0) { Serial.printf("servo %d does not answer\n", id); return; }
  // Shift the encoder zero (RAM only, EEPROM stays locked) so the current
  // position reads 2048: the swing then never crosses the 4095 -> 0 wrap.
  uint8_t params[2] = {REG_OFFSET, 2}, pkt[16], r[4];
  if (busSend(pkt, packet(id, 0x02, params, 2, pkt)) && busReceive(id, r, sizeof(r)) == 2) {
    int raw = r[0] | r[1] << 8;
    int offset = (raw & 0x800) ? -(raw & 0x7FF) : (raw & 0x7FF);
    offset += pos - 2048;
    if (offset > 2047) offset -= 4096;
    if (offset < -2047) offset += 4096;
    uint16_t v = abs(offset) | (offset < 0 ? 0x800 : 0);
    uint8_t d[2] = {(uint8_t)v, (uint8_t)(v >> 8)};
    writeReg(id, REG_OFFSET, d, 2);
    delay(20);
    pos = readPosition(id);
    Serial.printf("offset %d, position now %d\n", offset, pos);
  }
  // Position mode in RAM; hold the current position before enabling torque.
  writeByte(id, REG_MODE, 0); setGoal(id, pos, 1500); writeByte(id, REG_ACCEL, 50); writeByte(id, REG_TORQUE, 1);
  swing.on = true; swing.lift = lift; swing.id = id; swing.center = pos; swing.amp = constrain(deg, -170, 170) * 4096 / 360; swing.side = +1;
  if (lift) Serial.printf("servo %d lifts 0 -> %d deg, around position %d\n", id, deg, pos);
  else Serial.printf("servo %d swings +-%d deg around position %d\n", id, deg, pos);
  Serial.println("columns D,ms,goal,pos,speed,load,current,volts,temp | S,side,ms,error,peak_load_pct,peak_mA,volts,temp");
  swingGo();
}

// Swing between two raw positions, e.g. read with "scan" after moving the arm
// by hand. The encoder zero is left alone, so the direction is exactly the
// one that was shown by hand.
void betweenStart(uint8_t id, int from, int to) {
  demo.on = false; swing.on = false;
  int pos = readPosition(id);
  if (pos < 0) { Serial.printf("servo %d does not answer\n", id); return; }
  writeByte(id, REG_MODE, 0); setGoal(id, pos, 300); writeByte(id, REG_ACCEL, 50); writeByte(id, REG_TORQUE, 1);
  // side -1 = the start position: the first move goes there, logged and stall-guarded.
  swing.on = true; swing.lift = true; swing.id = id; swing.center = from; swing.amp = to - from; swing.side = -1;
  Serial.printf("servo %d swings between %d and %d (%+.1f deg), around position %d\n", id, from, to,
                (to - from) * 360.0 / 4096, from);
  Serial.println("columns D,ms,goal,pos,speed,load,current,volts,temp | S,side,ms,error,peak_load_pct,peak_mA,volts,temp");
  swingGo();
}

// One bus read of registers 56..70: position, speed, load, volts, temp, ..., current.
struct Sample { int pos, speed, load, current; float volts; int temp; };
bool readSample(uint8_t id, Sample &o) {
  uint8_t params[2] = {REG_POSITION, 15}, pkt[16], r[16];
  size_t len = packet(id, 0x02, params, 2, pkt);
  if (!busSend(pkt, len) || busReceive(id, r, sizeof(r)) != 15) return false;
  auto s16 = [](uint8_t lo, uint8_t hi, int bit) { int v = lo | hi << 8; return (v & (1 << bit)) ? -(v & ((1 << bit) - 1)) : v; };
  o.pos = (r[0] | r[1] << 8) & 0x0FFF;
  o.speed = s16(r[2], r[3], 15);   // steps/s
  o.load = s16(r[4], r[5], 10);    // 0.1 % of max torque
  o.volts = r[6] / 10.0f;
  o.temp = r[7];
  o.current = s16(r[13], r[14], 15);  // 6.5 mA per unit
  return true;
}

void swingStep() {
  Sample m;
  if (!readSample(swing.id, m)) return;
  int goal = swingGoal();
  // Raw data line for the Mac logger: D,ms,goal,pos,speed,load,current,volts,temp
  Serial.printf("D,%lu,%d,%d,%d,%d,%d,%.1f,%d\n", (unsigned long)millis(), goal, m.pos, m.speed, m.load,
                m.current, m.volts, m.temp);
  swing.peakLoad = max(swing.peakLoad, abs(m.load));
  // Stall guard: the arm is blocked (or far too heavy). Let go before the
  // servo overheats or its overload protection trips.
  static uint32_t stallStart = 0;
  if (abs(m.load) < 950) stallStart = 0;
  else if (!stallStart) stallStart = millis();
  else if (millis() - stallStart > 300) {
    stallStart = 0;
    writeByte(swing.id, REG_TORQUE, 0);
    swing.on = false;
    Serial.printf("STALL: load %.1f %% for 300 ms, torque off. Blocked arm or wrong direction?\n", m.load / 10.0);
    return;
  }
  swing.peakCurrent = max(swing.peakCurrent, abs(m.current));
  uint32_t elapsed = millis() - swing.t0;
  // Reached: close to the goal, or stopped near it (heavy loads settle a few
  // degrees short because the servo's P controller needs an error to push).
  static uint32_t stoppedSince = 0;
  if (abs(m.speed) > 20 || abs(m.pos - goal) > 120) stoppedSince = 0;
  else if (!stoppedSince) stoppedSince = millis();
  bool reached = abs(m.pos - goal) < 25 || (stoppedSince && millis() - stoppedSince > 100);
  static uint32_t holdStart = 0;
  if ((reached && elapsed > 150) || elapsed > 3000) {
    if (!holdStart) holdStart = millis();
    if (millis() - holdStart < 300) return;  // short hold at the end point, still logging
    holdStart = 0;
    Serial.printf("S,%c,%lu,%d,%.1f,%.0f,%.1f,%d\n", swing.side > 0 ? '+' : '-', (unsigned long)elapsed,
                  m.pos - goal, swing.peakLoad / 10.0, swing.peakCurrent * 6.5, m.volts, m.temp);
    swing.side = -swing.side;
    swingGo();
  }
}

// ---------------------------------------------------------------- bridge
// Host sends complete STS packets over USB; each is put on the bus and the
// servo's reply (raw bytes, echo removed) is sent back. Lets PC tools that
// expect a Feetech USB adapter talk through this ESP32.
bool bridgeOn = false;

void bridgeStep() {
  static uint8_t buf[80];
  static size_t n = 0;
  static uint32_t lastByte = 0;
  while (Serial.available() && n < sizeof(buf)) { buf[n++] = Serial.read(); lastByte = millis(); }
  if (n >= 3 && buf[0] == '~' && buf[1] == '~' && buf[2] == '~') { bridgeOn = false; n = 0; Serial.println("bridge off"); return; }
  while (n && buf[0] != 0xFF && buf[0] != '~') { memmove(buf, buf + 1, --n); }  // resync
  if (n && millis() - lastByte > 50) n = 0;                                    // stale partial packet
  if (n < 4 || buf[1] != 0xFF) return;
  size_t len = buf[3] + 4;
  if (len > sizeof(buf)) { n = 0; return; }
  if (n < len) return;
  uint8_t id = buf[2];
  if (busSend(buf, len) && id != BROADCAST) {
    // Forward the reply as raw bytes until a whole packet or 3 ms of silence.
    uint8_t r[64]; size_t k = 0; uint32_t t0 = micros();
    while (micros() - t0 < 3000 && k < sizeof(r)) {
      if (!Serial1.available()) continue;
      r[k++] = Serial1.read(); t0 = micros();
      if (k >= 4 && r[0] == 0xFF && r[1] == 0xFF && k == (size_t)r[3] + 4) break;
    }
    if (k) Serial.write(r, k);
  }
  memmove(buf, buf + len, n - len); n -= len;
}

// ---------------------------------------------------------------- commands
void scan() {
  int found = 0;
  for (int id = 0; id <= 30; id++) {
    int r = ping(id);
    if (r >= 0) { Serial.printf("ID %d: servo found, position %d\n", id, readPosition(id)); found++; }
    else if (r == RX_CORRUPT) { Serial.printf("ID %d: garbled reply - probably TWO servos share this ID\n", id); found++; }
  }
  if (!found) Serial.println("no servos found");
}

void command(String line) {
  line.trim();
  int a = -1, b = INT32_MIN;  // INT32_MIN = not given
  char word[12] = {0};
  sscanf(line.c_str(), "%11s %d %d", word, &a, &b);
  String cmd(word);
  if (cmd == "scan") scan();
  else if (cmd == "stop" || cmd == "s") stopAll();
  else if (cmd == "id" && a >= 0 && b > 0 && b < 253) Serial.println(changeId(a, b) ? "ID changed" : "ID change failed");
  else if (cmd == "swing" && a > 0) swingStart(a, b > 0 ? b : 45);
  else if (cmd == "bridge") { stopAll(); bridgeOn = true; Serial.println("bridge on"); }
  else if (cmd == "between" && a > 0) {
    int c = INT32_MIN; sscanf(line.c_str(), "%*s %*d %*d %d", &c);
    if (b >= 0 && b < 4096 && c >= 0 && c < 4096) betweenStart(a, b, c); else Serial.println("between <id> <from 0..4095> <to 0..4095>");
  }
  else if (cmd == "lifthere" && a > 0) swingStart(a, b != INT32_MIN && b != 0 ? b : 90, true, false);
  else if (cmd == "lift" && a > 0) swingStart(a, b != INT32_MIN && b != 0 ? b : 90, true);
  else if (cmd == "demo") demoStart(a > 0 ? a : 1, b > 0 ? b : 2);
  else if (cmd == "cw" && (a == 1 || a == -1)) { cwSign = a == 1 ? 1 : -1; Serial.printf("clockwise sign %+d\n", cwSign); if (demo.on) demoApply(); }
  else if (line.length()) Serial.println("commands: scan | id <old> <new> | demo <a> <b> | swing <id> [deg] | lift <id> [deg] | lifthere <id> [deg] | between <id> <a> <b> | bridge | stop | cw <+1|-1>");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  busBegin();
  Serial.println("\nSTS3215 bus on GPIO21 ready");
  scan();
}

void loop() {
  if (bridgeOn) { bridgeStep(); return; }
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { command(line); line = ""; }
    else line += c;
  }
  if (demo.on) demoStep();
  if (swing.on) swingStep();
  else delay(5);
}
