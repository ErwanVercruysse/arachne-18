// ESP32-C3 Super Mini -> Feetech STS3215 over ONE wire (half-duplex UART).
//
// Wiring: GPIO21 --[560 ohm]-- servo signal, shared GND, servo on 12 V.
// UART1 TX and RX are both routed to GPIO21 through the GPIO matrix.
// Every byte we send is also received back (echo); it is read and discarded
// before the servo's reply is parsed, so our own transmission is ignored.
//
// USB serial monitor (115200): prints status. Commands:
//   s = stop (torque off)   g = go   r = reverse   + / - = faster / slower
//
// The servo runs in wheel mode (continuous 360 deg rotation, no endpoint).
// The mode is written to RAM only (EEPROM stays locked): after a power cycle
// the servo is back in normal position mode.

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_reg.h"

constexpr int BUS_PIN = 21;
constexpr uint32_t BAUD = 1000000;  // STS3215 factory default
constexpr uint8_t SERVO_ID = 1;

// STS3215 registers
constexpr uint8_t REG_MODE = 33, REG_TORQUE = 40, REG_ACCEL = 41, REG_GOAL = 42, REG_SPEED = 46, REG_POSITION = 56;

int direction = +1;  // +1: position counts up. Flip with 'r' if it turns the wrong way.
int speed = 3000;    // steps/s; 4096 steps = 1 turn. 3000 = ~264 deg/s, near the 12 V maximum
bool running = true;

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

// Read a status packet: FF FF ID LEN ERR PARAMS... CHK. Returns param count or -1.
int busReceive(uint8_t *params, size_t maxParams, uint32_t timeoutUs = 3000) {
  uint8_t buf[64];
  size_t n = 0;
  uint32_t t0 = micros();
  while (micros() - t0 < timeoutUs && n < sizeof(buf)) {
    if (!Serial1.available()) continue;
    buf[n++] = Serial1.read();
    if (n == 1 && buf[0] != 0xFF) n = 0;               // resync on header
    if (n == 2 && buf[1] != 0xFF) n = 0;
    if (n >= 4 && n == (size_t)buf[3] + 4) {           // complete packet
      uint8_t sum = 0;
      for (size_t i = 2; i < n - 1; i++) sum += buf[i];
      if ((uint8_t)~sum != buf[n - 1]) return -1;
      size_t count = buf[3] - 2;
      if (count > maxParams) count = maxParams;
      memcpy(params, buf + 5, count);
      return count;
    }
  }
  return -1;
}

void packet(uint8_t instr, const uint8_t *params, uint8_t count, uint8_t *out, size_t &len) {
  out[0] = out[1] = 0xFF; out[2] = SERVO_ID; out[3] = count + 2; out[4] = instr;
  uint8_t sum = SERVO_ID + count + 2 + instr;
  for (uint8_t i = 0; i < count; i++) { out[5 + i] = params[i]; sum += params[i]; }
  out[5 + count] = ~sum;
  len = 6 + count;
}

bool writeReg(uint8_t reg, const uint8_t *data, uint8_t count) {
  uint8_t params[16], pkt[24]; size_t len;
  params[0] = reg; memcpy(params + 1, data, count);
  packet(0x03, params, count + 1, pkt, len);
  if (!busSend(pkt, len)) return false;
  uint8_t reply[4];
  return busReceive(reply, sizeof(reply)) >= 0;
}

int readPosition() {
  uint8_t params[2] = {REG_POSITION, 2}, pkt[16], reply[4]; size_t len;
  packet(0x02, params, 2, pkt, len);
  if (!busSend(pkt, len)) return -2;  // echo problem
  if (busReceive(reply, sizeof(reply)) != 2) return -1;  // no answer from servo
  int v = reply[0] | (reply[1] << 8);
  return (v & 0x8000) ? -(v & 0x7FFF) : v;
}

bool ping() {
  uint8_t pkt[8]; size_t len; uint8_t reply[4];
  packet(0x01, nullptr, 0, pkt, len);
  return busSend(pkt, len) && busReceive(reply, sizeof(reply)) >= 0;
}

// Wheel-mode speed: magnitude in steps/s, bit 15 = reverse.
bool setSpeed(int stepsPerSecond) {
  uint16_t v = abs(stepsPerSecond);
  if (stepsPerSecond < 0) v |= 0x8000;
  uint8_t d[2] = {(uint8_t)v, (uint8_t)(v >> 8)};
  return writeReg(REG_SPEED, d, 2);
}

bool wheelMode(bool on) { uint8_t m = on ? 1 : 0; return writeReg(REG_MODE, &m, 1); }

bool torque(bool on) { uint8_t v = on; return writeReg(REG_TORQUE, &v, 1); }

// ---------------------------------------------------------------- program
bool start() {
  bool ok = wheelMode(true);
  uint8_t accel = 20; ok &= writeReg(REG_ACCEL, &accel, 1);
  ok &= torque(true) && setSpeed(direction * speed);
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nSTS3215 one-wire continuous rotation on GPIO21");
  busBegin();
  while (!ping()) {
    int p = readPosition();
    Serial.println(p == -2 ? "Echo wrong/missing: check GPIO21, 560R and GND"
                           : "Servo does not answer: 12 V on? ID 1? 1 Mbaud?");
    delay(1000);
  }
  Serial.println(start() ? "Servo found, spinning" : "Servo found, but a setup write failed");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') { running = false; setSpeed(0); torque(false); Serial.println("stopped, torque off"); }
    if (c == 'g') { running = true; start(); Serial.println("go"); }
    if (c == 'r') { direction = -direction; if (running) setSpeed(direction * speed); Serial.printf("direction %+d\n", direction); }
    if (c == '+' || c == '-') {
      speed = constrain(speed + (c == '+' ? 250 : -250), 250, 3400);
      if (running) setSpeed(direction * speed);
      Serial.printf("speed %d steps/s (%.0f deg/s)\n", speed, speed * 360.0 / 4096);
    }
  }
  // Unwrap the 0..4095 encoder so full turns are counted.
  static int last = -1;
  static long total = 0;
  static uint32_t lastPrint = 0;
  int pos = readPosition();
  if (pos >= 0) {
    if (last >= 0) {
      int d = pos - last;
      if (d > 2048) d -= 4096;
      if (d < -2048) d += 4096;
      total += d;
    }
    last = pos;
  }
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("position %4d   turns %+.2f\n", pos, total / 4096.0);
  }
  delay(10);
}
