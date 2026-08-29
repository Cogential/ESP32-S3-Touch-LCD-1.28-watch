// Minimal CST816S capacitive touch reader over Wire I2C.
// Shares the I2C bus (SDA=6/SCL=7) with the QMI8658 IMU. Only reads the
// "any touch right now" state — plenty for tap-to-switch-screen navigation.
#pragma once
#include <Wire.h>

class TouchCST816S {
public:
  bool begin(TwoWire &wire, uint8_t rstPin, uint8_t intPin, uint8_t addr = 0x15) {
    _wire = &wire;
    _addr = addr;
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, LOW);
    delay(10);
    digitalWrite(rstPin, HIGH);
    delay(100); // CST816S wants some settle time post-reset before it'll ACK
    if (intPin != 0xFF) pinMode(intPin, INPUT);
    // Probe: reading the chip-id register should succeed if the device is present.
    uint8_t id;
    if (!readReg(0xA7, &id, 1)) return false;

    setAutoSleep(false);
    return true;
  }

  // 0xFE is DisAutoSleep: 1 forbids auto-sleep, 0 permits it.
  //
  // Which setting is right depends entirely on how touches are being
  // detected. While the watch is awake it *polls* over I2C, and an
  // auto-slept controller answers those polls with "no finger" until its
  // own interrupt rouses it — so auto-sleep has to be off, or touches go
  // unnoticed. In ESP32 deep sleep nothing polls: the wake source is the
  // INT line, which the controller still asserts on touch from its
  // low-power state. So auto-sleep is exactly wrong awake and exactly
  // right asleep, and gets toggled at each transition.
  bool setAutoSleep(bool allow) { return writeReg(0xFE, allow ? 0x00 : 0x01); }

  // Returns true if a finger is currently down, and fills x/y (0..239).
  bool read(int16_t &x, int16_t &y) {
    uint8_t buf[6];
    if (!readReg(0x01, buf, 6)) return false; // 0x01=gesture,0x02=fingerNum,0x03..0x06=xy
    uint8_t fingerNum = buf[1];
    if (fingerNum == 0) return false;
    x = ((buf[2] & 0x0F) << 8) | buf[3];
    y = ((buf[4] & 0x0F) << 8) | buf[5];
    return true;
  }

private:
  TwoWire *_wire = nullptr;
  uint8_t _addr = 0x15;

  bool writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint8_t *out, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((int)_addr, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) out[i] = _wire->read();
    return true;
  }
};
