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

    // Disable auto-sleep (0xFE = DisAutoSleep, 1 = off). Left enabled, the
    // controller powers down its sensing after a stretch of no touch and
    // then reports "no finger" to I2C polls until its own interrupt wakes
    // it — so polling alone silently stops seeing touches, which is
    // exactly the state the watch sits in once the backlight has dimmed.
    writeReg(0xFE, 0x01);
    return true;
  }

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
