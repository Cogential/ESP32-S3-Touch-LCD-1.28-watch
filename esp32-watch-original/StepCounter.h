// Simple accelerometer-magnitude pedometer.
//
// Idea: gravity always contributes ~9.8 m/s^2 to the magnitude of the
// accel vector no matter how the watch is oriented. Walking motion adds
// an oscillating component on top of that. We track a slow-moving
// baseline (exponential moving average) and count a step each time the
// instantaneous magnitude rises far enough above that baseline, with a
// minimum time between steps so noise/vibration can't multi-count.
#pragma once
#include <Arduino.h>

class StepCounter {
public:
  void begin(uint32_t startCount = 0) {
    _steps = startCount;
    _baseline = 9.8f;
    _above = false;
    _lastStepMs = 0;
  }

  // Call this often (e.g. every ~20ms) with the latest accel reading in m/s^2.
  void update(float ax, float ay, float az) {
    float mag = sqrtf(ax * ax + ay * ay + az * az);

    // Slow baseline tracks "gravity + current posture".
    _baseline += (mag - _baseline) * 0.02f;

    float dev = mag - _baseline;
    uint32_t now = millis();

    if (!_above && dev > kRiseThreshold) {
      _above = true;
      if (now - _lastStepMs > kMinStepIntervalMs) {
        _steps++;
        _lastStepMs = now;
      }
    } else if (_above && dev < kFallThreshold) {
      _above = false;
    }
  }

  uint32_t count() const { return _steps; }
  void set(uint32_t v) { _steps = v; }

private:
  // Tuned to shrug off a wrist flick/tap, not just a slow walking swing —
  // wrist-worn accel-only step counting is inherently a rough heuristic
  // without gyro/frequency analysis, so this is a tradeoff, not a perfect
  // fix: raising these further would miss more real steps too.
  static constexpr float kRiseThreshold = 3.5f;   // m/s^2 above baseline to arm a step
  static constexpr float kFallThreshold = 1.2f;   // must dip back down before next step counts
  static constexpr uint32_t kMinStepIntervalMs = 300; // debounce: caps ~200 steps/min

  uint32_t _steps = 0;
  float _baseline = 9.8f;
  bool _above = false;
  uint32_t _lastStepMs = 0;
};
