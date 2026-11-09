// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#include "ServoHAL.h"
#include "config.h"

// ============================================================
// LEDC 直驱实现（6 个独立物理通道）
//  - 频率 50Hz（20ms 周期）、14bit 分辨率（ESP32-S3 LEDC 上限，1.22µs/步）
//  - duty = pulse_us * 16384 / 20000
//  - Arduino core 3.x: ledcAttach(pin, freq, res) 自动分配通道
//    Arduino core 2.x: ledcSetup/ledcAttachPin（保留兼容分支）
// ============================================================

namespace {
const int gpioPins[SERVO_COUNT] = SERVO_GPIO;          // L0 L1 L2 | R0 R1 R2
const bool invert[SERVO_COUNT] = SERVO_INVERT;
const uint32_t PERIOD_US = 1000000UL / SERVO_PWM_FREQ;       // 20000µs
const uint32_t DUTY_MAX = (1UL << SERVO_PWM_RESOLUTION) - 1; // 65535
}  // namespace

bool ServoHAL::init() {
  _ready = true;

  for (int ch = 0; ch < SERVO_COUNT; ch++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttach(gpioPins[ch], SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION)) {
      log_e("LEDC attach failed: GPIO%d (channel %d)", gpioPins[ch], ch);
      _ready = false;
    }
#else
    // core 2.x：手动分配通道（舵机用 0..5，振动电机用 6）
    if (ledcSetup(ch, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION) == 0) { _ready = false; }
    ledcAttachPin(gpioPins[ch], ch);
#endif
  }

  if (_ready) centerAll();
  return _ready;
}

bool ServoHAL::selfTest() {
  return _ready;
}

void ServoHAL::_writePin(int pin, uint16_t pulseUs) {
  uint32_t duty = (uint32_t)pulseUs * (DUTY_MAX + 1) / PERIOD_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  // core 2.x：由 init 时固定映射 pin->channel，这里反查
  for (int ch = 0; ch < SERVO_COUNT; ch++) {
    if (gpioPins[ch] == pin) { ledcWrite(ch, duty); return; }
  }
#endif
}

uint16_t ServoHAL::_pulseForAngle(float angleDeg) const {
  // 原版脉宽公式：pulse = PULSE_MIN + angle/180 * (PULSE_MAX-PULSE_MIN)
  float pulse = SERVO_PULSE_MIN_US + (angleDeg / 180.0f) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
  if (pulse < SERVO_PULSE_MIN_US) pulse = SERVO_PULSE_MIN_US;
  if (pulse > SERVO_PULSE_MAX_US) pulse = SERVO_PULSE_MAX_US;
  return (uint16_t)(pulse + 0.5f);
}

void ServoHAL::setAngle(int channel, float angleDeg) {
  if (channel < 0 || channel >= SERVO_COUNT) return;

  // 1) 软限位钳位
  if (angleDeg < SERVO_ANGLE_MIN) angleDeg = SERVO_ANGLE_MIN;
  if (angleDeg > SERVO_ANGLE_MAX) angleDeg = SERVO_ANGLE_MAX;
  // 2) 该通道反相
  if (invert[channel]) angleDeg = SERVO_ANGLE_MIN + SERVO_ANGLE_MAX - angleDeg;

  _writePin(gpioPins[channel], _pulseForAngle(angleDeg));
}

void ServoHAL::setPulseUs(int channel, uint16_t pulseUs) {
  if (channel < 0 || channel >= SERVO_COUNT) return;
  if (pulseUs < SERVO_PULSE_MIN_US) pulseUs = SERVO_PULSE_MIN_US;
  if (pulseUs > SERVO_PULSE_MAX_US) pulseUs = SERVO_PULSE_MAX_US;
  _writePin(gpioPins[channel], pulseUs);
}

void ServoHAL::centerAll() {
  for (int ch = 0; ch < SERVO_COUNT; ch++) setAngle(ch, 90.0f);
}
