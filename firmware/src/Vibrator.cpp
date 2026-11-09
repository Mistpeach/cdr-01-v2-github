// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#include "Vibrator.h"
#include "config.h"

void Vibrator::init() {
#if VIBRATOR_PIN >= 0
  _pin = VIBRATOR_PIN;
  pinMode(_pin, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(_pin, VIBRATOR_PWM_FREQ, VIBRATOR_PWM_RESOLUTION);
  ledcWrite(_pin, 0);
#else
  ledcSetup(6, VIBRATOR_PWM_FREQ, VIBRATOR_PWM_RESOLUTION);
  ledcAttachPin(_pin, 6);
  ledcWrite(6, 0);
#endif
  Serial.printf("[SELF TEST] Vibrator PWM ready. pin=%d freq=%d\n", _pin, VIBRATOR_PWM_FREQ);
#endif
}

void Vibrator::_apply() {
  if (_pin < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(_pin, _duty);
#else
  ledcWrite(6, _duty);
#endif
}

void Vibrator::setPercent(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  // 与原版一致：percent 0-100 → duty 0-255（8bit）
  _duty = map(percent, 0, 100, 0, 255);
  _apply();
}
