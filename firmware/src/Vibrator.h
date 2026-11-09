// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#pragma once
// ============================================================
// Vibrator —— 振动电机抽象层（原版 GPIO8，PWM 1kHz/8bit）
// ============================================================

#include <Arduino.h>

class Vibrator {
public:
  void init();
  // 设置百分比 0-100（内部钳位）
  void setPercent(int percent);
  void stop() { setPercent(0); }
  bool enabled() const { return _pin >= 0; }

private:
  int _pin = -1;
  int _duty = 0;
  void _apply();
};
