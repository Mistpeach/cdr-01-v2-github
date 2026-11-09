// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#pragma once
// ============================================================
// StatusLed —— 状态灯抽象层
// 原版用 WS2811 @ GPIO35（N16R8 上被八线 PSRAM 占用，必须换脚）。
// 支持三种模式（config.h 的 LED_TYPE）：
//   0 = 关闭（LED_STATUS/LED_TEST 命令仍正常响应）
//   1 = 普通 GPIO LED（非黑即亮，闪烁节奏由上层状态机控制）
//   2 = WS2812 灯珠（neopixelWrite 内置驱动，无需 FastLED）
// ============================================================

#include <Arduino.h>

class StatusLed {
public:
  void init();
  // 立即显示颜色。普通 GPIO 模式：非黑且亮度>0 → 点亮；黑 → 熄灭。
  void showColor(uint8_t r, uint8_t g, uint8_t b);
  void setBrightness(uint8_t brightness);   // 0-255
  uint8_t brightness() const { return _brightness; }

private:
  int _type = 0;
  uint8_t _pin = 0;
  uint8_t _brightness = 255;
  bool _activeHigh = true;
};
