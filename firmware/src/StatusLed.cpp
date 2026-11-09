// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#include "StatusLed.h"
#include "config.h"

void StatusLed::init() {
  _type = LED_TYPE;
  _activeHigh = LED_GPIO_ACTIVE_HIGH;
  _brightness = LED_MAX_BRIGHTNESS;

  if (_type == 1) {
    _pin = LED_PIN;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, _activeHigh ? LOW : HIGH);  // 初始熄灭
  } else if (_type == 2) {
    _pin = LED_WS2812_PIN;
    // neopixelWrite 内部走 RMT，无需初始化
  }
  // _type == 0：关闭
}

void StatusLed::setBrightness(uint8_t brightness) {
  _brightness = brightness;
}

void StatusLed::showColor(uint8_t r, uint8_t g, uint8_t b) {
  if (_type == 0) return;

  if (_type == 1) {
    // 普通 LED：黑 = 灭；任何可见颜色（且亮度>0）= 亮
    bool on = _brightness > 0 && (r > 0 || g > 0 || b > 0);
    digitalWrite(_pin, on == _activeHigh ? HIGH : LOW);
    return;
  }

  // WS2812：按亮度缩放后经 RMT 输出（GRB 线序由 neopixelWrite 处理）
  uint8_t scale = _brightness;
  uint32_t rr = (uint32_t)r * scale / 255;
  uint32_t gg = (uint32_t)g * scale / 255;
  uint32_t bb = (uint32_t)b * scale / 255;
  neopixelWrite((uint8_t)_pin, (uint8_t)rr, (uint8_t)gg, (uint8_t)bb);
}
