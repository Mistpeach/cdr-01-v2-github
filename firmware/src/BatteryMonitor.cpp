// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#include "BatteryMonitor.h"
#include "config.h"

void BatteryMonitor::init() {
  _source = BATTERY_SOURCE;
  if (_source == 1) {
    // 分压测量：adcAttenuation 用 11dB（满量程 ~3.3V 附近，量程 0~3100mV）
    analogReadResolution(12);
    pinMode(BATTERY_ADC_PIN, INPUT);
  }
}

uint8_t BatteryMonitor::levelFromVoltage(float voltage) const {
  // 与原版一致的线性换算：0% = 3.17V，100% = 4.17V
  if (voltage <= BATTERY_EMPTY_VOLTAGE) return 0;
  if (voltage >= BATTERY_FULL_VOLTAGE) return 100;
  float level = (voltage - BATTERY_EMPTY_VOLTAGE) * 100.0f / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE);
  return (uint8_t)(level + 0.5f);
}

BatteryReading BatteryMonitor::read() {
  BatteryReading reading;
  reading.current = BATTERY_FIXED_CURRENT;

  if (_source == 1) {
    // 多采样平均抑制噪声
    uint32_t sum = 0;
    const int samples = 32;
    for (int i = 0; i < samples; i++) sum += analogRead(BATTERY_ADC_PIN);
    float adc = (float)sum / samples;
    float voltage = adc / 4095.0f * BATTERY_ADC_VREF * BATTERY_ADC_DIVIDER;
    reading.voltage = voltage;
    reading.level = levelFromVoltage(voltage);
  } else {
    reading.voltage = BATTERY_FIXED_VOLTAGE;
    reading.level = BATTERY_FIXED_LEVEL;
  }

  if (reading.level > 100) reading.level = 100;
  return reading;
}
