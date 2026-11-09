// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#pragma once
// ============================================================
// BatteryMonitor —— 电量监测抽象层
// 原版经 ATOMIC Motion Base 的 INA226 读取电压/电流；
// 本板无该芯片，提供两种来源（config.h 的 BATTERY_SOURCE）：
//   0 = 固定值（USB/外接电源场景）
//   1 = ADC 分压测量电压（电量线性换算与原版公式一致）
// ============================================================

#include <Arduino.h>

struct BatteryReading {
  float voltage;
  float current;
  uint8_t level;
};

class BatteryMonitor {
public:
  void init();
  BatteryReading read();
  uint8_t levelFromVoltage(float voltage) const;

private:
  int _source = 0;
};
