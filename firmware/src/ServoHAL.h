// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#pragma once
// ============================================================
// ServoHAL —— 舵机硬件抽象层
// ESP32-S3 LEDC 直驱 6 个 SG90（替代原版两台 ATOMIC Motion Base）。
// 物理通道 0..5 完全独立：
//   0..2 = 左侧（与原版左机 S1/S2/S3 等价）
//   3..5 = 右侧（与原版右机 S1/S2/S3 等价）
// ============================================================

#include <Arduino.h>

class ServoHAL {
public:
  // 初始化 6 路 LEDC（50Hz/14bit）。返回 false 表示某路申请失败。
  bool init();

  // 物理通道 0..5 设角度（度）。内部做软限位钳位；
  // 该通道若在 SERVO_INVERT 中配置了反相，则输出 180°-angle 的脉宽。
  void setAngle(int channel, float angleDeg);

  // 直接写脉宽（µs），绕过角度换算（测试/校准用）。
  void setPulseUs(int channel, uint16_t pulseUs);

  // 全部回中 90°（1500µs）。
  void centerAll();

  // 上电自检：验证 6 路 LEDC 均已挂载
  bool selfTest();

private:
  bool _ready = false;
  uint16_t _pulseForAngle(float angleDeg) const;
  void _writePin(int pin, uint16_t pulseUs);
};
