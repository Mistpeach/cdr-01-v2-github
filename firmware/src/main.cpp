// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

// ============================================================
// CDR01 移植固件 —— ESP32-S3 N16R8 + 6×SG90 直驱
// 入口：初始化并委托给 RobotController。
// 协议/行为与原版 0721robotics/cdr01-firmware 一致，
// 细节见 include/config.h 与 docs。
// ============================================================

#include <Arduino.h>
#include "RobotController.h"

RobotController robot;

void setup() {
  robot.setup();
}

void loop() {
  robot.loop();
}
