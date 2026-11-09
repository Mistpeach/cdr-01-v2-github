// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach
//
// Derived from cdr01-firmware DeltaRobotController.cpp/.h by 0721Robotics (Apache-2.0):
//   https://github.com/0721robotics/cdr01-firmware
// Changes (Apache-2.0 4(b)): BLE transport replaced by the V2 framed protocol;
// Wi-Fi/UDP and the legacy text protocol removed; dual-serial logging, per-side
// command routing and transport-decoupled command processing added.

#include "RobotController.h"

#include <ctype.h>
#include <string.h>

// ============================================================
// RobotController —— CDR01 v2 全局层
// 运动引擎在 DeltaRobot.cpp（左右各一实例）；通讯层为 V2Transport/V2Protocol。
// 已移除 WiFi/UDP/mDNS 与旧版 CDR01 BLE 协议。
// ============================================================

// ============================================================
// 启动 / 主循环
// ============================================================

void RobotController::printBoth(const String& line) {
  Serial.println(line);
  Serial0.println(line);   // UART0（GPIO43/44）镜像：部分 N16R8 板此口接 USB 串口桥
}

void RobotController::setup() {
  Serial.begin(SERIAL_BAUD);
  Serial0.begin(SERIAL_BAUD);   // 双口日志兜底（见 printBoth）

  printBoth(String("[BOOT] CDR01v2 ") + CDR01_FIRMWARE_VERSION + " starting...");

  // ---- 上电自检 1：舵机 LEDC ----
  if (!servo.init()) {
    printBoth("[SELF TEST] Servo LEDC init FAILED (check GPIO conflicts)");
  } else {
    printBoth("[SELF TEST] Servo LEDC ready: 6ch @50Hz/14bit (left 0-2, right 3-5)");
  }
  servo.centerAll();

  setupLed();
  vibrator.init();

#if BUTTON_PIN >= 0
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.printf("[SELF TEST] Button ready. pin=%d\n", BUTTON_PIN);
#endif

  // ---- 左右两个 delta 实例（物理通道偏移：左 0，右 3）----
  auto responder = [this](const String& msg) { sendResponse(msg); };
  auto vibratorSetter = [this](int percent, bool report) {
    vibrator.setPercent(percent);
    if (report) Serial.printf("Vibrator percent: %d%%\n", percent);
  };
  left.init(&servo, 0, "L", responder, vibratorSetter);
  right.init(&servo, 3, "R", responder, vibratorSetter);

  // ---- 上电自检 2：双侧回中（原版启动即 JOINT 90 90 90 200）----
  processCommand("JOINT 90 90 90 200", SIDE_BOTH);

#if SELF_TEST_SWING
  printBoth("[SELF TEST] Swing check...");
  processCommand("JOINT 100 100 100 300", SIDE_BOTH);
  delay(350);
  processCommand("JOINT 80 80 80 300", SIDE_BOTH);
  delay(350);
  processCommand("JOINT 90 90 90 300", SIDE_BOTH);
#endif

  // ---- 默认 REPEAT_RANDOM 命令（与原版一致，两侧各自注册）----
  left.addDefaultRepeatCommand("REPEAT_POINT 0 0 -50 0 0 50 1000");
  left.addDefaultRepeatCommand("REPEAT_POINT 80 0 0 -80 0 0 1000");
  left.addDefaultRepeatCommand("REPEAT_POINT 0 80 0 0 -80 0 1000");
  left.addDefaultRepeatCommand("REPEAT_CIRCLE 0 0 0 75 2000");
  right.addDefaultRepeatCommand("REPEAT_POINT 0 0 -50 0 0 50 1000");
  right.addDefaultRepeatCommand("REPEAT_POINT 80 0 0 -80 0 0 1000");
  right.addDefaultRepeatCommand("REPEAT_POINT 0 80 0 0 -80 0 1000");
  right.addDefaultRepeatCommand("REPEAT_CIRCLE 0 0 0 75 2000");

  setupTCodeSettings();
  setupBLE();

  printBoth(String("[SELF TEST] Firmware: ") + firmwareVersionValue());
  printBoth(String("[SELF TEST] Servo channels: ") + SERVO_COUNT
            + " (pulse " + SERVO_PULSE_MIN_US + ".." + SERVO_PULSE_MAX_US + " us)");
  printBoth("Setup complete");
}

void RobotController::loop() {
  // BLE 写入数据在这里解析/执行（onWrite 只入队，避免在 BLE 回调里 notify 死锁）
  transport.poll();

  // 10ms 节拍
  unsigned long now = millis();
  if (now - lastTickTime >= MOTION_TICK_MS) {
    lastTickTime = now;
    left.executeStep();
    right.executeStep();
  }

  if (Serial.available()) {
    commandBuffer = Serial.readStringUntil('\n');
    String received = commandBuffer;
    received.trim();
    if (!looksLikeTCodeCommand(received.c_str())) printBoth(commandBuffer);
    pendingSide = SIDE_BOTH;   // 串口命令不带侧属性（可用 L:/R: 前缀）
    newCommandAvailable = true;
  } else if (Serial0.available()) {
    // UART0 也接受命令（部分 N16R8 板的 USB 串口走 UART0 桥）
    commandBuffer = Serial0.readStringUntil('\n');
    String received = commandBuffer;
    received.trim();
    if (!looksLikeTCodeCommand(received.c_str())) printBoth(commandBuffer);
    pendingSide = SIDE_BOTH;
    newCommandAvailable = true;
  }

  if (newCommandAvailable) {
    processCommand(commandBuffer, pendingSide);
    newCommandAvailable = false;
  }

  left.tick();
  right.tick();
  transport.tick(now, isBleConnected());

  updateBatteryLevel(false);
  updateBatteryMonitoring(false);
  checkTCodeTimeout();

  handleButton();
  updateLed();
}

// ============================================================
// 命令处理与路由
// ============================================================

DeltaRobot* RobotController::sideRobot(int side) {
  return side == SIDE_LEFT ? &left : &right;
}

void RobotController::processCommand(const String& command, int forceSide) {
  String trimmed = command;
  trimmed.trim();
  if (trimmed.length() == 0) return;

  if (handleVersionCommand(trimmed)) return;
  if (handleLedCommand(trimmed)) return;
  if (handleTCodeConfigCommand(trimmed)) return;

  ParsedTCodeCommand tcodeCommand;
  if (looksLikeTCodeCommand(trimmed.c_str())) {
    if (!parseTCodeCommand(trimmed.c_str(), tcodeCommand)) {
      sendError("INVALID_TCODE", "Invalid T-Code command");
      return;
    }

    if (tcodeDebugEnabled) {
      Serial.print("TCODE ");
      Serial.println(tcodeDebugSummary(tcodeCommand));
    }
    handleTCodeCommand(tcodeCommand);
    return;
  }

  // ---- 侧前缀扩展（串口调试用）：L:xxx / R:xxx ----
  int side = forceSide;
  String body = trimmed;
  if (body.length() >= 3 && body[1] == ':' &&
      (body[0] == 'L' || body[0] == 'l' || body[0] == 'R' || body[0] == 'r')) {
    side = (toupper((unsigned char)body[0]) == 'L') ? SIDE_LEFT : SIDE_RIGHT;
    body = body.substring(2);
    body.trim();
  }

  ParsedCommand parsed;
  bool parsedOk = parseCommand(body.c_str(), parsed);

  // 新命令抢占当前动作：只清除"被命令目标侧"的动作，
  // 否则 CSV 里左列命令会打断右侧正在执行的 REPEAT_*（左右独立控制的前提）。
  if (parsed.type != CommandType::Battery) {
    if (side == SIDE_LEFT) {
      left.stopAll();
    } else if (side == SIDE_RIGHT) {
      right.stopAll();
    } else {
      left.stopAll();
      right.stopAll();
    }
  }

  printBoth("Received Command: " + trimmed);

  if (!parsedOk) {
    String message = "Invalid ";
    message += commandTypeName(parsed.type);
    message += " command";
    sendError("INVALID_COMMAND", message);
    return;
  }

  switch (parsed.type) {
    case CommandType::Battery:
      handleBatteryCommand();
      return;

    case CommandType::Stop:
      if (side == SIDE_BOTH) {
        left.stopAll();
        right.stopAll();
        setLedBaseMode(LedBaseMode::Idle);
        sendOk("STOP");
      } else {
        sideRobot(side)->stopAll();
        sendOk(String("[") + (side == SIDE_LEFT ? "L" : "R") + "] STOP");
      }
      return;

    case CommandType::Vibrate:
      vibrator.setPercent(parsed.intValue);
      sendOk("VIBRATE set");
      return;

    default:
      break;
  }

  routeMotionCommand(parsed, side);
}

void RobotController::routeMotionCommand(const ParsedCommand& parsed, int side) {
  switch (side) {
    case SIDE_LEFT:
      left.executeCommand(parsed);
      break;
    case SIDE_RIGHT:
      right.executeCommand(parsed);
      break;
    default:
      left.executeCommand(parsed);
      right.executeCommand(parsed);
      break;
  }
}

void RobotController::handleTCodeCommand(const ParsedTCodeCommand& command) {
  unsigned long now = millis();
  lastTCodeReceiveTime = now;
  lastBleActivityTime = now;

  if (command.stop) {
    left.tcodeStop();
    right.tcodeStop();
    vibrator.setPercent(0);
    lastTCodeReceiveTime = 0;
    sendTCodeDeviceResponse("OK DSTOP");
    return;
  }

  if (command.deviceCommand >= 0) {
    switch (command.deviceCommand) {
      case 0:
        sendTCodeDeviceResponse(firmwareVersionValue());
        break;
      case 1:
        sendTCodeDeviceResponse("TCode v0.3");
        break;
      case 2:
        sendTCodeDeviceResponse(tcodeCapabilitiesValue());
        break;
      default:
        sendError("UNSUPPORTED_TCODE_DEVICE_COMMAND", "Unsupported T-Code device command");
        break;
    }
    return;
  }

  if (command.saveCommand && !command.hasLinear && !command.hasVibrate) {
    sendTCodeDeviceResponse("OK");
    return;
  }

  if (command.hasVibrate) {
    float vibrateMagnitude = 0.0f;
    for (int i = 0; i < 10; i++) {
      if (command.vibrate[i].active && command.vibrate[i].magnitude > vibrateMagnitude) {
        vibrateMagnitude = command.vibrate[i].magnitude;
      }
    }
    vibrator.setPercent((int)(vibrateMagnitude * 100.0f + 0.5f));
  }

  left.applyTCodeAxes(command);
  right.applyTCodeAxes(command);
}

bool RobotController::handleVersionCommand(const String& command) {
  if (command != "VERSION") return false;
  sendValue("version", firmwareVersionValue());
  return true;
}

// ============================================================
// T-Code 轴槽位配置（6 槽位：0-2 左，3-5 右）
// ============================================================

bool RobotController::handleTCodeConfigCommand(const String& command) {
  if (command == "TCODE_STATUS") {
    sendValue("tcode", tcodeStatusValue());
    return true;
  }

  if (command == "TCODE_SAVE") {
    saveTCodeAxisSettings();
    sendOk("TCODE settings saved");
    return true;
  }

  if (command == "TCODE_RESET") {
    clearSavedTCodeAxisSettings();
    resetTCodeAxisSettings();
    sendOk("TCODE settings reset");
    return true;
  }

  if (command == "TCODE_DEBUG ON") {
    tcodeDebugEnabled = true;
    sendOk("TCODE_DEBUG on");
    return true;
  }

  if (command == "TCODE_DEBUG OFF") {
    tcodeDebugEnabled = false;
    sendOk("TCODE_DEBUG off");
    return true;
  }

  if (!command.startsWith("TCODE_AXIS ")) return false;

  char firstText[4] = {0};
  char axisText[4] = {0};
  char coordText[2] = {0};
  int invert = 0;
  float minPercent = 0;
  float maxPercent = 0;

  int slot = -1;
  int count = sscanf(command.c_str(), "TCODE_AXIS %3s %3s %1s %d %f %f",
                     firstText, axisText, coordText, &invert, &minPercent, &maxPercent);
  if (count == 6 && strlen(firstText) == 1 && firstText[0] >= '0' && firstText[0] <= '5') {
    slot = firstText[0] - '0';
  } else {
    count = sscanf(command.c_str(), "TCODE_AXIS %3s %1s %d %f %f",
                   axisText, coordText, &invert, &minPercent, &maxPercent);
    if (count != 5) {
      sendError("INVALID_TCODE_AXIS", "Use TCODE_AXIS slot A0 X|Y|Z invert min_percent max_percent");
      return true;
    }

    char legacyType = 0;
    int legacyChannel = -1;
    if (parseTCodeAxisId(axisText, legacyType, legacyChannel) && legacyChannel >= 0 && legacyChannel <= 2) {
      slot = legacyChannel;
    }
  }

  int coord = -1;
  char coordChar = (char)toupper((unsigned char)coordText[0]);
  if (coordChar == 'X') coord = 0;
  if (coordChar == 'Y') coord = 1;
  if (coordChar == 'Z') coord = 2;

  char sourceType = 0;
  int sourceChannel = -1;
  bool validAxisId = parseTCodeAxisId(axisText, sourceType, sourceChannel);

  if (slot < 0 || slot > 5 || !validAxisId || coord < 0 ||
      minPercent < -100.0f || maxPercent > 100.0f || minPercent >= maxPercent) {
    sendError("INVALID_TCODE_AXIS", "Invalid TCODE_AXIS values");
    return true;
  }

  if (slot < 3) {
    left.setAxisSlot(slot, sourceType, sourceChannel, coord, invert != 0, minPercent, maxPercent);
  } else {
    right.setAxisSlot(slot - 3, sourceType, sourceChannel, coord, invert != 0, minPercent, maxPercent);
  }
  sendOk("TCODE_AXIS set");
  return true;
}

void RobotController::resetTCodeAxisSettings() {
  left.resetAxisSlots();
  right.resetAxisSlots();
}

void RobotController::setupTCodeSettings() {
  resetTCodeAxisSettings();
  loadTCodeAxisSettings();
}

void RobotController::saveTCodeAxisSettings() {
  tcodePreferences.begin("cdr01_tcode", false);
  for (int i = 0; i < 6; i++) {
    DeltaRobot* robot = i < 3 ? &left : &right;
    int local = i < 3 ? i : i - 3;
    char axisType;
    int channel, coord;
    bool invert;
    float minP, maxP;
    robot->getAxisSlot(local, axisType, channel, coord, invert, minP, maxP);

    String index = String(i);
    String label = String(axisType) + String(channel);
    tcodePreferences.putString(("axis" + index).c_str(), label);
    tcodePreferences.putInt(("coord" + index).c_str(), coord);
    tcodePreferences.putBool(("invert" + index).c_str(), invert);
    tcodePreferences.putFloat(("min" + index).c_str(), minP);
    tcodePreferences.putFloat(("max" + index).c_str(), maxP);
  }
  tcodePreferences.putBool("saved", true);
  tcodePreferences.end();
}

void RobotController::clearSavedTCodeAxisSettings() {
  tcodePreferences.begin("cdr01_tcode", false);
  tcodePreferences.clear();
  tcodePreferences.end();
}

bool RobotController::loadTCodeAxisSettings() {
  // 首次上电时命名空间不存在；用读写方式打开可自动创建，避免刷出
  // "nvs_open failed: NOT_FOUND" 这种吓人但无害的错误日志。
  tcodePreferences.begin("cdr01_tcode", false);
  bool saved = tcodePreferences.getBool("saved", false);
  if (saved) {
    for (int i = 0; i < 6; i++) {
      DeltaRobot* robot = i < 3 ? &left : &right;
      int local = i < 3 ? i : i - 3;
      String index = String(i);
      String savedAxis = tcodePreferences.getString(("axis" + index).c_str(), "");
      char axisType = 0;
      int channel = -1;
      if (!parseTCodeAxisId(savedAxis, axisType, channel)) continue;

      int coord = tcodePreferences.getInt(("coord" + index).c_str(), i == 0 || i == 3 ? 0 : (i == 1 || i == 4 ? 2 : 1));
      bool invert = tcodePreferences.getBool(("invert" + index).c_str(), false);
      float minP = tcodePreferences.getFloat(("min" + index).c_str(), -100.0f);
      float maxP = tcodePreferences.getFloat(("max" + index).c_str(), 100.0f);
      robot->setAxisSlot(local, axisType, channel, coord, invert, minP, maxP);
    }
  }
  tcodePreferences.end();
  return saved;
}

String RobotController::tcodeStatusValue() const {
  String status = "";
  for (int i = 0; i < 6; i++) {
    const DeltaRobot* robot = i < 3 ? &left : &right;
    int local = i < 3 ? i : i - 3;
    char axisType;
    int channel, coord;
    bool invert;
    float minP, maxP;
    robot->getAxisSlot(local, axisType, channel, coord, invert, minP, maxP);

    if (i > 0) status += " ";
    status += String(i);
    status += ":";
    status += String(axisType);
    status += String(channel);
    status += "=";
    status += robot->coordName(coord);
    status += invert ? ":inv:" : ":norm:";
    status += String(minP, 1);
    status += "..";
    status += String(maxP, 1);
  }
  return status;
}

String RobotController::tcodeCapabilitiesValue() const {
  String capabilities = "";
  for (int i = 0; i < 6; i++) {
    const DeltaRobot* robot = i < 3 ? &left : &right;
    if (i > 0) capabilities += " ";
    capabilities += robot->axisLabel(i < 3 ? i : i - 3);
    capabilities += "-0000-9999";
  }
  capabilities += " V0-0000-9999";
  return capabilities;
}

String RobotController::tcodeDebugSummary(const ParsedTCodeCommand& command) const {
  String summary = "";
  for (int i = 0; i < 6; i++) {
    const DeltaRobot* robot = i < 3 ? &left : &right;
    int local = i < 3 ? i : i - 3;
    char axisType;
    int channel, coord;
    bool invert;
    float minP, maxP;
    robot->getAxisSlot(local, axisType, channel, coord, invert, minP, maxP);
    if (channel < 0 || channel >= TCODE_CHANNEL_COUNT) continue;

    const TCodeAxisCommand* axisCommand = nullptr;
    for (int j = 0; j < command.axisCount; j++) {
      if (command.axes[j].axisType == axisType && command.axes[j].channel == channel) {
        axisCommand = &command.axes[j].command;
        break;
      }
    }
    if (axisCommand == nullptr || !axisCommand->active) continue;
    if (summary.length() > 0) summary += " ";
    summary += robot->axisLabel(local);
    summary += "=";
    summary += String(axisCommand->magnitude, 4);
    if (axisCommand->hasInterval) {
      summary += "I";
      summary += String(axisCommand->intervalMs);
    }
    if (axisCommand->hasSpeed) {
      summary += "S";
      summary += String(axisCommand->speedPerSecond, 4);
    }
  }

  for (int i = 0; i < 10; i++) {
    if (!command.vibrate[i].active) continue;
    if (summary.length() > 0) summary += " ";
    summary += "V";
    summary += String(i);
    summary += "=";
    summary += String(command.vibrate[i].magnitude, 4);
  }

  if (command.stop) {
    if (summary.length() > 0) summary += " ";
    summary += "DSTOP";
  }

  if (summary.length() == 0) summary = "device";
  return summary;
}

bool RobotController::parseTCodeAxisId(const String& value, char& axisType, int& channel) const {
  String axis = value;
  axis.trim();
  if (axis.length() != 2) return false;

  char type = (char)toupper((unsigned char)axis[0]);
  char number = axis[1];
  if (type < 'A' || type > 'Z' || number < '0' || number > '9') return false;

  axisType = type;
  channel = number - '0';
  return true;
}

void RobotController::handleBatteryCommand() {
  BatteryReading battery = readBattery();
  lastBatteryReading = battery;
  lastBatteryReadTime = millis();
  setBatteryLevelValue(battery.level, true);

  String value = "voltage=";
  value += String(battery.voltage);
  value += ",current=";
  value += String(battery.current);
  value += ",level=";
  value += String(battery.level);
  sendValue("battery", value);
}

// ============================================================
// LED（与原版逻辑一致，移除 UDP 分支）
// ============================================================

void RobotController::setupLed() {
  led.init();
  showImmediateLed(RGBColors::White);
}

void RobotController::updateLed() {
  unsigned long now = millis();
  BatteryLedState batteryState = batteryLedState();

  if (ledTestActive && now - ledTestStartTime >= ledTestDuration) ledTestActive = false;
  if (ledNotice != LedNotice::None && now >= ledNoticeUntilTime) ledNotice = LedNotice::None;

  bool forceVisible = batteryState == BatteryLedState::Critical || ledTestActive;
  if (ledModeSetting == LedModeSetting::Off && !forceVisible) {
    led.setBrightness(0);
    showImmediateLed(RGBColors::Black);
    return;
  }

  int brightnessPercent = ledBrightnessPercent;
  if (ledModeSetting == LedModeSetting::Dim && brightnessPercent > ledDimPercent) {
    brightnessPercent = ledDimPercent;
  }
  uint8_t brightness = (uint8_t)((ledMaxBrightness * brightnessPercent + 50) / 100);
  led.setBrightness(brightness);

  RGB color = RGBColors::Black;

  if (batteryState == BatteryLedState::Critical) {
    color = (now / 180) % 2 == 0 ? RGBColors::Red : RGBColors::Black;
  } else if (ledTestActive) {
    unsigned long phase = (now - ledTestStartTime) / 500;
    switch (phase) {
      case 0: color = RGBColors::Red; break;
      case 1: color = RGBColors::Green; break;
      case 2: color = RGBColors::Blue; break;
      case 3: color = RGBColors::White; break;
      default: color = RGBColors::Purple; break;
    }
  } else if (isBleConnected() || isBleLedActive(now)) {
    color = RGBColors::Blue;
  } else if (batteryState == BatteryLedState::Charging) {
    color = breathingColor(RGBColors::Green, now, 2200);
  } else if (batteryState == BatteryLedState::Charged) {
    color = RGBColors::Green;
  } else if (batteryState == BatteryLedState::Low) {
    color = (now % 2000) < 350 ? RGBColors::Orange : RGBColors::Black;
  } else if (ledNotice != LedNotice::None) {
    bool on = ((ledNoticeUntilTime - now) / 180) % 2 == 0;
    color = on ? RGBColors::Red : RGBColors::Black;
  } else {
    color = ledBaseColor();
  }

  showImmediateLed(color);
}

void RobotController::showImmediateLed(const RGB& color) {
  led.showColor(color.r, color.g, color.b);
}

void RobotController::setLedBaseMode(LedBaseMode mode) {
  ledBaseMode = mode;
}

void RobotController::notifyLed(LedNotice notice) {
  ledNotice = notice;
  ledNoticeUntilTime = millis() + ledNoticeDuration;
}

bool RobotController::handleLedCommand(const String& command) {
  if (command == "LED_STATUS") {
    sendValue("led", ledStatusValue());
    return true;
  }

  if (command == "LED_TEST") {
    ledTestActive = true;
    ledTestStartTime = millis();
    sendOk("LED_TEST started");
    return true;
  }

  if (command.startsWith("LED_BRIGHTNESS ")) {
    String value = command.substring(15);
    value.trim();
    int percent = value.toInt();
    if (value.length() == 0 || (percent == 0 && value != "0") || percent < 0 || percent > 100) {
      sendError("INVALID_LED", "Use LED_BRIGHTNESS percent, 0-100");
      return true;
    }

    ledBrightnessPercent = percent;
    sendOk("LED_BRIGHTNESS set");
    return true;
  }

  if (command.startsWith("LED_MODE ")) {
    String value = command.substring(9);
    value.trim();
    value.toUpperCase();

    if (value == "AUTO") {
      ledModeSetting = LedModeSetting::Auto;
    } else if (value == "DIM") {
      ledModeSetting = LedModeSetting::Dim;
    } else if (value == "OFF") {
      ledModeSetting = LedModeSetting::Off;
    } else {
      sendError("INVALID_LED", "Use LED_MODE auto|dim|off");
      return true;
    }

    sendOk("LED_MODE set");
    return true;
  }

  return false;
}

String RobotController::ledStatusValue() const {
  unsigned long now = millis();
  BatteryLedState batteryState = batteryLedState();
  String value = "reason=";

  if (batteryState == BatteryLedState::Critical) {
    value += "battery_critical";
  } else if (ledTestActive) {
    value += "test";
  } else if (isBleConnected() || isBleLedActive(now)) {
    value += "ble_connected";
  } else if (batteryState == BatteryLedState::Charging) {
    value += "charging";
  } else if (batteryState == BatteryLedState::Charged) {
    value += "charged";
  } else if (batteryState == BatteryLedState::Low) {
    value += "battery_low";
  } else if (ledNotice != LedNotice::None && now < ledNoticeUntilTime) {
    value += "notice";
  } else {
    value += ledBaseModeName();
  }

  value += ",mode=";
  value += ledModeSettingName();
  value += ",brightness=";
  value += String(ledBrightnessPercent);
  value += ",base=";
  value += ledBaseModeName();
  value += ",battery=";
  value += batteryLedStateName();
  value += ",level=";
  value += String(lastBatteryReading.level);
  value += ",voltage=";
  value += String(lastBatteryReading.voltage);
  value += ",current=";
  value += String(lastBatteryReading.current);
  value += ",ble=";
  value += isBleConnected() ? "connected" : "disconnected";
  return value;
}

RobotController::BatteryLedState RobotController::batteryLedState() const {
  if (lastBatteryReading.level <= batteryCriticalLevel || lastBatteryReading.voltage <= batteryCriticalVoltage) {
    return BatteryLedState::Critical;
  }

  if (lastBatteryReading.current <= batteryChargingCurrentThreshold) {
    return BatteryLedState::Charging;
  }

  if (lastBatteryReading.level >= 95 && lastBatteryReading.current <= batteryChargedCurrentThreshold) {
    return BatteryLedState::Charged;
  }

  if (lastBatteryReading.level <= batteryLowLevel) {
    return BatteryLedState::Low;
  }

  return BatteryLedState::Normal;
}

const char* RobotController::ledBaseModeName() const {
  switch (ledBaseMode) {
    case LedBaseMode::Home: return "home";
    case LedBaseMode::RandomWeak: return "random_weak";
    case LedBaseMode::RandomStrong: return "random_strong";
    default: return "idle";
  }
}

const char* RobotController::ledModeSettingName() const {
  switch (ledModeSetting) {
    case LedModeSetting::Dim: return "dim";
    case LedModeSetting::Off: return "off";
    default: return "auto";
  }
}

const char* RobotController::batteryLedStateName() const {
  switch (batteryLedState()) {
    case BatteryLedState::Low: return "low";
    case BatteryLedState::Critical: return "critical";
    case BatteryLedState::Charging: return "charging";
    case BatteryLedState::Charged: return "charged";
    default: return "normal";
  }
}

RGB RobotController::ledBaseColor() const {
  switch (ledBaseMode) {
    case LedBaseMode::Home: return RGBColors::Cyan;
    case LedBaseMode::RandomWeak: return RGBColors::Orange;
    case LedBaseMode::RandomStrong: return RGBColors::Purple;
    default: return RGBColors::White;
  }
}

RGB RobotController::breathingColor(const RGB& color, unsigned long now, unsigned long period) const {
  unsigned long phase = now % period;
  unsigned long half = period / 2;
  uint8_t scale = 0;
  if (phase < half) {
    scale = (uint8_t)(40 + phase * 160 / half);
  } else {
    scale = (uint8_t)(40 + (period - phase) * 160 / half);
  }

  RGB result;
  result.r = (uint8_t)((uint32_t)color.r * scale / 256);
  result.g = (uint8_t)((uint32_t)color.g * scale / 256);
  result.b = (uint8_t)((uint32_t)color.b * scale / 256);
  return result;
}

bool RobotController::isBleConnected() const {
  if (deviceConnected) return true;
  return pServer != nullptr && pServer->getConnectedCount() > 0;
}

bool RobotController::isBleLedActive(unsigned long now) const {
  return lastBleActivityTime != 0 && now - lastBleActivityTime <= bleLedActiveMs;
}

void RobotController::updateBatteryMonitoring(bool force) {
  unsigned long now = millis();
  if (!force && lastBatteryReadTime != 0 && now - lastBatteryReadTime < batteryLedSampleInterval) return;

  lastBatteryReading = readBattery();
  lastBatteryReadTime = now;
}

// ============================================================
// 按键（两侧同步；单击循环 4 模式，双击回中）
// ============================================================

void RobotController::handleButton() {
#if BUTTON_PIN < 0
  return;
#else
  unsigned long now = millis();
  bool level = digitalRead(BUTTON_PIN);

  if (level != buttonLastLevel) {
    buttonLastLevel = level;
    if (level == LOW) {
      buttonPressArmed = true;
      buttonLastChangeTime = now;
    }
  }

  if (buttonPressArmed && level == LOW && (now - buttonLastChangeTime) >= buttonDebounceMs) {
    buttonPressArmed = false;
    if (buttonPendingSingle && now - buttonLastClickTime <= buttonDoubleClickMs) {
      buttonPendingSingle = false;
      buttonLastClickTime = 0;
      Serial.println("[Button] Resetting All Angles to 90");
      setLedBaseMode(LedBaseMode::Idle);
      left.resetTo90(100);
      right.resetTo90(100);
    } else {
      buttonLastClickTime = now;
      buttonPendingSingle = true;
    }
  }

  if (buttonPendingSingle && now - buttonLastClickTime > buttonDoubleClickMs) {
    buttonPendingSingle = false;
    buttonState = (buttonState + 1) % 4;
    left.stopAll();
    right.stopAll();

    switch (buttonState) {
      case 1:
        Serial.println("[Button] Moving to Home Position (both sides)");
        setLedBaseMode(LedBaseMode::Home);
        left.goHome(200);
        right.goHome(200);
        break;

      case 2:
        Serial.println("[Button] Starting REPEAT_RANDOM WEAK (both sides)");
        setLedBaseMode(LedBaseMode::RandomWeak);
        left.startRandomWeak();
        right.startRandomWeak();
        break;

      case 3:
        Serial.println("[Button] Starting REPEAT_RANDOM STRONG (both sides)");
        setLedBaseMode(LedBaseMode::RandomStrong);
        left.startRandomStrong();
        right.startRandomStrong();
        break;

      case 0:
        setLedBaseMode(LedBaseMode::Idle);
        Serial.println("[Button] Resetting All Angles to 90 (both sides)");
        left.resetTo90(200);
        right.resetTo90(200);
        break;
    }
  }
#endif
}

// ============================================================
// BLE / 电池 / 响应
// ============================================================

void RobotController::setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(BLE_MTU);   // 必须在 init 之后调用（GATT 栈已初始化）

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks(this));

  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

  // CMD：上位机 → 下位机（写 + 无响应写）
  pCmdCharacteristic = pService->createCharacteristic(
    BLE_CMD_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdCharacteristic->setCallbacks(new MyCharacteristicCallbacks(this));

  // EVT：下位机 → 上位机（通知 + 读）
  pEvtCharacteristic = pService->createCharacteristic(
    BLE_EVT_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pEvtCharacteristic->addDescriptor(new BLE2902());

  // 传输层接管 EVT
  transport.begin(pEvtCharacteristic, [this](uint8_t side, const String& cmd) {
    lastBleActivityTime = millis();
    processCommand(cmd, side);
  });

  pService->start();
  setupBatteryService();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->addServiceUUID(BLEUUID((uint16_t)BLE_BATTERY_SERVICE_UUID));
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("BLE advertising started (%s)...\n", BLE_DEVICE_NAME);
}

void RobotController::setupBatteryService() {
  BLEService *pBatteryService = pServer->createService(BLEUUID((uint16_t)BLE_BATTERY_SERVICE_UUID));
  pBatteryLevelCharacteristic = pBatteryService->createCharacteristic(
    BLEUUID((uint16_t)BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pBatteryLevelCharacteristic->addDescriptor(new BLE2902());
  BatteryReading battery = readBattery();
  lastBatteryReading = battery;
  lastBatteryReadTime = millis();
  setBatteryLevelValue(battery.level, false);
  pBatteryService->start();
}

void RobotController::updateBatteryLevel(bool forceNotify) {
  if (pBatteryLevelCharacteristic == nullptr) return;

  unsigned long now = millis();
  if (!forceNotify && (!isBleConnected() || now - lastBatteryNotifyTime < batteryNotifyInterval)) return;

  BatteryReading battery = readBattery();
  lastBatteryReading = battery;
  lastBatteryReadTime = now;
  setBatteryLevelValue(battery.level, true);
}

void RobotController::setBatteryLevelValue(uint8_t level, bool notify) {
  if (pBatteryLevelCharacteristic == nullptr) return;

  pBatteryLevelCharacteristic->setValue(&level, 1);
  lastBatteryNotifyTime = millis();
  if (notify && isBleConnected()) pBatteryLevelCharacteristic->notify();
}

void RobotController::checkTCodeTimeout() {
  if (lastTCodeReceiveTime == 0) return;
  if (millis() - lastTCodeReceiveTime <= tcodeTimeoutMs) return;

  left.tcodeTimeoutStop();
  right.tcodeTimeoutStop();
  vibrator.setPercent(0);
  lastTCodeReceiveTime = 0;
}

void RobotController::sendResponse(const String& response) {
  printBoth(response);
  transport.sendResponse(response);
}

void RobotController::sendTCodeDeviceResponse(const String& response) {
  sendResponse(response);
}

void RobotController::sendOk(const String& message) {
  String response = "OK ";
  response += message;
  sendResponse(response);
}

void RobotController::sendError(const String& code, const String& message) {
  notifyLed(LedNotice::Error);
  String response = "ERR ";
  response += code;
  response += " ";
  response += message;
  sendResponse(response);
}

void RobotController::sendValue(const String& key, const String& value) {
  String response = "VALUE ";
  response += key;
  response += " ";
  response += value;
  sendResponse(response);
}

String RobotController::firmwareVersionValue() const {
  String version = CDR01_FIRMWARE_VERSION;
  if (version.startsWith("v") || version.startsWith("V")) version = version.substring(1);
  return "CDR01v2 firmware " + version;
}

BatteryReading RobotController::readBattery() {
  BatteryReading battery = batteryMonitor.read();
  battery.level = batteryLevelFromVoltage(battery.voltage);
  return battery;
}

uint8_t RobotController::batteryLevelFromVoltage(float voltage) const {
  if (voltage <= batteryVoltageEmpty) return 0;
  if (voltage >= batteryVoltageFull) return 100;

  float level = (voltage - batteryVoltageEmpty) * 100.0 / (batteryVoltageFull - batteryVoltageEmpty);
  return (uint8_t)(level + 0.5);
}

// ============================================================
// BLE 回调
// ============================================================

void RobotController::MyServerCallbacks::onConnect(BLEServer* pServer) {
  parent->deviceConnected = true;
  parent->lastBleActivityTime = millis();
  Serial.println("Device connected.");
  parent->transport.onConnected();
}

void RobotController::MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  parent->deviceConnected = false;
  parent->lastBleActivityTime = 0;
  Serial.println("Device disconnected.");
  parent->transport.onDisconnected();

#if SAFETY_STOP_ON_BLE_DISCONNECT
  Serial.println("[SAFETY] BLE disconnected - stopping motion");
  parent->left.stopAll();
  parent->right.stopAll();
  parent->setLedBaseMode(RobotController::LedBaseMode::Idle);
#endif

  BLEDevice::startAdvertising();
}

void RobotController::MyCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t* param) {
  std::string raw = pCharacteristic->getValue();
  parent->deviceConnected = true;
  parent->transport.feed((const uint8_t*)raw.data(), raw.size());
}

void RobotController::MyCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  onWrite(pCharacteristic, nullptr);
}
