// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach
//
// Derived from cdr01-firmware DeltaRobotController.cpp/.h by 0721Robotics (Apache-2.0):
//   https://github.com/0721robotics/cdr01-firmware
// Changes (Apache-2.0 4(b)): BLE transport replaced by the V2 framed protocol;
// Wi-Fi/UDP and the legacy text protocol removed; dual-serial logging, per-side
// command routing and transport-decoupled command processing added.

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

// ============================================================
// RobotController —— CDR01 v2 固件全局层
// 传输：V2 BLE（CMD 帧收 / EVT 帧发，见 01-通信协议规范.md）+ USB 串口调试。
// 路由：帧 SIDE 字段（或串口 L:/R: 前缀）→ 左/右/双侧运动引擎。
// 运动引擎：DeltaRobot ×2（照搬原版，左右独立 6 通道）。
// 已移除：旧版 CDR01 文本协议/BLE 服务、WiFi/UDP/mDNS/WIFI_*/HOSTNAME_*。
// ============================================================

#include "config.h"
#include "DeltaCommandParser.h"
#include "DeltaRobot.h"
#include "ServoHAL.h"
#include "StatusLed.h"
#include "BatteryMonitor.h"
#include "Vibrator.h"
#include "V2Protocol.h"
#include "V2Transport.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Preferences.h>

// 简单 RGB 结构（替代原版 FastLED 的 CRGB）
struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  RGB() : r(0), g(0), b(0) {}
  RGB(uint8_t rr, uint8_t gg, uint8_t bb) : r(rr), g(gg), b(bb) {}
};

namespace RGBColors {
const RGB Black(0, 0, 0);
const RGB White(255, 255, 255);
const RGB Red(255, 0, 0);
const RGB Green(0, 255, 0);
const RGB Blue(0, 0, 255);
const RGB Cyan(0, 255, 255);
const RGB Orange(255, 128, 0);
const RGB Purple(128, 0, 255);
}

class RobotController {
public:
  RobotController()
    : pServer(nullptr),
      pCmdCharacteristic(nullptr),
      pEvtCharacteristic(nullptr),
      pBatteryLevelCharacteristic(nullptr),
      deviceConnected(false) {}

  void setup();
  void loop();

  static const int SIDE_BOTH = 0;
  static const int SIDE_LEFT = 1;
  static const int SIDE_RIGHT = 2;

private:
  enum class LedBaseMode {
    Idle,
    Home,
    RandomWeak,
    RandomStrong,
  };

  enum class LedModeSetting {
    Auto,
    Dim,
    Off,
  };

  enum class LedNotice {
    None,
    Error,
  };

  enum class BatteryLedState {
    Normal,
    Low,
    Critical,
    Charging,
    Charged,
  };

  // ---- 常量 ----
  const float batteryVoltageEmpty = BATTERY_EMPTY_VOLTAGE;
  const float batteryVoltageFull = BATTERY_FULL_VOLTAGE;
  const float batteryCriticalVoltage = 3.22;
  const uint8_t batteryLowLevel = 20;
  const uint8_t batteryCriticalLevel = 5;
  const float batteryChargingCurrentThreshold = -0.30;
  const float batteryChargedCurrentThreshold = 0.05;
  const unsigned long batteryNotifyInterval = 30000;
  const unsigned long batteryLedSampleInterval = 5000;
  const unsigned long tcodeTimeoutMs = TCODE_TIMEOUT_MS;
  const unsigned long bleLedActiveMs = 3000;

  // ---- HAL 对象与两侧机器人 ----
  ServoHAL servo;
  DeltaRobot left;
  DeltaRobot right;
  StatusLed led;
  BatteryMonitor batteryMonitor;
  Vibrator vibrator;
  V2Transport transport;

  // ---- 传输与命令 ----
  String commandBuffer = "";
  bool newCommandAvailable = false;
  int pendingSide = SIDE_BOTH;              // 下一条命令的路由侧（串口=BOTH，BLE 由帧 SIDE 决定）

  // ---- T-Code 全局状态 ----
  unsigned long lastTCodeReceiveTime = 0;
  unsigned long lastBleActivityTime = 0;
  bool tcodeDebugEnabled = false;

  Preferences tcodePreferences;

  // ---- 按键 ----
  int buttonState = 0;
  bool buttonLastLevel = HIGH;
  unsigned long buttonLastChangeTime = 0;
  unsigned long buttonLastClickTime = 0;
  bool buttonPressArmed = false;
  bool buttonPendingSingle = false;
  const unsigned long buttonDebounceMs = BUTTON_DEBOUNCE_MS;
  const unsigned long buttonDoubleClickMs = BUTTON_DOUBLE_CLICK_MS;

  // ---- LED 状态 ----
  LedBaseMode ledBaseMode = LedBaseMode::Idle;
  LedModeSetting ledModeSetting = LedModeSetting::Auto;
  LedNotice ledNotice = LedNotice::None;
  unsigned long ledNoticeUntilTime = 0;
  bool ledTestActive = false;
  unsigned long ledTestStartTime = 0;
  int ledBrightnessPercent = 100;
  const uint8_t ledMaxBrightness = LED_MAX_BRIGHTNESS;
  const int ledDimPercent = 20;
  const unsigned long ledNoticeDuration = 1800;
  const unsigned long ledTestDuration = 2500;

  // ---- 电池 ----
  unsigned long lastBatteryNotifyTime = 0;
  unsigned long lastBatteryReadTime = 0;
  BatteryReading lastBatteryReading = {0.0f, 0.0f, 100};

  // ---- 节拍调度 ----
  unsigned long lastTickTime = 0;

  // ---- 命令处理 ----
  void processCommand(const String& command, int forceSide);
  void handleTCodeCommand(const ParsedTCodeCommand& command);
  bool handleVersionCommand(const String& command);
  bool handleTCodeConfigCommand(const String& command);
  String tcodeDebugSummary(const ParsedTCodeCommand& command) const;
  void resetTCodeAxisSettings();
  void setupTCodeSettings();
  void saveTCodeAxisSettings();
  void clearSavedTCodeAxisSettings();
  bool loadTCodeAxisSettings();
  String tcodeStatusValue() const;
  String tcodeCapabilitiesValue() const;
  bool parseTCodeAxisId(const String& value, char& axisType, int& channel) const;
  void handleBatteryCommand();

  DeltaRobot* sideRobot(int side);
  void routeMotionCommand(const ParsedCommand& parsed, int side);

  // ---- LED ----
  void setupLed();
  void updateLed();
  void showImmediateLed(const RGB& color);
  void setLedBaseMode(LedBaseMode mode);
  void notifyLed(LedNotice notice);
  bool handleLedCommand(const String& command);
  String ledStatusValue() const;
  BatteryLedState batteryLedState() const;
  const char* ledBaseModeName() const;
  const char* ledModeSettingName() const;
  const char* batteryLedStateName() const;
  RGB ledBaseColor() const;
  RGB breathingColor(const RGB& color, unsigned long now, unsigned long period) const;
  bool isBleConnected() const;
  bool isBleLedActive(unsigned long now) const;
  void updateBatteryMonitoring(bool force);

  void handleButton();

  // ---- BLE ----
  void setupBLE();
  void setupBatteryService();
  void updateBatteryLevel(bool forceNotify);
  void setBatteryLevelValue(uint8_t level, bool notify);
  void checkTCodeTimeout();
  // 双口日志：同时写 USB CDC（Serial）与 UART0（Serial0/GPIO43-44），
  // 避免不同板子"Arduino Serial 落在另一个 COM 口"导致看不到打印。
  void printBoth(const String& line);
  void sendResponse(const String& response);
  void sendTCodeDeviceResponse(const String& response);
  void sendOk(const String& message);
  void sendError(const String& code, const String& message);
  void sendValue(const String& key, const String& value);
  String firmwareVersionValue() const;
  BatteryReading readBattery();
  uint8_t batteryLevelFromVoltage(float voltage) const;

  BLEServer *pServer;
  BLECharacteristic *pCmdCharacteristic;
  BLECharacteristic *pEvtCharacteristic;
  BLECharacteristic *pBatteryLevelCharacteristic;
  bool deviceConnected;

  class MyServerCallbacks : public BLEServerCallbacks {
    RobotController* parent;
  public:
    MyServerCallbacks(RobotController* p) : parent(p) {}
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
  };

  class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    RobotController* parent;
  public:
    MyCharacteristicCallbacks(RobotController* p) : parent(p) {}
    void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t* param) override;
    void onWrite(BLECharacteristic *pCharacteristic) override;
  };
};

#endif
