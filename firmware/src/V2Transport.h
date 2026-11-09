// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#pragma once
// ============================================================
// V2Transport —— v2 BLE 传输层（CMD 收 / EVT 发 + 心跳 + 重同步）
//
// 重要：BLE 的 onWrite 回调运行在 BLE 栈任务里，在该上下文里调用
//   notify()（esp_ble_gatts_send_indicate）会让栈自锁（连接不断但主循环卡死）。
//   因此 feed() 只把原始字节放进线程安全队列，真正的解析/响应由主循环
//   调用 poll() 完成（loop 上下文 notify 才安全）。
// ============================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "config.h"
#include <functional>

class V2Transport {
public:
  // evtChar: EVT 特征（NOTIFY+READ）；onCommand: 收到命令帧回调（在 loop 上下文调用）
  void begin(BLECharacteristic* evtChar, std::function<void(uint8_t side, const String& cmd)> onCommand);

  // BLE onWrite 调用：仅入队（线程安全、无阻塞、不发通知）
  void feed(const uint8_t* data, size_t len);
  // 调试用：喂入十六进制字符串
  void feed(const String& hexFrame);

  // 主循环调用：取出入队数据 → 解析 → 分发（响应在此上下文发送）
  void poll();

  // 发送一条响应（EVT type=0x01）
  void sendResponse(const String& text);

  // 连接建立 / 断开（连接回调里只置标志，READY 延迟到 tick 发送）
  void onConnected();
  void onDisconnected();

  // 周期任务：发 READY、1Hz 心跳
  void tick(uint32_t nowMs, bool connected);

  bool ready() const { return _evt != nullptr; }

private:
  BLECharacteristic* _evt = nullptr;
  std::function<void(uint8_t, const String&)> _onCommand;

  // 入队缓冲（BLE 任务写、loop 任务取，用临界区保护）
  uint8_t _inbox[V2_RX_BUFFER];
  volatile size_t _inboxLen = 0;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  // 解析缓冲（仅 loop 上下文访问）
  uint8_t _rx[V2_RX_BUFFER];
  size_t _rxLen = 0;
  uint32_t _lastHeartbeat = 0;
  volatile bool _readyPending = false;

  void _sendFrame(uint8_t type, const String& payload);
  void _parse();
  void _drop(size_t n);   // 丢弃解析缓冲前 n 字节
};
