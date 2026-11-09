// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#include "V2Transport.h"
#include "V2Protocol.h"

void V2Transport::begin(BLECharacteristic* evtChar, std::function<void(uint8_t, const String&)> onCommand) {
  _evt = evtChar;
  _onCommand = onCommand;
  _rxLen = 0;
  _inboxLen = 0;
}

void V2Transport::onConnected() {
  _rxLen = 0;
  _inboxLen = 0;
  _readyPending = true;   // READY 延迟到 tick（loop 上下文）发送
}

void V2Transport::onDisconnected() {
  _rxLen = 0;
  portENTER_CRITICAL(&_mux);
  _inboxLen = 0;
  portEXIT_CRITICAL(&_mux);
  _readyPending = false;
}

// ---- BLE onWrite：仅入队（不可在此发通知）----
void V2Transport::feed(const uint8_t* data, size_t len) {
  if (len == 0) return;
  portENTER_CRITICAL(&_mux);
  if (_inboxLen + len <= sizeof(_inbox)) {
    memcpy(_inbox + _inboxLen, data, len);
    _inboxLen += len;
  } else {
    _inboxLen = 0;   // 溢出：丢弃并让上层看到 ERR（由 poll 发现）
  }
  portEXIT_CRITICAL(&_mux);
}

void V2Transport::feed(const String& hexFrame) {
  String s = hexFrame;
  s.trim();
  uint8_t buf[V2_RX_BUFFER];
  size_t n = 0;
  int start = 0;
  while (start < (int)s.length() && n < sizeof(buf)) {
    int sp = s.indexOf(' ', start);
    String tok = sp >= 0 ? s.substring(start, sp) : s.substring(start);
    tok.trim();
    if (tok.length() == 2) {
      buf[n++] = (uint8_t)strtol(tok.c_str(), nullptr, 16);
    }
    if (sp < 0) break;
    start = sp + 1;
  }
  if (n > 0) feed(buf, n);
}

// ---- 主循环：搬出队列并解析（响应在此上下文发送，通知安全）----
void V2Transport::poll() {
  size_t n = 0;
  uint8_t tmp[V2_RX_BUFFER];
  portENTER_CRITICAL(&_mux);
  n = _inboxLen;
  if (n > 0) {
    memcpy(tmp, _inbox, n);
    _inboxLen = 0;
  }
  portEXIT_CRITICAL(&_mux);
  if (n == 0) return;

  if (_rxLen + n > sizeof(_rx)) {
    _rxLen = 0;
    sendResponse("ERR FRAME");
    return;
  }
  memcpy(_rx + _rxLen, tmp, n);
  _rxLen += n;
  _parse();
}

void V2Transport::_sendFrame(uint8_t type, const String& payload) {
  if (_evt == nullptr) return;
  uint8_t buf[V2_MAX_PAYLOAD + 7];
  size_t total = V2Protocol::encodeEvt(type, payload, buf, sizeof(buf));
  if (total == 0) return;

  // EVT 特征值 = 最近一帧（二进制）；Read 返回该帧字节，Notify 发送同值
  _evt->setValue(buf, total);
  _evt->notify();
}

void V2Transport::sendResponse(const String& text) {
  _sendFrame(V2_EVT_RESPONSE, text);
}

void V2Transport::tick(uint32_t nowMs, bool connected) {
  if (!connected) return;

  // 连接后先发 READY（在 loop 上下文，安全）
  if (_readyPending) {
    _readyPending = false;
    _lastHeartbeat = nowMs;
    _sendFrame(V2_EVT_RESPONSE, "READY");
    return;
  }

  if (nowMs - _lastHeartbeat >= V2_HEARTBEAT_INTERVAL_MS) {
    _lastHeartbeat = nowMs;
    _sendFrame(V2_EVT_HEARTBEAT, "HB");
  }
}

void V2Transport::_drop(size_t n) {
  if (n >= _rxLen) {
    _rxLen = 0;
    return;
  }
  memmove(_rx, _rx + n, _rxLen - n);
  _rxLen -= n;
}

void V2Transport::_parse() {
  while (_rxLen >= 8) {   // 最小帧 = 7 + 1(payload)
    // 1) 找帧头
    if (!(_rx[0] == V2_MAGIC_0 && _rx[1] == V2_MAGIC_1)) {
      _drop(1);
      continue;
    }

    uint16_t payloadLen = (uint16_t)_rx[3] | ((uint16_t)_rx[4] << 8);
    if (payloadLen == 0 || payloadLen > V2_MAX_PAYLOAD) {
      sendResponse("ERR FRAME");
      _drop(2);   // 丢弃帧头，重同步
      continue;
    }

    size_t total = 7 + payloadLen;
    if (_rxLen < total) return;   // 等待更多数据

    uint16_t crc = (uint16_t)_rx[5 + payloadLen] | ((uint16_t)_rx[6 + payloadLen] << 8);
    uint16_t calc = V2Protocol::crc16(_rx + 2, 3 + payloadLen);   // FIELD(1)+LEN(2)+payload
    if (crc != calc) {
      sendResponse("ERR CRC");
      _drop(2);   // 丢弃帧头，重同步
      continue;
    }

    uint8_t side = _rx[2];
    char tmp[V2_MAX_PAYLOAD + 1];
    memcpy(tmp, _rx + 5, payloadLen);
    tmp[payloadLen] = '\0';
    String cmd(tmp);

    if (_onCommand) _onCommand(side, cmd);
    _drop(total);
  }
}
