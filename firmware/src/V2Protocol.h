// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#pragma once
// ============================================================
// V2Protocol —— CDR01 v2 帧编解码 + CRC16（见 01-通信协议规范.md）
// 帧布局：CD 01 | FIELD | LEN(2B LE) | payload | CRC16(2B LE)
// 总长 = 7 + LEN。CRC 覆盖 FIELD+LEN+payload。
// ============================================================

#include <Arduino.h>

class V2Protocol {
public:
  // CRC-16/CCITT-FALSE（初值 0xFFFF，多项式 0x1021，不反转）
  static uint16_t crc16(const uint8_t* data, size_t len);

  // 编码 CMD 帧（side + ASCII 命令）。返回帧总长；0 = payload 非法/超长。
  static size_t encodeCmd(uint8_t side, const String& payload, uint8_t* out, size_t outSize);
  static size_t encodeCmd(uint8_t side, const char* payload, size_t payloadLen, uint8_t* out, size_t outSize);

  // 编码 EVT 帧（type + ASCII 文本）。
  static size_t encodeEvt(uint8_t type, const String& payload, uint8_t* out, size_t outSize);

  // 解析单个完整帧（data 以 magic 开头）。返回：
  //   >0 = 帧总长（=7+LEN），field 与 payload 已填充
  //    0 = 数据不足（需更多字节）
  //   -1 = CRC 错误；-2 = 结构非法（LEN=0 或 >240）
  static int decode(const uint8_t* data, size_t len, uint8_t& field, String& payload);
};
