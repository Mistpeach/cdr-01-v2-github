// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#include "V2Protocol.h"
#include "config.h"

uint16_t V2Protocol::crc16(const uint8_t* data, size_t len) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    c ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (c & 0x8000) {
        c = (uint16_t)((c << 1) ^ 0x1021);
      } else {
        c = (uint16_t)(c << 1);
      }
    }
  }
  return c;
}

static size_t encodeCommon(uint8_t field, const char* payload, size_t payloadLen,
                           uint8_t* out, size_t outSize) {
  if (payloadLen == 0 || payloadLen > V2_MAX_PAYLOAD) return 0;
  size_t total = 7 + payloadLen;
  if (out == nullptr || outSize < total) return 0;

  out[0] = V2_MAGIC_0;
  out[1] = V2_MAGIC_1;
  out[2] = field;
  out[3] = (uint8_t)(payloadLen & 0xFF);
  out[4] = (uint8_t)((payloadLen >> 8) & 0xFF);
  memcpy(out + 5, payload, payloadLen);

  uint16_t crc = V2Protocol::crc16(out + 2, 3 + payloadLen);   // FIELD(1)+LEN(2)+payload
  out[5 + payloadLen] = (uint8_t)(crc & 0xFF);
  out[6 + payloadLen] = (uint8_t)((crc >> 8) & 0xFF);
  return total;
}

size_t V2Protocol::encodeCmd(uint8_t side, const String& payload, uint8_t* out, size_t outSize) {
  return encodeCommon(side, payload.c_str(), payload.length(), out, outSize);
}

size_t V2Protocol::encodeCmd(uint8_t side, const char* payload, size_t payloadLen, uint8_t* out, size_t outSize) {
  return encodeCommon(side, payload, payloadLen, out, outSize);
}

size_t V2Protocol::encodeEvt(uint8_t type, const String& payload, uint8_t* out, size_t outSize) {
  return encodeCommon(type, payload.c_str(), payload.length(), out, outSize);
}

int V2Protocol::decode(const uint8_t* data, size_t len, uint8_t& field, String& payload) {
  if (len < 6) return 0;   // 至少 magic(2)+field(1)+len(2)+crc(2)，payload 最少 1 字节 → 实际 <8 才不足
  if (len < 8) return 0;

  uint16_t payloadLen = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
  if (payloadLen == 0 || payloadLen > V2_MAX_PAYLOAD) return -2;

  size_t total = 7 + payloadLen;
  if (len < total) return 0;

  uint16_t crc = (uint16_t)data[5 + payloadLen] | ((uint16_t)data[6 + payloadLen] << 8);
  uint16_t calc = crc16(data + 2, 3 + payloadLen);   // FIELD(1)+LEN(2)+payload
  if (crc != calc) return -1;

  field = data[2];
  char tmp[V2_MAX_PAYLOAD + 1];
  memcpy(tmp, data + 5, payloadLen);
  tmp[payloadLen] = '\0';
  payload = String(tmp);
  return (int)total;
}
