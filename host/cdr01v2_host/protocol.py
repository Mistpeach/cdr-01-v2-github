# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""CDR01 v2 协议编解码（与固件 V2Protocol 对称，见 01-通信协议规范.md）。

帧布局：CD 01 | FIELD | LEN(2B LE) | payload | CRC16(2B LE)，总长 = 7 + LEN。
CRC-16/CCITT-FALSE：初值 0xFFFF，多项式 0x1021，不反转。
"""

MAGIC_0 = 0xCD
MAGIC_1 = 0x01

SIDE_BOTH = 0x00
SIDE_LEFT = 0x01
SIDE_RIGHT = 0x02

EVT_RESPONSE = 0x01
EVT_HEARTBEAT = 0x02

MAX_PAYLOAD = 240


class FrameError(Exception):
    """帧解析错误（CRC 错、长度非法、帧头错、截断）。"""


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            if c & 0x8000:
                c = ((c << 1) ^ 0x1021) & 0xFFFF
            else:
                c = (c << 1) & 0xFFFF
    return c


def _encode(field: int, payload: bytes) -> bytes:
    if not payload or len(payload) > MAX_PAYLOAD:
        raise ValueError("payload 长度需在 1..%d 之间" % MAX_PAYLOAD)
    body = bytes([field]) + len(payload).to_bytes(2, "little") + payload
    c = crc16(body)
    return bytes([MAGIC_0, MAGIC_1]) + body + c.to_bytes(2, "little")


def encode_cmd(side: int, text) -> bytes:
    """编码命令帧。text 为 str（按 utf-8）或 bytes。"""
    payload = text.encode("utf-8") if isinstance(text, str) else bytes(text)
    return _encode(side, payload)


def encode_evt(evt_type: int, text) -> bytes:
    payload = text.encode("utf-8") if isinstance(text, str) else bytes(text)
    return _encode(evt_type, payload)


def decode(data: bytes):
    """解析单个完整帧，返回 (field, payload_text)。

    field：CMD 帧里是 SIDE，EVT 帧里是 TYPE。
    失败抛 FrameError。
    """
    if len(data) < 8:
        raise FrameError("帧过短")
    if data[0] != MAGIC_0 or data[1] != MAGIC_1:
        raise FrameError("帧头错误")
    plen = data[3] | (data[4] << 8)
    if plen == 0 or plen > MAX_PAYLOAD:
        raise FrameError("长度非法")
    if len(data) < 7 + plen:
        raise FrameError("帧被截断")
    crc = data[5 + plen] | (data[6 + plen] << 8)
    if crc16(data[2:2 + 3 + plen]) != crc:
        raise FrameError("CRC 校验失败")
    return data[2], data[5:5 + plen].decode("utf-8", "replace")
