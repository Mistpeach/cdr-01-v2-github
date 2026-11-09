# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""bleak BLE 连接层：扫描/连接/自动重连/收发。"""

import asyncio
import logging
import time

import bleak

from . import protocol

DEVICE_NAME = "CDR01v2"
SERVICE_UUID = "b4a00001-3f21-4c8e-9d5a-6b7e0f1a2c3d"
CMD_UUID = "b4a00002-3f21-4c8e-9d5a-6b7e0f1a2c3d"
EVT_UUID = "b4a00003-3f21-4c8e-9d5a-6b7e0f1a2c3d"
BATTERY_SERVICE_UUID = "0000180f-0000-1000-8000-00805f9b34fb"
BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"

log = logging.getLogger("cdr01v2.ble")


class V2Device:
    """单条连接：负责扫描连接、订阅 EVT、发送 CMD 帧。"""

    def __init__(self, name=DEVICE_NAME):
        self.name = name
        self.client = None
        self.on_event = None          # 回调(type, text)
        self.on_state = None          # 回调("connected"/"disconnected"/...)
        self.address = None
        self.last_heartbeat = 0.0
        self.battery_level = None

    @property
    def connected(self):
        return self.client is not None and self.client.is_connected

    async def connect(self):
        dev = await self._discover()
        if dev is None:
            raise RuntimeError("未扫描到设备 %r" % self.name)
        client = bleak.BleakClient(dev.address, disconnected_callback=self._on_disconnect)
        await client.connect()
        # 枚举服务确认特征存在（bleak 3.x 无 get_services()，改用 services 属性）
        _ = client.services
        await client.start_notify(EVT_UUID, self._on_evt)
        self.client = client
        self.address = dev.address
        self.last_heartbeat = time.monotonic()
        # 尝试读电池（可选，失败不影响）
        try:
            raw = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            if raw:
                self.battery_level = raw[0]
        except Exception:
            pass
        self._emit_state("connected")

    async def _discover(self):
        # 优先按上次地址快速找
        if self.address:
            try:
                d = await bleak.BleakScanner.find_device_by_address(self.address, timeout=2.0)
                if d:
                    return d
            except Exception:
                pass
        devs = await bleak.BleakScanner.discover(timeout=4.0, return_adv=True)
        # bleak 0.22 / 3.x 返回 {地址: (BLEDevice, AdvertisementData)}
        for _addr, item in devs.items():
            dev = item[0] if isinstance(item, tuple) else item
            if (dev.name or "") == self.name:
                return dev
        return None

    async def disconnect(self):
        if self.client:
            try:
                await self.client.disconnect()
            except Exception:
                pass
        self.client = None

    async def send(self, side, text):
        if not self.connected:
            raise RuntimeError("未连接")
        frame = protocol.encode_cmd(side, text)
        try:
            await self.client.write_gatt_char(CMD_UUID, frame, response=False)
        except Exception:
            # 回退带响应写
            await self.client.write_gatt_char(CMD_UUID, frame, response=True)

    def _on_evt(self, _char, data):
        try:
            field, text = protocol.decode(bytes(data))
        except protocol.FrameError as e:
            log.warning("EVT 帧解析失败: %s", e)
            return
        if field == protocol.EVT_HEARTBEAT:
            self.last_heartbeat = time.monotonic()
            return
        if self.on_event:
            self.on_event(field, text)

    def _on_disconnect(self, _client):
        self.client = None
        self._emit_state("disconnected")

    def _emit_state(self, state):
        if self.on_state:
            self.on_state(state)


class DeviceManager:
    """在独立 asyncio loop 中运行的连接管理器，含自动重连。"""

    def __init__(self, device: V2Device, reconnect=True, backoff=(1, 2, 4, 8, 10)):
        self.device = device
        self.reconnect = reconnect
        self.backoff = backoff
        self._want_stop = False

    async def run(self):
        """主循环：连接成功后保持；断开后按退避重连。"""
        attempt = 0
        while not self._want_stop:
            try:
                await self.device.connect()
                attempt = 0
                # 保持直到断开
                while self.device.connected and not self._want_stop:
                    await asyncio.sleep(0.5)
            except Exception as e:
                log.info("连接失败: %s", e)
            if self._want_stop:
                break
            if not self.reconnect:
                break
            delay = self.backoff[min(attempt, len(self.backoff) - 1)]
            log.info("%.0fs 后重连…", delay)
            await asyncio.sleep(delay)
            attempt += 1

    def stop(self):
        self._want_stop = True
