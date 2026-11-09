<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Mistpeach -->
<!-- Based on the CDR01 project by 0721Robotics (https://github.com/0721robotics/cdr01-project);
     modifications by the CDR01 v2 project. -->

# CDR01 v2 固件（PlatformIO）

CDR01 v2 下位机：ESP32-S3 N16R8 直驱 6×SG90（GPIO 4/5/15/16/17/18）。
运动引擎/命令语义照搬原版已移植代码；**通讯层为全新 V2 协议**（单 BLE 连接、二进制帧+CRC16）。

## 与 v1（esp32s3-cdr01-firmware）的差别

- ✅ 全新 BLE：广播名 `CDR01v2`，服务/特征 UUID 见 `../01-通信协议规范.md`；
  CMD（写，上位机→下位机）+ EVT（通知，下位机→上位机：响应+1Hz 心跳）。
- ✅ 单连接左右复用：CMD 帧 `SIDE` 字段 = 双侧/左/右，不再需要双连接。
- ❌ 移除旧版 `CDR01` 文本协议与 GATT 服务（旧 App 无法连接，预期行为）。
- ❌ 移除 WiFi/UDP/mDNS 及 `WIFI_*`/`HOSTNAME_*` 命令。
- 保留：串口调试通道（文本命令 + `L:`/`R:` 前缀）、6 通道独立、T-Code 轴槽位（0-2 左 / 3-5 右）、LED/按键/电池/振动。

## 编译

```bash
pio run            # 首次需下载 espressif32@6.9.0 工具链
pio run -t upload  # 烧录（USB-CDC，115200）
pio device monitor
```

> ✅ 已编译验证：PlatformIO Core 6.2.0 + espressif32@6.9.0（Arduino core 3.0.7），
> RAM 14.7%、Flash 15.0%、16MB 分区。

## 串口自检（烧录后）

```
VERSION                    → VALUE version CDR01v2 firmware 0.1.0
JOINT 90 90 90 200         → [L] OK JOINT started + [R] OK JOINT started
L:JOINT 60 90 90 400       → [L] OK JOINT started（仅左侧臂0动）
R:JOINT 60 90 90 400       → [R] OK JOINT started（仅右侧臂0动）
STOP                       → OK STOP
BATTERY                    → VALUE battery voltage=5.00,current=0.10,level=100
```

BLE 帧格式与测试向量见 `../01-通信协议规范.md`（可用 nRF Connect 手写 hex 帧验证）。

## 目录

```
firmware/
├── platformio.ini
├── include/config.h        # 全部可调常量（V2 UUID/帧参数/引脚/几何/安全）
└── src/
    ├── main.cpp
    ├── DeltaCommandParser.*  # 命令/T-Code 解析（照搬原版）
    ├── DeltaRobot.*          # 单侧运动引擎 ×2（照搬原版，左右独立）
    ├── RobotController.*     # 全局层 + V2 帧路由（通讯层重写）
    ├── V2Protocol.*          # ★ 帧编解码 + CRC16
    ├── V2Transport.*         # ★ BLE CMD/EVT 收发 + 心跳 + 重同步
    └── ServoHAL.* StatusLed.* BatteryMonitor.* Vibrator.*
```
