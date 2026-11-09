<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Mistpeach -->
<!-- Based on the CDR01 project by 0721Robotics (https://github.com/0721robotics/cdr01-project);
     modifications by the CDR01 v2 project. -->

# CDR01 v2（分发包）

推翻原版通讯协议、上下位机自成一对的复刻版：**运动控制/脚本命令语义照搬原作者**（轨迹与原版一致），
**通讯层全新设计**（单 BLE 连接、二进制帧+CRC16）。硬件不变：ESP32-S3 N16R8 直驱 6×SG90。

## 目录

```
cdr-01v2-Distribute/
├── README.md                   # 本文件
├── LICENSE                     # Apache-2.0（软件：firmware/ + host/）
├── NOTICE                      # 第三方归属与修改声明
├── LICENSES/
│   └── CC-BY-4.0.txt           # 文档许可全文
├── 00-技术栈与开发计划.md        # 技术栈决策 + 阶段计划
├── 01-通信协议规范.md            # V2 协议完整规范（帧/CRC/UUID/命令语法/T-Code/错误码）
├── firmware/                   # 下位机（PlatformIO 工程）
│   ├── include/config.h        #   全部可调参数集中在此
│   ├── src/                    #   V2Protocol / V2Transport / RobotController / DeltaRobot ...
│   ├── platformio.ini
│   └── README.md
├── host/                       # 上位机（Python + PySide6 + bleak）
│   ├── cdr01v2_host/           #   protocol / ble / csvloader / player / media / ui_main
│   ├── tests/run_tests.py      #   协议与调度自测
│   ├── requirements.txt
│   ├── run.bat                 #   Windows 一键启动
│   └── README.md
└── docs/
    ├── 02-上位机使用手册.md
    ├── 03-固件烧录与排障.md
    └── 04-联调验收清单.md
```

## 快速开始

### 下位机

```bash
cd firmware
pio run -t upload        # 烧录（烧录前关闭占用串口的监视器）
pio device monitor       # 115200 查看日志
```

详细步骤与排障见 `docs/03-固件烧录与排障.md`。

### 上位机

```bash
cd host
python -m pip install -r requirements.txt
python -m cdr01v2_host.main        # 或 Windows 下双击 run.bat
```

需要 Python 3.10+。视频同步需要额外安装 **libmpv**（Windows 放 `mpv-2.dll`）。
详细用法见 `docs/02-上位机使用手册.md`。

## 核心特性

- 单 BLE 连接，CMD（写）/ EVT（通知）双特征，帧 `CD 01 | SIDE | LEN | 命令文本 | CRC16`。
- 左右 6 通道独立（L0-L2 / R0-R2），单连接内按 SIDE 字段分侧。
- 原作者格式 CSV 直接播放；mpv 视频同步；L/R 独立驱动；自动重连 + 心跳。
- 运动引擎/IK/插值/REPEAT/T-Code 照搬原版已移植代码，轨迹一致。

## 验证状态

- 固件：编译通过（Flash 约 15%）。
- 上位机：协议/调度自测 5/5 通过；模块语法检查通过。
- 实机 BLE 端到端已验收：`READY`、1Hz 心跳、双侧 JOINT 路由、`STOP`、VERSION 均正确。
- 已修复：BLE 回调内同步 notify 死锁、左右命令抢占互不干扰、SERVO LEDC 14 位、setMTU 时序、CH343 串口日志可见性。
- 待环境项：GUI 交互细节、视频实播、exe 打包（见 `docs/04-联调验收清单.md`）。

## License

本分发包按工件类型分许可：

| 工件 | 许可 | 文件 |
| --- | --- | --- |
| 软件（`firmware/`、`host/`） | **Apache-2.0** | `LICENSE` |
| 文档（`*.md`） | **CC BY 4.0** | `LICENSES/CC-BY-4.0.txt` |

本项目的固件派生自 0721Robotics 的
[cdr01-firmware](https://github.com/0721robotics/cdr01-firmware)（Apache-2.0），
文档基于 [cdr-01](https://github.com/0721robotics/cdr01-project)。
各文件的修改说明见文件头 SPDX 注释与 `NOTICE`。

**本分发包是非官方衍生项目，与原作者无隶属关系，也未获其背书。**
