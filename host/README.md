<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Mistpeach -->
<!-- Based on the CDR01 project by 0721Robotics (https://github.com/0721robotics/cdr01-project);
     modifications by the CDR01 v2 project. -->

# CDR01 v2 上位机（Python + PySide6 + bleak）

仿 cdr-player 的桌面控制台，使用全新 V2 协议（单 BLE 连接、CMD/EVT 帧、CRC16）。

## 功能

- 连接：按广播名 `CDR01v2` 扫描、连接、断开、**自动重连**（退避 1/2/4/8/10s）、心跳检测。
- 手动控制：目标侧（双侧/左/右）+ 任意命令输入 + 快捷按钮（回中/Home/测试圆/随机/STOP）+ 振动滑杆。
- 脚本播放：载入**原作者格式 CSV**（`时间ms,左命令,右命令`），10ms 时间轴调度，左/右列分侧下发，
  播放/暂停/停止/拖动进度（跨度大时自动补发两侧最近命令保持姿态连续）。
- 视频同步：mpv（python-mpv）视频/音频与脚本时间轴联动；同名 CSV 自动加载；无 libmpv 自动降级纯脚本模式。
- 日志：实时显示 EVT 响应（带 `[L]`/`[R]` 标签）与发送记录。

## 目录

```
host/
├── requirements.txt
├── cdr01v2_host/
│   ├── protocol.py     # ★ 帧编解码 + CRC16（与固件 V2Protocol 对称）
│   ├── csvloader.py    # 原作者 CSV 解析
│   ├── player.py       # 时间轴调度器（单调时钟 + 外部时钟推进，可单测）
│   ├── media.py        # mpv 封装（无 libmpv 自动降级）
│   ├── ble.py          # bleak 连接层 + 自动重连（DeviceManager）
│   ├── ui_main.py      # PySide6 主窗口 + HostController（asyncio 线程桥）
│   └── main.py         # 入口
└── tests/run_tests.py  # 协议向量/CSV/调度器/外部时钟 自测（无需 bleak/PySide6）
```

## 安装与运行

```bash
pip install -r requirements.txt
# 视频同步（P4）需要 libmpv：见 ../00-技术栈与开发计划.md
python -m cdr01v2_host.main
```

> ✅ 已验证：`python tests/run_tests.py` 4 项自测全过（协议帧向量与固件/规范一致、
> CRC 容错、CSV 解析、时间轴调度）。GUI 与 BLE 需在装了 PySide6/bleak 且有实机的环境运行。
