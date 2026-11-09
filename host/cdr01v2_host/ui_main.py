# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""PySide6 主窗口：连接面板 + 手动控制 + CSV 播放器 + 视频同步 + 日志。"""

import asyncio
import os
import threading
import time

from PySide6.QtCore import QObject, Qt, QTimer, Signal, Slot
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QComboBox, QFileDialog, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPlainTextEdit, QPushButton, QSlider, QSplitter, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from . import protocol
from .ble import V2Device, DeviceManager
from .csvloader import load_csv, CsvError
from .player import Scheduler
from .media import MediaPlayer


# ---- 信号桥：asyncio 线程 → Qt 主线程 ----
class HostSignals(QObject):
    event = Signal(str)        # 响应文本
    state = Signal(str)        # connected / disconnected / ...
    battery = Signal(int)


class HostController(QObject):
    """持有事件循环线程与连接管理器；通过信号把设备事件抛给 UI。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.signals = HostSignals(self)
        self.device = V2Device()
        self.loop = asyncio.new_event_loop()
        self.manager = DeviceManager(self.device)
        self.device.on_event = self._on_event
        self.device.on_state = self.signals.state.emit
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.manager.run())

    def _on_event(self, _type, text):
        self.signals.event.emit(text)

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    @Slot()
    def connect_device(self):
        # DeviceManager.run 已在后台重连；这里触发一次立即连接尝试
        async def _go():
            if not self.device.connected:
                try:
                    await self.device.connect()
                except Exception:
                    pass
        self._submit(_go())

    @Slot()
    def disconnect_device(self):
        self._submit(self.device.disconnect())

    @Slot(int, str)
    def send_command(self, side, text):
        text = text.strip()
        if not text:
            return
        self.signals.event.emit("> " + text)

        async def _go():
            try:
                await self.device.send(side, text)
            except Exception as e:
                self.signals.event.emit("[发送失败] %s" % e)
        self._submit(_go())

    @property
    def connected(self):
        return self.device.connected


# ---- 主窗口 ----
class MainWindow(QMainWindow):
    def __init__(self, controller: HostController):
        super().__init__()
        self.controller = controller
        self.scheduler = Scheduler([], now_ms=lambda: int(time.monotonic() * 1000))
        self.media = MediaPlayer()
        self._video_loaded = False
        self._tick = QTimer(self)
        self._tick.setInterval(10)
        self._tick.timeout.connect(self._on_tick)

        self.setWindowTitle("CDR01 v2 控制台")
        self.resize(980, 640)
        self._build_ui()

        controller.signals.event.connect(self._append_log)
        controller.signals.state.connect(self._on_state)

    # ---------------- UI 构建 ----------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(self._build_connection_panel())
        left_layout.addWidget(self._build_manual_panel())
        left_layout.addStretch(1)
        splitter.addWidget(left)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(self._build_player_panel(), 3)
        right_layout.addWidget(self._build_log_panel(), 2)
        splitter.addWidget(right)
        splitter.setSizes([360, 600])

    def _build_connection_panel(self):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.addWidget(QLabel("<b>连接</b>"))

        self.status_label = QLabel("状态：未连接")
        lay.addWidget(self.status_label)
        self.battery_label = QLabel("电量：--")
        lay.addWidget(self.battery_label)

        row = QHBoxLayout()
        self.connect_btn = QPushButton("连接 CDR01v2")
        self.connect_btn.clicked.connect(self.controller.connect_device)
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.controller.disconnect_device)
        row.addWidget(self.connect_btn)
        row.addWidget(self.disconnect_btn)
        lay.addLayout(row)
        return box

    def _build_manual_panel(self):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.addWidget(QLabel("<b>手动控制</b>"))

        row = QHBoxLayout()
        row.addWidget(QLabel("目标侧"))
        self.side_combo = QComboBox()
        self.side_combo.addItem("双侧", protocol.SIDE_BOTH)
        self.side_combo.addItem("左侧", protocol.SIDE_LEFT)
        self.side_combo.addItem("右侧", protocol.SIDE_RIGHT)
        row.addWidget(self.side_combo)
        lay.addLayout(row)

        row2 = QHBoxLayout()
        self.cmd_edit = QLineEdit()
        self.cmd_edit.setPlaceholderText("命令，如 POINT 0 0 0 300")
        self.cmd_edit.returnPressed.connect(self._send_manual)
        send_btn = QPushButton("发送")
        send_btn.clicked.connect(self._send_manual)
        row2.addWidget(self.cmd_edit)
        row2.addWidget(send_btn)
        lay.addLayout(row2)

        quick = [
            ("回中(双侧)", "JOINT 90 90 90 200", protocol.SIDE_BOTH),
            ("Home(双侧)", "POINT 0 0 0 300", protocol.SIDE_BOTH),
            ("测试圆(双侧)", "CIRCLE 0 0 0 50 3000", protocol.SIDE_BOTH),
            ("随机弱(双侧)", "REPEAT_RANDOM", protocol.SIDE_BOTH),
            ("STOP", "STOP", protocol.SIDE_BOTH),
        ]
        grid = QHBoxLayout()
        for label, cmd, side in quick:
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, c=cmd, s=side: self.controller.send_command(s, c))
            grid.addWidget(b)
        lay.addLayout(grid)

        vr = QHBoxLayout()
        vr.addWidget(QLabel("振动"))
        self.vibrate_slider = QSlider(Qt.Horizontal)
        self.vibrate_slider.setRange(0, 100)
        self.vibrate_slider.setValue(0)
        self.vibrate_slider.sliderReleased.connect(self._send_vibrate)
        vr.addWidget(self.vibrate_slider)
        lay.addLayout(vr)
        return box

    def _build_player_panel(self):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.addWidget(QLabel("<b>脚本 / 视频播放</b>"))

        row = QHBoxLayout()
        load_btn = QPushButton("载入 CSV")
        load_btn.clicked.connect(self._load_csv)
        load_video_btn = QPushButton("载入视频")
        load_video_btn.clicked.connect(self._load_video)
        self.play_btn = QPushButton("播放")
        self.play_btn.clicked.connect(self._play)
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.clicked.connect(self._pause)
        self.stop_btn = QPushButton("停止")
        self.stop_btn.clicked.connect(self._stop)
        row.addWidget(load_btn)
        row.addWidget(load_video_btn)
        row.addWidget(self.play_btn)
        row.addWidget(self.pause_btn)
        row.addWidget(self.stop_btn)
        lay.addLayout(row)

        self.file_label = QLabel("脚本：未载入")
        lay.addWidget(self.file_label)
        self.video_label = QLabel("视频：未载入" + ("" if self.media.available else "（未检测到 mpv/libmpv，纯脚本模式）"))
        lay.addWidget(self.video_label)

        self.seek_slider = QSlider(Qt.Horizontal)
        self.seek_slider.setRange(0, 0)
        self.seek_slider.sliderReleased.connect(self._seek)
        lay.addWidget(self.seek_slider)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["时间ms", "左命令", "右命令"])
        self.table.horizontalHeader().setStretchLastSection(True)
        lay.addWidget(self.table)
        return box

    def _build_log_panel(self):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.addWidget(QLabel("<b>日志</b>"))
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        lay.addWidget(self.log)
        return box

    # ---------------- 槽 ----------------
    @Slot()
    def _send_manual(self):
        side = self.side_combo.currentData()
        self.controller.send_command(side, self.cmd_edit.text())
        self.cmd_edit.clear()

    @Slot()
    def _send_vibrate(self):
        self.controller.send_command(protocol.SIDE_BOTH, "VIBRATE %d" % self.vibrate_slider.value())

    @Slot()
    def _load_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择 CSV 脚本", "", "CSV (*.csv);;所有文件 (*)")
        if path:
            self._load_csv_path(path)

    def _load_csv_path(self, path):
        try:
            events = load_csv(path)
        except CsvError as e:
            self._append_log("[CSV 错误] %s" % e)
            return False
        self.scheduler = Scheduler(events, now_ms=lambda: int(time.monotonic() * 1000))
        self._refresh_table(events)
        self.file_label.setText("脚本：%s  （%d 条）" % (path, len(events)))
        self.seek_slider.setRange(0, self.scheduler.total_ms())
        self._append_log("已载入 %d 条脚本事件" % len(events))
        return True

    @Slot()
    def _load_video(self):
        if not self.media.available:
            self._append_log("[提示] 未检测到 mpv/libmpv，无法载入视频（可继续纯脚本模式）")
            return
        path, _ = QFileDialog.getOpenFileName(
            self, "选择视频/音频", "",
            "媒体文件 (*.mp4 *.mkv *.webm *.avi *.mov *.mp3 *.m4a *.flac *.wav);;所有文件 (*)")
        if not path:
            return
        self.media.load(path)
        self._video_loaded = True
        self.video_label.setText("视频：%s" % os.path.basename(path))
        self._append_log("已载入视频：%s" % path)
        # 同名 CSV 自动加载（仿 cdr-player 拖视频自动读脚本）
        base = os.path.splitext(path)[0]
        for ext in (".csv",):
            if os.path.exists(base + ext):
                self._load_csv_path(base + ext)
                break

    def _refresh_table(self, events):
        self.table.setRowCount(len(events))
        for r, (t, left, right) in enumerate(events):
            self.table.setItem(r, 0, QTableWidgetItem(str(t)))
            self.table.setItem(r, 1, QTableWidgetItem(left))
            self.table.setItem(r, 2, QTableWidgetItem(right))

    @Slot()
    def _play(self):
        if self.scheduler.count() == 0:
            self._append_log("[提示] 请先载入 CSV")
            return
        self.scheduler.play()
        if self._video_loaded and self.media.available:
            self.media.play()
        self._tick.start()

    @Slot()
    def _pause(self):
        self.scheduler.pause()
        if self._video_loaded and self.media.available:
            self.media.pause()

    @Slot()
    def _stop(self):
        self.scheduler.stop()
        if self._video_loaded and self.media.available:
            self.media.stop()
            self._video_loaded = False
            self.video_label.setText("视频：未载入")
        self._tick.stop()
        self.controller.send_command(protocol.SIDE_BOTH, "STOP")
        self.seek_slider.setValue(0)

    @Slot()
    def _seek(self):
        ms = self.seek_slider.value()
        self.scheduler.seek(ms)
        if self._video_loaded and self.media.available:
            self.media.seek(ms / 1000.0)
        # 跨度较大时补发两侧最近命令，保持姿态连续
        cmds = self.scheduler.resync_commands()
        for side, text in cmds:
            self.controller.send_command(side, text)

    @Slot()
    def _on_tick(self):
        # 视频同步模式：以 mpv 时间轴为主时钟
        if self._video_loaded and self.media.available and self.media.playing:
            pos = self.media.position_ms()
            if pos is not None:
                self.seek_slider.setValue(pos)
                for side, text in self.scheduler.advance_commands(pos):
                    self.controller.send_command(side, text)
                total = self.scheduler.total_ms()
                if total and pos >= total:
                    self._finish_playback()
                return

        if not self.scheduler.playing:
            return
        cmds = self.scheduler.due_commands()
        for side, text in cmds:
            self.controller.send_command(side, text)
        self.seek_slider.setValue(self.scheduler.elapsed_ms())
        _events, finished = self.scheduler.due()
        if finished:
            self._finish_playback()

    def _finish_playback(self):
        self.scheduler.pause()
        if self._video_loaded and self.media.available:
            self.media.pause()
        self._tick.stop()
        self.controller.send_command(protocol.SIDE_BOTH, "STOP")
        self._append_log("[脚本] 播放结束，已 STOP")

    @Slot(str)
    def _append_log(self, text):
        self.log.appendPlainText(text)
        self.log.moveCursor(QTextCursor.End)

    @Slot(str)
    def _on_state(self, state):
        if state == "connected":
            self.status_label.setText("状态：已连接")
        elif state == "disconnected":
            self.status_label.setText("状态：未连接")
        else:
            self.status_label.setText("状态：%s" % state)
