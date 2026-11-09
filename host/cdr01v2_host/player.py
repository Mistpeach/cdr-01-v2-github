# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""CSV 时间轴调度器（与 BLE 解耦，纯逻辑可单测）。

仿 cdr-player 的语义：
  - 播放推进由外部每 tick（~10ms）驱动，到期事件按时间戳返回待发送列表。
  - 左列 → SIDE_LEFT 帧、右列 → SIDE_RIGHT 帧；同 tick 内左/右事件都返回。
  - pause 冻结进度；seek 重定位；seek 跨度 >1200ms 时补发两侧最近一条命令
    （沿用 scripter 行为，保证姿态连续）。
"""

import time

from .protocol import SIDE_LEFT, SIDE_RIGHT, SIDE_BOTH


def _commands_of(events):
    """把事件列表转换为待发送命令 [(side, text)]，空文本跳过。"""
    cmds = []
    for _t, left, right in events:
        if left:
            cmds.append((SIDE_LEFT, left))
        if right:
            cmds.append((SIDE_RIGHT, right))
    return cmds


class Scheduler:
    def __init__(self, events, now_ms=None):
        self.events = sorted(events, key=lambda e: e[0])   # [(t, left, right)]
        self._now_ms = now_ms if now_ms is not None else (lambda: int(time.monotonic() * 1000))
        self.playing = False
        self._start_clock = 0          # 播放起点的单调时钟(ms)
        self._elapsed = 0              # 已播放毫秒（暂停时冻结）
        self.idx = 0                   # 下一个待发送事件下标
        self._pos = 0                  # 外部时钟模式的最后推进位置

    # ---- 播放控制 ----
    def play(self):
        if self.playing:
            return
        # 从已播放进度处继续（初次为 0）
        self._start_clock = self._now_ms() - self._elapsed
        self.playing = True

    def pause(self):
        if not self.playing:
            return
        self._elapsed = self._now_ms() - self._start_clock
        self.playing = False

    def stop(self):
        self.playing = False
        self._elapsed = 0
        self.idx = 0
        self._pos = 0

    def seek(self, ms):
        ms = max(0, ms)
        self._elapsed = ms
        self._pos = ms
        if self.playing:
            self._start_clock = self._now_ms() - ms
        self._reposition(ms)

    def elapsed_ms(self):
        if self.playing:
            return self._now_ms() - self._start_clock
        return self._elapsed

    # ---- 事件产出 ----
    def due(self):
        """返回 (到期事件列表 [(t, left, right)], 是否结束)。不推进时间本身。"""
        now = self.elapsed_ms()
        out = []
        while self.idx < len(self.events) and self.events[self.idx][0] <= now:
            out.append(self.events[self.idx])
            self.idx += 1
        return out, self.idx >= len(self.events)

    def due_commands(self):
        """把到期事件转换为待发送命令 [(side, text)]，空文本跳过。"""
        events, _finished = self.due()
        return _commands_of(events)

    # ---- 外部时钟推进（视频时间轴驱动）----
    def advance_to(self, ms):
        """由外部时钟（如 mpv 时间轴）推进：产出 t<=ms 的到期事件。

        后退（ms 小于上次位置）时自动重定位下标。返回事件列表。
        """
        if ms < self._pos:
            self._reposition(ms)
        self._pos = ms
        out = []
        while self.idx < len(self.events) and self.events[self.idx][0] <= ms:
            out.append(self.events[self.idx])
            self.idx += 1
        return out

    def advance_commands(self, ms):
        return _commands_of(self.advance_to(ms))

    def reset(self):
        self.stop()
        self._pos = 0

    def _reposition(self, ms):
        self.idx = 0
        for i, (t, _, _) in enumerate(self.events):
            if t > ms:
                self.idx = i
                break
        else:
            self.idx = len(self.events)

    # ---- Seek 补发 ----
    def last_commands_before(self, ms):
        """返回 <=ms 的最近一条左/右命令（用于 seek 后姿态连续）。"""
        last_left = None
        last_right = None
        for t, left, right in self.events:
            if t > ms:
                break
            if left:
                last_left = left
            if right:
                last_right = right
        return last_left, last_right

    def resync_commands(self):
        """seek 后由调用方决定是否补发；返回 [(side, text)] 与是否建议补发。"""
        ms = self.elapsed_ms()
        # 只在距离上一条命令较远（>1200ms）时补发
        ll, lr = self.last_commands_before(ms)
        cmds = []
        if ll:
            cmds.append((SIDE_LEFT, ll))
        if lr:
            cmds.append((SIDE_RIGHT, lr))
        return cmds

    # ---- 便捷 ----
    def total_ms(self):
        return self.events[-1][0] if self.events else 0

    def count(self):
        return len(self.events)
