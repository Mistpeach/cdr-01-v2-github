# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""mpv 视频/音频播放封装；libmpv 不可用时自动降级为纯脚本模式。"""

import logging

log = logging.getLogger("cdr01v2.media")


class MediaPlayer:
    def __init__(self):
        self._mpv = None
        self._path = None
        try:
            import mpv
            self._mpv = mpv.MPV(
                ytdl=False,
                osc=False,
                input_vo_keyboard=False,
                video=True,
                audio=True,
                keep_open="no",
            )
        except Exception as e:  # python-mpv 未装或找不到 libmpv
            log.info("mpv 不可用，进入纯脚本模式: %s", e)
            self._mpv = None

    @property
    def available(self):
        return self._mpv is not None

    @property
    def path(self):
        return self._path

    def load(self, path):
        self._path = path
        if self._mpv:
            self._mpv.play(path)

    def play(self):
        if self._mpv:
            self._mpv.pause = False

    def pause(self):
        if self._mpv:
            self._mpv.pause = True

    def stop(self):
        if self._mpv:
            try:
                self._mpv.command("stop")
            except Exception:
                pass

    def seek(self, seconds):
        if self._mpv:
            try:
                self._mpv.seek(seconds, reference="absolute")
            except Exception:
                pass

    def position_ms(self):
        """当前播放位置（毫秒）；无视频或未定位返回 None。"""
        if not self._mpv:
            return None
        try:
            t = self._mpv.time_pos
        except Exception:
            return None
        return int(t * 1000) if t is not None else None

    def duration_ms(self):
        if not self._mpv:
            return None
        try:
            d = self._mpv.duration
        except Exception:
            return None
        return int(d * 1000) if d else None

    @property
    def playing(self):
        if not self._mpv:
            return False
        try:
            return self._mpv.pause is False and self._mpv.time_pos is not None
        except Exception:
            return False
