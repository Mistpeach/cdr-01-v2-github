# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""原作者 CSV 脚本解析（3 列：时间ms,左命令,右命令）。

格式约定（与原版 cdr-player/scripter 一致）：
  - 无表头；每行恰好 3 列；第 4 列起忽略。
  - 第 1 列 = 整数毫秒时间戳（升序，允许相等）。
  - 第 2/3 列 = 左右两侧的命令文本；空单元格 = 该侧不动作（保持当前动作）。
  - UTF-8（容忍 BOM），RFC4180 转义。
"""

import csv


class CsvError(Exception):
    """CSV 解析错误（带行号与原因）。"""


def load_csv(path) -> list:
    """读取并解析 CSV，返回按时间升序的事件列表 [(time_ms, left, right), ...]。"""
    rows = []
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        for lineno, row in enumerate(reader, start=1):
            if not row or (len(row) == 1 and row[0].strip() == ""):
                continue
            if len(row) < 3:
                raise CsvError("第 %d 行列数不足（需 3 列：时间,左,右）" % lineno)

            raw_t = row[0].strip()
            if raw_t == "":
                raise CsvError("第 %d 行时间戳为空" % lineno)
            try:
                t = int(float(raw_t))
            except ValueError:
                raise CsvError("第 %d 行时间戳不是整数毫秒：%r" % (lineno, raw_t))

            left = row[1].strip()
            right = row[2].strip()
            rows.append((t, left, right))

    rows.sort(key=lambda r: r[0])
    return rows
