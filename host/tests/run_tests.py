# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""CDR01 v2 上位机逻辑层自测（无需 bleak/PySide6，纯标准库）。

运行：python tests/run_tests.py
覆盖：协议帧编解码（与 01-通信协议规范.md 的测试向量对齐）、CSV 解析、时间轴调度器。
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cdr01v2_host import protocol
from cdr01v2_host.csvloader import load_csv, CsvError
from cdr01v2_host.player import Scheduler

_PASS = 0


def check(name, fn):
    global _PASS
    try:
        fn()
        _PASS += 1
        print("  ok  %s" % name)
    except AssertionError as e:
        print("FAIL  %s: %s" % (name, e))
        raise


def hx(s):
    return bytes(int(x, 16) for x in s.split())


def test_protocol_vectors():
    vec = [
        ("STOP(both)", protocol.SIDE_BOTH, "STOP", "CD 01 00 04 00 53 54 4F 50 81 DC"),
        ("POINT L", protocol.SIDE_LEFT, "POINT 0 0 0 300",
         "CD 01 01 0F 00 50 4F 49 4E 54 20 30 20 30 20 30 20 33 30 30 94 43"),
        ("REPEAT_LINE R", protocol.SIDE_RIGHT, "REPEAT_LINE -50 0 -25 50 0 -25 700",
         "CD 01 02 22 00 52 45 50 45 41 54 5F 4C 49 4E 45 20 2D 35 30 20 30 20 2D 32 35 20 35 30 20 30 20 2D 32 35 20 37 30 30 30 11"),
    ]
    for name, side, text, hexs in vec:
        got = protocol.encode_cmd(side, text)
        assert got == hx(hexs), "%s: %s != %s" % (name, got.hex().upper(), hexs)

    assert protocol.encode_evt(protocol.EVT_HEARTBEAT, "HB") == hx("CD 01 02 02 00 48 42 04 54")
    assert protocol.encode_evt(protocol.EVT_RESPONSE, "[L] OK POINT started") == \
        hx("CD 01 01 14 00 5B 4C 5D 20 4F 4B 20 50 4F 49 4E 54 20 73 74 61 72 74 65 64 B2 14")


def test_protocol_decode():
    field, text = protocol.decode(hx("CD 01 00 04 00 53 54 4F 50 81 DC"))
    assert field == protocol.SIDE_BOTH and text == "STOP"

    # CRC 错 → FrameError
    bad = bytearray(hx("CD 01 00 04 00 53 54 4F 50 81 DC"))
    bad[-1] ^= 0xFF
    try:
        protocol.decode(bytes(bad))
        assert False, "CRC 错未抛异常"
    except protocol.FrameError as e:
        assert "CRC" in str(e)

    # 帧头错
    try:
        protocol.decode(hx("00 01 00 04 00 53 54 4F 50 81 DC"))
        assert False
    except protocol.FrameError:
        pass

    # 长度非法（0）
    try:
        protocol.decode(hx("CD 01 00 00 00 53 54 4F 50 81 DC"))
        assert False
    except protocol.FrameError:
        pass

    # 超长 payload 拒绝
    try:
        protocol.encode_cmd(protocol.SIDE_BOTH, "x" * 241)
        assert False
    except ValueError:
        pass


def test_csvloader():
    content = (
        "\ufeff150286,REPEAT_POINT 0 0 -70 0 0 50 500,\r\n"
        "78023,REPEAT_CIRCLE 0 0 -85 100 5000,\r\n"
        "151882,STOP,STOP\r\n"
        "150429,,REPEAT_POINT 0 0 -70 0 0 50 500\r\n"
        "9,\"CIRCLE 0 0 0 50,3000\",\r\n"
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, encoding="utf-8") as f:
        f.write(content)
        path = f.name
    try:
        events = load_csv(path)
        times = [e[0] for e in events]
        assert times == sorted(times), "未按时间排序"
        assert events[0][0] == 9 and events[0][1] == "CIRCLE 0 0 0 50,3000", "引号转义/首条"
        # 空单元格保持空串
        ev = dict(((e[0]), (e[1], e[2])) for e in events)
        assert ev[150286][1] == "", "左列空单元格"
        assert ev[78023][1] == "", "右列空单元格"
    finally:
        os.unlink(path)

    # 错误：时间戳非整数
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, encoding="utf-8") as f:
        f.write("abc,STOP,STOP\n")
        bad = f.name
    try:
        try:
            load_csv(bad)
            assert False, "非法时间戳未报错"
        except CsvError:
            pass
    finally:
        os.unlink(bad)


def test_scheduler():
    clock = {"t": 0}

    def now():
        return clock["t"]

    events = [(1000, "A", "B"), (1100, "", "C"), (2000, "D", "")]
    s = Scheduler(events, now_ms=now)
    assert s.count() == 3 and s.total_ms() == 2000

    s.play()
    clock["t"] = 1050
    cmds = s.due_commands()
    assert cmds == [(protocol.SIDE_LEFT, "A"), (protocol.SIDE_RIGHT, "B")], cmds

    clock["t"] = 1150
    cmds = s.due_commands()
    assert cmds == [(protocol.SIDE_RIGHT, "C")], cmds  # 左列空 → 不发左

    # 结束判定
    clock["t"] = 2100
    _, finished = s.due()
    assert finished is True

    # seek + resync
    s.stop()
    s.play()
    s.seek(1500)
    cmds = s.resync_commands()
    assert cmds == [(protocol.SIDE_LEFT, "A"), (protocol.SIDE_RIGHT, "C")], cmds

    # pause 冻结
    s.stop()
    clock["t"] = 0
    s.play()
    clock["t"] = 500
    s.pause()
    clock["t"] = 99999
    assert s.elapsed_ms() == 500
    s.play()
    clock["t"] = 100000
    assert s.elapsed_ms() == 501, "恢复后从 500 继续"


def test_advance_to():
    # 外部时钟（视频时间轴）驱动
    events = [(1000, "A", "B"), (1100, "", "C"), (2000, "D", "")]
    s = Scheduler(events)
    assert s.advance_commands(0) == []
    assert s.advance_commands(1050) == [(protocol.SIDE_LEFT, "A"), (protocol.SIDE_RIGHT, "B")]
    assert s.advance_commands(1150) == [(protocol.SIDE_RIGHT, "C")]
    # 后退重定位：不重复产出"过去已发过"的事件
    assert s.advance_commands(1000) == []
    assert s.advance_commands(1050) == []
    # 继续推进仍能产出后续事件
    assert s.advance_commands(1150) == [(protocol.SIDE_RIGHT, "C")]
    assert s.advance_commands(2500) == [(protocol.SIDE_LEFT, "D")]


if __name__ == "__main__":
    check("协议测试向量（编码）", test_protocol_vectors)
    check("协议解码/容错", test_protocol_decode)
    check("CSV 解析", test_csvloader)
    check("时间轴调度器", test_scheduler)
    check("外部时钟推进(视频同步)", test_advance_to)
    print("\n全部 %d 项自测通过" % _PASS)
