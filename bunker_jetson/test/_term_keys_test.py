"""验证 TermKeyReader 自适应按键灵敏度、组合键与稳定性。

场景（在 pty 伪终端上模拟真实键盘行为）：
  1. 单键按住（有 auto-repeat 事件流）→ 一直判定为按下，不闪烁
  2. 松开后（repeat 停止）→ 在窗口内判定为松开
  3. 同时按住 W+A → 两个键都判定为按下（组合键）
  4. 松开 A 后 W 事件中断 0.4s（真实终端 repeat 重启延迟）→ W 仍按下
  5. 只按一下（无 repeat）→ 初始窗口内保持
"""
from __future__ import annotations

import os
import pty
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _term_keys import TermKeyReader  # noqa: E402


def make_reader():
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    r = TermKeyReader(slave)
    r.start()
    time.sleep(0.15)
    return r, master


def close(master):
    try:
        os.close(master)
    except OSError:
        pass


def test_hold_while_repeating():
    """按住 w（30Hz auto-repeat）→ 持续判定按下，不闪烁。"""
    r, master = make_reader()
    try:
        os.write(master, b"w")  # 初始按下
        time.sleep(0.05)
        for _ in range(5):
            os.write(master, b"w")  # 模拟 20Hz auto-repeat
            time.sleep(0.05)
            assert r.pressed("w"), "repeat 期间 w 应一直按下"
        # 速度稳定性：长按期间 pressed() 连续采样必须全为 True（不闪烁）
        for _ in range(10):
            os.write(master, b"w")
            time.sleep(0.05)
            for _ in range(3):
                assert r.pressed("w"), "长按期间 w 闪烁（速度不稳）"
        print("PASS 按住（auto-repeat）w 持续判定按下，无闪烁")
    finally:
        close(master)


def test_release_detection():
    """松开（repeat 停止）后判定松开。"""
    r, master = make_reader()
    try:
        os.write(master, b"w")
        time.sleep(0.05)
        for _ in range(10):
            os.write(master, b"w")
            time.sleep(0.04)
        assert r.pressed("w")
        # 模拟松开：不再有任何事件
        t0 = time.monotonic()
        while r.pressed("w") and time.monotonic() - t0 < 2.0:
            time.sleep(0.02)
        dt = time.monotonic() - t0
        assert not r.pressed("w"), "松开后 w 应判定为松开"
        assert dt < 1.0, f"松开后应在 1s 内判定松开, 实际 {dt:.2f}s"
        print(f"PASS 松开检测（{dt*1000:.0f}ms）")
    finally:
        close(master)


def test_combo_keys():
    """同时按住 W+A → 两个键都按下（组合键识别）。"""
    r, master = make_reader()
    try:
        os.write(master, b"w")
        os.write(master, b"a")
        time.sleep(0.05)
        for _ in range(10):
            os.write(master, b"w")
            os.write(master, b"a")
            time.sleep(0.05)
        assert r.pressed("w"), "组合按住时 w 应按下"
        assert r.pressed("a"), "组合按住时 a 应按下"
        assert r.pressed("W"), "大小写不敏感"
        # 松开 a，w 仍保持（独立跟踪）
        for _ in range(10):
            os.write(master, b"w")
            time.sleep(0.04)
        assert r.pressed("w"), "a 松开后 w 仍应按下"
        print("PASS 组合键 W+A 同时识别，松开单个不影响另一个")
    finally:
        close(master)


def test_combo_release_other_stays():
    """关键 bug 回归：W+A 组合，松开 A 后 W 事件中断 0.8s（真实终端
    repeat 重启 + SSH 抖动），期间 W 必须保持按下。"""
    r, master = make_reader()
    try:
        # 同时按住 W+A（建立组合状态）
        for _ in range(10):
            os.write(master, b"w")
            os.write(master, b"a")
            time.sleep(0.04)
        assert r.pressed("w") and r.pressed("a")
        # 松开 A：A 停止发事件，W 的 repeat 也暂停 0.8s
        time.sleep(0.8)
        assert r.pressed("w"), "BUG: 松开 A 后 W 在 repeat 重启延迟期间被误判松开"
        print("PASS 组合键松开 A 后 W 在 0.8s 中断期内保持按下")
        # W 恢复 repeat
        for _ in range(5):
            os.write(master, b"w")
            time.sleep(0.04)
        assert r.pressed("w")
        print("PASS W repeat 恢复后仍按下")
    finally:
        close(master)


def test_single_tap_initial_hold():
    """单次按下（无 repeat）→ 初始窗口内保持。"""
    r, master = make_reader()
    try:
        os.write(master, b"s")
        time.sleep(0.1)
        assert r.pressed("s"), "单次按下后应保持（初始窗口）"
        print("PASS 单次按下保持（初始窗口）")
    finally:
        close(master)


def test_combo_last_key_repeat_keeps_other():
    """终端只连发最后一键：组合后只写 a，w 仍应判按住（抢走连发，不是松开）。"""
    r, master = make_reader()
    try:
        for _ in range(8):
            os.write(master, b"w")
            os.write(master, b"a")
            time.sleep(0.04)
        assert r.pressed("w") and r.pressed("a")
        for _ in range(20):
            os.write(master, b"a")
            time.sleep(0.04)
        assert r.pressed("w"), "只连发 A 时 W 应保持（last-key-repeat 抢连发）"
        assert r.pressed("a")
        print("PASS 只连发最后一键时，被抢走连发的键仍按住")
    finally:
        close(master)


def test_combo_shared_pause_holds():
    """W+A 同时暂停（auto-repeat 重启）→ 两键都保持按下。"""
    r, master = make_reader()
    try:
        for _ in range(8):
            os.write(master, b"w")
            os.write(master, b"a")
            time.sleep(0.04)
        # 两者都暂停 0.8s（模拟 repeat 重启延迟）
        time.sleep(0.8)
        assert r.pressed("w"), "共同暂停期间 W 应保持按下"
        assert r.pressed("a"), "共同暂停期间 A 应保持按下"
        print("PASS 组合共同暂停 0.8s 内两键都保持")
    finally:
        close(master)


def test_ctrl_c_arrives_as_byte_not_sigint():
    """Ctrl+C 应作为 \\x03 字节进入队列，而非触发 SIGINT。

    回归：旧实现用 tty.setcbreak() 保留 ISIG，Ctrl+C 被终端驱动转成
    SIGINT → KeyboardInterrupt 中断整个 mock_cloud 事件循环，导致 vl/vm/vb
    按 Ctrl+C 时直接退出云端（用户报 bug）。关闭 ISIG 后 Ctrl+C 会作为
    \\x03 字节被 read_char() 读到，命令模式才能区分「只关雷达」vs「退出」。
    """
    import termios
    r, master = make_reader()
    try:
        # 断言 slave 的 lflag 里 ISIG 已清除（这是修复的核心）
        attrs = termios.tcgetattr(r._fd)
        assert not (attrs[3] & termios.ISIG), "start() 必须关闭 ISIG，否则 Ctrl+C 触发 SIGINT"
        # 端到端：写入 \x03，应作为字节被读出
        os.write(master, b"\x03")
        deadline = time.monotonic() + 1.0
        ch = None
        while time.monotonic() < deadline:
            ch = r.read_char(0.05)
            if ch is not None:
                break
        assert ch == "\x03", f"Ctrl+C 应作为 \\x03 字节被读取，实际 {ch!r}"
        print("PASS Ctrl+C 作为字节读取（ISIG 已关闭）")
    finally:
        close(master)


def test_output_newline_carriage_return_preserved():
    """start() 后 stdout 的 \n 仍应被终端转换为 \r\n（排版不右移）。

    回归：旧实现清掉 OPOST 后 \n 只换行不回车，help/日志每一行都从上一行
    末尾的列开始打印 → 行首随机空格、排版逐行右移。修复后应保留
    OPOST|ONLCR，从 slave 侧读到输出时应是 CRLF 而不是裸 LF。
    """
    import termios
    r, master = make_reader()
    try:
        attrs = termios.tcgetattr(r._fd)
        # 修复核心：OPOST 与 ONLCR 必须置位（\n → \r\n）
        assert attrs[1] & termios.OPOST, "start() 必须保留 OPOST（否则 \n 不回车）"
        assert attrs[1] & termios.ONLCR, "start() 必须启用 ONLCR（\n → \r\n）"
        # 输入侧 raw 不受影响（ICANON/ECHO/ISIG 仍关闭）
        assert not (attrs[3] & termios.ICANON)
        assert not (attrs[3] & termios.ECHO)
        print("PASS 输出保留 OPOST|ONLCR（\n→\r\n），输入侧 raw 不受影响")
    finally:
        close(master)


def main():
    test_hold_while_repeating()
    test_release_detection()
    test_combo_keys()
    test_combo_release_other_stays()
    test_single_tap_initial_hold()
    test_combo_last_key_repeat_keeps_other()
    test_combo_shared_pause_holds()
    test_ctrl_c_arrives_as_byte_not_sigint()
    test_output_newline_carriage_return_preserved()
    print("ALL PASS — 自适应按键灵敏度、组合键稳定性、Ctrl+C/排版正常")


if __name__ == "__main__":
    main()
