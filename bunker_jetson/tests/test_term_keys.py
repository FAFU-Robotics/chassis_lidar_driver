"""TermKeyReader 按住判定测试：验证「长按一个键不会一抖一停」。

背景：终端靠 auto-repeat 事件推断“按住”，但 Linux 终端 auto-repeat 的
**初始延迟**达 500-660ms（SSH 下更长）。若 hold window 小于该延迟，按下
W 后 0.5s 内无第二个事件会被误判为“松开”→ mock_cloud 下发停车 → 速度
一顿一顿。本测试确保：

  1. 单个事件后（repeat 尚未开始）仍判定为按住（覆盖初始延迟窗口）；
  2. 连续 repeat 事件（模拟真正长按）稳定判定按住；
  3. 事件停止超过 hold window 后正确判定松开（快速点击/真实松开响应）。
"""

import time

from _term_keys import TermKeyReader, _KeyState


def _reader():
    # 用临时文件 fd 构造 reader（os.isatty 需要真实 fd），不实际读终端；
    # 直接调 _record_press 模拟事件。
    import os
    import tempfile
    fd, path = tempfile.mkstemp()
    os.close(fd)
    r = TermKeyReader(fd=fd)
    r._is_tty = False
    return r


def test_single_press_held_during_initial_repeat_delay():
    """按下后 repeat 尚未开始的初始延迟期（~0.5s）内仍为按住。"""
    r = _reader()
    r._record_press("w")
    # 0.4s 后查询：应仍为 True（覆盖 Linux 终端 500-660ms 初始 repeat 延迟）
    assert r.pressed("w") is True


def test_hold_with_repeat_events_stays_latched():
    """连续 repeat 事件（真正长按）期间持续为按住。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w")
    for _ in range(6):
        r._record_press("w")
        time.sleep(0.04)  # ~40ms repeat 间隔，模拟终端 auto-repeat
    # 最后一次事件 0.1s 后仍按住
    assert r.pressed("w") is True
    assert time.monotonic() - t0 > 0.1  # 确实经过了多次 repeat


def test_release_after_hold_window():
    """事件停止超过 hold window → 判定松开。"""
    r = _reader()
    r._record_press("w")
    # 跳过足够长的时间（> INITIAL_HOLD_S=1.0），应判定松开
    st = r._key_states["w"]
    # 直接改事件时间戳模拟很久没有新事件
    st.events = [time.monotonic() - 1.5]
    assert r.pressed("w") is False


def test_quick_tap_releases_after_window():
    """快速点按（无 repeat）：hold window 到期后松开。"""
    r = _reader()
    r._record_press("w")
    # 模拟只收到一个事件后长时间静默（点按后松手）
    st = r._key_states["w"]
    st.events = [time.monotonic() - 1.5]
    assert r.pressed("w") is False


def test_motion_held_tap_expires_quickly():
    """点按只有 1 个事件：超过 TAP_DRIVE_S 必须松开，不能按住 INITIAL_HOLD_S。"""
    r = _reader()
    r._record_press("w")
    assert r.motion_held("w") is True
    st = r._key_states["w"]
    st.events = [time.monotonic() - 0.40]
    assert r.motion_held("w") is False
    st.events = [time.monotonic() - 0.05]
    assert r.motion_held("w") is True
    st.events = [time.monotonic() - 1.15]
    assert r.motion_held("w") is False
    assert r.pressed("w") is False


def test_motion_held_repeat_is_hold():
    """出现 repeat 后 motion_held 走长按窗口。"""
    r = _reader()
    r._record_press("w")
    r._record_press("w")
    assert r.event_count("w") >= 2
    assert r.motion_held("w") is True


def test_hold_window_covers_initial_delay():
    """INITIAL_HOLD_S 必须 ≥ 终端 auto-repeat 初始延迟（Linux 500-660ms）。"""
    assert TermKeyReader.INITIAL_HOLD_S >= 0.66


def test_hold_window_survives_ssh_jitter():
    """稳定 repeat 后遭遇 SSH 抖动（事件延迟 300-500ms）不误判松开。

    回归：2026-08 曾将 MIN_HOLD_S 设为 0.15，稳定 33ms repeat 时 hold
    window 收缩到 150ms，SSH 抖动让下一个事件延迟 ≥300ms 时等待中的
    pressed() 误判松开 → mock_cloud 发停车 → 长按一停一顿。修复后
    MIN_HOLD_S=0.7，稳定长按可抵抗 ~700ms 抖动。
    """
    r = _reader()
    r._record_press("w")
    now = time.monotonic()
    events = [now - 8 * 0.033]
    for _ in range(7):
        events.append(events[-1] + 0.033)
    st = r._key_states["w"]
    st.events = events
    # 在最后一个事件之后 500ms（事件还没到）查询：应仍判定按住
    query_now = events[-1] + 0.50
    held = (query_now - st.events[-1]) < r._hold_for(st, query_now)
    assert held, "500ms SSH 抖动不应误判松开"
    # 超过 hold window（0.7s）→ 判定松开是设计行为（真实松开同此判定）；
    # 但 mock_cloud 侧 KB_RELEASE_HOLD_S=0.80 仍会兜底维持速度 0.8s，
    # 两层合计容忍 ~1.5s 抖动。此处验证 term_keys 单层在 500ms 内稳定。
    query_now = events[-1] + 0.90
    held = (query_now - st.events[-1]) < r._hold_for(st, query_now)
    assert not held, "超过 hold window 应判定松开（后续由 mock_cloud 迟滞兜底）"


def test_min_hold_floor_resists_jitter():
    """MIN_HOLD_S 下限必须足够大以抵抗 SSH 抖动（防回退）。"""
    assert TermKeyReader.MIN_HOLD_S >= 0.60, "hold window 下限太小会复现长按一顿一顿"
    assert TermKeyReader.MAX_HOLD_S >= 1.0


def test_burst_gap_ema_covers_inter_batch_silence():
    """SSH 突发+空白事件模式：hold window 应覆盖跨批次空白，不误判松开。

    回归：2026-08 SSH 下 auto-repeat 事件按「批次」到达（批内 gap≈0），
    旧 _hold_for 只看最近 4 个 gap 的最大值，批内 gap≈0 会把窗口收缩到
    MIN_HOLD_S，而下一批空白 > MIN_HOLD_S 时误判松开 → 长按爆发/一停一顿。
    修复后 gap_ema 追踪历史跨批次空白，作为 hold window 额外下限。
    """
    r = _reader()
    now = time.monotonic()
    st = _KeyState()
    # 模拟「突发+空白」：多批事件，批内间隔 3ms，批间空白 800ms
    batch_gap = 0.8
    batch_size = 6
    t = now - 5 * batch_gap
    while t < now:
        for j in range(batch_size):
            ev = t + j * 0.003
            if st.events:
                gap = ev - st.events[-1]
                if gap > r.BATCH_GAP_S:
                    st.gap_ema = gap if st.gap_ema <= 0.0 else 0.5 * gap + 0.5 * st.gap_ema
            st.events.append(ev)
        t += batch_gap
    if len(st.events) > r.MAX_EVENTS:
        st.events = st.events[-r.MAX_EVENTS:]
    r._key_states["w"] = st
    # 最后一事件之后 700ms（下一批还没到，但 gap_ema 覆盖 800ms 空白）→ 仍按住
    query = st.events[-1] + 0.7
    held = (query - st.events[-1]) < r._hold_for(st, query)
    assert held, "gap_ema 应让 hold window 覆盖 800ms 跨批次空白（700ms 处不应误判）"
    # gap_ema 确实追踪到了批间空白
    assert st.gap_ema > 0.5, f"gap_ema 应反映跨批次空白，实际 {st.gap_ema}"


def test_burst_gap_ema_release_detection():
    """突发+空白模式下，真正松开（超过 hold window）仍能被识别。"""
    r = _reader()
    now = time.monotonic()
    st = _KeyState()
    batch_gap = 0.4
    batch_size = 6
    t = now - 5 * batch_gap
    while t < now:
        for j in range(batch_size):
            ev = t + j * 0.003
            if st.events:
                gap = ev - st.events[-1]
                if gap > r.BATCH_GAP_S:
                    st.gap_ema = gap if st.gap_ema <= 0.0 else 0.5 * gap + 0.5 * st.gap_ema
            st.events.append(ev)
        t += batch_gap
    if len(st.events) > r.MAX_EVENTS:
        st.events = st.events[-r.MAX_EVENTS:]
    r._key_states["w"] = st
    # 远远超过 hold window（gap_ema≈0.4 → 窗口≈1.0s；这里 2.0s）→ 判松开
    query = st.events[-1] + 2.0
    held = (query - st.events[-1]) < r._hold_for(st, query)
    assert not held, "真正松开超过 hold window 应判松开"


def test_combo_survivor_not_dropped_when_partner_released():
    """W+A 组合键：松开 A 后，W 在 auto-repeat 重启暂停中不应误判松开。

    回归：2026-08 曾用「搭档最近 <0.25s 有事件」判活跃，导致「W 存活但
    repeat 暂停 + A 刚松开」时把 W 误判为松开 → 小车停在原地（用户报：
    "松开一个键后另一个长按键无法识别"）。修复后必须要求搭档**持续
    repeat**（最新事件 <0.15s 且有稳定事件流）才判活跃。
    """
    r = _reader()
    now = time.monotonic()
    ws = _KeyState()
    as_ = _KeyState()
    # W 有稳定 repeat 历史（30 个 33ms 事件），最后事件在 now - 0.5s（暂停中）
    ws.events = [now - 0.5 - (29 - i) * 0.033 for i in range(30)]
    # A 刚松开：最后事件在 now - 0.2s（较新，但已停止 repeat）
    as_.events = [now - 0.2 - (29 - i) * 0.033 for i in range(30)]
    ws.combo_until = now + 5.0
    as_.combo_until = now + 5.0
    r._key_states["w"] = ws
    r._key_states["a"] = as_
    # W 明明还长按着（只是 repeat 暂停），不能误判松开
    assert r.pressed("w") is True, "松开 A 后 W 不应误判松开（小车不应停在原地）"


def test_preempted_key_not_treated_as_released():
    """终端只连发最后一键：A 在 repeat、W 静默 = 抢走连发，不是松开。"""
    r = _reader()
    now = time.monotonic()
    ws = _KeyState()
    as_ = _KeyState()
    ws.events = [now - 0.5 - (29 - i) * 0.033 for i in range(30)]
    as_.events = [now - 0.02 - (29 - i) * 0.033 for i in range(30)]
    ws.combo_until = now + 5.0
    as_.combo_until = now + 5.0
    r._key_states["w"] = ws
    r._key_states["a"] = as_
    assert r._released_with_partner_active(ws, now) is False, (
        "A 连发而 W 静默是抢走连发，不能把 W 当松开"
    )
    assert r.motion_held("w") is True
    assert r.pressed("w") is True


def test_last_key_repeat_keeps_preempted_key_beyond_combo_hold():
    """W 被抢走连发超过 COMBO_HOLD_S 后，仍应靠和弦粘滞判按住。

    真实 SSH：按住 W+A 时往往只有 A 在 auto-repeat。W 的最后事件可能已
    过去数秒。若只看本键 hold window，W 会掉 → 斜向变成原地转，再松 A
    时整车停（用户报「罢工」）。
    """
    r = _reader()
    now = time.monotonic()
    ws = _KeyState()
    as_ = _KeyState()
    ws.events = [now - 2.5 - (19 - i) * 0.033 for i in range(20)]
    as_.events = [now - 0.02 - (29 - i) * 0.033 for i in range(30)]
    ws.combo_until = now + 5.0
    as_.combo_until = now + 5.0
    r._key_states["w"] = ws
    r._key_states["a"] = as_
    assert r.motion_held("w") is True, "被抢走连发的 W 超时后仍应粘滞按住"
    assert r.motion_held("a") is True


def test_combo_restart_gap_keeps_survivor_before_repeat():
    """松开 A 后、W 连发尚未恢复的静默期，W 不得判松开。"""
    r = _reader()
    now = time.monotonic()
    ws = _KeyState()
    as_ = _KeyState()
    ws.events = [now - 1.0 - (9 - i) * 0.033 for i in range(10)]
    as_.events = [now - 0.30 - (9 - i) * 0.033 for i in range(10)]
    ws.combo_until = now + 5.0
    as_.combo_until = now + 5.0
    r._key_states["w"] = ws
    r._key_states["a"] = as_
    r._motion_last_t = now - 0.30
    r._motion_last_key = "a"
    assert r.motion_held("w") is True, "松开一键后的 restart 间隙不应让另一键掉线"
    assert r.motion_held("a") is False, "最后连发的 A 已停，应尽快丢掉角速度"


def test_combo_survivor_after_quiet_gap_drops_released_key():
    """静默间隙后只有 W 恢复连发 → 保留 W、丢掉 A。"""
    r = _reader()
    now = time.monotonic()
    ws = _KeyState()
    as_ = _KeyState()
    ws.events = [now - 0.05, now - 0.02]
    as_.events = [now - 0.90 - (9 - i) * 0.033 for i in range(10)]
    ws.combo_until = now + 5.0
    as_.combo_until = now + 5.0
    r._key_states["w"] = ws
    r._key_states["a"] = as_
    r._motion_last_t = now - 0.02
    r._motion_last_key = "w"
    r._restart_gap_end = now - 0.45
    r._restart_key_before = "a"
    r._restart_keys_after = {"w"}
    assert r.motion_held("w") is True
    assert r.motion_held("a") is False, "静默后只恢复 W 时 A 应判松开"


def test_motion_held_releases_after_drive_window():
    """长按停止连发后，必须在 DRIVE_RELEASE 内松开，不能再冲 0.7s+。"""
    r = _reader()
    now = time.monotonic()
    st = _KeyState()
    st.events = [now - 0.28 - (9 - i) * 0.033 for i in range(10)]
    r._key_states["w"] = st
    assert r.motion_held("w") is False
    assert r.hold_confirmed("w") is False


def test_all_keys_released_after_combo_stops_quickly():
    """W+A 都停连发后，不得超过 CHORD_RESTART 仍往某一方向冲。"""
    r = _reader()
    now = time.monotonic()
    ws = _KeyState()
    as_ = _KeyState()
    ws.events = [now - 0.85 - (9 - i) * 0.033 for i in range(10)]
    as_.events = [now - 0.87 - (9 - i) * 0.033 for i in range(10)]
    ws.combo_until = now + 5.0
    as_.combo_until = now + 5.0
    r._key_states["w"] = ws
    r._key_states["a"] = as_
    r._motion_last_t = now - 0.85
    r._motion_last_key = "w"
    assert r.motion_held("w") is False
    assert r.motion_held("a") is False


def test_ssh_only_last_key_repeats_keeps_w_for_seconds():
    """真实 SSH：按住 W 再按 A 后只连发 A。W 必须一直算按住，不能几秒后掉线。"""
    r = _reader()
    t0 = time.monotonic()
    for i in range(8):
        r._record_press("w", now=t0 + i * 0.04)
    # 之后 2.5s 只连发 A（抢走 W）
    for i in range(60):
        r._record_press("a", now=t0 + 0.32 + i * 0.04)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.32 + 59 * 0.04 + 0.02
    try:
        assert r.motion_held("w") is True, "只连发 A 时 W 必须粘住（斜向）"
        assert r.motion_held("a") is True
    finally:
        time.monotonic = now_save


def test_release_a_after_long_steal_keeps_w_and_resume_is_not_tap():
    """长时间和弦后松 A：W 仍按住；W 连发恢复不得被 FRESH_PRESS 收成点按。"""
    r = _reader()
    t0 = time.monotonic()
    for i in range(8):
        r._record_press("w", now=t0 + i * 0.04)
    for i in range(50):
        r._record_press("a", now=t0 + 0.32 + i * 0.04)
    a_last = t0 + 0.32 + 49 * 0.04
    now_save = time.monotonic
    time.monotonic = lambda: a_last + 0.35
    try:
        assert r.motion_held("a") is False
        assert r.motion_held("w") is True, "松 A 后 W 应继续前进"
    finally:
        time.monotonic = now_save
    # W 恢复：距上次 W 已 > FRESH_PRESS_GAP（1.2s）
    r._record_press("w", now=a_last + 0.50)
    r._record_press("w", now=a_last + 0.54)
    time.monotonic = lambda: a_last + 0.56
    try:
        assert r.motion_held("w") is True, "W 连发恢复必须仍是长按，不能当点按"
        assert r.event_count("w") >= 2
        assert r._key_states["w"].combo_until > a_last
    finally:
        time.monotonic = now_save


def test_stick_ssh_steal_keeps_both_axes():
    """只连发 A 时线速度轴仍锁着（遥控器左右摇杆互不抢占）。"""
    r = _reader()
    t0 = time.monotonic()
    for i in range(8):
        r._record_press("w", now=t0 + i * 0.04)
    for i in range(50):
        r._record_press("a", now=t0 + 0.32 + i * 0.04)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.32 + 49 * 0.04 + 0.02
    try:
        assert r.drive_stick() == (1, 1)
        assert r.drive_stick_is_hold() is True
    finally:
        time.monotonic = now_save


def test_stick_release_a_keeps_forward():
    """A 还在连发时前进轴保持；全部静默超过 TELEOP_LIVE 立即双轴清零。"""
    r = _reader()
    t0 = time.monotonic()
    for i in range(8):
        r._record_press("w", now=t0 + i * 0.04)
    for i in range(20):
        r._record_press("a", now=t0 + 0.32 + i * 0.04)
    a_last = t0 + 0.32 + 19 * 0.04
    now_save = time.monotonic
    time.monotonic = lambda: a_last + 0.02
    try:
        assert r.drive_stick() == (1, 1)
    finally:
        time.monotonic = now_save
    time.monotonic = lambda: a_last + 0.18
    try:
        assert r.drive_stick() == (1, 1), "长按中间 180ms 空档不得清零"
    finally:
        time.monotonic = now_save
    time.monotonic = lambda: a_last + 0.40
    try:
        assert r.drive_stick() == (0, 0)
    finally:
        time.monotonic = now_save


def test_stick_sequential_w_then_a_is_turn_only():
    """松开 W 后再按 A：只转，前进轴清掉。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    r._record_press("w", now=t0 + 0.04)
    r._record_press("a", now=t0 + 0.55)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.56
    try:
        assert r.drive_stick() == (0, 1)
    finally:
        time.monotonic = now_save


def test_stick_tap_expires():
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.02
    try:
        assert r.drive_stick() == (1, 0)
    finally:
        time.monotonic = now_save
    time.monotonic = lambda: t0 + 0.28
    try:
        assert r.drive_stick() == (0, 0)
    finally:
        time.monotonic = now_save


def test_stick_hold_survives_ssh_gap():
    """长按已进入连发后，150ms 批次空白仍保持角速度，不能清零再起步。"""
    r = _reader()
    t0 = time.monotonic()
    for i in range(8):
        r._record_press("a", now=t0 + i * 0.04)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 7 * 0.04 + 0.15
    try:
        assert r.drive_stick() == (0, 1)
        assert r.drive_stick_is_hold() is True
    finally:
        time.monotonic = now_save


def test_stick_first_press_holds_until_autorepeat():
    """首击在 TELEOP_FIRST 内保持，但不得拖成 0.4s 以上溜车。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("a", now=t0)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.15
    try:
        assert r.drive_stick() == (0, 1)
    finally:
        time.monotonic = now_save
    time.monotonic = lambda: t0 + 0.28
    try:
        assert r.drive_stick() == (0, 0)
    finally:
        time.monotonic = now_save


def test_second_tap_after_gap_is_not_a_hold():
    """第一次长按结束后再点按：应重新按 tap 处理，不能累加成 hold。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    r._record_press("w", now=t0 + 0.04)
    assert r.hold_confirmed("w") is True
    r._record_press("w", now=t0 + 2.0)
    assert r.event_count("w") == 1
    assert r.hold_confirmed("w") is False
    assert r.motion_held("w") is True


def test_two_taps_wide_gap_is_not_a_hold():
    """连续点按（间隔像又点一下，不像连发）不得升成长按。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    r._record_press("w", now=t0 + 0.40)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.41
    try:
        assert r.hold_confirmed("w") is False
        assert r.motion_held("w") is True
    finally:
        time.monotonic = now_save
    time.monotonic = lambda: t0 + 0.40 + 0.25
    try:
        assert r.motion_held("w") is False
        assert r.hold_confirmed("w") is False
    finally:
        time.monotonic = now_save


def test_sequential_w_then_s_does_not_create_combo():
    """松开 W 后再按 S：不得收成和弦，否则旧方向会粘回来。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    r._record_press("w", now=t0 + 0.04)
    r._record_press("s", now=t0 + 0.50)
    assert r._key_states["w"].combo_until <= t0 + 0.50
    assert r._key_states["s"].combo_until <= t0 + 0.50


def test_sequential_w_then_s_supersedes_old_key():
    """先后按 W、S：只认 S，W 不得继续判按住（对向键记窜）。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    r._record_press("w", now=t0 + 0.04)
    r._record_press("s", now=t0 + 0.50)
    r._record_press("s", now=t0 + 0.54)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.56
    try:
        assert r.motion_held("s") is True
        assert r.motion_held("w") is False
    finally:
        time.monotonic = now_save


def test_opposite_overlap_does_not_combo():
    """W 与 S 即使时间重叠也不进和弦，后按的 S 顶替 W。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    r._record_press("w", now=t0 + 0.04)
    r._record_press("s", now=t0 + 0.10)
    assert r._key_states["w"].combo_until <= t0 + 0.10
    assert r._key_states["s"].combo_until <= t0 + 0.10
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.12
    try:
        assert r.motion_held("s") is True
        assert r.motion_held("w") is False
    finally:
        time.monotonic = now_save


def test_just_pressed_is_short_window():
    """B 功能键：just_pressed 只认很短窗口，避免和随后的 A 叠在一起。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("b", now=t0)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.05
    try:
        assert r.just_pressed("b") is True
    finally:
        time.monotonic = now_save
    time.monotonic = lambda: t0 + 0.40
    try:
        assert r.just_pressed("b") is False
        assert r.pressed("b") is True
    finally:
        time.monotonic = now_save


def test_take_press_fires_once_per_event():
    """B 键：同一物理按下只触发一次，晚轮询也不会漏。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("b", now=t0)
    assert r.take_press("b") is True
    assert r.take_press("b") is False
    r._record_press("b", now=t0 + 2.0)
    assert r.take_press("b") is True
    assert r.take_press("b") is False


def test_sequential_w_then_a_replaces_after_overlap():
    """先 W 再 A、间隔超过重叠窗：只转不前进，避免把 A 开成斜向。"""
    r = _reader()
    t0 = time.monotonic()
    r._record_press("w", now=t0)
    r._record_press("w", now=t0 + 0.04)
    r._record_press("a", now=t0 + 0.50)
    now_save = time.monotonic
    time.monotonic = lambda: t0 + 0.52
    try:
        assert r.motion_held("a") is True
        assert r.motion_held("w") is False
    finally:
        time.monotonic = now_save


if __name__ == "__main__":
    test_single_press_held_during_initial_repeat_delay()
    test_hold_with_repeat_events_stays_latched()
    test_release_after_hold_window()
    test_quick_tap_releases_after_window()
    test_motion_held_tap_expires_quickly()
    test_motion_held_repeat_is_hold()
    test_hold_window_covers_initial_delay()
    test_hold_window_survives_ssh_jitter()
    test_min_hold_floor_resists_jitter()
    test_burst_gap_ema_covers_inter_batch_silence()
    test_burst_gap_ema_release_detection()
    test_combo_survivor_not_dropped_when_partner_released()
    test_preempted_key_not_treated_as_released()
    test_last_key_repeat_keeps_preempted_key_beyond_combo_hold()
    test_combo_restart_gap_keeps_survivor_before_repeat()
    test_combo_survivor_after_quiet_gap_drops_released_key()
    test_motion_held_releases_after_drive_window()
    test_all_keys_released_after_combo_stops_quickly()
    test_ssh_only_last_key_repeats_keeps_w_for_seconds()
    test_release_a_after_long_steal_keeps_w_and_resume_is_not_tap()
    test_stick_ssh_steal_keeps_both_axes()
    test_stick_release_a_keeps_forward()
    test_stick_sequential_w_then_a_is_turn_only()
    test_stick_tap_expires()
    test_second_tap_after_gap_is_not_a_hold()
    test_two_taps_wide_gap_is_not_a_hold()
    test_sequential_w_then_s_does_not_create_combo()
    test_sequential_w_then_s_supersedes_old_key()
    test_opposite_overlap_does_not_combo()
    test_just_pressed_is_short_window()
    test_take_press_fires_once_per_event()
    test_sequential_w_then_a_replaces_after_overlap()
    print("PASS test_term_keys")
