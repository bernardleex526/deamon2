#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MOTION-GUARD-TEST: motion_control 纯逻辑单测（标准库 unittest，离线）。

被测包: /home/user/new_chase/motion_control（core.py，无 ROS / 无 socket / 无 IO）
约定（来自 motion_control/README.md 合同 + MOTION-GUARD-TEST 任务）：
- 默认 output_enabled=False：绝不放行、绝不发送（连零速都不发）；
- BasicStatus 七个官方字段必须精确 int 匹配（bool 不算），坏输入立即 disarm 锁存；
- 单调钟：非法（非 finite / bool / 负数）或回拨 → clock_invalid 并 disarm；
  时间戳 0 是合法单调钟测试值；
- TTL：command 0.3 / status 0.5 / tracking 0.5 / scan(source_age+停留) ≤ 0.5，
  未来量最多 0.1；过期/坏输入解除 arm 后不自动恢复，须再次显式 arm()
  （arm/evaluate 阻塞路径会清除 exclusive，重新放行需再次人工确认独占）；
- 限幅先 clamp 再死区，绝不提高速度，不取绝对值代替符号：
  x ∈ [0, 0.3]（禁后退），y 恒 0，yaw ∈ [-0.6, 0.6]，|x|<0.2 → 0，|yaw|<0.5 → 0；
- GuardedOutput：仅放行后调 send；blocked 只补发一次零速；send 异常原样
  传播并 disarm（清 exclusive），绝不吞错伪成功。

运行（离线，不启动 ROS / DDS / 任何节点）:
  cd /home/user/new_chase && python3 -B -m unittest test.test_motion_guard -v
"""

import unittest

from motion_control.core import (
    CMD_TTL,
    MAX_VX,
    MAX_YAW,
    SCAN_FUTURE,
    SCAN_TTL,
    STATUS_TTL,
    TRACKING_TTL,
    X_DEADZONE,
    YAW_DEADZONE,
    Decision,
    GuardedOutput,
    MotionGuard,
    Velocity,
)

ZERO = Velocity()


def good_status(**overrides):
    """七个官方字段的合法 BasicStatus（int 精确值），可按字段覆盖/删除。"""
    status = {
        "MotionState": 17,
        "ControlUsageMode": 1,
        "Direction": 0,
        "HES": 0,
        "Charge": 0,
        "Sleep": 0,
        "Gait": 4097,
    }
    for key, value in overrides.items():
        if value is None:
            status.pop(key, None)
        else:
            status[key] = value
    return status


class SendRecorder(object):
    """记录 send 调用（velocity, 次数）；不做任何真实传输。"""

    def __init__(self):
        self.calls = []

    def __call__(self, velocity):
        self.calls.append(velocity)

    @property
    def count(self):
        return len(self.calls)

    @property
    def velocities(self):
        return [(v.x, v.y, v.yaw) for v in self.calls]


def feed_all_good(guard, t=0.0, command=None):
    """写入全套合法新鲜输入（时间戳均为 t，单调钟不回拨）。"""
    guard.update_status(good_status(), t)
    guard.update_scan(True, True, 0.0, t)
    guard.update_tracking(True, t)
    guard.update_command(ZERO if command is None else command, t)


def make_ready_guard(command=None, t=0.0):
    """output_enabled=True + exclusive=True + 全新输入；arm 由用例显式调用。"""
    guard = MotionGuard(output_enabled=True)
    feed_all_good(guard, t=t, command=command)
    guard.set_exclusive_control(True)
    return guard


def rearm(guard, now):
    """blocked 后重新放行：先人工重新确认独占，再显式 arm。"""
    guard.set_exclusive_control(True)
    return guard.arm(now)


class TestConstants(unittest.TestCase):
    """限幅 / 死区 / TTL 常量与合同一致（防止常量被静默改动）。"""

    def test_limits_and_deadzones(self):
        self.assertEqual(MAX_VX, 0.3)
        self.assertEqual(MAX_YAW, 0.6)
        self.assertEqual(X_DEADZONE, 0.2)
        self.assertEqual(YAW_DEADZONE, 0.5)

    def test_ttls(self):
        self.assertEqual(CMD_TTL, 0.3)
        self.assertEqual(STATUS_TTL, 0.5)
        self.assertEqual(SCAN_TTL, 0.5)
        self.assertEqual(TRACKING_TTL, 0.5)
        self.assertEqual(SCAN_FUTURE, 0.1)

    def test_decision_shape(self):
        d = Decision(ZERO, False, False, "disarmed")
        self.assertFalse(d.armed)
        self.assertFalse(d.allowed)
        self.assertEqual(d.reason, "disarmed")
        self.assertEqual(d.velocity, ZERO)


class TestDefaultNoSend(unittest.TestCase):
    """默认 output_enabled=False：绝不放行、绝不调用 send（连零速都不发）。"""

    def test_output_enabled_rejects_non_bool(self):
        for bad in ("true", 1, 0, None, [True]):
            with self.assertRaises(TypeError):
                MotionGuard(output_enabled=bad)

    def test_arm_refuses_without_output_enabled(self):
        guard = MotionGuard()  # 默认 False
        feed_all_good(guard, t=0.0, command=Velocity(0.3, 0.0, 0.6))
        guard.set_exclusive_control(True)
        d = guard.arm(0.0)
        self.assertFalse(d.allowed)
        self.assertFalse(d.armed)
        self.assertEqual(d.reason, "output_disabled")
        self.assertEqual(d.velocity, ZERO)

    def test_evaluate_refuses_without_output_enabled(self):
        guard = MotionGuard()
        feed_all_good(guard, t=0.0)
        guard.set_exclusive_control(True)
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "output_disabled")
        self.assertEqual(d.velocity, ZERO)

    def test_guarded_output_never_sends_when_disabled(self):
        """即使输入全部新鲜 + exclusive，output_enabled=False 时 tick 不发任何帧。"""
        guard = MotionGuard(output_enabled=False)
        send = SendRecorder()
        out = GuardedOutput(guard, send)
        feed_all_good(guard, t=0.0, command=Velocity(0.3, 0.0, 0.6))
        guard.set_exclusive_control(True)
        for t in (0.0, 0.1, 0.2):
            d = out.tick(t)
            self.assertFalse(d.allowed)
        self.assertEqual(send.count, 0)  # 连零速都不发


class TestStatusFields(unittest.TestCase):
    """完整状态字段：七个官方字段精确 int 匹配，坏值立即 status_invalid。"""

    def _reject(self, status, msg=None):
        """坏 status 经 evaluate 暴露锁存的 status_invalid（arm 因 exclusive
        已被坏输入清除而返回 disarmed，故用 evaluate 验证锁存原因）。"""
        guard = MotionGuard(output_enabled=True)
        guard.update_status(status, 0.0)
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "status_invalid", msg)
        self.assertFalse(d.allowed)
        self.assertFalse(d.armed)
        self.assertEqual(d.velocity, ZERO)

    def test_all_fields_exact_match_ready(self):
        guard = make_ready_guard(command=Velocity(0.3, 0.0, 0.6), t=0.0)
        d = guard.arm(0.0)
        self.assertEqual(d.reason, "ready")
        self.assertTrue(d.allowed)
        self.assertTrue(d.armed)
        self.assertEqual(d.velocity, Velocity(0.3, 0.0, 0.6))

    def test_missing_motion_state(self):
        self._reject(good_status(MotionState=None))

    def test_missing_control_usage_mode(self):
        self._reject(good_status(ControlUsageMode=None))

    def test_missing_gait(self):
        self._reject(good_status(Gait=None))

    def test_wrong_motion_state_value(self):
        self._reject(good_status(MotionState=18))

    def test_wrong_gait_value(self):
        self._reject(good_status(Gait=4096))

    def test_bool_is_not_int(self):
        """Direction: True 是 bool，不算 int 0。"""
        self._reject(good_status(Direction=True))

    def test_float_is_not_int(self):
        """ControlUsageMode: 1.0 是 float，不算精确 int。"""
        self._reject(good_status(ControlUsageMode=1.0))

    def test_non_dict_status(self):
        self._reject(None)
        self._reject([("MotionState", 17)])

    def test_extra_fields_ignored(self):
        """多余字段（如 StatusCode）不强制，忽略后仍放行。"""
        guard = make_ready_guard(t=0.0)
        guard.update_status(good_status(StatusCode=5), 0.0)  # 未列字段
        self.assertEqual(guard.arm(0.0).reason, "ready")

    def test_invalid_received_at(self):
        for bad in (-0.1, float("nan"), float("inf"), True, "0"):
            guard = MotionGuard(output_enabled=True)
            guard.update_status(good_status(), bad)
            self.assertEqual(guard.evaluate(0.0).reason, "status_invalid",
                             "bad=%r" % (bad,))


class TestInvalidInputsLockReasons(unittest.TestCase):
    """错误输入：立即解除 arm 并锁存对应 invalid/missing，不忽略 bad 保留 good。"""

    def test_bad_scan_replaces_good(self):
        guard = make_ready_guard()
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.update_scan(True, True, float("nan"), 0.0)  # source_age NaN
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "scan_invalid")
        self.assertFalse(d.allowed)
        self.assertEqual(d.velocity, ZERO)

    def test_bad_command_replaces_good(self):
        guard = make_ready_guard()
        self.assertEqual(guard.arm(0.0).reason, "ready")
        for bad in (Velocity(float("nan"), 0.0, 0.0),
                    Velocity(float("inf"), 0.0, 0.0),
                    Velocity("0.3", 0.0, 0.0),
                    Velocity(True, 0.0, 0.0)):
            guard.update_command(bad, 0.0)
            d = guard.evaluate(0.0)
            self.assertEqual(d.reason, "command_invalid", "bad=%r" % (bad,))
            self.assertFalse(d.allowed)

    def test_command_non_velocity_object(self):
        guard = make_ready_guard()
        for bad in (None, (0.3, 0.0, 0.0), {"x": 0.3}):
            guard.update_command(bad, 0.0)
            self.assertEqual(guard.evaluate(0.0).reason, "command_invalid")

    def test_bad_tracking_replaces_good(self):
        guard = make_ready_guard()
        guard.update_tracking("yes", 0.0)  # 非 bool
        self.assertEqual(guard.evaluate(0.0).reason, "tracking_invalid")

    def test_tracking_not_valid(self):
        guard = make_ready_guard()
        guard.update_tracking(False, 0.0)
        self.assertEqual(guard.evaluate(0.0).reason, "tracking_missing")

    def test_scan_not_valid(self):
        guard = make_ready_guard()
        guard.update_scan(False, True, 0.0, 0.0)
        self.assertEqual(guard.evaluate(0.0).reason, "scan_missing")

    def test_scan_not_clear(self):
        """clear=False 视为无可用通行判定，同样不放行。"""
        guard = make_ready_guard()
        guard.update_scan(True, False, 0.0, 0.0)
        self.assertEqual(guard.evaluate(0.0).reason, "scan_missing")

    def test_scan_source_age_out_of_window(self):
        """source_age 必须落在 [-SCAN_FUTURE, SCAN_TTL]，NaN/Inf 拒绝。"""
        for bad in (SCAN_TTL + 0.01, -SCAN_FUTURE - 0.01,
                    float("nan"), float("inf")):
            guard = make_ready_guard()
            guard.update_scan(True, True, bad, 0.0)
            self.assertEqual(guard.evaluate(0.0).reason, "scan_invalid",
                             "source_age=%r" % (bad,))

    def test_scan_inputs_must_be_bool(self):
        for bad in (1, "True", None):
            guard = make_ready_guard()
            guard.update_scan(bad, True, 0.0, 0.0)
            self.assertEqual(guard.evaluate(0.0).reason, "scan_invalid")
            guard = make_ready_guard()
            guard.update_scan(True, bad, 0.0, 0.0)
            self.assertEqual(guard.evaluate(0.0).reason, "scan_invalid")

    def test_missing_initial_reasons_in_order(self):
        """全新 guard 按喂入顺序逐项暴露 missing：status→scan→tracking→command。"""
        guard = MotionGuard(output_enabled=True)
        guard.set_exclusive_control(True)
        self.assertEqual(guard.arm(0.0).reason, "status_missing")
        guard.update_status(good_status(), 0.0)
        self.assertEqual(rearm(guard, 0.0).reason, "scan_missing")
        guard.update_scan(True, True, 0.0, 0.0)
        self.assertEqual(rearm(guard, 0.0).reason, "tracking_missing")
        guard.update_tracking(True, 0.0)
        self.assertEqual(rearm(guard, 0.0).reason, "command_missing")
        guard.update_command(ZERO, 0.0)
        self.assertEqual(rearm(guard, 0.0).reason, "ready")


class TestTTLExpiry(unittest.TestCase):
    """scan / tracking / command / status TTL 过期判定（单调钟，含边界）。"""

    def test_command_ttl_boundary(self):
        command = Velocity(0.3, 0.0, 0.6)
        guard = make_ready_guard(command=command, t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        # 恰好 CMD_TTL 时仍新鲜（> TTL 才算过期）
        d = guard.evaluate(CMD_TTL)
        self.assertEqual(d.reason, "ready")
        self.assertEqual(d.velocity, Velocity(0.3, 0.0, 0.6))
        # 超过 CMD_TTL 一点 → command_stale，输出清零
        d = guard.evaluate(CMD_TTL + 0.0001)
        self.assertEqual(d.reason, "command_stale")
        self.assertEqual(d.velocity, ZERO)

    def test_command_future_received_at(self):
        guard = MotionGuard(output_enabled=True)
        guard.update_status(good_status(), 0.0)
        guard.update_scan(True, True, 0.0, 0.0)
        guard.update_tracking(True, 0.0)
        guard.update_command(ZERO, 0.1)  # 相对 now=0 的未来时间戳
        guard.set_exclusive_control(True)
        self.assertEqual(guard.arm(0.0).reason, "command_stale")

    def test_status_ttl_boundary(self):
        guard = make_ready_guard(t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.update_command(ZERO, STATUS_TTL)  # 命令流持续，保持 command 新鲜
        d = guard.evaluate(STATUS_TTL)  # 恰好 STATUS_TTL 仍新鲜
        self.assertEqual(d.reason, "ready")
        d = guard.evaluate(STATUS_TTL + 0.0001)
        self.assertEqual(d.reason, "status_stale")
        self.assertEqual(d.velocity, ZERO)

    def test_status_future_received_at(self):
        guard = make_ready_guard(t=0.0)
        guard.update_status(good_status(), 0.1)  # 未来
        self.assertEqual(guard.evaluate(0.0).reason, "status_stale")

    def test_tracking_ttl_boundary(self):
        guard = make_ready_guard(t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        # 其余输入同步刷新，隔离 tracking 自身的 TTL 边界
        guard.update_command(ZERO, TRACKING_TTL)
        guard.update_status(good_status(), TRACKING_TTL)
        guard.update_scan(True, True, 0.0, TRACKING_TTL)
        d = guard.evaluate(TRACKING_TTL)  # 恰好 TRACKING_TTL 仍新鲜
        self.assertEqual(d.reason, "ready")
        d = guard.evaluate(TRACKING_TTL + 0.0001)
        self.assertEqual(d.reason, "tracking_stale")
        self.assertEqual(d.velocity, ZERO)

    def test_scan_ttl_includes_dwell(self):
        """source_age + 停留时间 ≤ SCAN_TTL，防缓存续命。"""
        guard = MotionGuard(output_enabled=True)
        feed_all_good(guard, t=0.0, command=ZERO)
        guard.update_scan(True, True, 0.4, 0.0)  # 收到时已 0.4
        guard.set_exclusive_control(True)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        d = guard.evaluate(0.09)  # 0.4 + 0.09 = 0.49 ≤ 0.5
        self.assertEqual(d.reason, "ready")
        d = guard.evaluate(0.1)  # 恰好 0.5，仍新鲜
        self.assertEqual(d.reason, "ready")
        d = guard.evaluate(0.11)  # 0.51 > 0.5
        self.assertEqual(d.reason, "scan_stale")
        self.assertEqual(d.velocity, ZERO)

    def test_scan_ttl_no_renewal_by_other_updates(self):
        """不重发 scan，后续 update/evaluate 不能给 scan 续命。"""
        guard = make_ready_guard(t=0.0)
        guard.update_tracking(True, 0.5)
        guard.update_command(ZERO, 0.5)
        guard.update_status(good_status(), 0.5)
        self.assertEqual(guard.arm(0.5).reason, "ready")  # scan 恰好 0.5 新鲜
        d = guard.evaluate(0.51)
        self.assertEqual(d.reason, "scan_stale")

    def test_scan_future_received_at(self):
        guard = make_ready_guard(t=0.0)
        guard.update_scan(True, True, 0.0, 0.1)  # received_at 未来
        self.assertEqual(guard.evaluate(0.0).reason, "scan_stale")

    def test_stale_then_refresh_still_blocked_until_rearm(self):
        """过期本身解除 arm：刷新数据后 evaluate 仍为零，需显式 re-arm。"""
        command = Velocity(0.3, 0.0, 0.6)
        guard = make_ready_guard(command=command, t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        self.assertEqual(guard.evaluate(CMD_TTL + 0.1).reason, "command_stale")
        guard.update_command(command, CMD_TTL + 0.1)
        d = guard.evaluate(CMD_TTL + 0.1)
        self.assertFalse(d.allowed)
        self.assertEqual(d.reason, "disarmed")
        self.assertEqual(d.velocity, ZERO)
        d = rearm(guard, CMD_TTL + 0.1)  # 重新确认独占 + 显式 arm
        self.assertEqual(d.reason, "ready")
        self.assertEqual(d.velocity, command)


class TestClockRollback(unittest.TestCase):
    """单调钟：非法（非 finite / bool / 负数）或回拨 → clock_invalid 并 disarm。"""

    def test_rollback_detected(self):
        guard = make_ready_guard(t=0.0)
        feed_all_good(guard, t=1.0, command=Velocity(0.3, 0.0, 0.6))  # 保持新鲜
        self.assertEqual(guard.arm(1.0).reason, "ready")
        self.assertEqual(guard.evaluate(1.0).reason, "ready")
        d = guard.evaluate(0.999)  # 回拨
        self.assertEqual(d.reason, "clock_invalid")
        self.assertFalse(d.allowed)
        self.assertEqual(d.velocity, ZERO)

    def test_rollback_via_arm(self):
        guard = make_ready_guard(t=0.0)
        feed_all_good(guard, t=1.0, command=Velocity(0.3, 0.0, 0.6))
        self.assertEqual(guard.arm(1.0).reason, "ready")
        self.assertEqual(guard.arm(0.5).reason, "clock_invalid")

    def test_rollback_disarms_ready_state(self):
        guard = make_ready_guard(t=0.0)
        feed_all_good(guard, t=1.0, command=Velocity(0.3, 0.0, 0.6))
        self.assertEqual(guard.arm(1.0).reason, "ready")
        self.assertEqual(guard.arm(0.5).reason, "clock_invalid")
        # 恢复到合法（更晚）时间也不自动恢复
        d = guard.evaluate(1.1)
        self.assertEqual(d.reason, "disarmed")
        self.assertFalse(d.allowed)

    def test_invalid_now_values(self):
        for bad in (-0.001, float("nan"), float("inf"), True, "1.0"):
            guard = make_ready_guard(t=0.0)
            d = guard.evaluate(bad)
            self.assertEqual(d.reason, "clock_invalid", "now=%r" % (bad,))
            self.assertFalse(d.allowed)

    def test_timestamp_zero_is_legal(self):
        """timestamps == 0 是合法的单调钟测试值。"""
        guard = make_ready_guard(command=Velocity(0.3, 0.0, 0.6), t=0.0)
        d = guard.arm(0.0)
        self.assertEqual(d.reason, "ready")
        self.assertEqual(d.velocity, Velocity(0.3, 0.0, 0.6))

    def test_repeated_same_now_is_not_rollback(self):
        guard = make_ready_guard(t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        for _ in range(3):
            self.assertEqual(guard.evaluate(0.0).reason, "ready")


class TestExclusiveAndExplicitArm(unittest.TestCase):
    """exclusive 软条件 + 显式 arm：无独占不放行，arm 后还需持续新鲜。"""

    def test_no_exclusive_refuses_arm(self):
        guard = MotionGuard(output_enabled=True)
        feed_all_good(guard, t=0.0, command=Velocity(0.3, 0.0, 0.6))
        d = guard.arm(0.0)  # 未 set_exclusive_control(True)
        self.assertEqual(d.reason, "disarmed")
        self.assertFalse(d.allowed)
        self.assertFalse(d.armed)
        self.assertEqual(d.velocity, ZERO)

    def test_explicit_arm_grants(self):
        guard = make_ready_guard(command=Velocity(0.3, 0.0, 0.6), t=0.0)
        d = guard.arm(0.0)
        self.assertEqual(d.reason, "ready")
        self.assertTrue(d.armed)
        self.assertTrue(d.allowed)
        self.assertEqual(d.velocity, Velocity(0.3, 0.0, 0.6))

    def test_re_exclusive_requires_rearm(self):
        """set_exclusive_control(False) 解除 arm；重新 True 也不自动恢复。"""
        guard = make_ready_guard(t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.set_exclusive_control(False)
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "disarmed")
        guard.set_exclusive_control(True)  # 人工重新确认独占
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "disarmed")  # 不自动恢复
        self.assertEqual(guard.arm(0.0).reason, "ready")  # 必须显式 arm

    def test_non_bool_exclusive_treated_as_false(self):
        guard = make_ready_guard(t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.set_exclusive_control(1)  # 非 bool → 按未确认处理
        self.assertEqual(guard.evaluate(0.0).reason, "disarmed")

    def test_explicit_disarm_sticks(self):
        guard = make_ready_guard(t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        self.assertIsNone(guard.disarm())
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "disarmed")
        self.assertFalse(d.allowed)
        # 数据仍全部新鲜，也绝不自动恢复
        self.assertEqual(guard.evaluate(0.0).reason, "disarmed")

    def test_evaluate_never_arms(self):
        """evaluate 只维持已 arm 状态，绝不主动放行未 arm 的 guard。"""
        guard = MotionGuard(output_enabled=True)
        feed_all_good(guard, t=0.0, command=Velocity(0.3, 0.0, 0.6))
        guard.set_exclusive_control(True)
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "disarmed")
        self.assertFalse(d.armed)
        # arm 后 evaluate 才放行
        self.assertEqual(guard.arm(0.0).reason, "ready")
        self.assertEqual(guard.evaluate(0.0).reason, "ready")


class TestNoAutoRecovery(unittest.TestCase):
    """故障不自动恢复：恢复数据后 evaluate 仍为零，必须再次显式 arm()。"""

    def test_recovery_after_bad_input_no_auto_arm(self):
        command = Velocity(0.3, 0.0, 0.6)
        guard = make_ready_guard(command=command, t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.update_tracking(False, 0.0)  # 故障：tracking 失效
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "tracking_missing")
        self.assertEqual(d.velocity, ZERO)
        # 数据恢复（合法输入重发），仍不自动放行
        guard.update_tracking(True, 0.0)
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "disarmed")
        self.assertFalse(d.allowed)
        self.assertEqual(d.velocity, ZERO)
        # 显式 re-arm（重新确认独占）后恢复
        d = rearm(guard, 0.0)
        self.assertEqual(d.reason, "ready")
        self.assertEqual(d.velocity, command)

    def test_rearm_requires_reconfirm_exclusive_after_block(self):
        """阻塞路径解除 arm 同时清除 exclusive：重新放行需再次人工确认独占。"""
        command = Velocity(0.3, 0.0, 0.6)
        guard = make_ready_guard(command=command, t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.update_command(Velocity(float("nan"), 0.0, 0.0), 0.0)  # 坏输入
        guard.update_command(command, 0.0)  # 数据恢复
        self.assertEqual(guard.arm(0.0).reason, "disarmed")  # exclusive 已清除
        d = rearm(guard, 0.0)
        self.assertEqual(d.reason, "ready")

    def test_recovery_after_explicit_disarm(self):
        """disarm 兜底后，数据即使全部新鲜也绝不自动恢复。"""
        command = Velocity(0.3, 0.0, 0.6)
        guard = make_ready_guard(command=command, t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.disarm()
        feed_all_good(guard, t=0.2, command=command)  # 全部新鲜重发
        d = guard.evaluate(0.2)
        self.assertEqual(d.reason, "disarmed")
        self.assertFalse(d.allowed)
        d = rearm(guard, 0.2)  # 只能显式 arm
        self.assertEqual(d.reason, "ready")
        self.assertEqual(d.velocity, command)

    def test_no_recovery_while_any_input_still_bad(self):
        guard = make_ready_guard(t=0.0)
        self.assertEqual(guard.arm(0.0).reason, "ready")
        guard.update_status(good_status(MotionState=18), 0.0)
        guard.update_tracking(True, 0.0)  # 其余恢复不改变 status 锁存
        self.assertEqual(guard.evaluate(0.0).reason, "status_invalid")


class TestClampAndDeadzone(unittest.TestCase):
    """限幅先 clamp 再死区，绝不提高速度，不取绝对值代替符号；y 恒 0。"""

    def _limited(self, x, y=0.0, yaw=0.0):
        guard = make_ready_guard(t=0.0)
        guard.update_command(Velocity(x, y, yaw), 0.0)
        return guard.arm(0.0)

    def test_x_clamp_upper(self):
        self.assertEqual(self._limited(1.0).velocity.x, MAX_VX)
        self.assertEqual(self._limited(0.35).velocity.x, MAX_VX)

    def test_x_forbid_reverse(self):
        """x 禁后退：负值 clamp 到 0，绝不取绝对值放大。"""
        self.assertEqual(self._limited(-0.5).velocity.x, 0.0)
        self.assertEqual(self._limited(-1.0).velocity.x, 0.0)

    def test_x_deadzone(self):
        self.assertEqual(self._limited(X_DEADZONE - 0.01).velocity.x, 0.0)
        self.assertEqual(self._limited(0.05).velocity.x, 0.0)

    def test_x_pass_through(self):
        self.assertEqual(self._limited(X_DEADZONE).velocity.x, X_DEADZONE)
        self.assertEqual(self._limited(0.25).velocity.x, 0.25)
        self.assertEqual(self._limited(MAX_VX).velocity.x, MAX_VX)

    def test_yaw_clamp(self):
        self.assertEqual(self._limited(0.0, 0.0, 1.0).velocity.yaw, MAX_YAW)
        self.assertEqual(self._limited(0.0, 0.0, -1.0).velocity.yaw, -MAX_YAW)

    def test_yaw_deadzone_keeps_sign(self):
        self.assertEqual(self._limited(0.0, 0.0, YAW_DEADZONE - 0.01).velocity.yaw, 0.0)
        self.assertEqual(self._limited(0.0, 0.0, -(YAW_DEADZONE - 0.01)).velocity.yaw, 0.0)
        self.assertEqual(self._limited(0.0, 0.0, 0.5).velocity.yaw, 0.5)
        self.assertEqual(self._limited(0.0, 0.0, -0.5).velocity.yaw, -0.5)

    def test_y_always_zero(self):
        self.assertEqual(self._limited(0.3, 0.5, 0.0).velocity.y, 0.0)
        self.assertEqual(self._limited(0.3, -0.35, 0.0).velocity.y, 0.0)

    def test_zero_command_ready_but_zero(self):
        d = self._limited(0.0, 0.0, 0.0)
        self.assertEqual(d.reason, "ready")
        self.assertEqual(d.velocity, ZERO)

    def test_combined_limits(self):
        d = self._limited(0.9, 0.4, -0.8)
        self.assertEqual(d.velocity, Velocity(0.3, 0.0, -0.6))


class TestGuardedOutputSendBoundary(unittest.TestCase):
    """GuardedOutput：放行才 send；blocked 只补发一次零速；异常传播并 disarm。"""

    COMMAND = Velocity(0.3, 0.0, 0.6)

    def _make(self):
        guard = make_ready_guard(command=self.COMMAND, t=0.0)
        send = SendRecorder()
        return guard, GuardedOutput(guard, send), send

    def test_sends_only_after_allowed(self):
        """从未放行过：blocked tick 不发任何东西（含零速）。"""
        guard, out, send = self._make()
        guard.disarm()
        d = out.tick(0.0)
        self.assertEqual(d.reason, "disarmed")
        self.assertEqual(send.count, 0)
        # 放行后发送限幅指令
        self.assertEqual(rearm(guard, 0.0).reason, "ready")
        d = out.tick(0.0)
        self.assertTrue(d.allowed)
        self.assertEqual(send.velocities, [(0.3, 0.0, 0.6)])

    def test_blocked_sends_exactly_one_zero(self):
        """曾放行后转入 blocked：只补发一次零速，之后不再重复发零速。"""
        guard, out, send = self._make()
        self.assertEqual(guard.arm(0.0).reason, "ready")
        out.tick(0.0)
        self.assertEqual(send.velocities, [(0.3, 0.0, 0.6)])
        # 转入 blocked：补发一次零速
        guard.disarm()
        out.tick(0.0)
        self.assertEqual(send.velocities,
                         [(0.3, 0.0, 0.6), (0.0, 0.0, 0.0)])
        # 继续 blocked：不再补发
        out.tick(0.0)
        out.tick(0.0)
        self.assertEqual(send.count, 2)

    def test_blocked_by_staleness_sends_one_zero(self):
        """超时转 blocked 同样只补发一次零速。"""
        guard, out, send = self._make()
        self.assertEqual(guard.arm(0.0).reason, "ready")
        out.tick(0.0)
        out.tick(CMD_TTL + 0.1)  # command_stale
        self.assertEqual(send.count, 2)
        self.assertEqual(send.calls[-1], ZERO)
        out.tick(CMD_TTL + 0.2)
        self.assertEqual(send.count, 2)  # 不重复

    def test_send_exception_propagates_and_disarms(self):
        """send 异常：原样传播给调用方，guard.disarm()（清 exclusive）。"""
        guard, _, _ = self._make()

        class Boom(object):
            def __call__(self, velocity):
                raise RuntimeError("transport down")

        out = GuardedOutput(guard, Boom())
        self.assertEqual(guard.arm(0.0).reason, "ready")
        with self.assertRaises(RuntimeError) as ctx:
            out.tick(0.0)
        self.assertEqual(str(ctx.exception), "transport down")
        # 异常路径解除 arm 且清除 exclusive → 不吞错伪成功
        d = guard.evaluate(0.0)
        self.assertEqual(d.reason, "disarmed")
        self.assertFalse(d.allowed)
        self.assertEqual(guard.arm(0.0).reason, "disarmed")  # exclusive 已清除

    def test_recovery_after_send_exception_requires_reconfirm(self):
        """send 异常 disarm 后：恢复数据 + 重新确认独占 + arm 才能再放行。"""
        guard, _, _ = self._make()
        attempts = []

        class Flaky(object):
            def __call__(self, velocity):
                attempts.append(velocity)
                if len(attempts) == 1:
                    raise RuntimeError("down")
                # 第二次成功

        out = GuardedOutput(guard, Flaky())
        self.assertEqual(guard.arm(0.0).reason, "ready")
        with self.assertRaises(RuntimeError):
            out.tick(0.0)
        self.assertEqual(guard.arm(0.0).reason, "disarmed")  # 不自动恢复
        feed_all_good(guard, t=0.0, command=self.COMMAND)  # 数据恢复
        d = rearm(guard, 0.0)
        self.assertEqual(d.reason, "ready")
        d = out.tick(0.0)
        self.assertTrue(d.allowed)
        self.assertEqual(attempts, [self.COMMAND, self.COMMAND])

    def test_zero_send_exception_propagates(self):
        """补发零速的 send 异常同样传播并 disarm，不吞。"""
        guard, _, _ = self._make()
        seen = []

        class Flaky(object):
            def __call__(self, velocity):
                seen.append(velocity)
                if len(seen) >= 2:  # 第一次放行帧成功，第二次（零速）失败
                    raise RuntimeError("zero send failed")

        out = GuardedOutput(guard, Flaky())
        self.assertEqual(guard.arm(0.0).reason, "ready")
        out.tick(0.0)
        with self.assertRaises(RuntimeError):
            out.tick(CMD_TTL + 0.1)  # blocked → 补零速 → send 抛异常
        # 异常路径 disarm 清除 exclusive → 即使数据尚新鲜（arm 会走 exclusive
        # 检查）也只能 disarmed，证明异常确实解除并锁存
        self.assertEqual(guard.arm(CMD_TTL + 0.1).reason, "disarmed")
        self.assertEqual(seen, [self.COMMAND, ZERO])

    def test_tick_returns_decision(self):
        guard, out, send = self._make()
        d = out.tick(0.0)
        self.assertIsInstance(d, Decision)
        self.assertEqual(d.reason, "disarmed")  # 未显式 arm
        self.assertEqual(send.count, 0)

    def test_guarded_output_type_checks(self):
        guard = MotionGuard()
        with self.assertRaises(TypeError):
            GuardedOutput(None, lambda v: None)
        with self.assertRaises(TypeError):
            GuardedOutput(guard, "not-callable")


if __name__ == "__main__":
    unittest.main()
