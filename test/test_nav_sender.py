#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NAVCMD-SENDER-TEST: NavCmdSender /NAV_CMD 发送边界单测（纯内存替身，无 ROS 运行时）。

被测包: /home/user/new_chase/motion_control（nav_sender.py + core.py）
约定来源: motion_control/NAV_SENDER.md 合同 + NAVCMD-SENDER-TEST 任务说明:
- 默认 enabled=False：不 import drdds、不访问 node、不创建 publisher、不发送
  任何消息（连零速都不发）；__call__ 恒返回 False；
- enabled 必须精确 bool（1 / "true" / None 均 TypeError）；message_type 必须是
  消息类或 None；
- topic 固定 /NAV_CMD；publisher 懒创建：首次 enabled 发送才创建（depth=1），
  构造时不创建；
- 消息映射：data.x_vel / y_vel / yaw_vel（SI 单位，y 恒 0）；header.frame_id
  为 uint64 从 0 递增、到顶回绕；header.stamp 取 node.get_clock().now().to_msg()
  （节点时钟，非墙钟）；
- 独立再次限幅：x ∈ [0, 0.3]、y = 0、yaw ∈ [-0.6, 0.6]；
  |x| < 0.2 → 0、|yaw| < 0.5 → 0；死区内置零，绝不向上抬；
- 非法输入（bool / NaN / Inf / 溢出 int / str / None / 非 Velocity 对象）
  → ValueError 且 fault 锁存，后续调用拒绝（RuntimeError），直到重新实例化；
- publisher 创建失败（含 RuntimeError / ValueError 等任意异常类型）、实际
  topic 不符、publish 失败 → 原异常传播且 fault 锁存，不重试、不复活；
- close 幂等：只销毁自身 publisher，不 shutdown node，不补发零速；
  close 后 enabled sender 拒绝发送；
- GuardedOutput 组合：默认（output_enabled=False / 未人工 arm）绝不发送；
  send 异常 → guard.disarm（清 exclusive，重新放行需再次人工确认独占）。

运行（离线，不启动 ROS / DDS / 任何真实 publisher；-B 避免 __pycache__）:
  python3 -B -m unittest discover -s /home/user/new_chase/test -p test_nav_sender.py -v

真实 drdds.msg.NavCmd 仅用于消息类型 / 字段验证（不 rclpy.init、不建节点、
不产生 DDS 流量）；未 source Foxy 环境时相关用例**显式 skip**（skip 原因
写明需 source /opt/ros/foxy/setup.bash），绝不静默假通过。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motion_control.core import (
    MAX_VX,
    MAX_YAW,
    X_DEADZONE,
    YAW_DEADZONE,
    GuardedOutput,
    MotionGuard,
    Velocity,
)
from motion_control.nav_sender import NavCmdSender

# ---- 真实消息类型（仅 import 类做字段验证；无 rclpy.init / 无节点 / 无 DDS）----
try:
    from drdds.msg import NavCmd as RealNavCmd
    _HAVE_REAL_NAVCMD = True
except Exception:
    RealNavCmd = None
    _HAVE_REAL_NAVCMD = False

try:
    from builtin_interfaces.msg import Time as RosTime
    _HAVE_ROSTIME = True
except Exception:
    RosTime = None
    _HAVE_ROSTIME = False

_REAL_SKIP_REASON = (
    "drdds.msg.NavCmd 不可用：需在 GOS-104 上 source /opt/ros/foxy/setup.bash "
    "后运行真实消息类型用例；本 skip 为显式环境原因，非静默跳过"
)


# ==================== 测试替身（纯内存，无 ROS / 无网络） ====================

class FakeMsg:
    """最小 NavCmd 替身：data.x_vel/y_vel/yaw_vel + header.frame_id/stamp。"""

    class _Data(object):
        def __init__(self):
            self.x_vel = None
            self.y_vel = None
            self.yaw_vel = None

    class _Header(object):
        def __init__(self):
            self.frame_id = None
            self.stamp = None

    def __init__(self):
        self.data = FakeMsg._Data()
        self.header = FakeMsg._Header()


class FakePublisher:
    def __init__(self, topic_name="/NAV_CMD"):
        self.topic_name = topic_name
        self.published = []
        self.publish_calls = 0
        self.publish_error = None

    def publish(self, msg):
        self.publish_calls += 1
        if self.publish_error is not None:
            raise self.publish_error
        self.published.append(msg)


class _FakeTime(object):
    def __init__(self, stamp):
        self._stamp = stamp

    def to_msg(self):
        return self._stamp


class FakeClock:
    def __init__(self, stamp=None):
        self.stamp = stamp

    def now(self):
        return _FakeTime(self.stamp)


class FakeNode:
    def __init__(self, publisher=None, create_error=None, stamp=None):
        self.pub = publisher if publisher is not None else FakePublisher()
        self.create_error = create_error
        self.created = []       # [(msg_type, topic, depth, kwargs), ...]
        self.destroyed = []
        self.shutdown_calls = 0
        self.clock = FakeClock(stamp)

    def create_publisher(self, msg_type, topic, depth, **kwargs):
        if self.create_error is not None:
            raise self.create_error
        self.created.append((msg_type, topic, depth, kwargs))
        return self.pub

    def destroy_publisher(self, pub):
        self.destroyed.append(pub)

    def get_clock(self):
        return self.clock

    def shutdown(self):
        self.shutdown_calls += 1  # 合同：sender 绝不 shutdown 调用方 node


class PoisonNode(object):
    """任何属性访问即抛 AssertionError 的毒化 node，用于证明绝不触碰。"""

    def __getattr__(self, name):
        raise AssertionError("PoisonNode 被触碰: %s" % (name,))


def make_sender(node, **overrides):
    """enabled 替身 sender：默认用 FakeMsg，绝不触发 drdds 惰性 import。"""
    kwargs = {"node": node, "enabled": True, "message_type": FakeMsg}
    kwargs.update(overrides)
    return NavCmdSender(**kwargs)


def good_status():
    """七个官方字段的合法 BasicStatus（int 精确值），与 test_motion_guard 一致。"""
    return {
        "MotionState": 17,
        "ControlUsageMode": 1,
        "Direction": 0,
        "HES": 0,
        "Charge": 0,
        "Sleep": 0,
        "Gait": 4097,
    }


# ==================== 构造 / 默认禁用替身 ====================

class TestConstructionAndDisabled(unittest.TestCase):

    def test_disabled_default_lazy_no_untoward(self):
        """enabled=False：恒 False、无 fault、close 幂等、绝不 import drdds。"""
        saved = {k: v for k, v in sys.modules.items()
                 if k == "drdds" or k.startswith("drdds.")}
        for k in saved:
            del sys.modules[k]
        try:
            s = NavCmdSender(node=None, enabled=False)
            self.assertIs(s.enabled, False)
            self.assertEqual(s.topic, "/NAV_CMD")
            self.assertIsNone(s.fault)
            self.assertIs(s.closed, False)
            self.assertIs(s(Velocity(x=0.25, y=0.1, yaw=0.55)), False)
            self.assertIsNone(s.fault)
            s.close()
            s.close()  # 幂等
            reimported = [k for k in ("drdds", "drdds.msg") if k in sys.modules]
            self.assertEqual(
                reimported, [],
                "disabled sender 绝不允许 import drdds（含任何触发路径）")
        finally:
            sys.modules.update(saved)

    def test_disabled_poison_node_never_touched(self):
        """disabled 时传入毒化 node：任何操作都不触碰 node（含 close）。"""
        poison = PoisonNode()
        s = NavCmdSender(node=poison, enabled=False)
        self.assertIs(s(Velocity(x=0.25)), False)
        self.assertIs(s(Velocity(x=0.0)), False)
        self.assertIsNone(s.fault)
        s.close()
        s.close()

    def test_exact_bool_and_message_type_checks(self):
        """enabled 必须精确 bool；message_type 必须是消息类或 None。"""
        node = FakeNode()
        for bad in (1, 0, "true", None, 1.0):
            with self.assertRaises(TypeError, msg="enabled=%r" % (bad,)):
                NavCmdSender(node=node, enabled=bad)
        for bad in ("NavCmd", 42, object()):
            with self.assertRaises(TypeError, msg="message_type=%r" % (bad,)):
                NavCmdSender(node=node, enabled=True, message_type=bad)
        self.assertEqual(node.created, [], "TypeError 路径绝不创建 publisher")
        # enabled=True + node=None 允许构造；错误推迟到首次发送（懒创建）
        lazy = NavCmdSender(node=None, enabled=True)
        with self.assertRaises(RuntimeError):
            lazy(Velocity(x=0.25))
        self.assertIsNotNone(lazy.fault)


# ==================== 懒创建 publisher / topic / 创建故障 ====================

class TestLazyPublisher(unittest.TestCase):

    def test_no_pub_created_before_first_send(self):
        node = FakeNode()
        s = make_sender(node)
        self.assertEqual(node.created, [], "构造后绝不创建 publisher")
        self.assertTrue(s(Velocity(x=0.25)))
        self.assertEqual(len(node.created), 1)
        self.assertTrue(s(Velocity(x=0.25)))
        self.assertEqual(len(node.created), 1, "publisher 必须复用，不重复创建")

    def test_fixed_topic_depth_no_topic_parameter(self):
        node = FakeNode()
        s = make_sender(node)
        s(Velocity(x=0.25))
        self.assertEqual(len(node.created), 1)
        msg_type, topic, depth, kwargs = node.created[0]
        self.assertEqual(topic, "/NAV_CMD", "topic 固定 /NAV_CMD")
        self.assertEqual(depth, 1, "depth 固定 1")
        self.assertNotIn("topic", kwargs, "不接受 topic 参数 / remap")
        self.assertIs(msg_type, FakeMsg)

    def test_topic_mismatch_destroys_and_never_publishes(self):
        node = FakeNode(publisher=FakePublisher(topic_name="/OTHER"))
        s = make_sender(node)
        with self.assertRaises(RuntimeError):
            s(Velocity(x=0.25))
        self.assertEqual(len(node.destroyed), 1, "topic 不符的 publisher 必须销毁")
        self.assertIs(node.destroyed[0], node.pub)
        self.assertEqual(node.pub.published, [], "错误 topic 绝不发布")
        self.assertTrue(str(s.fault).startswith("topic_mismatch"))
        # 锁存：后续调用拒绝，且不再创建新 publisher
        with self.assertRaises(RuntimeError):
            s(Velocity(x=0.25))
        self.assertEqual(len(node.created), 1)

    def test_creation_runtime_error_latched(self):
        node = FakeNode(create_error=RuntimeError("create boom"))
        s = make_sender(node)
        with self.assertRaises(RuntimeError):
            s(Velocity(x=0.25))
        self.assertTrue(str(s.fault).startswith("publisher_failed"))
        with self.assertRaises(RuntimeError, msg="创建失败后锁存，后续拒绝"):
            s(Velocity(x=0.25))
        self.assertEqual(node.pub.published, [])

    def test_creation_value_error_latched(self):
        """合同：创建失败无论异常类型（ValueError / RuntimeError / ...）
        都必须锁存 fault；后续调用一律拒绝（RuntimeError），不得重试。"""
        node = FakeNode(create_error=ValueError("create boom"))
        s = make_sender(node)
        with self.assertRaises(ValueError):
            s(Velocity(x=0.25))
        with self.assertRaises(RuntimeError,
                               msg="ValueError 创建失败也必须锁存，后续拒绝"):
            s(Velocity(x=0.25))
        self.assertIsNotNone(s.fault)


# ==================== 消息内容（替身消息） ====================

class TestMessageContent(unittest.TestCase):

    def test_mapping_si_units_and_sequence(self):
        node = FakeNode()
        s = make_sender(node)
        self.assertTrue(s(Velocity(x=0.25, y=0.3, yaw=0.55)))
        self.assertTrue(s(Velocity(x=0.25, y=0.3, yaw=0.55)))
        m0, m1 = node.pub.published
        self.assertAlmostEqual(m0.data.x_vel, 0.25)
        self.assertEqual(m0.data.y_vel, 0.0, "y 恒为 0（basic 步态横移禁用）")
        self.assertAlmostEqual(m0.data.yaw_vel, 0.55)
        self.assertEqual(m0.header.frame_id, 0, "frame_id uint64 从 0 递增")
        self.assertEqual(m1.header.frame_id, 1)
        self.assertIs(m0.header.stamp, node.clock.stamp,
                      "stamp 必须取 node.get_clock().now().to_msg()")
        self.assertIs(m1.header.stamp, node.clock.stamp)

    def test_sequence_wrap_uint64(self):
        """frame_id 到 2^64-1 后必须回绕到 0（直接置于边界，真实跑 2^64 条不现实）。"""
        node = FakeNode()
        s = make_sender(node)
        s._frame_id = (1 << 64) - 1
        s(Velocity(x=0.25))
        s(Velocity(x=0.25))
        m0, m1 = node.pub.published
        self.assertEqual(m0.header.frame_id, (1 << 64) - 1)
        self.assertEqual(m1.header.frame_id, 0, "uint64 到顶必须回绕到 0")

    def test_limits_and_deadzones(self):
        """独立再次限幅：先 clamp 再死区，y 恒 0，绝不向上抬。"""
        node = FakeNode()
        s = make_sender(node)
        cases = [
            (Velocity(x=-1.0, yaw=-0.8), (0.0, 0.0, -MAX_YAW)),
            (Velocity(x=0.0, yaw=0.0), (0.0, 0.0, 0.0)),
            (Velocity(x=X_DEADZONE - 0.01, yaw=YAW_DEADZONE - 0.01),
             (0.0, 0.0, 0.0)),
            (Velocity(x=X_DEADZONE, yaw=YAW_DEADZONE),
             (X_DEADZONE, 0.0, YAW_DEADZONE)),
            (Velocity(x=0.25, y=0.3, yaw=0.55), (0.25, 0.0, 0.55)),
            (Velocity(x=0.5, yaw=0.8), (MAX_VX, 0.0, MAX_YAW)),
            (Velocity(x=MAX_VX, yaw=-MAX_YAW), (MAX_VX, 0.0, -MAX_YAW)),
            (Velocity(x=1e9, yaw=-1e9), (MAX_VX, 0.0, -MAX_YAW)),
        ]
        for v, expected in cases:
            with self.subTest(v=(v.x, v.y, v.yaw)):
                self.assertTrue(s(v))
                m = node.pub.published[-1]
                got = (m.data.x_vel, m.data.y_vel, m.data.yaw_vel)
                self.assertEqual(got, expected,
                                 "v=%r got=%r want=%r" % (v, got, expected))


# ==================== 真实 NavCmd 类型（skip 须明示） ====================

@unittest.skipUnless(_HAVE_REAL_NAVCMD, _REAL_SKIP_REASON)
class TestRealNavCmd(unittest.TestCase):

    @unittest.skipUnless(_HAVE_REAL_NAVCMD and _HAVE_ROSTIME, _REAL_SKIP_REASON)
    def test_mapping_stamp_frame_id_float32(self):
        stamp = RosTime()
        stamp.sec = 7
        stamp.nanosec = 9
        node = FakeNode(stamp=stamp)
        s = NavCmdSender(node=node, enabled=True, message_type=RealNavCmd)
        self.assertTrue(s(Velocity(x=0.25, y=0.3, yaw=0.55)))
        self.assertTrue(s(Velocity(x=0.25, y=0.3, yaw=0.55)))
        m0, m1 = node.pub.published
        self.assertIsInstance(m0, RealNavCmd)
        self.assertIsInstance(m1, RealNavCmd)
        self.assertAlmostEqual(m0.data.x_vel, 0.25, places=6)
        self.assertEqual(m0.data.y_vel, 0.0)
        self.assertAlmostEqual(m0.data.yaw_vel, 0.55, places=6)
        self.assertEqual(m0.header.frame_id, 0)
        self.assertEqual(m1.header.frame_id, 1)
        self.assertEqual(m0.header.stamp, stamp, "stamp 必须来自 node 时钟")
        self.assertEqual(m1.header.stamp, stamp)

    def test_lazy_import_real_type(self):
        """message_type=None：首次发送才惰性 import 真实 NavCmd 并创建 pub。"""
        stamp = RosTime()
        stamp.sec = 1
        stamp.nanosec = 2
        node = FakeNode(stamp=stamp)
        s = NavCmdSender(node=node, enabled=True, message_type=None)
        self.assertTrue(s(Velocity(x=0.25)))
        self.assertEqual(len(node.created), 1)
        msg_type, topic, depth, _ = node.created[0]
        self.assertIs(msg_type, RealNavCmd,
                      "message_type=None 必须惰性 import 真实 NavCmd")
        self.assertEqual(topic, "/NAV_CMD")
        self.assertEqual(depth, 1)
        self.assertIsInstance(node.pub.published[0], RealNavCmd)


# ==================== 非法输入与锁存 ====================

class TestInvalidInput(unittest.TestCase):

    @staticmethod
    def _bad_values():
        return [
            ("nan", Velocity(x=float("nan"), y=0.0, yaw=0.0)),
            ("inf", Velocity(x=0.25, y=float("inf"), yaw=0.0)),
            ("-inf", Velocity(x=0.25, y=0.0, yaw=float("-inf"))),
            ("overflow_int_10^400", Velocity(x=10 ** 400, y=0.0, yaw=0.0)),
            ("bool_x", Velocity(x=True, y=0.0, yaw=0.0)),
            ("bool_y", Velocity(x=0.25, y=False, yaw=0.0)),
            ("str_yaw", Velocity(x=0.25, y=0.0, yaw="0.5")),
            ("None_y", Velocity(x=0.25, y=None, yaw=0.0)),
        ]

    def test_invalid_values_valueerror_no_publish(self):
        for label, v in self._bad_values():
            with self.subTest(case=label):
                node = FakeNode()
                s = make_sender(node)
                with self.assertRaises(ValueError):
                    s(v)
                self.assertEqual(node.created, [],
                                 "非法输入绝不创建 publisher")
                self.assertEqual(node.pub.published, [])
                self.assertIsNotNone(s.fault)
                self.assertTrue(str(s.fault).startswith("invalid_input"))

    def test_invalid_objects_valueerror(self):
        for label, bad in (("None", None), ("object", object()),
                           ("Velocity_class", Velocity), ("int", 42),
                           ("str", "vel")):
            with self.subTest(case=label):
                node = FakeNode()
                s = make_sender(node)
                with self.assertRaises(ValueError):
                    s(bad)
                self.assertEqual(node.pub.published, [])

    def test_invalid_latches_and_subsequent_refused(self):
        node = FakeNode()
        s = make_sender(node)
        with self.assertRaises(ValueError):
            s(Velocity(x=float("nan")))
        with self.assertRaises(RuntimeError, msg="非法输入后必须锁存，后续拒绝"):
            s(Velocity(x=0.25))
        self.assertEqual(node.pub.published, [])


# ==================== publish 失败锁存 ====================

class TestPublishFailure(unittest.TestCase):

    def test_publish_error_propagates_and_latches_no_retry(self):
        boom = RuntimeError("dds down")
        node = FakeNode()
        node.pub.publish_error = boom
        s = make_sender(node)
        with self.assertRaises(RuntimeError) as cm:
            s(Velocity(x=0.25))
        self.assertIs(cm.exception, boom, "publish 失败必须原样传播，不吞错")
        self.assertEqual(node.pub.published, [])
        self.assertTrue(str(s.fault).startswith("publish_failed"))
        with self.assertRaises(RuntimeError, msg="锁存后拒绝发送，不得重试"):
            s(Velocity(x=0.25))
        self.assertEqual(len(node.created), 1)
        self.assertEqual(node.pub.publish_calls, 1, "publish 恰好尝试一次")


# ==================== close ====================

class TestClose(unittest.TestCase):

    def test_close_destroys_once_idempotent_no_extra_zero(self):
        node = FakeNode()
        s = make_sender(node)
        s(Velocity(x=0.25))
        s(Velocity(x=0.25))
        pub = node.pub
        s.close()
        self.assertIs(s.closed, True)
        self.assertEqual(node.destroyed, [pub], "只销毁自身 publisher")
        s.close()
        self.assertEqual(node.destroyed, [pub], "close 必须幂等，不重复销毁")
        self.assertEqual(len(node.pub.published), 2, "close 不得补发零速")
        self.assertEqual(node.shutdown_calls, 0, "close 不得 shutdown node")
        with self.assertRaises(RuntimeError, msg="close 后 enabled sender 拒绝发送"):
            s(Velocity(x=0.25))
        self.assertEqual(len(node.pub.published), 2)

    def test_close_without_pub_no_destroy(self):
        node = FakeNode()
        s = make_sender(node)
        s.close()
        self.assertEqual(node.destroyed, [], "从未创建 publisher 则无销毁")
        self.assertEqual(node.shutdown_calls, 0)
        # disabled sender close 同样无任何副作用
        node2 = FakeNode()
        s2 = NavCmdSender(node=node2, enabled=False)
        s2.close()
        s2.close()
        self.assertEqual(node2.created, [])
        self.assertEqual(node2.destroyed, [])
        self.assertEqual(node2.shutdown_calls, 0)


# ==================== GuardedOutput 组合 ====================

class TestGuardedOutputIntegration(unittest.TestCase):

    @staticmethod
    def _feed(guard, t):
        guard.update_status(good_status(), t)
        guard.update_scan(True, True, 0.0, t)
        guard.update_tracking(True, t)
        guard.update_command(Velocity(x=0.25, y=0.0, yaw=0.55), t)

    def test_default_output_disabled_no_pub_no_send(self):
        """默认 output_enabled=False：即便全部条件就绪也绝不创建 pub / 发送。"""
        node = FakeNode()
        sender = make_sender(node)
        guard = MotionGuard(output_enabled=False)
        self._feed(guard, 10.0)
        guard.set_exclusive_control(True)
        out = GuardedOutput(guard, sender)
        d = out.tick(10.0)
        self.assertFalse(d.allowed)
        self.assertEqual(d.reason, "output_disabled")
        self.assertEqual(node.created, [])
        self.assertEqual(node.pub.published, [])

    def test_armed_tick_sends_then_disarm_single_zero(self):
        node = FakeNode()
        sender = make_sender(node)
        guard = MotionGuard(output_enabled=True)
        self._feed(guard, 10.0)
        guard.set_exclusive_control(True)
        self.assertTrue(guard.arm(10.0).allowed)
        out = GuardedOutput(guard, sender)
        d = out.tick(10.0)
        self.assertTrue(d.allowed)
        self.assertAlmostEqual(node.pub.published[0].data.x_vel, 0.25)
        self.assertAlmostEqual(node.pub.published[0].data.yaw_vel, 0.55)
        self.assertEqual(len(node.pub.published), 1)
        # disarm 后：只补发一次零速，之后不再发送
        guard.disarm()
        out.tick(10.01)
        self.assertEqual(len(node.pub.published), 2)
        self.assertEqual(node.pub.published[1].data.x_vel, 0.0)
        self.assertEqual(node.pub.published[1].data.y_vel, 0.0)
        self.assertEqual(node.pub.published[1].data.yaw_vel, 0.0)
        out.tick(10.02)
        self.assertEqual(len(node.pub.published), 2, "blocked 零速只补一次")

    def test_send_failure_disarms_and_exclusive_cleared(self):
        node = FakeNode()
        node.pub.publish_error = RuntimeError("dds down")
        sender = make_sender(node)
        guard = MotionGuard(output_enabled=True)
        self._feed(guard, 10.0)
        guard.set_exclusive_control(True)
        self.assertTrue(guard.arm(10.0).allowed)
        out = GuardedOutput(guard, sender)
        with self.assertRaises(RuntimeError):
            out.tick(10.0)
        self.assertEqual(node.pub.published, [])
        # send 异常 → guard.disarm 且清 exclusive：
        # 不重新人工确认独占，即使数据仍新鲜也不得放行
        d = guard.arm(10.1)
        self.assertFalse(d.allowed)
        self.assertEqual(d.reason, "disarmed")


if __name__ == "__main__":
    unittest.main(verbosity=2)
