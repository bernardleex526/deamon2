"""nav_sender: /NAV_CMD 发送边界适配器（默认禁用替身，不是完整运行节点）。

- 默认 enabled=False：不 import ROS 消息、不访问 node、不创建 publisher、
  不发送任何消息（连零速都不发）；__call__ 返回 False。
- enabled=True 是未来现场集成授权选项，本轮仅用于替身 node 测试：必须传入
  调用方自有 rclpy Node。topic 固定 /NAV_CMD，无 topic 参数，不接受 remap。
  publisher 懒创建：首次 enabled 发送时才创建（depth=1，reliable/volatile 默认）。
- 节点所有权归调用方；close() 只销毁自身 publisher，不 shutdown node，
  也不补发零速（停止零速由 GuardedOutput 在销毁 sender 前负责）。
- 故障锁存：输入非法（ValueError）或 publish 失败后锁存 fault，后续调用拒绝，
  直到重新实例化；不自动复活、不重试。

警告：本模块不构成已接真实状态的完整运行节点，不能直接试走。见 NAV_SENDER.md。
"""

import math
import threading

from .core import (
    Velocity,
    MAX_VX,
    MAX_YAW,
    X_DEADZONE,
    YAW_DEADZONE,
)

__all__ = ["NavCmdSender"]

TOPIC = "/NAV_CMD"
DEPTH = 1
_UINT64_MASK = (1 << 64) - 1


def _is_num(v) -> bool:
    """有限 int/float；拒绝 bool / NaN / Inf / 无法转 float 的超大 int（溢出）。"""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return False
    try:
        return math.isfinite(v)
    except (OverflowError, ValueError):
        return False


def _limit(x: float, yaw: float):
    """独立再次限幅（不依赖 core 私有 _limit）：先 clamp 再死区，绝不向上抬。"""
    x = 0.0 if x < 0.0 else (MAX_VX if x > MAX_VX else x)
    yaw = -MAX_YAW if yaw < -MAX_YAW else (MAX_YAW if yaw > MAX_YAW else yaw)
    if abs(x) < X_DEADZONE:
        x = 0.0
    if abs(yaw) < YAW_DEADZONE:
        yaw = 0.0
    return x, 0.0, yaw


class NavCmdSender:
    """固定 /NAV_CMD 的速度发送适配器（无 CLI / 无 main / 无后台线程）。"""

    def __init__(self, node=None, enabled=False, message_type=None):
        if not isinstance(enabled, bool):
            raise TypeError("enabled 必须是精确 bool（不接受 str/int/None 等）")
        if message_type is not None and not isinstance(message_type, type):
            raise TypeError("message_type 必须是消息类或 None")
        self._node = node
        self._enabled = enabled
        self._message_type = message_type
        self._pub = None
        self._fault = None
        self._closed = False
        self._frame_id = 0
        self._lock = threading.Lock()

    # ---- 只读状态（供调用方 / tester 检查） ----
    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def topic(self) -> str:
        return TOPIC

    @property
    def fault(self):
        """故障锁存原因（str）或 None。"""
        return self._fault

    @property
    def closed(self) -> bool:
        return self._closed

    # ---- 内部 ----
    def _msg_class(self):
        if self._message_type is None:
            from drdds.msg import NavCmd  # 惰性 import：默认禁用时绝不 import ROS
            self._message_type = NavCmd
        return self._message_type

    def _ensure_publisher(self):
        if self._pub is not None:
            return self._pub
        if self._node is None:
            raise RuntimeError(
                "NavCmdSender: enabled=True 需要传入调用方自有 rclpy Node")
        pub = self._node.create_publisher(self._msg_class(), TOPIC, DEPTH)
        actual = getattr(pub, "topic_name", None)
        if actual != TOPIC:
            try:
                self._node.destroy_publisher(pub)  # best-effort，不保证一定销毁成功
            except Exception:
                pass  # 销毁失败不掩盖 topic 不符这一主错误
            self._fault = "topic_mismatch: actual=%r" % (actual,)
            raise RuntimeError(
                "NavCmdSender: publisher 实际 topic %r != %r，绝不发布；"
                "已请求销毁该 publisher（best-effort，未保证销毁成功）"
                % (actual, TOPIC))
        self._pub = pub
        return pub

    def _validated(self, velocity: Velocity):
        if not isinstance(velocity, Velocity):
            raise ValueError(
                "velocity 必须是 core.Velocity 实例，得到 %r"
                % type(velocity).__name__)
        if not (_is_num(velocity.x) and _is_num(velocity.y)
                and _is_num(velocity.yaw)):
            raise ValueError(
                "velocity 分量必须是有限 int/float（拒绝 bool/NaN/Inf/溢出）")
        return _limit(float(velocity.x), float(velocity.yaw))

    def _make_msg(self, x, y, yaw):
        msg = self._msg_class()()
        msg.data.x_vel = x
        msg.data.y_vel = y
        msg.data.yaw_vel = yaw
        msg.header.frame_id = self._frame_id  # uint64 从 0 递增，到顶回绕
        self._frame_id = (self._frame_id + 1) & _UINT64_MASK
        msg.header.stamp = self._node.get_clock().now().to_msg()  # 节点时钟，非墙钟
        return msg

    # ---- 发送边界 ----
    def __call__(self, velocity: Velocity) -> bool:
        """发送一条 NavCmd（SI 单位，已再次限幅）。enabled=False 时恒返回 False。"""
        with self._lock:
            if not self._enabled:
                return False  # 默认替身：不访问 node，不创建 publisher，不发送
            if self._closed:
                raise RuntimeError("NavCmdSender 已 close，拒绝发送")
            if self._fault is not None:
                raise RuntimeError(
                    "NavCmdSender 故障锁存，拒绝发送（需重新实例化）: %s"
                    % self._fault)
            try:
                x, y, yaw = self._validated(velocity)
            except ValueError as e:
                self._fault = 'invalid_input: %s' % (e,)  # 锁存，直到重新实例化
                raise
            try:
                pub = self._ensure_publisher()
            except Exception as e:
                if self._fault is not None:  # topic 不符已在内部锁存（保持原 reason）
                    raise
                # 创建期任何异常（import / msg factory / node.create_publisher，
                # 含 ValueError）都锁存并原样 raise；后续调用拒绝，不再触 create
                self._fault = "publisher_failed: %s: %r" % (type(e).__name__, e)
                raise
            try:
                pub.publish(self._make_msg(x, y, yaw))
            except Exception as e:
                self._fault = "publish_failed: %r" % (e,)
                raise  # 原样传播，不吞错、不重试、不自动复活
            return True

    def close(self):
        """幂等关闭：只销毁自身 publisher（best-effort），不 shutdown 节点，
        不补发零速。close 后 enabled sender 的 __call__ 拒绝发送；
        disabled sender 恒为惰性替身，__call__ 仍返回 False。"""
        with self._lock:
            self._closed = True
            pub, self._pub = self._pub, None
        if pub is not None and self._node is not None:
            try:
                self._node.destroy_publisher(pub)
            except Exception:
                pass  # best-effort：销毁失败不抛出，不掩盖调用方后续清理
