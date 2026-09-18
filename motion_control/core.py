"""MOTION-GUARD-CORE: 运动放行门控核心（纯逻辑，无 ROS、无 socket、无 IO）。

这是离线纯逻辑 core + 未来发送边界适配合同的一部分：
- 尚未连接实时 BasicStatus / 实际 ScanReader / 目标输出，也不是实机验收；
- 本模块自身不做任何网络来源认证（UDP 来源核查是未来 adapter 的责任）；
- clear（路径无障碍）由上游实际避障/扫描校验给出，本 core 不做几何计算；
- exclusive control 是人工确认的软条件，不等于主动证明 planner 已停止，
  真实接入方必须自行验证 planner / 充电占用与指令来源，不能仅靠这个 bool。

时间约定：所有时间均为调用方提供的单调钟秒（float），不使用墙钟判 TTL。
"""

import math
import threading
from dataclasses import dataclass
from typing import Callable

__all__ = [
    "Velocity", "Decision", "MotionGuard", "GuardedOutput",
    "MAX_VX", "MAX_YAW", "X_DEADZONE", "YAW_DEADZONE",
    "CMD_TTL", "STATUS_TTL", "SCAN_TTL", "TRACKING_TTL", "SCAN_FUTURE",
]


# ---- 限幅 / 死区（SI 单位：m/s、rad/s；与 M20 basic gait 4097 一致） ----
MAX_VX = 0.3        # x ∈ [0, 0.3]，禁止后退
MAX_YAW = 0.6       # yaw ∈ [-0.6, 0.6]
X_DEADZONE = 0.2    # |x| < 0.2 置零（basic 步态有效区，绝不向上放大）
YAW_DEADZONE = 0.5  # |yaw| < 0.5 置零
# y 恒为 0：basic 步态 y 最小有效 0.35 > 上限 0.3，横移永远不可达，直接禁用

# ---- 新鲜度（单调秒） ----
CMD_TTL = 0.3
STATUS_TTL = 0.5
SCAN_TTL = 0.5
TRACKING_TTL = 0.5
SCAN_FUTURE = 0.1   # 上游 source_age 允许的最多未来量

# 原始已验证 BasicStatus 必须精确匹配的官方字段（exact int，非 bool）。
# 只承诺这些字段；StatusCode 等未列字段不强制；多余字段忽略。
EXPECTED_STATUS = {
    "MotionState": 17,
    "ControlUsageMode": 1,
    "Direction": 0,
    "HES": 0,
    "Charge": 0,
    "Sleep": 0,
    "Gait": 4097,
}


@dataclass(frozen=True)
class Velocity:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Decision:
    velocity: Velocity
    armed: bool
    allowed: bool
    reason: str  # 稳定 lowercase 标识，见 README


ZERO = Velocity()


def _is_num(v) -> bool:
    """有限数值；拒绝 bool / 字符串 / NaN / Inf。"""
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _valid_ts(t) -> bool:
    return _is_num(t) and t >= 0.0


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _limit(v: Velocity) -> Velocity:
    """先 clamp 再置 basic 步态死区，绝不提高到最低速度；不取绝对值代替符号。"""
    x = _clamp(v.x, 0.0, MAX_VX)
    yaw = _clamp(v.yaw, -MAX_YAW, MAX_YAW)
    if abs(x) < X_DEADZONE:
        x = 0.0
    if abs(yaw) < YAW_DEADZONE:
        yaw = 0.0
    return Velocity(x, 0.0, yaw)


class MotionGuard:
    """运动放行门控。默认 output_enabled=False（绝不输出）。

    所有 update_* 输入一旦非法立即解除 arm 并锁存为对应 invalid/missing，
    不忽略 bad 保留旧 good。恢复数据后 evaluate 仍为零，需再次显式 arm()。
    """

    def __init__(self, output_enabled: bool = False):
        if not isinstance(output_enabled, bool):
            raise TypeError("output_enabled must be exactly bool, not str/other")
        self._output_enabled = output_enabled
        self._lock = threading.RLock()  # 内部共享状态的多线程保护（stdlib）
        self._armed = False
        self._exclusive = False
        self._last_now = None  # 最后一次合法 evaluate/arm 的单调时间
        self._status_reason = "status_missing"; self._status_at = 0.0
        self._scan_reason = "scan_missing"
        self._scan_at = 0.0; self._scan_age0 = 0.0
        self._tracking_reason = "tracking_missing"; self._tracking_at = 0.0
        self._command_reason = "command_missing"
        self._command_at = 0.0; self._command = ZERO

    # ---- 时钟 ----
    def _check_clock(self, now) -> bool:
        """返回 True 表示 now 非法（非 finite/bool/负数/回拨）。"""
        if not _valid_ts(now):
            return True
        if self._last_now is not None and now < self._last_now:
            return True
        self._last_now = now
        return False

    def _disarm(self):
        self._armed = False
        self._exclusive = False  # 重新放行必须再次人工确认独占

    # ---- 输入 ----
    def update_status(self, status: dict, received_at: float):
        with self._lock:
            if not isinstance(status, dict) or not _valid_ts(received_at):
                self._status_reason = "status_invalid"; self._disarm(); return
            for key, expected in EXPECTED_STATUS.items():
                v = status.get(key)
                if not _is_int(v) or v != expected:
                    self._status_reason = "status_invalid"; self._disarm(); return
            self._status_reason = None
            self._status_at = float(received_at)

    def update_scan(self, valid: bool, clear: bool, source_age: float,
                    received_at: float):
        with self._lock:
            if (not isinstance(valid, bool) or not isinstance(clear, bool)
                    or not _is_num(source_age)
                    or source_age < -SCAN_FUTURE or source_age > SCAN_TTL
                    or not _valid_ts(received_at)):
                self._scan_reason = "scan_invalid"; self._disarm(); return
            if not valid:
                self._scan_reason = "scan_missing"; self._disarm(); return
            self._scan_reason = None
            self._scan_at = float(received_at)
            self._scan_age0 = float(source_age)
            # clear=False 视为无可用通行判定，同样不放行
            if not clear:
                self._scan_reason = "scan_missing"; self._disarm(); return

    def update_tracking(self, valid: bool, received_at: float):
        with self._lock:
            if not isinstance(valid, bool) or not _valid_ts(received_at):
                self._tracking_reason = "tracking_invalid"; self._disarm(); return
            if not valid:
                self._tracking_reason = "tracking_missing"; self._disarm(); return
            self._tracking_reason = None
            self._tracking_at = float(received_at)

    def update_command(self, command: Velocity, received_at: float):
        with self._lock:
            v = self._extract(command)
            if v is None or not _valid_ts(received_at):
                self._command_reason = "command_invalid"; self._disarm(); return
            self._command_reason = None
            self._command = v
            self._command_at = float(received_at)

    @staticmethod
    def _extract(command):
        if command is None:
            return None
        try:
            x, y, yaw = command.x, command.y, command.yaw
        except AttributeError:
            return None
        if not (_is_num(x) and _is_num(y) and _is_num(yaw)):
            return None
        if isinstance(command, Velocity):
            return command
        return Velocity(float(x), float(y), float(yaw))

    def set_exclusive_control(self, confirmed: bool):
        """人工确认独占控制的软条件；False（或非 bool）立即解除 arm。"""
        with self._lock:
            self._exclusive = isinstance(confirmed, bool) and confirmed
            if not self._exclusive:
                self._disarm()

    def disarm(self) -> None:
        """立即解除 arm（返回 None）。恢复数据后也不会自动 arm。"""
        with self._lock:
            self._disarm()

    # ---- 判定 ----
    def _blocking_reason(self, now: float):
        if self._status_reason is not None:
            return self._status_reason
        if self._status_at > now or now - self._status_at > STATUS_TTL:
            return "status_stale"
        if self._scan_reason is not None:
            return self._scan_reason
        if self._scan_at > now:
            return "scan_stale"
        if self._scan_age0 + (now - self._scan_at) > SCAN_TTL:
            return "scan_stale"
        if self._tracking_reason is not None:
            return self._tracking_reason
        if self._tracking_at > now or now - self._tracking_at > TRACKING_TTL:
            return "tracking_stale"
        if self._command_reason is not None:
            return self._command_reason
        if self._command_at > now or now - self._command_at > CMD_TTL:
            return "command_stale"
        return None

    def arm(self, now: float) -> Decision:
        """尝试放行；需 output_enabled + exclusive + 全部输入有效新鲜。"""
        with self._lock:
            if self._check_clock(now):
                self._disarm()
                return Decision(ZERO, False, False, "clock_invalid")
            if not self._output_enabled:
                return Decision(ZERO, False, False, "output_disabled")
            if not self._exclusive:
                self._disarm()
                return Decision(ZERO, False, False, "disarmed")
            reason = self._blocking_reason(now)
            if reason is not None:
                self._disarm()
                return Decision(ZERO, False, False, reason)
            self._armed = True
            return Decision(_limit(self._command), True, True, "ready")

    def evaluate(self, now: float) -> Decision:
        with self._lock:
            if self._check_clock(now):
                self._disarm()
                return Decision(ZERO, False, False, "clock_invalid")
            if not self._output_enabled:
                return Decision(ZERO, False, False, "output_disabled")
            reason = self._blocking_reason(now)
            if reason is not None:
                self._disarm()
                return Decision(ZERO, False, False, reason)
            if not self._armed:
                return Decision(ZERO, False, False, "disarmed")
            return Decision(_limit(self._command), True, True, "ready")


class GuardedOutput:
    """发送边界：guard 放行才调用 send；绝不创建真实 transport。

    - output_enabled=False 或从未放行过：绝不调用 send（连零速都不发），
      避免干扰既有控制链路；
    - 曾放行后转入 blocked（失败/显式 disarm/超时）：只补发一次零速，
      之后不再重复发送零速；
    - send 异常：guard.disarm()（同时清除 exclusive，阻止自动恢复），
      并把异常原样传播给调用方，绝不吞错伪成功。断网时无法保证零速
      真实到达机器，只能依赖机端看门狗 / 现场急停。
    """

    def __init__(self, guard: MotionGuard, send: Callable[[Velocity], None]):
        if not isinstance(guard, MotionGuard):
            raise TypeError("guard must be a MotionGuard instance")
        if not callable(send):
            raise TypeError("send must be callable")
        self._guard = guard
        self._send = send
        self._pending_zero = False  # 已放行过输出，blocked 时欠一次零速

    def tick(self, now: float) -> Decision:
        d = self._guard.evaluate(now)
        if d.allowed:
            try:
                self._send(d.velocity)
            except Exception:
                self._guard.disarm()
                raise
            self._pending_zero = True
            return d
        if self._pending_zero:
            self._pending_zero = False
            try:
                self._send(Velocity())
            except Exception:
                self._guard.disarm()
                raise
        return d
