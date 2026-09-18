"""motion_control: M20Pro 运动放行门控（MOTION-GUARD-CORE）。

纯逻辑离线 core + 发送边界（GuardedOutput），无 ROS、无 socket。
默认 output_enabled=False：绝不输出、绝不调用真实发送回调。
真实发送端尚未接入；本包不是实机验收，不能直接启动机器人运动。

典型用法见 README.md 与 core.py 模块文档。
"""

from .core import (
    Decision,
    GuardedOutput,
    MotionGuard,
    Velocity,
)

__all__ = ["Velocity", "Decision", "MotionGuard", "GuardedOutput"]
