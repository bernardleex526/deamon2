#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NEW-10: nav_cmd_preview.py 纯函数单元测试（仅静态语法检查 + importlib 导入）。

被测文件: /home/user/new_chase/src/jie_deamon/scripts/nav_cmd_preview.py
- to_nav_values(vx, vy, wz) : Twist -> NavCmd data 值 (x_vel, y_vel, yaw_vel)
- is_fresh(now, stamp, window=0.3) : raw 指令新鲜度判断

约束（本任务约定）：
- 本测试允许 importlib 加载该模块（模块顶层只有常量与纯函数，无 ROS import）；
  但绝不执行 main()，不运行任何被测节点/进程。
- 断言语义来自 NEW_CHASE.md 第 5 节契约 + GAIT_MIN_VX/VY/YAW、PREVIEW_MAX_* 常量
  （先按 src 当前常量值静态核对，不改变 src）。
- 阈值取值说明：src 中 GAIT_MIN_VY=0.35 且预览路径 vy 恒为 0（y 阈值实际不可见），
  因此本文件仅用任务给定的 x(0.2)/yaw(0.5) 阈值断言 vy==0.0，不对 vy 阈值做边界断言。

运行（主代理执行，本任务不运行）:
  python3 -B -m unittest discover -s /home/user/new_chase/test -p test_preview_values.py -v
"""

import ast
import importlib.util
import unittest

SRC = '/home/user/new_chase/src/jie_deamon/scripts/nav_cmd_preview.py'

# 与 src 顶层常量一致（任务给定值；若 src 变更导致失败，需主代理复核而非放宽断言）
PREVIEW_MAX_VX = 0.3
PREVIEW_MAX_WZ = 0.6


def load_module():
    """importlib 按 src 真实路径加载模块（不运行 main；顶层无 ROS 导入）。"""
    spec = importlib.util.spec_from_file_location('nav_cmd_preview_under_test', SRC)
    if spec is None or spec.loader is None:  # pragma: no cover - 防御
        raise RuntimeError('无法从 %s 创建 importlib spec' % SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestSyntaxOnly(unittest.TestCase):
    """静态语法检查：python3 -B 下 ast.parse 通过且顶层无 ROS import。"""

    def test_ast_parse_ok(self):
        with open(SRC, 'r', encoding='utf-8') as f:
            tree = ast.parse(f.read(), filename=SRC)
        self.assertTrue(isinstance(tree, ast.Module))

    def test_no_ros_import_at_module_top_level(self):
        """main() 之外不应导入 ROS（顶层 import/FromImport 不得出现 rclpy 等模块）。"""
        with open(SRC, 'r', encoding='utf-8') as f:
            tree = ast.parse(f.read(), filename=SRC)
        ros_tokens = ('rclpy', 'geometry_msgs', 'drdds', 'sensor_msgs')
        for node in tree.body:
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or '']
            else:
                continue
            for name in names:
                for token in ros_tokens:
                    self.assertNotIn(token, name,
                                     '模块顶层不得导入 ROS 模块: %s' % name)

    def test_module_import_has_no_ros(self):
        """importlib 加载成功即可用（若顶层误加 ROS 导入且环境无 ROS，会在此失败）。"""
        mod = load_module()
        self.assertTrue(callable(mod.to_nav_values))
        self.assertTrue(callable(mod.is_fresh))


class TestToNavValues(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_module()
        cls.fn = staticmethod(cls.mod.to_nav_values)

    def test_valid_pass_through(self):
        """有效 (.25,0,.55) 原值输出（含单位语义：SI m/s、rad/s）。"""
        self.assertEqual(self.fn(0.25, 0.0, 0.55), (0.25, 0.0, 0.55))

    def test_threshold_kept_at_boundary(self):
        """恰好等于阈值 (x=0.2, yaw=0.5) 保留（阈值判定为 < 严格小于）。"""
        self.assertEqual(self.fn(0.2, 0.0, 0.5), (0.2, 0.0, 0.5))

    def test_below_threshold_zeroed(self):
        """0.199 / 0.499 低于阈值置零（vy 恒 0）。"""
        self.assertEqual(self.fn(0.199, 0.0, 0.499), (0.0, 0.0, 0.0))

    def test_negative_vx_clamped_to_zero(self):
        """vx<0 置 0：-0.3 → 0.0；负 yaw 保留（限幅内 -0.55 → -0.55）。"""
        self.assertEqual(self.fn(-0.3, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(self.fn(-0.05, 0.0, -0.55), (0.0, 0.0, -0.55))

    def test_vy_always_zero(self):
        """vy 恒为 0（预览限幅 y<0.35<gait 阈值恒无效，无论输入多大/多小）。"""
        self.assertEqual(self.fn(0.25, 0.0, 0.0)[1], 0.0)
        self.assertEqual(self.fn(0.25, 1.0, 0.0)[1], 0.0)
        self.assertEqual(self.fn(0.25, -1.0, 0.0)[1], 0.0)
        self.assertEqual(self.fn(0.25, float('nan'), 0.0), (0.0, 0.0, 0.0))

    def test_huge_values_clamped(self):
        """限幅：超出上限截断到 0.3 / 0.6（含负 yaw → -0.6）；inf 非有限 → 全零。"""
        self.assertEqual(self.fn(1e9, 1e9, 1e9), (PREVIEW_MAX_VX, 0.0, PREVIEW_MAX_WZ))
        self.assertEqual(self.fn(1e9, 1e9, -1e9), (PREVIEW_MAX_VX, 0.0, -PREVIEW_MAX_WZ))
        self.assertEqual(self.fn(-1e9, 0.0, -1e9), (0.0, 0.0, -PREVIEW_MAX_WZ))
        self.assertEqual(self.fn(float('inf'), 0.0, 0.0), (0.0, 0.0, 0.0))

    def test_nan_returns_zero(self):
        """NaN 输入 → 全零（不允许输出 nan）。"""
        self.assertEqual(self.fn(float('nan'), 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(self.fn(0.25, 0.0, float('nan')), (0.0, 0.0, 0.0))

    def test_str_bad_returns_zero(self):
        """非数字字符串/None → 全零；不抛异常。"""
        self.assertEqual(self.fn('bad', 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(self.fn(None, 0.0, 0.0), (0.0, 0.0, 0.0))

    def test_float_overflow_returns_zero(self):
        """float 溢出到 inf（1e309）→ 非有限 → 全零。"""
        self.assertEqual(self.fn(1e309, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(self.fn(0.25, 0.0, 1e309), (0.0, 0.0, 0.0))


class TestIsFresh(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_module()
        cls.fn = staticmethod(cls.mod.is_fresh)

    def test_within_window_fresh(self):
        self.assertTrue(self.fn(10.0, 9.8))          # delta=0.2 <= 0.3
        self.assertTrue(self.fn(10.0, 10.0))         # delta=0

    def test_future_stamp_not_fresh(self):
        """stamp 在未来（delta<0）不新鲜。契约要求 delta∈[0, window]；
        （编写时快照：第2轮 src is_fresh 已含该判定；语义以主代理复核为准，
        本断言不放宽。）"""
        self.assertFalse(self.fn(10.0, 10.1))

    def test_expired_not_fresh(self):
        """超过 window 不新鲜。"""
        self.assertFalse(self.fn(10.0, 9.6))         # delta=0.4 > 0.3

    def test_boundary_fp_caution(self):
        """0.3s 边界注意浮点误差（FP）：
        - delta 用字面量 0.3（同一 double 表示）→ 新鲜；
        - 0.4-0.1 实际是 0.30000000000000004 > 0.3 → 不新鲜（按 delta<=window 语义）。
        主代理运行时如遇边界翻转，属 FP 现象，需复核语义而非放宽断言。
        """
        self.assertTrue(self.fn(0.3, 0.0, 0.3))      # delta=0.3（同一表示）
        self.assertFalse(self.fn(0.4, 0.1, 0.3))     # 0.4-0.1 == 0.30000000000000004

    def test_inf_nan_stamp_not_fresh(self):
        """inf/nan 时间戳 → False（不允许把异常时间当新鲜）。契约要求三个输入
        finite；（编写时快照：第2轮 src is_fresh 已含 finite 判定，语义以主代理
        复核为准，本断言不放宽。）"""
        self.assertFalse(self.fn(10.0, float('inf')))
        self.assertFalse(self.fn(10.0, float('-inf')))
        self.assertFalse(self.fn(10.0, float('nan')))

    def test_inf_now_not_fresh(self):
        self.assertFalse(self.fn(float('inf'), 9.8))

    def test_type_errors_false(self):
        """类型错误（None/str）→ False，不抛异常。"""
        self.assertFalse(self.fn(None, 9.8))
        self.assertFalse(self.fn(10.0, None))
        self.assertFalse(self.fn('x', 9.8))
        self.assertFalse(self.fn(10.0, 'y'))

    def test_window_variants(self):
        """window 可指定且非负；window=0 仅同时刻新鲜；window<0 → False 而非异常。"""
        self.assertTrue(self.fn(10.0, 9.9, 0.2))
        self.assertFalse(self.fn(10.0, 9.9, 0.0))
        self.assertTrue(self.fn(10.0, 10.0, 0.0))
        self.assertFalse(self.fn(10.0, 9.9, -0.1))   # window 非负语义


if __name__ == '__main__':
    unittest.main(verbosity=2)
