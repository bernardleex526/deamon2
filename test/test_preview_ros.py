#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NEW-10: M20 preview ROS 集成测试（修复第1轮；本任务不运行）。

范围（主代理收齐功能/导出编码确认并停止修改后，方可运行）：
- 固定隔离域 ROS_DOMAIN_ID=47（绝不 0）+ 只读 UDP-only profile
  /home/user/new_chase/config/diagnostic_udp_only.xml；清洁 env：只 source
  /opt/ros/foxy/setup.bash + /home/user/new_chase/install/setup.bash，
  不导入任何旧 workspace（禁止 /home/user/m20_chase 相关路径）。
- 被测可执行（精确路径，已经只读 ls 确认二进制名与 web_root）：
    /home/user/new_chase/install/jie_deamon/lib/jie_deamon/robot_nexus
    /home/user/new_chase/install/jie_deamon/lib/jie_deamon/nav_cmd_preview.py
    web_root=/home/user/new_chase/install/jie_deamon/share/jie_deamon/web（存在 index.html）
- 本测试自己的 ROS 节点只：
    * 发布合成 LaserScan 到 /new_chase/scan（frame=lidar_link）
    * 订阅 /new_chase/cmd_vel_raw（Twist）与 /new_chase/nav_cmd_preview（NavCmd），
      回调记录 (接收monotonic时间, 消息) 以判定“零速在关闭请求之后收到”
  绝不向 /NAV_CMD /cmd_vel /d1_cmd 发布；graph 断言这些话题在本测试的
  隔离域 47 中无任何 publisher（遇其它 publisher 禁止继续，立即失败）。
- HTTP 8080/8890 先检查空闲才允许 enable_web 启动；被占用则 SkipTest
  （不抢占、不停止他人进程；总结果按 incomplete 记录，不算全 pass）。
- 所有等待有界：单场景 ≤10s（ready 各 ≤7.5s），整体硬上限 90s
  （TotalBudgetExceeded → 中止并如实报告，不伪造通过）。
- 子进程：精确 PID，SIGINT + wait(5s)；正常退出不强杀；5s 未退报错并
  保留 PID（不 pkill、不 kill -9）。
- 节点 stderr 唯一定名 /home/user/new_chase/evidence/NEW-10-node-stderr.log
  追加写（不覆盖、不删除、不清理）；负向用例按字节偏移读取本进程专属
  log 段断言拒绝语义（多关键词，不依赖固定脆弱英文）。

入口模式（本文件唯一入口）：必须以显式 env 运行，否则退出码 2：
  ROS_DOMAIN_ID=47 NEW10_ALLOW_RUN=1 python3 -B /home/user/new_chase/test/test_preview_ros.py
_main 会为测试主进程本身强制 approved UDP profile / RMW / 新 install 路径
（避免主进程意外使用 SHM 500MB profile），并预检 rclpy/drdds 可导入——
缺环境退出码 2（不产生 Skip 伪通过）。

负向重映射测试同样只在 domain47 内进行，绝不 domain0；预期被测进程
以 exit 2 快速拒绝且未创建真实控制 publisher（stderr 须含拒绝语义，
且不是 ImportError/exec 127 之类环境错误）。
"""

import os
import signal
import socket
import subprocess
import sys
import time
import unittest
import json
import urllib.error
import urllib.request

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
ROS_DOMAIN_ID = 47                      # 固定隔离域，绝不 domain 0
NEW_CHASE_ROOT = '/home/user/new_chase'
ROBOT_NEXUS_BIN = NEW_CHASE_ROOT + '/install/jie_deamon/lib/jie_deamon/robot_nexus'
NAV_PREVIEW_BIN = NEW_CHASE_ROOT + '/install/jie_deamon/lib/jie_deamon/nav_cmd_preview.py'
WEB_ROOT = NEW_CHASE_ROOT + '/install/jie_deamon/share/jie_deamon/web'
FASTRTPS_PROFILE = NEW_CHASE_ROOT + '/config/diagnostic_udp_only.xml'
NODE_STDERR_LOG = NEW_CHASE_ROOT + '/evidence/NEW-10-node-stderr.log'  # 追加写，永不删除

# 清洁环境 source 命令（只 Foxy + 本 install，不加载任何旧 workspace）
CLEAN_SOURCE = ('source /opt/ros/foxy/setup.bash && '
                'source %s/install/setup.bash && ' % NEW_CHASE_ROOT)

SCAN_TOPIC = '/new_chase/scan'
RAW_TOPIC = '/new_chase/cmd_vel_raw'
PREVIEW_TOPIC = '/new_chase/nav_cmd_preview'
SCAN_FRAME = 'lidar_link'
FOLLOW_DIST = 1.2

HTTP_PORT = 8080
WS_PORT = 8890
HTTP_BASE = 'http://127.0.0.1:%d' % HTTP_PORT

TOTAL_BUDGET_S = 90.0
SCENE_TIMEOUT_S = 10.0
WAIT_READY_S = 15.0
GRACE_STOP_S = 5.0
ZERO_REACTION_S = 0.2                   # 关 moving/active 后 ≤0.2s 零速（看门狗路径）
REJECT_REACTION_S = 0.25                # 单条坏帧后 ≤0.25s 观测新零+失能
SCAN_RATE_HZ = 10.0
PUMP_S = 0.05

N_ANGLES = 360
ANGLE_MIN = -3.141592653589793
ANGLE_INCREMENT = 2 * 3.141592653589793 / N_ANGLES
RANGE_MIN = 0.10
RANGE_MAX = 20.0
TARGET_CLUSTER = [2.00, 2.01, 2.02, 2.03]   # 近 x≈2m 目标簇（角 -0.05..0.05）
FAR_CLUSTER = [8.00, 8.01, 8.02, 8.03]      # 有效但远离目标的远点（目标丢失触发）
FAR_X = 8.0

FORBIDDEN_TOPICS = ('/NAV_CMD', '/cmd_vel', '/d1_cmd')

# 负向拒绝语义关键词（多来源，不把单一脆弱英文当唯一标准）：
# Python: “拒绝启动(exit 2): 检测到被禁止的重映射规则…”；C++: “非法（必须严格
# 位于 /new_chase/ 下且非真实控制话题）/ 拒绝任何把预览raw重映射到真实控制话题”。
REJECT_KEYWORDS = ('拒绝', '禁止', '非法', 'forbidden', 'rejected',
                   'refus', 'not allowed', 'denied', 'real control', '真实控制')
# 非拒绝语义的环境错误（不得据此判定 guard 生效）：
NOT_GUARD_KEYWORDS = ('ImportError', 'ModuleNotFoundError', 'command not found',
                      'No such file', 'Permission denied', 'exec format')

_start_monotonic = time.monotonic()


def _remaining():
    return TOTAL_BUDGET_S - (time.monotonic() - _start_monotonic)


class TotalBudgetExceeded(Exception):
    pass


def _ensure_budget(where):
    if _remaining() <= 0:
        raise TotalBudgetExceeded('整体 90s 预算耗尽于 %s' % where)


# ---------------------------------------------------------------------------
# HTTP helpers
# ---------------------------------------------------------------------------
def _http_get(path, timeout=2.0):
    with urllib.request.urlopen(HTTP_BASE + path, timeout=timeout) as resp:
        return resp.status, resp.read().decode('utf-8', 'replace')


def _http_post(path, body, timeout=2.0):
    req = urllib.request.Request(HTTP_BASE + path,
                                 data=body.encode('utf-8'), method='POST')
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read().decode('utf-8', 'replace')


def _http_post_expect_400(path, body, timeout=2.0):
    """负向请求：返回实际状态码（由调用方 assert 400，不放宽为任意 HTTP）。"""
    req = urllib.request.Request(HTTP_BASE + path,
                                 data=body.encode('utf-8'), method='POST')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, resp.read().decode('utf-8', 'replace')
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode('utf-8', 'replace')


def _port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(1.0)
    try:
        s.bind(('0.0.0.0', port))
        return True
    except OSError:
        return False
    finally:
        s.close()


# ---------------------------------------------------------------------------
# 进程管理
# ---------------------------------------------------------------------------
class _ProcHarness:
    """精确路径进程启动 + 有界 ready 等待 + SIGINT 优雅终止。"""

    @staticmethod
    def _bounded_wait(pred, timeout, pump=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pred():
                return True
            if pump is not None:
                pump()
            time.sleep(PUMP_S)
        return pred()

    @classmethod
    def _spawn(cls, name, argv, stderr_file=None, wait_ready=None, pump=None):
        """启动一个被测可执行（精确路径 + domain47 env + 清洁 source）。

        wait_ready: callable -> bool；有界等待（各 ≤WAIT_READY_S/2），
        防止一个进程挂起拖死另一个（分开等待）。失败时优雅终止该进程：
        终止错误必须记录并连同 PID 一起 raise（不隐藏）。
        """
        _ensure_budget('spawn ' + name)
        env = {
            'HOME': '/home/user', 'USER': 'user', 'LOGNAME': 'user',
            'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
            'TERM': 'xterm', 'LANG': 'C.UTF-8',
            'ROS_DOMAIN_ID': str(ROS_DOMAIN_ID),
            'FASTRTPS_DEFAULT_PROFILES_FILE': FASTRTPS_PROFILE,
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'ROS_LOCALHOST_ONLY': '0',
            'PYTHONDONTWRITEBYTECODE': '1',
            'PYTHONUNBUFFERED': '1',
        }
        proc = subprocess.Popen(
            argv, env=env, stdout=subprocess.DEVNULL,
            stderr=stderr_file if stderr_file is not None else subprocess.DEVNULL,
            cwd=NEW_CHASE_ROOT)
        if wait_ready is not None:
            half = WAIT_READY_S / 2.0
            if not cls._bounded_wait(wait_ready, half, pump=pump):
                try:
                    cls._stop_procs({name: proc})
                except AssertionError as stop_err:
                    raise AssertionError(
                        '进程 %s (pid=%d) 在 %.0fs 内未 ready；且停止时: %s'
                        % (name, proc.pid, half, stop_err))
                raise AssertionError('进程 %s (pid=%d) 在 %.0fs 内未 ready'
                                     % (name, proc.pid, half))
        return proc

    @classmethod
    def _stop_procs(cls, procs):
        """SIGINT + wait(5s)；正常退出不强杀；5s 未退报错保留 PID。"""
        errors = []
        for name, proc in procs.items():
            if proc.poll() is None:
                try:
                    proc.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    continue
            try:
                proc.wait(timeout=GRACE_STOP_S)
            except subprocess.TimeoutExpired:
                errors.append('进程 %s (pid=%d) SIGINT 后 %ds 未退出；'
                              '按约定保留 PID 不强杀/不 kill -9'
                              % (name, proc.pid, GRACE_STOP_S))
        if errors:
            raise AssertionError('; '.join(errors))


def _make_scan_msg(LaserScan, stamp_msg, frame=SCAN_FRAME, ranges=None):
    """合成 LaserScan：默认近 x≈2m 目标簇（角 -0.05..0.05 多点），其余 inf；
    range_min/max/angle 元数据合法；传入 ranges 时完全使用给定值。"""
    msg = LaserScan()
    msg.header.frame_id = frame
    msg.header.stamp = stamp_msg
    msg.angle_min = ANGLE_MIN
    msg.angle_max = 3.141592653589793
    msg.angle_increment = ANGLE_INCREMENT
    msg.time_increment = 0.0
    msg.scan_time = 0.1
    msg.range_min = RANGE_MIN
    msg.range_max = RANGE_MAX
    if ranges is not None:
        r = list(ranges)
    else:
        r = [float('inf')] * N_ANGLES
        idx0 = int((-0.05 - ANGLE_MIN) / ANGLE_INCREMENT)
        for k, val in enumerate(TARGET_CLUSTER):
            r[(idx0 + k) % N_ANGLES] = val
    msg.ranges = r
    return msg


def _paired_shutdown(node=None):
    """成对清理：销毁节点 + rclpy.shutdown；返回错误列表（不吞，由调用方处理）。"""
    errs = []
    if node is not None:
        try:
            node.destroy_node()
        except Exception as e:
            errs.append(e)
    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        errs.append(e)
    return errs


def _raise_if_no_inflight(errs):
    """仅当当前无在途异常时 raise 清理错误（避免掩盖原始错误）。"""
    if errs and sys.exc_info()[0] is None:
        raise errs[0]


# ---------------------------------------------------------------------------
# 功能场景
# ---------------------------------------------------------------------------
class TestPreviewFunctional(unittest.TestCase):
    """功能场景（共享一对被测进程；按序执行）。

    覆盖：Web 静态资源 / 初始零速 / 选点+使能正向（raw 与 NavCmd 分别
    有界等待）/ 坏选点 400 / 关 moving≤0.2s 零（看门狗 walltimer 路径，
    暂停 scan）/ 关 active≤0.2s 零（同 walltimer 路径）/ 目标丢失与
    恢复门控 / 坏 frame / 旧戳 / 未来戳单帧当帧零+失能 / scan 停止
    >0.5s / graph 禁区检查。
    """

    proc_nexus = None
    proc_preview = None
    stderr_file = None

    @classmethod
    def setUpClass(cls):
        _ensure_budget('functional setUpClass')
        if not (_port_free(HTTP_PORT) and _port_free(WS_PORT)):
            raise unittest.SkipTest('8080/8890 被他人占用：按约定不启动/不抢占')
        try:
            import rclpy
            from drdds.msg import NavCmd
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import LaserScan
        except ImportError as e:
            # 不做 Skip 伪通过：缺环境让 setUpClass 直接 error（主代理需补环境）
            raise AssertionError(
                'rclpy/drdds 不可用（需 Foxy 环境，主代理应以清洁env运行）: %s' % e)
        cls._rclpy = rclpy
        cls._Twist = Twist
        cls._LaserScan = LaserScan
        cls._NavCmd = NavCmd

        rclpy.init(args=None)
        cls.node = rclpy.create_node('new10_preview_values_tester')
        # 回调记录 (接收monotonic时间, 消息)：确保零速消息在关闭请求之后收到
        cls.raw_msgs = []
        cls.preview_msgs = []
        cls.node.create_subscription(
            Twist, RAW_TOPIC, lambda m: cls.raw_msgs.append((time.monotonic(), m)), 10)
        cls.node.create_subscription(
            NavCmd, PREVIEW_TOPIC, lambda m: cls.preview_msgs.append((time.monotonic(), m)), 10)
        cls.scan_pub = cls.node.create_publisher(LaserScan, SCAN_TOPIC, 10)

        # 节点 stderr 追加写固定 evidence 日志（本任务不运行故不创建；运行时追加）
        cls.stderr_file = open(NODE_STDERR_LOG, 'ab')

        nexus_params = [
            'm20_preview:=true',
            'active:=true',
            'enable_web:=true',
            'web_root:=' + WEB_ROOT,
            'rotation_angle:=0.0',
            'follow_distance:=%f' % FOLLOW_DIST,
            'scan_topic:=' + SCAN_TOPIC,
            'expected_scan_frame:=' + SCAN_FRAME,
            # 几何与 M20 launch 一致（frame 前后0.46 左右0.31 rect 0.7）
            'rectangle_width:=0.7',
            'robot_frame_front:=0.46',
            'robot_frame_back:=0.46',
            'robot_frame_left:=0.31',
            'robot_frame_right:=0.31',
        ]
        nexus_argv = [
            'bash', '-c',
            CLEAN_SOURCE + 'exec ' + ROBOT_NEXUS_BIN + ' --ros-args'
            + ''.join(' -p ' + p for p in nexus_params),
        ]
        preview_argv = ['bash', '-c', CLEAN_SOURCE + 'exec ' + NAV_PREVIEW_BIN]

        try:
            cls.proc_nexus = _ProcHarness._spawn(
                'robot_nexus', nexus_argv, stderr_file=cls.stderr_file,
                wait_ready=lambda: cls._http_home_ok())
            cls.proc_preview = _ProcHarness._spawn(
                'nav_cmd_preview', preview_argv, stderr_file=cls.stderr_file,
                wait_ready=lambda: len(cls.preview_msgs) > 0,
                pump=cls._pump)
        except Exception:
            cls._teardown_class()
            raise
        # ready 已分开有界等待（_spawn 内各 ≤7.5s）。

    # -- helpers ------------------------------------------------------------
    @classmethod
    def _http_home_ok(cls):
        try:
            code, body = _http_get('/')
            return code == 200 and '预览' in body
        except Exception:
            return False

    @classmethod
    def _pump(cls, seconds=PUMP_S):
        cls._rclpy.spin_once(cls.node, timeout_sec=seconds)

    @classmethod
    def _publish_scan(cls, **kw):
        _ensure_budget('publish_scan')
        stamp = kw.pop('stamp', None)      # 修复r1: test_09/10 旧/未来stamp经kw传入
        if stamp is None:
            stamp = cls.node.get_clock().now().to_msg()
        msg = _make_scan_msg(cls._LaserScan, stamp, **kw)
        cls.scan_pub.publish(msg)

    @classmethod
    def _scan_pump(cls, seconds, stamp=None, frame=SCAN_FRAME, ranges=None):
        """以 10Hz 发布 fresh scan 并泵 spin，持续 seconds（期间 HTTP 可读）。"""
        end = time.monotonic() + seconds
        last = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last >= 1.0 / SCAN_RATE_HZ:
                s = stamp if stamp is not None else cls.node.get_clock().now().to_msg()
                cls.scan_pub.publish(
                    _make_scan_msg(cls._LaserScan, s, frame=frame, ranges=ranges))
                last = now
            cls._pump()
            time.sleep(0.001)

    def _bounded(self, pred, timeout=SCENE_TIMEOUT_S, msg=''):
        _ensure_budget('bounded wait')
        left = min(timeout, max(_remaining(), 0.0))
        ok = _ProcHarness._bounded_wait(pred, left, pump=self._pump)
        self.assertTrue(ok, msg or 'bounded wait 超时(%.1fs)' % left)
        return ok

    def _last_raw(self):
        return self.raw_msgs[-1][1] if self.raw_msgs else None

    def _last_preview(self):
        return self.preview_msgs[-1][1] if self.preview_msgs else None

    def _raw_zero_after(self, t0):
        """t0 之后收到的第一条 raw 消息（要求存在且全零）；返回 (recv_t, msg) 或 None。"""
        for tt, m in self.raw_msgs:
            if tt >= t0:
                return (tt, m) if (m.linear.x == 0.0 and m.linear.y == 0.0
                                   and m.angular.z == 0.0) else None
        return None

    def _wait_new_zero(self, t0, timeout):
        """有界等待 t0 之后的新零速 raw（暂停 scan 时验 walltimer 路径）。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._pump()
            hit = self._raw_zero_after(t0)
            if hit is not None:
                return hit
            time.sleep(0.001)
        self._pump()
        return self._raw_zero_after(t0)

    def _raw_is_zero(self):
        m = self._last_raw()
        return m is not None and (m.linear.x == 0.0 and m.linear.y == 0.0
                                  and m.angular.z == 0.0)

    def _status(self):
        code, body = _http_get('/api/status')
        self.assertEqual(code, 200)
        return json.loads(body)

    def _status_flags(self, selected=None, moving=None):
        try:
            st = self._status()
        except Exception:
            return False
        if selected is not None and st.get('is_selected') is not selected:
            return False
        if moving is not None and st.get('is_moving_enabled') is not moving:
            return False
        return True

    def _settle_zero(self, seconds=0.8):
        """恢复 fresh scan 流直至 raw 全零（有界；用于回到基线）。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._publish_scan()
            self._pump()
            if self._raw_is_zero():
                return True
            time.sleep(0.001)
        return self._raw_is_zero()

    def _goto_forward(self):
        """唯一重建正向跟踪态入口：清缓冲 → 选点+使能(200) → 持续 fresh scan
        → 分别有界等待 raw forward 与 NavCmd 正向更新（防旧帧假通过）。"""
        self.raw_msgs.clear()
        self.preview_msgs.clear()
        code, _ = _http_post('/api/set_target', '{"x":2.0,"y":0.0}')
        self.assertEqual(code, 200)
        code, _ = _http_post('/api/set_moving', '{"enabled":true}')
        self.assertEqual(code, 200)
        t_gate = time.monotonic()
        # raw forward（float64，严格 ≤0.3）
        end = time.monotonic() + SCENE_TIMEOUT_S
        last = 0.0
        raw_hit = None
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last >= 1.0 / SCAN_RATE_HZ:
                self._publish_scan()
                last = now
            self._pump()
            m = self._last_raw()
            if m is not None and m.linear.x > 0.0:
                raw_hit = time.monotonic()
                break
            time.sleep(0.001)
        self.assertIsNotNone(raw_hit, '选点+使能后未观察到 raw forward>0')
        # NavCmd 正向更新（持续 fresh scan，等待 gate 之后收到的 preview 正向帧）
        end = time.monotonic() + SCENE_TIMEOUT_S
        last = 0.0
        nav_hit = None
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last >= 1.0 / SCAN_RATE_HZ:
                self._publish_scan()
                last = now
            self._pump()
            for tt, m in self.preview_msgs:
                if tt >= t_gate and m.data.x_vel > 0.0:
                    nav_hit = m
                    break
            if nav_hit is not None:
                break
            time.sleep(0.001)
        self.assertIsNotNone(nav_hit, '未观察到 NavCmd 预览正向更新')
        return raw_hit, nav_hit

    def _reset_to_baseline(self):
        """回到基线：moving off → fresh scan → raw 零。"""
        try:
            _http_post('/api/set_moving', '{"enabled":false}')
        except Exception:
            pass
        self.assertTrue(self._settle_zero(), '回到基线（raw 零）失败')

    def _assert_no_forbidden_publishers(self):
        """graph 检查：/NAV_CMD /cmd_vel /d1_cmd 无任何 publisher（隔离域47）。"""
        for topic in FORBIDDEN_TOPICS:
            n = self.node.count_publishers(topic)
            self.assertEqual(
                n, 0,
                '发现禁止话题 %s 有 %d 个 publisher（隔离域 %d 应无任何真实控制发布者；'
                '按约定遇其它 publisher 禁止继续）' % (topic, n, ROS_DOMAIN_ID))

    # -- scenarios ----------------------------------------------------------
    def test_01_web_static_and_status(self):
        """GET / 200 且含“预览”；/app.js 200；/api/status 初始 m20_preview=true。"""
        self.assertTrue(_ProcHarness._bounded_wait(
            lambda: self._http_home_ok(), WAIT_READY_S / 2.0),
            'Web 首页未就绪或缺少“预览”标识')
        code, body = _http_get('/')
        self.assertEqual(code, 200)
        self.assertIn('预览', body)
        code, body = _http_get('/app.js')
        self.assertEqual(code, 200)
        code, body = _http_get('/api/status')
        self.assertEqual(code, 200)
        st = json.loads(body)
        self.assertIs(st.get('m20_preview'), True)

    def test_02_initial_no_selection_raw_zero(self):
        """未选点 + fresh scan 流 → raw 全零，preview 全零。"""
        self._reset_to_baseline()
        self.raw_msgs.clear()
        self.preview_msgs.clear()
        self._scan_pump(1.2)
        self.assertIsNotNone(self._last_raw(), 'fresh scan 后应收到 raw 零速消息')
        self.assertTrue(self._raw_is_zero(),
                        '未选点时 raw 应全零: %s' % (self._last_raw(),))
        m = self._last_preview()
        self.assertIsNotNone(m, '应收到 NavCmd 预览（20Hz）')
        self.assertEqual((m.data.x_vel, m.data.y_vel, m.data.yaw_vel),
                         (0.0, 0.0, 0.0))

    def test_03_select_target_and_moving_forward(self):
        """set_target(2,0)+set_moving(true)+fresh scans → raw forward>0 且严格≤0.3、
        vy=0；NavCmd 正向更新（float32 表示容差 ≤0.3+1e-6，>0，y=0）。"""
        self._reset_to_baseline()
        raw_hit, nav_hit = self._goto_forward()
        self.assertIsNotNone(raw_hit)
        m = self._last_raw()
        self.assertGreater(m.linear.x, 0.0)
        self.assertLessEqual(m.linear.x, 0.3, 'raw forward 不得超过预览上限 0.3（float64 严格）')
        self.assertEqual(m.linear.y, 0.0, 'raw vy 恒 0')
        # 使用“在使能之后到达”的 NavCmd 正向帧（_goto_forward 内有界等待所得）
        self.assertIsNotNone(nav_hit, '应观察到使能后的 NavCmd 预览正向更新')
        self.assertGreater(nav_hit.data.x_vel, 0.0, 'nav preview x_vel 应 >0')
        self.assertLessEqual(nav_hit.data.x_vel, 0.3 + 1e-6,
                             'nav preview x_vel ≤0.3+1e-6（float32 表示容差，非放宽速度契约）')
        self.assertEqual(nav_hit.data.y_vel, 0.0)

    def test_04_bad_target_inputs_400_no_crash(self):
        """坏选点 {} / 缺 y / 字符串 / 非有限 / 1e / 1..2 / 1junk → HTTP 400，
        进程不崩（之后 /api/status 仍 200）。"""
        self._reset_to_baseline()
        bad_bodies = [
            '{}',
            '{"x":2.0}',
            '{"x":"a","y":1.0}',
            '{"x":2.0,"y":NaN}',
            '{"x":2.0,"y":1e999}',
            '{"x":1e,"y":1.0}',
            '{"x":1..2,"y":1.0}',
            '{"x":1junk,"y":1.0}',
        ]
        for body in bad_bodies:
            code, _ = _http_post_expect_400('/api/set_target', body)
            self.assertEqual(code, 400, '坏选点 %r 应 HTTP 400' % body)
        code, _ = _http_get('/api/status')
        self.assertEqual(code, 200, '坏选点后 Web 服务不应崩溃')
        self.assertIsNone(self.proc_nexus.poll(),
                          'robot_nexus 不得因坏选点退出')

    def test_05_disable_moving_zero_within_02s_walltimer(self):
        """关闭 moving 后暂停 scan，≤0.2s 内由看门狗 walltimer 产生新零
        （校验接收时间在关闭请求之后，不拿缓存零假通过）。"""
        self._reset_to_baseline()
        self._goto_forward()
        self.assertGreater(self._last_raw().linear.x, 0.0,
                           '关闭前最后一条 raw 应为 forward（无旧零残留）')
        code, _ = _http_post('/api/set_moving', '{"enabled":false}')
        self.assertEqual(code, 200)
        t_close = time.monotonic()
        # 暂停发送 scan：零只能来自 walltimer（≤100ms 周期）
        hit = self._wait_new_zero(t_close, ZERO_REACTION_S)
        self.assertIsNotNone(
            hit, '关闭 moving 且暂停 scan 后 %.2fs 内未收到新零速消息（看门狗 walltimer 未生效）'
            % ZERO_REACTION_S)
        recv_t, _ = hit
        self.assertLessEqual(recv_t - t_close, ZERO_REACTION_S,
                             '零速消息应在关闭后 ≤0.2s 内到达')
        self.assertTrue(self._settle_zero())

    def test_06_disable_active_zero_walltimer_no_moving_change(self):
        """关闭 active 后暂停 scan，≤0.2s 新零（walltimer 路径）；
        代码不承诺 moving 自动 false（只速度零）：恢复 active 前显式 moving=false。"""
        self._reset_to_baseline()
        self._goto_forward()
        self.assertGreater(self._last_raw().linear.x, 0.0)
        code, _ = _http_post('/api/set_active', '{"active":false}')
        self.assertEqual(code, 200)
        t_close = time.monotonic()
        hit = self._wait_new_zero(t_close, ZERO_REACTION_S)
        self.assertIsNotNone(
            hit, '关闭 active 且暂停 scan 后 ≤%.2fs 应有新零速消息' % ZERO_REACTION_S)
        self.assertLessEqual(hit[0] - t_close, ZERO_REACTION_S)
        # 不断言 moving 自动 false（契约只承诺零速）；显式关闭避免污染后续场景
        code, _ = _http_post('/api/set_moving', '{"enabled":false}')
        self.assertEqual(code, 200)
        code, _ = _http_post('/api/set_active', '{"active":true}')
        self.assertEqual(code, 200)
        self.assertTrue(self._settle_zero())

    def test_07_target_lost_and_recovery_gating(self):
        """目标丢失（远点 scan）→ 零速 + selected=false + moving=false；
        原目标回来不自动恢复；先 moving（未选点）→ HTTP 400 拒绝 + 零；
        重新 set_target 但未 moving → 零；再次明确使能 → forward。"""
        self._reset_to_baseline()
        self._goto_forward()
        # 目标丢失：仅远点（8m）的有效 scan
        ok = _ProcHarness._bounded_wait(
            lambda: self._raw_is_zero() and self._status_flags(selected=False,
                                                               moving=False),
            SCENE_TIMEOUT_S, pump=self._pump_scan_far)
        self.assertTrue(ok, '目标丢失后应零速且 selected/moving=false')
        # 原目标回来（fresh 近点 scan）→ 不自动恢复
        self.raw_msgs.clear()
        self.preview_msgs.clear()
        self._scan_pump(1.0)
        self.assertTrue(self._raw_is_zero(),
                        '原目标回来不得自动恢复（需重新选点+使能）')
        st = self._status()
        self.assertIs(st.get('is_selected'), False,
                      '丢失后 selected 应保持 false（原目标回来也不自动置位）')
        self.assertIs(st.get('is_moving_enabled'), False)
        # 先 moving（未选点）→ 预期 HTTP 400 拒绝（反序授权被拒），随后零验证
        code, _ = _http_post_expect_400('/api/set_moving', '{"enabled":true}')
        self.assertEqual(code, 400,
                         '未选点使能 moving 应被拒绝（HTTP 400），实际 %s' % code)
        self.raw_msgs.clear()
        self._scan_pump(1.0)
        self.assertTrue(self._raw_is_zero(), '使能被拒后应保持零')
        # 重新 set_target 但未 moving → 零
        code, _ = _http_post('/api/set_moving', '{"enabled":false}')
        self.assertEqual(code, 200)
        code, _ = _http_post('/api/set_target', '{"x":2.0,"y":0.0}')
        self.assertEqual(code, 200)
        self.raw_msgs.clear()
        self._scan_pump(1.0)
        self.assertTrue(self._raw_is_zero(), '重新选点但未使能 moving 应保持零')
        # 再次明确使能（已重选点）→ 恢复 forward
        code, _ = _http_post('/api/set_moving', '{"enabled":true}')
        self.assertEqual(code, 200)
        self.assertTrue(self._wait_forward(), '重新选点+使能后应恢复 forward')
        self._reset_to_baseline()

    def _pump_scan_far(self):
        """泵 + 发布仅远点（8m）scan（触发目标丢失语义）。"""
        msg = _make_scan_msg(self._LaserScan, self.node.get_clock().now().to_msg())
        r = [float('inf')] * N_ANGLES
        idx0 = int((-0.05 - ANGLE_MIN) / ANGLE_INCREMENT)
        for k, val in enumerate(FAR_CLUSTER):
            r[(idx0 + k) % N_ANGLES] = val
        msg.ranges = r
        self.scan_pub.publish(msg)
        self._pump()

    def _wait_forward(self, timeout=SCENE_TIMEOUT_S):
        """有界等待 raw forward>0（持续 fresh scan 流）。"""
        end = time.monotonic() + timeout
        last = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last >= 1.0 / SCAN_RATE_HZ:
                self._publish_scan()
                last = now
            self._pump()
            m = self._last_raw()
            if m is not None and m.linear.x > 0.0:
                return True
            time.sleep(0.001)
        return False

    def _single_rejected_frame_then_zero(self, reason, **scan_kw):
        """公共模式：forward 已建立（最后一条为 forward，无旧零残留）→ 发送
        一条被拒帧 → 暂停 scan → ≤REJECT_REACTION_S 内观测“当帧新零+失能”
        （按接收时间判定，不拿缓存零）；随后 valid scan 仍不自动追。"""
        self._reset_to_baseline()
        self._goto_forward()
        self.assertGreater(self._last_raw().linear.x, 0.0,
                           '%s: 拒收前最后一条 raw 应为 forward' % reason)
        self.raw_msgs.clear()
        self.preview_msgs.clear()
        t_reject = time.monotonic()
        # 发送单条被拒帧，随后暂停 scan
        self._publish_scan(**scan_kw)
        hit = self._wait_new_zero(t_reject, REJECT_REACTION_S)
        self.assertIsNotNone(
            hit, '%s: 单条被拒帧后 %.2fs 内未观测到当帧新零（接收时间须 ≥ 发帧时刻）'
            % (reason, REJECT_REACTION_S))
        recv_t, _ = hit
        self.assertGreaterEqual(recv_t, t_reject)
        ok = _ProcHarness._bounded_wait(
            lambda: self._status_flags(selected=False, moving=False),
            SCENE_TIMEOUT_S, pump=self._pump)
        self.assertTrue(ok, '%s: 被拒后应失能（moving=false, selected=false）' % reason)
        # 随后 valid scan 仍不自动追
        self.raw_msgs.clear()
        self.preview_msgs.clear()
        self._scan_pump(1.0)
        self.assertTrue(self._raw_is_zero(),
                        '%s: 恢复 valid scan 不得自动续追' % reason)
        self.assertIs(self._status().get('is_selected'), False)
        self._reset_to_baseline()

    def test_08_bad_frame_single_frame_zero_disable(self):
        """frame 不符（≠lidar_link）：单条坏帧 → 当帧零 + 失能；valid scan 不自动追。"""
        self._single_rejected_frame_then_zero(
            '坏frame', frame='wrong_frame')

    def test_09_old_stamp_single_frame_zero_disable(self):
        """旧时间戳（now-1.0s）：单条 → 当帧零 + 失能；valid scan 不自动追。"""
        old = self.node.get_clock().now() - self._rclpy.duration.Duration(seconds=1.0)
        self._single_rejected_frame_then_zero('旧戳', stamp=old.to_msg())

    def test_10_future_stamp_single_frame_zero_disable(self):
        """未来时间戳（now+0.5s > future_max 0.1s）：单条坏帧 → 当帧零 + 失能。"""
        fut = self.node.get_clock().now() + self._rclpy.duration.Duration(seconds=0.5)
        self._single_rejected_frame_then_zero('未来戳', stamp=fut.to_msg())

    def test_11_scan_stop_05s_raw_zero_status_clear(self):
        """scan 停止 >0.5s → 看门狗 raw 零 + status 清（selected/moving false）。"""
        self._reset_to_baseline()
        self._goto_forward()
        # 停止发 scan，仅泵 spin；看门狗（stale >0.5s）必须零 + 清
        ok = _ProcHarness._bounded_wait(
            lambda: self._raw_is_zero() and self._status_flags(selected=False,
                                                               moving=False),
            2.5, pump=self._pump)
        self.assertTrue(ok, 'scan 停止 >0.5s 应 raw 零且 status 清除')

    def test_12_graph_no_forbidden_publishers(self):
        """graph 观察：/NAV_CMD /cmd_vel /d1_cmd 在隔离域47 无任何 publisher
        （本测试与被测节点都不创建真实控制发布者）。"""
        self._assert_no_forbidden_publishers()

    # -- teardown：先清理，不先查预算；清理错误不吞；保留日志文件 ----------
    @classmethod
    def _teardown_class(cls):
        stop_err = None
        procs = {}
        if cls.proc_nexus is not None:
            procs['robot_nexus'] = cls.proc_nexus
        if cls.proc_preview is not None:
            procs['nav_cmd_preview'] = cls.proc_preview
        if procs:
            try:
                _ProcHarness._stop_procs(procs)
            except AssertionError as e:
                stop_err = e          # 记录但不跳过自身资源清理
        node_errs = _paired_shutdown(cls.node)
        if cls.stderr_file is not None:
            try:
                cls.stderr_file.flush()
                cls.stderr_file.close()   # 保留日志文件，不删除
            except Exception as e:
                node_errs.append(e)
        if stop_err is not None:
            if node_errs:
                stop_err = AssertionError('%s; 另有节点清理错误: %s'
                                          % (stop_err, node_errs))
            raise stop_err
        _raise_if_no_inflight(node_errs)

    @classmethod
    def tearDownClass(cls):
        cls._teardown_class()


# ---------------------------------------------------------------------------
# 负向重映射（仅 domain47，绝不 domain0）
# ---------------------------------------------------------------------------
class TestPreviewNegativeGuards(unittest.TestCase):
    """负向重映射检查：预期被测进程以 exit 2 快速拒绝且未创建真实控制 publisher；
    stderr（专属 log 段，按字节偏移读取，不删除）须含拒绝语义且不是
    ImportError / exec 失败之类环境错误（避免“其他错误 → 假 PASS”）。"""

    def _run_rejected(self, name, argv, remapped_topic):
        _ensure_budget('negative ' + name)
        self.assertEqual(
            os.environ.get('ROS_DOMAIN_ID', str(ROS_DOMAIN_ID)),
            str(ROS_DOMAIN_ID),
            '负向重映射测试只允许在 domain%d 运行' % ROS_DOMAIN_ID)
        import rclpy
        rclpy.init(args=None)
        node = rclpy.create_node('new10_neg_guard_tester')
        seg_err = None
        try:
            stderr_file = open(NODE_STDERR_LOG, 'ab')
            seg_start = os.path.getsize(NODE_STDERR_LOG)
            stderr_file.write(('\n==== [%s] %s ====\n'
                               % (time.strftime('%H:%M:%S'), name)).encode('utf-8'))
            stderr_file.flush()
            proc = _ProcHarness._spawn(name, argv, stderr_file=stderr_file)
            try:
                pump = lambda: rclpy.spin_once(node, timeout_sec=PUMP_S)
                ended = _ProcHarness._bounded_wait(
                    lambda: proc.poll() is not None
                    or node.count_publishers(remapped_topic) > 0,
                    SCENE_TIMEOUT_S, pump=pump)
                saw_pub = node.count_publishers(remapped_topic) > 0
                if saw_pub:
                    _ProcHarness._stop_procs({name: proc})
                    self.fail('%s (pid=%d) 已在 %s 上创建真实 publisher：'
                              'remap guard 未拦截（按约定不继续）'
                              % (name, proc.pid, remapped_topic))
                if not ended:
                    _ProcHarness._stop_procs({name: proc})
                    self.fail('%s (pid=%d) 预期快速拒绝退出，%ds 内仍存活；'
                              'remap guard 未拦截 %s'
                              % (name, proc.pid, SCENE_TIMEOUT_S, remapped_topic))
            finally:
                try:
                    _ProcHarness._stop_procs({name: proc})
                except AssertionError as e:
                    seg_err = e
            rc = proc.returncode
            # exit 必须是 guard 语义的 2（非 0=假通过；非 2=其他错误也算未拦截）
            self.assertEqual(
                rc, 2,
                '%s 重映射 %s 应被拒绝（exit 2），实际 exit=%s；'
                '非 0 非语义退出不得放宽为通过' % (name, remapped_topic, rc))
            # 读取本进程专属 log 段（只读，不删除）：拒绝语义 + 非环境错误
            with open(NODE_STDERR_LOG, 'rb') as f:
                f.seek(seg_start)
                seg = f.read().decode('utf-8', 'replace')
            self.assertTrue(
                any(k in seg for k in REJECT_KEYWORDS),
                '%s (exit=2) stderr 缺少拒绝语义关键词 %s；segment=%r'
                % (name, REJECT_KEYWORDS, seg[-500:]))
            for bad in NOT_GUARD_KEYWORDS:
                self.assertNotIn(
                    bad, seg,
                    '%s stderr 含环境错误 %r——不得据此判定 guard 生效' % (name, bad))
            # graph 禁区：域 47 内不得残留真实控制 publisher
            for topic in FORBIDDEN_TOPICS:
                n = node.count_publishers(topic)
                self.assertEqual(n, 0,
                                 '%s 上出现 %d 个 publisher（应 0）' % (topic, n))
        finally:
            _raise_if_no_inflight(_paired_shutdown(node))
            if seg_err is not None and sys.exc_info()[0] is None:
                raise seg_err

    def test_a_nav_preview_remap_to_nav_cmd_rejected(self):
        """nav_cmd_preview 重映射 /new_chase/nav_cmd_preview:=/NAV_CMD →
        预期 exit 2 快速拒绝（argv fail-closed 扫描，不创建任何发布者）。"""
        argv = ['bash', '-c',
                CLEAN_SOURCE + 'exec ' + NAV_PREVIEW_BIN + ' --ros-args'
                ' -r /new_chase/nav_cmd_preview:=/NAV_CMD']
        self._run_rejected('nav_cmd_preview_remap', argv, '/NAV_CMD')

    def test_b_nexus_raw_remap_to_cmd_vel_rejected(self):
        """robot_nexus 重映射 /new_chase/cmd_vel_raw:=/cmd_vel →
        预期 exit 2（解析+创建后复核双拦截，stderr 含“非法/拒绝”中文语义）。"""
        argv = ['bash', '-c',
                CLEAN_SOURCE + 'exec ' + ROBOT_NEXUS_BIN + ' --ros-args'
                ' -p m20_preview:=true -p active:=true -p enable_web:=false'
                ' -r /new_chase/cmd_vel_raw:=/cmd_vel']
        self._run_rejected('robot_nexus_remap', argv, '/cmd_vel')


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def _main():
    if os.environ.get('ROS_DOMAIN_ID') != str(ROS_DOMAIN_ID):
        sys.stderr.write(
            'NEW-10 ROS 集成测试必须显式以隔离域运行（绝不 domain0）:\n'
            '  ROS_DOMAIN_ID=%d NEW10_ALLOW_RUN=1 python3 -B %s\n'
            % (ROS_DOMAIN_ID, os.path.abspath(__file__)))
        return 2
    if os.environ.get('NEW10_ALLOW_RUN') != '1':
        sys.stderr.write(
            '本测试由主代理在功能/导出编码停止后运行：'
            '需显式 NEW10_ALLOW_RUN=1（防止误启动被测进程）。\n')
        return 2
    # 主进程本身强制 approved UDP profile / RMW / 新 install 路径
    # （避免主进程意外使用 SHM 500MB profile；导入来自 Foxy+新 install）。
    _force_main_env()
    # 缺环境不 Skip 伪通过：预检导入，失败 exit 2
    try:
        import rclpy  # noqa: F401
        from drdds.msg import NavCmd  # noqa: F401
        from geometry_msgs.msg import Twist  # noqa: F401
        from sensor_msgs.msg import LaserScan  # noqa: F401
    except ImportError as e:
        sys.stderr.write('NEW-10 ROS 测试缺环境（%s）——按约定 exit 2，不伪通过\n' % e)
        return 2
    unittest.main(module=__name__, argv=[sys.argv[0], '-v', '-b'])
    return 0


def _force_main_env():
    """为测试主进程强制清洁/隔离 env（UDP-only profile、fastrtps、无字节码）。"""
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = FASTRTPS_PROFILE
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    ros_py = ['/opt/ros/foxy/lib/python3.8/site-packages',
              '/opt/ros/foxy/local/lib/python3.8/dist-packages',
              NEW_CHASE_ROOT + '/install/jie_deamon/local/lib/python3.8/dist-packages']
    old = os.environ.get('PYTHONPATH', '')
    for p in reversed(ros_py):
        if p not in old.split(':'):
            old = p + ':' + old
    os.environ['PYTHONPATH'] = old
    ld = os.environ.get('LD_LIBRARY_PATH', '')
    for p in ('/opt/ros/foxy/opt/rviz_foxy/lib', '/opt/ros/foxy/lib',
              NEW_CHASE_ROOT + '/install/jie_deamon/lib'):
        if p not in ld.split(':'):
            ld = ld + ':' + p if ld else p
    os.environ['LD_LIBRARY_PATH'] = ld
    ament = os.environ.get('AMENT_PREFIX_PATH', '')
    for p in ('/opt/ros/foxy', NEW_CHASE_ROOT + '/install/jie_deamon'):
        if p not in ament.split(':'):
            ament = p + ':' + ament if ament else p
    os.environ['AMENT_PREFIX_PATH'] = ament


if __name__ == '__main__':
    sys.exit(_main())
