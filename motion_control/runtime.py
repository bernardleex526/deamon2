"""motion_control.runtime — 有界 dryrun/live 运行入口（NEW-RUNTIME 本轮新增）。

入口: python3 -B -m motion_control.runtime --seconds 20
- 默认 dryrun：MotionGuard(output_enabled=True) 但绝不 arm；sender enabled=False
  → 绝不创建 /NAV_CMD publisher、绝不发任何速度（连零速都不发）；evaluate 显示
  真实阻断原因，actual output_enabled=false 另行报告；严禁因 debug arm 变 real。
- live 需 --live --commissioned --exclusive-control-confirmed 三旗 + 遥测开启 +
  ROS_DOMAIN_ID==0；arm 仅经 /new_chase/arm 显式人工请求（绝不自动 arm）；
  live 窗口=首次 arm 成功起单调 10s（含边界 >=）；首次失效/发送异常/人工
  disarm/到期/所有权 fatal → 本进程 session 永久锁死，re-arm 拒绝，重启=重新授权。
- 进程环境 fail-closed 校验（批准 UDP-only profile 路径+XML 无 SHM+白名单+
  RMW+域），防 500MB SHM 配置；ROS 导入只在 main()（不 import 即无网络）。
- 主循环名义 20Hz 精确 deadline 调度（工作超时则顺延，不忙转/不追发）；每周期
  批量 spin_once(0) 非阻塞 drain（≤16 次、累计 ≤5ms 开始下一次；rclpy 合同下
  单次回调执行不可抢占，长回调仅使下一周期 deadline 顺延）；
  HTTP GET 由独立有界 worker 线程执行（主循环只读不可变缓存，永不阻塞）；
  输入收完再以新鲜单调时刻 tick；遥测 fatal 事件（decode/recv/heartbeat/
  comm_fault/错误来源）→ 立即 disarm，live armed_once → 立即 session lock。
- 隔离测试经 Controller 注入 fake，不经 CLI。详见 RUNTIME.md。
"""

import json
import math
import os
import sys
import threading
import time
import uuid

from motion_control.core import Decision, GuardedOutput, MotionGuard, Velocity, ZERO
from motion_control.inputs import (
    WEB_HTTP_TIMEOUT, check_scan, fetch_web_status, parse_web_status,
    target_generation)
from motion_control.nav_sender import NavCmdSender
from motion_control.telemetry import AOS_HOST, AOS_PORT, TelemetryClient

__all__ = ["Controller", "RuntimeArgsError", "parse_runtime_args",
           "check_process_env", "validate_approved_profile_xml", "WebStatusWorker",
           "main", "DIAG_TOPIC", "ARM_SERVICE", "SCAN_TOPIC", "RAW_TOPIC",
           "LIVE_SESSION_SECONDS", "RUN_PERIOD", "APPROVED_FASTRTPS_PROFILE",
           "SPIN_BATCH_MAX", "SPIN_BATCH_BUDGET"]

DIAG_TOPIC = '/new_chase/motion_guard_status'
ARM_SERVICE = '/new_chase/arm'
SCAN_TOPIC = '/new_chase/scan'
RAW_TOPIC = '/new_chase/cmd_vel_raw'
NODE_PREFIX = 'motion_runtime_'    # + 随机 hex 后缀：专属随机节点名（防同名误排）
LIVE_SESSION_SECONDS = 10.0        # live 窗口：首次 arm 成功起 10s（单调，>= 锁）
WEB_MIN_INTERVAL = 0.1             # worker GET ≤10Hz；timeout 0.1（主循环不阻塞）
DIAG_PERIOD = 1.0                  # 诊断 JSON 1Hz（发布 + stdout）
RUN_PERIOD = 0.05                  # 名义 20Hz；overrun 顺延
OWNERSHIP_CHECK_INTERVAL = 0.1     # 图核查 ≤0.1s（非 1s）
SPIN_BATCH_MAX = 16                # 每周期最多 spin_once(0) 批处理次数（有界 drain）
SPIN_BATCH_BUDGET = 0.005          # 批处理累计预算 5ms（开始下一次前检查）
LIVE_DISCOVERY_WAIT = 3.0          # live 启动前未发现态的有界等待上限（秒）
DISCOVERY_POLL = 0.05              # 发现等待查询间隔（50ms sleep）
APPROVED_FASTRTPS_PROFILE = '/home/user/new_chase/config/diagnostic_udp_only.xml'
REQUIRED_RMW = 'rmw_fastrtps_cpp'
APPROVED_INTERFACE_WHITELIST = ('10.21.31.104', '127.0.0.1')
DRY_ALLOWED_DOMAINS = ('0', '47')  # 47 仅 dryrun 隔离测试
FATAL_TELEMETRY_KINDS = frozenset(
    {'decode_error', 'recv_error', 'heartbeat_error', 'comm_fault'})


class RuntimeArgsError(ValueError):
    pass


def parse_runtime_args(argv):
    """fail-closed 手工解析：仅允许列出的旗标；remap(':=')/未知项/位置参数 → 异常。"""
    opts = {'seconds': 20, 'live': False, 'commissioned': False,
            'exclusive': False, 'telemetry': True, 'motion_seconds': LIVE_SESSION_SECONDS}
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '--seconds':
            if i + 1 >= len(argv):
                raise RuntimeArgsError('--seconds missing value')
            try:
                opts['seconds'] = int(argv[i + 1])
            except ValueError:
                raise RuntimeArgsError('--seconds must be int: %r' % (argv[i + 1],))
            i += 2
        elif arg == '--motion-seconds':
            if i + 1 >= len(argv):
                raise RuntimeArgsError('--motion-seconds missing value')
            try:
                value = float(argv[i + 1])
            except ValueError:
                raise RuntimeArgsError('--motion-seconds must be finite in (0,10]')
            if not math.isfinite(value) or not 0 < value <= LIVE_SESSION_SECONDS:
                raise RuntimeArgsError('--motion-seconds must be finite in (0,10]')
            opts['motion_seconds'] = value
            i += 2
        elif arg == '--live':
            opts['live'] = True; i += 1
        elif arg == '--commissioned':
            opts['commissioned'] = True; i += 1
        elif arg == '--exclusive-control-confirmed':
            opts['exclusive'] = True; i += 1
        elif arg == '--no-telemetry':
            opts['telemetry'] = False; i += 1
        elif ':=' in arg:
            raise RuntimeArgsError('remap-like rule %r rejected' % (arg,))
        else:
            raise RuntimeArgsError('unknown arg %r (allowed: --seconds/--live/'
                                   '--commissioned/--exclusive-control-confirmed/'
                                   '--no-telemetry)' % (arg,))
    if opts['live']:
        if not (opts['commissioned'] and opts['exclusive']):
            raise RuntimeArgsError('live requires --live --commissioned '
                                   '--exclusive-control-confirmed together')
        if not opts['telemetry']:
            raise RuntimeArgsError('live forbids --no-telemetry (BasicStatus '
                                   'push requires heartbeat registration)')
        if not 20 <= opts['seconds'] <= 120:
            raise RuntimeArgsError('live --seconds bounded 20..120')
    elif not 10 <= opts['seconds'] <= 120:
        raise RuntimeArgsError('dryrun --seconds bounded 10..120')
    return opts


def validate_approved_profile_xml(path):
    """校验批准的 UDP-only Fast-DDS profile（stdlib ElementTree，只读）。

    要求：根 profiles（官方命名空间）、恰 1 个 UDPv4 transport_descriptor
    （绝不接受 SHM，防 500MB 段）、interfaceWhiteList 精确 {127.0.0.1,
    10.21.31.104}、恰 1 个 default participant、userTransports 恰 [udp_transport]、
    useBuiltinTransports=false。返回错误 str 或 None。
    """
    import xml.etree.ElementTree as ET
    ns = '{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    try:
        root = ET.parse(path).getroot()
    except Exception as e:
        return 'xml parse failed: %r' % (e,)
    if root.tag != ns + 'profiles':
        return 'root element is not profiles'
    descriptors = root.findall('.//%stransport_descriptor' % ns)
    if len(descriptors) != 1:
        return 'exactly 1 transport_descriptor required, got %d' % len(descriptors)
    if descriptors[0].findtext(ns + 'type') != 'UDPv4':
        return 'transport type must be UDPv4 (SHM forbidden)'
    addresses = [a.text for a in descriptors[0].findall(
        '%sinterfaceWhiteList/%saddress' % (ns, ns))]
    if sorted(addresses or []) != list(APPROVED_INTERFACE_WHITELIST):
        return 'interface whitelist must be exactly %r, got %r' % (
            APPROVED_INTERFACE_WHITELIST, addresses)
    participants = root.findall('%sparticipant' % ns)
    if len(participants) != 1:
        return 'exactly 1 participant required, got %d' % len(participants)
    if participants[0].get('is_default_profile') != 'true':
        return 'participant must be is_default_profile=true'
    user_transports = [t.text for t in participants[0].findall(
        '%srtps/%suserTransports/%stransport_id' % (ns, ns, ns))]
    if user_transports != ['udp_transport']:
        return 'userTransports must be exactly [udp_transport], got %r' % (
            user_transports,)
    use_builtin = participants[0].findtext('%srtps/%suseBuiltinTransports' % (ns, ns))
    if use_builtin != 'false':
        return 'useBuiltinTransports must be false, got %r' % (use_builtin,)
    return None


def check_process_env(live):
    """进程环境 fail-closed 校验（防 500MB SHM 配置）。返回错误 str 或 None。"""
    domain = os.environ.get('ROS_DOMAIN_ID')
    if live:
        if domain != '0':
            return 'live requires ROS_DOMAIN_ID==0, got %r' % (domain,)
    elif domain not in DRY_ALLOWED_DOMAINS:
        return 'dryrun ROS_DOMAIN_ID must be 0 or 47 (isolation), got %r' % (domain,)
    rmw = os.environ.get('RMW_IMPLEMENTATION')
    if rmw != REQUIRED_RMW:
        return 'RMW_IMPLEMENTATION must be %r, got %r' % (REQUIRED_RMW, rmw)
    profile = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    if profile != APPROVED_FASTRTPS_PROFILE:
        return 'FASTRTPS_DEFAULT_PROFILES_FILE must be exactly %r, got %r' % (
            APPROVED_FASTRTPS_PROFILE, profile)
    error = validate_approved_profile_xml(profile)
    if error is not None:
        return 'approved profile invalid: %s' % error
    return None


class WebStatusWorker:
    """独立有界 HTTP GET worker：缓存不可变解析结果 + 完成时刻（单调秒）。

    主循环只读 latest()，永不阻塞；仅新结果（seq 变化）由调用方应用，
    不重复刷新 tracking 时间。stop()+join(timeout) 有限回收，不线程泄漏；
    fetch 自身超时 ≤0.1s，线程单次阻塞有界。
    """

    def __init__(self, fetch, period=WEB_MIN_INTERVAL, monotonic=time.monotonic):
        self._fetch = fetch
        self._period = period
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._latest = None            # (parsed, completed_at, seq)
        self._pending_invalid = None   # 至少保留一次尚未消费的失败，防好结果覆盖
        self._seq = 0
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name='web-status-worker',
                                        daemon=True)  # daemon：join 失败不挂进程
        self._thread.start()

    def _run(self):
        while not self._stop_event.is_set():
            started = self._monotonic()
            try:
                text = self._fetch()
            except Exception as e:
                parsed = {'valid': False, 'reason': 'http_error:%s' % type(e).__name__,
                          'flags': None, 'target': None, 'm20_preview': None}
            else:
                parsed = parse_web_status(text)
            completed = self._monotonic()
            with self._lock:
                self._seq += 1
                self._latest = (parsed, completed, self._seq)
                if not parsed['valid']:
                    self._pending_invalid = self._latest
            self._stop_event.wait(max(0.0, self._period - (self._monotonic() - started)))

    def latest(self):
        with self._lock:
            return self._latest

    def take_pending(self):
        """原子取出最多两个结果：未消费失败优先，然后最新结果。

        有界内存，不积压队列；即使主循环暂时滞后，好结果也不能抹掉
        worker 已观察到的失败。重复最新结果仍由 Controller 的 seq 去重。
        """
        with self._lock:
            invalid, latest = self._pending_invalid, self._latest
            self._pending_invalid = None
            if invalid is not None and invalid != latest:
                return (invalid, latest)
            return (latest,) if latest is not None else ()

    def stop(self, join_timeout=1.0):
        """幂等停止；返回 (already_stopped, joined)。"""
        already = self._thread is None
        self._stop_event.set()
        thread, self._thread = self._thread, None
        joined = True
        if thread is not None and thread.is_alive():
            thread.join(timeout=join_timeout)
            joined = not thread.is_alive()
        return already, joined


class Controller:
    """纯注入控制器：guard/sender/telemetry/web_fetch/graph_probe 全可注入，
    不触 core 私有变量。构造即校验 live == sender.enabled（精确 bool）：
    dryrun 绝不允许 sender enabled=True（堵 guard 外部 arm 绕过输出）。
    dryrun 契约：guard output_enabled=True（evaluate 显示真实阻断原因）但绝不
    arm；GuardedOutput 从未放行时连零速都不发。"""

    def __init__(self, *, guard, sender, telemetry=None, web_fetch=None,
                 monotonic=None, live=False, graph_probe=None,
                 session_seconds=LIVE_SESSION_SECONDS):
        if not isinstance(guard, MotionGuard):
            raise TypeError('guard must be MotionGuard')
        if not isinstance(sender, NavCmdSender):
            raise TypeError('sender must be NavCmdSender')
        if not isinstance(live, bool):
            raise TypeError('live must be exact bool')
        if live != sender.enabled:
            raise ValueError('live(%r) must exactly match sender.enabled(%r)' % (
                live, sender.enabled))
        if (type(session_seconds) not in (int, float)
                or not math.isfinite(session_seconds)
                or not 0 < session_seconds <= LIVE_SESSION_SECONDS):
            raise ValueError('session_seconds must be finite in (0,10]')
        self._session_seconds = float(session_seconds)
        self._guard, self._sender, self._telemetry = guard, sender, telemetry
        self._web_fetch = web_fetch
        self._monotonic = monotonic if monotonic is not None else time.monotonic
        self._live = live
        self._graph_probe = graph_probe
        self._output = GuardedOutput(guard, self._counting_send)
        self._session_locked = self._armed_once = False
        self._lock_reason = self._deadline = None
        self._prev_target = None
        self._web_seq_consumed = 0
        self.pub_confirmed = False      # sender 曾成功构建并发布（pub 存在证据）
        self.send_calls = self.send_errors = 0
        self.scan_rx = self.scan_valid = self.scan_clear = 0
        self.raw_rx = self.web_rx = self.web_ok = 0
        self.web_target_gen = 0
        self.scan_last_reason = self.web_last_reason = None
        self.web_flags = self.web_target = None
        self.ownership = {'nav_foreign_publishers': None, 'raw_publishers': None}
        self.live_block_reason = None
        self.last_decision = self.last_decision_at = None

    def _counting_send(self, velocity):
        result = self._sender(velocity)
        self.send_calls += 1
        if result:
            self.pub_confirmed = True   # 懒 publisher 创建成功的存在证据
        return result

    # ---- 输入入口 ----
    def on_scan(self, scan, mono_arrival, wall_now, mono_now=None):
        """scan 回调：core 校验 + 适配层 recv_age 门控（不改 core）。"""
        mono_now = self._monotonic() if mono_now is None else mono_now
        self.scan_rx += 1
        result = check_scan(scan, wall_now, recv_age=mono_now - mono_arrival)
        self.scan_last_reason = result['reason']
        self.scan_valid += 1 if result['valid'] else 0
        self.scan_clear += 1 if result['clear'] else 0
        # source_age 已在 inputs 综合为 max(0, 墙龄, recv_age)（保守、≥0）
        self._guard.update_scan(result['valid'], result['clear'],
                                result['source_age'], mono_now)

    @staticmethod
    def _conv(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        try:
            value = float(value)
        except (OverflowError, ValueError):
            return None
        return value if math.isfinite(value) else None

    @classmethod
    def _safe_twist(cls, msg):
        """Twist 三分量（x/y/yaw）全部校验；任一非法 → 该分量 NaN → core 判
        command_invalid（大整数绝不溢出 core，坏指令绝不降级为零速放行）。"""
        try:
            cx, cy, yaw = (cls._conv(msg.linear.x), cls._conv(msg.linear.y),
                           cls._conv(msg.angular.z))
        except (AttributeError, TypeError):
            cx = cy = yaw = None
        return Velocity(float('nan') if cx is None else cx,
                        float('nan') if cy is None else cy,
                        float('nan') if yaw is None else yaw)

    def on_raw(self, msg, mono_now=None):
        """raw Twist 回调（三分量安全有限转换；NaN → core command_invalid）。"""
        mono_now = self._monotonic() if mono_now is None else mono_now
        self.raw_rx += 1
        self._guard.update_command(self._safe_twist(msg), mono_now)

    # ---- web（worker 消费 / 注入直取，共享 _apply_web） ----
    def consume_web(self, worker):
        """读 worker 缓存；仅新结果（seq 变化）应用，绝不重复刷新 tracking。"""
        if worker is None:
            return
        items = (worker.take_pending() if hasattr(worker, 'take_pending')
                 else (worker.latest(),))  # 保留纯内存 latest 替身接口
        for item in items:
            if item is None:
                continue
            parsed, completed_at, seq = item
            if seq <= self._web_seq_consumed:
                continue
            self._web_seq_consumed = seq
            self._apply_web(parsed, completed_at)

    def poll_web(self, now=None):
        """tester 注入路径：直接 fetch+parse+apply（fetch 异常 → invalid）。"""
        if self._web_fetch is None:
            return
        try:
            text = self._web_fetch()
        except Exception as e:
            self._apply_web({'valid': False,
                             'reason': 'http_error:%s' % type(e).__name__,
                             'flags': None, 'target': None, 'm20_preview': None},
                            self._monotonic())
            return
        self._apply_web(parse_web_status(text), self._monotonic())

    def _apply_web(self, parsed, completed_at):
        self.web_rx += 1
        self.web_last_reason = parsed['reason']
        if parsed['valid']:
            self.web_ok += 1
            self.web_flags = dict(parsed['flags'])
            self.web_target = parsed['target']
            self.web_target_gen, _ = target_generation(
                self._prev_target, self.web_target_gen, parsed['target'])
            self._prev_target = parsed['target']
        else:
            self.web_flags = parsed['flags']
            self.web_target = None
        self._guard.update_tracking(bool(parsed['valid']), completed_at)

    # ---- 遥测（fatal 事件 → 立即 disarm；live armed_once → 立即 lock） ----
    def poll_telemetry(self, now=None):
        mono_now = self._monotonic() if now is None else now
        if self._telemetry is None:
            return []
        events = self._telemetry.poll(mono_now)
        for event in events:
            kind = event.get('kind')
            if kind == 'status':
                self._guard.update_status(event['status'], event['received_at'])
            elif kind in FATAL_TELEMETRY_KINDS:
                self._guard.disarm()
                self._lock_telemetry_fatal(kind)
        return events

    def _lock_telemetry_fatal(self, kind):
        if self._live and self._armed_once and not self._session_locked:
            self._session_locked = True
            self._lock_reason = 'telemetry_%s' % kind

    # ---- 所有权（probe 只记录；fatal 处置立即 disarm + live lock） ----
    def probe_ownership(self):
        """ROS 图核查（注入 callable）：/NAV_CMD 他方 publisher 数 + raw 数。
        自身 publisher 仅在 pub_confirmed 且图中找到专属随机节点名时减 1；
        纯同名排除禁止（专属随机名撞名概率与限制记录于 RUNTIME.md）。"""
        if self._graph_probe is not None:
            try:
                nav_foreign, raw_count = self._graph_probe()
                # 原样记录（不 int() 强转）：strict type 判定由
                # check_ownership_fatal / request_arm 执行（'0' 不是合法计数）
                self.ownership = {'nav_foreign_publishers': nav_foreign,
                                  'raw_publishers': raw_count}
            except Exception as e:
                self.ownership = {'nav_foreign_publishers':
                                  'probe_error:%s' % (type(e).__name__,),
                                  'raw_publishers': None}
        return self.ownership

    def check_ownership_fatal(self, now=None):
        """≤0.1s 频率调用：fatal（nav!=0 / raw!=1 / probe error）→ disarm；
        live armed_once → 立即 session lock。返回 (ownership, fatal)。"""
        ownership = self.probe_ownership()
        nav = ownership.get('nav_foreign_publishers')
        raw = ownership.get('raw_publishers')
        fatal = (not (type(nav) is int and nav == 0)
                 or not (type(raw) is int and raw == 1))
        if fatal:
            self.live_block_reason = 'ownership_fatal: %r' % (ownership,)
            self._guard.disarm()
            if self._live and self._armed_once and not self._session_locked:
                self._session_locked = True
                self._lock_reason = self.live_block_reason
        elif not self._session_locked:
            # 当前健康（nav==0 且 raw==1）→ 清除旧阻断原因；
            # session_locked 后的历史 reason 绝不清（保留证据）
            self.live_block_reason = None
        return ownership, fatal

    # ---- arm / disarm（arm 服务唯一入口；绝不自动 arm、绝不自动恢复） ----
    def request_arm(self, want, now=None):
        """→ (success, message)。dryrun 永拒 want=True；live 只允许首次 arm。"""
        mono_now = self._monotonic() if now is None else now
        if not isinstance(want, bool):
            return False, 'arm request must be bool'
        if not want:  # 显式人工 disarm：即刻 flush 一次零速（fresh now）
            was_armed = self._armed_once
            self._guard.disarm()
            if self._live and was_armed:
                if not self._session_locked:
                    self._session_locked = True
                    self._lock_reason = 'manual_disarm'
                try:
                    self._output.tick(self._monotonic())
                except Exception as e:
                    self.send_errors += 1
                    self._lock_reason = 'send_error:%s' % type(e).__name__
            return True, 'disarmed'
        if self._session_locked or self._armed_once:
            if self._lock_reason is None:
                self._lock_reason = 'session_locked: cannot re-arm to refresh 10s'
            return False, self._lock_reason
        if not self._live:
            return False, 'dryrun: arm rejected (output stays disabled)'
        if self._sender.enabled is not True:
            return False, 'sender_not_enabled'
        ownership = self.probe_ownership()
        nav_foreign = ownership.get('nav_foreign_publishers')
        raw_count = ownership.get('raw_publishers')
        if type(nav_foreign) is not int:
            self._session_locked = True
            self._lock_reason = 'ownership_probe_failed: %r' % (nav_foreign,)
            return False, self._lock_reason
        if nav_foreign != 0:
            self._session_locked = True
            self._lock_reason = ('ownership: %d foreign /NAV_CMD publisher(s)'
                                 % nav_foreign)
            return False, self._lock_reason
        if type(raw_count) is not int or raw_count != 1:
            self._session_locked = True
            self._lock_reason = ('ownership: /new_chase/cmd_vel_raw publishers=%r '
                                 '(need exactly 1)' % (raw_count,))
            return False, self._lock_reason
        # 每次显式 arm 请求即人工确认独占控制（core bad-input 会清 exclusive）
        self._guard.set_exclusive_control(True)
        decision = self._guard.arm(mono_now)
        if not decision.allowed:
            return False, 'arm_rejected: %s' % decision.reason
        self._armed_once = True
        self._deadline = mono_now + self._session_seconds
        return True, 'armed; live window = %.3fs from now' % self._session_seconds

    # ---- 主循环 tick ----
    def tick(self, now=None):
        """门控评估 + 发送边界；deadline 用 >= 边界；发送异常同 tick 立即
        session lock（合成 blocked decision，不把异常留到下一 tick）。"""
        mono_now = self._monotonic() if now is None else now
        if (self._live and not self._session_locked
                and self._deadline is not None and mono_now >= self._deadline):
            self._session_locked = True
            self._lock_reason = 'deadline_expired'
            self._guard.disarm()
        try:
            decision = self._output.tick(mono_now)
        except Exception as e:
            self._guard.disarm()
            self.send_errors += 1
            decision = Decision(ZERO, False, False, 'send_error:%s' % type(e).__name__)
            if self._live and self._armed_once and not self._session_locked:
                self._session_locked = True
                self._lock_reason = decision.reason
        if (self._live and self._armed_once and not self._session_locked
                and not decision.allowed):
            # 首次失效（输入坏/过期/ disarm）→ 本进程永久锁死
            self._session_locked = True
            self._lock_reason = 'first_invalid: %s' % decision.reason
        self.last_decision = decision
        self.last_decision_at = mono_now
        return decision

    def disarm_for_input_error(self, source, exc):
        """回调意外异常保守处置：disarm（清 exclusive）、记录原因，不崩溃。"""
        self._guard.disarm()
        self._lock_reason = 'input_error:%s:%r' % (source, exc)

    def shutdown_output(self, now=None):
        """有序停止：disarm 后驱动一次 output.tick（live 曾放行只补一次零速；
        dryrun 从未放行 → 绝不发送任何东西）。"""
        mono_now = self._monotonic() if now is None else now
        self._guard.disarm()
        return self._output.tick(mono_now)

    def snapshot(self, now=None):
        """诊断/总结快照（经 _jsonable 保证 JSON 可序列化）。"""
        mono_now = self._monotonic() if now is None else now
        decision = self.last_decision
        seconds_left = None
        if self._deadline is not None and not self._session_locked:
            seconds_left = max(0.0, self._deadline - mono_now)
        return _jsonable({
            'mode': 'live' if self._live else 'dryrun',
            'output_enabled': bool(self._sender.enabled),
            'pub_confirmed': bool(self.pub_confirmed),
            'decision': {
                'reason': decision.reason if decision else None,
                'allowed': bool(decision.allowed) if decision else False,
                'armed': bool(decision.armed) if decision else False,
                'velocity': [decision.velocity.x, decision.velocity.y,
                             decision.velocity.yaw] if decision else [0.0, 0.0, 0.0],
            },
            'session': {'locked': bool(self._session_locked),
                        'reason': self._lock_reason,
                        'seconds_left': seconds_left,
                        'armed_once': bool(self._armed_once)},
            'status': {'count': getattr(self._telemetry, 'status_count', 0) or 0,
                       'last': getattr(self._telemetry, 'last_status', None),
                       'received_at': getattr(self._telemetry,
                                              'status_received_at', None),
                       'last_error': getattr(self._telemetry, 'last_error', None)},
            'scan': {'rx': self.scan_rx, 'valid': self.scan_valid,
                     'clear': self.scan_clear, 'last_reason': self.scan_last_reason},
            'web': {'rx': self.web_rx, 'ok': self.web_ok, 'flags': self.web_flags,
                    'target': self.web_target, 'generation': self.web_target_gen,
                    'last_reason': self.web_last_reason},
            'raw': {'rx': self.raw_rx},
            'sender': {'calls': self.send_calls, 'errors': self.send_errors,
                       'enabled': bool(self._sender.enabled)},
            'ownership': dict(self.ownership),
            'live_block_reason': self.live_block_reason,
        })


def _jsonable(obj):
    """JSON 安全化：非有限浮点→None；未知类型→str。"""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    return obj if obj is None or isinstance(obj, (bool, int, str)) else str(obj)


def _spin_batch(executor, budget_deadline, monotonic):
    """单线程非阻塞批量 drain：每次 spin_once(timeout_sec=0) 处理一个已就绪
    回调；最多 SPIN_BATCH_MAX 次且累计 SPIN_BATCH_BUDGET 内开始下一次。
    局限（rclpy 合同，如实声明）：单个回调执行不可抢占，长回调可能超出预算，
    只能影响下一周期 deadline（顺延），不停栈不忙转不无限 drain。"""
    for _ in range(SPIN_BATCH_MAX):
        if monotonic() >= budget_deadline:
            break
        executor.spin_once(timeout_sec=0)


def _wait_live_discovery(controller, monotonic=time.monotonic, stop_check=None):
    """live 启动前对"未发现态"（nav==0 且 raw==0）的有界等待：每 50ms sleep
    后再查询，最长 LIVE_DISCOVERY_WAIT=3s；通过条件 = nav==0 且 raw==1。
    任何 nav>0 / raw>1 / probe 异常 / 非法类型 → 立即拒绝（不等待）；
    3s 后仍 raw==0 → 拒绝。不创建 NAV publisher、不 arm、不动服务、不改 mode。
    ⚠️ 有界等待 ≠ 运动授权：ownership/exclusive 仍必须人工确认。
    SIGINT（sleep 中断）→ KeyboardInterrupt 正常 finally 退出；rclpy 停止 →
    stop_check 置真后返回 fatal（正常回收）。返回 fatal bool。"""
    end = monotonic() + LIVE_DISCOVERY_WAIT
    while True:
        if stop_check is not None and stop_check():
            return True   # 停止优先于发现完成；禁止停止后继续初始化发送路径
        ownership = controller.probe_ownership()
        nav = ownership.get('nav_foreign_publishers')
        raw = ownership.get('raw_publishers')
        if not (type(nav) is int and type(raw) is int):
            return True   # probe 异常/非法类型：立即拒绝，不等待
        if nav != 0 or raw not in (0, 1):
            return True   # 已发现他方 publisher / 多 raw 源：立即拒绝
        if raw == 1:
            return False  # raw1 nav0：发现完成，通过
        if monotonic() >= end:
            return True   # 3s 内 raw==0 仍未见 robot_nexus → 拒绝
        time.sleep(DISCOVERY_POLL)


def _print_diag(snapshot):
    print(json.dumps(snapshot, ensure_ascii=False, allow_nan=False,
                     separators=(',', ':'), default=str), flush=True)


def main(args=None):
    """独立有界入口；环境校验先于 ROS import；rclpy.init(args=[])（无 remap）；
    finally 全量资源回收；summary 与最终 exit code 一致。"""
    argv = list(args if args is not None else sys.argv[1:])
    try:
        opts = parse_runtime_args(argv)
    except RuntimeArgsError as e:
        sys.stderr.write('motion_runtime refuse start (exit 2): %s\n' % (e,))
        return 2
    env_error = check_process_env(opts['live'])
    if env_error is not None:
        sys.stderr.write('motion_runtime refuse start (exit 2): %s\n' % (env_error,))
        return 2

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    import uuid
    node_name = NODE_PREFIX + uuid.uuid4().hex[:8]

    node = executor = telemetry = sender = controller = worker = None
    exit_code = 0
    cleanup_errors = []
    try:
        rclpy.init(args=[])  # 显式空 argv：无任何重映射（use_global_arguments 亦关）
        node = Node(node_name, use_global_arguments=False, cli_args=[],
                    start_parameter_services=False, enable_rosout=False)
        sender = NavCmdSender(node=node, enabled=opts['live'], message_type=None)
        if opts['telemetry']:
            telemetry = TelemetryClient(AOS_HOST, AOS_PORT)
        controller = Controller(
            guard=MotionGuard(output_enabled=True), sender=sender,
            telemetry=telemetry,
            web_fetch=None, monotonic=time.monotonic, live=opts['live'],
            session_seconds=opts['motion_seconds'])

        def graph_probe():
            infos = node.get_publishers_info_by_topic('/NAV_CMD')
            foreign = len(infos)
            if controller.pub_confirmed:
                # 仅专属随机节点名可作为自身证据减 1；同名碰撞概率与限制记录在案
                foreign -= min(1, sum(1 for info in infos
                                      if info.node_name == node_name))
            return foreign, len(node.get_publishers_info_by_topic(RAW_TOPIC))
        controller._graph_probe = graph_probe

        worker = WebStatusWorker(
            fetch=lambda: fetch_web_status(timeout=WEB_HTTP_TIMEOUT))
        worker.start()

        # 启动前所有权核查：dryrun 保持原单次核查（记录不等待）；live 改为
        # 对"未发现态"（nav==0 且 raw==0）做 ≤3s 有界发现等待，fatal 直接拒绝
        if opts['live']:
            fatal = _wait_live_discovery(
                controller, stop_check=lambda: not rclpy.ok())
        else:
            ownership, fatal = controller.check_ownership_fatal(time.monotonic())
        if opts['live'] and fatal:
            ownership = controller.probe_ownership()
            raise RuntimeError('live refuse: ownership fatal at startup: %r '
                               '(ROS graph does not prove planner stopped; '
                               'operator must confirm planner inactive and '
                               'charge idle on site)' % (ownership,))

        def on_scan(msg):
            try:
                controller.on_scan(msg, time.monotonic(), time.time())
            except Exception as e:
                controller.disarm_for_input_error('scan', e)

        def on_raw(msg):
            try:
                controller.on_raw(msg)
            except Exception as e:
                controller.disarm_for_input_error('raw', e)

        def on_arm(request, response):
            try:
                success, message = controller.request_arm(bool(request.data))
            except Exception as e:
                success, message = False, 'arm_service_error: %r' % (e,)
            response.success = bool(success)
            response.message = str(message)[:250]
            return response

        node.create_subscription(LaserScan, SCAN_TOPIC, on_scan,
                                 qos_profile_sensor_data)
        node.create_subscription(Twist, RAW_TOPIC, on_raw, 1)
        node.create_service(SetBool, ARM_SERVICE, on_arm)
        diag_pub = node.create_publisher(String, DIAG_TOPIC, 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        sys.stderr.write(
            'motion_runtime: mode=%s seconds=%d telemetry=%s sender_enabled=%s '
            '(dryrun never moves; live window=%.3fs from first arm; exit0!=movable)\n'
            % ('live' if opts['live'] else 'dryrun', opts['seconds'],
               bool(opts['telemetry']), bool(sender.enabled), opts['motion_seconds']))
        t_end = time.monotonic() + opts['seconds']
        next_deadline = time.monotonic() + RUN_PERIOD
        last_probe = last_diag = 0.0
        while time.monotonic() < t_end:
            if not rclpy.ok():
                break
            _spin_batch(executor, time.monotonic() + SPIN_BATCH_BUDGET,
                        time.monotonic)   # 批量非阻塞 drain（≤16 次/≤5ms，见 helper）
            fresh = time.monotonic()
            if opts['telemetry']:
                controller.poll_telemetry(fresh)      # 心跳/收包均非阻塞
            controller.consume_web(worker)            # 只读缓存，不阻塞
            if fresh - last_probe >= OWNERSHIP_CHECK_INTERVAL:
                last_probe = fresh
                controller.check_ownership_fatal(fresh)
            controller.tick(time.monotonic())         # 输入收完后取新鲜单调时刻
            now = time.monotonic()
            if now - last_diag >= DIAG_PERIOD:
                last_diag = now
                snapshot = controller.snapshot(now)
                try:
                    diag_pub.publish(String(data=json.dumps(
                        snapshot, ensure_ascii=False, allow_nan=False,
                        separators=(',', ':'), default=str)))
                except Exception as e:
                    cleanup_errors.append('diag_publish: %r' % (e,))
                _print_diag(snapshot)
            if next_deadline <= now:
                next_deadline = now + RUN_PERIOD   # overrun → 顺延（不忙转/不追发）
            else:
                time.sleep(next_deadline - now)
                next_deadline += RUN_PERIOD
    except KeyboardInterrupt:
        sys.stderr.write('motion_runtime: SIGINT, normal end\n')
    except Exception:
        import traceback
        traceback.print_exc()
        exit_code = 2
    finally:
        def _cleanup(tag, func):
            try:
                func()
            except BaseException as e:
                cleanup_errors.append('%s: %r' % (tag, e))
        # 停止输出必须先于任何可能阻塞的 join；失败仍继续后续资源回收。
        # live 曾激活：disarm 后补发一次零速；dryrun 从未放行不发送。
        if controller is not None:
            _cleanup('final_zero', lambda: controller.shutdown_output(time.monotonic()))
        if worker is not None:
            def stop_worker():
                already, joined = worker.stop()
                if not already and not joined:
                    cleanup_errors.append('web_worker_join_timeout')
            _cleanup('web_worker_stop', stop_worker)
        if sender is not None:  # 只关自身 publisher；零速已由 GuardedOutput 负责
            _cleanup('sender_close', sender.close)
        if telemetry is not None:
            _cleanup('telemetry_close', telemetry.close)
        if executor is not None:
            if node is not None:
                _cleanup('executor.remove_node', lambda: executor.remove_node(node))
            _cleanup('executor.shutdown', executor.shutdown)
        if node is not None:
            _cleanup('node.destroy_node', node.destroy_node)
        _cleanup('rclpy.shutdown', lambda: rclpy.ok() and rclpy.shutdown())
        # 先按 cleanup 错误确定最终 exit code，再打印一致的 summary
        if cleanup_errors:
            for err in cleanup_errors:
                sys.stderr.write('motion_runtime cleanup error: %s\n' % (err,))
            exit_code = 2
        summary = {'exit_code': exit_code,
                   'cleanup_errors': [str(e) for e in cleanup_errors],
                   'snapshot': controller.snapshot() if controller else None,
                   'note': 'exit 0 = bounded run finished (dryrun end/SIGINT); '
                           'NOT equal to movable'}
        _print_diag({'kind': 'summary', **_jsonable(summary)})
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
