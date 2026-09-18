#!/usr/bin/env python3
"""
NEW-06: M20 标准 NavCmd 预览转换节点（仅算法预览，机器不会运动）

职责：
- 订阅预览raw Twist（/new_chase/cmd_vel_raw，robot_nexus 算法输出）
- 转换为标准 drdds/msg/NavCmd 消息并在预览话题发布，仅供观测/联调验证
- 本阶段绝不发布到 /NAV_CMD /MOTION_STATE /GAIT /cmd_vel /d1_cmd 等真实控制话题
  （创建发布者前解析重映射后最终名，非法配置 exit 2；
  无 live/arm/enable_motion 参数，本阶段就是不可驱动 preview）

NEW-06 第2轮P0: Foxy 的 rclpy 没有 Node.resolve_topic_name（已核对安装目录
/opt/ros/foxy/lib/python3.8/site-packages/rclpy/，仅 expand_topic_name.py:18
expand_topic_name，且不应用重映射规则），手动近似解析也不可靠。
因此本 preview 节点**完全禁止所有话题重映射**：
1) main() 入口在创建任何实体前对原始 argv 做 fail-closed 扫描：
   仅允许精确元规则 __node / __name / __ns（launch 生成的 -r __node:=xxx 属此），
   节点限定形式/wildcard/rostopic:// 等其余规则一律 exit 2；
2) 预览话题改名仅可通过 preview_topic 参数（白名单严格 /new_chase/ 前缀，
   创建前 expand 校验），raw_topic 参数同样校验不指向真实控制话题；
3) 创建后 Publisher.topic_name 复核保留为兜底断言。
本节点不声称支持完整 remap 解析（docs 已如实记录）。

固定 basic 预览说明（TODO，后续任务完成）：
- 尚未验证 M20 官方运动模式的导航模式/独占控制权
- 真实收流已证 SDK root 链路（主代理实测：150帧/15s/10.01Hz, user0），
  普通ROS话题链路尚未打通；preview默认输入仍无来源，合成路径未承诺可用
- 机器人不会因本节点运动
"""

import math
import sys

# 预览限幅（与 C++ 侧 applyPreviewLimits 一致，幂等兜底）
PREVIEW_MAX_VX = 0.3   # m/s
PREVIEW_MAX_VY = 0.3   # m/s
PREVIEW_MAX_WZ = 0.6   # rad/s

# M20 basic gait 4097 有效阈值（官方ROS运动文档）：
# 低于阈值置零，不可向上放大
GAIT_MIN_VX = 0.2   # m/s
GAIT_MIN_VY = 0.35  # m/s
GAIT_MIN_YAW = 0.5  # rad/s

# raw 指令新鲜度窗口（秒）；过期视为无指令 → 输出全零预览
CMD_FRESH_WINDOW = 0.3

# 预览发布频率（20Hz 零预览）
PREVIEW_RATE_HZ = 20.0

# 预览话题命名空间前缀（解析重映射后的最终名必须严格位于其下）
PREVIEW_NAMESPACE_PREFIX = '/new_chase/'

# 真实控制话题（最终解析名黑名单）
FORBIDDEN_TOPICS = {'/cmd_vel', '/d1_cmd', '/nav_cmd', '/motion_state', '/gait'}


def to_nav_values(vx, vy, wz):
    """Twist(SI: m/s, rad/s) -> NavCmd data 字段值 (x_vel, y_vel, yaw_vel)。

    规则（可无ROS导入测试）：
    - 非有限输入 -> 全零（不允许输出inf/nan）
    - 先限幅到预览上限，再保守禁止后退/横移
    - 低于 M20 basic gait 4097 有效阈值的分量置零，绝不向上放大
    - 数值保持 SI 单位（m/s, rad/s），不是归一化值
    """
    try:
        vx = float(vx)
        vy = float(vy)
        wz = float(wz)
    except (TypeError, ValueError, OverflowError):  # P2: 大整数float()可抛OverflowError
        return 0.0, 0.0, 0.0
    if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
        return 0.0, 0.0, 0.0

    # 1) 预览限幅（幂等：C++侧已限，这里兜底）
    vx = max(-PREVIEW_MAX_VX, min(PREVIEW_MAX_VX, vx))
    vy = max(-PREVIEW_MAX_VY, min(PREVIEW_MAX_VY, vy))
    wz = max(-PREVIEW_MAX_WZ, min(PREVIEW_MAX_WZ, wz))

    # 2) 保守策略：禁止后退与横移（M20预览阶段仅前进向）
    if vx < 0.0:
        vx = 0.0
    vy = 0.0

    # 3) 低于gait有效阈值置零（不向上放大）
    if abs(vx) < GAIT_MIN_VX:
        vx = 0.0
    if abs(vy) < GAIT_MIN_VY:
        vy = 0.0
    if abs(wz) < GAIT_MIN_YAW:
        wz = 0.0

    return vx, vy, wz


def is_fresh(now_mono_sec, stamp_mono_sec, window=CMD_FRESH_WINDOW):
    """raw指令新鲜度判断（单调时钟秒）。

    P2修复: 三个输入必须finite；window>=0；delta必须落在 [0, window]
    （负delta=来自未来的指令同样判不新鲜，杜绝 future-stamp 绕过）。
    """
    try:
        now = float(now_mono_sec)
        stamp = float(stamp_mono_sec)
        win = float(window)
    except (TypeError, ValueError, OverflowError):
        return False
    if not (math.isfinite(now) and math.isfinite(stamp) and math.isfinite(win)):
        return False
    if win < 0.0:
        return False
    delta = now - stamp
    if not math.isfinite(delta):
        return False
    return 0.0 <= delta <= win


# NEW-06 第2轮P0: 本preview节点完全禁止话题重映射(topic remapping)。
# 只允许精确ROS节点/namespace元规则 __node / __name / __ns（节点限定形式如
# "nav:__node" 不支持，同样拒绝；wildcard、rostopic:// 等一律拒绝，exit 2）。
# 预览话题改名只能用 preview_topic 参数（白名单 /new_chase/ 前缀）。
# docs 已如实记录本节点禁止话题remap，不再声称支持完整remap解析。
ALLOWED_META_RULES = {'__node', '__name', '__ns'}


def _scan_cli_remap_rules(argv):
    """扫描原始argv中的重映射规则（未解析字符串，先于rclpy.init处理）。

    形式: '-r rule' / '--remap rule' / '--remap=rule' / '-rrule' / 裸 'a:=b'。
    参数规则（-p/--param/--params-file 后随项或对应=前缀项）不算remap，跳过；
    其余任何含':='且无法完整识别的项按remap候选处理（fail closed）。

    返回 (allowed_meta_rules, forbidden_rules)。
    """
    params_flags = {'-p', '--param', '--params-file', '--param-file'}
    rules = []
    i = 0
    n = len(argv)
    while i < n:
        arg = argv[i]
        if arg in ('-r', '--remap'):
            if i + 1 >= n:
                rules.append(None)  # 尾随悬空选项——无法识别，按禁止处理
                i += 1
            else:
                rules.append(argv[i + 1])
                i += 2
        elif arg.startswith('--remap='):
            rules.append(arg[len('--remap='):])
            i += 1
        elif arg.startswith('-r') and len(arg) > 2 and not arg.startswith('--'):
            rules.append(arg[2:])
            i += 1
        elif arg in params_flags or arg.startswith('--param=') or \
                arg.startswith('--params-file=') or arg.startswith('--param-file=') or \
                arg.startswith('-p'):
            # 参数选项：跳过其后随值项（其值可能含 ':='，属参数规则非重映射）
            if arg in params_flags:
                i += 2
            else:
                i += 1
        elif ':=' in arg:
            # 裸 a:=b 形式（ROS2 CLI合法remap形式）——按remap规则候选处理
            rules.append(arg)
            i += 1
        else:
            i += 1

    allowed, forbidden = [], []
    for r in rules:
        if r is None or ':=' not in r:
            # 悬空选项/无法解析 → fail closed
            forbidden.append(r)
            continue
        src, dst = r.split(':=', 1)
        src = src.strip()
        if src in ALLOWED_META_RULES:
            allowed.append((src, dst.strip()))
        else:
            # 节点限定('nav:__node')、话题/通配/rostopic等一律禁止
            forbidden.append(r)
    return allowed, forbidden


def validate_preview_topic_resolved(resolved):
    """最终解析名白名单：严格 /new_chase/ 前缀且非真实控制话题。"""
    if not resolved.startswith(PREVIEW_NAMESPACE_PREFIX):
        return False
    if resolved in FORBIDDEN_TOPICS:
        return False
    return True


def main(args=None):
    import time

    # NEW-06 第2轮P0: 在创建Node/任何实体之前，先对原始argv做fail-closed重映射扫描。
    # 本节点所有重映射均来自全局argv（Node构造无local cli_args）；
    # 除 __node/__name/__ns 元规则外，任何规则（含话题/wildcard/rostopic:///
    # 节点限定形式）一律 exit 2，不创建任何发布者。
    scan_args = list(args if args is not None else sys.argv)
    _, forbidden_rules = _scan_cli_remap_rules(scan_args)
    if forbidden_rules:
        sys.stderr.write(
            'nav_cmd_preview 拒绝启动(exit 2): 检测到被禁止的重映射规则 %s\n'
            '本节点禁止所有话题重映射；预览话题仅可通过 preview_topic 参数'
            '(严格 %s* 前缀)修改。\n'
            % (forbidden_rules, PREVIEW_NAMESPACE_PREFIX))
        sys.exit(2)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node as RclpyNode

    from geometry_msgs.msg import Twist
    from drdds.msg import NavCmd

    class NavCmdPreviewNode(RclpyNode):
        def __init__(self):
            super().__init__('nav_cmd_preview')
            self.declare_parameter('raw_topic', '/new_chase/cmd_vel_raw')
            self.declare_parameter('preview_topic', '/new_chase/nav_cmd_preview')

            # NEW-06 第3轮P0修复: Foxy rclpy Parameter 无 as_string()（那是C++ API），
            # 真实API（rclpy/parameter.py:144-151 已核对）: type_ / value / get_parameter_value()。
            # 这里用 .value 并校验类型必须是字符串。
            raw_param = self.get_parameter('raw_topic')
            preview_param = self.get_parameter('preview_topic')
            raw_topic = self._require_string_param('raw_topic', raw_param)
            preview_topic = self._require_string_param('preview_topic', preview_param)

            # NEW-06: 话题remap已在main()入口fail-closed禁止；此处仅需展开相对名
            # （无任何话题remap规则可应用，expand即最终名），创建发布者前严格校验
            from rclpy.expand_topic_name import expand_topic_name as _expand
            try:
                resolved = _expand(preview_topic, self.get_name(), self.get_namespace())
            except (ValueError, RuntimeError) as e:
                raise RuntimeError('nav_cmd_preview 拒绝启动(exit 2): '
                                   'preview_topic=%r 无法展开: %s' % (preview_topic, e))
            if not validate_preview_topic_resolved(resolved):
                raise RuntimeError(
                    'nav_cmd_preview 拒绝启动(exit 2): preview_topic=%r 解析为 %r，'
                    '非法（必须严格位于 %s* 且非真实控制话题 %s）'
                    % (preview_topic, resolved, PREVIEW_NAMESPACE_PREFIX,
                       sorted(FORBIDDEN_TOPICS)))

            # 仅预览发布器（NavCmd 标准消息，供观测；非真实 /NAV_CMD）
            self._pub = self.create_publisher(NavCmd, preview_topic, 10)
            self._sub = self.create_subscription(Twist, raw_topic, self._on_raw, 10)

            # NEW-06 P0: 创建后复核最终解析名（Foxy Publisher.topic_name property）——
            # 保留作兜底断言，不作为主防线（主防线在argv扫描+创建前校验）
            actual = self._pub.topic_name
            if not validate_preview_topic_resolved(actual):
                raise RuntimeError(
                    'nav_cmd_preview 拒绝启动(exit 2): 发布者最终解析名 %r 非法'
                    '（兜底断言：解析名不在预览白名单）' % actual)

            # raw_topic(订阅侧，无发布权)轻量校验：不允许指向真实控制话题；
            # 不在/new_chase/下时仅告警（可选约束，预览改名走preview_topic白名单）
            # NEW-06 第3轮修复: expand失败与校验拒绝分离，拒绝不被吞掉
            try:
                raw_resolved = _expand(raw_topic, self.get_name(), self.get_namespace())
            except (ValueError, RuntimeError) as e:
                raise RuntimeError('nav_cmd_preview 拒绝启动(exit 2): '
                                   'raw_topic=%r 无法展开: %s' % (raw_topic, e))
            if raw_resolved in FORBIDDEN_TOPICS:
                raise RuntimeError('nav_cmd_preview 拒绝启动(exit 2): '
                                   'raw_topic=%r 指向真实控制话题' % raw_topic)
            if not raw_resolved.startswith(PREVIEW_NAMESPACE_PREFIX):
                self.get_logger().warn(
                    'raw_topic=%r 不在 %s* 下(仅订阅侧, 允许但请确认无控制语义)'
                    % (raw_topic, PREVIEW_NAMESPACE_PREFIX))

            import threading
            self._lock = threading.Lock()
            self._last_cmd_mono = None   # (单调时钟秒, vx, vy, wz)
            self._seq = 0                # header.frame_id: 递增uint64

            # 20Hz 定时零预览：raw 过期/缺失时输出全零
            self._timer = self.create_timer(1.0 / PREVIEW_RATE_HZ, self._tick)

            self.get_logger().info(
                'nav_cmd_preview 启动: raw=%s -> preview=%s (解析=%s) @%.0fHz '
                '(仅算法/NavCmd预览, 机器不会运动; 未验证导航模式/独占控制; '
                'SDK export→ROS链路已实证(NEW-09 r1, /new_chase/points); '
                '本节点仅预览非运动控制; 禁止话题重映射)'
                % (raw_topic, preview_topic, actual, PREVIEW_RATE_HZ))

        # NEW-06 第3轮: 参数类型严格校验（必须是字符串参数）
        @staticmethod
        def _require_string_param(name, param):
            from rclpy.parameter import Parameter as RclpyParameter
            if param.type_ != RclpyParameter.Type.STRING:
                raise RuntimeError(
                    'nav_cmd_preview 拒绝启动(exit 2): 参数 %s 类型非法(%s)，'
                    '必须是字符串' % (name, param.type_))
            value = param.value  # Foxy rclpy真实API（rclpy/parameter.py:148）
            if not isinstance(value, str) or not value:
                raise RuntimeError(
                    'nav_cmd_preview 拒绝启动(exit 2): 参数 %s 值 %r 非字符串' % (name, value))
            return value

        def _on_raw(self, msg):
            now_mono = time.monotonic()
            with self._lock:
                self._last_cmd_mono = (
                    now_mono, float(msg.linear.x), float(msg.linear.y),
                    float(msg.angular.z))

        def _tick(self):
            now_mono = time.monotonic()
            with self._lock:
                cached = self._last_cmd_mono
            vx, vy, wz = 0.0, 0.0, 0.0
            if cached is not None and is_fresh(now_mono, cached[0]):
                vx, vy, wz = to_nav_values(cached[1], cached[2], cached[3])
            # 过期/缺失 -> 全零预览（不保留旧指令，杜绝指令残留）

            msg = NavCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._seq & 0xFFFFFFFFFFFFFFFF
            self._seq = (self._seq + 1) & 0xFFFFFFFFFFFFFFFF
            msg.data.x_vel = float(vx)
            msg.data.y_vel = float(vy)
            msg.data.yaw_vel = float(wz)
            self._pub.publish(msg)

    # NEW-06 第3轮修复（graceful生命周期）:
    # - KeyboardInterrupt（SIGINT：launch timeout --signal=INT / 用户Ctrl-C）视为
    #   正常结束（exit 0），不当作崩溃；
    # - executor 显式 shutdown/remove_node，避免ctx被signal shutdown后二次报错；
    # - rclpy.shutdown 前用 rclpy.ok() 守卫（ctx已被信号关闭时不二次shutdown）；
    # - cleanup 异常不吞掉：逐项记录并置 exit 2，绝不伪成功。
    # 非法配置/运行异常仍 exit 2；保持20Hz/TTL0.3原契约。
    rclpy.init(args=args)
    node = None
    executor = None
    exit_code = 0
    try:
        node = NavCmdPreviewNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        # SIGINT正常结束（ros2 launch Ctrl-C / timeout --signal=INT 有界自测）
        print('nav_cmd_preview: 收到SIGINT, 正常结束', file=sys.stderr)
    except Exception:
        import traceback
        traceback.print_exc()
        exit_code = 2
    finally:
        # NEW-06 第3轮修复: 析构期间屏蔽SIGINT——首次有界实测发现 launch 信号升级
        # 会在 destroy_node 中再次触发 KeyboardInterrupt（BaseException 不被
        # except Exception 捕获）导致 process has died；这里先 IGN 再析构，
        # 结束后恢复原handler。cleanup异常仍逐项记录并置 exit 2，不伪成功。
        import signal as _signal
        sigint_handler = None
        try:
            sigint_handler = _signal.signal(_signal.SIGINT, _signal.SIG_IGN)
        except (ValueError, OSError):
            sigint_handler = None
        try:
            cleanup_errors = []
            if executor is not None:
                if node is not None:
                    try:
                        executor.remove_node(node)
                    except BaseException as e:
                        cleanup_errors.append('executor.remove_node: %r' % (e,))
                try:
                    executor.shutdown()
                except BaseException as e:
                    cleanup_errors.append('executor.shutdown: %r' % (e,))
            if node is not None:
                try:
                    node.destroy_node()
                except BaseException as e:
                    cleanup_errors.append('node.destroy_node: %r' % (e,))
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except BaseException as e:
                cleanup_errors.append('rclpy.shutdown: %r' % (e,))
            if cleanup_errors:
                for err in cleanup_errors:
                    sys.stderr.write('nav_cmd_preview cleanup错误: %s\n' % err)
                exit_code = 2
        finally:
            if sigint_handler is not None:
                try:
                    _signal.signal(_signal.SIGINT, sigint_handler)
                except (ValueError, OSError):
                    pass
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
