"""motion_control.inputs — 输入侧纯函数校验（scan / web status，本轮新增）。

纯函数、无 ROS/IO；duck-typed 消息可被替身注入。metadata malformed 一律
判 valid=False 并给 reason，绝不抛异常穿出本模块。
scan：期望 frame=lidar_link；点坐标直接按 scan 角算 body 坐标（厂商点云已 body，
不再 TF）；stamp 精确 int（sec>=0、0<=nsec<1e9，bool/NaN/浮点拒绝）且非零；
now_wall-stamp ∈ [-0.1,0.5]；angle 元数据 finite、increment>0、max>=min、
range_min>0、range_max>range_min、ranges 非空、末角 finite（防 cos 溢出）；
inf/越界忽略；包络 x∈[-0.46,0.46] y∈[-0.31,0.31] 内回波忽略（未保证自滤除）；
至少 1 有效回波且 ≥1 有效点在包络外；停止区（包络外）x∈[-0.90,0.85] 且
|y|≤0.70 → clear=False 单帧立即停不去抖。
⚠️ 声明几何未标定，本文件不是通用避障。
web：/api/status 必须 dict + 4 bool（is_active/is_selected/is_tracking_valid/
is_moving_enabled）全精确 True + target x/y 有限；GET 成功≠tracking 有效；
HTTP 仅 GET 127.0.0.1:8080/api/status，绝无 POST；反复 GET 不能延续 scan 新鲜度
（scan/tracking 各自独立 TTL，由 core guard 判定）；目标 generation 只能按坐标
变化生成，无法区分算法跟踪与人工重新选点（如实声明）。
"""

import json
import math
from urllib.request import Request, urlopen as urlopen_default

__all__ = [
    "FRAME_ID", "SCAN_FUTURE", "SCAN_MAX_AGE", "BODY_X_MIN", "BODY_X_MAX",
    "BODY_Y_MIN", "BODY_Y_MAX", "STOP_X_MIN", "STOP_X_MAX", "STOP_HALF_Y",
    "WEB_URL", "WEB_HTTP_TIMEOUT", "WEB_REQUIRED_FLAGS",
    "check_scan", "parse_web_status", "fetch_web_status", "target_generation",
]

FRAME_ID = 'lidar_link'
SCAN_FUTURE = 0.1        # stamp 允许的未来量（与 core SCAN_FUTURE 一致）
SCAN_MAX_AGE = 0.5       # stamp 最大墙钟陈旧
BODY_X_MIN, BODY_X_MAX = -0.46, 0.46
BODY_Y_MIN, BODY_Y_MAX = -0.31, 0.31
STOP_X_MIN, STOP_X_MAX = -0.90, 0.85
STOP_HALF_Y = 0.70
WEB_URL = 'http://127.0.0.1:8080/api/status'
WEB_HTTP_TIMEOUT = 0.1   # worker 单次最坏阻塞 ≪ TTL0.5；主循环不直接调用 HTTP
WEB_REQUIRED_FLAGS = ('is_active', 'is_selected', 'is_tracking_valid',
                      'is_moving_enabled')


def _finite_num(value):
    """有限 int/float；拒绝 bool/NaN/Inf/溢出大整数（float() OverflowError 安全）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        value = float(value)
    except (OverflowError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _stamp_seconds(header):
    """严格读 header.stamp → float 秒；结构/类型/范围异常返回 None（不抛）。

    sec/nsec 必须精确 int（bool 拒绝）、sec>=0、0<=nsec<1e9、合成值有限；
    NaN/浮点/负值一律 None → 上层判 malformed（不再被 max(0,·) 隐藏）。
    """
    try:
        stamp = header.stamp
        sec, nsec = stamp.sec, stamp.nanosec
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    if type(sec) is not int or type(nsec) is not int:
        return None
    if sec < 0 or not 0 <= nsec < 1000000000:
        return None
    try:
        value = sec + nsec * 1e-9
    except OverflowError:
        return None
    return value if math.isfinite(value) else None


def check_scan(scan, now_wall, recv_age=None):
    """校验一帧 duck-typed LaserScan → dict（本函数绝不抛异常）：

    {valid, clear, source_age, reason, valid_count, point_count, envelope_skipped}
    reason（稳定 lowercase）：ok / stop_zone（valid=True）；malformed /
    frame_mismatch / stamp_zero / stamp_stale / stamp_future / delivery_late /
    meta_invalid / empty_ranges / no_valid_points / envelope_only（valid=False）。

    recv_age：本帧 ROS 接收→回调处理延迟秒；须有限且 0..0.5 否则 delivery_late。
    source_age = max(0, 墙龄, recv_age)（保守综合，不做重复相加；≥0 保证 core
    以 source_age+停留 ≤ TTL 判定真实陈旧）。
    """
    result = {'valid': False, 'clear': False, 'source_age': float('nan'),
              'reason': 'malformed', 'valid_count': 0, 'point_count': 0,
              'envelope_skipped': 0}

    def _fail(reason):
        result['reason'] = reason
        return result

    try:
        frame_id = scan.header.frame_id
        angle_min = float(scan.angle_min)
        angle_max = float(scan.angle_max)
        angle_inc = float(scan.angle_increment)
        range_min = float(scan.range_min)
        range_max = float(scan.range_max)
        ranges = scan.ranges
    except (AttributeError, TypeError, ValueError, OverflowError):
        return _fail('malformed')
    if not (math.isfinite(angle_min) and math.isfinite(angle_max)
            and math.isfinite(angle_inc) and math.isfinite(range_min)
            and math.isfinite(range_max)):
        return _fail('meta_invalid')
    if frame_id != FRAME_ID:
        return _fail('frame_mismatch')
    stamp = _stamp_seconds(scan.header)
    if stamp is None:
        return _fail('malformed')
    if stamp == 0.0:
        return _fail('stamp_zero')
    try:
        now_wall = float(now_wall)
    except (TypeError, ValueError, OverflowError):
        return _fail('malformed')
    if not math.isfinite(now_wall):
        return _fail('malformed')
    age = now_wall - stamp
    if age > SCAN_MAX_AGE:
        return _fail('stamp_stale')
    if age < -SCAN_FUTURE:
        return _fail('stamp_future')
    if recv_age is not None:
        recv_age = _finite_num(recv_age)
        if recv_age is None or recv_age < 0.0 or recv_age > SCAN_MAX_AGE:
            return _fail('delivery_late')
        result['source_age'] = max(0.0, age, recv_age)
    else:
        result['source_age'] = max(0.0, age)
    if angle_inc <= 0.0 or angle_max < angle_min:
        return _fail('meta_invalid')
    if range_min <= 0.0 or range_max <= range_min:
        return _fail('meta_invalid')
    try:
        point_count = len(ranges)
    except TypeError:
        return _fail('malformed')
    if point_count == 0:
        return _fail('empty_ranges')
    result['point_count'] = point_count
    if not math.isfinite(angle_min + angle_inc * (point_count - 1)):
        return _fail('meta_invalid')  # 末角溢出（inf/nan）保守整帧拒绝，防 cos 溢出

    stop_hit = False
    try:
        for i in range(point_count):
            r = _finite_num(ranges[i])
            if r is None or not (range_min <= r <= range_max):
                continue
            result['valid_count'] += 1
            angle = angle_min + i * angle_inc
            x = r * math.cos(angle)   # body 坐标：直接按 scan 角计算，无 TF
            y = r * math.sin(angle)
            if (BODY_X_MIN <= x <= BODY_X_MAX) and (BODY_Y_MIN <= y <= BODY_Y_MAX):
                result['envelope_skipped'] += 1  # 包络内=自身结构，忽略
                continue
            if (STOP_X_MIN <= x <= STOP_X_MAX) and abs(y) <= STOP_HALF_Y:
                stop_hit = True  # 单帧立即停；继续统计供诊断
    except (OverflowError, ValueError, TypeError, LookupError):
        return _fail('malformed')
    if result['valid_count'] == 0:
        return _fail('no_valid_points')
    if result['valid_count'] <= result['envelope_skipped']:
        return _fail('envelope_only')  # 有效点全在包络内：无可用外部观测
    result['valid'] = True
    if stop_hit:
        result['reason'] = 'stop_zone'   # clear=False：停止区有回波
        return result
    result['clear'] = True
    result['reason'] = 'ok'
    return result


def _reject_constant(name):
    raise ValueError('nonfinite json constant: %s' % (name,))


def _no_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate json key: %r' % (key,))
        result[key] = value
    return result


def parse_web_status(raw):
    """解析 /api/status → {valid, reason, flags, target, m20_preview}。

    valid=True 要求 dict + 4 必需 bool 全精确 True + target dict 且 x/y 有限。
    失败原因：bad_json / not_dict / flag_not_true:<key> / target_missing /
    target_nonfinite；绝不把 GET 成功当 tracking。
    """
    out = {'valid': False, 'reason': 'bad_json', 'flags': None, 'target': None,
           'm20_preview': None}
    if isinstance(raw, (bytes, bytearray)):
        try:
            raw = bytes(raw).decode('utf-8')
        except UnicodeDecodeError:
            return out
    if isinstance(raw, str):
        try:
            data = json.loads(raw, object_pairs_hook=_no_duplicate_keys,
                              parse_constant=_reject_constant)
        except ValueError:
            return out
    else:
        data = raw
    if not isinstance(data, dict):
        out['reason'] = 'not_dict'
        return out
    preview = data.get('m20_preview')
    out['m20_preview'] = preview if isinstance(preview, bool) else None
    flags = {}
    for key in WEB_REQUIRED_FLAGS:
        value = data.get(key)
        if value is not True:
            out['reason'] = 'flag_not_true:%s' % key
            out['flags'] = dict(flags)
            return out
        flags[key] = True
    target = data.get('target')
    if not isinstance(target, dict):
        out['reason'] = 'target_missing'
        out['flags'] = flags
        return out
    x = _finite_num(target.get('x'))
    y = _finite_num(target.get('y'))
    if x is None or y is None:
        out['reason'] = 'target_nonfinite'
        out['flags'] = flags
        return out
    out['valid'] = True
    out['reason'] = 'ok'
    out['flags'] = flags
    out['target'] = (x, y)
    return out


def fetch_web_status(timeout=WEB_HTTP_TIMEOUT, url=WEB_URL, urlopen=None):
    """GET /api/status（仅 GET，绝无 POST）→ 响应文本；网络异常上抛。
    timeout 必须 (0, 0.1]；urlopen 可注入替身；反复 GET 失败不延续任何新鲜度。"""
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) \
            or not math.isfinite(timeout) or not 0.0 < timeout <= 0.1:
        raise ValueError('timeout must be in (0, 0.1]')
    opener = urlopen if urlopen is not None else urlopen_default
    request = Request(url, method='GET')  # 只读 GET；无任何 POST/控制路径
    with opener(request, timeout=timeout) as response:
        body = response.read(65536)
    if isinstance(body, bytes):
        body = body.decode('utf-8', 'replace')
    return body


def target_generation(prev_target, prev_gen, new_target):
    """按坐标变化推进目标 generation → (new_gen, changed)。
    ⚠️ 坐标变化无法区分算法跟踪质心更新与人工重新选点；仅诊断展示，
    绝不用作身份认证或自动恢复依据。"""
    if not (isinstance(prev_gen, int) and not isinstance(prev_gen, bool)
            and prev_gen >= 0):
        raise ValueError('prev_gen must be non-negative int')
    changed = True
    if isinstance(prev_target, tuple) and len(prev_target) == 2:
        changed = prev_target != new_target
    return ((prev_gen + 1) if changed else prev_gen), changed
