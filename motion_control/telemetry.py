"""motion_control.telemetry — 只读遥测 UDP 客户端（basic_server 协议，本轮新增）。

只读合同：仅接收机器人主动上报；唯一出站帧=心跳 Type=100 Cmd=100（默认 1Hz），
绝无速度/模式/起立/其它 outbound Type。服务端固定 AOS 10.21.31.103:30000；
connected UDP socket 锁定来源（recvfrom 再复核端点，错误来源按输入错误处置）。
新鲜度只用单调钟接收时刻；报文 Time 无认证签名，不作任何依据。
预算：非阻塞 recvfrom，每次 poll ≤32 包，不阻塞主循环（心跳 send 亦非阻塞）。
malformed（长度/JSON/重复键/NaN/Infinity/1e999/错类型/缺字段）保守拒绝并上报，
不崩溃、不让坏包续期 good；事件由调用方按输入错误立即 disarm（本模块只报告）。

协议依据：官方在线文档（URL 记录于 RUNTIME.md；本任务不读旧工程文档）。要点：
APDU 16B 头 '<4sHHB7s' = magic eb91eb90 + uint16LE ASDU 长度 + uint16LE 报文ID
(0..65535 循环) + 格式 1(JSON) + 预留 7×0；心跳 100/100 ≥1Hz（服务端 2s 无请求
停推）；BasicStatus=Type 1002/Cmd 6 @2Hz（Items.BasicStatus 嵌套或 Items 平铺，
须完整 7 字段 int，partial 拒绝不合并；ControlUsageMode=0 等是合法遥测，放行与否
由 core guard 判定）；ACK Items.ErrorCode：0=仅受理（不刷新状态），非 0=通信故障。
测试可注入 fake socket（send/recvfrom/setblocking/close）与 fake 单调钟。
"""

import datetime
import json
import math
import socket
import struct
import time

__all__ = [
    "HEADER", "FORMAT_JSON", "AOS_HOST", "AOS_PORT", "HEARTBEAT_TYPE",
    "HEARTBEAT_CMD", "ROBOT_TYPE", "BASIC_STATUS_CMD", "REQUIRED_STATUS_FIELDS",
    "MAX_PACKETS_PER_POLL", "encode_heartbeat", "decode_apdu",
    "extract_basic_status", "decode_basic_status", "decode_ack_error",
    "TelemetryClient",
]

HEADER = struct.Struct('<4sHHB7s')           # 16 字节 APDU 头
MAGIC = bytes.fromhex('eb91eb90')
FORMAT_JSON = 1                              # ASDU 格式：1 = JSON
RESERVED = bytes(7)
AOS_HOST = '10.21.31.103'
AOS_PORT = 30000
HEARTBEAT_TYPE = 100
HEARTBEAT_CMD = 100
ROBOT_TYPE = 1002
BASIC_STATUS_CMD = 6
# core.EXPECTED_STATUS 覆盖的 7 个官方字段（完整布局才收，partial 拒绝）
REQUIRED_STATUS_FIELDS = ('MotionState', 'Gait', 'Charge', 'HES',
                          'ControlUsageMode', 'Direction', 'Sleep')
MAX_PACKETS_PER_POLL = 32
RECV_BUFSIZE = 65536


def _reject_constant(name):
    raise ValueError('nonfinite json constant: %s' % (name,))


def _no_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate json key: %r' % (key,))
        result[key] = value
    return result


def _check_finite(value, path='body'):
    """递归拒绝非有限浮点（覆盖 1e999 等不经 parse_constant 的溢出）。"""
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError('nonfinite number at %s' % (path,))
    elif isinstance(value, dict):
        for key in value:
            _check_finite(value[key], '%s.%s' % (path, key))
    elif isinstance(value, (list, tuple)):
        for i, item in enumerate(value):
            _check_finite(item, '%s[%d]' % (path, i))


def encode_heartbeat(sequence, dt=None):
    """编码心跳 APDU（纯函数）。sequence 精确 uint16 int；dt 可注入（测试），
    缺省本地时间 YYYY-MM-DD HH:MM:SS；非法 → ValueError。"""
    if type(sequence) is not int or not 0 <= sequence <= 0xFFFF:
        raise ValueError('heartbeat sequence must be uint16 int, got %r' % (sequence,))
    if dt is None:
        dt = datetime.datetime.now()
    try:
        time_text = dt.strftime('%Y-%m-%d %H:%M:%S')
    except (AttributeError, ValueError) as e:
        raise ValueError('heartbeat time invalid: %r' % (e,))
    body = json.dumps(
        {'PatrolDevice': {'Type': HEARTBEAT_TYPE, 'Command': HEARTBEAT_CMD,
                          'Time': time_text, 'Items': {}}},
        ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(body) > 0xFFFF:
        raise ValueError('heartbeat body too large')
    return HEADER.pack(MAGIC, len(body), sequence, FORMAT_JSON, RESERVED) + body


def _loads_strict(text):
    body = json.loads(text, object_pairs_hook=_no_duplicate_keys,
                      parse_constant=_reject_constant)
    _check_finite(body)
    return body


def decode_apdu(packet):
    """严格解码 APDU → (message_id, PatrolDevice dict)；任何结构异常 → ValueError。"""
    if not isinstance(packet, (bytes, bytearray, memoryview)):
        raise ValueError('packet must be bytes-like')
    packet = bytes(packet)
    if len(packet) < HEADER.size:
        raise ValueError('truncated header: %d bytes' % len(packet))
    magic, length, message_id, fmt, reserved = HEADER.unpack_from(packet)
    if magic != MAGIC:
        raise ValueError('bad magic')
    if fmt != FORMAT_JSON:
        raise ValueError('non-json format: %d' % fmt)
    if reserved != RESERVED:
        raise ValueError('nonzero reserved bytes')
    if len(packet) != HEADER.size + length:
        raise ValueError('length mismatch: header %d, datagram %d'
                         % (length, len(packet) - HEADER.size))
    try:
        body = _loads_strict(packet[HEADER.size:].decode('utf-8'))
    except UnicodeDecodeError as e:  # ValueError 子类，统一显式 ValueError
        raise ValueError('invalid utf-8 body: %s' % (e,))
    if not isinstance(body, dict) or not isinstance(body.get('PatrolDevice'), dict):
        raise ValueError('missing PatrolDevice object')
    value = body['PatrolDevice']
    if type(value.get('Type')) is not int or type(value.get('Command')) is not int:
        raise ValueError('Type/Command must be exact int')
    if not isinstance(value.get('Time'), str):
        raise ValueError('Time must be string')
    if not isinstance(value.get('Items'), dict):
        raise ValueError('Items must be object')
    return message_id, value


def extract_basic_status(value):
    """PatrolDevice dict → (status|None, ack_code|None)；malformed → ValueError。

    - Items.ErrorCode 存在 → ACK 帧：返回 (None, code)；code 非精确 int → ValueError；
    - Type=1002/Cmd=6 且完整 7 字段 int（嵌套或平铺）→ (status整体快照, None)，
      partial/错类型 → ValueError（绝不与旧状态合并）；
    - 其它（Cmd=3/4/5 等）→ (None, None)：忽略不刷新。
    """
    items = value['Items']
    if 'ErrorCode' in items:
        code = items['ErrorCode']
        if type(code) is not int:
            raise ValueError('ErrorCode must be exact int')
        return None, code
    if value['Type'] != ROBOT_TYPE or value['Command'] != BASIC_STATUS_CMD:
        return None, None
    if 'BasicStatus' in items:
        status = items['BasicStatus']
    elif 'MotionState' in items:
        status = items  # 平铺布局
    else:
        raise ValueError('1002/6 without BasicStatus content')
    if not isinstance(status, dict):
        raise ValueError('BasicStatus must be object')
    for key in REQUIRED_STATUS_FIELDS:
        if type(status.get(key)) is not int:
            raise ValueError('incomplete/wrongtype BasicStatus field %r' % (key,))
    return dict(status), None  # 整体快照；多余字段（Version/OOA 等）core 忽略


def decode_basic_status(packet):
    """状态帧 → status dict；非状态（ACK/其它）→ None；malformed → ValueError。"""
    _, value = decode_apdu(packet)
    status, _ = extract_basic_status(value)
    return status


def decode_ack_error(packet):
    """ACK 帧 → Items.ErrorCode int；非 ACK → None；malformed → ValueError。"""
    _, value = decode_apdu(packet)
    if 'ErrorCode' not in value['Items']:
        return None
    code = value['Items']['ErrorCode']
    if type(code) is not int:
        raise ValueError('ErrorCode must be exact int')
    return code


class TelemetryClient:
    """connected UDP 遥测客户端（收状态 + 1Hz 心跳，无其它出站）。

    sock=None 自建 connected 非阻塞 UDP；可注入 fake socket/fake 单调钟。
    poll(now) -> events（kind=status/ack_ok/comm_fault/decode_error/ignored/
    heartbeat_sent/heartbeat_error/recv_error）。只有有效 status 刷新
    last_status/status_received_at（单调秒）；无有效数据绝不更新时间戳；
    decode/recv/heartbeat 失败只计数并记 last_error（disarm/锁会话由调用方处置）；
    心跳失败也计入间隔（序列号 0..65535 循环）；send 失败不抛出（事件上报）。
    """

    def __init__(self, host=AOS_HOST, port=AOS_PORT, *,
                 sock=None, monotonic=None, heartbeat_interval=1.0):
        if not isinstance(host, str) or not host:
            raise ValueError('host must be non-empty str')
        if type(port) is not int or not 0 < port <= 0xFFFF:
            raise ValueError('port must be uint16 int')
        if type(heartbeat_interval) not in (int, float) \
                or not math.isfinite(heartbeat_interval) or heartbeat_interval <= 0.0:
            raise ValueError('heartbeat_interval must be positive finite')
        self._host, self._port = host, port
        self._monotonic = monotonic if monotonic is not None else time.monotonic
        self._heartbeat_interval = float(heartbeat_interval)
        if sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect((host, port))
            sock.setblocking(False)
            self._owns_socket = True
        else:
            try:
                sock.setblocking(False)
            except AttributeError:
                pass  # fake socket 允许省略
            self._owns_socket = False
        self._sock = sock
        self._sequence = 0
        self._last_heartbeat_at = None
        self._closed = False
        self.last_status = None           # 最近完整 BasicStatus dict
        self.status_received_at = None    # 最近有效状态单调接收时刻
        self.last_error = None
        self.packets_total = self.packets_rejected = 0
        self.status_count = self.ack_count = self.fault_count = 0
        self.heartbeat_sent = self.heartbeat_failed = 0

    def send_heartbeat(self, dt=None):
        """发一条 100/100 心跳，返回所用序列号；socket 错误上抛（poll 内已捕获）。"""
        if self._closed or self._sock is None:
            raise RuntimeError('TelemetryClient closed')
        seq = self._sequence
        self._sock.send(encode_heartbeat(seq, dt))
        self._sequence = (self._sequence + 1) & 0xFFFF
        self.heartbeat_sent += 1
        return seq

    def poll(self, now=None):
        """排空 socket（≤32 包）+ 维持 1Hz 心跳 → 事件列表；无有效数据不刷新时间戳。

        单次解析分类：malformed → decode_error；ACK 0 → ack_ok；ACK 非 0 →
        comm_fault；其它 Type/Command → ignored；错误来源端点 → decode_error
        （wrong source 保守按输入错误处置，由调用方 disarm）。
        """
        if now is None:
            now = self._monotonic()
        if isinstance(now, bool) or not isinstance(now, (int, float)) \
                or not math.isfinite(now):
            raise ValueError('poll now must be finite monotonic seconds')
        if self._closed or self._sock is None:
            return []
        events = []
        if (self._last_heartbeat_at is None
                or now - self._last_heartbeat_at >= self._heartbeat_interval):
            self._last_heartbeat_at = now
            try:
                seq = self.send_heartbeat()
                events.append({'kind': 'heartbeat_sent', 'sequence': seq, 'at': now})
            except OSError as e:
                self.heartbeat_failed += 1
                self.last_error = 'heartbeat_send_failed: %s' % (e,)
                events.append({'kind': 'heartbeat_error', 'error': self.last_error,
                               'at': now})
        for _ in range(MAX_PACKETS_PER_POLL):
            try:
                data, addr = self._sock.recvfrom(RECV_BUFSIZE)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as e:
                self.last_error = 'recv_failed: %s' % (e,)
                events.append({'kind': 'recv_error', 'error': self.last_error, 'at': now})
                break
            self.packets_total += 1
            if addr[0] != self._host or addr[1] != self._port:
                self.packets_rejected += 1
                self.last_error = 'unexpected_source: %r' % (addr,)
                events.append({'kind': 'decode_error', 'error': self.last_error, 'at': now})
                continue
            recv_at = self._monotonic()
            try:
                _, value = decode_apdu(data)
                status, ack_code = extract_basic_status(value)
            except ValueError as e:
                self.packets_rejected += 1
                self.last_error = 'decode_error: %s' % (e,)
                events.append({'kind': 'decode_error', 'error': self.last_error, 'at': recv_at})
                continue
            if status is not None:
                self.last_status = status
                self.status_received_at = recv_at
                self.status_count += 1
                events.append({'kind': 'status', 'status': status, 'received_at': recv_at})
            elif ack_code is None:
                events.append({'kind': 'ignored', 'at': recv_at})  # 其它正常状态：忽略
            elif ack_code == 0:
                self.ack_count += 1
                events.append({'kind': 'ack_ok', 'at': recv_at})  # 受理≠状态刷新
            else:
                self.fault_count += 1
                self.last_error = 'comm_fault: ACK ErrorCode=%d' % (ack_code,)
                events.append({'kind': 'comm_fault', 'error_code': ack_code,
                               'error': self.last_error, 'at': recv_at})
        return events

    def close(self):
        """幂等关闭；仅自建 socket 由本模块 close（注入 socket 由注入方负责）。"""
        self._closed = True
        if self._owns_socket and self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
