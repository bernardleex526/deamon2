"""Documented basic_server JSON/UDP framing and fail-closed velocity gate."""
import datetime
import json
import math
import socket
import struct
import time

HEADER = struct.Struct('<4sHHB7s')
MAGIC = bytes.fromhex('eb91eb90')
ZERO = (0.0, 0.0, 0.0)
GAITS = {4097: (0.2, 0.35, 0.5), 4099: (0.15, 0.3, 0.4),
         12290: (0.15, 0.25, 0.35), 12291: (0.15, 0.3, 0.4)}


def encode(message_type, command, items, message_id, timestamp=None):
    if not 0 <= message_id <= 65535:
        raise ValueError('message ID outside uint16')
    body = json.dumps({'PatrolDevice': {
        'Type': message_type, 'Command': command,
        'Time': timestamp or datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'Items': items}}, ensure_ascii=False, allow_nan=False,
        separators=(',', ':')).encode('utf-8')
    if len(body) > 65535:
        raise ValueError('ASDU too large')
    return HEADER.pack(MAGIC, len(body), message_id, 1, bytes(7)) + body


def decode(packet):
    if len(packet) < HEADER.size:
        raise ValueError('truncated header')
    magic, length, message_id, fmt, reserved = HEADER.unpack_from(packet)
    if magic != MAGIC or fmt != 1 or reserved != bytes(7):
        raise ValueError('invalid JSON protocol header')
    if len(packet) != HEADER.size + length:
        raise ValueError('invalid datagram length')
    body = json.loads(packet[HEADER.size:].decode('utf-8'))
    if not isinstance(body, dict) or not isinstance(body.get('PatrolDevice'), dict):
        raise ValueError('missing PatrolDevice')
    value = body['PatrolDevice']
    if (type(value.get('Type')) is not int or type(value.get('Command')) is not int
            or not isinstance(value.get('Items'), dict)):
        raise ValueError('invalid ASDU fields')
    return message_id, value


def velocity_items(velocity):
    x, y, yaw = velocity
    if not all(math.isfinite(v) for v in velocity):
        raise ValueError('nonfinite velocity')
    return dict(X=x, Y=y, Z=0.0, Roll=0.0, Pitch=0.0, Yaw=yaw)


class UdpClient:
    """A single connected socket preserves the status subscription source port."""
    def __init__(self, host, port=30000):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((host, port))
        self.socket.setblocking(False)
        self.message_id = 0

    def send(self, kind, command, items):
        packet = encode(kind, command, items, self.message_id)
        self.socket.send(packet)
        self.message_id = (self.message_id + 1) & 65535

    def heartbeat(self):
        # This firmware streams BasicStatus after heartbeat registration.
        self.send(100, 100, {})

    def receive(self):
        # Bound work so telemetry cannot starve the 20 Hz stop timer.
        result = []
        for _ in range(64):
            try:
                packet = self.socket.recv(65536)
            except BlockingIOError:
                break
            try:
                result.append(decode(packet))
            except (ValueError, UnicodeError):
                continue
        return result

    def close(self):
        self.socket.close()


class Guard:
    def __init__(self, clock=time.monotonic, limits=(0.3, 0.3, 0.6),
                 command_timeout=0.3, scan_timeout=0.5, status_timeout=1.5):
        if (len(limits) != 3 or any(not math.isfinite(v) or v <= 0 for v in limits)
                or any(not math.isfinite(v) or v <= 0 for v in
                       (command_timeout, scan_timeout, status_timeout))):
            raise ValueError('limits and timeouts must be positive and finite')
        self.clock = clock
        self.limits = limits
        self.command_timeout = command_timeout
        self.scan_timeout = scan_timeout
        self.status_timeout = status_timeout
        self.armed = False
        self.reason = 'not armed'
        self.status = {}
        self.status_at = self.scan_at = self.command_at = -math.inf
        self.command = ZERO
        self.scan_clear = False

    def disarm(self, reason):
        self.armed = False
        self.reason = reason
        self.command = ZERO
        self.command_at = -math.inf

    def update_status(self, items):
        # Some manual pages show flat Items; accept either complete layout.
        value = items.get('BasicStatus', items)
        required = ('MotionState', 'Gait', 'Charge', 'HES', 'ControlUsageMode', 'Sleep', 'Direction')
        if not isinstance(value, dict) or any(type(value.get(k)) is not int for k in required):
            self.status = {}
            self.disarm('incomplete BasicStatus')
            return
        self.status = value.copy()
        self.status_at = self.clock()
        if self.status_problem():
            self.disarm(self.status_problem())

    def status_problem(self):
        s = self.status
        if self.clock() - self.status_at > self.status_timeout:
            return 'BasicStatus stale'
        if s.get('MotionState') != 17:
            return 'not RL control'
        if s.get('ControlUsageMode') != 1:
            return 'not navigation usage mode'
        if s.get('Direction') != 0:
            return 'body forward direction not selected'
        if s.get('HES') != 0 or s.get('Charge') != 0 or s.get('Sleep') != 0:
            return 'estop, charging or sleeping'
        if s.get('Gait') not in GAITS:
            return 'unsupported gait'
        return ''

    def update_scan(self, valid, clear):
        self.scan_clear = valid and clear
        if valid:
            self.scan_at = self.clock()
        if not self.scan_clear:
            self.disarm('invalid scan or obstacle')

    def update_command(self, velocity):
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
            self.disarm('invalid velocity')
            return
        if self.armed:
            self.command = tuple(velocity)
            self.command_at = self.clock()

    def arm(self):
        problem = self.status_problem()
        if not self.scan_clear or self.clock() - self.scan_at > self.scan_timeout:
            problem = problem or 'scan unavailable'
        if problem:
            self.disarm(problem)
            return False
        self.armed = True
        self.command = ZERO
        # Require a new command after arming, allow one command interval.
        self.command_at = self.clock()
        self.reason = 'armed'
        return True

    def output(self):
        if not self.armed:
            return ZERO
        problem = self.status_problem()
        if self.clock() - self.scan_at > self.scan_timeout or not self.scan_clear:
            problem = problem or 'scan stale'
        if self.clock() - self.command_at > self.command_timeout:
            problem = problem or 'command stale'
        if problem:
            self.disarm(problem)
            return ZERO
        # Never round a tiny tracking request UP to the minimum gait speed.
        minimum = GAITS[self.status['Gait']]
        return tuple(0.0 if abs(v) < low else max(-limit, min(limit, v))
                     if limit >= low else 0.0
                     for v, low, limit in zip(self.command, minimum, self.limits))


def inspect_scan(scan, now, max_age=0.5, future_tolerance=0.1,
                 stop_front=0.7, stop_back=0.7, stop_half_width=0.4):
    """Scan coordinates must already be body x-forward/y-left. No self masking."""
    stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
    if (not math.isfinite(now) or not -future_tolerance <= now - stamp <= max_age
            or not scan.ranges or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0
            or not 0 <= scan.range_min < scan.range_max):
        return False, False
    valid = 0
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        valid += 1
        angle = scan.angle_min + i * scan.angle_increment
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if -stop_back <= x <= stop_front and abs(y) <= stop_half_width:
            return True, False
    return valid > 0, valid > 0
