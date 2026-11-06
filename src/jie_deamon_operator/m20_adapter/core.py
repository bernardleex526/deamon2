"""M20 basic_server framing, gait-aware fail-closed velocity gate, scan inspection.

Design notes (see docs/M20_BLOCKERS_CLEARED.md):

* ``Cmd=25`` (SI m/s, rad/s) is only accepted by the robot in *navigation* usage
  mode; ``Cmd=21`` (normalized [-1, 1] per axis) is only accepted in *regular*
  and *auxiliary* mode.  The gate therefore no longer hard-codes
  ``ControlUsageMode == 1``: it checks the mode that matches the selected
  control profile, and ``auto`` picks the profile from the live BasicStatus.
* Vendor-documented per-gait effective speed ranges (software >= V1.1.7) are
  encoded in :data:`GAIT_TABLE`.  A command below the gait minimum is either
  dropped (``zero`` policy, default, fail-closed) or snapped up to the minimum
  (``snap`` policy, opt-in) but is never silently rounded up without saying so.
* Stop-zone inspection ignores returns inside the body footprint plus a margin,
  because lidar returns from the robot's own legs are persistent and otherwise
  make the bridge impossible to arm.
"""
import datetime
import json
import math
import socket
import struct
import time

HEADER = struct.Struct('<4sHHB7s')
MAGIC = bytes.fromhex('eb91eb90')
ZERO = (0.0, 0.0, 0.0)

#: Vendor BasicStatus ControlUsageMode values (modes.txt).
MODE_REGULAR = 0
MODE_NAV = 1
MODE_AUX = 2
MODE_NAMES = {MODE_REGULAR: 'regular', MODE_NAV: 'navigation', MODE_AUX: 'auxiliary'}

#: basic_server Type=2 sub-commands.
CMD_AXIS_NORMALIZED = 21   # regular / auxiliary mode only
CMD_VELOCITY_SI = 25       # navigation mode only

#: gait id -> (name, x_min, y_min, yaw_min, x_max, y_max, yaw_max)
#: Ranges are copied from the vendor "运动控制（basic_server 协议）" page,
#: section 4.5 "各个步态的有效速度范围" (software >= V1.1.7).
GAIT_TABLE = {
    4097: ('standard-basic', 0.20, 0.35, 0.50, 2.0, 1.0, 2.0),
    4099: ('standard-stairs', 0.15, 0.30, 0.40, 2.0, 1.0, 2.0),
    12290: ('agile-flat', 0.15, 0.25, 0.35, 2.0, 1.0, 1.5),
    12291: ('agile-stairs', 0.15, 0.30, 0.40, 2.0, 1.0, 2.0),
}

#: Backwards-compatible minimum-speed view (x_min, y_min, yaw_min).
GAITS = {gait: (row[1], row[2], row[3]) for gait, row in GAIT_TABLE.items()}

#: Nominal per-axis full-scale values used to normalize Cmd=21.  The vendor
#: page only says "relative to that axis' maximum speed"; these are the largest
#: documented values for any gait and MUST be verified on the robot.
DEFAULT_AXIS_MAX = (2.0, 1.0, 2.0)


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


def normalize_velocity(velocity, axis_max=DEFAULT_AXIS_MAX):
    """Convert SI (m/s, rad/s) to the Cmd=21 normalized [-1, 1] representation."""
    out = []
    for value, full_scale in zip(velocity, axis_max):
        if not math.isfinite(value):
            raise ValueError('nonfinite velocity')
        if not math.isfinite(full_scale) or full_scale <= 0.0:
            raise ValueError('axis maximum must be positive and finite')
        out.append(max(-1.0, min(1.0, value / full_scale)))
    return tuple(out)


def normalized_items(velocity, axis_max=DEFAULT_AXIS_MAX):
    x, y, yaw = normalize_velocity(velocity, axis_max)
    return dict(X=x, Y=y, Z=0.0, Roll=0.0, Pitch=0.0, Yaw=yaw)


def select_command(profile, usage_mode, allow_auxiliary=False):
    """Return the basic_server sub-command for a profile/mode pair.

    ``profile`` is ``auto``, ``si`` or ``normalized``.  Returns ``None`` when the
    combination is not permitted by the vendor documentation.
    """
    if profile == 'si':
        return CMD_VELOCITY_SI if usage_mode == MODE_NAV else None
    if profile == 'normalized':
        if usage_mode == MODE_REGULAR:
            return CMD_AXIS_NORMALIZED
        if usage_mode == MODE_AUX and allow_auxiliary:
            return CMD_AXIS_NORMALIZED
        return None
    if profile == 'auto':
        if usage_mode == MODE_NAV:
            return CMD_VELOCITY_SI
        if usage_mode == MODE_REGULAR:
            return CMD_AXIS_NORMALIZED
        if usage_mode == MODE_AUX and allow_auxiliary:
            return CMD_AXIS_NORMALIZED
        return None
    raise ValueError('unknown control profile: ' + str(profile))


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
    """Fail-closed velocity gate.

    ``limits`` are the operator's safety limits in SI units.  ``axis_max`` is the
    per-axis full scale used only when normalizing for Cmd=21.  ``profile`` is
    the requested control profile; the gate resolves the actual sub-command from
    the live usage mode and refuses to pass velocity when they disagree.
    """

    def __init__(self, clock=time.monotonic, limits=(0.3, 0.3, 0.6),
                 command_timeout=0.3, scan_timeout=0.5, status_timeout=1.5,
                 profile='auto', allow_auxiliary=False,
                 axis_enable=(True, True, True), gait_table=None,
                 gait_min_policy='zero', min_request=0.02,
                 axis_max=DEFAULT_AXIS_MAX, stop_hits_required=1,
                 allow_reverse=False, allow_reverse_operator=True,
                 arm_requires_command=True):
        if (len(limits) != 3 or any(not math.isfinite(v) or v <= 0 for v in limits)
                or any(not math.isfinite(v) or v <= 0 for v in
                       (command_timeout, scan_timeout, status_timeout))):
            raise ValueError('limits and timeouts must be positive and finite')
        if profile not in ('auto', 'si', 'normalized'):
            raise ValueError('profile must be auto, si or normalized')
        if gait_min_policy not in ('zero', 'snap'):
            raise ValueError('gait_min_policy must be zero or snap')
        if len(axis_enable) != 3 or len(axis_max) != 3:
            raise ValueError('axis_enable and axis_max must have three entries')
        if any(not math.isfinite(v) or v <= 0 for v in axis_max):
            raise ValueError('axis_max must be positive and finite')
        if int(stop_hits_required) < 1:
            raise ValueError('stop_hits_required must be >= 1')
        self.clock = clock
        self.limits = tuple(limits)
        self.command_timeout = command_timeout
        self.scan_timeout = scan_timeout
        self.status_timeout = status_timeout
        self.profile = profile
        self.allow_auxiliary = bool(allow_auxiliary)
        self.axis_enable = tuple(bool(v) for v in axis_enable)
        self.gait_table = dict(gait_table or GAIT_TABLE)
        self.gait_min_policy = gait_min_policy
        self.min_request = float(min_request)
        self.axis_max = tuple(axis_max)
        self.stop_hits_required = int(stop_hits_required)
        self.allow_reverse = bool(allow_reverse)
        self.allow_reverse_operator = bool(allow_reverse_operator)
        self.arm_requires_command = bool(arm_requires_command)
        self.command_source = 'algorithm'
        self.armed = False
        self.reason = 'not armed'
        self.status = {}
        self.status_at = self.scan_at = self.command_at = -math.inf
        self.command = ZERO
        self.scan_clear = False
        self.command_kind = None
        self.axis_notes = []
        self._moving = [False, False, False]
        self._hit_streak = 0

    # ---------------------------------------------------------------- status
    def disarm(self, reason):
        self.armed = False
        self.reason = reason
        self.command = ZERO
        self.command_at = -math.inf
        self._moving = [False, False, False]

    def update_status(self, items):
        # Some manual pages show flat Items; accept either complete layout.
        value = items.get('BasicStatus', items)
        required = ('MotionState', 'Gait', 'Charge', 'HES', 'ControlUsageMode', 'Sleep', 'Direction')
        if not isinstance(value, dict) or any(type(value.get(k)) is not int for k in required):
            self.status = {}
            self.command_kind = None
            self.disarm('incomplete BasicStatus')
            return
        self.status = value.copy()
        self.status_at = self.clock()
        self.command_kind = select_command(self.profile, value['ControlUsageMode'],
                                           self.allow_auxiliary)
        self._refresh_axis_notes()
        problem = self.status_problem()
        if problem:
            self.disarm(problem)

    def _refresh_axis_notes(self):
        """Record axes that the current gait/limit combination cannot use."""
        notes = []
        gait = self.status.get('Gait')
        row = self.gait_table.get(gait)
        if row:
            for index, axis in enumerate(('x', 'y', 'yaw')):
                if not self.axis_enable[index]:
                    notes.append('%s axis disabled by configuration' % axis)
                    continue
                effective_max = min(self.limits[index], row[4 + index])
                if row[1 + index] > effective_max:
                    notes.append('%s axis unusable: gait %s minimum %.2f > limit %.2f'
                                 % (axis, row[0], row[1 + index], effective_max))
        self.axis_notes = notes

    def status_problem(self):
        s = self.status
        if self.clock() - self.status_at > self.status_timeout:
            return 'BasicStatus stale'
        if s.get('MotionState') != 17:
            return 'not RL control'
        if self.command_kind is None:
            mode = s.get('ControlUsageMode')
            return ('usage mode %s (%s) incompatible with control profile %s'
                    % (mode, MODE_NAMES.get(mode, 'unknown'), self.profile))
        if s.get('Direction') != 0:
            return 'body forward direction not selected'
        if s.get('HES') != 0 or s.get('Charge') != 0 or s.get('Sleep') != 0:
            return 'estop, charging or sleeping'
        if s.get('Gait') not in self.gait_table:
            return 'unsupported gait'
        return ''

    # ------------------------------------------------------------------ scan
    def update_scan(self, valid, clear, hits=0):
        """Feed one scan.

        ``hits`` is the number of stop-zone returns in this frame; the gate needs
        ``stop_hits_required`` consecutive frames with hits before it declares an
        obstacle, which suppresses single-frame noise while staying fail-closed
        for persistent returns.
        """
        self.scan_clear = bool(valid) and bool(clear)
        if valid:
            self.scan_at = self.clock()
        self._hit_streak = self._hit_streak + 1 if (valid and not clear) else 0
        if valid and not clear and self._hit_streak < self.stop_hits_required:
            # Not enough evidence yet: keep the previous verdict but do not
            # disarm on a single noisy frame.
            return
        if not self.scan_clear:
            self.disarm('invalid scan or obstacle')

    def update_command(self, velocity, source='algorithm'):
        """Record a velocity request from ``algorithm`` or ``operator``.

        The arrival time is recorded even while disarmed, so that ``arm()`` can
        refuse to enable a gate whose velocity stream is not actually live.  The
        value itself is only latched while armed.  ``source`` selects the reverse
        policy: algorithm output may not drive backwards by default, while an
        operator teleoperation client may.
        """
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
            self.disarm('invalid velocity')
            return
        self.command_at = self.clock()
        self.command_source = source if source in ('algorithm', 'operator') else 'algorithm'
        if self.armed:
            self.command = tuple(velocity)

    # ------------------------------------------------------------------ gate
    def arm(self):
        problem = self.status_problem()
        if not self.scan_clear or self.clock() - self.scan_at > self.scan_timeout:
            problem = problem or 'scan unavailable'
        if self.arm_requires_command and \
                self.clock() - self.command_at > self.command_timeout:
            problem = problem or 'no fresh command stream'
        if problem:
            self.disarm(problem)
            return False
        self.armed = True
        self.command = ZERO
        # Require a new command after arming, allow one command interval.
        self.command_at = self.clock()
        self.reason = 'armed'
        return True

    def _quantize(self, index, value):
        """Apply axis enable, safety limit, gait range and minimum-speed policy.

        ``zero`` (default) drops any request below the gait minimum; ``snap``
        raises a non-zero request to the gait minimum because the vendor range is
        exclusive around zero.  ``min_request`` is a noise gate so that a few
        millimetres of tracking error do not start a step.
        """
        if not self.axis_enable[index]:
            return 0.0
        if index == 0 and value < 0.0:
            # Algorithm output never drives backwards by default; a target closer
            # than follow_distance stops the robot instead of reversing into
            # blind space behind it.  An operator teleoperation client may reverse.
            allowed = (self.allow_reverse_operator if self.command_source == 'operator'
                       else self.allow_reverse)
            if not allowed:
                self._moving[index] = False
                return 0.0
        row = self.gait_table.get(self.status.get('Gait'))
        if row is None:
            return 0.0
        low = row[1 + index]
        high = min(self.limits[index], row[4 + index])
        if low > high:
            # The gait cannot move this axis inside the operator's safety limit.
            self._moving[index] = False
            return 0.0
        magnitude = min(abs(value), high)
        if magnitude < self.min_request:
            self._moving[index] = False
            return 0.0
        if magnitude >= low:
            self._moving[index] = True
            return math.copysign(magnitude, value)
        if self.gait_min_policy == 'snap':
            self._moving[index] = True
            return math.copysign(low, value)
        self._moving[index] = False
        return 0.0

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
        # Never round a tiny tracking request UP to the minimum gait speed unless
        # the operator explicitly opted into the snap policy.
        return tuple(self._quantize(i, v) for i, v in enumerate(self.command))

    def wire_items(self):
        """Items payload for the currently selected sub-command."""
        if self.command_kind == CMD_AXIS_NORMALIZED:
            return normalized_items(self.output(), self.axis_max)
        return velocity_items(self.output())

    def status_dict(self):
        mode = self.status.get('ControlUsageMode')
        now = self.clock()
        return dict(armed=self.armed, reason=self.reason, command_kind=self.command_kind,
                    usage_mode=mode, usage_mode_name=MODE_NAMES.get(mode, 'unknown'),
                    gait=self.status.get('Gait'), axis_notes=self.axis_notes,
                    limits=list(self.limits), axis_enable=list(self.axis_enable),
                    gait_min_policy=self.gait_min_policy,
                    command_source=self.command_source,
                    arm_requires_command=self.arm_requires_command,
                    allow_reverse=self.allow_reverse,
                    allow_reverse_operator=self.allow_reverse_operator,
                    since_command=round(now - self.command_at, 3),
                    since_scan=round(now - self.scan_at, 3),
                    since_status=round(now - self.status_at, 3))


class ScanReport:
    """Result of one scan inspection."""
    __slots__ = ('valid', 'clear', 'valid_bins', 'stop_hits', 'self_hits',
                 'age', 'closest_stop', 'closest_self', 'reason', 'sector_bins')

    def __init__(self, valid, clear, valid_bins=0, stop_hits=0, self_hits=0,
                 age=float('nan'), closest_stop=None, closest_self=None, reason=''):
        self.valid = valid
        self.clear = clear
        self.valid_bins = valid_bins
        self.stop_hits = stop_hits
        self.self_hits = self_hits
        self.age = age
        self.closest_stop = closest_stop
        self.closest_self = closest_self
        self.reason = reason
        self.sector_bins = 0

    def as_dict(self):
        return dict(valid=self.valid, clear=self.clear, valid_bins=self.valid_bins,
                    sector_bins=self.sector_bins, stop_hits=self.stop_hits,
                    self_hits=self.self_hits,
                    age=None if math.isnan(self.age) else round(self.age, 3),
                    closest_stop=self.closest_stop, closest_self=self.closest_self,
                    reason=self.reason)


def inspect_scan_report(scan, now, max_age=0.5, future_tolerance=0.1,
                        stop_front=0.7, stop_back=0.7, stop_half_width=0.4,
                        self_front=0.41, self_back=0.41, self_left=0.253,
                        self_right=0.253, self_margin_front=0.05,
                        self_margin_back=0.15, self_margin_left=0.15,
                        self_margin_right=0.15, min_range=0.0,
                        sector_min=-1.7453292519943295, sector_max=1.7453292519943295,
                        min_sector_bins=1):
    """Scan coordinates must already be body x-forward/y-left.

    Returns a :class:`ScanReport`.  ``valid`` means the scan itself is usable;
    ``clear`` means no stop-zone return outside the (body + margin) footprint.
    Returns inside the footprint are counted as ``self_hits`` and never block the
    gate, which is what allows the bridge to arm despite persistent leg returns.

    Margins are per side on purpose: the rear and flanks need a generous margin
    because the legs sweep outside the 0.82 x 0.506 m torso box, while the front
    keeps a small margin so the forward stop zone stays wide.
    """
    stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
    age = now - stamp
    if (not math.isfinite(now) or not -future_tolerance <= age <= max_age
            or not scan.ranges or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0
            or not 0 <= scan.range_min < scan.range_max):
        return ScanReport(False, False, age=age, reason='malformed or stale scan')
    front = self_front + self_margin_front
    back = self_back + self_margin_back
    left = self_left + self_margin_left
    right = self_right + self_margin_right
    valid_bins = stop_hits = self_hits = sector_bins = 0
    closest_stop = closest_self = None
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        valid_bins += 1
        angle = scan.angle_min + i * scan.angle_increment
        if not sector_min <= angle <= sector_max:
            continue
        sector_bins += 1
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if distance < min_range:
            continue
        if -back <= x <= front and -right <= y <= left:
            self_hits += 1
            if closest_self is None or distance < closest_self['distance']:
                closest_self = dict(x=round(x, 4), y=round(y, 4), distance=round(distance, 4))
            continue
        if -stop_back <= x <= stop_front and abs(y) <= stop_half_width:
            stop_hits += 1
            if closest_stop is None or distance < closest_stop['distance']:
                closest_stop = dict(x=round(x, 4), y=round(y, 4), distance=round(distance, 4))
    if sector_bins == 0:
        return ScanReport(False, False, valid_bins, stop_hits, self_hits, age,
                          closest_stop, closest_self, 'no valid returns in guard sector')
    if sector_bins < min_sector_bins:
        return ScanReport(False, False, valid_bins, stop_hits, self_hits, age,
                          closest_stop, closest_self,
                          'guard sector coverage too low (%d < %d)'
                          % (sector_bins, min_sector_bins))
    report = ScanReport(True, stop_hits == 0, valid_bins, stop_hits, self_hits, age,
                        closest_stop, closest_self,
                        '' if stop_hits == 0 else 'stop-zone returns')
    report.sector_bins = sector_bins
    return report


def inspect_scan(scan, now, max_age=0.5, future_tolerance=0.1,
                 stop_front=0.7, stop_back=0.7, stop_half_width=0.4,
                 self_front=0.41, self_back=0.41, self_left=0.253, self_right=0.253,
                 self_margin_front=0.05, self_margin_back=0.15, self_margin_left=0.15,
                 self_margin_right=0.15, min_range=0.0):
    """Backwards-compatible wrapper returning ``(valid, clear)``."""
    report = inspect_scan_report(
        scan, now, max_age=max_age, future_tolerance=future_tolerance,
        stop_front=stop_front, stop_back=stop_back, stop_half_width=stop_half_width,
        self_front=self_front, self_back=self_back, self_left=self_left,
        self_right=self_right, self_margin_front=self_margin_front,
        self_margin_back=self_margin_back, self_margin_left=self_margin_left,
        self_margin_right=self_margin_right, min_range=min_range)
    return report.valid, report.clear
