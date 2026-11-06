"""Single-owner operator lease. No network or ROS dependencies."""
import math
import time
import secrets

from m20_adapter.core import ZERO


class OperatorControl:
    def __init__(self, guard, clock=time.monotonic, lease_timeout=0.6, preview=False):
        self.guard = guard
        self.clock = clock
        self.lease_timeout = lease_timeout
        self.preview = preview
        self.raw_velocity = ZERO
        self.owner = None
        self.mode = 'stopped'
        self.last_input = -math.inf
        self.sequences = {}
        self.target = None
        self.session = secrets.randbelow(2**51) + 1
        self.reason = 'Press start after checking the robot and target'

    def stop(self, reason):
        self.guard.disarm(reason)
        self.owner = None
        self.mode = 'stopped'
        self.target = None
        self.reason = reason
        self.raw_velocity = ZERO

    def tick(self):
        if self.owner and self.clock() - self.last_input > self.lease_timeout:
            self.stop('operator connection timed out; press start again')
        elif self.owner and self.preview and (not self.guard.scan_clear or
                self.clock() - self.guard.scan_at > self.guard.scan_timeout):
            self.stop('scan unavailable or obstacle')
        elif self.owner and not self.preview and not self.guard.armed:
            self.stop(self.guard.reason)

    @staticmethod
    def vector(value, size):
        try:
            valid = (isinstance(value, list) and len(value) == size
                     and all(type(v) in (int, float) and math.isfinite(v) for v in value))
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError('invalid finite vector')
        return tuple(value)

    def handle(self, request, source):
        self.tick()
        client = request.get('client')
        seq = request.get('seq')
        if (not isinstance(client, str) or not 1 <= len(client) <= 64
                or type(seq) is not int or not 0 <= seq < 2**53):
            raise ValueError('client and integer seq required')
        owner = source + ':' + client
        op = request.get('op')
        full = owner not in self.sequences and len(self.sequences) >= 128
        if full and op != 'stop':
            raise ValueError('session capacity reached; restart gateway while stopped')
        if seq <= self.sequences.get(owner, -1):
            raise ValueError('out-of-order or replayed command')
        if not full:
            self.sequences[owner] = seq
        # Any authenticated operator may STOP, but only one may drive.
        if op == 'stop':
            self.stop('operator stopped')
            return
        if op == 'action':
            raise ValueError('D1 actions are not mapped to M20; use factory controller')
        if self.owner is not None and self.owner != owner:
            raise ValueError('another operator owns control; stop before switching')
        if op == 'start':
            if self.owner is not None:
                raise ValueError('stop before restarting or changing mode')
            mode = request.get('mode')
            if mode not in ('direct', 'follow'):
                raise ValueError('mode must be direct or follow')
            target = self.vector(request.get('target'), 2) if mode == 'follow' else None
            if target and (target[0] <= 0 or math.hypot(*target) > 12):
                raise ValueError('select a target in front of the robot within 12 m')
            if self.preview and (not self.guard.scan_clear or
                    self.clock() - self.guard.scan_at > self.guard.scan_timeout):
                raise ValueError('scan unavailable or obstacle')
            if not self.preview and not self.guard.arm():
                raise ValueError(self.guard.reason)
            self.owner, self.mode, self.target = owner, mode, target
            self.session += 1
            self.last_input = self.clock()
            self.reason = 'operator active'
        elif op in ('heartbeat', 'velocity'):
            if self.owner != owner:
                raise ValueError('press start first; connection recovery never arms')
            if op == 'velocity':
                if self.mode != 'direct':
                    raise ValueError('velocity requires direct mode')
                velocity = self.vector(request.get('velocity'), 3)
                self.raw_velocity = velocity
                if not self.preview:
                    self.guard.update_command(velocity, source='operator')
            self.last_input = self.clock()
        else:
            raise ValueError('unsupported operation')

    def follow_command(self, velocity):
        self.tick()
        if self.owner and self.mode == 'follow':
            self.raw_velocity = self.vector(list(velocity), 3)
            if not self.preview:
                # Algorithm output keeps the algorithm reverse policy.
                self.guard.update_command(velocity, source='algorithm')

    def snapshot(self):
        return dict(owner=self.owner, mode=self.mode, target=self.target,
                    preview=self.preview, raw_velocity=self.raw_velocity,
                    reason=self.reason, actions_supported=[],
                    lease_timeout=self.lease_timeout)
