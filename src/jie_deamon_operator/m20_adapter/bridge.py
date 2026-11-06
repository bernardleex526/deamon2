"""ROS input to a documented basic_server motion command.

Dry-run never opens a socket.  Live mode selects Cmd=25 (SI, navigation mode) or
Cmd=21 (normalized, regular/auxiliary mode) from the live BasicStatus, so the
operator is no longer forced to switch usage mode just to make the bridge work.
An optional authenticated operator gateway (phone / Android) owns an exclusive
control lease on top of the same gate.
"""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import SetBool

from m20_adapter.core import (CMD_AXIS_NORMALIZED, GAIT_TABLE, GAITS, Guard, UdpClient,
                              inspect_scan_report, normalized_items, velocity_items)
from m20_adapter.operator_control import OperatorControl
from m20_adapter.operator_server import OperatorServer
from m20_adapter.preflight import check_preflight

DEFAULTS = dict(
    dry_run=True, commissioned=False, aos_host='10.21.31.103', aos_port=30000,
    control_profile='auto', allow_auxiliary_mode=False,
    max_vx=0.3, max_vy=0.3, max_wz=0.6,
    axis_enable_x=True, axis_enable_y=True, axis_enable_yaw=True,
    allow_reverse=False, allow_reverse_operator=True,
    gait_min_policy='zero', min_request=0.02,
    axis_max_x=2.0, axis_max_y=1.0, axis_max_yaw=2.0,
    scan_frame='lidar_link',
    stop_front=0.7, stop_back=0.7, stop_half_width=0.4, stop_hits_required=1,
    self_front=0.41, self_back=0.41, self_left=0.253, self_right=0.253,
    self_margin_front=0.05, self_margin_back=0.14,
    self_margin_left=0.07, self_margin_right=0.07,
    scan_min_range=0.0,
    guard_sector_min=-1.7453292519943295, guard_sector_max=1.7453292519943295,
    min_sector_bins=20,
    require_preflight=False, preflight_path='/tmp/m20_preflight.json',
    preflight_max_age=600.0, control_hz=20.0,
    enable_operator=False, operator_token_file='', operator_page='',
    operator_host='0.0.0.0', operator_port=8080, android_port=8889)


class Bridge(Node):
    def __init__(self):
        super().__init__('m20_bridge')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.dry_run = bool(p('dry_run'))
        self.commissioned = bool(p('commissioned'))
        if not self.dry_run and not self.commissioned:
            raise RuntimeError('live transport requires commissioned:=true')
        if self.get_parameter('use_sim_time').value and not self.dry_run:
            raise RuntimeError('live transport cannot use simulation time')
        self.enable_operator = bool(p('enable_operator'))
        self.preflight_report = None
        if not self.dry_run and bool(p('require_preflight')):
            self.preflight_report = check_preflight(
                p('preflight_path'), p('preflight_max_age'))
            if not self.preflight_report['ok']:
                raise RuntimeError('preflight gate rejected live start: '
                                   + self.preflight_report['reason'])
        self.guard = Guard(
            limits=(p('max_vx'), p('max_vy'), p('max_wz')),
            profile=p('control_profile'), allow_auxiliary=bool(p('allow_auxiliary_mode')),
            axis_enable=(p('axis_enable_x'), p('axis_enable_y'), p('axis_enable_yaw')),
            gait_table=GAIT_TABLE, gait_min_policy=p('gait_min_policy'),
            min_request=p('min_request'),
            axis_max=(p('axis_max_x'), p('axis_max_y'), p('axis_max_yaw')),
            stop_hits_required=p('stop_hits_required'),
            allow_reverse=bool(p('allow_reverse')),
            allow_reverse_operator=bool(p('allow_reverse_operator')),
            # With the operator gateway the command stream starts only after the
            # operator presses start, so a pre-existing stream cannot be required.
            arm_requires_command=not self.enable_operator)
        self.scan_frame = p('scan_frame')
        self.stop = dict(stop_front=p('stop_front'), stop_back=p('stop_back'),
                         stop_half_width=p('stop_half_width'))
        self.self_mask = dict(self_front=p('self_front'), self_back=p('self_back'),
                              self_left=p('self_left'), self_right=p('self_right'),
                              self_margin_front=p('self_margin_front'),
                              self_margin_back=p('self_margin_back'),
                              self_margin_left=p('self_margin_left'),
                              self_margin_right=p('self_margin_right'))
        self.sector = dict(sector_min=p('guard_sector_min'),
                           sector_max=p('guard_sector_max'),
                           min_sector_bins=int(p('min_sector_bins')))
        if any(not math.isfinite(v) or v <= 0 for v in self.stop.values()):
            raise ValueError('stop dimensions must be positive and finite')
        self.min_range = float(p('scan_min_range'))
        self.client = None if self.dry_run else UdpClient(p('aos_host'), p('aos_port'))
        self.last_query = -float('inf')
        self.last_kind = None
        self.last_reason = None
        self.last_command = None
        self.last_report = None
        self.counters = dict(commands=0, armed_commands=0, scans=0, statuses=0)
        self.operator = None
        self.server = None
        self.follow_key = None
        self.follow_ready = False
        self.follow_started = -float('inf')
        self.target_pub = self.create_publisher(Float64MultiArray, '/robot_nexus/operator_target', 1)
        self.create_subscription(Float64MultiArray, '/robot_nexus/operator_velocity',
                                 self.tagged_command, 1)
        self.scan_preview = []
        if self.enable_operator:
            token_file = p('operator_token_file') or os.environ.get('M20_OPERATOR_TOKEN_FILE', '')
            if not token_file:
                raise ValueError('set M20_OPERATOR_TOKEN_FILE or use scripts/m20ctl preview')
            token = Path(token_file).read_text().strip()
            self.operator = OperatorControl(self.guard, preview=self.dry_run)
            self.server = OperatorServer(token, Path(p('operator_page')).read_bytes(),
                                         p('operator_host'), p('operator_port'), p('android_port'))
            self.server.start()
        self.preview = self.create_publisher(Twist, '/m20/cmd_vel_guarded', 1)
        self.status_pub = self.create_publisher(String, '/m20/bridge_status', 1)
        self.report_pub = self.create_publisher(String, '/m20/scan_report', 1)
        self.create_subscription(Twist, '/m20/cmd_vel_raw', self.command, 1)
        self.create_subscription(LaserScan, '/m20/scan', self.scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/m20/arm', self.arm)
        if self.dry_run:
            self.create_subscription(String, '/m20/mock_basic_status', self.mock_status, 1)
        period = 1.0 / max(1.0, float(p('control_hz')))
        self.timer = self.create_timer(period, self.tick,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            'm20_bridge started: dry_run=%s profile=%s operator=%s y_axis=%s gait_min_policy=%s'
            % (self.dry_run, p('control_profile'), self.enable_operator,
               p('axis_enable_y'), p('gait_min_policy')))

    def command(self, msg):
        """Raw algorithm intent; ignored while the operator lease owns control."""
        self.counters['commands'] += 1
        if self.guard.armed:
            self.counters['armed_commands'] += 1
        self.last_command = (msg.linear.x, msg.linear.y, msg.angular.z)
        if os.environ.get('M20_BRIDGE_DEBUG'):
            self.get_logger().warn('cmd armed=%s reason=%r vx=%.3f'
                                   % (self.guard.armed, self.guard.reason, msg.linear.x))
        if not self.operator:
            self.guard.update_command(self.last_command, source='algorithm')

    def tagged_command(self, msg):
        if (self.operator and self.operator.mode == 'follow' and len(msg.data) == 4
                and msg.data[0] == self.operator.session):
            self.follow_ready = True
            self.operator.follow_command(tuple(msg.data[1:]))

    def scan(self, msg):
        self.counters['scans'] += 1
        report = inspect_scan_report(
            msg, self.get_clock().now().nanoseconds / 1e9,
            min_range=self.min_range, **self.stop, **self.self_mask, **self.sector)
        if msg.header.frame_id != self.scan_frame:
            report.valid = False
            report.reason = 'wrong frame: %r' % msg.header.frame_id
        self.last_report = report
        self.guard.update_scan(report.valid, report.clear, report.stop_hits)
        if self.report_pub.get_subscription_count():
            out = String()
            out.data = json.dumps(report.as_dict())
            self.report_pub.publish(out)
        if self.operator:
            self.scan_preview = [
                [distance * math.cos(msg.angle_min + i * msg.angle_increment),
                 distance * math.sin(msg.angle_min + i * msg.angle_increment)]
                for i, distance in enumerate(msg.ranges)
                if i % 4 == 0 and math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max]

    def arm(self, request, response):
        if self.operator and request.data:
            response.success = False
            response.message = 'Use the operator start button (exclusive control lease)'
            return response
        if self.operator:
            self.operator.stop('ROS operator stopped')
        if request.data:
            response.success = self.guard.arm()
        else:
            self.guard.disarm('operator disarmed')
            response.success = True
        response.message = self.guard.reason
        return response

    def mock_status(self, msg):
        self.counters['statuses'] += 1
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('status must be an object')
            self.guard.update_status(value)
        except ValueError:
            self.guard.disarm('invalid mock status')

    def _zero_items(self):
        if self.guard.command_kind == CMD_AXIS_NORMALIZED:
            return normalized_items((0.0, 0.0, 0.0), self.guard.axis_max)
        return velocity_items((0.0, 0.0, 0.0))

    def tick(self):
        try:
            if self.client:
                for _, value in self.client.receive():
                    items = value['Items']
                    if items.get('ErrorCode', 0) != 0:
                        self.guard.disarm('basic_server rejected command: ' + str(items))
                    elif value['Type'] == 1002 and value['Command'] == 6:
                        # Query ACKs are not status and must not refresh freshness.
                        if 'BasicStatus' in items or 'MotionState' in items:
                            self.guard.update_status(items)
                if time.monotonic() - self.last_query >= 1.0:
                    self.client.heartbeat()
                    self.last_query = time.monotonic()
            if self.operator:
                self.operator.tick()
                self.server.drain(self.operator)
                self.sync_follow()
            velocity = self.guard.output()
            if self.client and self.guard.command_kind is not None:
                self.client.send(2, self.guard.command_kind, self.guard.wire_items())
            if self.guard.command_kind != self.last_kind:
                self.get_logger().warn('control command changed: %s -> %s (usage mode %s)'
                                       % (self.last_kind, self.guard.command_kind,
                                          self.guard.status.get('ControlUsageMode')))
                self.last_kind = self.guard.command_kind
            if self.guard.reason != self.last_reason:
                self.get_logger().warn('gate reason: %r -> %r (armed=%s, since_command=%.3f)'
                                       % (self.last_reason, self.guard.reason, self.guard.armed,
                                          time.monotonic() - self.guard.command_at))
                self.last_reason = self.guard.reason
        except OSError as exc:
            self.guard.disarm('UDP failure: ' + str(exc))
            velocity = (0.0, 0.0, 0.0)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity
        self.preview.publish(msg)
        status = self.guard.status_dict()
        status.update(dry_run=self.dry_run, velocity=list(velocity),
                      counters=dict(self.counters),
                      last_command=None if self.last_command is None else list(self.last_command),
                      scan=None if self.last_report is None else self.last_report.as_dict())
        out = String()
        out.data = json.dumps(status)
        self.status_pub.publish(out)
        if self.server:
            minimums = GAITS.get(self.guard.status.get('Gait'),
                                 (0., 0., 0.) if self.dry_run else None)
            speeds = [min(cap, max(low, preferred)) if low <= cap else 0.
                      for cap, low, preferred in zip(self.guard.limits, minimums,
                                                     (.2, .25, .4))] if minimums else [0., 0., 0.]
            self.server.update(dict(ready=True, dry_run=self.dry_run,
                                    armed=self.guard.armed, reason=self.guard.reason,
                                    velocity=velocity, operator=self.operator.snapshot(),
                                    scan=self.scan_preview,
                                    follow_ready=self.follow_ready,
                                    control_speeds=speeds,
                                    speed_limits=self.guard.limits,
                                    gait_minimums=minimums,
                                    axis_notes=self.guard.axis_notes,
                                    command_kind=self.guard.command_kind,
                                    robot_status=self.guard.status))

    def sync_follow(self):
        key = self.operator.session if self.operator.mode == 'follow' else None
        if key != self.follow_key:
            self.follow_key = key
            self.follow_ready = False
            self.follow_started = time.monotonic()
            target = self.operator.target if key else (0., 0.)
            msg = Float64MultiArray()
            msg.data = [float(key or 0), float(target[0]), float(target[1])]
            self.target_pub.publish(msg)
        if key and not self.follow_ready:
            self.guard.update_command((0., 0., 0.), source='algorithm')
            if time.monotonic() - self.follow_started > 1.0:
                self.operator.stop('follow node did not acknowledge the selected target')

    def close(self):
        if self.operator:
            self.operator.stop('bridge shutting down')
        if self.client:
            try:
                self.client.send(2, self.guard.command_kind or 25, self._zero_items())
            except OSError:
                pass
            self.client.close()
        if self.server:
            self.server.close()


def main():
    rclpy.init()
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
