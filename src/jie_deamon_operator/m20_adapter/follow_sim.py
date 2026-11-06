"""Moving-person follow simulation for the M20 stack.

Publishes a synthetic lidar cloud containing a person that walks along a
scripted path, drives the dry-run bridge with synthetic BasicStatus, and checks
that the algorithm emits the expected ``/m20/cmd_vel_raw`` and that the gate
emits the expected ``/m20/cmd_vel_guarded``.

It never contacts AOS, never opens a UDP control socket, and refuses to run
outside an isolated domain.
"""
import json
import math
import os
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import SetBool

ROOM_RADIUS = 5.0
PERSON_HALF_WIDTH = 0.15
WALK_STEP = 0.04          # m per 0.1 s tick -> 0.4 m/s, keeps centroid tracking
SELF_RETURNS = [(0.40, 0.20, 0.1), (-0.37, 0.26, 0.1)]


class PersonSim(Node):
    """Synthetic cloud + BasicStatus source and velocity observer."""

    def __init__(self):
        super().__init__('m20_follow_sim')
        self.declare_parameter('usage_mode', 1)
        self.declare_parameter('gait', 12290)
        self.cloud = self.create_publisher(PointCloud2, '/m20/mock_points',
                                           qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/m20/mock_basic_status', 1)
        self.observed = dict(raw=(0.0, 0.0, 0.0), guarded=(0.0, 0.0, 0.0),
                             status={}, scan_at=0.0, scan_frame='', scan_bins=0)
        self.create_subscription(Twist, '/m20/cmd_vel_raw', self._raw, 1)
        self.create_subscription(Twist, '/m20/cmd_vel_guarded', self._guarded, 1)
        self.create_subscription(String, '/m20/bridge_status', self._status, 1)
        self.create_subscription(LaserScan, '/m20/scan', self._scan, qos_profile_sensor_data)
        self.person = None
        self.goal = None
        self.obstacles = []
        self.self_returns = False
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ observation
    def _raw(self, msg):
        self.observed['raw'] = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _guarded(self, msg):
        self.observed['guarded'] = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _status(self, msg):
        try:
            self.observed['status'] = json.loads(msg.data)
        except ValueError:
            pass

    def _scan(self, msg):
        self.observed['scan_at'] = time.monotonic()
        self.observed['scan_frame'] = msg.header.frame_id
        self.observed['scan_bins'] = len(msg.ranges)

    # --------------------------------------------------------------- scenario
    def configure(self, person=None, obstacles=(), self_returns=False,
                  usage_mode=1, gait=12290):
        self.goal = person
        if person is None:
            self.person = None
        elif self.person is None:
            self.person = person
        self.obstacles = list(obstacles)
        self.self_returns = self_returns
        self.set_parameters([Parameter('usage_mode', value=int(usage_mode)),
                             Parameter('gait', value=int(gait))])

    def person_settled(self):
        if self.goal is None:
            return self.person is None
        if self.person is None:
            return False
        return math.dist(self.person, self.goal) < 1e-6

    def tick(self):
        if self.goal is not None and self.person is not None:
            dx, dy = self.goal[0] - self.person[0], self.goal[1] - self.person[1]
            distance = math.hypot(dx, dy)
            if distance <= WALK_STEP:
                self.person = tuple(self.goal)
            else:
                self.person = (self.person[0] + dx / distance * WALK_STEP,
                               self.person[1] + dy / distance * WALK_STEP)
        self._publish_cloud()
        status = String()
        status.data = json.dumps(dict(BasicStatus=dict(
            MotionState=17, Gait=int(self.get_parameter('gait').value),
            Charge=0, HES=0, Sleep=0, Direction=0,
            ControlUsageMode=int(self.get_parameter('usage_mode').value))))
        self.status.publish(status)

    def _publish_cloud(self):
        points = [(ROOM_RADIUS * math.cos(i * math.pi / 180),
                   ROOM_RADIUS * math.sin(i * math.pi / 180), 0.2) for i in range(360)]
        if self.person is not None:
            x, y = self.person
            for i in range(-10, 11):
                points.append((x, y + i * (PERSON_HALF_WIDTH / 10.0), 0.2))
        points.extend((x, y, z) for x, y, z in self.obstacles)
        if self.self_returns:
            points.extend(SELF_RETURNS)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.height, msg.width = 1, len(points)
        msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.point_step, msg.row_step = 12, len(points) * 12
        msg.is_bigendian, msg.is_dense = False, True
        msg.data = b''.join(struct.pack('<fff', *point) for point in points)
        self.cloud.publish(msg)


def _near(value, target, tol):
    return abs(value - target) <= tol


SCENARIOS = [
    dict(name='01_approach_2.0m', person=(2.0, 0.0), settle=2.5,
         checks=[('raw vx forward', lambda o: o['raw'][0] > 0.25),
                 ('guarded vx forward', lambda o: o['guarded'][0] > 0.25),
                 ('no lateral', lambda o: _near(o['raw'][1], 0.0, 1e-6)),
                 ('armed', lambda o: o['status'].get('armed') is True)]),
    dict(name='02_hold_at_follow_distance', person=(1.2, 0.0), settle=2.5,
         checks=[('raw vx ~0', lambda o: _near(o['raw'][0], 0.0, 0.05)),
                 ('guarded vx 0', lambda o: _near(o['guarded'][0], 0.0, 1e-6))]),
    dict(name='03_retreat_blocked', person=(0.9, 0.0), settle=2.5,
         checks=[('raw vx negative', lambda o: o['raw'][0] < -0.1),
                 ('guarded vx 0 (no reverse)', lambda o: _near(o['guarded'][0], 0.0, 1e-6))]),
    dict(name='04_turn_left_large', person=(1.0, 1.0), settle=3.0,
         checks=[('raw wz positive', lambda o: o['raw'][2] > 0.3),
                 ('guarded wz above gait min', lambda o: o['guarded'][2] > 0.3)]),
    dict(name='05_turn_right_large', person=(1.0, -1.0), settle=3.0,
         checks=[('raw wz negative', lambda o: o['raw'][2] < -0.3),
                 ('guarded wz negative', lambda o: o['guarded'][2] < -0.3)]),
    dict(name='06_small_angle_below_gait_min', person=(1.4, 0.35), settle=3.0,
         checks=[('raw wz small positive', lambda o: 0.1 < o['raw'][2] < 0.35),
                 ('guarded wz 0 by gait minimum', lambda o: _near(o['guarded'][2], 0.0, 1e-6))]),
    dict(name='07_target_lost', person=None, settle=2.0,
         checks=[('raw zero', lambda o: o['raw'] == (0.0, 0.0, 0.0)),
                 ('guarded zero', lambda o: o['guarded'] == (0.0, 0.0, 0.0))]),
    dict(name='08_self_returns_do_not_block', person=(2.0, 0.0), self_returns=True,
         settle=2.5, arm=True,
         checks=[('armed despite leg returns', lambda o: o['status'].get('armed') is True),
                 ('guarded vx forward', lambda o: o['guarded'][0] > 0.25)]),
    dict(name='09_front_obstacle_disarms', person=(2.0, 0.0), obstacles=[(0.5, 0.0, 0.2)],
         settle=2.0,
         checks=[('disarmed', lambda o: o['status'].get('armed') is False),
                 ('reason mentions obstacle',
                  lambda o: 'obstacle' in str(o['status'].get('reason'))),
                 ('guarded zero', lambda o: o['guarded'] == (0.0, 0.0, 0.0))]),
    dict(name='10_rear_obstacle_outside_sector', person=(2.0, 0.0), obstacles=[(-0.6, 0.0, 0.2)],
         settle=2.5, arm=True,
         checks=[('armed (rear outside +-100deg)', lambda o: o['status'].get('armed') is True),
                 ('guarded vx forward', lambda o: o['guarded'][0] > 0.25)]),
    dict(name='11_regular_mode_cmd21', person=(2.0, 0.0), usage_mode=0,
         settle=2.5, arm=True,
         checks=[('command_kind 21', lambda o: o['status'].get('command_kind') == 21),
                 ('usage_mode 0', lambda o: o['status'].get('usage_mode') == 0),
                 ('guarded vx forward', lambda o: o['guarded'][0] > 0.25)]),
    dict(name='12_navigation_mode_cmd25', person=(2.0, 0.0), usage_mode=1,
         settle=2.5, arm=True,
         checks=[('command_kind 25', lambda o: o['status'].get('command_kind') == 25),
                 ('usage_mode 1', lambda o: o['status'].get('usage_mode') == 1)]),
]


def ensure_isolated_environment():
    """Force an isolated ROS domain before ``rclpy.init()``.

    An explicitly requested robot domain (``ROS_DOMAIN_ID=0``) or a non-localhost
    setting is refused.  When nothing is configured, the simulation selects the
    isolated domain itself so CI and a fresh development shell work unchanged.
    """
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    local_only = os.environ.get('ROS_LOCALHOST_ONLY', '')
    if domain == '0':
        raise SystemExit('refusing to run in ROS_DOMAIN_ID=0 (robot domain)')
    if local_only not in ('', '1'):
        raise SystemExit('refusing to run with ROS_LOCALHOST_ONLY=%r' % local_only)
    os.environ['ROS_DOMAIN_ID'] = domain or '83'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'


def run(args):
    ensure_isolated_environment()
    launch = None
    if not args.no_launch:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'jie_deamon', 'm20.launch.py',
             'cloud_topic:=/m20/mock_points', 'dry_run:=true', 'enable_web:=false'],
            env=dict(os.environ), start_new_session=True)
    rclpy.init()
    sim = PersonSim()
    executor = SingleThreadedExecutor()
    executor.add_node(sim)
    target_pub = sim.create_publisher(Point, '/robot_nexus/target', 1)
    results = []

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if launch is not None and launch.poll() is not None:
                raise AssertionError('launch exited unexpectedly')
            executor.spin_once(timeout_sec=0.05)

    def until(predicate, timeout=15.0, what=''):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if launch is not None and launch.poll() is not None:
                raise AssertionError('launch exited unexpectedly')
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        raise AssertionError('timeout waiting for %s (%r)' % (what, sim.observed))

    def call(name, enabled):
        client = sim.create_client(SetBool, name)
        until(client.service_is_ready, 20.0, name)
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        until(future.done, 10.0, name + ' response')
        return future.result()

    def arm(timeout=12.0):
        """Arm with retries; the gate refuses until the velocity stream is live."""
        end = time.monotonic() + timeout
        message = ''
        while time.monotonic() < end:
            result = call('/m20/arm', True)
            message = result.message
            if result.success:
                return True
            spin(0.3)
        raise AssertionError('arm never accepted: ' + message)

    try:
        until(lambda: sim.observed['scan_at'] > 0 and bool(sim.observed['status']),
              20.0, 'scan and bridge status')
        sim.configure(person=(2.0, 0.0))
        spin(1.0)
        target_pub.publish(Point(x=2.0, y=0.0, z=0.0))
        response = call('/robot_nexus/set_moving', True)
        assert response.success, response.message
        spin(0.5)
        arm()

        for scenario in SCENARIOS:
            sim.configure(person=scenario.get('person'),
                          obstacles=scenario.get('obstacles', ()),
                          self_returns=scenario.get('self_returns', False),
                          usage_mode=scenario.get('usage_mode', 1),
                          gait=scenario.get('gait', 12290))
            if scenario.get('person') is not None:
                # Wait for the synthetic person to finish walking, then have the
                # operator re-select the target at its current position (the
                # algorithm only searches within TARGET_RADIUS of the target).
                until(sim.person_settled, 25.0, 'person walking to %r' % (scenario['person'],))
                target_pub.publish(Point(x=sim.person[0], y=sim.person[1], z=0.0))
                spin(0.5)
            settle = float(scenario.get('settle', 2.0))
            spin(settle)
            if scenario.get('arm'):
                arm()
                spin(0.8)
            failures = []
            for label, check in scenario['checks']:
                try:
                    ok = bool(check(sim.observed))
                except Exception as exc:  # noqa: BLE001
                    ok = False
                    label = '%s (raised %s)' % (label, exc)
                if not ok:
                    failures.append(label)
            results.append((scenario['name'], failures, tuple(sim.observed['raw']),
                            tuple(sim.observed['guarded']),
                            sim.observed['status'].get('reason'),
                            sim.observed['status'].get('command_kind')))
            mark = 'PASS' if not failures else 'FAIL'
            print('%-4s %-38s raw=%s guarded=%s cmd=%s reason=%s'
                  % (mark, scenario['name'], tuple(round(v, 3) for v in sim.observed['raw']),
                     tuple(round(v, 3) for v in sim.observed['guarded']),
                     sim.observed['status'].get('command_kind'),
                     sim.observed['status'].get('reason')))
            for label in failures:
                print('       failed: %s' % label)
    finally:
        if launch is not None and launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
        executor.shutdown()
        sim.destroy_node()
        rclpy.shutdown()

    failed = [r for r in results if r[1]]
    print('\n%d/%d scenarios passed' % (len(results) - len(failed), len(results)))
    if args.report:
        with open(args.report, 'w', encoding='utf-8') as handle:
            json.dump([dict(name=n, failures=f, raw=list(raw), guarded=list(guarded),
                            reason=reason, command_kind=kind)
                       for n, f, raw, guarded, reason, kind in results],
                      handle, ensure_ascii=False, indent=2)
        print('report: ' + args.report)
    return 1 if failed else 0


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-launch', action='store_true',
                        help='assume m20.launch.py is already running')
    parser.add_argument('--report', default='')
    args = parser.parse_args()
    raise SystemExit(run(args))


if __name__ == '__main__':
    main()
