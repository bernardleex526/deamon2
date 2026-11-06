#!/usr/bin/env python3
"""Bounded, read-only GOS preflight for the M20 follow stack.

Every check is read-only: the only packet this tool sends is the documented
registration heartbeat (Type=100 Cmd=100).  It never sends velocity, never
changes motion state, usage mode or gait, and never starts or stops a service.

The report it writes is consumed by ``m20_bridge`` when
``require_preflight:=true``, so the live control socket cannot be opened unless
these checks passed recently on this host.

Exit code 0 = every required check passed, 1 = at least one required check failed.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from m20_adapter.core import GAIT_TABLE, MODE_NAMES, UdpClient  # noqa: E402
from m20_adapter.preflight import build_report, write_report  # noqa: E402

REQUIRED_STATUS = ('MotionState', 'Gait', 'Charge', 'HES', 'ControlUsageMode',
                   'Sleep', 'Direction')


def check(name, ok, detail, severity='required', fix=None):
    entry = dict(name=name, ok=bool(ok), detail=detail, severity=severity)
    if fix:
        entry['fix'] = fix
    return entry


def run(cmd, timeout=10):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return out.returncode, out.stdout.strip(), out.stderr.strip()
    except (OSError, subprocess.SubprocessError) as exc:
        return 127, '', str(exc)


def systemctl_state(unit):
    code_a, active, _ = run(['systemctl', 'is-active', unit])
    code_e, enabled, _ = run(['systemctl', 'is-enabled', unit])
    return active or 'unknown', enabled or 'unknown'


def check_root():
    uid = os.geteuid() if hasattr(os, 'geteuid') else -1
    return check('root', uid == 0,
                 'euid=%d' % uid,
                 fix='vendor requires su/root to receive /LIDAR/POINTS: run under sudo -i')


def check_relay():
    active, enabled = systemctl_state('multicast-relay.service')
    ok = active == 'active' and enabled == 'enabled'
    return check('multicast_relay', ok, 'active=%s enabled=%s' % (active, enabled),
                 fix='sudo systemctl enable --now multicast-relay.service '
                     '(factory default is disabled; without it there is no cloud after reboot)')


def check_planner():
    active, _ = systemctl_state('planner.service')
    return check('planner_idle', active != 'active',
                 'planner.service=%s' % active,
                 fix='stop the factory planner before handing control to this bridge: '
                     'sudo systemctl stop planner.service')


def check_time_sync():
    code, out, _ = run(['timedatectl', 'show', '-p', 'NTPSynchronized'])
    synced = out.endswith('=yes')
    detail = out or 'timedatectl unavailable'
    ptp = shutil.which('pmc')
    if ptp:
        code2, out2, _ = run([ptp, '-u', '-i', 'eth0', 'GET TIME_STATUS_NP'], timeout=5)
        if code2 == 0:
            detail += '; pmc reachable'
    return check('time_sync', synced, detail, severity='info',
                 fix='scan freshness depends on PTP/NTP: check rsdriver ptp=0x1 and NOS master')


def check_ros(args):
    """Subscribe-only sensor and graph checks. Returns (checks, extra)."""
    checks, extra = [], {}
    try:
        import rclpy  # noqa: F401
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
    except Exception as exc:  # pragma: no cover - depends on the host image
        checks.append(check('ros_available', False, 'rclpy import failed: %s' % exc,
                            fix='source /opt/robot/scripts/setup_ros2.sh'))
        return checks, extra
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2

    rclpy.init()
    node = rclpy.create_node('m20_preflight')
    samples = {'cloud': [], 'scan': []}
    seen = {}
    try:
        def receive(kind, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp
            samples[kind].append((time.monotonic(), age))
            info = seen.setdefault(kind, dict(frame=msg.header.frame_id))
            if kind == 'cloud':
                info.update(points=msg.width * msg.height,
                            fields=sorted(f.name for f in msg.fields),
                            data_bytes=len(msg.data))
            else:
                info.update(bins=len(msg.ranges))

        node.create_subscription(PointCloud2, args.cloud_topic,
                                 lambda m: receive('cloud', m), qos_profile_sensor_data)
        node.create_subscription(LaserScan, args.scan_topic,
                                 lambda m: receive('scan', m), qos_profile_sensor_data)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        cloud = seen.get('cloud', {})
        cloud_samples = samples['cloud']
        hz = 0.0
        if len(cloud_samples) > 1:
            span = cloud_samples[-1][0] - cloud_samples[0][0]
            hz = (len(cloud_samples) - 1) / span if span else 0.0
        fresh = bool(cloud_samples) and -0.1 <= cloud_samples[-1][1] <= 0.5
        ok = (len(cloud_samples) >= 3 and fresh and cloud.get('points', 0) > 0
              and {'x', 'y', 'z'}.issubset(set(cloud.get('fields', []))))
        checks.append(check(
            'lidar_cloud', ok,
            '%s: samples=%d hz=%.2f frame=%s points=%s age=%.3fs'
            % (args.cloud_topic, len(cloud_samples), hz, cloud.get('frame'),
               cloud.get('points'), cloud_samples[-1][1] if cloud_samples else float('nan')),
            fix='check rsdriver.service and multicast-relay.service; '
                '/LIDAR/POINTS requires root and the relay'))
        extra['cloud'] = dict(samples=len(cloud_samples), hz=round(hz, 3),
                              frame=cloud.get('frame'), points=cloud.get('points'))

        scan = seen.get('scan', {})
        scan_samples = samples['scan']
        scan_hz = 0.0
        if len(scan_samples) > 1:
            span = scan_samples[-1][0] - scan_samples[0][0]
            scan_hz = (len(scan_samples) - 1) / span if span else 0.0
        checks.append(check(
            'scan_topic', bool(scan_samples),
            '%s: samples=%d hz=%.2f frame=%s bins=%s'
            % (args.scan_topic, len(scan_samples), scan_hz, scan.get('frame'),
               scan.get('bins')),
            severity='required' if args.require_scan else 'info',
            fix='start pointcloud_to_laserscan (m20.launch.py) before going live'))
        extra['scan'] = dict(samples=len(scan_samples), hz=round(scan_hz, 3),
                             frame=scan.get('frame'), bins=scan.get('bins'))

        nav_pubs = node.get_publishers_info_by_topic('/NAV_CMD')
        checks.append(check(
            'no_nav_cmd_publisher', len(nav_pubs) == 0,
            '/NAV_CMD publishers=%d' % len(nav_pubs),
            fix='another velocity owner is publishing /NAV_CMD; stop it first'))
        extra['nav_cmd_publishers'] = len(nav_pubs)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return checks, extra


def check_aos(args):
    """Heartbeat-only AOS telemetry check. Sends no velocity."""
    checks, extra = [], {}
    if args.no_aos:
        checks.append(check('aos_link', True, 'skipped by --no-aos', severity='info'))
        return checks, extra
    client = None
    statuses = []
    try:
        client = UdpClient(args.aos_host, args.aos_port)
        end = time.monotonic() + args.seconds
        last = 0.0
        while time.monotonic() < end:
            if time.monotonic() - last >= 1.0:
                client.heartbeat()
                last = time.monotonic()
            for _, value in client.receive():
                items = value['Items']
                if value['Type'] == 1002 and value['Command'] == 6 and \
                        ('BasicStatus' in items or 'MotionState' in items):
                    statuses.append(items.get('BasicStatus', items))
            time.sleep(0.05)
    except OSError as exc:
        checks.append(check('aos_link', False, 'UDP error: %s' % exc,
                            fix='check the AOS address/route; AOS is 10.21.31.103:30000'))
        return checks, extra
    finally:
        if client:
            client.close()

    checks.append(check('aos_link', len(statuses) >= 2,
                        'BasicStatus received=%d in %.1fs' % (len(statuses), args.seconds),
                        fix='AOS basic_server did not stream status after heartbeat'))
    if statuses:
        last = statuses[-1]
        missing = [k for k in REQUIRED_STATUS if type(last.get(k)) is not int]
        checks.append(check('aos_basic_status', not missing,
                            'fields=%s missing=%s' % (sorted(last), missing),
                            fix='firmware too old or unexpected layout; target >= V1.1.7'))
        mode = last.get('ControlUsageMode')
        gait = last.get('Gait')
        checks.append(check(
            'aos_motion_state', last.get('MotionState') == 17,
            'MotionState=%s (17=RL control required)' % last.get('MotionState'),
            fix='stand the robot up with the factory controller until MotionState=17'))
        checks.append(check(
            'aos_gait_known', gait in GAIT_TABLE,
            'Gait=%s (%s)' % (gait, GAIT_TABLE.get(gait, ('unknown',))[0]),
            fix='select one of 4097/4099/12290/12291'))
        if gait in GAIT_TABLE:
            row = GAIT_TABLE[gait]
            notes = []
            for index, axis in enumerate(('x', 'y', 'yaw')):
                limit = (args.max_vx, args.max_vy, args.max_wz)[index]
                if row[1 + index] > min(limit, row[4 + index]):
                    notes.append('%s min %.2f > limit %.2f' % (axis, row[1 + index], limit))
            checks.append(check('gait_speed_window', True,
                                'gait=%s limits x/y/yaw=%.2f/%.2f/%.2f %s'
                                % (row[0], args.max_vx, args.max_vy, args.max_wz,
                                   ('unusable axes: ' + ', '.join(notes)) if notes else 'ok'),
                                severity='info'))
        extra['basic_status'] = last
        extra['usage_mode_name'] = MODE_NAMES.get(mode, 'unknown')
    return checks, extra


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--output', default='/tmp/m20_preflight.json')
    parser.add_argument('--cloud-topic', default='/LIDAR/POINTS')
    parser.add_argument('--scan-topic', default='/m20/scan')
    parser.add_argument('--require-scan', action='store_true')
    parser.add_argument('--aos-host', default='10.21.31.103')
    parser.add_argument('--aos-port', type=int, default=30000)
    parser.add_argument('--no-aos', action='store_true')
    parser.add_argument('--skip-ros', action='store_true')
    parser.add_argument('--max-vx', type=float, default=0.3)
    parser.add_argument('--max-vy', type=float, default=0.3)
    parser.add_argument('--max-wz', type=float, default=0.6)
    args = parser.parse_args()

    checks = [check_root(), check_relay(), check_planner(), check_time_sync()]
    extra = {}
    if args.skip_ros:
        checks.append(check('ros_available', True, 'skipped by --skip-ros', severity='info'))
    else:
        ros_checks, ros_extra = check_ros(args)
        checks.extend(ros_checks)
        extra.update(ros_extra)
    aos_checks, aos_extra = check_aos(args)
    checks.extend(aos_checks)
    extra.update(aos_extra)

    report = build_report(checks, extra=extra)
    write_report(args.output, report)
    required = [c for c in checks if c.get('severity', 'required') == 'required']
    failed = [c for c in required if not c['ok']]
    print(json.dumps({'output': args.output, 'ok': report['ok'],
                      'required_passed': '%d/%d' % (len(required) - len(failed), len(required)),
                      'failed': [c['name'] for c in failed]}, indent=2))
    for entry in checks:
        mark = 'PASS' if entry['ok'] else ('FAIL' if entry.get('severity') == 'required' else 'WARN')
        print('%-5s %-22s %s' % (mark, entry['name'], entry['detail']))
        if not entry['ok'] and entry.get('fix'):
            print('      fix: %s' % entry['fix'])
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
