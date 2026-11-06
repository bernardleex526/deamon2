#!/usr/bin/env python3
"""Bounded read-only sensor probe; never publishes or opens a control socket."""
import argparse
import json
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--scan-topic', default='/m20/mapping_scan')
    parser.add_argument('--cloud-only', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('m20_readonly_sensor_probe')
    samples = {'cloud': [], 'scan': []}
    details = {}

    def receive(kind, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = node.get_clock().now().nanoseconds / 1e9 - stamp
        samples[kind].append((time.monotonic(), age))
        details[kind] = dict(frame=msg.header.frame_id, stamp=stamp)
        if kind == 'cloud':
            details[kind].update(points=msg.width * msg.height,
                                 fields=[f.name for f in msg.fields],
                                 data_bytes=len(msg.data))
        else:
            details[kind].update(bins=len(msg.ranges),
                                 valid=sum(math.isfinite(x) and
                                           msg.range_min <= x <= msg.range_max
                                           for x in msg.ranges))

    node.create_subscription(PointCloud2, '/LIDAR/POINTS',
                             lambda msg: receive('cloud', msg), qos_profile_sensor_data)
    node.create_subscription(LaserScan, args.scan_topic,
                             lambda msg: receive('scan', msg), qos_profile_sensor_data)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for kind, data in samples.items():
            details.setdefault(kind, {})
            details[kind].update(count=len(data))
            if data:
                elapsed = data[-1][0] - data[0][0]
                details[kind].update(hz=(len(data) - 1) / elapsed if elapsed else 0,
                                     min_age=min(x[1] for x in data),
                                     max_age=max(x[1] for x in data))
        details['publishers'] = [dict(node=e.node_name, namespace=e.node_namespace,
                                      qos=str(e.qos_profile))
                                 for e in node.get_publishers_info_by_topic('/LIDAR/POINTS')]
        def fresh(kind):
            data = samples[kind]
            return bool(data) and (-0.1 <= data[-1][1] <= 0.5 and
                                   time.monotonic() - data[-1][0] <= 0.5)

        cloud = details['cloud']
        ready = (fresh('cloud') and cloud['points'] > 0 and cloud['data_bytes'] > 0
                 and {'x', 'y', 'z'}.issubset(cloud['fields']))
        if not args.cloud_only:
            ready = ready and fresh('scan') and details['scan']['valid'] > 0
        details['usable_fresh_samples'] = ready
        print(json.dumps(details, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
