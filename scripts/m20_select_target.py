#!/usr/bin/env python3
"""Select a target once; never enable following, arm, change mode, or send velocity."""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('--frame', default='lidar_link')
    parser.add_argument('--timeout', type=float, default=5.)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.timeout)) or not 0 < args.timeout <= 30:
        parser.error('Coordinates must be finite; timeout must be in (0, 30] seconds')
    import rclpy
    from std_msgs.msg import String
    from rcl_interfaces.srv import SetParametersAtomically
    from m20_adapter.selection import make_selection_request

    rclpy.init()
    node = rclpy.create_node('m20_manual_target_selection')
    context = {}

    def tracking(msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict) and type(value.get('fault_epoch')) is int:
                context.clear()
                context.update(value)
        except ValueError:
            pass

    node.create_subscription(String, '/robot_nexus/tracking_status', tracking, 1)
    client = node.create_client(SetParametersAtomically, '/robot_nexus/select_target')
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            stamp = context.get('stamp')
            if (client.service_is_ready() and type(stamp) in (int, float)
                    and -.1 <= node.get_clock().now().nanoseconds/1e9-stamp <= .5):
                break
        else:
            raise RuntimeError('No fresh selection context/service; check the M20 profile and scans')
        request = make_selection_request(args.x, args.y, args.frame,
                                         node.get_clock().now().to_msg(), context['fault_epoch'])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('Selection response timed out; inspect status, do not auto-retry')
        result = future.result().result
        print(json.dumps(dict(accepted=result.successful, reason=result.reason,
                              enable_or_arm_requested=False)))
        return 0 if result.successful else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
