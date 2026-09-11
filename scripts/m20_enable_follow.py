#!/usr/bin/env python3
"""Enable calculation once for the current selection; never arm or send UDP."""
import json
import time


def main():
    import rclpy
    from std_msgs.msg import String
    from rcl_interfaces.srv import SetParametersAtomically
    from m20_adapter.selection import make_enable_request
    rclpy.init()
    node = rclpy.create_node('m20_manual_follow_enable')
    context = {}

    def tracking(msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                context.clear()
                context.update(value)
        except ValueError:
            pass

    node.create_subscription(String, '/robot_nexus/tracking_status', tracking, 1)
    client = node.create_client(SetParametersAtomically, '/robot_nexus/enable_follow')
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            stamp = context.get('stamp')
            if (client.service_is_ready() and context.get('tracking_valid') is True
                    and type(stamp) in (int, float)
                    and -.1 <= node.get_clock().now().nanoseconds/1e9-stamp <= .5):
                break
        else:
            raise RuntimeError('No fresh valid target; select and verify it first')
        request = make_enable_request(node.get_clock().now().to_msg(),
                                      context['fault_epoch'], context['selection_id'])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Enable response timed out; inspect state, do not auto-retry')
        result = future.result().result
        print(json.dumps(dict(accepted=result.successful, reason=result.reason,
                              arm_requested=False)))
        return 0 if result.successful else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
