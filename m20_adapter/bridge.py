"""ROS input to documented basic_server Cmd=25. Dry-run never opens a socket."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from m20_adapter.core import Guard, UdpClient, inspect_scan, velocity_items


class Bridge(Node):
    def __init__(self):
        super().__init__('m20_bridge')
        defaults = dict(dry_run=True, commissioned=False, aos_host='10.21.31.103',
                        aos_port=30000, max_vx=0.3, max_vy=0.3, max_wz=0.6,
                        scan_frame='lidar_link', stop_front=0.7, stop_back=0.7,
                        stop_half_width=0.4)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.dry_run = p('dry_run')
        self.commissioned = p('commissioned')
        if not self.dry_run and not self.commissioned:
            raise RuntimeError('live transport requires commissioned:=true')
        if self.get_parameter('use_sim_time').value and not self.dry_run:
            raise RuntimeError('live transport cannot use simulation time')
        self.guard = Guard(limits=(p('max_vx'), p('max_vy'), p('max_wz')))
        self.scan_frame = p('scan_frame')
        self.stop = dict(stop_front=p('stop_front'), stop_back=p('stop_back'),
                         stop_half_width=p('stop_half_width'))
        import math
        if any(not math.isfinite(v) or v <= 0 for v in self.stop.values()):
            raise ValueError('stop dimensions must be positive and finite')
        self.client = None if self.dry_run else UdpClient(p('aos_host'), p('aos_port'))
        self.last_query = -float('inf')
        self.preview = self.create_publisher(Twist, '/m20/cmd_vel_guarded', 1)
        self.status_pub = self.create_publisher(String, '/m20/bridge_status', 1)
        self.create_subscription(Twist, '/m20/cmd_vel_raw', self.command, 1)
        self.create_subscription(LaserScan, '/m20/scan', self.scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/m20/arm', self.arm)
        if self.dry_run:
            self.create_subscription(String, '/m20/mock_basic_status', self.mock_status, 1)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def command(self, msg):
        self.guard.update_command((msg.linear.x, msg.linear.y, msg.angular.z))

    def scan(self, msg):
        valid, clear = inspect_scan(msg, self.get_clock().now().nanoseconds / 1e9, **self.stop)
        self.guard.update_scan(valid and msg.header.frame_id == self.scan_frame, clear)

    def arm(self, request, response):
        if request.data:
            response.success = self.guard.arm()
        else:
            self.guard.disarm('operator disarmed')
            response.success = True
        response.message = self.guard.reason
        return response

    def mock_status(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('status must be an object')
            self.guard.update_status(value)
        except ValueError:
            self.guard.disarm('invalid mock status')

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
            velocity = self.guard.output()
            if self.client:
                self.client.send(2, 25, velocity_items(velocity))
        except OSError as exc:
            self.guard.disarm('UDP failure: ' + str(exc))
            velocity = (0.0, 0.0, 0.0)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity
        self.preview.publish(msg)
        status = String()
        status.data = json.dumps(dict(dry_run=self.dry_run, armed=self.guard.armed,
                                      reason=self.guard.reason, velocity=velocity))
        self.status_pub.publish(status)

    def close(self):
        if self.client:
            try:
                self.client.send(2, 25, velocity_items((0.0, 0.0, 0.0)))
            except OSError:
                pass
            self.client.close()


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
