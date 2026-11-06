"""Synthetic point cloud and BasicStatus, never connected to robot sockets."""
import json
import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class MockInputs(Node):
    def __init__(self):
        super().__init__('m20_mock_inputs')
        self.declare_parameter('obstacle', False)
        self.declare_parameter('target', True)
        self.declare_parameter('self_returns', False)
        self.declare_parameter('usage_mode', 1)
        self.declare_parameter('gait', 12290)
        self.cloud = self.create_publisher(PointCloud2, '/m20/mock_points', qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/m20/mock_basic_status', 1)
        self.create_timer(0.1, self.tick)

    def tick(self):
        points = [(5.0 * math.cos(i * math.pi / 180),
                   5.0 * math.sin(i * math.pi / 180), 0.2) for i in range(360)]
        if self.get_parameter('target').value:
            points.extend((1.8, i * 0.01, 0.2) for i in range(-10, 11))
        if self.get_parameter('obstacle').value:
            points.append((0.6, 0.0, 0.2))
        if self.get_parameter('self_returns').value:
            # Persistent leg returns observed on the real robot, one inside the
            # front footprint and one rear-left.
            points.extend([(0.40, 0.20, 0.1), (-0.37, 0.26, 0.1)])
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
        status = String()
        status.data = json.dumps(dict(BasicStatus=dict(
            MotionState=17, Gait=int(self.get_parameter('gait').value),
            Charge=0, HES=0, Sleep=0, Direction=0,
            ControlUsageMode=int(self.get_parameter('usage_mode').value))))
        self.status.publish(status)


def main():
    rclpy.init()
    node = MockInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
