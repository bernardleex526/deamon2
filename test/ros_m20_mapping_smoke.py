"""Verify the passive mapping launch and effective scan parameters offline."""
import os
import math
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '83'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from m20_adapter.mock_inputs import MockInputs


def main():
    rclpy.init()
    mock, observer = MockInputs(), Node('m20_mapping_smoke_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(mock)
    executor.add_node(observer)
    scans = []
    observer.create_subscription(LaserScan, '/m20/mapping_scan', scans.append,
                                 qos_profile_sensor_data)
    process = subprocess.Popen(
        ['ros2', 'launch', 'jie_deamon', 'm20_mapping.launch.py',
         'cloud_topic:=/m20/mock_points'], start_new_session=True)

    def until(predicate):
        end = time.monotonic() + 15
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError('passive launch exited')
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('passive mapping condition timed out')

    try:
        until(lambda: bool(scans))
        assert any(math.isfinite(r) and 1.5 < r < 2.0 for r in scans[-1].ranges)
        client = observer.create_client(GetParameters, '/m20_cloud_to_scan/get_parameters')
        until(client.service_is_ready)
        request = GetParameters.Request()
        request.names = ['min_height', 'max_height', 'range_max']
        future = client.call_async(request)
        until(future.done)
        values = [p.double_value for p in future.result().values]
        assert values == [-0.1, 0.5, 12.0], values
        names = observer.get_node_names()
        assert 'robot_nexus' not in names and 'm20_bridge' not in names, names
        assert observer.count_publishers('/m20/cmd_vel_raw') == 0
        assert observer.count_publishers('/cmd_vel') == 0
        print('PASS: passive scan received, YAML applied, no controller or velocity publisher')
    finally:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10)
        finally:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
        executor.shutdown()
        mock.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
