"""Actual ROS graph smoke test. Requires a built/sourced workspace; no robot."""
import json
import os
import signal
import subprocess
import time

# Keep synthetic data off the robot domain and physical network.
os.environ['ROS_DOMAIN_ID'] = '83'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from m20_adapter.mock_inputs import MockInputs


def main():
    process = subprocess.Popen(
        ['ros2', 'launch', 'jie_deamon', 'm20.launch.py',
         'cloud_topic:=/m20/mock_points', 'dry_run:=true', 'enable_web:=false'],
        start_new_session=True)
    rclpy.init()
    mock, observer = MockInputs(), Node('m20_smoke_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(mock)
    executor.add_node(observer)
    observed = dict(raw=0., guarded=0., status={}, scan_at=-1.)
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    observer.create_subscription(Twist, '/m20/cmd_vel_raw',
                                 lambda m: observed.update(raw=m.linear.x), 1)
    observer.create_subscription(Twist, '/m20/cmd_vel_guarded',
                                 lambda m: observed.update(guarded=m.linear.x), 1)
    observer.create_subscription(String, '/m20/bridge_status',
                                 lambda m: observed.update(status=json.loads(m.data)), 1)
    observer.create_subscription(LaserScan, '/m20/scan',
                                 lambda m: observed.update(scan_at=time.monotonic()),
                                 qos_profile_sensor_data)
    target = observer.create_publisher(Point, '/robot_nexus/target', 1)

    def until(predicate, timeout=15.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError('launch exited unexpectedly')
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('ROS condition timed out: ' + repr(observed))

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError('launch exited unexpectedly')
            executor.spin_once(timeout_sec=0.05)

    def call(name, enabled):
        client = observer.create_client(SetBool, name)
        until(client.service_is_ready)
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        until(future.done)
        assert future.result().success, future.result().message

    def arm(timeout=12.):
        """Arm with retries.

        The gate deliberately refuses to arm until the velocity stream is live
        (reason ``no fresh command stream``) and the scan is clear, so an operator
        retries instead of being granted a grace window that expires at once.
        """
        end = time.monotonic() + timeout
        message = ''
        while time.monotonic() < end:
            client = observer.create_client(SetBool, '/m20/arm')
            until(client.service_is_ready)
            request = SetBool.Request()
            request.data = True
            future = client.call_async(request)
            until(future.done)
            message = future.result().message
            if future.result().success:
                return
            spin(0.3)
        raise AssertionError('arm never accepted: ' + message)

    try:
        until(lambda: observed['scan_at'] > 0 and bool(observed['status']))
        assert observed['guarded'] == 0. and not observed['status']['armed']
        until(lambda: target.get_subscription_count() > 0)
        target.publish(Point(x=1.8, y=0., z=0.))
        call('/robot_nexus/set_moving', True)
        until(lambda: observed['raw'] > 0.2)
        arm()
        until(lambda: observed['guarded'] > 0.2)
        mock.set_parameters([Parameter('target', value=False)])
        until(lambda: observed['raw'] == 0. and observed['guarded'] == 0.)
        mock.set_parameters([Parameter('target', value=True), Parameter('obstacle', value=True)])
        until(lambda: observed['status'].get('reason') == 'invalid scan or obstacle')
        assert not observed['status']['armed'] and observed['guarded'] == 0.
        mock.set_parameters([Parameter('obstacle', value=False)])
        fresh_after = time.monotonic() + 0.2
        until(lambda: observed['scan_at'] > fresh_after)
        assert observed['guarded'] == 0. and not observed['status']['armed']
        arm()
        until(lambda: observed['guarded'] > 0.2)
        executor.remove_node(mock)
        until(lambda: not observed['status'].get('armed') and observed['guarded'] == 0.)
        print('PASS: default zero, forward tracking, target loss, obstacle latch, input loss')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        executor.shutdown()
        mock.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
