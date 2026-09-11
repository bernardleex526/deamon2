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
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import String
from std_srvs.srv import SetBool
from m20_adapter.mock_inputs import MockInputs
from m20_adapter.selection import make_selection_request, make_enable_request


def main():
    process = subprocess.Popen(
        ['ros2', 'launch', 'jie_deamon', 'm20.launch.py',
         'cloud_topic:=/m20/mock_points', 'dry_run:=true'], start_new_session=True)
    rclpy.init()
    mock, observer = MockInputs(), Node('m20_smoke_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(mock)
    executor.add_node(observer)
    observed = dict(raw=0., guarded=0., status={}, tracking={}, scan_at=-1.)
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    observer.create_subscription(Twist, '/m20/cmd_vel_raw',
                                 lambda m: observed.update(raw=m.linear.x), 1)
    observer.create_subscription(Twist, '/m20/cmd_vel_guarded',
                                 lambda m: observed.update(guarded=m.linear.x), 1)
    observer.create_subscription(String, '/m20/bridge_status',
                                 lambda m: observed.update(status=json.loads(m.data)), 1)
    observer.create_subscription(String, '/robot_nexus/tracking_status',
                                 lambda m: observed.update(tracking=json.loads(m.data)), 1)
    observer.create_subscription(LaserScan, '/m20/scan',
                                 lambda m: observed.update(scan_at=time.monotonic()),
                                 qos_profile_sensor_data)

    def until(predicate, timeout=15.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError('launch exited unexpectedly')
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('ROS condition timed out: ' + repr(observed))

    def call(name, enabled):
        if name == '/robot_nexus/set_moving' and enabled:
            client = observer.create_client(SetParametersAtomically, '/robot_nexus/enable_follow')
            try:
                until(client.service_is_ready)
                request = make_enable_request(observer.get_clock().now().to_msg(),
                    observed['tracking']['fault_epoch'], observed['tracking']['selection_id'])
                future = client.call_async(request)
                until(future.done)
                assert future.result().result.successful, future.result().result.reason
            finally:
                observer.destroy_client(client)
            return
        client = observer.create_client(SetBool, name)
        try:
            until(client.service_is_ready)
            request = SetBool.Request()
            request.data = enabled
            future = client.call_async(request)
            until(future.done)
            assert future.result().success, future.result().message
        finally:
            observer.destroy_client(client)

    def select():
        client = observer.create_client(SetParametersAtomically, '/robot_nexus/select_target')
        try:
            until(lambda: client.service_is_ready() and observed['tracking'].get('fault_epoch', 0) > 0)
            previous = observed['tracking'].get('selection_id', 0)
            request = make_selection_request(1.8, 0., 'lidar_link', observer.get_clock().now().to_msg(),
                                             observed['tracking']['fault_epoch'])
            future = client.call_async(request)
            until(future.done)
            assert future.result().result.successful, future.result().result.reason
            until(lambda: observed['tracking'].get('selection_id', 0) > previous)
        finally:
            observer.destroy_client(client)

    try:
        until(lambda: observed['scan_at'] > 0 and bool(observed['status']))
        assert observed['guarded'] == 0. and not observed['status']['armed']
        select()
        until(lambda: observed['tracking'].get('tracking_valid', False))
        call('/robot_nexus/set_moving', True)
        until(lambda: observed['raw'] > 0.2)
        call('/m20/arm', True)
        until(lambda: observed['guarded'] > 0.2)
        mock.set_parameters([Parameter('target', value=False)])
        until(lambda: observed['raw'] == 0. and observed['guarded'] == 0.
              and not observed['status'].get('armed', True)
              and not observed['tracking'].get('tracking_valid', True))
        # Returning points must not automatically reacquire the selected identity.
        mock.set_parameters([Parameter('target', value=True)])
        until_time = time.monotonic() + .5
        until(lambda: time.monotonic() >= until_time)
        assert not observed['tracking']['tracking_valid'] and observed['raw'] == 0.
        select()
        until(lambda: observed['tracking'].get('tracking_valid', False))
        call('/robot_nexus/set_moving', True)
        call('/m20/arm', True)
        until(lambda: observed['guarded'] > .2)
        mock.set_parameters([Parameter('target', value=True), Parameter('obstacle', value=True)])
        until(lambda: observed['status'].get('reason') == 'invalid scan or obstacle')
        assert not observed['status']['armed'] and observed['guarded'] == 0.
        mock.set_parameters([Parameter('obstacle', value=False)])
        fresh_after = time.monotonic() + 0.2
        until(lambda: observed['scan_at'] > fresh_after)
        assert observed['guarded'] == 0. and not observed['status']['armed']
        call('/m20/arm', True)
        until(lambda: observed['guarded'] > 0.2)
        mock.set_parameters([Parameter('stamp_delay', value=.6)])
        until(lambda: not observed['tracking'].get('tracking_valid', True)
              and not observed['status'].get('armed', True) and observed['raw'] == 0.)
        mock.set_parameters([Parameter('stamp_delay', value=0.)])
        settle = time.monotonic() + .6
        until(lambda: time.monotonic() > settle)
        assert not observed['tracking']['tracking_valid']
        select()
        until(lambda: observed['tracking'].get('tracking_valid', False))
        call('/robot_nexus/set_moving', True)
        call('/m20/arm', True)
        until(lambda: observed['guarded'] > .2)
        executor.remove_node(mock)
        until(lambda: not observed['status'].get('armed') and observed['guarded'] == 0.)
        print('PASS: default zero, forward tracking, latched target loss, explicit reselection, obstacle latch, delayed scans, input loss')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
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
