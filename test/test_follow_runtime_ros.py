"""Isolated ROS integration. Hard refusal outside domain 47, approved UDP only.

No robot telemetry, HTTP, mode commands, production services or domain-0 output.
The fixed /NAV_CMD sender is exercised only inside DDS domain 47, with a test
subscriber. This proves software delivery, NOT robot response or physical stop.
Run separately: python3 -B -m unittest test_follow_runtime_ros -v
"""
import json
import os
import subprocess
import sys
import time
import unittest

from motion_control.runtime import check_process_env


class TestIsolatedROS(unittest.TestCase):
    def setUp(self):
        if os.environ.get('ROS_DOMAIN_ID') != '47':
            self.fail('REFUSE: integration requires ROS_DOMAIN_ID=47; no DDS initialized')
        self.assertIsNone(check_process_env(False))
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import LaserScan
        from drdds.msg import NavCmd
        self.rclpy, self.Twist, self.LaserScan = rclpy, Twist, LaserScan
        rclpy.init(args=[])
        self.nodes = []
        self.addCleanup(self.cleanup)
        self.executor = SingleThreadedExecutor()
        self.node = Node('follow_test_inputs', use_global_arguments=False,
                         start_parameter_services=False, enable_rosout=False)
        self.nodes.append(self.node)
        self.executor.add_node(self.node)
        self.raw_pub = self.node.create_publisher(Twist, '/new_chase/cmd_vel_raw', 1)
        self.scan_pub = self.node.create_publisher(LaserScan, '/new_chase/scan', qos_profile_sensor_data)
        self.outputs = []
        self.node.create_subscription(NavCmd, '/NAV_CMD',
                                     lambda m: self.outputs.append((m.data.x_vel, m.data.y_vel, m.data.yaw_vel)), 10)
        self.controller = self.sender = None
        self.skip_input = None
        self.near = False
        self.last_publish = 0.0

    def cleanup(self):
        if self.controller is not None:
            self.controller.shutdown_output()
        if self.sender is not None:
            self.sender.close()
        for node in reversed(self.nodes):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        self.rclpy.shutdown()

    def messages(self):
        raw = self.Twist()
        raw.linear.x = 0.25
        msg = self.LaserScan()
        msg.header.frame_id = 'lidar_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.angle_min, msg.angle_max, msg.angle_increment = 0.0, 0.0, 0.01
        msg.range_min, msg.range_max = 0.1, 20.0
        msg.ranges = [0.7 if self.near else 2.0]
        return raw, msg

    def cycle(self):
        from motion_control.runtime import _spin_batch
        now = time.monotonic()
        if now - self.last_publish >= 0.05:
            raw, scan = self.messages()
            if self.skip_input != 'raw':
                self.raw_pub.publish(raw)
            if self.skip_input != 'scan':
                self.scan_pub.publish(scan)
            if self.controller is not None:
                if self.skip_input != 'web':
                    self.controller.poll_web()
                if self.skip_input != 'status':
                    self.sock.queue.append((self.status_packet, ('10.21.31.103', 30000)))
                    self.controller.poll_telemetry()
            self.last_publish = now
        _spin_batch(self.executor, time.monotonic() + 0.005, time.monotonic)
        if self.controller is not None:
            self.controller.check_ownership_fatal()
            self.controller.tick()
        time.sleep(0.005)

    def until(self, predicate, timeout=4):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.cycle()
            if predicate():
                return
        self.fail('timed out waiting for isolated ROS condition')

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cycle()

    def setup_controller(self, session_seconds=10.0):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from motion_control.core import MotionGuard, EXPECTED_STATUS
        from motion_control.nav_sender import NavCmdSender
        from motion_control.runtime import Controller
        from motion_control.telemetry import TelemetryClient
        from test_follow_runtime import FakeSocket, packet, _web_payload
        node = Node('follow_test_controller', use_global_arguments=False,
                    start_parameter_services=False, enable_rosout=False)
        self.nodes.append(node)
        self.executor.add_node(node)
        self.sender = NavCmdSender(node=node, enabled=True)
        self.sock = FakeSocket()  # send() records heartbeat in memory only
        self.status_packet = packet({'BasicStatus': dict(EXPECTED_STATUS)})
        telemetry = TelemetryClient(sock=self.sock)
        def graph():
            infos = node.get_publishers_info_by_topic('/NAV_CMD')
            own = 1 if self.controller.pub_confirmed and any(
                i.node_name == node.get_name() for i in infos) else 0
            return len(infos) - own, len(node.get_publishers_info_by_topic('/new_chase/cmd_vel_raw'))
        self.controller = Controller(guard=MotionGuard(True), sender=self.sender,
                                     telemetry=telemetry, web_fetch=_web_payload,
                                     live=True, graph_probe=graph,
                                     session_seconds=session_seconds)
        node.create_subscription(self.LaserScan, '/new_chase/scan',
            lambda m: self.controller.on_scan(m, time.monotonic(), time.time()), qos_profile_sensor_data)
        node.create_subscription(self.Twist, '/new_chase/cmd_vel_raw', self.controller.on_raw, 1)
        self.until(lambda: self.controller.scan_rx > 3 and self.controller.raw_rx > 3)
        self.assertEqual(self.outputs, [])  # even valid inputs cannot auto-arm
        self.assertEqual(node.count_publishers('/NAV_CMD'), 0)
        self.assertTrue(self.controller.request_arm(True)[0])
        self.until(lambda: any(v[0] > 0 for v in self.outputs))

    def assert_stopped(self):
        self.until(lambda: self.outputs and self.outputs[-1] == (0.0, 0.0, 0.0))
        self.assertTrue(self.controller.snapshot()['session']['locked'])
        self.skip_input = None
        self.near = False
        n = len(self.outputs)
        self.pump(0.25)
        self.assertTrue(all(v == (0.0, 0.0, 0.0) for v in self.outputs[n:]))
        self.assertFalse(self.controller.request_arm(True)[0])

    def test_real_message_delivery_and_manual_disarm(self):
        self.setup_controller()
        self.assertAlmostEqual(self.outputs[-1][0], 0.25)
        self.assertEqual(self.outputs[-1][1:], (0.0, 0.0))
        self.controller.request_arm(False)
        self.assert_stopped()

    def test_scan_stream_loss(self):
        self.setup_controller()
        self.skip_input = 'scan'
        self.assert_stopped()

    def test_command_stream_loss(self):
        self.setup_controller()
        self.skip_input = 'raw'
        self.assert_stopped()

    def test_status_stream_loss(self):
        self.setup_controller()
        self.skip_input = 'status'
        self.assert_stopped()

    def test_web_stream_loss(self):
        self.setup_controller()
        self.skip_input = 'web'
        self.assert_stopped()

    def test_near_obstacle(self):
        self.setup_controller()
        self.near = True
        self.assert_stopped()

    def test_competing_raw_publisher(self):
        self.setup_controller()
        other = self.node.create_publisher(self.Twist, '/new_chase/cmd_vel_raw', 1)
        self.assert_stopped()
        self.node.destroy_publisher(other)

    def test_competing_nav_publisher(self):
        self.setup_controller()
        from drdds.msg import NavCmd
        # Advertise only; never publishes even within this isolated domain.
        other = self.node.create_publisher(NavCmd, '/NAV_CMD', 1)
        self.assert_stopped()
        self.node.destroy_publisher(other)

    def test_real_clock_ten_second_session(self):
        self.setup_controller()
        self.until(lambda: self.controller.snapshot()['session']['locked'], timeout=11)
        self.assertEqual(self.controller.snapshot()['session']['reason'], 'deadline_expired')
        self.assert_stopped()

    def test_real_clock_short_session(self):
        self.setup_controller(session_seconds=1.0)
        self.until(lambda: self.controller.snapshot()['session']['locked'], timeout=2)
        self.assertEqual(self.controller.snapshot()['session']['reason'], 'deadline_expired')
        self.assert_stopped()

    def test_main_dryrun_receives_streams_rejects_arm_without_nav_publisher(self):
        from std_srvs.srv import SetBool
        client = self.node.create_client(SetBool, '/new_chase/arm')
        # Exercise production main loop, subscriptions, service, diagnostics and
        # shutdown. Replace only HTTP fetch, explicitly disable real telemetry.
        code = '''
import motion_control.runtime as rt
from test_follow_runtime import _web_payload
original = rt.WebStatusWorker
rt.WebStatusWorker = lambda fetch: original(fetch=_web_payload)
raise SystemExit(rt.main(['--seconds', '10', '--no-telemetry']))
'''
        env = dict(os.environ)
        env['PYTHONPATH'] = '/home/user/new_chase:/home/user/new_chase/test:' + env.get('PYTHONPATH', '')
        process = subprocess.Popen([sys.executable, '-B', '-c', code], env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.until(client.service_is_ready)
            request = SetBool.Request()
            request.data = True
            future = client.call_async(request)
            self.until(future.done)
            self.assertFalse(future.result().success)
            self.assertIn('dryrun', future.result().message)
            self.until(lambda: process.poll() is not None, timeout=13)
            out, err = process.communicate(timeout=2)
            self.assertEqual(process.returncode, 0, err)
            summary = json.loads(out.strip().splitlines()[-1])
            self.assertEqual(summary['cleanup_errors'], [])
            snapshot = summary['snapshot']
            self.assertGreater(snapshot['scan']['valid'], 100)
            self.assertGreater(snapshot['raw']['rx'], 100)
            self.assertEqual(snapshot['sender']['calls'], 0)
            self.assertFalse(snapshot['pub_confirmed'])
            self.assertEqual(self.outputs, [])
            self.assertEqual(self.node.count_publishers('/NAV_CMD'), 0)
            print('DOMAIN47_MAIN_SUMMARY', json.dumps(summary))
        finally:
            if process.poll() is None:
                process.send_signal(2)
            remaining_out, remaining_err = process.communicate(timeout=5)
            if process.returncode != 0:
                print('MAIN_CHILD_ERROR', process.returncode, remaining_err)


if __name__ == '__main__':
    unittest.main()
