#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FOLLOW-RUNTIME-TEST: parse_web_status / decode_basic_status 冒烟（标准库 unittest，离线）。"""

import unittest
import json
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch, Mock

from motion_control.core import MotionGuard, EXPECTED_STATUS
from motion_control.nav_sender import NavCmdSender
from motion_control.inputs import check_scan
from motion_control.runtime import (
    Controller, _spin_batch, _wait_live_discovery, parse_runtime_args,
    RuntimeArgsError, WebStatusWorker)
from motion_control.telemetry import (
    TelemetryClient, HEADER, MAGIC, FORMAT_JSON, RESERVED, decode_apdu)
from test_nav_sender import FakeNode, FakeMsg

from motion_control.inputs import WEB_REQUIRED_FLAGS, parse_web_status
from motion_control.telemetry import decode_basic_status


def _web_payload(**overrides):
    data = {key: True for key in WEB_REQUIRED_FLAGS}
    data['target'] = {'x': 1.2, 'y': -0.5}
    data.update(overrides)
    return data


class TestParseWebStatus(unittest.TestCase):

    def test_all_flags_true_finite_target_valid(self):
        out = parse_web_status(_web_payload())
        self.assertTrue(out['valid'])
        self.assertEqual(out['reason'], 'ok')
        self.assertEqual(out['target'], (1.2, -0.5))
        self.assertEqual(out['flags'], {key: True for key in WEB_REQUIRED_FLAGS})

    def test_rejects_false_flag(self):
        out = parse_web_status(_web_payload(is_tracking_valid=False))
        self.assertFalse(out['valid'])
        self.assertEqual(out['reason'], 'flag_not_true:is_tracking_valid')

    def test_rejects_nan_target(self):
        payload = _web_payload(target={'x': float('nan'), 'y': 0.5})
        out = parse_web_status(payload)
        self.assertFalse(out['valid'])
        self.assertEqual(out['reason'], 'target_nonfinite')


class TestDecodeBasicStatus(unittest.TestCase):

    def test_bad_bytes_raise_valueerror(self):
        with self.assertRaises(ValueError):
            decode_basic_status(b'bad')


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def scan(distance=2.0, stamp=1000, **changes):
    msg = NS(header=NS(frame_id='lidar_link', stamp=NS(sec=stamp, nanosec=0)),
             angle_min=0.0, angle_max=0.0, angle_increment=0.01,
             range_min=0.1, range_max=20.0, ranges=[distance])
    for key, value in changes.items():
        setattr(msg, key, value)
    return msg


def twist(x=0.25, y=0.0, yaw=0.0):
    return NS(linear=NS(x=x, y=y), angular=NS(z=yaw))


class Harness:
    """Pure memory: real controller/guard/sender, fake transport and clock."""
    def __init__(self, live=True, session_seconds=10.0):
        self.clock = Clock()
        self.node = FakeNode()
        self.graph = (0, 1)
        self.payload = _web_payload()
        self.events = []
        self.guard = MotionGuard(output_enabled=True)
        self.sender = NavCmdSender(node=self.node, enabled=live, message_type=FakeMsg)
        self.controller = Controller(
            guard=self.guard, sender=self.sender, live=live, monotonic=self.clock,
            telemetry=NS(poll=lambda now: self.events),
            web_fetch=lambda: self.payload, graph_probe=lambda: self.graph,
            session_seconds=session_seconds)

    def fresh(self, omit=None):
        c, t = self.controller, self.clock()
        if omit != 'status':
            self.events = [dict(kind='status', status=dict(EXPECTED_STATUS), received_at=t)]
            c.poll_telemetry()
            self.events = []
        if omit != 'scan':
            c.on_scan(scan(), t, 1000.0)
        if omit != 'web':
            c.poll_web()
        if omit != 'raw':
            c.on_raw(twist())

    def arm_and_send(self):
        self.fresh()
        ok, why = self.controller.request_arm(True)
        assert ok, why
        assert self.controller.tick().allowed

    def velocities(self):
        return [(m.data.x_vel, m.data.y_vel, m.data.yaw_vel)
                for m in self.node.pub.published]


class TestController(unittest.TestCase):
    def test_default_dryrun_never_creates_publisher(self):
        h = Harness(live=False)
        h.fresh()
        self.assertFalse(h.controller.request_arm(True)[0])
        h.controller.tick()
        h.controller.shutdown_output()
        self.assertEqual(h.node.created, [])
        self.assertEqual(h.velocities(), [])

    def test_live_requires_manual_arm_then_maps_and_limits(self):
        h = Harness()
        h.fresh()
        h.controller.tick()
        self.assertEqual(h.node.created, [])
        h.controller.on_raw(twist(9, 3, -9))
        self.assertTrue(h.controller.request_arm(True)[0])
        self.assertTrue(h.controller.tick().allowed)
        self.assertEqual(h.velocities(), [(0.3, 0.0, -0.6)])
        self.assertEqual(h.node.created[0][1], '/NAV_CMD')

    def test_missing_each_input_rejects_arm_without_publication(self):
        for missing in ('status', 'scan', 'web', 'raw'):
            with self.subTest(missing=missing):
                h = Harness()
                h.fresh(omit=missing)
                self.assertFalse(h.controller.request_arm(True)[0])
                h.controller.tick()
                self.assertEqual(h.node.created, [])

    def test_each_input_expiry_stops_and_cannot_rearm(self):
        for missing in ('status', 'scan', 'web', 'raw'):
            with self.subTest(missing=missing):
                h = Harness()
                h.arm_and_send()
                h.clock.now += 0.51
                h.fresh(omit=missing)
                self.assertFalse(h.controller.tick().allowed)
                self.assertEqual(h.velocities()[-1], (0, 0, 0))
                h.fresh()
                self.assertFalse(h.controller.request_arm(True)[0])
                h.controller.tick()
                self.assertEqual(len(h.velocities()), 2)

    def test_bad_then_good_before_tick_never_resumes(self):
        for source in ('scan', 'raw', 'web', 'status'):
            with self.subTest(source=source):
                h = Harness()
                h.arm_and_send()
                c = h.controller
                if source == 'scan':
                    c.on_scan(scan(0.7), h.clock(), 1000.0)
                elif source == 'raw':
                    c.on_raw(twist(y=float('nan')))
                elif source == 'web':
                    h.payload = _web_payload(is_tracking_valid=False)
                    c.poll_web()
                    h.payload = _web_payload()
                else:
                    h.events = [dict(kind='status', status={}, received_at=h.clock())]
                    c.poll_telemetry()
                h.fresh()
                self.assertFalse(c.tick().allowed)
                self.assertEqual(h.velocities()[-1], (0, 0, 0))
                self.assertFalse(c.request_arm(True)[0])

    def test_exact_ten_second_deadline_with_fresh_inputs(self):
        h = Harness()
        h.arm_and_send()
        h.clock.now = 109.999
        h.fresh()
        self.assertTrue(h.controller.tick().allowed)
        self.assertFalse(h.controller.request_arm(True)[0])
        h.clock.now = 110.0
        h.fresh()
        self.assertFalse(h.controller.tick().allowed)
        self.assertEqual(h.controller.snapshot()['session']['reason'], 'deadline_expired')
        self.assertEqual(h.velocities()[-1], (0, 0, 0))
        h.clock.now = 111
        h.fresh()
        self.assertFalse(h.controller.request_arm(True)[0])

    def test_short_window_stops_at_exact_one_second(self):
        h = Harness(session_seconds=1.0)
        h.arm_and_send()
        h.clock.now = 100.999
        h.fresh()
        self.assertTrue(h.controller.tick().allowed)
        h.clock.now = 101.0
        h.fresh()
        self.assertFalse(h.controller.tick().allowed)
        self.assertEqual(h.velocities()[-1], (0, 0, 0))
        self.assertEqual(h.controller.snapshot()['session']['reason'], 'deadline_expired')
        self.assertFalse(h.controller.request_arm(True)[0])

    def test_window_cannot_exceed_ten_seconds_or_be_invalid(self):
        for value in (0, -1, 10.1, float('nan'), float('inf'), True, '1'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Harness(session_seconds=value)

    def test_motion_seconds_cli_is_bounded(self):
        self.assertEqual(parse_runtime_args(['--motion-seconds', '1'])['motion_seconds'], 1)
        self.assertEqual(parse_runtime_args([])['motion_seconds'], 10)
        for value in ('0', '-1', '10.1', 'nan', 'inf', 'bad'):
            with self.subTest(value=value), self.assertRaises(RuntimeArgsError):
                parse_runtime_args(['--motion-seconds', value])
    def test_manual_disarm_flushes_one_zero_immediately(self):
        h = Harness()
        h.arm_and_send()
        self.assertTrue(h.controller.request_arm(False)[0])
        self.assertEqual(h.velocities(), [(0.25, 0, 0), (0, 0, 0)])
        h.controller.request_arm(False)
        h.controller.shutdown_output()
        self.assertEqual(len(h.velocities()), 2)
        self.assertFalse(h.controller.request_arm(True)[0])

    def test_shutdown_flushes_zero(self):
        h = Harness()
        h.arm_and_send()
        h.controller.shutdown_output()
        self.assertEqual(h.velocities()[-1], (0, 0, 0))
        self.assertFalse(h.controller.request_arm(True)[0])

    def test_ownership_conflicts_or_unknown_stop_and_lock(self):
        for graph in ((1, 1), (0, 2), (0, 0), (None, 1), (False, 1), (0, True)):
            with self.subTest(graph=graph):
                h = Harness()
                h.arm_and_send()
                h.graph = graph
                self.assertTrue(h.controller.check_ownership_fatal()[1])
                h.graph = (0, 1)
                h.controller.check_ownership_fatal()
                self.assertFalse(h.controller.tick().allowed)
                self.assertEqual(h.velocities()[-1], (0, 0, 0))
                self.assertFalse(h.controller.request_arm(True)[0])

    def test_ownership_probe_exception_rejects_arm(self):
        h = Harness()
        h.fresh()
        def fail():
            raise OSError('graph failure')
        h.controller._graph_probe = fail
        self.assertFalse(h.controller.request_arm(True)[0])
        self.assertEqual(h.node.created, [])

    def test_healthy_discovery_clears_unlocked_diagnostic(self):
        h = Harness()
        h.graph = (0, 0)
        h.controller.check_ownership_fatal()
        self.assertIsNotNone(h.controller.live_block_reason)
        h.graph = (0, 1)
        h.controller.check_ownership_fatal()
        self.assertIsNone(h.controller.live_block_reason)

    def test_all_telemetry_faults_latch_even_if_next_packet_good(self):
        for kind in ('decode_error', 'recv_error', 'heartbeat_error', 'comm_fault'):
            with self.subTest(kind=kind):
                h = Harness()
                h.arm_and_send()
                h.events = [dict(kind=kind), dict(kind='status',
                            status=dict(EXPECTED_STATUS), received_at=h.clock())]
                h.controller.poll_telemetry()
                h.fresh()
                self.assertFalse(h.controller.tick().allowed)
                self.assertFalse(h.controller.request_arm(True)[0])

    def test_publish_error_latches_without_recreation(self):
        h = Harness()
        h.arm_and_send()
        h.node.pub.publish_error = OSError('wire failed')
        self.assertFalse(h.controller.tick().allowed)
        self.assertEqual(h.controller.send_errors, 1)
        h.node.pub.publish_error = None
        h.fresh()
        self.assertFalse(h.controller.request_arm(True)[0])
        h.controller.tick()
        self.assertEqual(len(h.node.created), 1)
        self.assertEqual(len(h.velocities()), 1)  # cannot claim zero was delivered

    def test_cached_web_result_does_not_refresh_timestamp(self):
        h = Harness()
        h.arm_and_send()
        item = (parse_web_status(_web_payload()), h.clock(), 1)
        worker = NS(latest=lambda: item)
        h.controller.consume_web(worker)
        h.clock.now += 0.51
        h.fresh(omit='web')
        h.controller.consume_web(worker)
        self.assertFalse(h.controller.tick().allowed)
        self.assertEqual(h.controller.last_decision.reason, 'tracking_stale')

    def test_callback_exception_cannot_resume(self):
        h = Harness()
        h.arm_and_send()
        h.controller.disarm_for_input_error('scan', ValueError('bad callback'))
        h.fresh()
        self.assertFalse(h.controller.tick().allowed)
        self.assertFalse(h.controller.request_arm(True)[0])

    def test_http_failure_disarms_even_if_fetch_recovers(self):
        h = Harness()
        h.arm_and_send()
        def fail():
            raise TimeoutError('HTTP timeout')
        h.controller._web_fetch = fail
        h.controller.poll_web()
        self.assertIn('http_error', h.controller.web_last_reason)
        h.controller._web_fetch = _web_payload
        h.fresh()
        self.assertFalse(h.controller.tick().allowed)
        self.assertEqual(h.velocities()[-1], (0, 0, 0))
        self.assertFalse(h.controller.request_arm(True)[0])

    def test_web_worker_cannot_overwrite_observed_invalid_with_good(self):
        h = Harness()
        h.arm_and_send()
        release = threading.Event()
        calls = []
        def fetch():
            calls.append(1)
            if len(calls) == 1:
                return _web_payload(is_tracking_valid=False)
            if len(calls) > 2:
                release.wait(1)
            return _web_payload()
        worker = WebStatusWorker(fetch, period=0.001, monotonic=h.clock)
        worker.start()
        try:
            end = time.monotonic() + 1
            while time.monotonic() < end:
                item = worker.latest()
                if item is not None and item[2] >= 2:
                    break
                time.sleep(0.001)
            self.assertGreaterEqual(worker.latest()[2], 2)
            self.assertTrue(worker.latest()[0]['valid'])
            h.controller.consume_web(worker)
            self.assertFalse(h.controller.tick().allowed,
                             'worker observed a loss; later good cannot hide it')
            self.assertEqual(h.velocities()[-1], (0, 0, 0))
        finally:
            release.set()
            self.assertTrue(worker.stop()[1])


class TestInputValidation(unittest.TestCase):
    def test_each_web_flag_requires_exact_true(self):
        for key in WEB_REQUIRED_FLAGS:
            for value in (False, None, 1, 'true'):
                with self.subTest(key=key, value=value):
                    self.assertFalse(parse_web_status(_web_payload(**{key: value}))['valid'])

    def test_duplicate_json_keys_rejected(self):
        self.assertFalse(parse_web_status('{"is_active":true,"is_active":false}')['valid'])

    def test_scan_clear_near_body_and_no_returns(self):
        for distance, valid, clear, reason in (
                (2, True, True, 'ok'), (0.7, True, False, 'stop_zone'),
                (0.2, False, False, 'envelope_only'),
                (float('inf'), False, False, 'no_valid_points')):
            with self.subTest(distance=distance):
                out = check_scan(scan(distance), 1000.0)
                self.assertEqual((out['valid'], out['clear'], out['reason']), (valid, clear, reason))

    def test_scan_stale_future_frame_and_delivery(self):
        for msg, wall, delay, reason in (
                (scan(), 1000.6, 0, 'stamp_stale'),
                (scan(), 999.8, 0, 'stamp_future'),
                (scan(), 1000, 0.6, 'delivery_late'),
                (scan(header=NS(frame_id='map', stamp=NS(sec=1000, nanosec=0))),
                 1000, 0, 'frame_mismatch')):
            with self.subTest(reason=reason):
                self.assertEqual(check_scan(msg, wall, delay)['reason'], reason)

    def test_scan_malformed_ranges_fail_closed_without_throwing(self):
        for ranges in (None, {}, {0: 2.0, 2: 2.0}):
            with self.subTest(ranges=ranges):
                self.assertFalse(check_scan(scan(ranges=ranges), 1000)['valid'])


class TestScheduling(unittest.TestCase):
    def test_main_shutdown_stops_output_before_waiting_for_web_worker(self):
        self.check_main_shutdown(False)

    def test_worker_shutdown_exception_does_not_skip_other_cleanup(self):
        self.check_main_shutdown(True)

    def check_main_shutdown(self, stop_error):
        # No ROS init/DDS: patch the main lifecycle boundaries, retain real
        # Controller+GuardedOutput+sender using the memory publisher.
        from motion_control.runtime import main
        import contextlib
        import io
        h = Harness()
        h.arm_and_send()
        worker = Mock()
        def stop():
            self.assertEqual(h.velocities()[-1], (0, 0, 0),
                             'must stop output before a potentially 1s worker join')
            if stop_error:
                raise RuntimeError('worker join failed')
            return False, True
        worker.stop.side_effect = stop
        with contextlib.ExitStack() as stack:
            for name, value in (
                    ('motion_control.runtime.check_process_env', None),
                    ('rclpy.init', None), ('rclpy.ok', False),
                    ('rclpy.node.Node', Mock()),
                    ('rclpy.executors.SingleThreadedExecutor', Mock()),
                    ('motion_control.runtime.NavCmdSender', h.sender),
                    ('motion_control.runtime.Controller', h.controller),
                    ('motion_control.runtime.TelemetryClient', Mock()),
                    ('motion_control.runtime.WebStatusWorker', worker),
                    ('motion_control.runtime._wait_live_discovery', False)):
                stack.enter_context(patch(name, return_value=value))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            self.assertEqual(main(['--live', '--commissioned',
                                   '--exclusive-control-confirmed']), 2 if stop_error else 0)
        worker.stop.assert_called_once()
        self.assertEqual(h.node.destroyed, [h.node.pub])

    def test_batch_count_and_time_limits(self):
        clock = Clock()
        executor = NS(spin_once=lambda **kw: calls.append(kw))
        calls = []
        _spin_batch(executor, clock() + 0.005, clock)
        self.assertEqual(len(calls), 16)
        self.assertTrue(all(x == {'timeout_sec': 0} for x in calls))
        calls.clear()
        def slow(**kw):
            calls.append(kw)
            clock.sleep(0.006)
        executor.spin_once = slow
        _spin_batch(executor, clock() + 0.005, clock)
        self.assertEqual(len(calls), 1)

    def test_discovery_waits_only_for_absent_raw(self):
        h = Harness()
        h.graph = (0, 0)
        def advance(seconds):
            h.clock.sleep(seconds)
            h.graph = (0, 1)
        with patch('motion_control.runtime.time.sleep', side_effect=advance):
            self.assertFalse(_wait_live_discovery(h.controller, h.clock))
        self.assertEqual(h.node.created, [])

    def test_discovery_timeout_is_bounded(self):
        h = Harness()
        h.graph = (0, 0)
        with patch('motion_control.runtime.time.sleep', side_effect=h.clock.sleep):
            self.assertTrue(_wait_live_discovery(h.controller, h.clock))
        self.assertLessEqual(h.clock(), 103.051)

    def test_discovery_invalid_counts_reject_without_wait(self):
        for graph in ((1, 1), (0, 2), (None, 0), (False, 1), (0, -1)):
            with self.subTest(graph=graph):
                h = Harness()
                h.graph = graph
                with patch('motion_control.runtime.time.sleep', side_effect=h.clock.sleep) as sleep:
                    self.assertTrue(_wait_live_discovery(h.controller, h.clock))
                sleep.assert_not_called()

    def test_discovery_stop_wins_over_healthy_graph(self):
        h = Harness()
        self.assertTrue(_wait_live_discovery(h.controller, h.clock, stop_check=lambda: True))

    def test_cli_requires_all_live_flags_and_rejects_remaps(self):
        for argv in (['--live'], ['--live', '--commissioned'],
                     ['--live', '--commissioned', '--exclusive-control-confirmed', '--no-telemetry'],
                     ['/NAV_CMD:=/cmd_vel'], ['--seconds', '0']):
            with self.subTest(argv=argv), self.assertRaises(RuntimeArgsError):
                parse_runtime_args(argv)
        self.assertFalse(parse_runtime_args([])['live'])


def packet(items, command=6):
    body = json.dumps({'PatrolDevice': dict(Type=1002, Command=command,
                      Time='2026-09-18 00:00:00', Items=items)}).encode()
    return HEADER.pack(MAGIC, len(body), 0, FORMAT_JSON, RESERVED) + body


class FakeSocket:
    def __init__(self):
        self.queue = []
        self.sent = []

    def send(self, data):
        self.sent.append(data)
        return len(data)

    def recvfrom(self, size):
        if not self.queue:
            raise BlockingIOError()
        return self.queue.pop(0)


class TestTelemetry(unittest.TestCase):
    def test_ack_and_other_packets_never_refresh_status(self):
        clock, sock = Clock(), FakeSocket()
        client = TelemetryClient(sock=sock, monotonic=clock)
        addr = ('10.21.31.103', 30000)
        sock.queue = [(packet({'BasicStatus': dict(EXPECTED_STATUS)}), addr)]
        client.poll()
        self.assertEqual(client.status_received_at, 100)
        clock.now = 101
        sock.queue = [(packet({'ErrorCode': 0}), addr), (packet({}, command=3), addr)]
        kinds = [e['kind'] for e in client.poll()]
        self.assertIn('ack_ok', kinds)
        self.assertIn('ignored', kinds)
        self.assertEqual(client.status_received_at, 100)
        for data in sock.sent:
            _, value = decode_apdu(data)
            self.assertEqual((value['Type'], value['Command']), (100, 100))

    def test_partial_malformed_wrong_source_are_faults(self):
        sock = FakeSocket()
        client = TelemetryClient(sock=sock, monotonic=Clock())
        sock.queue = [(packet({'BasicStatus': {'MotionState': 17}}), ('10.21.31.103', 30000)),
                      (b'bad', ('10.21.31.103', 30000)),
                      (packet({'BasicStatus': dict(EXPECTED_STATUS)}), ('127.0.0.1', 30000))]
        events = client.poll()
        self.assertEqual(sum(e['kind'] == 'decode_error' for e in events), 3)
        self.assertIsNone(client.status_received_at)


if __name__ == '__main__':
    unittest.main()
