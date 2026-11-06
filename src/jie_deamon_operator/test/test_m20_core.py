import math
import socket
import struct
import tempfile
import unittest
from types import SimpleNamespace as NS

from m20_adapter.core import (CMD_AXIS_NORMALIZED, CMD_VELOCITY_SI, GAIT_TABLE, Guard,
                              MODE_AUX, MODE_NAV, MODE_REGULAR, ZERO, HEADER, MAGIC,
                              UdpClient, encode, decode, inspect_scan, inspect_scan_report,
                              normalize_velocity, normalized_items, select_command,
                              velocity_items)
from m20_adapter.preflight import (REPORT_VERSION, build_report, check_preflight,
                                   write_report)


def status(**changes):
    value = dict(MotionState=17, Gait=12290, Charge=0, HES=0,
                 ControlUsageMode=1, Sleep=0, Direction=0)
    value.update(changes)
    return {'BasicStatus': value}


class ProtocolTests(unittest.TestCase):
    def test_heartbeat_only_registers_telemetry(self):
        from unittest.mock import Mock
        client = UdpClient.__new__(UdpClient)
        client.send = Mock()
        client.heartbeat()
        client.send.assert_called_once_with(100, 100, {})

    def test_known_header_and_si_units(self):
        packet = encode(2, 25, velocity_items((0.3, -0.25, 0.4)), 0x1234, '2026-09-06 12:00:00')
        self.assertEqual(packet[:4], bytes.fromhex('eb91eb90'))
        self.assertEqual(packet[4:6], struct.pack('<H', len(packet) - 16))
        self.assertEqual(packet[6:16], bytes.fromhex('34120100000000000000'))
        _, msg = decode(packet)
        self.assertEqual((msg['Type'], msg['Command']), (2, 25))
        self.assertEqual(msg['Items'], dict(X=0.3, Y=-0.25, Z=0., Roll=0., Pitch=0., Yaw=0.4))

    def test_normalized_cmd21_units(self):
        items = normalized_items((0.3, -0.25, 0.6), (2.0, 1.0, 2.0))
        self.assertAlmostEqual(items['X'], 0.15)
        self.assertAlmostEqual(items['Y'], -0.25)
        self.assertAlmostEqual(items['Yaw'], 0.3)
        self.assertEqual(items['Z'], 0.0)
        # A value beyond full scale saturates instead of wrapping.
        self.assertEqual(normalize_velocity((5.0, -5.0, 5.0), (2.0, 1.0, 2.0)), (1.0, -1.0, 1.0))

    def test_utf8_length(self):
        packet = encode(100, 100, {'label': '\u6d4b\u8bd5'}, 0)
        self.assertEqual(HEADER.unpack_from(packet)[1], len(packet[16:]))
        self.assertEqual(decode(packet)[1]['Items']['label'], '\u6d4b\u8bd5')

    def test_bad_packets(self):
        packet = encode(100, 100, {}, 0)
        for bad in (b'', packet[:15], packet[:-1], packet + b'x', b'xxxx' + packet[4:],
                    packet[:8] + b'\x00' + packet[9:], packet[:9] + b'1' + packet[10:]):
            with self.subTest(packet=bad), self.assertRaises(ValueError):
                decode(bad)

    def test_bad_json_and_structure(self):
        for body in (b'no', b'[]', b'{"PatrolDevice":{}}',
                     b'{"PatrolDevice":{"Type":2,"Command":25,"Items":[]}}'):
            with self.subTest(body=body), self.assertRaises(ValueError):
                decode(HEADER.pack(MAGIC, len(body), 0, 1, bytes(7)) + body)

    def test_encoder_bounds(self):
        for value in (-1, 65536):
            with self.assertRaises(ValueError):
                encode(2, 25, {}, value)
        with self.assertRaises(ValueError):
            encode(2, 25, {'x': 'a' * 65536}, 1)
        with self.assertRaises(ValueError):
            velocity_items((float('nan'), 0, 0))

    def test_real_loopback_udp_source_port_status_and_id_wrap(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.bind(('127.0.0.1', 0))
        server.settimeout(1.0)
        client = UdpClient('127.0.0.1', server.getsockname()[1])
        try:
            client.message_id = 65535
            client.send(1002, 6, {})
            packet, source = server.recvfrom(65536)
            self.assertEqual(decode(packet)[0], 65535)
            server.sendto(encode(1002, 6, status(), 65535), source)
            # Wait for readability, without assuming datagram scheduling latency.
            import select
            self.assertTrue(select.select([client.socket], [], [], 1)[0])
            self.assertEqual(client.receive()[0][1]['Items'], status())
            client.send(2, 25, velocity_items(ZERO))
            packet, next_source = server.recvfrom(65536)
            self.assertEqual(next_source, source)
            self.assertEqual(decode(packet)[0], 0)
        finally:
            client.close()
            server.close()


class CommandSelectionTests(unittest.TestCase):
    def test_vendor_mode_matrix(self):
        # modes.txt / motion_basic.txt: Cmd=25 navigation only, Cmd=21 regular+aux.
        self.assertEqual(select_command('auto', MODE_NAV), CMD_VELOCITY_SI)
        self.assertEqual(select_command('auto', MODE_REGULAR), CMD_AXIS_NORMALIZED)
        self.assertIsNone(select_command('auto', MODE_AUX))
        self.assertEqual(select_command('auto', MODE_AUX, True), CMD_AXIS_NORMALIZED)
        self.assertIsNone(select_command('si', MODE_REGULAR))
        self.assertEqual(select_command('si', MODE_NAV), CMD_VELOCITY_SI)
        self.assertIsNone(select_command('normalized', MODE_NAV))
        self.assertEqual(select_command('normalized', MODE_REGULAR), CMD_AXIS_NORMALIZED)
        with self.assertRaises(ValueError):
            select_command('nonsense', MODE_NAV)

    def test_gait_table_matches_vendor_ranges(self):
        self.assertEqual(GAIT_TABLE[4097][1:4], (0.20, 0.35, 0.50))
        self.assertEqual(GAIT_TABLE[4097][4:], (2.0, 1.0, 2.0))
        self.assertEqual(GAIT_TABLE[12290][1:4], (0.15, 0.25, 0.35))
        self.assertEqual(GAIT_TABLE[12290][6], 1.5)


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.guard = Guard(clock=lambda: self.now)

    def ready(self, **changes):
        self.guard.update_status(status(**changes))
        self.guard.update_scan(True, True)
        # The gate refuses to arm unless the velocity stream is already live.
        self.guard.update_command((0.3, 0., 0.))
        self.assertTrue(self.guard.arm())

    def test_default_disarmed_and_arm_requires_status_and_scan(self):
        self.assertEqual(self.guard.output(), ZERO)
        self.assertFalse(self.guard.arm())
        self.guard.update_status(status())
        self.assertFalse(self.guard.arm())

    def test_arm_requires_live_command_stream(self):
        self.guard.update_status(status())
        self.guard.update_scan(True, True)
        self.assertFalse(self.guard.arm())
        self.assertEqual(self.guard.reason, 'no fresh command stream')
        self.guard.update_command((0.3, 0., 0.))
        self.assertTrue(self.guard.arm())
        # A stale stream (no command for > command_timeout) cannot arm either.
        self.now += 0.31
        self.guard.disarm('test')
        self.assertFalse(self.guard.arm())
        self.assertEqual(self.guard.reason, 'no fresh command stream')

    def test_arm_without_command_stream_for_operator_gateway(self):
        # The operator gateway starts the stream only after the operator presses
        # start, so its gate is constructed with arm_requires_command=False.
        guard = Guard(clock=lambda: self.now, arm_requires_command=False)
        guard.update_status(status())
        guard.update_scan(True, True)
        self.assertTrue(guard.arm())

    def test_limits_and_signs(self):
        self.ready()
        self.guard.update_command((2., -2., 3.))
        self.assertEqual(self.guard.output(), (0.3, -0.3, 0.6))

    def test_axis_can_be_disabled_by_configuration(self):
        guard = Guard(clock=lambda: self.now, axis_enable=(True, False, True))
        guard.update_status(status())
        guard.update_scan(True, True)
        guard.update_command((0.3, 0.3, 0.6))
        self.assertTrue(guard.arm())
        guard.update_command((0.3, 0.3, 0.6))
        self.assertEqual(guard.output(), (0.3, 0.0, 0.6))
        self.assertTrue(any('y axis disabled' in note for note in guard.axis_notes))

    def test_reverse_policy_depends_on_command_source(self):
        self.ready()
        self.guard.update_command((-0.3, 0., 0.), source='algorithm')
        self.assertEqual(self.guard.output(), ZERO, 'algorithm may not reverse by default')
        self.guard.update_command((-0.3, 0., 0.), source='operator')
        self.assertEqual(self.guard.output()[0], -0.3, 'operator teleoperation may reverse')

    def test_lateral_enabled_passes_and_clamps(self):
        guard = Guard(clock=lambda: self.now, axis_enable=(True, True, True))
        guard.update_status(status())
        guard.update_scan(True, True)
        guard.update_command((0.3, 0., 0.))
        self.assertTrue(guard.arm())
        guard.update_command((0., 0.9, 0.))
        # 12290 yaw min 0.35 but y min 0.25 -> 0.3 safety limit wins.
        self.assertEqual(guard.output(), (0.0, 0.3, 0.0))

    def test_regular_mode_is_now_usable(self):
        self.ready(ControlUsageMode=MODE_REGULAR)
        self.assertEqual(self.guard.command_kind, CMD_AXIS_NORMALIZED)
        self.guard.update_command((0.3, 0., 0.))
        self.assertEqual(self.guard.output(), (0.3, 0.0, 0.0))
        self.assertAlmostEqual(self.guard.wire_items()['X'], 0.15)

    def test_auxiliary_mode_needs_opt_in(self):
        self.guard.update_status(status(ControlUsageMode=MODE_AUX))
        self.assertEqual(self.guard.status_problem(),
                         'usage mode 2 (auxiliary) incompatible with control profile auto')
        guard = Guard(clock=lambda: self.now, allow_auxiliary=True)
        guard.update_status(status(ControlUsageMode=MODE_AUX))
        self.assertEqual(guard.command_kind, CMD_AXIS_NORMALIZED)

    def test_unknown_mode_rejected(self):
        self.guard.update_status(status(ControlUsageMode=7))
        self.assertIn('incompatible', self.guard.status_problem())

    def test_deadband_never_amplifies(self):
        self.ready()
        self.guard.update_command((0.14, -0.24, 0.34))
        self.assertEqual(self.guard.output(), ZERO)

    def test_snap_policy_raises_nonzero_request_to_gait_minimum(self):
        guard = Guard(clock=lambda: self.now, gait_min_policy='snap')
        guard.update_status(status())
        guard.update_scan(True, True)
        guard.update_command((0.3, 0., 0.))
        self.assertTrue(guard.arm())
        guard.update_command((0.16, 0., 0.30))
        # 12290: x 0.16 >= min 0.15 (kept), yaw 0.30 < min 0.35 -> snapped up.
        self.assertEqual(guard.output(), (0.16, 0.0, 0.35))
        guard.update_command((0.01, 0., 0.))
        self.assertEqual(guard.output()[0], 0.0, 'noise gate must still zero tiny requests')

    def test_standard_basic_gait_disables_lateral_by_minimum(self):
        guard = Guard(clock=lambda: self.now, axis_enable=(True, True, True),
                      limits=(0.3, 0.3, 0.6))
        guard.update_status(status(Gait=4097))
        guard.update_scan(True, True)
        guard.update_command((0.3, 0., 0.))
        self.assertTrue(guard.arm())
        guard.update_command((0.3, 0.3, 0.6))
        # 0x1001 y minimum 0.35 > 0.3 limit -> y unusable, reported not silent.
        self.assertEqual(guard.output(), (0.3, 0.0, 0.6))
        self.assertTrue(any('y axis unusable' in note for note in guard.axis_notes), guard.axis_notes)

    def test_reverse_blocked_unless_enabled(self):
        self.ready()
        self.guard.update_command((-0.3, 0., 0.))
        self.assertEqual(self.guard.output(), ZERO)
        guard = Guard(clock=lambda: self.now, allow_reverse=True)
        guard.update_status(status())
        guard.update_scan(True, True)
        guard.update_command((0.3, 0., 0.))
        guard.arm()
        guard.update_command((-0.3, 0., 0.))
        self.assertEqual(guard.output()[0], -0.3)

    def test_small_limit_below_gait_minimum(self):
        self.guard.limits = (0.1, 0.1, 0.1)
        self.ready()
        self.guard.update_command((1., 1., 1.))
        self.assertEqual(self.guard.output(), ZERO)

    def test_command_loss_latches(self):
        self.ready()
        self.guard.update_command((0.3, 0., 0.))
        self.now += 0.31
        self.assertEqual(self.guard.output(), ZERO)
        self.guard.update_command((0.3, 0., 0.))
        self.assertFalse(self.guard.armed)
        self.assertEqual(self.guard.output(), ZERO)

    def test_scan_loss_despite_continuous_commands(self):
        self.ready()
        self.now += 0.51
        self.guard.update_command((0.3, 0., 0.))
        self.assertEqual(self.guard.output(), ZERO)
        self.assertFalse(self.guard.armed)

    def test_status_loss_despite_scan_and_commands(self):
        self.ready()
        self.now += 1.51
        self.guard.update_scan(True, True)
        self.guard.update_command((0.3, 0., 0.))
        self.assertEqual(self.guard.output(), ZERO)
        self.assertFalse(self.guard.armed)

    def test_all_status_interlocks(self):
        for key, value in [('MotionState', 1), ('HES', 1), ('Charge', 1), ('Sleep', 1),
                           ('Gait', 99), ('Direction', 1)]:
            with self.subTest(key=key):
                self.ready()
                self.guard.update_status(status(**{key: value}))
                self.assertFalse(self.guard.armed)
                self.assertEqual(self.guard.output(), ZERO)

    def test_partial_and_wrong_type_status_rejected(self):
        for value in ({}, {'BasicStatus': []}, status(HES=False), status(Charge='0')):
            with self.subTest(value=value):
                self.ready()
                self.guard.update_status(value)
                self.assertFalse(self.guard.armed)

    def test_flat_complete_status_supported(self):
        self.guard.update_status(status()['BasicStatus'])
        self.guard.update_scan(True, True)
        self.guard.update_command((0.3, 0., 0.))
        self.assertTrue(self.guard.arm())

    def test_obstacle_and_invalid_scan_latch(self):
        for valid, clear in ((True, False), (False, False)):
            self.ready()
            self.guard.update_scan(valid, clear)
            self.assertFalse(self.guard.armed)
            self.guard.update_scan(True, True)
            self.assertEqual(self.guard.output(), ZERO)

    def test_stop_hits_debounce(self):
        guard = Guard(clock=lambda: self.now, stop_hits_required=3)
        guard.update_status(status())
        guard.update_scan(True, True)
        guard.update_command((0.3, 0., 0.))
        self.assertTrue(guard.arm())
        guard.update_scan(True, False, hits=1)
        self.assertTrue(guard.armed, 'a single noisy frame must not disarm')
        guard.update_scan(True, True)
        guard.update_scan(True, False, hits=1)
        guard.update_scan(True, False, hits=4)
        guard.update_scan(True, False, hits=4)
        self.assertFalse(guard.armed, 'persistent returns must disarm')

    def test_rearm_drops_old_command(self):
        self.ready()
        self.guard.update_command((0.3, 0., 0.))
        self.guard.disarm('stop')
        self.guard.update_command((0.3, 0., 0.))
        self.assertTrue(self.guard.arm())
        self.assertEqual(self.guard.output(), ZERO)

    def test_nonfinite_commands(self):
        for v in (float('nan'), float('inf'), -float('inf')):
            self.ready()
            self.guard.update_command((v, 0., 0.))
            self.assertEqual(self.guard.output(), ZERO)
            self.assertFalse(self.guard.armed)

    def test_status_dict_reports_mode_and_notes(self):
        guard = Guard(clock=lambda: self.now, axis_enable=(True, True, True))
        guard.update_status(status(ControlUsageMode=MODE_REGULAR, Gait=4097))
        info = guard.status_dict()
        self.assertEqual(info['command_kind'], CMD_AXIS_NORMALIZED)
        self.assertEqual(info['usage_mode_name'], 'regular')
        self.assertTrue(any('y axis unusable' in note for note in info['axis_notes']),
                        info['axis_notes'])
        disabled = Guard(clock=lambda: self.now, axis_enable=(True, False, True))
        disabled.update_status(status(Gait=4097))
        self.assertTrue(any('disabled by configuration' in note
                            for note in disabled.status_dict()['axis_notes']))

    def test_bad_configuration(self):
        for kw in ({'limits': (0, 1, 1)}, {'scan_timeout': -1},
                   {'status_timeout': float('nan')}, {'profile': 'x'},
                   {'gait_min_policy': 'x'}, {'axis_max': (1, 1, 0)},
                   {'stop_hits_required': 0}, {'axis_enable': (True, True)}):
            with self.assertRaises(ValueError):
                Guard(**kw)


class ScanTests(unittest.TestCase):
    def scan(self, ranges=(2.,), angle=0., increment=0.01):
        return NS(header=NS(stamp=NS(sec=100, nanosec=0)), ranges=ranges,
                  angle_min=angle, angle_increment=increment, range_min=0.1, range_max=12.)

    def test_fresh_clear(self):
        self.assertEqual(inspect_scan(self.scan(), 100.2), (True, True))

    def test_timestamp_old_or_future(self):
        for now in (100.6, 99.8):
            self.assertEqual(inspect_scan(self.scan(), now), (False, False))

    def test_front_and_side_obstacles_inside_sector(self):
        # Stop zone is x in [-0.7, 0.7] and |y| <= 0.4, outside the self mask
        # (which needs BOTH |x| <= front and |y| <= side to be treated as self).
        for distance, angle in ((0.6, 0.0), (0.610, 0.611), (0.610, -0.611)):
            with self.subTest(angle=angle):
                self.assertEqual(inspect_scan(self.scan((distance,), angle), 100.), (True, False))

    def test_rear_obstacle_outside_guard_sector(self):
        # Rear return at 180 deg is outside +-100 deg and must not trip the stop
        # zone; the side/front returns at 5 m keep sector coverage.
        scan = self.scan((0.6, 5.0, 5.0, 5.0), angle=-math.pi, increment=math.pi / 2)
        report = inspect_scan_report(scan, 100.)
        self.assertTrue(report.valid, report.reason)
        self.assertTrue(report.clear)
        self.assertEqual(report.sector_bins, 3)

    def test_no_sector_coverage_fails_closed(self):
        report = inspect_scan_report(self.scan((0.6,), angle=math.pi), 100.)
        self.assertFalse(report.valid)
        self.assertIn('sector', report.reason)

    def test_min_sector_bins_fails_closed(self):
        report = inspect_scan_report(self.scan((5.0,), 0.0), 100., min_sector_bins=20)
        self.assertFalse(report.valid)
        self.assertIn('coverage', report.reason)

    def test_self_returns_inside_footprint_are_ignored(self):
        # Real 2026-09-08 observation: persistent leg return at x=-0.37, y=+0.26.
        scan = self.scan((0.4478,), angle=math.atan2(0.25687, -0.36685))
        report = inspect_scan_report(scan, 100., sector_min=-math.pi, sector_max=math.pi,
                                     self_margin_back=0.14, self_margin_left=0.07)
        self.assertEqual(report.self_hits, 1)
        self.assertEqual(report.stop_hits, 0)
        self.assertTrue(report.clear)

    def test_front_self_return_ignored_but_front_obstacle_kept(self):
        inside = inspect_scan_report(self.scan((0.447,), math.atan2(0.20, 0.40)), 100.)
        self.assertEqual(inside.self_hits, 1)
        self.assertTrue(inside.clear)
        outside = inspect_scan_report(self.scan((0.62,), 0.0), 100.)
        self.assertEqual(outside.stop_hits, 1)
        self.assertFalse(outside.clear)
        self.assertEqual(outside.closest_stop['distance'], 0.62)

    def test_min_range_filter(self):
        report = inspect_scan_report(self.scan((0.3, 5.0), -0.6), 100., min_range=0.4)
        self.assertEqual(report.stop_hits, 0)

    def test_empty_invalid_and_no_return(self):
        for values in ((), (float('nan'),), (float('inf'),), (0.,), (-1.,), (15.,)):
            self.assertEqual(inspect_scan(self.scan(values), 100.), (False, False))

    def test_bad_geometry(self):
        scan = self.scan()
        scan.angle_increment = 0.
        self.assertEqual(inspect_scan(scan, 100.), (False, False))


class PreflightTests(unittest.TestCase):
    def test_all_pass_report_accepted(self):
        checks = [dict(name='root', ok=True), dict(name='relay', ok=True)]
        report = build_report(checks, now=1000.0)
        self.assertTrue(report['ok'])
        with tempfile.TemporaryDirectory() as tmp:
            path = write_report(tmp + '/report.json', report)
            result = check_preflight(path, max_age=600, now=1100.0)
        self.assertTrue(result['ok'], result['reason'])

    def test_failed_required_check_rejected(self):
        report = build_report([dict(name='root', ok=False, fix='sudo -i')], now=1000.0)
        self.assertFalse(report['ok'])
        with tempfile.TemporaryDirectory() as tmp:
            path = write_report(tmp + '/report.json', report)
            result = check_preflight(path, max_age=600, now=1000.0)
        self.assertFalse(result['ok'])
        self.assertIn('root', result['reason'])

    def test_info_check_does_not_block(self):
        report = build_report([dict(name='time_sync', ok=False, severity='info')], now=1000.0)
        self.assertTrue(report['ok'])

    def test_stale_wrong_version_and_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = tmp + '/report.json'
            self.assertIn('missing', check_preflight(path, now=1000.0)['reason'])
            write_report(path, build_report([dict(name='root', ok=True)], now=1000.0))
            self.assertIn('age', check_preflight(path, max_age=10, now=1100.0)['reason'])
            stale = build_report([dict(name='root', ok=True)], now=1000.0)
            stale['version'] = REPORT_VERSION + 1
            write_report(path, stale)
            self.assertIn('version', check_preflight(path, now=1000.0)['reason'])


if __name__ == '__main__':
    unittest.main()
