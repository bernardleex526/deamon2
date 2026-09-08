import math
import socket
import struct
import unittest
from types import SimpleNamespace as NS

from m20_adapter.core import (Guard, ZERO, HEADER, MAGIC, UdpClient,
                              encode, decode, velocity_items, inspect_scan)


def status(**changes):
    value = dict(MotionState=17, Gait=12290, Charge=0, HES=0, ControlUsageMode=1, Sleep=0, Direction=0)
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


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.guard = Guard(clock=lambda: self.now)

    def ready(self):
        self.guard.update_status(status())
        self.guard.update_scan(True, True)
        self.assertTrue(self.guard.arm())

    def test_default_disarmed_and_arm_requires_status_and_scan(self):
        self.assertEqual(self.guard.output(), ZERO)
        self.assertFalse(self.guard.arm())
        self.guard.update_status(status())
        self.assertFalse(self.guard.arm())

    def test_limits_and_signs(self):
        self.ready()
        self.guard.update_command((2., -2., 3.))
        self.assertEqual(self.guard.output(), (0.3, -0.3, 0.6))

    def test_deadband_never_amplifies(self):
        self.ready()
        self.guard.update_command((0.14, -0.24, 0.34))
        self.assertEqual(self.guard.output(), ZERO)

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
        for key, value in [('MotionState', 1), ('ControlUsageMode', 0), ('HES', 1),
                           ('Charge', 1), ('Sleep', 1), ('Gait', 99), ('Direction', 1)]:
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
        self.assertTrue(self.guard.arm())

    def test_obstacle_and_invalid_scan_latch(self):
        for valid, clear in ((True, False), (False, False)):
            self.ready()
            self.guard.update_scan(valid, clear)
            self.assertFalse(self.guard.armed)
            self.guard.update_scan(True, True)
            self.assertEqual(self.guard.output(), ZERO)

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

    def test_bad_configuration(self):
        for kw in ({'limits': (0, 1, 1)}, {'scan_timeout': -1},
                   {'status_timeout': float('nan')}):
            with self.assertRaises(ValueError):
                Guard(**kw)


class ScanTests(unittest.TestCase):
    def scan(self, ranges=(2.,), angle=0.):
        return NS(header=NS(stamp=NS(sec=100, nanosec=0)), ranges=ranges,
                  angle_min=angle, angle_increment=0.01, range_min=0.1, range_max=12.)

    def test_fresh_clear(self):
        self.assertEqual(inspect_scan(self.scan(), 100.2), (True, True))

    def test_timestamp_old_or_future(self):
        for now in (100.6, 99.8):
            self.assertEqual(inspect_scan(self.scan(), now), (False, False))

    def test_front_rear_and_side_obstacles(self):
        for distance, angle in ((0.6, 0.), (0.6, math.pi), (0.3, math.pi / 2)):
            self.assertEqual(inspect_scan(self.scan((distance,), angle), 100.), (True, False))

    def test_empty_invalid_and_no_return(self):
        for values in ((), (float('nan'),), (float('inf'),), (0.,), (-1.,), (15.,)):
            self.assertEqual(inspect_scan(self.scan(values), 100.), (False, False))

    def test_bad_geometry(self):
        scan = self.scan()
        scan.angle_increment = 0.
        self.assertEqual(inspect_scan(scan, 100.), (False, False))


if __name__ == '__main__':
    unittest.main()
