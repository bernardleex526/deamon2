"""Constructor and delayed-message contracts; mock transport, isolated ROS domain."""
import json
import os
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

os.environ['ROS_DOMAIN_ID'] = '85'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.parameter import Parameter
from m20_adapter.bridge import Bridge
from m20_adapter.core import Guard
from m20_adapter.selection import make_selection_request, make_enable_request


class BridgeContracts(unittest.TestCase):
    def test_enable_request_preserves_selection_and_epoch(self):
        request = make_enable_request(NS(sec=123, nanosec=456), 2**60, 7)
        values = {p.name: p.value for p in request.parameters}
        self.assertEqual(set(values), {'stamp_sec', 'stamp_nanosec', 'fault_epoch', 'selection_id'})
        self.assertTrue(all(v.type == 2 for v in values.values()))
        self.assertEqual(values['selection_id'].integer_value, 7)
        self.assertEqual(values['fault_epoch'].integer_value, 2**60)
        with self.assertRaises(ValueError):
            make_enable_request(NS(sec=123, nanosec=456), 1, 0)

    def test_selection_request_preserves_epoch_and_stamp(self):
        request = make_selection_request(1.82, 0., 'lidar_link', NS(sec=123, nanosec=456), 2**60)
        values = {p.name: p.value for p in request.parameters}
        self.assertEqual(len(values), 6)
        self.assertEqual(values['fault_epoch'].integer_value, 2**60)
        self.assertEqual(values['stamp_sec'].integer_value, 123)
        self.assertEqual(values['stamp_nanosec'].integer_value, 456)
        self.assertEqual(values['frame_id'].string_value, 'lidar_link')
        with self.assertRaises(ValueError):
            make_selection_request(1.82, 0., 'lidar_link', NS(sec=-1, nanosec=0), 1)

    def test_live_constructor_requires_tracking_and_expected_gait(self):
        for extra in [[], ['-p', 'require_tracking_status:=true']]:
            rclpy.init(args=['--ros-args', '-p', 'dry_run:=false', '-p', 'commissioned:=true'] + extra)
            try:
                with patch('m20_adapter.bridge.UdpClient') as udp:
                    with self.assertRaises(RuntimeError):
                        node = Bridge()
                        node.destroy_node()
                    udp.assert_not_called()
            finally:
                rclpy.shutdown()

    def test_clock_mode_cannot_change_after_start(self):
        rclpy.init(args=[])
        node = Bridge()
        try:
            result = node.set_parameters([Parameter('use_sim_time', value=True)])
            self.assertFalse(result[0].successful)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_tracking_does_not_gain_a_second_age_budget(self):
        now = [100.]
        guard = Guard(clock=lambda: now[0], require_tracking=True)
        status = dict(MotionState=17, Gait=4097, Charge=0, HES=0,
                      ControlUsageMode=1, Sleep=0, Direction=0)
        guard.update_status(status)
        guard.update_scan(True, True)
        obj = NS(guard=guard, get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(now[0]*1e9))))
        Bridge.tracking(obj, NS(data=json.dumps(dict(stamp=99.71, selection_id=1, tracking_valid=True))))
        self.assertTrue(guard.arm())
        guard.update_command((.22, 0., 0.))
        now[0] += .02
        self.assertEqual(guard.output(), (0., 0., 0.))
        self.assertFalse(guard.armed)


if __name__ == '__main__':
    unittest.main()
