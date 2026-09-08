import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from scripts import m20_status_probe


class StatusProbeTests(unittest.TestCase):
    def run_probe(self, items):
        client = Mock()
        client.receive.return_value = [(0, dict(Type=1002, Command=6, Items=items))]
        with patch.object(m20_status_probe, 'UdpClient', return_value=client), \
                patch('sys.argv', ['probe', '--seconds', '0.001']), \
                contextlib.redirect_stdout(io.StringIO()):
            result = m20_status_probe.main()
        client.heartbeat.assert_called_once_with()
        client.send.assert_not_called()
        client.close.assert_called_once_with()
        return result

    def test_error_and_incomplete_status_fail(self):
        for items in ({'ErrorCode': 1}, {'BasicStatus': {'MotionState': 17}}, {}):
            with self.subTest(items=items):
                self.assertEqual(self.run_probe(items), 1)

    def test_complete_manual_mode_is_valid_telemetry_not_motion_permission(self):
        items = {'BasicStatus': dict(MotionState=17, Gait=4097, Charge=0, HES=0,
                                     ControlUsageMode=0, Sleep=0, Direction=0)}
        self.assertEqual(self.run_probe(items), 0)
