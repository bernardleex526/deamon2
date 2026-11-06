import unittest

from m20_adapter.core import Guard
from m20_adapter.operator_control import OperatorControl


class OperatorTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        # The operator gateway owns the command stream, so arming does not require
        # a pre-existing one (the bridge constructs its guard the same way).
        self.guard = Guard(clock=lambda: self.now, arm_requires_command=False)
        self.guard.update_status(dict(MotionState=17, Gait=12290, Charge=0,
                                      HES=0, ControlUsageMode=1, Sleep=0, Direction=0))
        self.guard.update_scan(True, True)
        self.control = OperatorControl(self.guard, clock=lambda: self.now)
        self.seq = 0

    def send(self, op, source='web', client='phone', **fields):
        self.seq += 1
        self.control.handle(dict(client=client, seq=self.seq, op=op, **fields), source)

    def test_direct_limits_and_no_automatic_takeover(self):
        self.send('start', mode='direct')
        self.send('velocity', velocity=[1., 1., 1.])
        self.assertEqual(self.guard.output(), (.3, .3, .6))
        with self.assertRaisesRegex(ValueError, 'another operator'):
            self.send('velocity', source='android', velocity=[-1., 0., 0.])
        self.assertEqual(self.guard.output(), (.3, .3, .6))

    def test_heartbeat_does_not_refresh_old_direct_velocity(self):
        self.send('start', mode='direct')
        self.send('velocity', velocity=[.3, 0., 0.])
        self.now += .31
        self.send('heartbeat')
        self.assertEqual(self.guard.output(), (0., 0., 0.))
        self.control.tick()
        self.assertIsNone(self.control.owner)

    def test_follow_lease_expires_despite_continuing_algorithm(self):
        self.send('start', mode='follow', target=[2., 0.])
        self.control.follow_command((.3, 0., 0.))
        self.assertEqual(self.guard.output(), (.3, 0., 0.))
        self.now += .61
        self.control.follow_command((.3, 0., 0.))
        self.assertFalse(self.guard.armed)
        with self.assertRaisesRegex(ValueError, 'start first'):
            self.send('heartbeat')

    def test_obstacle_latches_and_restart_requires_explicit_start(self):
        self.send('start', mode='direct')
        self.guard.update_scan(True, False)
        self.control.tick()
        self.guard.update_scan(True, True)
        self.assertFalse(self.guard.armed)
        self.send('start', mode='direct')
        self.assertTrue(self.guard.armed)
        self.assertEqual(self.guard.output(), (0., 0., 0.))

    def test_foreign_stop_and_replayed_start(self):
        self.send('start', mode='direct')
        self.send('stop', source='android')
        with self.assertRaisesRegex(ValueError, 'replayed'):
            self.control.handle(dict(client='phone', seq=1, op='start', mode='direct'), 'web')
        self.assertFalse(self.guard.armed)

    def test_unsupported_actions_and_nonfinite_input(self):
        for action in ('standup', 'liedown', 'passive'):
            with self.assertRaisesRegex(ValueError, 'not mapped'):
                self.send('action', action=action)
        with self.assertRaises(ValueError):
            self.send('start', mode='follow', target=[float('nan'), 0.])
        self.assertFalse(self.guard.armed)

    def test_follow_ignored_in_direct_mode(self):
        self.send('start', mode='direct')
        self.control.follow_command((.3, 0., 0.))
        self.assertEqual(self.guard.output(), (0., 0., 0.))

    def test_no_start_without_real_status(self):
        self.guard.status_at = -float('inf')
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.send('start', mode='direct')
        self.assertFalse(self.guard.armed)

    def test_extreme_integer_is_rejected_without_crash(self):
        with self.assertRaises(ValueError):
            self.send('start', mode='follow', target=[10**400, 0])

    def test_new_operator_can_stop_at_session_capacity(self):
        self.send('start', mode='direct')
        self.control.sequences.update({str(i): 1 for i in range(128)})
        self.send('stop', client='new-client')
        self.assertFalse(self.guard.armed)
        self.assertIsNone(self.control.owner)

    def test_preview_never_arms_without_robot_status(self):
        self.control = OperatorControl(self.guard, clock=lambda: self.now, preview=True)
        self.guard.status_at = -float('inf')
        self.send('start', mode='follow', target=[2., 0.])
        self.control.follow_command((.3, 0., 0.))
        self.assertEqual(self.control.raw_velocity, (.3, 0., 0.))
        self.assertFalse(self.guard.armed)
        self.assertEqual(self.guard.output(), (0., 0., 0.))
        self.now += .61
        self.control.tick()
        self.assertIsNone(self.control.owner)

    def test_new_start_always_changes_follow_session(self):
        self.send('start', mode='follow', target=[2., 0.])
        old = self.control.session
        self.send('stop')
        self.send('start', mode='follow', target=[2., 0.])
        self.assertNotEqual(old, self.control.session)


if __name__ == '__main__':
    unittest.main()
