"""Cross-component configuration contract."""
import copy
import unittest

from m20_adapter.profile import validate_follow_profile


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.config = {'robot_nexus': {'ros__parameters': dict(
            gait_aware_follow=True, minimum_target_points=3,
            min_follow_vx=.22, max_follow_vx=.30,
            min_follow_wz=.52, max_follow_wz=.60)},
            'm20_bridge': {'ros__parameters': dict(expected_gait=4097,
                require_tracking_status=True, max_vx=.30, max_wz=.60)}}

    def test_valid_profile(self):
        validate_follow_profile(self.config, live=True)

    def test_inconsistent_limits_and_gates_rejected(self):
        for node, key, value in [('m20_bridge', 'max_vx', .1),
                                 ('m20_bridge', 'expected_gait', 12290),
                                 ('m20_bridge', 'require_tracking_status', False),
                                 ('robot_nexus', 'min_follow_wz', .1),
                                 ('robot_nexus', 'minimum_target_points', 1),
                                 ('robot_nexus', 'tracking_timeout', .6),
                                 ('m20_bridge', 'tracking_timeout', .5),
                                 ('robot_nexus', 'max_follow_vx', float('nan'))]:
            config = copy.deepcopy(self.config)
            config[node]['ros__parameters'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_follow_profile(config, live=True)

    def test_legacy_allowed_only_in_dry_run(self):
        validate_follow_profile({}, live=False)
        with self.assertRaises(ValueError):
            validate_follow_profile({}, live=True)
