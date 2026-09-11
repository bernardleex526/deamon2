"""Field layout and generator compatibility regression."""
from types import SimpleNamespace as NS
import unittest
from m20_adapter.cloud_points import xyz_rows


class CloudRowsTests(unittest.TestCase):
    def test_foxy_generator_respects_field_offsets(self):
        fields = [NS(name=n, offset=o, count=1) for n, o in [('z', 0), ('x', 4), ('y', 8)]]
        points = (p for p in [(3., 1., 2.), (6., 4., 5.)])
        self.assertEqual(xyz_rows(points, fields), [(1., 2., 3.), (4., 5., 6.)])

    def test_missing_coordinate_is_rejected(self):
        with self.assertRaises(ValueError):
            xyz_rows(iter([]), [])

    def test_humble_structured_array(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('NumPy is exercised in the ROS CI jobs')
        fields = [NS(name=n, offset=o, count=1) for n, o in [('z', 0), ('x', 4), ('y', 8)]]
        points = np.array([(3., 1., 2.)], dtype=[('z', '<f4'), ('x', '<f4'), ('y', '<f4')])
        self.assertEqual(xyz_rows(points, fields), [(1., 2., 3.)])
