import math
import unittest
from route import make_route, nearest_error, relative_pose, transform_pose


class GeometryTests(unittest.TestCase):
    def test_smooth_start_end_and_envelope(self):
        route = make_route()
        for p in (route['poses'][0], route['poses'][-1]):
            self.assertAlmostEqual(p['y_m'], 0)
            self.assertAlmostEqual(p['yaw_rad'], 0)
            self.assertAlmostEqual(p['curvature_per_m'], 0)
        self.assertLessEqual(max(abs(p['y_m']) for p in route['poses']), 0.25 + 1e-8)
        self.assertGreater(max(p['y_m'] for p in route['poses']), 0.249)
        self.assertLess(min(p['y_m'] for p in route['poses']), -0.249)
        self.assertLess(route['max_model_steering_deg'], 20)
        self.assertGreater(route['min_model_radius_m'], 0.75)
        self.assertGreater(route['length_m'], 4)

    def test_rotated_anchor_roundtrip(self):
        pose = dict(x_m=2, y_m=0.25, yaw_rad=0.3)
        anchor = (3, -2, math.pi/2)
        result = transform_pose(pose, anchor)
        self.assertAlmostEqual(result[0], 2.75)
        self.assertAlmostEqual(result[1], 0)
        local = relative_pose(*result, anchor)
        for value, expected in zip(local, (2, 0.25, 0.3)):
            self.assertAlmostEqual(value, expected)

    def test_segment_projection_and_side(self):
        poses = [dict(x_m=0, y_m=0, distance_m=0), dict(x_m=2, y_m=0, distance_m=2)]
        self.assertEqual(nearest_error(1, 0.3, poses), (0.3, 1))
        self.assertEqual(nearest_error(1, -0.3, poses), (-0.3, 1))
        self.assertEqual(nearest_error(3, 0, poses), (1, 2))

    def test_invalid_dimensions(self):
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                make_route(length=value)


if __name__ == '__main__':
    unittest.main()
