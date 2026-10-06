import math
import unittest
from tracking_core import generate, anchor, project, stats, inferred_pwm


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.path = generate()

    def test_smooth_path_and_radius(self):
        p = self.path['points']
        self.assertLess(max(abs(q['curvature']) for q in p), 1.00001)
        self.assertAlmostEqual(p[0]['yaw'], p[-1]['yaw'])
        self.assertLess(p[-1]['x'], 6); self.assertLess(p[-1]['y'], 6)
        self.assertAlmostEqual(p[0]['curvature'], p[-1]['curvature'])

    def test_signed_offset_and_heading_wrap(self):
        r = project(self.path, .2, .1, math.radians(359))
        self.assertAlmostEqual(r['cross_track_m'], .1)
        self.assertAlmostEqual(r['heading_error_deg'], -1)
        self.assertAlmostEqual(r['s_m'], .2)

    def test_anchor_preserves_geometry(self):
        a = anchor(self.path, 3, -2, math.pi/2)
        r = project(a, 2.9, -1.8, math.pi/2)
        self.assertAlmostEqual(r['cross_track_m'], .1)
        self.assertAlmostEqual(r['s_m'], .2)

    def test_no_data_and_idle_not_perfect_tracking(self):
        self.assertEqual(stats([], self.path)['status'], 'no_data')
        r = stats([dict(x=0,y=0,yaw=0)]*100, self.path)
        self.assertEqual(r['status'], 'insufficient_motion')
        self.assertIsNone(r['tracking_metrics'])

    def test_exact_trajectory_and_partial(self):
        r = stats(self.path['points'], self.path)
        self.assertEqual(r['status'], 'complete_geometry')
        self.assertLess(r['tracking_metrics']['all']['rmse_m'], 1e-10)
        self.assertEqual(stats(self.path['points'][:80], self.path)['status'], 'partial')

    def test_quality_flag(self):
        rows = [dict(p, quality='ok') for p in self.path['points']]
        rows[40]['quality'] = 'pose_jump'
        self.assertEqual(stats(rows,self.path)['status'],'review_pose_quality')

    def test_pwm_is_limited(self):
        p = dict(speed_epsilon_mps=.01, motor_neutral_pwm=1500, servo_center_pwm=1489,
            wheelbase_m=.248, left_angle_deg=30, right_angle_deg=30, servo_left_pwm=1679,
            servo_right_pwm=1299, max_speed_mps=.2, forward_pwm_per_mps=250,
            reverse_pwm_per_mps=250, motor_min_pwm=1500, motor_max_pwm=1600)
        self.assertEqual(inferred_pwm(.2, 100, p)[:2], (1550,1679))
        self.assertTrue(inferred_pwm(.2,100,p)[3])
        self.assertEqual(inferred_pwm(.2,-100,p)[:2],(1550,1299))
        self.assertEqual(inferred_pwm(0,100,p)[:2],(1500,1489))

if __name__ == '__main__':
    unittest.main()
