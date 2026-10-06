import json
import math
from pathlib import Path
import tempfile
import unittest
from model import build_model, add_path_angles
from collect_run import collect_latest, collect_run


def sample():
    return dict(run_id='s-curve-20260928-120000-1',source='/saved',session={'outcome':'succeeded','elapsed_s':4},
        metadata={},reference={'anchor':{'x':10,'y':20,'yaw':math.pi/2},
        'points':[{'x':10,'y':20},{'x':10,'y':22}]},states=[],poses=[])


class StatisticsTests(unittest.TestCase):
    def test_per_point_heading_motion_and_imu_alignment(self):
        path=dict(t=[0.,.1,.2,.3,.4],actual=[[0,0],[0,.1],[0,.2],[0,.3],[0,.3]])
        imu=dict(t=[.01,.19,.3],yaw_deg=[179,-179,None])
        add_path_angles(path,imu)
        self.assertEqual(path['imu_yaw_deg'][:3],[179,-179,-179])
        self.assertIsNone(path['imu_yaw_deg'][3])
        self.assertIsNone(path['imu_yaw_deg'][4])
        self.assertAlmostEqual(path['imu_dt_s'][2],-.01)
        self.assertEqual(path['motion_deg'][:2],[None,None])
        self.assertAlmostEqual(path['motion_deg'][2],90)
        self.assertAlmostEqual(path['motion_deg'][3],90)

    def test_motion_bearing_has_no_false_angle_for_stops_or_gaps(self):
        path=dict(t=[0.,.1,.2,None,2.,2.1,2.2],
                  actual=[[0,0],[.001,0],[.002,0],[None,None],[5,5],[5.1,5],[5.2,5]])
        add_path_angles(path,dict(t=[],yaw_deg=[]))
        self.assertEqual(path['motion_deg'][:6],[None]*6)
        self.assertAlmostEqual(path['motion_deg'][6],0)
        self.assertEqual(path['imu_yaw_deg'],[None]*7)

    def test_heading_rotates_with_path_without_using_target_bearing(self):
        raw=sample();raw['states']=[dict(stamp=100,active=True)]
        raw['poses']=[dict(stamp=100,x=10,y=21,yaw=math.pi,quality='ok'),
                      dict(stamp=100.1,x=10,y=21.1,yaw=math.pi,quality='ok'),
                      dict(stamp=101.1,x=10,y=22,yaw=float('nan'),quality='ok')]
        heads=build_model(raw)['path']['headings']
        self.assertEqual(len(heads),1)
        self.assertAlmostEqual(heads[0]['x'],1)
        self.assertAlmostEqual(heads[0]['y'],0)
        self.assertAlmostEqual(heads[0]['yaw_deg'],90)
        self.assertEqual(heads[0]['t'],0)

    def test_imu_time_quaternion_validity_and_gaps(self):
        raw=sample();raw['states']=[dict(stamp=100,active=True)]
        q=dict(x=0,y=0,z=math.sqrt(2),w=math.sqrt(2))
        raw['imu']=[dict(stamp=100.1,angular_velocity=dict(x=.1,y=.2,z=.3),orientation=q),
                    dict(stamp=102,angular_velocity=dict(z=99),orientation=q,
                         orientation_available=False,gyro_available=False)]
        imu=build_model(raw)['imu']
        self.assertAlmostEqual(imu['t'][0],.1)
        self.assertAlmostEqual(imu['yaw_deg'][0],90)
        self.assertEqual(imu['wz'],[.3,None,None])
        self.assertEqual(imu['yaw_deg'][1:],[None,None])

    def test_collector_reads_imu_even_with_existing_state_csv(self):
        with tempfile.TemporaryDirectory() as folder:
            run=Path(folder);tracking=run/'tracking';tracking.mkdir()
            (run/'session.json').write_text(json.dumps(dict(stage='finished',goal_sent=True)))
            (tracking/'metadata.json').write_text('{}')
            (tracking/'closed_loop_state.csv').write_text('stamp,active\n10,True\n')
            entry=dict(topic='/IMU_data',message=dict(header=dict(stamp=dict(sec=10,nanosec=500000000)),
                       angular_velocity=dict(x=1,y=2,z=3),orientation_covariance=[-1]))
            (tracking/'raw.jsonl').write_text(json.dumps(entry)+'\n{broken\n')
            raw=collect_run(run)
            self.assertEqual(len(raw['states']),1)
            self.assertEqual(raw['imu'][0]['stamp'],10.5)
            self.assertFalse(raw['imu'][0]['orientation_available'])

    def test_invalid_and_idle_excluded_from_peak_and_average(self):
        raw=sample()
        for stamp,v,active,fresh in [(99,99,False,True),(100,1,True,True),(101,2,True,True),
                                     (102,100,True,False),(103,3,True,True),(104,90,False,True)]:
            raw['states'].append(dict(stamp=stamp,measured_v=v,active=active,feedback_fresh=fresh))
        model=build_model(raw)
        self.assertEqual(model['stats']['max_speed'],3)
        self.assertEqual(model['stats']['mean_speed'],1.5)
        self.assertEqual(model['stats']['speed_coverage_s'],1)
        self.assertIsNone(model['series']['measured_v'][3])

    def test_no_feedback_does_not_show_zero(self):
        raw=sample();raw['states']=[dict(stamp=1,active=True,feedback_fresh=False,measured_v=0)]
        model=build_model(raw)
        self.assertIsNone(model['stats']['max_speed'])
        self.assertIsNone(model['stats']['mean_speed'])

    def test_fixed_anchor_and_signed_error(self):
        raw=sample();raw['states']=[dict(stamp=t,active=True,feedback_fresh=True,measured_v=1) for t in (1,2)]
        raw['poses']=[dict(stamp=1.5,x=9.9,y=21,quality='ok')]
        model=build_model(raw)
        self.assertAlmostEqual(model['path']['actual'][0][0],1)
        self.assertAlmostEqual(model['path']['actual'][0][1],.1)
        self.assertAlmostEqual(model['stats']['max_error'],.1)
        self.assertAlmostEqual(model['path']['error'][0],.1)

    def test_jumps_gaps_and_nonfinite_values(self):
        raw=sample();raw['states']=[dict(stamp=1,active=True,feedback_fresh=True,measured_v=float('nan')),
            dict(stamp=5,active=True,feedback_fresh=True,measured_v=1)]
        raw['poses']=[dict(stamp=t,x=10,y=21,quality=q) for t,q in [(1,'ok'),(1.1,'pose_jump'),(1.2,'ok')]]
        model=build_model(raw)
        self.assertIn(None,model['series']['t'])
        self.assertIn([None,None],model['path']['actual'])
        self.assertIsNone(model['stats']['mean_speed'])
        json.dumps(model,allow_nan=False)

    def test_running_and_preflight_not_selected(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for name,stage,goal in [('s-curve-20260928-100000','finished',True),
                                    ('s-curve-20260928-110000','finished',False),
                                    ('s-curve-20260928-120000','driving',True)]:
                run=root/name;(run/'tracking').mkdir(parents=True)
                (run/'session.json').write_text(json.dumps({'stage':stage,'goal_sent':goal}))
                (run/'tracking/metadata.json').write_text('{}')
            selected=collect_latest(root)
            self.assertEqual(selected['run_id'],'s-curve-20260928-100000')
            self.assertEqual(len(selected['skipped_newer']),2)


if __name__=='__main__':unittest.main()
