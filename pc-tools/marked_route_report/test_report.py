import json
import math
from pathlib import Path
import tempfile
import unittest

from build import OrderedProjection, map_track, build
from collect import latest


class ReportTests(unittest.TestCase):
    def test_failed_run_is_not_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name,stage,outcome in [('marked-route-20260930-190000','finished','succeeded'),
                                        ('marked-route-20260930-200000','finished','failed_preflight'),
                                        ('marked-route-20260930-210000','driving',None)]:
                p=root/name;p.mkdir();(p/'session.json').write_text(json.dumps(dict(stage=stage,outcome=outcome)))
            result=latest(root)
            self.assertEqual(result['run_id'],'marked-route-20260930-200000')
            self.assertEqual(result['skipped_running'],['marked-route-20260930-210000'])

    def test_map_transform_uses_time_and_rotation(self):
        tf=lambda t,x:dict(stamp=t,translation=dict(x=x,y=2),rotation=dict(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
        raw=dict(transforms=[tf(10,1),tf(10.2,3)],poses=[dict(stamp=10.1,x=1,y=0,yaw=0,quality='ok')])
        track,_=map_track(raw,[])
        self.assertAlmostEqual(track[0]['x'],2)
        self.assertAlmostEqual(track[0]['y'],3)
        self.assertAlmostEqual(track[0]['yaw'],math.pi/2)
        raw['poses'][0]['stamp']=12
        self.assertEqual(map_track(raw,[])[0],[])

    def test_crossing_cannot_jump_to_later_branch(self):
        projection=OrderedProjection([[0,0],[3,0],[3,3],[0,3],[0,.1],[3,.1]])
        first=projection.project(0,0);second=projection.project(.4,.1)
        self.assertLess(second[2],1)
        self.assertAlmostEqual(second[1],.1)

    def test_empty_failure_has_no_fabricated_movement(self):
        raw=dict(run_id='marked-route-20260930-200000',source='record',session=dict(outcome='failed_preflight',goal_sent=False),
                 states=[],imu=[],poses=[],metadata={},events=[],plans=[],map_poses=[],transforms=[])
        report=build(raw)
        self.assertEqual(report['path'],[])
        self.assertIsNone(report['stats']['max_speed'])
        self.assertIsNone(report['stats']['max_error'])
        self.assertFalse(report['goal_sent'])


if __name__=='__main__':unittest.main()
