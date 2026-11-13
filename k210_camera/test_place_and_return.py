"""Same-scene placement guard tests; no hardware access."""
import copy,json,unittest
import numpy as np
from pathlib import Path
from check_grasp_board import ROOT
from place_and_return import placement_targets,supported_bottom_range

class PlacementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p=ROOT/'vision_grasp_results/grasp_trial_20261106_142707_cbe182.json'
        if not p.is_file():raise unittest.SkipTest('现场持物记录未包含在源码部署包')
        cls.trial=json.loads(p.read_text(encoding='utf-8'))
    def test_known_holding_pose_returns_only_body_targets(self):
        actual=self.trial['observations'][-1]['after']['joint_median'];limits={j:(125,875) for j in (3,4,5,6)}
        result=placement_targets(self.trial,actual,limits)
        self.assertEqual(set(result),{'3','4','5','6'})
        self.assertEqual(result,{'3':447,'4':127,'5':711,'6':481})
    def test_unknown_lift_must_not_be_replayed(self):
        trial=copy.deepcopy(self.trial);trial['commands'][-1]['task']['status']='outcome_unknown'
        with self.assertRaises(ValueError):placement_targets(trial,trial['observations'][-1]['after']['joint_median'],{j:(125,875) for j in (3,4,5,6)})
    def test_reused_baseline_returns_to_postclosure_feedback(self):
        trial=copy.deepcopy(self.trial)
        trial['lift_baseline_source']='preclosure_accepted_support_observation'
        trial['postclosure_feedback']=trial['observations'].pop(1)['after']
        actual=trial['observations'][-1]['after']['joint_median']
        self.assertEqual(placement_targets(trial,actual,{j:(125,875) for j in (3,4,5,6)}),
                         {'3':447,'4':127,'5':711,'6':481})
    def test_reset_pose_rejected(self):
        j={**self.trial['observations'][-1]['after']['joint_median'],'3':181,'5':412}
        with self.assertRaises(ValueError):placement_targets(self.trial,j,{k:(125,875) for k in (3,4,5,6)})
    def test_one_supported_corner_is_not_full_support(self):
        T=np.eye(4);a=np.radians(8);T[:3,:3]=[[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]];T[2,3]=-2.5
        lo,hi=supported_bottom_range({'T_board_object':T.tolist()})
        self.assertGreater(hi-lo,.4)

if __name__=='__main__':unittest.main()
