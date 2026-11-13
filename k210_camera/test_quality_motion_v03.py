"""Actual-scene nominal and biased-feedback replay; no hardware I/O.

Uses the same coarse/near planners as the executable pipeline. This checks
logic/path feasibility, not camera accuracy or actual contact quality.
"""
import copy,json,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from check_grasp_board import ROOT
from grasp_motion_profile import load_motion_profile,pitch_in_profile
from grasp_approach_policy import approach_decision,in_grasp_region
from sample_grasp_handeye import nominal_model
from approach_grasp_candidate import PregraspPathUnavailable,build_pregrasp_plan,plan_box_pregrasp
from tcp_pose_planner import choose_step
from guarded_visual_approach import compensated_targets
from fit_grasp_handeye import robot_frame

class OffsetPlanningTests(unittest.TestCase):
    def test_falls_back_to_farther_inward_grasp_after_first_offset_fails(self):
        motion={'adaptive_pitch_search':True,'grasp_region':{'along_length_cm':1.5}}
        with patch('approach_grasp_candidate.build_pregrasp_plan',
                   side_effect=[PregraspPathUnavailable('center'),PregraspPathUnavailable('0.8'),
                                PregraspPathUnavailable('0.4'),{'end':{}}]) as planner:
            result=plan_box_pregrasp(None,{},np.array([20.,0.,0.]),np.array([0.,0.,1.]),
                                     {},[0.,0.,0.],motion,[1.,0.,0.])
        self.assertEqual(result['grasp_offset_base_cm'],[-1.2,0.,0.])
        np.testing.assert_allclose([call.args[2][0] for call in planner.call_args_list],
                                   [20.,19.2,19.6,18.8])


class QualityMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene_path=ROOT/'vision_grasp_results/photo_20261106_061204_655266_seq22_abd05758_bbe409f0_commissioning_registered.json'
        if not cls.scene_path.exists():raise unittest.SkipTest('现场图像回放数据不在部署包中')
        cls.scene=json.loads(cls.scene_path.read_text(encoding='utf-8'))
    def setUp(self):
        self.motion=load_motion_profile(ROOT/'grasp_motion_quality_v03.json');self.m=nominal_model()
        s=self.scene;self.start=s['after']['joint_median'];t=s['geometry']['selected'];self.center=np.array(t['top_center_base_cm'])
        c=json.loads(Path(s['calibration']).read_text(encoding='utf-8'))['candidates'][0];self.Y=np.array(c['T_base_board_cm']);self.up=-self.Y[:3,2]
        self.limits={r['id']:(r['min'],r['max']) for r in s['after']['status']['data']['servo_limits']}
        self.offset=np.array(json.loads((ROOT/'calibration_boards/grasp_tcp_hover_candidate_20261105.json').read_text())['offset_tool_cm'])
        self.obj=np.eye(4);self.obj[:3,:3]=np.linalg.inv(self.Y)[:3,:3]@np.column_stack((t['length_axis_base'],t['width_axis_base'],self.Y[:3,2]))
        self.obj[:3,3]=(np.linalg.inv(self.Y)@np.r_[self.center,1])[:3]
    def tcp(self,j):
        T=robot_frame(j,self.m,'pitch_tool');return (np.linalg.inv(self.Y)@np.r_[T[:3,:3]@self.offset+T[:3,3],1])[:3]
    def actuate(self,goal,bias):
        previous=dict(goal)
        actual={j:previous[j]+bias.get(j,0) for j in previous}
        for _ in range(2):
            if max(abs(goal[j]-actual[j]) for j in goal)<=3:break
            previous=compensated_targets(goal,previous,actual,self.limits)
            actual={j:previous[j]+bias.get(j,0) for j in previous}
        return actual
    def replay(self,bias):
        coarse=build_pregrasp_plan(self.m,self.start,self.center,self.up,self.limits,self.offset,self.motion)
        j={**self.start,**self.actuate({k:coarse['end'][k] for k in ('3','4','5','6')},bias)}
        actions=[]
        for i in range(3):
            tcp=self.tcp(j);d=approach_decision(tcp,self.obj,self.motion)
            pitch=self.m.forward({int(k):v for k,v in j.items()})['pitch_deg']
            if d['ready'] and pitch_in_profile(pitch,self.motion):return coarse,actions,tcp,pitch
            if i==2:self.fail('超过两轮末端纠偏')
            plan=choose_step(self.m,j,self.Y[:3,:3]@np.array(d['delta_board_cm']),self.offset,self.limits,-tcp[2],self.up,
                             self.motion['minimum_nominal_pad_height_cm'],self.motion['preferred_nominal_pitch_deg'],self.motion['nominal_pitch_range_deg'])
            actions.append(plan);j={**j,**self.actuate(plan['targets'],bias)}
        self.fail('未到抓取区域')
    def test_nominal_full_path(self):
        coarse,actions,tcp,pitch=self.replay({})
        self.assertLessEqual(len(actions),2);self.assertTrue(in_grasp_region(tcp,self.obj,self.motion['grasp_region']))
        self.assertTrue(pitch_in_profile(pitch,self.motion))
        report={'hardware_commands_sent':False,'physical_quality_validated':False,'coarse_segments':coarse['waypoints'],
            'near_actions':actions,'final_tcp_board_cm':tcp.tolist(),'final_pitch_deg':pitch}
        (ROOT/'vision_grasp_results/quality_v03_complete_nominal_replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    def test_biased_feedback_with_bounded_corrections(self):self.replay({'3':-4,'4':-4,'5':6,'6':-1})
    def test_old_high_edge_grasp_is_rejected(self):
        tcp=np.array([self.obj[0,3],self.obj[1,3],-2.98])
        self.assertFalse(in_grasp_region(tcp,self.obj,self.motion['grasp_region']))
        self.assertFalse(pitch_in_profile(-64.56,self.motion))
    def test_strict_vertical_unreachable_rejected_before_coarse_motion(self):
        strict=copy.deepcopy(self.motion);strict['nominal_pitch_range_deg']=[-90,-90];strict['preferred_nominal_pitch_deg']=-90
        with self.assertRaises(ValueError):build_pregrasp_plan(self.m,self.start,self.center,self.up,self.limits,self.offset,strict)
    def test_farther_object_requires_new_vehicle_pose(self):
        radial=self.center.copy();radial[2]=0;radial/=np.linalg.norm(radial)
        with self.assertRaises(ValueError):build_pregrasp_plan(self.m,self.start,self.center+radial*3,self.up,self.limits,self.offset,self.motion)

    def test_raised_support_can_use_lower_clear_hover(self):
        scenes=list((ROOT/'vision_grasp_results').glob('*bacfda85*commissioning_registered.json'))
        if not scenes:self.skipTest('缺少抬高台面的回放基准')
        scene=json.loads(scenes[0].read_text(encoding='utf-8'))
        geometry=scene['geometry']
        up=-np.array(geometry['table_down_normal_base'])
        center=np.array(geometry['selected']['top_center_base_cm'])
        feedback=scene['after']
        limits={r['id']:(r['min'],r['max']) for r in feedback['status']['data']['servo_limits']}
        motion=load_motion_profile(ROOT/'grasp_motion_fixed_placement_trial.json')
        for rise,split in ((4,False),(8,True)):
            with self.subTest(rise=rise):
                result=build_pregrasp_plan(self.m,feedback['joint_median'],center+rise*up,up,limits,self.offset,motion)
                self.assertEqual(result['height_above_support_cm'],5.5)
                self.assertEqual(result['split_descent_precheck'],split)
                self.assertGreaterEqual(result['margin'],motion['pregrasp_min_top_clearance_cm'])

if __name__=='__main__':unittest.main()
