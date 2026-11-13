import unittest
from unittest.mock import Mock
from unittest.mock import patch

import numpy as np

from sample_grasp_handeye import nominal_model
from wrist_roll import move_empty_wrist, roll_transform, validate_wrist_roll, wrist_target
from approach_grasp_candidate import build_pregrasp_plan


class WristRollTests(unittest.TestCase):
    def setUp(self):
        self.cal = {
            'field_validated': True, 'degrees_per_count': .25,
            'position_range': [396,596], 'pivot_tool_cm': [0,0,0],
            'minimum_sweep_tcp_height_cm': 4, 'sweep_pitch_range_deg': [-90,-60],
            'camera_rotates_with_wrist': False,
        }
        self.joints = {'1':381,'2':496,'3':172,'4':324,'5':668,'6':575}

    def state(self, wrist):
        return {
            'joint_median': {**self.joints, '2':wrist},
            'joint_span': dict.fromkeys(self.joints,0),
            'joint_mad': dict.fromkeys(self.joints,0),
            'status': {'data': {'capabilities': {'writes_enabled': True},
                                'servo_limits': [{'id':1,'min':178,'max':700},{'id':2,'min':125,'max':875}]}},
        }

    def move(self, height, reader, sender, log):
        return move_empty_wrist('mock',564,self.state(496),height,np.array([0,0,1]),
                                nominal_model(),[0,0,0],self.cal,{},reader,sender,log,lambda:None)

    def test_signed_mapping_uses_measured_direction(self):
        theta = np.deg2rad(17)
        tool = np.array([[0,0,-1],[0,1,0],[1,0,0]])
        width = tool@np.array([0,np.cos(theta),np.sin(theta)])
        normal = np.array([0,0,1])
        validate_wrist_roll(self.cal)
        for scale,expected in ((.25,564),(-.25,428)):
            self.cal['degrees_per_count'] = scale
            target = wrist_target(self.cal,tool,width,normal)
            self.assertEqual(target,expected)
            np.testing.assert_allclose(tool@roll_transform(self.cal,target)[:3,1],width)

    def test_only_servo_two_moves_in_bounded_segments(self):
        reader = Mock(side_effect=[self.state(496),self.state(530),self.state(564)])
        sender = Mock(return_value={'task':{'status':'reached'}})
        log = []
        self.assertEqual(self.move(6,reader,sender,log)['joint_median']['2'],564)
        self.assertEqual([row['request']['targets'] for row in log],
                         [[{'id':2,'position':530}],[{'id':2,'position':564}]])

    def test_low_clearance_sends_no_motion(self):
        sender = Mock()
        with self.assertRaisesRegex(ValueError,'留空'):
            self.move(3,Mock(return_value=self.state(496)),sender,[])
        sender.assert_not_called()

    def test_probe_opens_jaw_in_same_request_as_first_wrist_step(self):
        opened=self.state(530)
        opened['joint_median']['1']=372
        finished=self.state(564)
        finished['joint_median']['1']=372
        reader=Mock(side_effect=[self.state(496),opened,finished])
        sender=Mock(return_value={'task':{'status':'reached'}})
        log=[]
        move_empty_wrist('mock',564,self.state(496),6,np.array([0,0,1]),
                         nominal_model(),[0,0,0],self.cal,{'open_jaw_minimum':370},
                         reader,sender,log,lambda:None,open_target=372)
        self.assertEqual(log[0]['request']['targets'],
                         [{'id':2,'position':530},{'id':1,'position':372}])
        self.assertEqual(log[1]['request']['targets'],[{'id':2,'position':564}])

    def test_unconfirmed_step_does_not_send_next_step(self):
        sender = Mock(return_value={'task':{'status':'outcome_unknown'}})
        with self.assertRaisesRegex(RuntimeError,'未确认'):
            self.move(6,Mock(return_value=self.state(496)),sender,[])
        self.assertEqual(sender.call_count,1)

    def test_probe_search_uses_lower_hover_and_filters_each_candidate(self):
        rejected={**self.joints,'3':180}
        accepted={**self.joints,'3':190}

        def candidates(*args):
            height=args[7]
            if height==6.0:
                return np.zeros(3),[(0,0,rejected,3.1),(1,0,accepted,3.1)]
            return np.zeros(3),[]

        def sweep_check(joints):
            if joints==rejected:
                raise ValueError('留空不足')

        profile={'pregrasp_height_cm':7.5,'adaptive_pitch_search':True,
                 'pregrasp_min_top_clearance_cm':3,'maximum_coarse_joint_delta':60,
                 'whole_path_precheck':False}
        with patch('approach_grasp_candidate.pregrasp_candidates',side_effect=candidates):
            plan=build_pregrasp_plan(None,self.joints,np.zeros(3),np.array([0,0,1]),{},
                                     np.zeros(3),profile,probe_check=sweep_check)
        self.assertEqual(plan['height_above_support_cm'],6.0)
        self.assertEqual(plan['end'],accepted)
