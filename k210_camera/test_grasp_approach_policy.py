"""Regression for the observed vertical oscillation. No hardware access."""
import unittest
import numpy as np
from grasp_approach_policy import approach_decision,in_grasp_region

class PolicyTests(unittest.TestCase):
    def scene(self,height,along=1.6,across=.3):
        obj=np.eye(4);obj[2,3]=-2.5
        return np.array([-along,-across,-height]),obj
    def test_reported_raise_does_not_reverse_to_descent_halfway(self):
        for height in (2.743,3.65,2.72,3.624):
            r=approach_decision(*self.scene(height))
            self.assertEqual(r['phase'],'raise_to_alignment_clearance')
            self.assertLess(r['delta_board_cm'][2],0)
    def test_horizontal_error_is_corrected_before_more_descent(self):
        r=approach_decision(*self.scene(4.3))
        self.assertEqual(r['phase'],'horizontal_alignment_before_descent')
        self.assertEqual(r['delta_board_cm'][2],0)
        self.assertLessEqual(np.linalg.norm(r['delta_board_cm'][:2]),.600001)
    def test_centered_pose_can_descend(self):
        r=approach_decision(*self.scene(4.3,.4,.1))
        self.assertEqual(r['phase'],'final_fine_descent');self.assertGreater(r['delta_board_cm'][2],0)
    def test_same_box_middle_band_and_width_limit(self):
        self.assertTrue(approach_decision(*self.scene(2.59,1.05,.26))['ready'])
        self.assertTrue(approach_decision(*self.scene(2.59,1.33,.26))['ready'])
        self.assertFalse(approach_decision(*self.scene(2.59,1.6,.26))['ready'])
        self.assertFalse(approach_decision(*self.scene(2.59,.5,.5))['ready'])
    def test_far_phase_can_request_larger_step(self):
        r=approach_decision(*self.scene(7.0,.4,.1))
        self.assertEqual(r['phase'],'coarse_descent_to_clearance')
        self.assertEqual(r['delta_board_cm'][2],2.5)
    def test_offset_does_not_narrow_the_accepted_contact_band(self):
        tcp,obj=self.scene(2.59,along=-.5,across=.1)
        motion={'version':'adaptive','grasp_region':{
            'along_length_cm':1.5,'across_width_cm':.45,
            'tcp_height_range_cm':[2.2,3.0],'tcp_height_target_cm':2.6}}
        result=approach_decision(tcp,obj,motion,grasp_offset_board=[1.2,0.,0.])
        self.assertTrue(result['ready'])

    def test_offset_width_correction_keeps_usable_longitudinal_position(self):
        tcp,obj=self.scene(4.,along=-.5,across=.6)
        motion={'version':'adaptive','grasp_region':{
            'along_length_cm':1.5,'across_width_cm':.45,
            'tcp_height_range_cm':[2.2,3.0],'tcp_height_target_cm':2.6},
            'adaptive_pitch_search':True,'maximum_near_step_cm':3.2}
        result=approach_decision(tcp,obj,motion,grasp_offset_board=[1.2,0.,0.])
        self.assertEqual(result['phase'],'horizontal_alignment_at_clearance')
        np.testing.assert_allclose(result['delta_board_cm'],[0.,.6,0.])

    def test_shifted_contact_is_shared_by_approach_and_closure(self):
        region={'along_length_cm':1.5,'across_width_cm':.45,
                'tcp_height_range_cm':[1.5,2.25],'tcp_height_target_cm':2.0}
        motion={'version':'adaptive','grasp_region':region,'adaptive_pitch_search':True,
                'maximum_near_step_cm':3.2}
        for height,along,across,accepted in ((1.85,1.64,.31,True),(1.43,1.64,.31,False),
                                              (1.85,1.64,.5,False),(1.85,2.8,.31,False)):
            tcp,obj=self.scene(height,along,across)
            kwargs={'grasp_offset_board':[-1.2,0.,0.]}
            self.assertEqual(approach_decision(tcp,obj,motion,**kwargs)['ready'],accepted)
            self.assertEqual(in_grasp_region(tcp,obj,region,**kwargs),accepted)

if __name__=='__main__':unittest.main()
