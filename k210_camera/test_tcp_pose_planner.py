"""Boundary/no-widening regression with recorded nominal hardware geometry."""
import unittest
import numpy as np
from sample_grasp_handeye import nominal_model
from tcp_pose_planner import choose_step

class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.model=nominal_model();self.j={'1':381,'2':497,'3':448,'4':124,'5':718,'6':483}
        self.limits={j:(125,875) for j in (3,4,5,6)}
        self.offset=[-.84068807,-1.04867801,-1.92142276]
    def test_one_count_start_excursion_can_plan_inward(self):
        p=choose_step(self.model,self.j,[0,0,1],self.offset,self.limits,2.93,np.array([0,0,1]),1.9)
        self.assertTrue(all(self.limits[int(j)][0]<=v<=self.limits[int(j)][1] for j,v in p['targets'].items()))
        self.assertGreaterEqual(p['joint_limit_margin'],15)
    def test_large_start_excursion_requires_inspection(self):
        with self.assertRaises(ValueError):
            choose_step(self.model,{**self.j,'4':114},[0,0,1],self.offset,self.limits,2.93,np.array([0,0,1]),1.9)

if __name__=='__main__':unittest.main()
