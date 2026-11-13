"""Board-pose outlier regressions without camera or arm access."""
import unittest
import numpy as np
import cv2
from register_current_scene import robust_board_pose_candidates

class BoardPoseTests(unittest.TestCase):
    def setUp(self):
        self.K=np.array([[600.,0,320],[0,600,240],[0,0,1.]])
        self.D=np.zeros(5);ids=np.arange(16)
        self.obj=np.array([[(int(i)%6+1)*1.44,(int(i)//6+1)*1.44,0] for i in ids],float)
        rv=np.array([.2,.1,.03]);tv=np.array([-4.,-3.,30.])
        self.image=cv2.projectPoints(self.obj,rv,tv,self.K,self.D)[0].reshape(-1,2)
        self.board={'profile':{'square_length':1.44,'squares_x':7},'corner_ids':ids.tolist(),'corners_px':self.image.tolist()}
    def test_one_bad_point_is_not_reintroduced_into_pose(self):
        points=self.image.copy();points[9]+=[8.,-7.];self.board['corners_px']=points.tolist()
        solutions,meta=robust_board_pose_candidates(self.board,self.K,self.D)
        self.assertEqual(meta['rejected_corner_ids'],[9]);self.assertEqual(meta['pose_inlier_count'],15)
        self.assertLess(solutions[0][1],.001)
    def test_excessive_outliers_rejected(self):
        points=self.image.copy();points[5]+=[15.,-12.];points[9]+=[-14.,10.];self.board['corners_px']=points.tolist()
        with self.assertRaises(ValueError):robust_board_pose_candidates(self.board,self.K,self.D)
    def test_clean_points_preserved(self):
        solutions,meta=robust_board_pose_candidates(self.board,self.K,self.D)
        self.assertEqual(meta['pose_inlier_count'],16);self.assertEqual(meta['rejected_corner_ids'],[])
        self.assertLess(solutions[0][1],.001)

if __name__=='__main__':unittest.main()
