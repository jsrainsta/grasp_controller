"""Analytical FOV normalization regression, no hardware or image collection."""
import unittest
import numpy as np
from track_box_reference import common_reference_area

class VisibilityTests(unittest.TestCase):
    def setUp(self):self.q=np.array([[50,50],[150,50],[150,150],[50,150]],np.float32)
    def test_identical_view_keeps_full_face_denominator(self):
        self.assertAlmostEqual(common_reference_area(self.q,np.eye(3),(200,200)),10000)
    def test_partial_crop_uses_only_shared_face(self):
        H=np.array([[1,0,100],[0,1,0],[0,0,1]],float)
        self.assertAlmostEqual(common_reference_area(self.q,H,(200,200)),4900)
    def test_face_outside_current_sensor_has_zero_support(self):
        H=np.array([[1,0,300],[0,1,0],[0,0,1]],float)
        self.assertEqual(common_reference_area(self.q,H,(200,200)),0)
    def test_horizon_crossing_is_rejected(self):
        inverse=np.array([[1,0,0],[0,1,0],[.02,0,-1]],float)
        with self.assertRaises(ValueError):common_reference_area(self.q,np.linalg.inv(inverse),(200,200))

if __name__=='__main__':unittest.main()
