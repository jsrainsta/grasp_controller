"""Temporal tie breaking uses only image-supported, near-best candidates."""
import unittest
import numpy as np
from box_rim_geometry import refine_grounded_rim


class TwoRims:
    def __init__(self, second_score=2.067):
        self.second_score = second_score

    def measure(self, pose):
        if not np.allclose(pose[:3,:3], np.eye(3)) or abs(pose[1,3]) > 1e-6:
            return None
        x = pose[0,3]
        if abs(x + .4) < 1e-6:
            score = 2.064
        elif abs(x - .2) < 1e-6:
            score = self.second_score
        else:
            return None
        return {'coverage': [1., 1., 1., 1.], 'distances': [score] * 4,
                'corners_pixel': np.array([[x,0.]])}


class ContinuityTests(unittest.TestCase):
    def test_mask_can_select_near_tie_but_cannot_rescue_weak_edges(self):
        def positive(corners):
            return corners[0,0]>0
        pose,_=refine_grounded_rim(TwoRims(),np.eye(4),accept_face=positive)
        self.assertAlmostEqual(pose[0,3],.2)
        self.assertIsNone(refine_grounded_rim(TwoRims(second_score=3.),np.eye(4),accept_face=positive))
        self.assertIsNone(refine_grounded_rim(TwoRims(),np.eye(4),accept_face=lambda corners:False))

    def test_without_prior_ambiguity_remains(self):
        self.assertIsNone(refine_grounded_rim(TwoRims(), np.eye(4)))

    def test_prior_resolves_close_scores(self):
        prior = np.eye(4)
        prior[0,3] = .19
        pose, rim = refine_grounded_rim(TwoRims(), np.eye(4), prior_pose=prior)
        self.assertAlmostEqual(pose[0,3], .2)
        self.assertEqual(rim['candidate_selection'], 'previous_confirmed_pose_tiebreak')

    def test_prior_cannot_replace_clearly_better_image_match(self):
        prior = np.eye(4)
        prior[0,3] = .2
        pose, _ = refine_grounded_rim(TwoRims(second_score=3.), np.eye(4), prior_pose=prior)
        self.assertAlmostEqual(pose[0,3], -.4)


if __name__ == '__main__':
    unittest.main()
