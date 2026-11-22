"""Exercise instance association and the conservative second-pass tracker without hardware."""
import unittest
from unittest.mock import patch

import numpy as np

from box_segmentation import overview_mask, polygon_mask
from track_box_reference import track


class SegmentationTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((80, 100), np.uint8)
        self.quad = [[20, 20], [60, 20], [60, 55], [20, 55]]
        self.mask = polygon_mask(self.image.shape, self.quad)
        self.ocr = {"lines": [{"text": "阿昔洛韦乳膏", "confidence": .95,
                              "polygon": [[30,30],[40,30],[40,40],[30,40]]}]}

    def test_overview_requires_one_named_instance(self):
        self.assertIs(overview_mask([self.mask], self.mask.shape, self.ocr), self.mask)
        self.assertIsNone(overview_mask([self.mask, self.mask], self.mask.shape, self.ocr))
        self.assertIsNone(overview_mask([self.mask], self.mask.shape, {"lines":[]}))
        self.assertIsNone(overview_mask([], self.mask.shape, self.ocr))

    @patch("track_box_reference._track")
    def test_near_rejects_wrong_or_ambiguous_instances(self, run):
        for masks in ([], [255-self.mask], [self.mask, self.mask]):
            outcome = track(self.image,self.image,self.quad,current_face_hint=self.quad,instance_masks=masks)
            self.assertEqual(outcome["status"], "needs_observation")
        run.assert_not_called()

    @patch("track_box_reference._track")
    def test_keeps_supported_inliers_and_retries_conflicting_inliers(self, run):
        measured={"status":"tracked_face_for_review", "inlier_current_pixel":[[30,30],[40,40]]}
        run.return_value=measured
        kwargs=dict(current_face_hint=self.quad, instance_masks=[self.mask])
        result=track(self.image,self.image,self.quad,**kwargs)
        self.assertEqual(result["segmentation"],"expv4s_confirmed_inliers")
        self.assertEqual(run.call_count,1)
        run.side_effect=[dict(measured,inlier_current_pixel=[[0,0]]),{"status":"needs_observation"}]
        self.assertEqual(track(self.image,self.image,self.quad,**kwargs)["status"],"needs_observation")
        self.assertIs(run.call_args.kwargs["instance"],self.mask)
