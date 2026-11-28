"""Regression checks for selecting one detected package and applying its height."""
import unittest
from unittest.mock import patch

from grasp_medicine import medicine, motion_for_box
from observe_grasp_target import detection_face_filter, select_detection


class MedicineDetectionTests(unittest.TestCase):
    @patch('observe_grasp_target.image_detections')
    def test_requested_class_is_selected_and_duplicate_is_rejected(self, infer):
        requested=medicine('红霉素软膏')
        red={'class_id':1,'confidence':.8,'xyxy':[10,10,110,50]}
        other={'class_id':0,'confidence':.95,'xyxy':[110,10,210,50]}
        infer.return_value=[other,red]
        detection=select_detection('photo.jpg',requested)
        accepts=detection_face_filter(detection)
        self.assertTrue(accepts([[12,15],[115,15],[115,43],[12,43]]))
        self.assertFalse(accepts([[120,15],[190,15],[190,43],[120,43]]))
        infer.return_value=[other,red,{**red,'xyxy':[20,70,120,110]}]
        self.assertEqual(select_detection('photo.jpg',requested)['status'],'ambiguous')

    def test_shorter_box_shifts_contact_while_respecting_floor(self):
        target=medicine('丁酸氢化可的松乳膏')
        self.assertEqual(target['dimensions_cm'],(12.,3.3,2.4))
        motion={'minimum_nominal_pad_height_cm':1.,
                'grasp_region':{'tcp_height_target_cm':2.,'tcp_height_range_cm':[1.,2.25]}}
        obs={'geometry':{'assumed_box_dimensions_cm':target['dimensions_cm']}}
        tuned=motion_for_box(motion,obs)
        self.assertEqual(tuned['grasp_region']['tcp_height_target_cm'],1.9)
        self.assertEqual(tuned['grasp_region']['tcp_height_range_cm'],[1.,2.15])
        self.assertEqual(motion['grasp_region']['tcp_height_target_cm'],2.)
