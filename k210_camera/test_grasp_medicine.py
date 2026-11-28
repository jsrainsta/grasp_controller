"""Regression checks for selecting one detected package and applying its height."""
import unittest
from unittest.mock import patch

from grasp_medicine import medicine, motion_for_box
import numpy as np
from observe_grasp_target import choose_rim_by_text, detection_face_filter, select_detection


class MedicineDetectionTests(unittest.TestCase):
    def test_ocr_resolves_only_unique_full_name_location(self):
        left=(None,{'corners_pixel':np.array([[0,0],[80,0],[80,60],[0,60]],np.float32)})
        right=(None,{'corners_pixel':np.array([[30,0],[110,0],[110,60],[30,60]],np.float32)})
        name='阿昔洛韦乳膏'
        line={'text':name,'confidence':.9,'polygon':[[75,20],[90,20],[90,30],[75,30]]}
        self.assertIs(choose_rim_by_text([left,right],[line],name)[0],right)
        line['polygon']=[[40,20],[60,20],[60,30],[40,30]]
        self.assertEqual(choose_rim_by_text([left,right],[line],name),(None,1,2))

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
