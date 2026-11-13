"""Perspective normalization regressions; no hardware access."""
import json,unittest
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
from check_grasp_board import ROOT
from track_box_reference import track
from grasp_visual_observer import GraspVisualObserver

class RectifiedTrackingTests(unittest.TestCase):
    def test_blank_image_cannot_become_evidence(self):
        im=np.full((480,640),255,np.uint8);q=[[100,100],[500,100],[500,260],[100,260]]
        result=track(im,im,q,current_face_hint=q,coverage_basis='common_visible')
        self.assertNotEqual(result['status'],'tracked_face_for_review')
    def test_known_projective_change_recovers_real_pixel_features(self):
        im=np.full((480,640),210,np.uint8);q=np.float32([[90,100],[510,100],[510,236],[90,236]])
        cv2.rectangle(im,(90,100),(510,236),245,-1)
        rng=np.random.default_rng(17)
        for row in range(4):
            text=''.join(rng.choice(list('ABCDEFGHJKLMNPRSTUVWXYZ23456789'),18))
            cv2.putText(im,text,(105,125+row*28),cv2.FONT_HERSHEY_SIMPLEX,.55,30,1,cv2.LINE_AA)
        hint=np.float32([[120,85],[535,115],[500,278],[105,235]])
        H=cv2.getPerspectiveTransform(q,hint);cur=cv2.warpPerspective(im,H,(640,480))
        result=track(im,cur,q,current_face_hint=hint,coverage_basis='common_visible')
        self.assertEqual(result['status'],'tracked_face_for_review')
        self.assertLess(result['median_inlier_error_px'],1.5)
    def test_saved_partial_view_has_valid_board_and_box_support(self):
        p=ROOT/'vision_grasp_results/hover_20261106_151836_739073.json'
        if not p.exists():self.skipTest('现场图像未包含在部署包')
        h=json.loads(p.read_text(encoding='utf-8'));v=GraspVisualObserver(Path(h['source']),ROOT/'calibration_boards/grasp_tcp_hover_candidate_20261105.json')
        c=h['captures'][-1];r=v.estimate(Path(c['image']),c['after']['joint_median'])
        self.assertGreaterEqual(r['tracking']['inlier_count'],20)
        self.assertLessEqual(r['final_board_fit_rmse_px'],1.0)
        if r['pose_estimator']=='independent_current_marker_camera_and_tracked_plane':
            self.assertLessEqual(r['current_view_shape_validation']['independent_marker_rmse_px'],.6)
        else:
            self.assertEqual(r['pose_estimator'],'two_plane_refinement')
            self.assertLessEqual(r['object_rmse_px'],3.0)

    def test_texture_pose_uses_its_own_matches_in_test40(self):
        journal=ROOT/'vision_grasp_results/visual_approach_20261108_220433_eb4dff.json'
        if not journal.exists():self.skipTest('现场图像未包含在部署包')
        data=json.loads(journal.read_text(encoding='utf-8'))
        with patch.object(Path,'glob',return_value=iter(())):
            observer=GraspVisualObserver(Path(data['source']),Path(data['tcp_profile']))
        for packet in data['observations']:
            if packet['integrity']['valid']:
                visual=observer.estimate(Path(packet['capture']['image_path']),packet['after']['joint_median'])
        self.assertEqual(visual['board']['pose_support'],'fixed_board_texture')
        self.assertLessEqual(visual['final_board_fit_rmse_px'],1.0)
        self.assertTrue(1.5<=-visual['tcp_board_cm'][2]<=2.1)

if __name__=='__main__':unittest.main()
