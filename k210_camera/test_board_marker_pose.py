"""Decoded-marker input rejection and saved-frame regression, no hardware."""
import json,unittest
from pathlib import Path
import cv2,numpy as np
from board_marker_pose import marker_plane_candidates
from check_grasp_board import ROOT

class MarkerPoseTests(unittest.TestCase):
    def setUp(self):
        d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board=cv2.aruco.CharucoBoard((7,5),1.44,.96,d)
        p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector=cv2.aruco.ArucoDetector(d,p);self.K=np.array([[600.,0,320],[0,600,240],[0,0,1]]);self.D=np.zeros(5)
    def test_empty_frame_is_not_board_evidence(self):
        poses,meta=marker_plane_candidates(np.full((480,640),255,np.uint8),self.K,self.D,self.board,self.detector)
        self.assertFalse(poses);self.assertEqual(meta['marker_count'],0)
    def test_wrong_board_ids_do_not_supply_pose(self):
        d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);im=np.full((480,640),255,np.uint8)
        for j,(x,y) in enumerate(((40,40),(250,40),(40,250),(250,250))):im[y:y+90,x:x+90]=cv2.aruco.generateImageMarker(d,30+j,90)
        poses,meta=marker_plane_candidates(im,self.K,self.D,self.board,self.detector)
        self.assertFalse(poses);self.assertEqual(meta['marker_count'],0)
    def test_current_failed_frame_can_use_measured_markers(self):
        p=ROOT/'vision_grasp_images/20261106/photo_20261106_070340_953311_seq43_83e59644.jpg'
        if not p.exists():self.skipTest('现场图未包含在部署包')
        k=json.loads((ROOT/'camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json').read_text())
        gray=cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),0)
        poses,meta=marker_plane_candidates(gray,np.array(k['camera_matrix_px']),np.array(k['distortion_coefficients']),self.board,self.detector)
        self.assertEqual(meta['marker_count'],6);self.assertTrue(poses);self.assertLess(poses[0][1],.6)

    def test_three_visible_markers_require_checked_near_mode(self):
        p=ROOT/'vision_grasp_images/20261108/photo_20261108_122609_430953_seq21_468c4ca6.jpg'
        if not p.exists():self.skipTest('现场图未包含在部署包')
        k=json.loads((ROOT/'camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json').read_text(encoding='utf-8'))
        gray=cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),0)
        args=(gray,np.array(k['camera_matrix_px']),np.array(k['distortion_coefficients']),self.board,self.detector)
        self.assertFalse(marker_plane_candidates(*args)[0])
        poses,meta=marker_plane_candidates(*args,allow_three=True)
        self.assertTrue(poses)
        self.assertEqual(meta['marker_count'],3)
        self.assertLessEqual(max(meta['three_marker_cross_check_max_px']),3.5)

if __name__=='__main__':unittest.main()
