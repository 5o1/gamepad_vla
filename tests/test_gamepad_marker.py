"""Board identity, optical-frame invariance, and temporary sticker contracts."""
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

VISION=all(importlib.util.find_spec(n) for n in ['cv2','scipy'])


@unittest.skipUnless(VISION,'isolated vision dependencies required')
class MarkerTests(unittest.TestCase):
    def setUp(self):
        from gamepad_vla.workstation.gamepad_marker import DEFAULT_MARKER
        self.config=json.loads(DEFAULT_MARKER.read_text())

    def test_id_spaces_and_print_size(self):
        from gamepad_vla.workstation.gamepad_marker import make_board
        board=make_board(self.config)
        self.assertNotIn(self.config['sticker']['id'],board.getIds())
        self.assertEqual(len(board.getChessboardCorners()),35)
        self.assertAlmostEqual(self.config['sticker']['size_m'],.015)
        self.assertTrue(self.config['sticker']['remove_before_production_collection'])
        self.assertFalse(self.config['sticker']['all_hand_states_clear'])

    def test_charuco_detection_is_not_the_old_calibration_board(self):
        import cv2
        from gamepad_vla.workstation.gamepad_marker import estimate_board,make_board
        k=np.array([[1000,0,900],[0,1000,700],[0,0,1.]])
        pattern=make_board(self.config).generateImage((1600,1200))
        image=np.full((1400,1800,3),255,np.uint8)
        image[100:1300,100:1700]=cv2.cvtColor(pattern,cv2.COLOR_GRAY2BGR)
        pose,q=estimate_board(image,k,np.zeros(5),self.config)
        self.assertIsNotNone(pose);self.assertEqual(q['charuco_corners'],35)
        source=Path(__file__).resolve().parents[1]/'references/calibration/2026-10-23-ego-upright/position_mid_2.png'
        pose,q=estimate_board(cv2.imread(str(source)),k,np.zeros(5),self.config)
        self.assertIsNone(pose)

    def test_world_extrinsic_exactly_cancels_for_relative_norms(self):
        from scipy.spatial.transform import Rotation
        from gamepad_vla.workstation.gamepad_marker import relative_motion
        mount=np.asarray(self.config['gamepad_from_board'])
        a=np.eye(4);a[:3,3]=[.1,.2,.6]
        b=a.copy();b[:3,3]+=[.003,-.002,.001]
        b[:3,:3]=Rotation.from_euler('x',.2,degrees=True).as_matrix()
        extra=np.eye(4);extra[:3,:3]=Rotation.from_euler('xyz',[8,-3,4],degrees=True).as_matrix();extra[:3,3]=[.04,-.03,.02]
        one=relative_motion(a,b,mount);two=relative_motion(extra@a,extra@b,mount)
        self.assertAlmostEqual(one['position_change_mm'],two['position_change_mm'],places=9)
        self.assertAlmostEqual(one['rotation_change_deg'],two['rotation_change_deg'],places=9)

    def test_planar_backprojection_preserves_unmeasured_normal_constraint(self):
        import cv2
        from scipy.spatial.transform import Rotation
        from gamepad_vla.workstation.gamepad_pose_check import DEFAULT_SPEC,pose_matrix
        from gamepad_vla.workstation.gamepad_marker import sticker_in_plane
        scene=json.loads(DEFAULT_SPEC.read_text());camera=scene['cameras'][0]
        k=np.array([[camera['intrinsics']['fx'],0,camera['intrinsics']['cx']],
                    [0,camera['intrinsics']['fy'],camera['intrinsics']['cy']],[0,0,1.]])
        camera_pad=np.linalg.inv(pose_matrix(camera['pose_optical']))@pose_matrix(scene['gamepad']['root_pose'])
        mount=np.asarray(self.config['gamepad_from_board']);plane=np.asarray(self.config['sticker']['gamepad_from_sticker_synthetic_only']).copy()
        plane[:3,:3]=plane[:3,:3]@np.diag([1.,-1.,-1.])
        offset=np.eye(4);offset[:2,3]=[.005,-.004];offset[:3,:3]=Rotation.from_euler('z',.7,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
        t=camera_pad@plane@offset;h=.0075
        obj=np.array([[-h,-h,0],[h,-h,0],[h,h,0],[-h,h,0]])
        pixels=cv2.projectPoints(obj,cv2.Rodrigues(t[:3,:3])[0],t[:3,3],k,np.zeros(5))[0].reshape(-1,2)
        result=sticker_in_plane(pixels,camera_pad@mount,self.config,k,np.zeros(5))
        self.assertAlmostEqual(result['lateral_mm'],5,places=3)
        self.assertAlmostEqual(result['slide_mm'],-4,places=3)
        self.assertAlmostEqual(result['angle_deg'],.7,places=3)
        self.assertIn('normal translation',result['unmeasured_dof'])

    def test_ten_reclamps_use_pairwise_spread_and_keep_unverified(self):
        from gamepad_vla.workstation.gamepad_marker import summarize_reclamping
        results=[]
        for i in range(10):
            pose=np.eye(4);pose[0,3]=i*.0001
            results.append({'sticker':{'detected':True,'card_from_sticker':pose.tolist()}})
        summary=summarize_reclamping(results)
        self.assertEqual(summary['count'],10)
        self.assertAlmostEqual(summary['maximum_pairwise_distance_mm'],.9)
        self.assertEqual(summary['status'],'complete')
        self.assertEqual(summary['verdict'],'indeterminate')
        results[3]={'sticker':{'detected':False}}
        self.assertEqual(summarize_reclamping(results)['status'],'incomplete')

    def test_same_image_sticker_card_ratio_ignores_world_and_hand_placement(self):
        from scipy.spatial.transform import Rotation
        camera_card=np.eye(4);camera_card[:3,3]=[.1,.05,.6]
        hand_applied=np.eye(4);hand_applied[:3,:3]=Rotation.from_euler('xyz',[2,3,8],degrees=True).as_matrix();hand_applied[:3,3]=[.03,.02,.01]
        slip=np.eye(4);slip[:3,3]=[.0002,-.0001,.0003]
        first=np.linalg.inv(camera_card)@(camera_card@hand_applied)
        second=np.linalg.inv(camera_card)@(camera_card@hand_applied@slip)
        recovered=np.linalg.inv(first)@second
        np.testing.assert_allclose(recovered,slip,atol=1e-12)


if __name__=='__main__':unittest.main()
