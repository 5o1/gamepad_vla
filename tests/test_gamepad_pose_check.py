"""Metrology conventions, rejected images and capture ownership (no hardware)."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import numpy as np

VISION = all(importlib.util.find_spec(name) for name in ('cv2', 'scipy'))


@unittest.skipUnless(VISION, 'vision checks run in isolated vision / station environment')
class PoseCheckTests(unittest.TestCase):
    def setUp(self):
        from gamepad_vla.workstation.gamepad_pose_check import DEFAULT_SPEC, load_hulls
        self.scene = json.loads(DEFAULT_SPEC.read_text())
        candidate = DEFAULT_SPEC.parents[2].parent/'xbox_gamepad_physics/build/resources/robotwin/full_controls/mobility.urdf'
        self.hulls = load_hulls(candidate if candidate.exists() else None)
        c = self.scene['cameras'][0]['intrinsics']
        self.meta = dict(width=c['width'],height=c['height'],
                         K=[c['fx'],0,c['cx'],0,c['fy'],c['cy'],0,0,1],D=[0.]*5)

    def test_axes_and_target_are_not_rounded(self):
        from gamepad_vla.workstation.gamepad_pose_check import components, perturbed_pose, pose_matrix
        target = pose_matrix(self.scene['gamepad']['root_pose'])
        actual = perturbed_pose(target, [3,-2,1,2,0,0])
        c = components(actual, target, self.scene)
        self.assertAlmostEqual(c['errors']['lateral_offset_mm'],3)
        self.assertAlmostEqual(c['errors']['distance_inside_table_edge_mm'],2)
        self.assertAlmostEqual(c['errors']['height_mm'],1)
        self.assertAlmostEqual(c['errors']['pitch_deg'],2)
        self.assertAlmostEqual(c['tilt_deg'],41.7560211437,places=6)
        self.assertAlmostEqual(c['position_error_mm'],np.sqrt(14))

    def test_green_and_uniform_board_cannot_become_a_pose_prior(self):
        from gamepad_vla.workstation.gamepad_pose_check import check_pose
        for color in ([60,145,65],[170,175,175]):
            image = np.full((720,1280,3),color,np.uint8)
            result, overlay = check_pose(image,self.meta,self.scene,self.hulls)
            self.assertFalse(result['detected'])
            self.assertEqual(result['status'],'not_detected')
            self.assertIsNone(result['estimated_pose'])
            self.assertIsNone(result['acceptance'])
            self.assertEqual(overlay.shape,image.shape)

    def test_zip_and_resolution_contract(self):
        import cv2
        from gamepad_vla.workstation.gamepad_pose_check import load_snapshot
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'snapshot.zip'
            png = cv2.imencode('.png',np.zeros((720,1280,3),np.uint8))[1].tobytes()
            with zipfile.ZipFile(path,'w') as archive:
                archive.writestr('image.png',png)
                archive.writestr('camera.json',json.dumps(self.meta))
            self.assertEqual(load_snapshot(path)[0].shape,(720,1280,3))
            self.meta['width']=640
            with zipfile.ZipFile(path,'w') as archive:
                archive.writestr('image.png',png)
                archive.writestr('camera.json',json.dumps(self.meta))
            with self.assertRaisesRegex(ValueError,'crop or resize'):
                load_snapshot(path)

    def test_reference_needs_detection(self):
        from gamepad_vla.workstation.gamepad_pose_check import check_pose, SilhouetteModel
        image = np.full((720,1280,3),[60,145,65],np.uint8)
        model = SilhouetteModel(self.hulls,self.scene,np.asarray(self.meta['K']).reshape(3,3),np.zeros(5),image.shape)
        x0,y0,x1,y1=model.roi
        image[y0:y1,x0:x1][model.silhouette(model.target).astype(bool)] = [175,180,182]
        result,_=check_pose(image,self.meta,self.scene,self.hulls)
        self.assertTrue(result['detected'])
        self.assertFalse(result['uncertainty']['calibration_known'])
        self.assertTrue(all(c['verdict']=='indeterminate' for c in result['acceptance'].values()))
        with self.assertRaisesRegex(ValueError,'no detected gamepad'):
            check_pose(image,self.meta,self.scene,self.hulls,reference={'detected':False})


class CaptureOwnershipTests(unittest.TestCase):
    def test_conflict_never_launches_camera(self):
        from gamepad_vla.workstation.gamepad_pose_capture import capture_snapshot
        with tempfile.TemporaryDirectory() as directory:
            with patch('gamepad_vla.workstation.gamepad_pose_capture.site_value',return_value='camera_3'), \
                 patch('gamepad_vla.workstation.gamepad_pose_capture.camera_owners',return_value=[{'pid':123}]), \
                 patch('gamepad_vla.workstation.gamepad_pose_capture.subprocess.Popen') as launch:
                with self.assertRaisesRegex(RuntimeError,'occupied'):
                    capture_snapshot(Path(directory)/'capture.zip')
                launch.assert_not_called()

    def test_burst_uses_one_owned_camera_and_always_stops(self):
        from unittest.mock import MagicMock
        from gamepad_vla.workstation.gamepad_pose_capture import capture_snapshot
        opener=MagicMock();opener.open.return_value.__enter__.return_value.read.return_value=b'zip-data'
        child=MagicMock();child.poll.return_value=None
        with tempfile.TemporaryDirectory() as directory:
            with patch('gamepad_vla.workstation.gamepad_pose_capture.site_value',return_value='camera_3'), \
                 patch('gamepad_vla.workstation.gamepad_pose_capture.camera_owners',return_value=[]), \
                 patch('gamepad_vla.workstation.gamepad_pose_capture.free_port',return_value=1234), \
                 patch('gamepad_vla.workstation.gamepad_pose_capture.build_opener',return_value=opener), \
                 patch('gamepad_vla.workstation.gamepad_pose_capture.subprocess.Popen',return_value=child) as launch, \
                 patch('gamepad_vla.workstation.gamepad_pose_capture.time.sleep'):
                paths=capture_snapshot(Path(directory)/'burst.zip',count=3)
                self.assertEqual(len(paths),3)
                self.assertTrue(all(p.read_bytes()==b'zip-data' for p in paths))
                launch.assert_called_once();child.wait.assert_called_once()
                self.assertTrue(any(call.args[0].full_url.endswith('/stop') for call in opener.open.call_args_list))


if __name__=='__main__':
    unittest.main()
