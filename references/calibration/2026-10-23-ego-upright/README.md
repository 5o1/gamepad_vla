# Ego camera re-aim and single-image recalibration (2026-10-23)

**Status: adopted for v4.5 and the collection scene.** The user tilted camera_3 forward so the gamepad near the far table edge stays in view. Its position was not intentionally changed.

## Input

- One 1280×720 colour image, [position_mid.png](position_mid.png), from the collection site's camera panel. Metadata in [position_mid.json](position_mid.json). The serial is replaced with the camera name.
- Board: STATION-CALIB-001 (A3 landscape, ChArUco 6×5, 50 mm squares, 39 mm markers, `DICT_4X4_50`, non-legacy).
- K/D: unchanged driver `camera_info`, identical to the 2026-10-20 archive.
- Board placement, from the user:
  - standing upright at the centre of the far (front) table edge, print facing the camera;
  - held by a back clamp and double-sided tape underneath;
  - the clamp contact is 18 mm inside the edge and the board is 5 mm thick, so the printed surface is at X = 625 − 23 = **602 mm** in the 2026-10-20 table frame (origin at the back-left table corner, X back→front, Y left→right, Z up);
  - the paper is centred on Y = 625 mm with its bottom edge on the table (Z = 0); tape thickness is ignored;
  - the pattern origin is therefore at (602, 810, 277) mm.

## Method

[solve_upright.py](solve_upright.py) detects ChArUco corners with OpenCV 5.0, runs `solvePnP ITERATIVE` and then `solvePnPRefineLM`, and composes the camera pose with the measured board pose. It converts to project world using the same table offset `(0.093415, 0.0002, 0.767) m` as 2026-10-20.

## Result

| Quantity | Value |
|---|---|
| ChArUco corners | 20 / 20 |
| Reprojection median / P95 / max | 0.18 / 0.33 / 0.58 px |
| Camera in table frame | (224.8, 653.5, 691.3) mm |
| Optical axis (world) | (0.728, 0.016, −0.686), about 43° forward of straight down |

Full transform: [result.json](result.json). Overlay: [position_mid_overlay.jpg](position_mid_overlay.jpg).

**Cross-check.** The 2026-10-20 three-placement fit put the camera at (222.7, 681.6, 677.8) mm. The difference, (2, −28, 14) mm, is within that fit's jackknife spread of (13.8, 29.3, 11.9) mm. The two calibrations are independent: board flat versus upright, and a different camera orientation.

**Limits.** This is a single view. Its absolute accuracy depends on the measured board pose, mainly the 602 mm print plane and the assumed centring. A low reprojection error does not show the pose is accurate. Adding left and right upright placements to a joint fit, as on 2026-10-20, would reduce this dependence.

## Second photo after a small camera adjustment (adopted)

The user adjusted the camera slightly, trying to level it, and took one more photo with the board in the same upright centre placement: [position_mid_2.png](position_mid_2.png), metadata in [position_mid_2.json](position_mid_2.json). The same script and assumptions were used.

| Quantity | Value |
|---|---|
| ChArUco corners | 20 / 20 |
| Reprojection median / P95 / max | 0.24 / 0.37 / 0.39 px |
| Camera in table frame | (224.9, 652.0, 691.5) mm, within 1.5 mm of the first photo |
| Optical axis (world) | (0.727, 0.026, −0.687); roll −1.5°, yaw 2.1° |

Full transform: [result_2.json](result_2.json). Overlay: [position_mid_2_overlay.jpg](position_mid_2_overlay.jpg).

The scene camera_3 now uses this pose. The gamepad tilt needed to face the camera is 39.73°, unchanged from the 39.8° fixture design, with a 2.5° residual.
