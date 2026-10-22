"""Single-image extrinsics for camera_3 with an upright ChArUco board at the far table edge.

Table frame (2026-10-20 convention): origin back-left table corner, X back->front,
Y left->right (= project world +y), Z up; mm. Board frame (board_spec.json):
origin = top-left outer pattern corner, x right / y down when facing the print,
z into the board.
"""
import json, sys
import numpy as np
import cv2

img_path, meta_path, out_path = sys.argv[1:4]
meta = json.load(open(meta_path))
K = np.array(meta["K"], float).reshape(3, 3); D = np.array(meta["D"], float)
img = cv2.imread(img_path)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard((6, 5), 50.0, 39.0, d)
board.setLegacyPattern(False)
det = cv2.aruco.CharucoDetector(board)
ch_corners, ch_ids, mk_corners, mk_ids = det.detectBoard(gray)
n = 0 if ch_ids is None else len(ch_ids)
print("markers", 0 if mk_ids is None else len(mk_ids), "charuco corners", n)
obj_all = board.getChessboardCorners()          # (20,3) mm, board frame
obj = obj_all[ch_ids.ravel()].astype(np.float64)
imgp = ch_corners.reshape(-1, 2).astype(np.float64)
ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
rvec, tvec = cv2.solvePnPRefineLM(obj, imgp, K, D, rvec, tvec)
proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
err = np.linalg.norm(proj.reshape(-1, 2) - imgp, axis=1)
R_cb, _ = cv2.Rodrigues(rvec)
T_cam_board = np.eye(4); T_cam_board[:3, :3] = R_cb; T_cam_board[:3, 3] = tvec.ravel()

# Board pose in the table frame from the user's measurements.
x_print = 625.0 - 23.0                       # printed surface, mm from back edge
paper_left_y, paper_top_z = 625.0 + 210.0, 297.0   # viewer-left paper edge (larger Y), A3 landscape, bottom on table
origin = np.array([x_print, paper_left_y - 25.0, paper_top_z - 20.0])
R_tb = np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)  # board x,y,z in table frame
T_table_board = np.eye(4); T_table_board[:3, :3] = R_tb; T_table_board[:3, 3] = origin
T_table_cam = T_table_board @ np.linalg.inv(T_cam_board)

offset_world = np.array([0.093415, 0.0002, 0.767])   # table back-left corner in project world (v4.x)
T_world_cam = T_table_cam.copy(); T_world_cam[:3, 3] = T_table_cam[:3, 3] / 1000.0 + offset_world


def quat_wxyz(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [float(w), float(x), float(y), float(z)]


axis_world = T_world_cam[:3, :3] @ np.array([0, 0, 1.0])
result = dict(
    image=img_path.split("/")[-1], corners=int(n),
    reprojection_px=dict(median=float(np.median(err)), p95=float(np.percentile(err, 95)), max=float(err.max())),
    board_distance_mm=float(np.linalg.norm(tvec)),
    assumptions=dict(print_surface_x_mm=x_print, paper_centered_y_mm=625.0, paper_bottom_z_mm=0.0,
                     paper="A3 landscape 420x297, print upright, facing the camera (-X)"),
    camera_in_table_mm=T_table_cam[:3, 3].round(1).tolist(),
    T_world_camera_optical=T_world_cam.tolist(),
    pose_optical_world=[*T_world_cam[:3, 3].round(6).tolist(), *[round(v, 9) for v in quat_wxyz(T_world_cam[:3, :3])]],
    optical_axis_world=axis_world.round(4).tolist(),
)
json.dump(result, open(out_path, "w"), indent=2)
print(json.dumps({k: result[k] for k in ("corners", "reprojection_px", "board_distance_mm", "camera_in_table_mm", "optical_axis_world", "pose_optical_world")}, indent=1))
vis = img.copy()
for p, q in zip(imgp, proj.reshape(-1, 2)):
    cv2.circle(vis, tuple(int(v) for v in p), 6, (0, 255, 0), 2)
    cv2.drawMarker(vis, tuple(int(v) for v in q), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
cv2.drawFrameAxes(vis, K, D, rvec, tvec, 100)
cv2.imwrite(out_path.replace(".json", "_overlay.jpg"), vis)
