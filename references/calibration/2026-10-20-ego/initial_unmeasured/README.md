# 顶相机首次拍摄归档（2026-10-20）

相机：camera_3；原图：1280×720 PNG。三张原图与拍摄时的内参、时间戳原样保留。

用户确认按 A3 PDF 打印，尚未记录每次摆放的孔位到台面边缘距离；打印比例尚未实量核对。**本批次没有生成相机相对台面的联合外参。** 后续带摆放测量的重拍应单独归档。

| 顺序 | 原图 | 拍摄元数据 |
|---|---|---|
| 1 | [PNG](images/position_1_20261020T132504_634113Z.png) | [JSON](images/position_1_20261020T132504_634113Z.json) |
| 2 | [PNG](images/position_1_20261020T132514_650725Z.png) | [JSON](images/position_1_20261020T132514_650725Z.json) |
| 3 | [PNG](images/position_1_20261020T132529_757799Z.png) | [JSON](images/position_1_20261020T132529_757799Z.json) |

`intrinsics.json` 为同一驱动内参的 3×3 K 格式；`board_spec.json` 来自标定板仓库，版本与提交见 `archive.json`。

采集台副本：`~/gamepad_vla_data/calibration/2026-10-20-ego/initial_unmeasured/`。

## 已执行角点检测

使用 `../station_calib_board` 的 `calib_board.detect`，在 计算节点 的现有环境运行。

| 顺序 | 检出角点 | 重投影中位数（px） | P95（px） |
|---|---:|---:|---:|
| 1 | 20 | 0.3270 | 0.6557 |
| 2 | 18 | 0.3856 | 0.7131 |
| 3 | 20 | 0.2676 | 0.6432 |

逐图 JSON 和角点叠图在 `diagnostics/`。其中的 `T_camera_optical_board` 是各次板相对相机的位姿，不能替代相机相对台面的外参；重投影误差也不是物理定位精度。
