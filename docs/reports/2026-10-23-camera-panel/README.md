# 任务 Q：采集网站相机面板

已合并预览、标定拍照和 RGB-D 录像入口，删除独立旧工具，未 commit、未重新标定。采集台通过整树同步及哈希比对；验收使用独立端口 18791，原 8780 服务及其仿真没有重启。

## 审阅者裁决

1. **RGB-D 只代理、不移植**（2026-10-23）：按需把站上 `ego_camera_photo_web.py` 作为回环子进程，用系统 Python 和其原有参数启动，转发 start/record/start/record/stop/stop/state。原样返回状态和质量报告，不复制编码、逐帧记录或统计。工具路径来自私有 `camera.ego_web_script`，没有修改 `~/workstation`。
2. **预览与标定迁移后删除旧脚本**：原 serve-camera 的 ROS color 节点、同帧 Image/CameraInfo 配对、2 s 过期检查、JPEG 与 PNG 编码迁至 `camera_service.py`，原始照片格式保留。启动使用系统 Python、ROS Humble、独立域及 loopback。

## 改动与验证

`camera_panel.py` 管理独立相机会话、占用检测、照片和自有进程组，`collection_server.py` 接入现有 cookie / 远程开关与审计。页面模块独立于仿真初始化，包含预览/网格/十字线、标定拍照/缩略图/下载、RGB-D 录像/原始质量报告。录像前释放预览；录像结束先释放上游恢复的 ROS 相机，再按需恢复本项目预览。上游另起 session 的子进程也纳入所有权清理；不结束别人的进程。

- 采集台全量：**183 passed、5 skipped**（4.52 s），无失败。跳过项：缺少 gymnasium、torch、resnake_gym 的可选测试，未开启的真实 ROS 集成测试，以及站点无 Node 的按钮测试。保留既有 aiohttp RequestKey 提示，未为本任务改造整个服务。
- 本地相机测试：**7 passed、1 skipped**（3.36 s）；仅 HTTP 测试因本地无 aiohttp 跳过，它已在站点执行。按钮测试在已有 Node 中实际执行 onclick，核对 POST、路由、JSON 和客户端标识；未新增依赖。
- 新增覆盖：HTTP 代理及权限/审计、旧 PNG/JSON 落盘格式、替身进程占用返回 409 且仍存活、录像原样代理与文件产物、恢复/不恢复预览、独立 session 子进程清理。
- `check_public_tree.py`：0 findings；`git diff --check` 通过。当前使用文档和测试不再引用旧入口。搜索仍可见历史部署 manifest/校验清单及归档照片原始路径，保留这些历史证据，不改写旧校验值或照片来源。

## 采集台冒烟

通过 `sync_tree.py` 部署（376 个文件，逐文件 SHA-256 校验），无头 Chromium 从临时网站按钮执行全过程，不开启远程控制，不启动仿真/手 SDK/臂节点。

| 项目 | 实测 |
|---|---|
| 30 s 预览 | 30.032 s，取得 457 张 JPEG，浏览器轮询 **15.217 fps**（不是独立传感器帧数） |
| 预览参数 | 1280×720 @ 请求 30 fps；窗口结束时配对接收率 14.189 fps、帧龄 0.567 s；恢复后截图帧龄 0.005 s |
| 标定拍照 | PNG 1 张（1,183,828 bytes）及同名 JSON，K 与 2026-10-20 camera_info 逐值相同 |
| RGB-D | 848×480 @ 请求 60 fps，**303 对帧，硬件时间跨度 5.068 s，59.591 fps** |
| 写入 | 303 accepted / 303 written / 0 queue drops，队列峰值 3/120 |
| 产物 | H.264 MP4、gray16le FFV1 MKV、内参/depth_scale JSON、303 行逐帧 jsonl、原始质量报告均已生成 |
| 退出 | 录像后恢复预览；最终停止后相机占用检测为空，临时服务与 RealSense 子进程退出 |
| 原仿真 | 同一 session 持续 running，revision 保持 17，仿真时间 1030.899 → 1084.716 s，远程控制始终关闭 |
| 浏览器 | JavaScript 错误 0；网格、十字线、照片缩略图和录像报告截图已人工查看 |

上游录像质量为 **warning**：深度帧号有 1 次跳帧及 1 次重复，时间戳亦有对应异常；RGB-D 时间差最大 16.810 ms，超过 60 fps 的一帧周期。原样展示这些警告，不把产物齐全写成无警告通过；本任务没有诊断其硬件原因或修改上游统计。

照片：`<data_root>/calibration/20261022-153950/q_smoke_20261022T154022_733540Z.{png,json}`。录像：`<data_root>/camera/20261022-153950/videos/ego_20261022_234024*`。会话/照片名使用站点 UTC，原录像工具使用其原有本地时区命名，未修改系统时钟或上游格式。K 为 `[[906.560791015625,0,647.8956298828125],[0,905.986328125,362.2329406738281],[0,0,1]]`。

截图及原始验收 JSON 只在私有 `.local/review/camera-q/camera-panel.png`、`.local/review/camera-q/hardware-result.json`；没有放进公开报告。可用 `scripts/station/check_camera_panel.py` 复现：默认只检查界面；显式 `--hardware` 才操作 RealSense，始终使用独立服务，不开启远程控制。运行需现场硬件授权和已有的浏览器环境。

## 修复次数与证据限度

每步上限 3 次，本轮记录如下：

1. 网页绑定测试夹具 1/3：首测用 `JS` 替换占位符误伤 `JSON`，改为独立占位符；随后通过。
2. HTTP 审计测试夹具 1/3：全量首测 181 passed / 1 failed / 5 skipped，测试读取被拒绝请求中的空 label；改用 `.get`，随后通过。
3. 生命周期实现 1/3：源码确认上游 ROS/录像另建 session，补齐自有子进程组清理并通过真实替身测试；最终硬件停止也确认无遗留相机进程。

现场设备核对曾把 USB/ASIC 标识与 SDK 序列号不同误当成不同设备。站点配置补齐后，使用 `rs-enumerate-devices -s` 只读确认是同一 ego D435，再执行已授权的相机验收；没有试用另一台设备。

这是界面、相机预览/拍照和录像集成验收，不是重新标定或操作成功验证。旧外参未更新；相机物理位姿改变后的数据应按[标定流程](../../../references/calibration/2026-10-20-ego/README.md)重新求解。原运行服务仍是旧进程，整树部署不会热替换它；已退出的临时实例验证了新页面，常用服务何时切换由现场安排。
