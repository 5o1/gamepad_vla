# 操作者画面

安装 `sim`、`web` 依赖及支持 NVENC 的 FFmpeg，填写站点配置。选择空闲或获授权的 GPU 后运行：

```bash
source scripts/site_settings.sh
python scripts/run_workstation.py --source udp --backend sapien --duration 60 \
  --operator-view configs/render/operator_view.yaml \
  --operator-run "$GAMEPAD_VLA_WORKSPACE/runs/operator-view"
```

浏览器地址和监听端口来自站点配置中的 `operator`。默认仅监听回环地址，远程访问使用既有 SSH 转发。`udp` 配置决定输入地址、端口和允许的源地址；命令行参数可以覆盖配置。

运行目录默认只保存启动文件、固定容量 mmap 与每次启动最多 1 MiB 的子进程输出，不写逐帧日志或录屏。诊断基准显式在上述命令增加 `--diagnostics`（采集 CLI `run_collection.py` 同样支持）；仅对本次启动生效，不使用 YAML/站点配置开启。每个诊断文件最多 900 秒、120000 条/视频单元或 32 MiB，达到任一上限后停止。默认网页统计使用内存滑动窗口。运行目录不应直接提交。独立 `run_workstation.py` 调试入口保留原虚拟视角与 `camera_3` 画中画；采集网页使用下述独立窗格。

采集服务也使用这条管线：实际完成状态 → teleop latest-state mmap → 独立 SAPIEN 孪生渲染 → NVENC H.264 → WebCodecs。SAPIEN 和 MuJoCo 物理均发布同一格式的双手双臂观测，画面统一由已有孪生渲染提供；没有独立的采集 MJPEG 场景。采集主画面固定为标定 `camera_3`，默认原生 1280×720、目标 60 fps。主图中没有画中画；实际信号示意图在单独的可折叠区域。

采集网页工具栏分别开关 Button face、Approach、Back、Left side、Right side 和 Overview。前五个相机随 spec 的手柄完整位姿变换；Overview 随手柄平移并保持世界竖直俯视。默认打开 Button face 和 Overview。关闭的相机不调用 `take_picture`；所有页面需求取并集，最后一个页面关闭后暂停辅助渲染。取消勾选会关闭该窗格，其余窗格按原生尺寸紧贴排列；关闭整个浏览器标签页或切到后台也会释放该页的辅助需求。

六格辅助流由同一个 `operator_view` 的独立渲染副本生成，继续使用 teleop mmap、NVENC 和 WebCodecs。各格原生拍照后直接填入拼接画面，格间无填充、边框或缩放。网页按原生像素显示，较窄窗口可横向滚动；点击任意相机窗格放大，再点或按 Esc 恢复。控制按钮全部位于视频区外，视频窗格紧贴排列。

修改 `configs/render/operator_view.yaml` 后重新启动仿真：

```yaml
collection_main:
  width: 1280
  height: 720
auxiliary:
  width: 1920
  height: 1080
  fps: 20
```

1920×1080 辅助流分为六个 640×540 原生相机；2560×1440 时三列为 853/853/854×720，完全覆盖每行每列。FOV 与格子宽高比匹配，不加黑边。主相机更改原生尺寸时按比例缩放标定 K。计算节点上的组合 fps、编码延迟和推荐默认值见 [AE 报告](../reports/2026-11-05-teleop-feedback/README.md)，采集台恢复后仍需按该报告复测完整采集链。

“Snapshot in 30 s” 按钮保存一份冻结的已完成仿真状态、最近的四路原始指令和七张独立 PNG；所有页面共享倒计时，可在开始拍照前取消。即使窗格关闭，也会单独渲染其相机；截图最少 1280×720，并且不低于主视角原生分辨率。快照不经过拼接、NVENC 或网页，也不写入 episode。配置、文件字段和取回方法见 [采集 API](collection-api.md#场景倒计时快照)。

手柄固定为 v0.4.1；新场景使用公开 `robotwin.create_scene`，默认光栅显示 ABXY。未匹配照片材质的新增 fixture 使用中性灰。默认场景仍为 v4.4，临时抬高场景由 `--spec` 显式选择。

`scripts/summarize_operator_view.py`、`scripts/glove_link/summarize_replay.py`、`scripts/extract_operator_evidence.py` 只读取显式诊断产生的文件。基准启动必须加 `--diagnostics`，录屏可能先达到字节上限，报告的窗口必须落在实际捕获范围内。
