# 操作者画面

安装 `sim`、`web` 依赖及支持 NVENC 的 FFmpeg，填写站点配置。选择空闲或获授权的 GPU 后运行：

```bash
source scripts/site_settings.sh
python scripts/run_workstation.py --source udp --backend sapien --duration 60 \
  --operator-view configs/render/operator_view.yaml \
  --operator-run "$GAMEPAD_VLA_WORKSPACE/runs/operator-view"
```

浏览器地址和监听端口来自站点配置中的 `operator`。默认仅监听回环地址，远程访问使用既有 SSH 转发。`udp` 配置决定输入地址、端口和允许的源地址；命令行参数可以覆盖配置。

运行目录默认只保存启动文件、固定容量 mmap 与每次启动最多 1 MiB 的子进程输出，不写逐帧日志或录屏。诊断基准显式在上述命令增加 `--diagnostics`（采集 CLI `run_collection.py` 同样支持）；仅对本次启动生效，不使用 YAML/站点配置开启。每个诊断文件最多 900 秒、120000 条/视频单元或 32 MiB，达到任一上限后停止。默认网页统计使用内存滑动窗口。运行目录不应直接提交。操作者虚拟视角与标定相机各有定义；画中画读取所选场景的 `camera_3`。

采集服务也使用这条管线：实际完成状态 → teleop latest-state mmap → 独立 SAPIEN 孪生渲染 → NVENC H.264 → WebCodecs。SAPIEN 和 MuJoCo 物理均发布同一格式的双手双臂观测，画面统一由已有孪生渲染提供；没有独立的采集 MJPEG 场景。默认主画面为标定 `camera_3`，网页可切换已有操作者虚拟视角，实际信号 overlay 随帧显示。采集画面保持标定的 1280×720、目标 60 fps。

手柄固定为 v0.4.1；新场景使用公开 `robotwin.create_scene`，默认光栅显示 ABXY。未匹配照片材质的新增 fixture 使用中性灰。默认场景仍为 v4.4，临时抬高场景由 `--spec` 显式选择。

`scripts/summarize_operator_view.py`、`scripts/glove_link/summarize_replay.py`、`scripts/extract_operator_evidence.py` 只读取显式诊断产生的文件。基准启动必须加 `--diagnostics`，录屏可能先达到字节上限，报告的窗口必须落在实际捕获范围内。
