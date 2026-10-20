# 操作者画面

安装 `sim`、`web` 依赖及支持 NVENC 的 FFmpeg，填写站点配置。选择空闲或获授权的 GPU 后运行：

```bash
source scripts/site_settings.sh
python scripts/run_workstation.py --source udp --backend sapien --duration 60 \
  --operator-view configs/render/operator_view.yaml \
  --operator-run "$GAMEPAD_VLA_WORKSPACE/runs/operator-view"
```

浏览器地址和监听端口来自站点配置中的 `operator`。默认仅监听回环地址，远程访问使用既有 SSH 转发。`udp` 配置决定输入地址、端口和允许的源地址；命令行参数可以覆盖配置。

运行目录保存进程日志、帧时间戳和录屏，不应直接提交。操作者虚拟视角与标定相机各有定义；画中画读取所选场景的 `camera_3`。
