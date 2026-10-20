# 独立复现

此流程复现当前 v4.4 场景资源、三视角静态渲染，以及已归档照片的标定数值。无需私有站点资料或硬件连接。历史场景、上游游戏训练和实机控制不在此流程内。

## 环境

所有命令从仓库根目录运行。准备 Python 3.10、Git 和可写工作目录；使用 Conda 创建环境或向安装脚本提供现有 Python 3.10。复制 `configs/site.example.json` 为忽略的 `configs/site.local.json`，填写 `workspace`；独立仿真不需要填写站点 IP 或设备序列号。

Ubuntu 的基础工具包括 `git`、`python3.10-venv`、`ca-certificates` 和 `fonts-dejavu-core`；GPU 驱动与 Vulkan ICD 按机器安装。

验证过的仿真组合为 Ubuntu 22.04.5、NVIDIA RTX 4090、驱动 595.99.02、SAPIEN 3.0.3、手柄包 v0.3.1。Python 依赖固定在 `constraints/sapien-py310.txt`。渲染需要可用的 Vulkan 驱动，不能只安装 CUDA。合成图默认使用 DejaVu Sans（Ubuntu 包 `fonts-dejavu-core`）；其他字体可传给 `compose_workcell_multiview.py --font`。

操作者视频另需支持 `h264_nvenc` 的 FFmpeg；验证环境为 FFmpeg 4.4.2。它不是静态截图的依赖。真实相机采集另需 ROS Humble、`cv_bridge`、RealSense ROS；既有采集日志记录 RealSense ROS 4.58.3 / librealsense 2.58.3，这些驱动不参与离线标定复算。

## 1. 获取公开输入

```bash
source scripts/site_settings.sh
PYTHONPATH=src python3 scripts/fetch_resources.py --group all
```

下载器按[资源清单](../references/external_resources.json)获取固定版本的文件与标定 Git 源码，校验后保存到工作目录。已有且同哈希的文件会复用，中断后可重跑；网络故障有界重试，不接受损坏文件或 LFS 指针。

默认直连；直连失败时，可显式传 `--proxy <代理URL>` 或填写私有站点配置的 `network.proxy_url`。不要把代理地址或凭据写进公开文档。`--group simulation` / `--group calibration` 可单独获取对应资源。

## 2. 建立仿真环境并生成资产

```bash
bash scripts/setup_sapien_env.sh --python python3.10
SIM_PY="$GAMEPAD_VLA_WORKSPACE/envs/sapien-3.0.3-py310/bin/python"
"$SIM_PY" scripts/prepare_resources.py
"$SIM_PY" scripts/verify_resources.py
```

省略 `--python` 时使用配置中的 Conda。安装脚本在独立目录创建环境、安装已下载的手柄 wheel，并从发布 ZIP 离线构建资源；已有目标目录会报错，避免覆盖其他环境。可通过 `PIP_INDEX_URL` 选择包镜像。

验收：下载文件全部通过清单校验；左右手规范化 XML 与记录的几何摘要一致；生成 64 个有效视觉分片；`verify_resources.py` 返回 `all_meshes_present=true`、`home_joint_limits_valid=true`。这些检查不启动 GPU，也不构成接触或实机验证。

## 3. 渲染三个视角

先选择空闲或获授权的 GPU，在私有配置中填写 `compute.cuda_visible_devices` 和相应的 `compute.render_device`（PCI 设备地址）；环境变量 `CUDA_VISIBLE_DEVICES`、`GAMEPAD_VLA_RENDER_DEVICE` 可覆盖它们。

```bash
"$SIM_PY" scripts/render_workcell.py configs/render/workcell_multiview_preview.yaml \
  --output "$GAMEPAD_VLA_WORKSPACE/runs/multiview-reproduction"
"$SIM_PY" scripts/compose_workcell_multiview.py \
  "$GAMEPAD_VLA_WORKSPACE/runs/multiview-reproduction"
```

输出为 `camera_3.png`、`side.png`、`opposite.png` 和 `multiview.png`。前三张均为 1280×720；相机位姿和 K 来自场景配置，物理步数为 0。检查 `metrics.json` 的 `passed`、相机参数和设备记录，并与[参考图](reports/2026-10-21-08r-multiview/render/multiview.png)比较。不同驱动的像素值可能不同，不要求 PNG 哈希相同。

## 4. 复算归档标定

使用单独环境，避免将仿真环境的 OpenCV/NumPy 版本混入归档算法。这里安装的是求解运行时，不是上游生成 PDF 的完整依赖集。

```bash
python3.10 -m venv "$GAMEPAD_VLA_WORKSPACE/envs/calibration-py310"
CALIB_PY="$GAMEPAD_VLA_WORKSPACE/envs/calibration-py310/bin/python"
"$CALIB_PY" -m pip install -r constraints/calibration-py310.txt -e .
"$CALIB_PY" scripts/reproduce_calibration.py \
  --output "$GAMEPAD_VLA_WORKSPACE/runs/calibration-reproduction"
```

输出目录必须是新目录。脚本复制归档到输出目录后求解，原始照片、测量和结果不会被覆盖。标定源码固定到清单中的提交；运行时使用 NumPy 1.26.0、OpenCV headless 4.11.0.86、PyYAML 6.0.3。

验收：三张图共 60 个角点，重投影中位数约 2.056 px、P95 约 4.681 px；`reproduction.json` 的 `passed=true`。数值比较要求外参矩阵元素最大差异小于 `1e-6`、重投影统计量差异小于 `1e-4 px`，这些是软件复现容差，不是物理标定精度。

## 可选路径和限制

基础开发测试可单独执行 `pip install -e '.[dev]'`、`pytest`，不下载仿真资源；可选上游未安装时会显示跳过项。完整来源和可选仓库见[外部资源](../references/robot_assets.md)。资产下载、CPU 几何检查、标定复算与 GPU 渲染应分别报告验证结果；没有独立 GPU 渲染记录时，不把 CPU 检查称为渲染成功。
