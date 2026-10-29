# gamepad_vla

ConCon 的灵巧手采集与控制模块：接收游戏手柄目标信号，驱动机械臂和 G20 灵巧手在实体手柄上复现操作。游戏与游戏策略由 `resnake_gym` 负责，手柄资产由 `xbox_gamepad_physics` 提供。

当前场景为 [station_g20_v4_4.json](configs/workcell/station_g20_v4_4.json)：双 AR5-5_0.8R、双 G20 和一台已标定的顶置相机。当前控制与记录链路以左手为主，右侧用于场景展示。主型号与现场铭牌一致，安装零位和新模型的接触行为仍待验证。见[场景图](docs/reports/2026-10-21-08r-multiview/render/multiview.png)、[标定归档](references/calibration/2026-10-20-ego/measured/README.md)和[硬件记录](references/linkerhand_station.md)。

从空工作目录准备资产、渲染和复算标定，请按[独立复现指南](docs/reproduce.md)执行；全部公开来源和固定版本见[资源清单](references/external_resources.json)。

本项目采用 [MIT 许可证](LICENSE)，Copyright (c) 2026 5o1。改编代码的来源、版权及外部资产的许可范围见[第三方声明](THIRD_PARTY_NOTICES.md)。

## 结构

| 目录 | 职责 |
|---|---|
| `src/gamepad_vla/` | 信号映射、运动学与可复用逻辑 |
| `src/gamepad_vla/tasks/` | 接触任务、残差控制环境与评测 |
| `src/gamepad_vla/workstation/` | 输入源、命令、后端、采集循环、记录和操作者画面 |
| `configs/` | 场景、材质及渲染参数 |
| `scripts/` | 运行、资产准备和维护入口；历史实验脚本保留在私有档案中 |
| `tests/` | CPU 回归测试 |
| `references/`、`docs/` | 原始证据、设计说明和精选实验报告 |

主链路为 `输入源 → 命令 → 后端 → 观测 → 记录器`。仿真和实机通过后端接口接入；图像渲染、编码与网页服务在独立进程运行。

## 安装与运行

基础开发环境使用 Python 3.10 或更高版本：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
cp -n configs/site.example.json configs/site.local.json
chmod 600 configs/site.local.json
python -m pytest tests -q
python scripts/run_workstation.py --source scripted --backend null --duration 1
```

填写 `configs/site.local.json` 中的工作目录、连接脚本及设备参数；也可用 `GAMEPAD_VLA_SITE_CONFIG` 指定其他配置文件。该文件只保存在本机，不加入 Git 或发行包。

可选依赖按用途安装：`sim` 为 SAPIEN，`web` 为操作者画面，`assets` 为资产处理，`data` 为 Parquet 转换。`vla` 暂缓开发，基础安装无需 Torch。ROS2 和 Isaac 使用各自的隔离环境。

仿真环境在配置的计算节点上由 `scripts/setup_sapien_env.sh` 创建，使用 Python 3.10、SAPIEN 3.0.3、手柄资产 v0.4.1 和 [版本约束](constraints/sapien-py310.txt)。先运行 `PYTHONPATH=src python3 scripts/fetch_resources.py --group simulation` 获取并校验公开资源，再安装环境并执行 `scripts/prepare_resources.py`。脚本不会覆盖已有环境；操作者视频另外需要支持 NVENC 的 FFmpeg。

在节点的仿真环境中：

```bash
source scripts/site_settings.sh
python scripts/render_workcell.py configs/render/workcell_multiview_preview.yaml \
  --output "$GAMEPAD_VLA_WORKSPACE/runs/multiview"
```

计算任务只使用空闲或获授权的 GPU；通过站点配置的 `compute.cuda_visible_devices` 和 `compute.render_device` 指定同一物理 GPU，也可用 `CUDA_VISIBLE_DEVICES`、`GAMEPAD_VLA_RENDER_DEVICE` 覆盖。场景默认读取 v4.4，可用 `--spec` 选择其他配置。

## 工程维护

- **职责边界**：复用逻辑放入 `src/`，脚本只负责参数和流程组织。一次性实验脚本放入忽略的私有目录；不要将固定日期或某次输出路径写入通用入口。
- **配置与资产**：当前场景中的 `workspace://...` 由站点配置的 `workspace` 解析，`GAMEPAD_VLA_WORKSPACE` 可覆盖。源资产、派生资产、数据和运行输出分别放在工作目录的 `assets/`、`assets/derived/`、`data/`、`runs/`。换机器时迁移完整资产目录并更新本地配置；新增资产保留来源和生成方法。历史场景保留原几何数值，路径和设备身份已与部署环境分离。
- **提交与同步**：代码、配置、测试及必要证据纳入 Git，提交前检查 `git diff --cached`。`scripts/sync_to_compute.sh` 同步 Git 跟踪及未忽略的新文件，记录提交和代码摘要，并保留节点上一版代码于 `.previous.*/code`（旧备份由维护者清理）。普通同步不传私有配置、`.local/`、资产或运行输出，并保留节点现有站点配置。首次部署可显式加 `--with-site-config`，通过 SSH 单独安装本地配置；它始终不进入源码归档。目标和连接脚本由 `compute` 配置提供。
- **验证**：行为修改补对应回归测试，场景修改检查几何、相机参数和资产引用。提交前运行 `python -m pytest tests -q`。CI 执行 CPU 测试及 wheel 构建；渲染、动力学和实机验证单独执行，单元测试不能替代它们。
- **依赖与发布**：运行依赖维护在 `pyproject.toml` 的对应 extra；仿真依赖升级后更新版本约束。用 `python -m build` 构建，再运行 `python scripts/check_distribution.py dist/*.whl` 检查网页资源。wheel 包含 Python 模块、网页和恢复命令；场景与开发脚本使用源码仓库或源码发行包。
- **采集与恢复**：每个 episode 保存配置、代码版本、运行环境，以及逐行刷新的命令和观测日志；正常结束导出兼容的 `steps.npz`。进程异常退出后，确认采集已停止，再执行 `gamepad-vla-recover <episode目录>`。恢复保留完整记录并报告残缺尾行；逐行刷新覆盖进程中断恢复，不保证断电后最后一条记录持久化。

私有盘点、现场照片、Agent 交接和原始运行日志放在忽略的 `.local/` 或外部私有存储。提交前运行 `python scripts/check_public_tree.py`；只有经过审核的报告和标定数据进入仓库。

实机命令需要单独的硬件执行授权。训练、采集和渲染使用各自输出目录；不要修改共享环境或兄弟仓库。

## 固定手柄控制

当前固定手柄阶段覆盖上游 `dpad5`：G20 左拇指通过 MuJoCo 接触产生完整执行报告，支持 IK 和残差 PPO。[任务指南](docs/usage/fixed-gamepad.md)提供准备、训练和闭环命令，[实验结果](docs/reports/2026-10-21-fixed-gamepad/README.md)记录三训练种子及长窗口验证。该路径使用固定开发分支与隔离环境，SAPIEN 展示环境保持独立。

双手全手柄的[阶段 0 可行性与设计报告](docs/reports/2026-10-21-two-hand-feasibility/README.md)按指定手指和持续背面握持检查固定掌位、臂 IK 与碰撞，提供独立复现脚本。奖励设计参考 [RoboPianist 官方实现](https://github.com/google-research/robopianist/blob/main/robopianist/suite/tasks/piano_with_shadow_hands.py)，仅借鉴正确按键、误触、指尖接近和能耗分项；有限静态搜索不证明动态操作或实物可行，本阶段没有实现训练。

[任务 S](docs/reports/2026-10-23-natural-grip-fixed/README.md)在只读固定手柄场景及 v4.5 臂安装参数下搜索自然臂姿，复用 K0 原生接触并显式检查桌面、桌架和两种立柱的距离。优化沿用 [SciPy TRF 最小二乘](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.optimize.least_squares.html)，比较固定差分步长、掌基网格中心种子及掌位/操作可达软偏好；原生两指触壳通过不代表整掌包握、控件可达或动态操作成功。

## 扩展工具与依据

- [操作者画面](docs/usage/operator-view.md)：`run_workstation.py --operator-view configs/render/operator_view.yaml --operator-run <新目录>`，配合 `--backend sapien` 和输入源参数。
- [相机面板](docs/usage/collection.md#调整并重新标定-ego-相机)：在采集网站预览、对正网格、拍标定照片及录 RGB-D；PNG 与同帧内参 JSON 保存在站点数据目录。旧的独立拍照入口已并入网站。
- [资产来源](references/robot_assets.md)记录当前模型依据；[模块边界](docs/architecture.md)说明采集与配置分工。

渲染沿用手柄资产 v0.4.1 的 example runner；网页串流适配其 latest-state/NVENC 链路，不修改上游。Python 资源打包依据 [Setuptools package data](https://setuptools.pypa.io/en/stable/userguide/datafiles.html)，CI 依据 [GitHub Python 工作流](https://docs.github.com/en/actions/tutorials/build-and-test-code/python)。

双手双臂参考轨迹采集使用独立入口 `scripts/run_collection.py`；输入单位、episode v4（兼容读取 v2/v3）、LeRobot v3 导出及以后部署的节点边界见[采集说明](docs/usage/collection.md)。双后端验收状态和设计适配的证据限度见[任务 L 报告](docs/reports/2026-10-22-collection-platform/README.md)，不以短测代替 L1/L2。

ROS 输入按 [Humble executor 源码](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py)的持续 spin 模式，在专用 SingleThreadedExecutor 线程中串行接收，使用 [Python FIFO 队列](https://docs.python.org/3.10/library/queue.html)向录制器逐条交接原始消息；每路 QoS 深度 128。控制环只选最新目标，接收事件不会因此丢弃。episode v3 的 `commands.jsonl` 保存未取整的原始 JointState、整数消息头时间戳、单调/墙钟接收时间和转换值；`steps.jsonl` 单独保存实际应用目标、关联消息序号及应用时钟。实际门槛和原始数据限制见 [M1 报告](docs/reports/2026-10-22-station-install/README.md)。真实 rclpy 回归测试需显式设置 `GAMEPAD_VLA_TEST_ROS2=1`、非默认 ROS 域，仅发布测试前缀话题；普通单元测试中的替身不能替代它。此改动不更改仿真物理或既有 L1/L2 结论。

采集网页及 REST 控制入口为 `scripts/collection_server.py`，供助手或 Codex 在采集台本机操作；[API 说明](docs/usage/collection-api.md)说明默认关闭的远程写控制、SSE、审计和语音规则。实现依据 [aiohttp 服务与流响应](https://docs.aiohttp.org/en/stable/web_quickstart.html)、[Python spawn](https://docs.python.org/3.10/library/multiprocessing.html)及 [MuJoCo 渲染相机定义](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjvglcamera)。CLI/网页共用采集控制器，episode v4 增加限幅标志并继续读取 v2/v3；独立渲染副本使用 v4.4 标定 ego 相机，MuJoCo 另对像素焦距比作图像校正，不修改物理模型。仿真、传输与浏览器验证不证明实物操作能力，本轮 N1 与采集台 N2 均通过；验收数值和限制见[任务 N 报告](docs/reports/2026-10-22-collection-web/README.md)。

相机预览复用本项目原有标定拍照 worker 的同帧 ROS Image/CameraInfo 配对与过期检查。RGB-D 只调用站上已有 ego 网页，使用私有 `camera.ego_web_script` 路径，通过回环 HTTP 原样代理录像状态及质量报告；不复制编码/统计代码。相机进程组依据 [Python Popen](https://docs.python.org/3.10/library/subprocess.html#subprocess.Popen) 的 `start_new_session` 隔离，清理接入 [aiohttp cleanup context](https://docs.aiohttp.org/en/stable/web_advanced.html#cleanup-context)。替身测试只验证接口、文件兼容与进程生命周期，真实图像、内参和录像质量仍以站点冒烟为准，见[任务 Q 报告](docs/reports/2026-10-23-camera-panel/README.md)。

生产采集默认不保存逐帧诊断或推流录屏；本次会话确需测量时用 `session-start --diagnostics`。详情与上限见[采集 API](docs/usage/collection-api.md)。闲置停止使用 Python 的 [monotonic](https://docs.python.org/3/library/time.html#time.monotonic) 计时，推流统计沿用有限 [deque](https://docs.python.org/3/library/collections.html#collections.deque) 滑窗；这些标准库机制只保证计时与内存边界，运行写盘量以[任务 U 实测](docs/reports/2026-10-29-no-prod-debug-logs/README.md)为准。
