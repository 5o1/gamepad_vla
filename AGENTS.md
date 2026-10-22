# Scope

This directory is ConCon's downstream part: dexterous-hand data collection,
generation and training. A hand controller, not necessarily a VLA, takes upstream
gamepad signals and physically actuates the gamepad to reproduce them. The pi0.5
conditioning path under `src/gamepad_vla/vla/` is deferred.

- Do not edit `resnake_gym`, which owns game logic and game RL, or
  `xbox_gamepad_physics`, which owns the gamepad asset. Copy their assets in and
  record provenance.
- Research new designs against primary sources before implementation. Record the
  adaptation and evidence limits in README.md.
- Human hand-motion data may be used, for example as an adversarial motion prior
  (user decision, 2026-10-14). Ask before introducing BC or DAgger training.
- Run training and other compute on the configured compute node and workspace,
  and only on GPUs that are free. Model weights, datasets, converted assets and
  run outputs stay there.
- Do not overwrite shared openpi, RoboTwin, Isaac or Python environments.
- The LinkerHand collection station is read-only by default and runs no
  training. Sending physical robot or hand commands, including CAN query
  frames, needs a separate hardware execution authorization.
- Passing gradient tests is not evidence of manipulation success.

## 采集控制

Codex 通过语音或文字控制采集前，先读 [采集 API 使用说明](docs/usage/collection-api.md)。经站点远程 shell 在采集台本机运行 `collection_ctl.py`：先查状态，远程控制关闭时请现场人员开启，不得绕过。停止使用幂等的 `record-stop --wait`，随后查状态并复述实际结果。硬件节点、急停和下电不在该 API 的能力范围。

采集画面与手柄资产固定使用 xbox-gamepad-physics **0.4.1**、SAPIEN **3.0.3**；新建渲染场景调用公开 `robotwin.create_scene`，采集画面复用 `operator_view` 的 teleop mmap/NVENC 管线。

## 采集平台硬性约定（2026-10-23，违反过就记在这里）

- **复用已有管线，不另写一套**：采集画面与推流只用 `operator_view`（手柄仓库 teleop mmap/NVENC 管线，加上 spec `appearance` 和标定 `camera_3`）。新建场景用手柄仓库公开的 `robotwin.create_scene` / `load_into_scene`。任务 L 曾另写渲染（白桌面、丢失可视部件、ABXY 不显示），已删除。审阅时检查是否出现第二套实现。
- **臂安装朝向用 v4.5 起的场景**：v4.4 及以前用的是厂家设计安装值，站上主臂下垂时仿真臂会向后反折。v4.5 绕各臂 J1（基座 z）轴修正：左 −90°，右 +90°，右臂再 +15°。右臂这 15° 对应 LinkerTA `offset_deg` 右 J1 的 −15°，是推断值，待真机验证。`station_g20_v4_4.json` 保留不动，供已发布的固定手柄结果复现。
- **主臂→从臂映射**：`arm_mapping.py` 逐值移植站上 `map_left/right` 和绝对限位，不要改。站上实际运行关闭启动限幅（`limit_around_initial:=false`），仿真对应 `--arm-startup none`。LinkerTA 输出已含 `negation` 与 `offset_deg`，下游不要重复补偿。
- **ROS 输入**：LinkerTA 臂话题是 BEST_EFFORT，重定向手话题是 RELIABLE；订阅端按发布端设置。仿真联调用独立 ROS 域与 loopback。站点配置 `collection.ros_topic_prefix` 正式采集必须为空，测试前缀只用于验收，用完复原。
- **主臂自然下垂会插进桌面**：遥操采集用 `--scene-variant gamepad-contact`，机器人只与手柄碰撞。
- **站上重定向节点**要用工作站 conda 环境的 Python 启动，系统 Python 缺依赖。
- **部署到采集台**：用 `scripts/station/sync_tree.py` 整树同步（git 跟踪文件加未忽略的新文件），同步后比对哈希。不要手挑文件，曾因漏传新文件导致网页按钮全部失效。
- **不确定但影响小的细节**：选一个合理处理，在场景 notes 或报告里注明，然后继续，留给真机实验验证，不要为此停下提问。
