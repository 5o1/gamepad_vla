# 采集台安装与验收（M1，审阅后续做）

日期按任务：2026-10-22。**本次 M1-c、M1-b 和资源验收通过，LeRobot 原生读回完成。M2 仅更新现场手册，没有执行硬件联调。** 不 commit，不修改兄弟仓库、物理后端或 L1/L2 门槛/结论。

首次安装、依赖和失败记录完整保留在 [initial-attempt.md](initial-attempt.md)。旧 M1-c 确实失败，没有覆盖其证据；以下为审阅者授权修复后的新实测。

## 审阅者裁决

1. 旧 M1-c 的 24000→4840 条、约 100 ms 延迟和取整后原值丢失，是任务 L 的实现缺陷；L 阶段仅用替身测试 ROS 是验收缺口，不修改门槛来迁就实现。新修复预算 3 次：独立线程持续 executor spin、每路 QoS 至少 100、线程安全无损事件队列、逐消息原值和双时钟、原值与实际应用目标分开、schema 升版、增加真实 rclpy 回归。正式复验要求四路约 100 Hz、至少 60 s、域 87 和测试话题，原始消息逐位相同且无乱序，60 Hz 控制下发布→应用 P95≤25 ms。
2. 4090 是桌面显卡。Xorg、gnome-shell、Firefox、RobotAssist 等常驻图形进程允许共存；**无其他 CUDA 计算任务、显存 <4 GiB、利用率 <30%** 时可开始 GPU 测试。测试期间不关闭或干扰桌面进程，结束确认显存回到基线。
3. 保持原写入范围、ROS 隔离、不访问硬件、不 commit、公开卫生约束；修复后在计算节点或本地重跑 L 单元测试。M2 仍然只写现场手册。

本次按以上裁决执行。采集台仅写安装根目录，未启动 SDK、珞石节点、LinkerTA、Manus 或相机，不访问 CAN/串口/USB，没有 sudo 或系统环境修改。

## 最终门槛与证据

| 项目 | 实测数值 | 结论 |
|---|---|---|
| L / M1-a，计算节点全部测试 | **127 通过、3 跳过、0 失败**，5.70 s；真实 rclpy 回归已执行 | 通过 |
| 采集台全部测试 | **126 通过、4 跳过、0 失败**；单独执行下述 60 s 真实 ROS 验收 | 通过；列明跳过项 |
| M1-c 原始消息 | 四路各 **6000/6000**，合计 **24000/24000**；原始 position 的 float64 字节、消息头和顺序完全一致，0 解码错误，时间戳单调 | 通过 |
| M1-c 60 Hz 控制 | **3661 行 / 61.016 s** 物理时间；含收尾与读回的测试墙钟 62.3975 s | ≥60 s，通过 |
| M1-c 发布→应用 | **P50 3.5435 ms / P95 6.9580 ms**，实际选中最新值的 14396 次首次应用 | P95≤25 ms，通过 |
| M1-b GPU 编码推流 | 完整 **300 s** 窗口 **17418 帧，58.06 fps**；逐秒最低 **57 fps**；17418 个不同物理状态 | ≥30 fps / 5 分钟，通过 |
| M1-b 独立 HTTP 客户端 | 同一 300 s 窗口 **17305 帧，57.6833 fps**，逐秒最低 **54 fps** | 通过 |
| M1-b 应用→编码 | **P50 23.7510 ms / P95 33.2875 ms**，分母 17418 编码帧 | 实测报告 |
| M1-b 浏览器和 overlay | 本机无头 Chromium，观察 **306.396 s**；**5168** 次 renderer 输入核对，原始实际信号及 overlay 映射错误均 **0**，JS 错误 0 | 通过 |
| M1-b 录制与导出 | 完整 v3 episode **325 s /19500 行**，严格读取成功；数值 Parquet 全字段往返一致，LeRobot 0.4.0 原生读完 **19500 行** | 通过 |
| 清理 | 测试进程 0，域 87 残留 0，CUDA 进程 0；显存 **360→858→360 MiB** | 通过 |

数值证据：[tests-resume.json](tests-resume.json)、[M1-c-resume.json](M1-c-resume.json)、[M1-b-resume.json](M1-b-resume.json)、[export-resume.json](export-resume.json)、[cleanup-resume.json](cleanup-resume.json)。

## 100 ms 延迟的根源和对照

旧代码每次 poll 只执行一次 spin_once，每路队列深度 10；四路合计 400 条/s，而 60/100 Hz 控制循环至多分别处理 60/100 次回调/s。收到的是积压队列中的旧消息；持续发布时，约十个 10 ms 周期对应约 100 ms 的消息年龄。stale_after_s=.25 只排除过期消息，既不调度接收，也不会主动等待 100 ms；源码没有用于这条路径的 100 ms 定时器。

修复前用**真实 rclpy** 做同台最小对照：保持四路各 100 Hz、发送 2 s、同一旧代码、深度 10、相同 stale 值，只改变 poll 频率：

| 旧代码 poll 频率 | 发布 / 接收 | 接收年龄 P50 / P95 |
|---|---:|---:|
| 60 Hz | 800 /120 | 96.646 /427.548 ms |
| 1000 Hz | 800 /800 | 1.984 /3.904 ms |

P95 包含发送结束后的积压排空阶段，因此不能与首次 60 s 验收的持续发布 P95 直接等同。对照证明：仅改变回调处理节奏就消除了该积压，支持“回调吞吐受控制环限制”为主要根源，不支持 stale 阈值或独立定时器造成固定等待。证据见 [spin-diagnosis.json](spin-diagnosis.json)。新版持续 spin 后，正式 M1-c 接收年龄 P50/P95 为 **0.3462/0.7120 ms**，进一步支持修复确实消除了结构性积压。

## 实现、schema 与适配依据

依据 [Humble executor 源码](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py)、[Node 的订阅 API](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py)和 [Python FIFO 文档](https://docs.python.org/3.10/library/queue.html)实施；不是用更深队列掩盖慢消费。

- `Ros2Source` 使用专用 **SingleThreadedExecutor 线程持续 spin**。单线程串行回调保留接收顺序；每路 QoS 深度 **128**，覆盖约 1.28 s 的 100 Hz 输入。
- 每条接收消息都进入线程安全 FIFO。事件包含全局接收序号、通道、原始 JointState 数值、整数 sec/nanosec、frame_id、接收单调 ns 和墙钟 ns，另存转换后的候选目标；原始 float64 不取整。错误消息也保留事件并计错误，而不是悄悄消失。
- 主线程按顺序写入事件日志，独立的 latest 缓存供控制环取最新值；60 Hz 控制可以跳过中间目标，**不能跳过原始消息录制**。结束时先停接收线程，再排空 FIFO、关闭 episode。
- schema 从 **v2 升到 v3**：`commands.jsonl` 是逐消息日志；`steps.jsonl.commands` 是实际应用的协议目标，`input_sequences` 关联被选中的接收事件，`applied_monotonic_ns` 是目标应用完成时间。事件中的 `converted_command` 只是该消息的转换结果，不谎称所有中间目标都被实际执行。
- 读写器检查 journal 数量和接收顺序，兼容读取旧 v2；旧 v2 缺少逐消息原值，不能补称无损。读写测试覆盖非整数电机值、signed zero 的 float64 字节、原值与取整值分离、日志截断拒绝和 v2 兼容。
- `run_collection.py` 接入 FIFO 排空、收尾和应用计时；`forward_collection.py` 保持只订阅/UDP 转发，显式释放不录制的事件队列。独立渲染器只加计时字段，未改物理。

新增可复用入口为 `scripts/check_ros2_collection.py` 和 opt-in 的 `tests/test_ros2_collection_integration.py`。计算节点已存在 Humble，SAPIEN Python 3.10 环境能真实导入 **rclpy 3.3.21**，无需安装或覆盖 ROS。通过 `GAMEPAD_VLA_TEST_ROS2=1` 显式启用，普通缺 ROS 环境中的跳过不会算作通过。

计算节点 3 个跳过项仍为 gymnasium、torch、resnake_gym 缺失；采集台额外跳过普通 pytest 中未显式启用的短 ROS 回归，但其真实 60 s 验收已独立执行。本地针对性单元测试 19 项通过。

## M1-c 正式复验口径

域固定 **87**，ROS_LOCALHOST_ONLY=1，沿用仅回环 UDP 的私有 DDS 配置。发布器只在 `/gamepad_vla_test/regression/` 前缀下发送四路 JointState；数采源订阅这些测试话题，绝未向真实指令话题发布。实际使用生产 Ros2Source、ArmMapper、Session 和完整碰撞场景的 SAPIEN CPU 后端，无渲染。测试驱动器用真实 ROS publisher 节点和独立发布线程，不使用替身。

启动命令：

```bash
source ~/gamepad_vla/code/.local/runtime.sh
"$P" scripts/check_ros2_collection.py --backend sapien --duration 60 \
  --replay "$W/data/august/states.npz" --output "$W/runs/m1-c-resume-1"
```

四路发布实测均约 100.009 Hz。逐路发布与接收序号均为 0–5999；对**落盘后**的原始 position 转为 float64 字节比较，不用容差或取整后比较代替逐位一致。接收单调时钟、墙钟和每路消息头时间戳均已核对顺序。

延迟起点为同机 publish 调用前的单调时钟，终点为 backend.apply 返回，不含物理响应时间。6000 条消息中，每路 **3599 条**被 60 Hz 控制选作最新目标，合计 14396 个延迟样本；其余 **9604 条**被更新目标覆盖，但仍全部保存在原始 journal 中，不是丢失。所有四路 P95 均低于 25 ms，详细分项见数值文件。

输入仍是 1147 行、约 38.2333 s 的 8 月样本，显式循环，不声称新增独立原始数据。手保持原电机值；8 月只有最终从臂 rad、没有原始主臂 deg，所以臂在此只转换成度数传输测试向量，再经既有 mapper，**不能认作恢复了原始主臂轨迹**。

## M1-b 与浏览器、编码和导出

运行前 nvidia-smi 无 CUDA 任务，显存 **360 MiB**、利用率 **8%**，符合审阅后的准入规则。始终保留桌面进程。GPU 选择仅作用于本任务，未修改系统 Vulkan 配置。

运行真实 CLI，完整 v4.4 场景，SAPIEN GPU 渲染副本、8 月回放、overlay、录制全部开启：

```bash
source ~/gamepad_vla/code/.local/runtime.sh
export CUDA_VISIBLE_DEVICES=0 GAMEPAD_VLA_RENDER_DEVICE=cuda:0
"$P" scripts/run_collection.py --backend sapien --source replay \
  --replay "$W/data/august/states.npz" --loop-replay \
  --duration 325 --rate-hz 60 --realtime --view --record \
  --host 127.0.0.1 --port 8780 --output "$W/runs/m1-b-resume-1/collection"
```

使用单独的 Playwright 1.55.0 环境和 Chromium headless shell build 1187，浏览器关闭 GPU 加速，profile/缓存/临时目录都在安装根目录。本机访问回环 HTTP；没有启动或控制已有 Firefox。浏览器 305 次图像采样得到 305 个不同图像摘要，画面宽 640，SVG overlay 存在。测试插桩只观察原 renderer 的真实 applyState 调用，逐次比较实际 gamepad_signal、canonical overlay 与 renderer 输入，原渲染路径继续执行。

截图已检查，可见仿真画面、录制状态和虚拟手柄：[browser-resume.png](browser-resume.png)。这是仿真视觉验收，不是实物操作能力证据。

完整编码跨度 **322.729 s**。预先按浏览器开始时刻确定连续 300 s 窗口，统计全部帧与全部一秒分桶，无剔除低速区间。HTTP 端使用独立 MJPEG 接收器；预览允许 latest-frame 丢弃旧帧，因此 HTTP 数量略少于编码数，不能用这个差额声称 ROS 指令丢失。未测量浏览器逐帧呈现 fps 或应用→显示延迟，不将编码/HTTP 帧率冒充这两个指标。

应用→编码计时从真实目标设置完成开始，经过物理步进、记录、进程间传送、渲染，到 JPEG 编码完成；同机 monotonic 时钟。单独的渲染+编码 P50/P95 为 **9.8399/15.2161 ms**，它与完整应用→编码 **23.7510/33.2875 ms** 是不同指标。

episode 完整 19500 行，实际 325 s、解码错误 0；8 月样本完成 8 次循环后进入第 9 遍，不是 325 s 独立数据。episode 的所有 overlay.values 与实际 gamepad_signal 相等。传到计算节点的 meta、steps、commands 文件逐字节哈希一致，见 [episode-transfer.json](episode-transfer.json)。用现有 data-tools 导出、现有 lerobot-check 原生读取全部 19500 行及形状：状态 56、动作 54、信号/mask 各 32、世界位姿 84。此次导出是数值数据，没有把异步 MJPEG 冒充逐物理帧对齐视频。

## M1-d 资源占用

同一 300 s 窗口内有 **283** 个资源样本，约每秒一次（查询开销导致非严格 1 Hz），全部计入：

| 指标 | P50 | P95 | 最大 |
|---|---:|---:|---:|
| GPU 显存（MiB） | 858 | 858 | 858 |
| GPU 利用率 | 22% | 46% | 53% |
| 本任务进程树 CPU（单核100%） | 175.23% | 186.14% | 193.55% |
| 整机 CPU | 6.19% | 6.60% | 7.75% |
| 本任务进程树 RSS（MiB） | 2109.51 | 2141.77 | 2150.97 |

进程树包含采集、渲染、测试浏览器和采样器；RSS 是进程 RSS 求和，可能重复计算共享页，不等于独占内存。相对测试前 GPU 基线增加约 **498 MiB**，测试后恢复 360 MiB。测试中的 GPU 利用率超过 30% 是本任务自身负载；30% 是启动前准入条件，不是运行上限。原始采样包含私有进程路径，留在采集台，不直接公开。

## 修复预算、隔离和交还

本次 ROS/录制采用 **第 1 轮实现（1/3）**：新增读写测试首次把内存 tuple 与 JSON list 直接比较而失败，修正测试断言的 JSON 归一化后通过；没有为此改变运行时目标或原始数值。计算节点真实集成、M1-c 正式 60 s 和 M1-b 长测均一次通过，没有以调参重跑门槛。后续文档和计数整理不改变已测运行时行为。

[protected-files.json](protected-files.json)列出前后内容哈希，确认 SAPIEN/MuJoCo 物理、接触判据、固定任务和 v4.4 场景均未改；未重写 L1/L2 历史证据。Humble、系统 Python、conda base、既有 workstation 和采集硬件保持原状。M2 精确命令、标定开关保持 0、禁止 SDK/从臂节点、话题核对、GPU 共享规则和停止/急停步骤见[采集台仅仿真联调手册](../../usage/collection.md#采集台仅仿真联调)，本次未执行。

测试和浏览器均已退出；pgrep 无匹配，进程环境检查中域 87 无残留，CUDA 任务为空。最终 GPU **360 MiB /4%**。急停脚本只核对位置，未执行；没有结束桌面或他人进程。

部署前后均使用 tar 管道；代码包含原有未提交改动。首次安装快照见 [deployment-snapshot.json](deployment-snapshot.json)，本轮测试代码及最终交还快照见 [resume-snapshot.json](resume-snapshot.json)。清单逐文件哈希包含未跟踪文件；tracked diff 哈希另列，不能代替未跟踪文件。站点配置、原始日志和测试驱动留在采集台 `.local/` 或运行目录，不回传公开仓库；报告仅公开已检查的数值和一张截图。

| 交付 | 路径 |
|---|---|
| 本地报告、数值、截图 | `docs/reports/2026-10-22-station-install/` |
| 本地 M2 手册 | `docs/usage/collection.md` |
| 可复用真实 ROS 回归 | `scripts/check_ros2_collection.py`、`tests/test_ros2_collection_integration.py` |
| 采集台代码 / 私有环境入口 | `~/gamepad_vla/code/`、`~/gamepad_vla/code/.local/runtime.sh` |
| SAPIEN / 浏览器环境 | `~/gamepad_vla/envs/sapien-3.0.3-py310/`、`~/gamepad_vla/envs/browser-py310/` |
| M1-c 原始事件、应用日志、episode | `~/gamepad_vla/runs/m1-c-resume-1/` |
| M1-b episode、编码/HTTP/浏览器和资源证据 | `~/gamepad_vla/runs/m1-b-resume-1/` |
| 安装与测试日志 | `~/gamepad_vla/logs/` |
| 本轮部署和交还清单 | `~/gamepad_vla/deployment/resume/` |
| 计算节点隔离代码副本 | 配置工作区内 `code/m1-resume-20261022/` |
| 计算节点 LeRobot / 原生读回 | 配置工作区内 `runs/m1-resume-20261022/station-m1-b/lerobot/` 和 `runs/m1-resume-20261022/logs/` |

公开报告不写计算节点绝对站点路径。模型和数据产物留在配置的工作区；没有提交 Git。
