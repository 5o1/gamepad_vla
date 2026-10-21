# 采集台安装与无硬件验收（M1）

任务日期：2026-10-22。**安装完成，M1 整体未通过：M1-c 无丢失、逐值一致门槛失败；M1-b 和 M1-d 因 GPU 有其他进程而未执行。** 失败后停止验收，没有修改运行时代码、调参或重跑门槛，没有启动硬件输入或执行 M2。未 commit。

## 门槛结果

| 项目 | 本次实测 | 结论 |
|---|---|---|
| M1-a 全部单元测试 | 124 通过、3 跳过、0 失败、0 错误；pytest 0.55 s | 通过，保留可选依赖跳过项 |
| M1-b SAPIEN GPU 渲染、推流、录制 | 没有执行；编码帧数无样本，持续时间、fps、应用→编码 P50/P95 均未测 | 未通过验收，不能填写 0 fps 或沿用 L 的 CPU 结果 |
| M1-c 隔离 ROS2 输入及 SAPIEN 录制 | 6300 行，物理末时刻 62.999 s、录制循环墙钟 63.6817 s；发布 24000 条，录到 4840 条不同指令，缺 19160 条（79.8333%）；28 条右手值改变 | **失败，已停止** |
| M1-c 发布→应用 | 已匹配的 4840 条首次应用：P50 **99.6542 ms**、P95 **99.9155 ms** | 缺失消息不进入延迟分母，不能据此声称链路完整 |
| M1-d M1-b 期间 GPU/CPU 占用 | M1-b 没有运行，负载曲线没有样本 | 未执行 |
| M1-b episode / LeRobot / 浏览器 | 没有本次 GPU episode、导出或原生读回；浏览器拟用无头方式，但没有执行 | 未执行，既有 L 的导出证据不替代本次门槛 |

M1-c 本身的 episode v2 严格读取成功，6300 行全部通过 schema、长度、场景哈希和单调仿真时钟检查；每行 overlay.values 与实际 gamepad_signal 完全相等。这只验证保存的数据，不构成浏览器 overlay 的视觉验收或 M1-b 渲染证据。

对应数值文件：[M1-a](M1-a.json)、[M1-c](M1-c.json)、[录制](M1-c-recording.json)、[发布](M1-c-publisher.json)、[逐路诊断](M1-c-diagnosis.json)、[安全与清理](safety-and-cleanup.json)。

## 复现审阅者的采集台状态

安装前只读检查确认：RTX 4090，驱动 580.173.02，356 MiB、4%；Vulkan 有 NVIDIA ICD；系统 Python 3.10.12、已有 miniconda、磁盘可用 521 GiB；PyPI、上交镜像、GitHub 均返回 HTTP 200，站上已有代理可用。安装根目录最初不存在，数据目录已存在；Humble setup 和急停脚本均存在。按实际 executable/进程命令筛查，没有正在运行的 ROS、Manus、相机、珞石或手 SDK 节点。

宽泛的 `pgrep -af` 会匹配 Cursor 旧终端中保存的命令文本，不能将这些 shell 当作正在运行的硬件节点。GPU 上实际存在 Xorg、gnome-shell、桌面集成、Firefox 和 RobotAssist。按用户“有别人的进程则等待或标为未执行”的约束，未将桌面负载自行豁免；安装期间复查仍有这些进程，因此未启动渲染、浏览器或任何 GPU 计算。RobotAssist 已在基线中存在，本任务没有启动它。

任务日期按用户指定。采集台原始 `date -u` 日志为 2026-10-21 20:40–20:46 UTC，与任务日期有差异；保留原始日志，未校时。延迟用同一台机器的 Unix 纳秒时钟配对，调度和耗时用 monotonic；未跨机器相减。

## 部署快照

通过指定 SSH profile 访问；普通命令用 `-n`，传输不带 `-n`，采用 tar 管道。包括当前未提交文件，不修改兄弟仓库。

- Git HEAD：`ac5001c34c20d381a6c37bfec4eeec0c860fe321`。
- 测试时 tracked diff（`git diff HEAD --binary`）SHA256：`6347fb9b7bd3ab08e4af39ff57d45d89a9c30032fb7e9e536b75e4e729ae9b14`。
- 测试时文件清单 SHA256：`b994624cea3622305307303dc63c8cb5fc0f319ba73ff8a2cf4eafa9b3d0c07c`。
- 268 个文件，17181903 bytes，远端全部校验通过。清单逐文件列内容哈希；未跟踪文件也包含在内，不能只用 tracked diff 覆盖它们。
- 排除全部 Git 忽略内容、`.venv/`、`.git/`、`.local/`、本地资产、以及大于 8 MiB 的文件；本次没有额外命中大文件阈值。站点私有配置重新在采集台建立，未将本地其他机器的配置照搬过去。

公开证据为 [deployment-snapshot.json](deployment-snapshot.json) 和 [files.sha256](files.sha256)。此清单冻结的是**实际测试时快照**，当时的使用说明尚未补入本报告。测试后只补交报告与 M2 文档，最终交还清单保存在采集台 `~/gamepad_vla/deployment/handoff-files.sha256` 和 `handoff.json`；实际运行时代码未改。

采集台代码为 `~/gamepad_vla/code/`。私有配置、隔离域/测试话题、DDS UDP 配置和测试脚本在 `~/gamepad_vla/code/.local/`；原始日志在 `~/gamepad_vla/logs/`，均未复制进公开报告。远端保留测试时 diff、快照和清单，位置为 `~/gamepad_vla/deployment/`。

## 环境和资源

环境说明及完整已安装包版本见 [environment.json](environment.json)。

| 项目 | 本次安装 |
|---|---|
| SAPIEN venv | `~/gamepad_vla/envs/sapien-3.0.3-py310/`，系统 Python 3.10.12 创建，未启用 system-site-packages |
| 核心固定版本 | SAPIEN 3.0.3、xbox-gamepad-physics 0.3.1、NumPy 2.2.6、aiohttp 3.14.5 |
| 版本约束 | 使用部署副本的 `constraints/sapien-py310.txt` 和 `pyproject.toml`，手柄使用资源清单固定的 release wheel |
| ROS ABI | source Humble setup 后保留其 PYTHONPATH；真实导入 rclpy、sensor_msgs 成功，随后实际完成 ROS 发布/订阅 |
| 可选 MuJoCo | 未安装；本站实时采集使用 SAPIEN，本轮没有准备独立 Python 3.11 与修补 wheel；未用普通 wheel 替代 |
| 数据工具 / LeRobot | 未向 SAPIEN 环境混装；关键门槛停止后没有安装或执行本次导出 |

按已有 `fetch_resources.py --group simulation`、`setup_sapien_env.sh --python <系统Python>`、`prepare_resources.py`、`verify_resources.py` 流程执行。64 项固定下载条目通过原有哈希校验；手柄资源从固定 release ZIP 离线构建；左右 G20 从公开固定 L25 源按规范化 XML 校验后准备，两个 AR5 0.8R 使用已有转换流程。生成 32 个网格的 64 个视觉分片，117 个引用网格全部存在，home 关节限位有效。没有修改 v4.4 场景、驱动参数、方向键任务或 L 门槛。

所有环境、下载、转换、pip/XDG 缓存、TMPDIR、ROS 日志、ROS_HOME 和 pytest 临时目录均定向到安装根目录。没有 sudo，没有修改系统 Python、conda base、Humble 安装或既有 workstation。数据目录本次未新增数据，录制留在安装根目录。

测试时的环境加载入口为 `~/gamepad_vla/code/.local/runtime.sh`；安装过程及 pip freeze 留在 `~/gamepad_vla/logs/`。M1-a 命令为：

```bash
source ~/gamepad_vla/code/.local/runtime.sh
"$P" -m pytest -q --junitxml="$W/logs/pytest.xml" \
  -o cache_dir="$W/cache/pytest" --basetemp="$W/tmp/pytest"
```

三个跳过项：gymnasium 缺失导致方向键测试模块跳过；torch 缺失导致 VLA 测试模块跳过；resnake_gym 未安装导致名字对照测试跳过。没有为消除可选项跳过而引入训练依赖。

## M1-c 方法与完整分母

采用真实 Humble rclpy 和 JointState，域固定为 87，ROS_LOCALHOST_ONLY=1，Fast DDS 使用仅回环 UDP 的私有 profile。没有启动 ROS CLI daemon、硬件 SDK 或驱动。四个 subscription 通过 ROS remapping 指向以下测试话题，测试发布器也只向这些指令话题发送：

- `/gamepad_vla_test/left_hand_command`
- `/gamepad_vla_test/right_hand_command`
- `/gamepad_vla_test/left_arm_command`
- `/gamepad_vla_test/right_arm_command`

真实话题没有发布消息。测试脚本为 `~/gamepad_vla/code/.local/m1_ros2.py`，启动命令为：

```bash
source ~/gamepad_vla/code/.local/runtime.sh
"$P" .local/m1_ros2.py > "$W/logs/m1-c.log" 2>&1
```

脚本直接使用未经修改的 Ros2Source、ArmMapper、CollectionSapienBackend、Session 和严格 episode 读取器。为逐条测量，循环显式复现入口的 poll→apply→step→record 顺序，增加 packet_sink 接收日志和首次应用时间；**本次 M1-c 使用测量脚本，不声称运行了 run_collection.py CLI**。完整 v4.4 碰撞场景，CPU PhysX、无渲染，采样请求 100 Hz、6300 次，原物理步长和增益不变。

输入来自已在计算节点导出的 8 月样本 `workstation_20260831_201954-0`：1147 行、30 Hz、周期 38.2333336 s，按原时间零阶保持并显式循环，测试只进入第二遍，不是 60 s 独立原始数据。NPZ 的 SHA256 为 `8fe8b3954ca51a8f35c72d3ac6985bd3f5e5e5071399fa641fc43987b05fc861`，仅转移既有数据，没有重新访问采集设备。

双手保留原 20 路电机值。8 月臂 action 是最终从臂 rad，缺少真实主臂 deg；本测试将其转为度数后，作为**合成主臂传输测试向量**交给现有 ArmMapper，未逆推或宣称恢复原始主臂轨迹。因而这里只验证 ROS 数值传输和记录，不能证明复现了 8 月真实主臂动作。两臂记录同时保存该测试原值和转换后目标。

每路 6000 条，首末发布跨度约 59.990 s，四路实测频率分别为 99.99990–99.99997 Hz。消息头时间戳与发布日志配对；等待四路订阅发现后开始发布。发布器退出后采集继续到 6300 行，保留收尾窗口。

| 路径 | 发布 | episode 不同时间戳指令 | 缺失 | 值不一致 | 乱序 | 发布→应用 P50 / P95 ms |
|---|---:|---:|---:|---:|---:|---:|
| 左手 | 6000 | 1210 | 4790 | 0 | 0 | 99.8552 / 99.9502 |
| 左臂 | 6000 | 1210 | 4790 | 0 | 0 | 99.6148 / 99.7315 |
| 右手 | 6000 | 1210 | 4790 | 28 | 0 | 99.6637 / 99.8069 |
| 右臂 | 6000 | 1210 | 4790 | 0 | 0 | 99.5391 / 99.6776 |

接收回调日志同样只有 4840 条；首次应用和 episode 中不同指令数也均为 4840。episode 中同一最新指令可能重复出现，6300 行不等于每路录到了 6300 条新消息。四路已保留的指令时间戳非递减，发布时间戳、观测墙钟和仿真时钟单调；解码错误 0，没有未知时间戳或乱序。所有已匹配臂原始度数逐值一致；不能把经过限幅的目标直接当作应与原始度数一致的字段。

延迟起点是实际 publish 调用前的 Unix ns，终点是 backend.apply 返回，即执行器目标设置完成；不含物理响应时间。按首次应用计数，重复保持值不重复进入分母。没有为缺失消息填零、删除失败分母或用平均录制行数代替消息对账。

## 原因证据、待验证假设和后续最小实验

**已确认事实：** `collection_sources.py::Ros2Source.poll` 每次仅调用一次非阻塞 spin_once，subscription 深度为 10，并只保留各路 latest；Session 只保存每个采样时刻的当前指令，不是消息事件日志。100 Hz 的单回调轮询不具备处理四路合计 400 条/s 的容量上界。本次回调数、应用数和最终 journal 的独立指令数相等，缺失已发生在进入应用日志之前。

**原因假设尚未通过对照实验确认：** 回调调度吞吐和有界订阅队列是大量缺失的主要原因；不能仅凭这次运行把 DDS、executor 调度及 latest 覆盖各自的贡献拆开。最小对照应固定同一发布序列、频率、场景、QoS 和 100 Hz 物理记录，只改为及时排空回调，分别测发布、回调、首次应用和 journal 四个计数。若回调恢复到 24000 而 journal 仍缺失，则还需处理采样快照合并事件的问题；若回调仍缺失，就推翻“仅一次 spin 是充分原因”的假设，继续检查 executor、QoS 和调度。**此对照未执行，按失败即停要求未修改适配器。**

**值差异已通过日志分析确认：** 28 条右手差异全部精确等于 `protocol.py` 现有电机 `round` 后限幅结果，最大绝对变化 0.4946823。原始非整数输入经适配器 encode/decode 后，原值没有被单独保留。该结果能解释本次所有值差异，但不能把“取整符合协议”当作本任务要求的逐值一致。后续若修复，需在新的授权任务中区分原始日志与执行器转换值，再单独验收。

M1-b 没有失败测量，只是未执行。其 ≥30 fps、≥5 分钟和应用→编码 P50/P95 仍需独立 GPU 窗口与编码端到端时间标记；现有 render_encode_s 不包括完整的指令应用到编码路径，不能换个指标冒充。浏览器检查方式选无头，尚未执行，未产生诊断图。

## 修复预算与清理交还

安装与资源准备一次完成；M1-a 执行一次；M1-c 执行一次、失败后没有修复或重跑；运行时代码修复次数为 0。清理证据脚本第一次因本地变量名复用报 AttributeError，改正变量名后重新收集只读清理证据，计该步骤 1 次修复；没有因此启动新测试或硬件进程。 交还步骤首轮卫生扫描把 OpenCV 四段版本和允许的 `~` 路径误判，且在产物生成失败后误尝试传输；tar 拒绝空输入，未改动远端文件。修正扫描并先确认产物存在后重新同步，计交还步骤 1 次修复。

测试发布器自行退出；测量脚本在 finally 中销毁 subscriber、关闭 Session/SAPIEN 并 shutdown rclpy。随后 `pgrep` 返回 1、无测试进程匹配；可读进程环境中 ROS_DOMAIN_ID=87 的残留数为 0。收尾 GPU 为 360 MiB、7%，与测试前复查 360 MiB 相同，较最初 356 MiB 多 4 MiB，变化来自既有 Xorg 图形基线；没有本任务的 GPU 进程。不能将这点桌面波动算作本任务的 GPU 负载测试。

采集台只留下安装根目录中的代码、环境、资源、测试数据和日志；未改写其他用户目录或既有数据。部署补交报告时不启动任何服务。私有站点配置留在采集台，不回传公开仓库。M2 仅写入 [采集台仅仿真联调手册](../../usage/collection.md#采集台仅仿真联调)，没有执行；M1-c 仍须修复后重新验收，当前安装不能作为可靠数采已通过的交付。

关键路径：

| 内容 | 采集台路径 |
|---|---|
| 代码与 M2 手册 | `~/gamepad_vla/code/`、`~/gamepad_vla/code/docs/usage/collection.md` |
| 本报告 | `~/gamepad_vla/code/docs/reports/2026-10-22-station-install/` |
| 私有配置及测量入口 | `~/gamepad_vla/code/.local/station.json`、`runtime.sh`、`m1_ros2.py` |
| SAPIEN 环境 | `~/gamepad_vla/envs/sapien-3.0.3-py310/` |
| 来源和生成资产 | `~/gamepad_vla/external/`、`~/gamepad_vla/assets/` |
| 8 月 NPZ | `~/gamepad_vla/data/august/states.npz` |
| M1-c episode | `~/gamepad_vla/runs/m1-c/recording/episode-000000/` |
| 发布、接收、应用日志 | `~/gamepad_vla/runs/m1-c/{published,received,applied}.jsonl` |
| 安装、测试和清理证据 | `~/gamepad_vla/logs/` |
| 部署快照与最终清单 | `~/gamepad_vla/deployment/` |
| 既有数据目录 / 急停 | `~/gamepad_vla_data/`、`~/gamepad_vla_estop/estop.sh`（未执行） |
