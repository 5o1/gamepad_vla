# 从示范扩增姿态与轨迹

`demogen` 将已结束的 collection v4 episode 拆成单手控件片段，以手柄为参考迁移掌位和指尖参考，再解臂 IK、拼接、变化和物理回放。只有信号、碰撞和自然姿态同时通过的候选才输出为 v4 episode。原生反馈合成示范已用于新位姿通过验收；当前没有真实人类示范。[任务 AA 报告](../reports/2026-11-04-demogen-framework/README.md)保留了恒等对照、正常接触校准和失败统计。

## 环境和文件

读取、切片、FK、IK 和拼接只需要已有的 NumPy。物理阶段复用 `CollectionSapienBackend` 或 `CollectionMujocoBackend`，不安装包、不创建环境、不调用硬件。SAPIEN 必须为 3.0.3，手柄包必须为 0.4.1；`auto` 优先选择已有 SAPIEN，其次 MuJoCo。缺包或资产时记录拒绝原因，不能跳过物理门输出训练数据。MuJoCo 路径本轮未实测。

| 路径 | 职责 |
|---|---|
| `src/gamepad_vla/demogen/episode.py` | v4 完整性校验、左右流、场景与固定手柄位姿 |
| `segments.py` | 控件切片、阶段、信号统计、重叠和截断标记 |
| `kinematics.py` | 复用 `SerialChain`，手柄系变换、自然姿态特征与局部 IK |
| `generate.py` | seed、跨 episode 选段、速度/幅度/扰动、握持过渡 |
| `validate.py` | 完整采集后端回放、逐物理子步的三个硬门 |
| `output.py`、`pipeline.py` | 原有 v4 writer、manifest、通过后输出 |
| `synthetic.py` | 有明确标记的合成示范测试夹具 |
| `scripts/demogen.py` | CLI 参数与编排 |
| `configs/demogen/` | 扩增参数与无扰动复核配置 |

在有现成环境和资产的机器执行。采集台只能在没有采集会话时做短测，使用 `nice -n 10`，测试文件放 `~/gamepad_vla` 下；不要在采集台长期生成数据或训练。部署复用 `scripts/station/sync_tree.py` 整树同步并校验，可以指定独立 `--remote-dir`，不覆盖采集服务目录。

输出由站点配置决定：有 `station.data_root` 时为 `<station.data_root>/demogen/<name>`，否则为工作目录下的 `data/demogen/<name>`。`--name` 仅为相对名称，拒绝越出该目录或覆盖已有输出。测试时可通过 `GAMEPAD_VLA_SITE_CONFIG` 指向独立测试站点配置，将数据根放在上述采集台测试目录内。不要将数据根设为公开源码目录。

## 真实示范的准备和切片

每个输入目录必须含 `meta.json`、`steps.jsonl`、`commands.jsonl`。读取器要求 schema v4、complete、正确的场景摘要和关节/指尖顺序、严格递增且完整的采样时钟、独立接收计数与消息边界；存在 `integrity.json` 时它也必须是 passed，`discarded.json` 存在则拒绝。v2/v3 仍由原采集工具读取，本模块不将其补称为完整 v4。

固定手柄位姿来自 episode 的 `scene_spec_text`。移动手柄缺少逐帧根位姿，本版本拒绝迁移。读取保留实测的 21 维手关节 rad；执行优先使用已知 applied joint targets，缺失时使用实测值并保留这一口径。原始电机指令仍留在源 episode，不会反算后伪称为原始指令。

例如在已有 SAPIEN 环境中，用实际示范目录设置 `EPISODE`：

```bash
python scripts/demogen.py index "$EPISODE" --side left --name left-index-001
```

`segments.json` 的帧区间均为 `[start, stop)`，保留来源目录、左右侧、active 区间、`approach/press/hold/release/return_to_grip` 阶段、峰值、均值和时长。阶段由信号推断，数字开关的 press/release 可以为空；不是接触力或人手运动阶段的真值。

- 数字控件：默认 on=0.30、off=0.15 的滞回，去除不足 2 帧脉冲，合并最多 1 帧缺口。按前 0.15 s、后 0.20 s 留余量，相邻操作的余量在间隙中分开；活动重叠会显式标记。
- 摇杆：按径向幅值识别操作，输出 `left_stick:dir0` 至 `dir7`、右侧对应名称，以及 `circle_positive/negative`。dir0 为记录坐标的 +X，dir2 为 +Y；不擅自将 Y 号解释为上/下。方向直方图、幅值范围和展开角度保留在统计中；净转角至少 270° 才标为转圈。
- 扳机：`left_trigger:travel` 或右侧名称，记录有效行程范围与峰值。
- `grip`：所有已知手柄信号中性、整段臂/手关节范围稳定且至少 0.30 s 的候选。中性区间中的接近/返回运动会分开处理。这个标签不证明握持力或持续触壳。

生成时只使用完整、可用且活动独立的操作片段；截断、复合操作和额外控件活动保留在库里供审阅，不能默默作为独立控件迁移。先检查每个真实示范的片段数、方向统计与完整性，再扩大生成数量。

## 生成与输出

```bash
nice -n 10 python scripts/demogen.py generate "$EPISODE" \
  --side left \
  --target-spec configs/workcell/placement_trials/d050-h220-t+10.json \
  --target-spec configs/workcell/placement_trials/d075-h190-t+10.json \
  --sequence dpad_up:press,dpad_down:press,dpad_up:press \
  --count 2 --seed 42 --backend sapien --name left-generated-001
```

可提供多个 episode；控件标签使用 index 的实际输出。每个 attempt 按顺序轮换目标 spec 和基础 episode；同一侧的操作片段可从所有输入 episode 中抽取。第 i 次 attempt 的 seed 为 `seed+i`，片段选择、速度、幅度和扰动均由该 seed 决定。相同源、代码、配置和 seed 可复现计划数组；Unix 记录时间及不同引擎/平台的浮点结果不承诺逐字节相同。

默认配置速度为 0.9–1.1 倍、幅度为 0.97–1.03 倍，独立关节扰动不超过 0.002 rad，配置硬上限为 0.01 rad。幅度变化是操作手指相对握持姿态的关节行程代理：数字键的 thumb、扳机/肩键的 index；不是经实物辨识的毫米下压或力控制。模拟信号目标随幅度缩放，数字键仍要求闭合。越关节/信号范围时如实拒绝，不靠剪裁通过。其他手指保留；变化后的 mimic 使用现有 G20 规则。可按现场实际用指修改这一映射后重新验收。

用 `--config configs/demogen/replay_check.json` 关闭速度、幅度和随机扰动，先测迁移与拼接本身。常规配置见 [default.json](../../configs/demogen/default.json)。它们都使用默认 0.25 s 过渡和 0.20 s 握持停留；单条轨迹最多 120 s。

恒等对照使用独立命令，不调用生成器：

```bash
python scripts/demogen.py identity "$EPISODE" --backend sapien \
  --config configs/demogen/replay_check.json --name identity-001
```

它在源手柄位姿逐条回放原 applied targets，不做 IK、切段重组、插值、变速或扰动，结果写 `identity.json`。如果输入没有原目标，只能回放实测关节值，这不等于恢复未知指令；本任务的合成源四路目标均已知。恒等对照失败时先查源示范与验证口径，不能直接归因于迁移。

掌、各指尖参考满足 `T_pad_frame = inverse(T_world_pad) @ T_world_frame`；迁移后为 `T_world_pad_new @ T_pad_frame`。FK 复用臂/手原 URDF 和采集 fingertip offsets；速度变化插值关节与位置，旋转使用最短弧 SLERP。单位幅度、零扰动时保留原手关节目标，不对手指做 IK；换用不同手部运动学会拒绝。指尖参考取源实测 q 的 FK，迁移后用解出的臂 q 与待执行手目标重新 FK，手柄系下任一指尖误差超过 **1 mm** 直接拒绝。该检查也能暴露源 applied target 与实测状态差异过大，不能仅用目标 FK 与自身比较。

片段之间先插值回基础 episode 的握持候选、停留，再插值到下一片段。整个掌轨迹（含过渡）重新解 7 维臂 IK。局部阻尼最小二乘同时考虑自然姿态范围，最终以位置误差 ≤0.5 mm、角度误差 ≤0.01 rad 和硬姿态条件判定；失败记录具体帧和残差，不代表全局无解。

自然姿态与任务 Z 使用相同轴线定义和范围：上臂相对竖直向下倾角 10–35°、肘的世界 X 大于肩、前臂俯仰 −10–25°、水平前臂朝向手柄网格中心的偏差 ≤30°。网格中心在手柄系的参考点来自已有几何归档，保存在配置/manifest；不是根原点。非操作侧保持源握持帧的关节值，仍参与全程碰撞和自然姿态验证。

物理阶段只设置机器人驱动目标，从第一帧的机器人状态 reset；不会逐帧写 qpos 或直接写手柄信号/关节。手柄保持后端默认中性初态。验证使用 full 环境碰撞，保留后端现有自碰撞过滤；不提供 free-space/gamepad-contact 通过路径。

1. 每个原生物理子步检查机器人接触距离。默认几何数值容差为 1 µm。`physical_check.json` 另启用经 full/gamepad-contact 对照校准的正常操作接触：**仅 SAPIEN 3.0.3、左拇指 distal—移动 dpad、全为 dpad_down 的序列**使用最多 30 µm；壳体、其他手指、手间、桌子和夹具仍为 1 µm。30 µm 来自最低闭合附近实测 22.8–27.0 µm 的求解器负距离，不来自按钮行程或 contact offset。任何超界仍拒绝；所有原始负距离和正常操作接触子步单独保留，不能把允许的数值接触报成原始距离为零。未校准控件、MuJoCo 和默认配置都不使用该例外。场景未建模几何及后端过滤的同侧自碰撞不在证据范围。
2. 每个子步检查所有实际可用信号：数字键按 0.30 判断，模拟轴默认误差上限 0.15。默认允许目标边沿前后 0.08 s 的采样网格包络，避免把正常驱动响应当误触；每个目标事件还必须在自己的区间内触发至少 0.03 s，并且目标通道可用。目标外信号和未松开也会拒绝。这里沿用固定手柄的操作阈值，尚未实物标定。
3. 用全部子步实测臂 q 和初态批量 FK，检查双臂自然姿态及关节限位。记录分母、范围和无效数；不能只验 IK 目标或稀疏导出帧。

每个输出目录有 `manifest.json` 和 `segments.json`。manifest 保存源 episode/文件摘要、目标 spec 摘要、片段区间、变换、seed、变化参数、代码 commit/工作树摘要、物理版本和拒绝原因。无通过候选时只有清单，不出现 `episode-*`；CLI 返回 2。参数/输入错误返回 1，有通过候选返回 0；以 manifest 的 accepted/rejected 数为准。

通过者才写 `episode-000000/` 等目录，继续使用 v4 `EpisodeWriter` 并严格读回，记录的是物理实测状态和信号，commands 是生成的关节目标，`generated=true`、`physics_validated=true`。源示范若为合成数据，输出也保留 synthetic 标记。原 `CollectionReplay` 可直接回放。没有原始手电机消息时 `motor_positions=null`；现有 54 维 LeRobot 导出器会明确拒绝这种 action，不补造电机值。本任务选择原 v4 输出接口。

## 合成示范和验收

先使用原生物理反馈源做有正样本的短测：

```bash
nice -n 10 python scripts/demogen.py synthetic --physical \
  --spec configs/workcell/station_g20_v4_4_raised_interim.json \
  --candidates docs/reports/2026-10-23-natural-grip-fixed/candidates.json \
  --repetitions 3 --name feedback-source-001

# DEMO 为上一条返回的 episode 路径。
nice -n 10 python scripts/demogen.py identity "$DEMO" \
  --config configs/demogen/physical_check.json --name feedback-identity-001

nice -n 10 python scripts/demogen.py generate "$DEMO" \
  --target-spec configs/workcell/placement_trials/d050-h220-t+10.json \
  --target-spec configs/workcell/placement_trials/d075-h190-t+10.json \
  --sequence dpad_down:press,dpad_down:press --count 2 --seed 42 \
  --config configs/demogen/physical_check.json --name feedback-generated-001
```

`--physical` 使用原 collection 后端读取实际信号，控制器达到约 0.34 即停止加深，实测峰值约 0.35。接近、回程各 1.5 s，减少 PD 跟踪滞后；非操作指收拢，另一臂先解自然避让姿态。`physical_check.json` 的拼接过渡为 1.5 s，握持候选停留 0.3 s，速度/幅度/扰动关闭，臂 IK 容差为 10 µm / 0.0002 rad，指尖仍严格为 1 mm。源信号是实际接触观测，四路 commands 完整保存；它是固定夹持手柄的拇指操作测试，不证明全手握持或真人示范能力。其他控件仍需自己的源示范与正常接触校准。

以下旧的纯运动学 fixture 仍用于验证拒绝流程：

```bash
nice -n 10 python scripts/demogen.py synthetic \
  --spec configs/workcell/station_g20_v4_4_raised_interim.json \
  --candidates docs/reports/2026-10-23-natural-grip-fixed/candidates.json \
  --repetitions 3 --name synthetic-check-001

# 将 DEMO 设为上一条输出的 episode 目录，再 index / generate。
python scripts/demogen.py index "$DEMO" --name synthetic-index-001
```

合成 fixture 使用任务 S 的自然臂与静态十字键手关节解，插值接近、保持、返回，信号是显式编写的标签；overlay 也标为 `synthetic_scripted_label`。它没有真实输入噪声、触觉、动作滞后或独立观测的实际按键信号。完整性通过只说明 v4 文件一致，不能证明这些标签能被物理复现。

```bash
# 本地 NumPy 环境；物理测试默认跳过。
python -m pytest tests/test_demogen.py tests/test_demogen_physics.py -q

# 已确认空闲的采集台，加载现有 runtime 后短时执行，不安装包。
GAMEPAD_VLA_TEST_DEMOGEN_PHYSICS=1 nice -n 10 \
  python -m pytest tests/test_demogen.py tests/test_demogen_physics.py -q \
  --basetemp ~/gamepad_vla/demogen-check
```

物理测试保留旧 fixture 的负例，并新增原生反馈源：原位严格几何对照、校准口径对照、两个新 placement；后者要求至少一条真实物理通过输出，再严格读回 v4。正向文件接口另有测试适配器覆盖，不能与原生物理分母混用。真实示范到来后，需逐侧核对采样/信号时钟、稳定握持和控件切片，先做无扰动源位姿与小幅换位姿回放，再测扰动范围和通过率；双手同步操作不属于当前单侧序列框架。
