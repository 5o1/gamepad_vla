# 固定手柄控制

当前实现以 G20 左拇指跟随方向键五状态（中性、上、下、左、右）为首个接触技能。手柄根部固定在桌面，AR5-5_0.8R 在给定关节姿态下焊接固定，手指保留动力学。上游游戏信号可从 `resnake_gym` 的 `timeline.jsonl` 读取；本阶段不包含手持手柄和视觉策略。

## 方法与依据

参考 [APRPianist 的固定修订](https://github.com/APRProject/APRPianist/tree/e31c01055a566c9764d76ded00de2710dc71b6be)：结构化状态、未来目标、IK 基础动作与有界 PPO 残差。这里用 G20 拇指的几何目标代替钢琴演示轨迹，方向切换时先根据实际指尖高度抬起，再进入下一按压姿态；不使用钢琴数据、指法或判别器权重。PPO 采用 SB3，默认因果协议的观测包含关节位置与速度、实际手柄信号、当前目标、当前相位基础动作和上一步残差。另保留预知未来十步目标的已知序列协议，用于对照。参考实现逐帧求 QP IK；这里使用预计算 IK 姿态与接触高度反馈，不声称等价复现其控制器。

动作仅写入机器人的位置执行器目标。手柄信号始终由接触求解后的关节位移经上游公共 API 产生，任务代码不按目标直接设置手柄状态。默认奖励参考 [RoboPianist 的按键与误按结构](https://github.com/google-research/robopianist/blob/main/robopianist/suite/tasks/piano_with_shadow_hands.py)，使用 `tolerance − wrong_press`，并以本任务阈值判断误按。这里对目标方向采用可靠闭合范围：超过 0.3 判定阈值再加 0.1 裕量后，更多按压行程不增加奖励；非目标通道仍按实际状态计误差与误按，中性目标也计匹配。上游只对请求键的满行程计算 tolerance，这里是面向数字手柄的适配。判定阈值本身仍为 0.3。奖励还包含指尖距离、能耗、接触力和动作变化；评测独立报告 F1、中性准确率、其他控件误触、响应延时及接触力。

第一版按参考路线建立无姿态先验的对照。APR 姿态奖励需要本任务的人类动作数据，尚未接入；BC/DAgger 也未引入。

## 依赖与资产

完整来源见 [任务资源清单](../../references/fixed_gamepad_resources.json)。手柄采用 `feat/mujoco-backend` 的固定提交 `84f43e6`，不是正式发布的 v0.3.1；MuJoCo 使用同源编译的 `3.16.0+concon.sdf3689.2`。修补版修正 SDF 符号，不能用官方 3.16.0 或曾失败的 `.1` 混合构建替代。wheel 来自清单指定的 GitHub Actions 构建，或按该 MuJoCo 分支的 `CONCON_PATCHES.md` 构建；校验值针对列出的 CPython 3.11 Linux x86_64 wheel；同源重建也可安装，脚本核对包内的源提交、版本和 patched 标记并记录新哈希，不能把新二进制当作已完成接触验证。

先按[复现指南](../reproduce.md)准备 G20 和机械臂资产。将手柄仓库检出到清单指定的提交，在配置的计算节点上执行：

```bash
bash scripts/setup_fixed_gamepad_env.sh --python python3.11 \
  --gamepad-source /path/to/pinned/gamepad-checkout \
  --mujoco-wheel /path/to/patched/mujoco.whl
```

该脚本创建独立环境，保留依赖版本和源修订；不会覆盖已有环境。若显式添加 `--inherit-site-packages`，基础 Python 的依赖只读共享，新安装仍进入独立环境；默认完整隔离。路径通过站点工作目录解析，资产与运行输出不加入 Git。

## 运行

加载站点配置并激活新环境，使用各自的新输出目录：

```bash
source scripts/site_settings.sh
source "$GAMEPAD_VLA_WORKSPACE/envs/fixed-gamepad-mujoco-py311/bin/activate"
python scripts/run_fixed_dpad.py inspect --output "$GAMEPAD_VLA_WORKSPACE/runs/dpad-inspect"
python scripts/run_fixed_dpad.py ik --eval-seeds 300 301 302 \
  --output "$GAMEPAD_VLA_WORKSPACE/runs/dpad-ik"
python scripts/run_fixed_dpad.py train --steps 16384 --num-envs 4 --seed 0 --initial-log-std -3 --device cpu \
  --output "$GAMEPAD_VLA_WORKSPACE/runs/dpad-ppo-seed0"
python scripts/run_fixed_dpad.py eval --checkpoint /path/to/policy.zip \
  --eval-seeds 300 301 302 --output "$GAMEPAD_VLA_WORKSPACE/runs/dpad-eval"
```

`--timeline` 读取上游目标记录；缺省使用带中性状态的随机保持序列。控制周期 20 ms，物理步长 0.5 ms，每步 40 次原生求解。评测检查点时必须选择其训练配置；两种目标协议的观测维度不同。新策略默认 `--initial-log-std -3`，初始探索标准差约 0.05，角度残差上限仍为 0.03 rad；`--initial-log-std 0` 可复算原较大探索对照。检查点继续训练保留已学习的标准差，不用该参数覆盖。`--num-envs` 用 SB3 的 `spawn` 子进程并行采样，默认 1；各环境独立保存监视记录，检查点按总采样步数保存。SB3 按完整 rollout 更新，因此实际训练步数可能向上取整，完成数写入运行记录。初次实验可用 CPU，无需占用 GPU；GPU 策略更新只选择空闲 GPU。评测保留每步目标、实际信号、残差、奖励、接触力和时间，未响应事件计为 missed，不从延时统计中悄悄丢弃。

默认配置为 [`fixed_dpad_causal.json`](../../configs/tasks/fixed_dpad_causal.json)：无未来目标、保持时间 0.1–0.6 秒、阻尼试验值 2 N·m·s/rad、计划按压深度 0.75 mm，并用共享参考选择一致 IK 姿态。预知序列对照配置为 [`fixed_dpad.json`](../../configs/tasks/fixed_dpad.json)，保留十步前瞻及阻尼 8。两者的残差范围均为 0.03 rad，方向切换时以实际指尖相对中性方向键表面沿法向抬高 4 mm 为抬起完成条件。该阶段仍按原始目标计分，不修改时间线或排除切换时段。每次复位后先用真实驱动向悬停姿态准备 0.5 秒，再开始目标时钟；准备时长、峰值接触力和初始信号单独记录，避免从全张开状态直接压入第一个方向。桌面摆放是该任务的独立配置，不修改已采用的展示场景。`fit_fixed_dpad_placement.py` 可基于 `inspect` 的手掌位姿重新搜索桌面内平移；运动学搜索结果仍需通过真实接触试验。

首轮实际接触结果见[实验报告](../reports/2026-10-21-fixed-gamepad/README.md)。

## 实际游戏输入与闭环

仓库提供[一段真实 N2 策略目标](../../references/targets/resnake_n2_seed520000/README.md)，完整保留 20 维报告、100 个逻辑 tick 和 10 Hz 原时序。可直接验证回放：

```bash
python scripts/run_fixed_dpad.py ik \
  --timeline references/targets/resnake_n2_seed520000/timeline.jsonl \
  --eval-seeds 0 --output "$GAMEPAD_VLA_WORKSPACE/runs/dpad-game-replay"
```

`record_game_targets.py` 可在游戏项目兼容环境中读取已有检查点，生成其他目标输入。原游戏模型和对应环境不随本项目安装；归档的目标回放不依赖它们。

`run_fixed_gamepad_game.py` 在游戏环境运行，显式传入本项目 MuJoCo 环境的 Python。它启动独立模拟工作进程，将游戏策略的完整目标报告交给手部控制器，并把实际接触状态转换成完整 20 维执行报告后送回游戏。没有用请求报告替代物理反馈，也不向控制器提供未来游戏决策。

```bash
PYTHONPATH="$PWD/src:/path/to/compatible/resnake-checkout/src${PYTHONPATH:+:$PYTHONPATH}" \
/path/to/game/python scripts/run_fixed_gamepad_game.py \
  --game-checkpoint /path/to/game-policy.pt \
  --controller-python "$GAMEPAD_VLA_WORKSPACE/envs/fixed-gamepad-mujoco-py311/bin/python" \
  --controller-checkpoint "$GAMEPAD_VLA_WORKSPACE/runs/dpad-ppo-seed0/policy.zip" \
  --ticks 1000 --seed 520021 --output "$GAMEPAD_VLA_WORKSPACE/runs/dpad-game-loop"
```

父进程通过进程级 `PYTHONPATH` 读取本项目与兼容的 `resnake_gym` 源码，子进程使用专用环境里已安装的固定手柄适配器和本项目；保留各自依赖环境，不向共享游戏环境安装下游依赖。`--controller-checkpoint` 可使用残差策略，省略则运行 IK。输出分别记录请求与执行报告、游戏结果和每个物理控制步。该入口是逻辑时间耦合仿真，既不证明实时吞吐，也不调用采集台硬件。

## 建模与验证边界

- AR5 焊接在工作姿态，手部重力补偿开启；手部自碰撞沿用原采集台关闭设置，手指与手柄、桌面保留接触。G20 碰撞为 URDF 网格的 MuJoCo 凸包近似，手柄使用上游原生 SDF。
- G20 的 `thumb_metacarpals_base2` 惯量违反三角不等式约 0.21%。派生 URDF 将其投影到最近有效惯量，保留质量、惯量轴和几何，记录完整 before/after；源 URDF 不变。没有启用会将三个主惯量全部平均的全局 `balanceinertia`。
- 手部 PD 与力矩上限尚未辨识。任务将每个独立手指关节的力矩上限从原后端占位的 60 N·m 降至试验值 0.3 N·m，后者也不是实测值。数字方向键按上游契约中的试验闭合阈值 0.3 计分。必须同时查看误按、接触力和穿透，不能仅凭 F1 判断动作质量或真机适用性。MuJoCo 手柄的材料和驱动同样未完成实物标定。
- 上游完整手柄测试为 25/26，Pair 仍失败；摇杆瞬态圆锥限位也没有严格不越界保证。这些上游测试不证明 G20 任务成功。
- 当前环境是方向键技能，尚未覆盖摇杆、扳机、ABXY 和双手协调。全手柄任务、APR 对照及实机复现仍需后续实现和验证。

## 记录与回放

闭环命令加 `--record-state` 可记录实际 qpos/qvel 及游戏状态，再运行：

```bash
python scripts/render_fixed_gamepad_replay.py --run /path/to/recorded-run \
  --output "$GAMEPAD_VLA_WORKSPACE/runs/dpad-replay" --software
```

回放显示已记录的碰撞几何与游戏状态，仅读取轨迹并作前向可视化，不重新运行接触轨迹。`--software` 强制 Mesa EGL/llvmpipe，不使用 GPU；另需 Pillow 和可用的 FFmpeg/libx264。视频、预览图和采样索引留在输出目录。导出 XML 会解析组合资产的相对路径，回放旧记录时生成派生 XML，原记录不变。
