# AB：V3 夹具碰撞体与后端

基线 HEAD `5750b93`，上游手柄 0.4.1 / HEAD `73c867636804ef80217c54564bcee01c18c7c695`。以用户追加裁决为准：**手柄固定根，装配位姿直接取 V3；夹具与手柄不计算碰撞。** 不再进行自由根手柄落入夹具的稳定性验收。

SAPIEN 选择 **14 个静态 CAD 三角网格 actor**，与原 834 盒子相同的结构件/软垫覆盖范围；保留全部 37 个 CAD 外观零件。采购螺钉、弹簧等仍是外观参考，没有额外增加其碰撞。MuJoCo 使用按对象拆开的 CAD 多凸块，不能把整件 CAD 网格当成一个凸包。当前采集服务使用 SAPIEN，默认场景和 8780 均未切换。方法 B 已在授权的独立 CAD 环境完成限时扫参，最终仍选 A 的 332 块供 MuJoCo 使用。

## 实现与碰撞语义

- `FixtureSpec` 新增 `type: "convex"`，`mesh` 指向以米为单位的多对象 OBJ，每个 `o` 是一个闭合、有体积的凸块。原 box/mesh spec 保持兼容。
- `type: "mesh"` 仍给 SAPIEN 精确的静态三角网格；可选 `convex_mesh` 指定 MuJoCo 的凸块替代。MuJoCo 对带碰撞且没有 `convex_mesh` 的 mesh 明确报错，避免悄悄填平夹爪间隙。
- SAPIEN 同时修改 `sapien_backend.py` 和实际采集使用的 `collection_sapien.py`。静态网格调用 3.0.3 既有 `add_nonconvex_collision_from_file` / `build_static`；凸块调用 `add_multiple_convex_collisions_from_file(decomposition="none")`。
- MuJoCo 每个 OBJ 对象建立一个 `mesh` asset 和 `mesh` geom。运行时 OBJ 读取只用 NumPy；检查闭合、外向凸性和每块 4–255 个引用顶点。
- **接触偏移全部显式为 0.2 mm**：SAPIEN 在 table、fixture、gamepad、hand、arm 的实际 shape 上设置 `contact_offset=0.0002`，rest offset 保持 0。MuJoCo 相应 geom 设置 `margin=0.0002`；两个引擎合成接触余量的规则不同，不把两者称为数值等价。
- 3.0.3 的 `Entity.find_component_by_type` 必须查询具体 `PhysxRigidStaticComponent` / `PhysxRigidDynamicComponent`。按基类查询会返回空，已用原生实验发现并修复，新增回归测试。

| 配对 | SAPIEN | MuJoCo |
|---|---|---|
| 夹具—手柄 | 共同忽略位 `0x20000000`、共同 filter ID `0x5842` | 夹具 `(contype, conaffinity)=(8,6)`；手柄 `(1,22)`，双向均无共同位 |
| 夹具—左/右机器人 | 允许；机器人原自碰撞忽略位保留 | 机器人类型 2 / 4，affinity 包含 8 |
| 手柄内部 | 保留资产 `0x80000000` 忽略位 | 手柄自身 mask 不匹配，资产内部 body excludes 保留 |
| 桌面 | 保留环境碰撞 | 类型 16、affinity 7，机器人/手柄仍能碰桌面 |

`full` 启用夹具与手/臂接触；**`gamepad-contact` 保留原语义：机器人只碰手柄，桌面和夹具不启用碰撞**。因此夹具阻挡验收在 full 进行，采集性能在 gamepad-contact 进行。MuJoCo 对 gamepad-contact 的原有显式拒绝未在本任务中扩展。

## 方法与精度

方法 A 重放 `fixture.py` 同一套加性盒、柱、棱柱与切除操作；凸体减凸体按切除体面平面划分。与接触无关的小钻孔允许约 1 mm 填平；所有口袋、夹爪、背托平面切除保留。Hirth 移动盘按半齿分块：跨整齿的凸包曾测得 1.64 mm 外胀，已拆开修正。对半径不超过 3.5 mm 的小钻孔使用内接正方形，较大孔使用带 0.5 mm 径向余量的内接八边形；对应的理论径向填平分别不超过约 1.026 / 0.94 mm。这样原始分块从首轮 1012 减到 516。之后在相邻块中贪心合并，以每 mm² 4 点检查新增体积的 CAD 距离和保护空间。为消除布尔切分的数值薄片，凸块顶点以 0.0001 mm 网格舍入后重取凸包；最大舍入距离 0.000086603 mm，接触面误差另外实测。目标是该候选族中减少块数，**没有证明全局最少**。

生成闭合实体的凸覆盖，不分解空心薄壳。源 CAD 表面以最终生产 STL 为独立测量基准；曲面继承 CAD 导出时的 tessellation 精度，不能把对 STL 的误差当作实体制造精度。误差检查沿用上游报告 8.3 的定义：外胀只统计落在 CAD 实体外的凸块表面点，内缩只统计未被凸块覆盖的 CAD 表面点，距离均到三角形而非到采样点。每 mm² 4 个面积采样，加全部顶点/面中心；内缩包含判定使用更严格的 0.00001 mm 容差。**凸块最大值是采样最大值，不是连续最大值证明。** 度量实现另用 10 mm 方块平移 1 mm 作对照，实际测到最大外胀 1.000000 mm。

<!-- AB_METRICS -->
| 方法 | 完成零件 | 块数 | 外胀 max / P95 mm | 内缩 max / P95 mm | 生成任务耗时 s |
|---|---:|---:|---:|---:|---:|
| A：CAD 构造＋合并 | 14/14 | 332 | 1.251921 / 0.649604 | 0.040937 / 0.026383 | 246.13 |
| B：CoACD t=0.005 | 5/14 | 177（部分） | 未完成 | 未完成 | 43.98 + 9×120 超时 |
| B：CoACD t=0.01 | 12/14 | 1609（部分） | 1.248637 / 0.215435 | 0.008301 / 0.001926 | 413.51 + 2×120 超时 |
| B：CoACD t=0.02 | 13/14 | 680（部分） | 3.024797 / 0.591670 | 0.016197 / 0.002250 | 286.54 + 1×120 超时 |
| B：CoACD t=0.03 | 14/14 | 653 | 3.160740 / 0.926172 | 0.009785 / 0.002751 | 350.48 |
| B：CoACD t=0.05 | 14/14 | 301 | 3.867020 / 1.499661 | 0.017134 / 0.003291 | 202.75 |
<!-- /AB_METRICS -->

B 扫参连同上述完整/部分误差统计总耗时 **953.56 s（15 分 54 秒）**，小于 20 分钟。B 的生成列为各已完成单线程任务耗时之和，4 个子进程并行；超时任务另列，不把该列当成整轮墙钟时间。A 为单进程构造与合并的合计。部分行的精度只覆盖已完成零件，不能据此认证整套夹具。详细参数、逐件状态和采样数见 [CoACD 扫参数据](coacd_sweep.json)。

**选择 A：332 块。** B 的完整 0.03/0.05 两组均超过 1.5 mm 外胀硬门；0.05 的下爪还侵入左/右 grip-back 保护体约 14.802 / 14.805 mm³。对所有已量化候选再逐件比较，在满足外胀、接触面、顶点数和保护空间硬门的候选中，每件最少块数仍来自 A（同块数优先 A）。[候选选择记录](selection.json)列出全部资格与拒绝原因。这只证明给定候选集合内的选择，不是所有可能凸分解的全局最优。

方法 B 已获用户授权，仅在 `.local/cad-env` 安装 **CoACD 1.0.14**，与上游版本一致。以总 1180 秒、每个零件至多 120 秒为限，4 个单线程 CPU 子进程扫描；初始 0.005 阈值按默认 20 nodes / 150 iterations 试验，复杂零件触及单件上限后，后续完整对照使用 10 nodes / 60 iterations / depth 3。成功项元数据记录实际参数，较早的试验不混入统一预算的对照。0.005 是探索批次，存在预算切换；没有完整生成元数据的超时项仅记为超时，不据此推定精度或完整阈值结果。原本已经凸的两根导杆、背垫和下垫直接取一块精确凸包；其余闭合实体送入 CoACD。`preprocess=off`、不限块数、seed 0、decimate on、max_ch_vertex=255。原始导杆直接运行细阈值时出现 81/88 块，已记录为首轮结果；未把这种多余分块伪装成必要的几何复杂度。所有候选仍按独立毫米误差和保护空间筛选，不以 threshold 或顶点上限代替硬门。

A 的硬门结果：外胀最大 1.251921 ≤ 1.5 mm；三处接触面最大误差 0.000058704 ≤ 0.1 mm；全部 332 块最多 128 顶点；54 个保护体交叠为零。最终选择资产逐字节等于已经独立验证的 A 候选，没有以未经复验的新组合替换。

## Cooking 与保护空间

静态三角网格的 14 件 cooking 前后，全部三角面可以按顶点一一对应。对应顶点最大变化 **0.000008181 mm**；由三角形重心插值，这也是相对原 STL 的双向连续表面距离上界，接触面远小于 0.1 mm。不是仅凭输入文件相同推断引擎没有改变几何。

凸块输入最多 128 个顶点，cooking 后最多 22 个；**低于 255 仍会被 PhysX 简化**。332 个输入对象全部成为独立形状。按同一对象 cooking 前后的表面面积采样、顶点与面中心测双向距离，最坏变化 **0.700002 mm**（下爪），详见 [逐件 cooking 与本体重叠](cooking_and_overlap.json)。这项误差不是外胀，也不能与 CAD 外胀直接相加当成实测值。当前 SAPIEN 使用三角网格，避开该凸块简化误差；该量化不构成 SAPIEN 凸块后备方案的精度验收。

保护空间复用完整控件关节行程的 17 点采样凸扫掠体、原夹具左右 grip-back keepout，以及任务 Z 的已位移控件接触点/法向，新增半径 1 mm、向外 3 mm 的指尖接触通道，共 54 个体积。逐体积使用闭合实体布尔交集，不把零碰撞 mask 当成几何净空。CAD 三角网格对这 54 个保护体的交叠为零；控件转动采样的弦误差上界另外列出，不能声称穷尽连续关节空间。完整实际手/臂网格的 34 个既有见证及 3 mm 接近包络另见 MuJoCo 结果；它们是 V3 已保留见证的重放，不是重新证明全控件可达。

手柄本体已排除碰撞，其重叠只作外观一致性参考。以原资产闭合 shell/front-plate 凸块的**并集**为基准，压缩垫装配位姿下，刚性 CAD 件重叠为 0；背垫、下垫、上垫分别约 **42.7040 / 8.7153 / 0.9429 mm³**。这是软垫与未实测碰撞底模的几何交叠，不能解释为真实材料压缩量。凸块的对应重叠数值另列于 cooking JSON。54 个保护体还测了截断于 10 mm 的最小间隙；扣除控件转动采样弦误差或 32 边接触通道的圆弧误差后，CAD 和所选 CAD 凸块的最小余量均为 1.0 mm。

## 原生物理验证

计算节点既有 SAPIEN 3.0.3 CPU 环境，1 ms 步长，无新依赖、无 GPU 计算。探针使用真实生产 URDF 中的 `index_distal` 与 AR5 `link3` 凸网格，分别从夹具底板上方自由落下；移除夹具作对照。

| 检查 | 结果 |
|---|---|
| 真实指尖凸网格挡住 | 底板高度 0.783750 m，末态最低点 0.78375006 m；非零支撑冲量 0.2932 N·s；去掉夹具后落至 −0.98099 m |
| 真实臂网格挡住 | 末态最低点 0.78354764 m，约 0.2024 mm 接触穿入；非零支撑冲量 0.3448 N·s；去掉夹具后落至 −0.99103 m |
| A / B / LB 动态小球按压 | 3/3 均达到归一化行程 1.0，与去掉夹具的对照相同；小球不接触夹具 |
| 按压时夹具—手柄接触 | 所有按压步累计 0 对；手柄内部也为 0 对；固定根矩阵变化为 0 |
| 有无过滤的正对照 | 与 SDF 手柄故意重叠的静态盒子，启用过滤 0 对、关闭过滤 340 对，排除“碰撞体本来不支持配对”导致的假通过 |

小球沿用上游 acceptance probe 的动态、限力 PD 方法：4 mm 半径、50 g、3000 N/m、12 Ns/m、16 N 总力上限、5 mm/s。接触起点从 cooked 控件的支持平面取得，未直接写手柄关节或目标。Pair 也做了附加诊断，但有/无夹具均未按动，两组轨迹相同；它不计入成功的三项，不能据此认证 Pair 的按压能力。原始结果保留于 [SAPIEN 结果](sapien_results.json) 和 [过滤正对照](filter_results.json)。

MuJoCo 使用既有 3.16.0 环境，实际编译每个凸块一个 mesh geom；当前采集服务不使用该后端。本任务的 MuJoCo 验证范围为导入、碰撞组正反对照与任务 Z 已有手/臂见证的几何重放，没有声称在 MuJoCo 完成动态按压采集。332 个 mesh geom 实际编译成功；夹具—手柄允许对数为 0，夹具—机器人允许对数为 21080；原生故意重叠盒子对照为过滤 0 / 未过滤 4 对。最终复验直接加载试验 spec 中的 `convex_mesh`，没有临时覆盖路径；编译后全部夹具顶点相对指定世界位姿的双向最近顶点误差最大 0.000009667 mm。计算节点 51 个 GLB/OBJ 的 SHA256 与来源记录全部一致。34 个已保留的手/臂状态及向外 3 mm 的完整凸几何接近包络均无交叠，见 [MuJoCo 原始结果](mujoco_results.json)。

## 性能

CPU 同一节点、同一后端版本、同一过滤/0.2 mm 设置；每配置 3 次构建，100 步预热后测 1000 个 `scene.step()`，下表取中位数。包含整个双臂/双手/手柄工作站，不是孤立夹具微基准。

| 场景 | gamepad-contact 加载 s | gamepad-contact 每步 µs | full 加载 s | full 每步 µs |
|---|---:|---:|---:|---:|
| 原 834 盒子 | 0.49235 | 525.28 | 0.51208 | 545.79 |
| 14 个 CAD 网格 | 0.47515 | 464.89 | 0.45966 | 466.61 |

采集模式两者的环境碰撞 shape 均为 0；full 场景含原有环境时，总 shape 从 843 降至 23。gamepad-contact 加载约减少 3.5%、步进约减少 11.5%；这是三次顺序重复的整体对比，未做统计显著性或独立因果归因。各次结果和 p95 见原始 JSON，首次构建为冷加载，后两次为进程内暖加载；表取三次中位数，原始值均保留，不作纯冷启动比较。

采集台先只读确认会话 idle、无 GPU 计算进程，再用 `nice` 启动各 10 秒的独立测试；只写 `~/gamepad_vla`，端口 **18980**，无 ROS 输入、无录制、无硬件命令。复用现有 `OperatorView` 的 camera_3（1280×720）+ overview / mmap / NVENC，30 Hz 发布中立完成态、配置渲染 60 Hz。丢弃启动后前 3 个状态样本，编码状态 fps 中位数：盒子 **60.27**，网格 **60.37**；两者均维持约 60 fps。apply-to-encode p95 分别约 57.45 / 64.86 ms，因此没有声称延迟也变好。未连真实浏览器，fps 指服务器编码输出；完整状态见 [流性能结果](stream_results.json)。测试结束关闭独立子进程，原 8780 服务保持 idle 和原场景。

## 复现与文件

```bash
# 既有 CAD 环境；本任务仅在此环境新增 coacd==1.0.14
.local/cad-env/bin/python hardware/gamepad_fixture/convex_collision.py --method cad
.local/cad-env/bin/python hardware/gamepad_fixture/merge_collision.py --dest .local/fixture-convex/cad_adaptive_merged --seconds 30
# 合并受单件时间上限影响，块数可能随机器变化；以最终目录独立验收
.local/cad-env/bin/python hardware/gamepad_fixture/check_collision.py .local/fixture-convex/cad_adaptive_merged --density 4
.local/cad-env/bin/python hardware/gamepad_fixture/scan_coacd.py
.local/cad-env/bin/python hardware/gamepad_fixture/select_collision.py
.local/cad-env/bin/python hardware/gamepad_fixture/scene_export.py --convex-dir .local/fixture-convex/selected
.venv/bin/python -m pytest tests/test_fixture_collision.py tests/test_collection.py tests/test_operator_collection.py tests/test_workstation_spec.py tests/test_workstation_v4.py -q
```

`gamepad_fixture_v3.json` 是新试验场景，`gamepad_fixture_v3_boxes.json` 保留原 834 盒子对照。运行资产依旧位于工作空间 `assets/derived/gamepad_fixture_v3`；OBJ/GLB 不强行纳入 git。源码整树同步使用 `scripts/station/sync_tree.py` 并核验 SHA256；忽略的派生资产另传入隔离测试目录。61 项相关单元回归通过，Python 编译检查与 `git diff --check` 通过。没有修改兄弟仓库或共享 Python 环境，没有 commit，也没有发送物理命令。

参考依据：[上游物理资产报告 2.4、3.2、7、8，尤其 8.2 表5](../../../../xbox_gamepad_physics/docs/reports/xbox_one_s_physics_asset_report.html)、[上游碰撞优化约定](../../../../xbox_gamepad_physics/docs/collision_optimization.md)、[SAPIEN 官方 3.0 PhysX API](https://sapien-sim.github.io/docs/api/sapien.pysapien.physx.html)、[MuJoCo 官方 mesh / collision mask 定义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh)、[CoACD 原始实现与参数](https://github.com/SarahWeiii/CoACD)。CoACD 阈值和顶点上限不能替代毫米误差和保护空间检查，这一点沿用上游 8.3–8.5 的证据边界。
