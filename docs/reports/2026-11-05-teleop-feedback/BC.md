# B/C：夹具通路与摇杆误按的实际测量

## 执行范围与数据

本轮在 scalelab01 的独立 `runs/teleop-feedback-ae-bc-20261105/` 中完成 CPU PhysX / lavapipe 实验，SAPIEN **3.0.3**、手柄资产包 **0.4.1**；没有连接采集台、使用 CUDA、安装依赖、修改 CAD 或生产增益。代码、姿态、实际 cooked 网格和图像都来自本仓库现有管线。环境及 14 件网格散列见 [环境摘要](environment-summary.json)。

静态测量重放 v3 报告保留的 **34 个控件状态和 4 个握姿**，另加旧报告拒绝的左 Dpad_down 见证作诊断，共 **39 条**。34 条不是 34 个不同控件：包括严格/现实两种口径及左摇杆八向的重复状态。既有状态来自任务 Z，再经 v3 的臂 IK 迁移；本轮不做新的姿态搜索，也没有更改原覆盖分母。

正式图片覆盖已保留的左 Dpad_up/left/right、LS、L3，右 A/B/X/Y、R3、RB。左 Dpad_down 单列旧拒绝见证；Z/v3 尚未通过的左 LB/LT、右 RS/RT 没有新造“通过姿态”，也没有由本轮静态截图推断为不可达。

离线图由 `SapienBackend → robotwin.create_scene/load_into_scene → operator_view.apply_render_state → scene_snapshot.snapshot_cameras/render_pending_snapshot` 生成，使用原有 appearance 和 camera_3；没有第二套场景或渲染器。**19 组、133 张 1280×720 独立原图均已浏览**，并放大检查 RB、Dpad_right、Dpad_down 等关键原图。按收尾要求，公开版保留其中 77 张姿态图与 8 张侧视对照图；保留姿态图的尺寸、散列和内外参见 [图像摘要](image-summary.json)。完整原图、原始轨迹和实验脚本移到 `.local/teleop-feedback-ae-20261105/full-report/`，计算节点原工作目录也保留，见[总报告归档说明](README.md#归档与复核)。

下表带链接的视角是公开保留图，无链接的视角原图在完整归档中；逐图观察文字保留。

静态图中，未操作的一侧设为其已接受现实握姿，以便看双手接近区；这是视觉组合，**不把分别通过的单手姿态写成双手连续操作或双手无碰撞证明**。距离实验按被检查的一侧、原来的另一侧 home 重放；C 的动态图同样保持另一侧 home。

## B：逐图观察

所有组均逐格查看 camera_3、face、front、back、left、right、overview。下表的每个链接对应一张独立原图；距离一栏来自 SAPIEN 网格计算，不靠像素估算。图中左右按相机画面变化，表中的左右始终指机器人侧。

| 冻结姿态 | camera_3 / 正面观察 | 前方观察 | 背面观察 | 最终左右斜侧观察 | overview / 实测最小夹具距离 |
|---|---|---|---|---|---|
| 左严格握姿 | [主视角](images/left_strict_hold_-1/camera_3.png)中手柄偏小；[face](images/left_strict_hold_-1/aux_face.png)可见双拇指互相遮挡中部按键 | [front](images/left_strict_hold_-1/aux_front.png)可看接近高度，但拇指仍遮按键 | [back](images/left_strict_hold_-1/aux_back.png)清楚看无名指/小指绕握把与中央导轨分离 | [left](images/left_strict_hold_-1/aux_left.png) / [right](images/left_strict_hold_-1/aux_right.png)可见夹爪侧缘、导轨与手背关系 | [俯视](images/left_strict_hold_-1/aux_overview.png)适合看双臂整体；≥20 mm |
| 左现实握姿 | 主 / [正](images/left_realistic_hold_-1/aux_face.png)：与严格握姿基本一致 | [前](images/left_realistic_hold_-1/aux_front.png)：拇指跨按键面 | [背](images/left_realistic_hold_-1/aux_back.png)：未见手指穿导杆 | 左 / 右：中央机构占据背部通路，但当前手指在外侧 | 俯：≥20 mm |
| 左 Dpad_up | 主 / [正](images/left_realistic_Dpad_up_0/aux_face.png)：拇指末节移至十字键上方 | [前](images/left_realistic_Dpad_up_0/aux_front.png)：拇指与下爪前唇上下关系比主视角清楚 | [背](images/left_realistic_Dpad_up_0/aux_back.png)：握把支撑手指基本不变 | 左 / 右：下唇仍位于拇指接近区下方 | 俯：10.874 mm，拇指末节—下爪 |
| 左 Dpad_left | 主 / [正](images/left_realistic_Dpad_left_0/aux_face.png)：拇指挡住部分十字键 | [前](images/left_realistic_Dpad_left_0/aux_front.png)：拇指较上方向更靠下唇 | [背](images/left_realistic_Dpad_left_0/aux_back.png)：导杆不接近无名指/小指 | 左 / 右：看得到唇缘，不能仅凭图判断毫米净隙 | 俯：6.278 mm，拇指末节—下爪 |
| 左 Dpad_right | 主 / [正](images/left_realistic_Dpad_right_0/aux_face.png)：拇指向十字键中部横跨 | [前](images/left_realistic_Dpad_right_0/aux_front.png)：下唇最接近已接受的 Dpad 拇指路径 | [背](images/left_realistic_Dpad_right_0/aux_back.png)：后方支撑指未进入中央导轨 | 左 / 右：侧视有助识别下唇所在层 | 俯：**4.565 mm**，拇指末节—下爪前唇 |
| 左 LS，第 0 向 | 主 / [正](images/left_realistic_LS_0/aux_face.png)：拇指移向左杆顶面 | [前](images/left_realistic_LS_0/aux_front.png)：能分辨指腹与杆帽的上下关系 | [背](images/left_realistic_LS_0/aux_back.png)：支撑指位置基本不变 | 左 / 右：未见夹爪接触拇指 | 俯：19.424 mm |
| 左 LS，第 6 向 | 主 / [正](images/left_realistic_LS_6/aux_face.png)：拇指顶在已倾斜的杆帽附近 | [前](images/left_realistic_LS_6/aux_front.png)：拇指以较陡角度接近杆帽 | [背](images/left_realistic_LS_6/aux_back.png)：导轨没有靠近握把支撑指 | 左 / 右：看得到指腹—杆帽关系，不能从图认证 L3 行程 | 俯：≥20 mm；此姿态用于 C 的误按复现 |
| 左 L3 | 主 / [正](images/left_realistic_L3_0/aux_face.png)：姿态与推杆图非常相近 | [前](images/left_realistic_L3_0/aux_front.png)：0.4 mm 轴向行程无法靠普通画面可靠识别 | [背](images/left_realistic_L3_0/aux_back.png)：支撑指未靠导杆 | 左 / 右：需结合连续信号判定点击 | 俯：19.387 mm |
| 右严格握姿 | 主 / [正](images/right_strict_hold_-1/aux_face.png)：右拇指覆盖中部按键区 | [前](images/right_strict_hold_-1/aux_front.png)：右拇指横跨面板 | [背](images/right_strict_hold_-1/aux_back.png)：右无名指/小指在右握把外侧，未夹导轨 | 左 / 右：背托/夹爪位置可辨 | 俯：≥20 mm |
| 右现实握姿 | 主 / [正](images/right_realistic_hold_-1/aux_face.png)：与严格握姿接近 | [前](images/right_realistic_hold_-1/aux_front.png)：拇指遮挡持续存在 | [背](images/right_realistic_hold_-1/aux_back.png)：中央弹簧与两侧指头分离 | 左 / 右：可看中央机构，不再只有手背 | 俯：≥20 mm |
| 右 A | 主 / [正](images/right_realistic_A_0/aux_face.png)：右拇指下移至 A 区 | [前](images/right_realistic_A_0/aux_front.png)：拇指与下爪前唇分离 | [背](images/right_realistic_A_0/aux_back.png)：右支撑指与背托没有新的接近 | 左 / 右：夹爪未进入拇指路径 | 俯：≥20 mm |
| 右 B | 主 / [正](images/right_realistic_B_0/aux_face.png)：右拇指移向面板右缘 | [前](images/right_realistic_B_0/aux_front.png)：指头不在中央夹爪上方 | [背](images/right_realistic_B_0/aux_back.png)：导轨关系不变 | 左 / 右：未见新的夹爪接近 | 俯：≥20 mm |
| 右 Y | 主 / [正](images/right_realistic_Y_0/aux_face.png)：右拇指上移至 Y | [前](images/right_realistic_Y_0/aux_front.png)：能看见上沿与拇指的高差 | [背](images/right_realistic_Y_0/aux_back.png)：食指仍留在肩部上方 | 左 / 右：未见与上爪相接 | 俯：≥20 mm |
| 右 R3 | 主 / [正](images/right_realistic_R3_0/aux_face.png)：右拇指移至右杆 | [前](images/right_realistic_R3_0/aux_front.png)：下唇与右杆通路可同时看 | [背](images/right_realistic_R3_0/aux_back.png)：支撑指仍在机构两侧 | 左 / 右：没有可见穿入 | 俯：12.446 mm，右拇指—下爪 |
| 右 RB | [主](images/right_realistic_RB_0/camera_3.png) / [正](images/right_realistic_RB_0/aux_face.png)：食指沿肩键伸向上沿 | [前](images/right_realistic_RB_0/aux_front.png)：食指和上爪外角处于同一接近区 | [背](images/right_realistic_RB_0/aux_back.png)：能明确看到食指末节靠上爪右上角；这是本组最紧通路 | [左](images/right_realistic_RB_0/aux_left.png) / [右](images/right_realistic_RB_0/aux_right.png)：可辨上爪侧缘 | [俯](images/right_realistic_RB_0/aux_overview.png)：**3.270 mm**，右食指末节—上爪 |
| 右 X | 主 / [正](images/right_realistic_X_0/aux_face.png)：拇指向左上跨面板 | [前](images/right_realistic_X_0/aux_front.png)：面板中央被两拇指进一步遮挡 | [背](images/right_realistic_X_0/aux_back.png)：夹爪外侧支撑指没有进入导轨 | 左 / 右：需与 face 联用辨别按键接近 | 俯：18.514 mm |
| 左旧拒绝 Dpad_down | [主](images/left_rejected_Dpad_down/camera_3.png) / [正](images/left_rejected_Dpad_down/aux_face.png)：拇指往下方按，贴近下爪唇 | [前](images/left_rejected_Dpad_down/aux_front.png)：拇指末节在下唇正上方，普通像素不足以看出亚毫米间隙 | [背](images/left_rejected_Dpad_down/aux_back.png)：导杆不是这个见证的最近障碍 | [左](images/left_rejected_Dpad_down/aux_left.png) / [右](images/left_rejected_Dpad_down/aux_right.png)：可定位到前唇层，不能仅凭图写“穿透” | [俯](images/left_rejected_Dpad_down/aux_overview.png)：**0.214 mm**，本轮实际静态网格为小正隙 |

两个 C 动态末态也逐幅查看：[240/60 的 face](images/dynamic_LS6_k240_limit60.0/aux_face.png)、[front](images/dynamic_LS6_k240_limit60.0/aux_front.png)、[back](images/dynamic_LS6_k240_limit60.0/aux_back.png)、[left](images/dynamic_LS6_k240_limit60.0/aux_left.png)、[right](images/dynamic_LS6_k240_limit60.0/aux_right.png)、[主](images/dynamic_LS6_k240_limit60.0/camera_3.png)、[俯](images/dynamic_LS6_k240_limit60.0/aux_overview.png)；[60/0.3 的 face](images/dynamic_LS6_k60_limit0.3/aux_face.png)、[front](images/dynamic_LS6_k60_limit0.3/aux_front.png)、[back](images/dynamic_LS6_k60_limit0.3/aux_back.png)、[left](images/dynamic_LS6_k60_limit0.3/aux_left.png)、[right](images/dynamic_LS6_k60_limit0.3/aux_right.png)、[主](images/dynamic_LS6_k60_limit0.3/camera_3.png)、[俯](images/dynamic_LS6_k60_limit0.3/aux_overview.png)。两者都能看见拇指停在杆帽附近；画面整体很相近，却有明显不同的实际杆角与点击量，说明需要信号和接触力，不能以姿态截图证明控制成功。

### 侧相机确实经过实际画面修改

原局部位置 `[±.34,0,.12]` 在双手握姿下几乎只看到同侧手背，见 [原左](images/side-comparison/old_left.png)、[原右](images/side-comparison/old_right.png)。比较了三组斜侧候选：`[±.34,.15,.28]`（[左](images/side-comparison/upper_left.png)、[右](images/side-comparison/upper_right.png)）、`[±.34,.24,.20]`（[左](images/side-comparison/rear_left.png)、[右](images/side-comparison/rear_right.png)）、`[±.34,.06,.38]`（[左](images/side-comparison/high_left.png)、[右](images/side-comparison/high_right.png)）。第二组更清楚展示导轨、弹簧和夹爪侧缘，已用于最终 helper，并重渲以上所有正式侧图；旧图只作对照。

### 静态网格最小间隙与碰撞对

对每个保存姿态，读取 SAPIEN **实际 cooked** 机器人碰撞凸网格/盒及 14 个夹具静态三角网格。计算三角面之间的点—面、边—边距离及边—三角形相交，AABB 筛选范围 20 mm；`≥20 mm` 是截断下界，不是精确等于 20 mm。另在设置该姿态后执行一拍 PhysX，读取接触对和 separation。39 个姿态的最近距离、接触对及各夹具跨姿态最小值见 [间隙摘要](bc-summary.json)，最近三角面在手柄坐标中的位置见 [closest-triangles.json](closest-triangles-summary.json)。

| 对象/姿态 | 最小表面距离 | 最近部件 | PhysX 几何接触 |
|---|---:|---|---|
| 34 个已保留控件状态 + 4 个握姿 | 最小 **3.2695 mm** | 右 RB：`right_index_distal` ↔ `slider_upper_jaw` | 全部没有夹具—被检查侧机器人的接触候选 |
| 左 Dpad_right | **4.5653 mm** | `thumb_distal` ↔ `cradle_lower_jaw` 前唇 | 无 |
| 左 Dpad_left / Dpad_up | 6.2782 / 10.8741 mm | 同上 | 无 |
| 右 R3 | 12.4456 mm | 右拇指末节 ↔ 下爪 | 无 |
| 上/下软垫，所有已接受状态 | 最小 7.4486 / 14.2690 mm | 软垫与手的关系 | 无 |
| 背垫、导杆两根、导杆帽、叉耳所在 column、齿盘、旋钮 | **全部 ≥20 mm** | 对本轮已接受单手状态 | 无；不支持“这些部件已经撞手”的说法 |
| 旧拒绝 Dpad_down 诊断见证 | **0.2136 mm 正隙** | 左拇指末节 ↔ 下爪前唇 | 1 对候选，最小 separation **+0.1711 mm**，不是穿透 |

旧 v3 报告的 Dpad_down 约 0.11 mm 穿透来自当时的盒子碰撞近似。本轮采用新 14 件实际静态网格，结果不同；不能把旧盒子穿透照抄成新网格事实。0.214 mm 小于两形状 contact offset 之和 0.4 mm，因此有正 separation 的候选接触并不矛盾。这也不能自动把旧覆盖表升级为 Dpad_down 已通过：还没有重做完整 Z 接触/控件行程谓词和动态轨迹。

**确认的局部问题**是下爪前唇占据 Dpad 拇指接近层、上爪右上角压缩 RB 食指通路。被测已接受姿态没有碰夹具。用户实际遥操“很容易被挡住”的全部发生路径未录到，当前证据不能把它归结为某个部件在所有握姿必撞；末态无碰撞也不证明到达末态的路径无碰撞。

### v3.1 瘦身建议：只估算，不改 CAD

优先局部退让，不整体缩小承压垫、背托、导轨、叉耳或 Hirth 锁紧结构。

| 顺序/部件 | 建议尺寸变化 | 预期收益与代价 | 保留的硬门及验证 |
|---|---|---|---|
| 1. 下爪前唇，Dpad 拇指通道 | 先在局部 `x≈−26…−12 mm` 区域，把前唇**向前突出端 z=26 退至 24 mm**，保持其起始 z=22、下承压垫 z=−8…14 与完整 56 mm 垫宽；入口用 ≥2 mm 圆角/斜面 | 为左拇指提供最多 2 mm 的 z 向退让；不会按这个改法减小原承压足迹。剩余前唇层厚 2 mm，仍高于既有 0.84 mm 打印壁厚门。防脱搭接减少，必须验证，不能写成已达标 | 下沿定位面和抗扭软垫不切；复核全部控件、接口、局部壁厚/悬垂及 4 N·m 拉拔/防脱工况。先单独比较新旧静态最小距离与 3 mm 接近包络，再做实物夹紧和拉拔试验。若 2 mm 余唇不满足防脱，不采用该切法，改从指腹方向找路径 |
| 2. 上爪右上外角及其左对称位置 | 在 `x=±18…±20 mm、y≈52…55.62 mm` 的**非承压外角**先做 2 mm 倒角，保留 y≈48.62 的上沿完整垫片窗口、Pair/USB 缺口和上爪 40 mm 名义承压宽 | 针对 RB 最近三角面（外角约 x20、y55.62），提供最多约 2 mm 的横向局部空间；实际最近距离需在改后 CAD 重算，不把 3.27 直接加 2 当结论 | 不切软垫槽底、弹簧座、M4 锁紧和导轨截面；原接触积分与锁紧载荷路径应保留。逐件打印门、最小连接截面、弹簧/锁紧全行程及抗扭复验通过才可采用 |
| 3. 背托、导杆、叉耳、齿盘与旋钮 | **本轮建议不缩**：背托宽 44、导杆直径 6、双剪 M8、r18…30 的 36 齿 Hirth 及其预紧参数维持 | 本轮各自 ≥20 mm，没有证据证明它们是这些见证的近障碍。为瘦而减小会付出实质刚度、锁紧或打印代价 | 尤其不削薄叉耳或改小齿盘。若今后实录路径显示具体阻挡，再针对该路径测量，不能用主观“太大”取消既有门 |

按上述局部切法，保持实际接触垫足迹和预紧力，原抗扭估算的输入不变；这是**设计约束下的预估，不是新 CAD 已经通过**。v3 可用接触面估算约 0.44879 N·m，对 0.4 N·m 只有约 1.122 裕量，不宜整体减小承压宽。背垫刚度近似与有效宽度三次方成正比：外宽从 44 改到 36 mm，相应有效宽 40→32，刚度约乘 **0.512**、0.4 N·m 转角由 0.663° 增至约 **1.294°**。这个代价没有被本轮 ≥20 mm 的背托间隙证据支持。

完整采纳前的最小验证是：在用户确认后才修改 CAD；固定原预紧/摩擦/材料假设，重新跑原 v3 抗扭、锁紧、壁厚/窄缝/打印方向硬门和新 14 网格距离/接近包络对照；任何一门不通过就拒绝该候选。材料摩擦、软垫压缩、蠕变和实打承载仍需按原 v3 验证单实测，本轮没有将其提升为实物认证。

## C：摇杆按下条件与轴向分力

### 运动契约与“触发阈值”必须分层

兄弟仓库只读检查版本 **0.4.1**。`build_data/references/full_controls.json`、`backends/common/physical.py` 和生成的 `resource_data/mujoco/full_controls/xbox_one_s.xml` 一致：

- 左/右点击是 `left_stick_click` / `right_stick_click` 的独立直线关节，**0…0.0004 m（0.4 mm）**。
- 压入方向是摇杆自身倾转坐标系的 **−z**；点击关节在 `*_stick_y_frame` 下，压轴随 x/y 倾角转动。本实验从实际 SAPIEN 点击关节框架求其 world x 轴（SAPIEN 的关节运动轴），不使用手柄根的固定 −z。
- 线性返回刚度 **18500 N/m**、阻尼 **12.0433 Ns/m**、返回驱动力上限 **10.4 N**；返回目标为 **−6 µm**（1.5% 行程预紧）。10.4 N 是该返回驱动上限，不是外界接触力的上限；到底后的关节限位还能承受额外反力。
- 资产输出点击量为连续的 `q / 0.0004`，没有一个覆盖所有消费者的唯一数字闭合阈值，也没有校准的机械突跳/迟滞曲线。
- 采集 SVG 的默认 `digitalThreshold=20%`（`collection_web/overlay/overlay-app.js`）是**显示按下**阈值；任务 Z 的目标状态用 **80% 行程**；常用 95% 到底验收是另一口径。它们不能互换，episode 中原始连续信号仍原样保存。

| 点击量口径 | 点击行程 | 仅弹簧准静态反力 `18500(q+6µm)` |
|---|---:|---:|
| 页面 20% 显示按下 | **0.080 mm** | **1.591 N** |
| 50% 对照 | 0.200 mm | 3.811 N |
| Z 的 80% 目标 | 0.320 mm | 6.031 N |
| 95% 到底对照 | 0.380 mm | 7.141 N |
| 100% 满行程 | 0.400 mm | 7.511 N |

该表不含重力、阻尼、其他接触和限位反力。20% 亮起明显早于参考元件的名义 7.4 N，不能将每一次亮起都称为实物电气触点闭合。

### 动态力测：初始 L3 为零，不继承点击目标

使用左 LS 的 8 个已接受 Z→v3 推杆目标，1 ms CPU PhysX；每例新建后端、reset，设置已接受握姿，**手柄所有关节从零开始**；之后仅插值机器人 drive target，200 ms 握持、1000 ms 握姿→推杆目标、500 ms 保持。期间不写手柄 qpos 或点击目标。初始状态复核见 [forces-initial-state.json](bc-summary.json)：两组复测中初始每个手柄关节均为 **0**。

48 例扫描最初使用 full 环境碰撞。为对应实际遥操的 **gamepad-contact**，又对第 6 向生产值和最软组合各做有/无环境碰撞对照：[生产碰撞模式](bc-summary.json)、[full 对照](bc-summary.json)。四例均无被检查侧机器人—夹具/桌面的几何接触；相同增益的轴向力、点击量与杆角的全部抽样值逐点相同（最大差 **0**）。所以该第 6 向误按复现不依赖夹具/桌面引导。没有将另外七向的 full 扫描泛称为已逐项做过生产模式对照。

从每一拍真实手—杆帽接触的 `impulse / 0.001 s` 得到力，对该拍实际点击压轴投影。接触来源是 **thumb_distal**。本轮的命令插值是诊断轨迹；任务 Z 验收的是静态接触见证，没有验收这条动态插值。右侧没有已通过的 RS 推杆八向见证，所以本轮**没有捏造右 RS 动态结果**；R3 的关节契约与左侧相同，实测力与成功率不从左侧外推。

`HAND_GAINS.force_limit=60` 施加在 G20 **转动关节**，单位是每关节 **N·m**，不是指尖 60 N。关节刚度 240 对应 N·m/rad。力测没有把这个数直接当下压力。

以第 6 向（目标 `left_stick_x≈0, left_stick_y=−0.321141 rad`）最明显。下表为同目标/同插值、只改变列出的增益与上限，阻尼仍为 8。

| 手关节刚度 / 力矩上限 | 轴向接触峰值 N | 最后 500 ms 轴向力中位 N | L3 峰值 / 稳态比例 | 实际稳态 x / y 倾角 rad |
|---|---:|---:|---:|---|
| **240 / 60（生产值）** | **21.632** | **10.807** | **1.000 / 1.000** | −0.4014 / −0.1777 |
| 240 / 1 | 13.299 | 8.410 | 1.000 / 1.000 | −0.3769 / −0.1804 |
| 240 / 0.3 | 5.428 | 2.906 | 0.673 / 0.408 | −0.2709 / −0.1150 |
| 120 / 60 | 14.996 | 8.182 | 1.000 / 1.000 | −0.2921 / −0.1751 |
| 60 / 60 | 6.475 | 4.773 | 0.948 / 0.791 | −0.1750 / −0.0571 |
| 60 / 0.3 | 4.053 | 1.120 | 0.506 / 0.151 | −0.0692 / **+0.0333** |

单独降低 stiffness 或 force_limit，均降低了这个复现中的轴向力；只降到 1 N·m 仍满按，不能称为解决。最软组合稳态小于页面 20% 阈值，但瞬时仍越过 20% 和 50%，并且实际 y 角反向，**不能当成有效推杆方案**。生产值也没有准确达到指定倾角，故本轮证明了“同推杆目标可产生足以按下 L3 的轴向接触”，不是八向操控成功。

在生产增益的八例中，第 5/6/7 向测到杆帽接触；其余五向没有杆帽接触、杆未按下，不能算推杆成功。第 5/7 向峰值分别 **3.498 / 2.811 N**，峰值点击量 **0.402 / 0.368**，足以触发页面 20%，却没有达到 Z 的 80%。第 6 向满行程，单纯抬显示阈值不能消除该例的真实模拟按压。

完整 **48 例（8 向×6 组）**的指标见 [力测摘要](bc-summary.json)；每 10 ms 的有界采样轨迹在完整归档中。初轮 24 例及全部 10 ms 采样轨迹保存在完整归档中；表内单因素结论使用后续 48 例扫描。

### 与实物 Xbox One S 的比较

资产引用的 [ALPS Alpine RKJXV122400R 原厂数据](https://tech.alpsalpine.com/e/products/detail/RKJXV122400R/)给出推按力 **7.4±3 N**、行程 **0.4(+0.5,−0.3) mm**。这与资产选取的名义力/行程来源一致。本轮没有找到 Microsoft 针对 Model 1708 整机 L3/R3 的公开推按力曲线；资产自身来源记录也明写 **Xbox component match unconfirmed**。因此这是参考元件数据，不能写成用户这只 Xbox One S 已实测或已确认采用该精确料号。

当前模型用线性弹簧把满行程与名义推力关联，没有真实触点突跳或载荷/卸载迟滞。生产增益复现的稳态 **10.807 N** 已略高于参考元件公开上界 **10.4 N**，峰值更高；这说明接触策略本身可以产生较大压入力，不能只靠猜测“模拟弹簧过软”解释。

### 结论与按代价排序的修复选项（未改生产值）

已确认两件不同的事：**页面默认 20% 会把约 1.6 N 等效弹簧压入显示成按下**；**实际推杆接触在某些目标/接近轨迹下会产生足够压满 L3 的轴向分力**。高增益/高力矩上限是本轮可控影响因素，姿态和接近路径也明显影响结果。没有用户首次遥操的逐帧状态/接触记录，尚不能量化这几个因素在那次反馈中的占比。

| 代价 | 选项 | 本轮证据 / 对一致性的影响 | 最小后续验证 |
|---|---|---|---|
| 低 | 将**操作者显示**的 L3/R3 数字着色阈值单独提高到待校准值，同时保留连续信号 | 可消除第 5/7 向的“浅按就亮”问题；不改变物理或 episode。80%/95% 都不是实测闭合点，不能假称更真实；第 6 向满按仍会亮 | 固定同一离线轨迹，只改变显示阈值，对比浅压、故意点击与推杆三组的误报/漏报；用实物 USB 点击状态与位移同步测量后选阈值 |
| 低至中 | 按手/拇指单独降低刚度、阻尼或力矩上限，先做明确的短离线对照 | 单变量实验确实降低轴向压力，但可能丢接触、倾角不足甚至反向；不能直接采用本表最软组合。它改变执行器顺应性，只有与实机手响应对齐时才改善一致性 | 固定起点与接近轨迹，逐项扫参数，必须同时测真实倾角误差、接触保持、L3 峰值、故意 L3 成功与全控件操作，不以“没按下”单指标选择 |
| 中 | 改拇指接近方向/杆帽侧缘接触点，控制切向推力，限制轴向分量 | 从接触几何直接减少压入，同时保留资产机械参数；更有利于把策略迁到实物。新策略未实现，现有 Z 末态不能证明其动态可行 | 同样的杆角目标，比较杆帽顶面与侧缘接近，保持增益相同，记录实际倾角和点击压轴接触力；若轴向力不减或倾角误差增加，拒绝该路径 |
| 中 | 若需要采集侧数字 L3/R3 标签，显式设置独立阈值/迟滞，保留原连续量 | 这是采集解释规则；不应静默覆盖原始 episode 数据。提高阈值可隐藏浅压，却不能改变真实压入力，也未必对应实物闭合 | 先测实物加载/卸载闭合位置与重复性，再离线重算标签并比较误报/漏报；本轮不改 schema 或生产参数 |
| 高 | 给资产提出带电气触点/突跳/迟滞或标定弹簧的建议，必要时校正 click 轴/行程 | 需要用户目标型号的实际数据；直接增大刚度/行程只会让仿真更难按，可能降低实物一致性。不给兄弟仓库直接提交参数“修复” | 力传感器+位移计缓慢加载/卸载，中心和多种倾角各重复至少 10 次，USB 同步读取 L3/R3；拟合力—位移、闭合/释放阈值，再用独立试次验证 |

## 复现入口与有限写盘

以下复现命令在计算节点保留的完整代码副本中执行；公开精简目录不含实验脚本或原始数据。本地归档脚本应先恢复到隔离代码副本的原报告相对位置，再运行。使用既有 SAPIEN 环境，设置 `GAMEPAD_VLA_WORKSPACE`、`XBOX_GAMEPAD_PHYSICS_RESOURCES`、`PYTHONPATH=<副本>/src`，`CUDA_VISIBLE_DEVICES=''`、`GAMEPAD_VLA_RENDER_DEVICE=''`、`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json`，并限制 BLAS/OMP 线程为 1。不要设置 `GAMEPAD_VLA_RENDER_DEVICE=cpu`，该字符串不是 SAPIEN 3.0.3 有效设备选择器。

```bash
# 在计算副本根目录；AE_OUT 是本次独立输出目录。
P="$GAMEPAD_VLA_WORKSPACE/envs/sapien-3.0.3-py310/bin/python"
SPEC=configs/workcell/placement_trials/gamepad_fixture_v3.json
SCRIPT=docs/reports/2026-11-05-teleop-feedback/bc_experiments.py
timeout 3000 nice -n 10 "$P" "$SCRIPT" clearance --spec "$SPEC" --out "$AE_OUT/clearance"
timeout 3000 nice -n 10 "$P" "$SCRIPT" forces --spec "$SPEC" --out "$AE_OUT/forces"
timeout 3000 nice -n 10 "$P" "$SCRIPT" render --spec "$SPEC" --out "$AE_OUT/render"
```

侧视位置比较脚本为 `bc_side_views.py`（完整归档），最终侧图及两个实际动态末态取证为 `bc_final_sides.py`（完整归档），最近三角面定位为 `bc_details.py`（完整归档）。后两者按本次独立运行目录组织输入；重用时先按脚本约定放置已有结果。它们都是本任务报告脚本，不接入生产循环。实验固定为有限姿态数、每例 1.7 s、最多 170 个抽样点及有限张 PNG；生产逐帧日志和 episode schema 没有新增写盘。
