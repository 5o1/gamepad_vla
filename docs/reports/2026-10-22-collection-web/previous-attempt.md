> 历史记录：以下是裁决 3/4 修复前的失败报告，已由 [最终报告](README.md) 和 R4-N2.json 取代；仅保留尝试与失败证据。

# 采集网页 v1（任务 N）

**N1 通过；N2 在安全替身退出后的恢复检查失败，已停止，没有重跑。任务 N 尚未完成验收。** 代码、测试及操作文档已交付，不能将本报告理解为可投入现场采集的通过结论。未 commit。

## 审阅者裁决

裁决 4：完整性结果必须绑定 episode 编号；新段开始清除旧“通过”，停止后明确保存中→检查中→通过/失败。status.integrity 含 episode_id、state（pending/running/passed/failed）；CLI 和驱动按停止返回的具体编号等待，仅接受该段的终态，超时失败。补连续两段期间页面/status/CLI不误报的测试，再从新目录完整重跑 N2；此前300s只作参考。刚启动的复验已收到中断信号，先清理后实施本裁决。

裁决 3（续做）：确认误报源于整条命令行子串匹配。按 argv 的程序身份、路径组成部分或 ROS 包名前缀匹配；排除 shell、搜索工具和终端沙箱包装器；检测周期改为 1 s。先在本地/计算节点、再在采集台执行真实形态 Python 替身阳性与 shell 文本阴性对照。修复驱动的异常清理。先在无遗留进程状态复测推流，若仍偏低则依据剖析定位；不改变物理、相机渲染、分辨率或目标帧率。随后使用新输出目录完整重跑全部 N2，不复用旧部分结果。本步修正预算 3 次；不 commit。


2026-10-22 裁决 1：新增 REST 控制。常驻入口为 `scripts/collection_server.py`，网页与标准库客户端共用 `/api/v1/*`；只绑定回环地址。保留此前画面连接修复与测试修正。API 实现幂等停止、重复开始 409、ROS 四路健康检查、显式 force 元数据、安全警告、客户端审计及跨站请求防护。

用户追加：SSE 推送状态，跨客户端操作后网页更新 P95 ≤0.5 s；显示最近 10 条控制事件。远程写控制每次启动默认关闭，只能由网页会话或显式启动参数开启，HttpOnly、SameSite=Strict cookie 区分网页。该开关防误操作，不能对抗刻意绕过。文档以 Codex 为读者，在 AGENTS.md 链接；全新只读 Codex 回答三个情景，保留原文。验收结束退出服务。

裁决 2：允许在 `~/gamepad_vla/envs/data-tools-py310/` 新建独立数据环境，安装项目已有 data 依赖，版本与计算节点一致。不得修改 SAPIEN、系统 Python、conda base 或共享环境。导出解释器从 `.local/` 站点配置读取。

原范围、被拒绝的功能、M1 隔离和 GPU 规则、禁止 commit、每个故障最多 3 次修正，以及 **N2 失败后停止报告**的约束继续有效。本轮没有实现动作提示、关节条形图、接触告警、握持/误触监控或多视角。

## 基线复核与证据范围

开始时 HEAD 为 `bab0ccd`，工作树干净。读取并核对了 AGENTS、CLAUDE、README、采集/M2 手册、L/M1 报告、采集入口、输入源、映射、schema、网页和软件停止脚本。

复核 [M1 归档](../2026-10-22-station-install/README.md)得到下列**历史实测**，本轮不改写这些数值：

| M1 项目 | 归档值 |
|---|---:|
| 四路 100 Hz 原始输入 | 各 6000/6000，合计 24000/24000，原始 float64 字节与消息头一致、无重排 |
| 发布→应用 P50 / P95 | 3.5435 / 6.9580 ms，分母 14396 次首次应用 |
| 300 s 编码推流 | 17418 帧，58.06 fps |
| 应用→编码 P50 / P95 | 23.7510 / 33.2875 ms |
| 当时 episode schema | v3 |

这不是本轮 ROS 无退化的证据：N2 在进入 ROS 阶段前失败，本轮真实 ROS 60 s 复验**未执行**。

源码还显示，原两个 collection 后端创建的是调试斜视角，相机没有使用 spec 的标定定义。本轮在**独立渲染副本**接入已有 `camera_3`，使用 v4.4 光学位姿和内参，分辨率 1280×720。SAPIEN 复用 `create_spec_camera`；MuJoCo 在渲染相机中设置光学坐标和偏心视锥，并对 fx/fy 做像素校正。没有修改物理实例。[protected-files.json](protected-files.json)确认两个物理后端、v4.4 配置、M1 ROS 验证器内容不变。新视角的吞吐不能直接继承 M1 旧视角的性能结论。

工作期间发现其他并行工作修改了 `feasibility.py`，并新增手柄高度相关脚本、报告。本任务未编辑或回退它们。已测部署归档中包含当时的并发工作，其内容与现在的工作树也有差别；151 项结果针对**已部署快照**，不覆盖后来出现的其他改动。[deployment.json](deployment.json)记录准确归档、源码摘要和逐文件清单，不能把它解释为纯净 HEAD 或全部改动均由本任务产生。

## 实现与接口

- `CollectionController` 是 CLI、服务共用的采集循环。`run_collection.py` 保留原参数；服务管理唯一自有仿真进程，渲染继续独立。短回看退出前完成首帧；启动中的取消只处理自有渲染器。
- `SessionManager` 管理会话、episode 列表和导出。网页只调用 REST v1，SSE 每 50 ms 发送实测快照；重连后先获取完整 status。尚未退出的子进程报告 `stopping`。录制中的列表从控制器读取行数和时长，未完整关闭的数据不显示负时长。
- 远程控制默认关闭，不持久化。网页 cookie 每次服务重新生成。写操作要求 JSON、非空客户端标识、同源 Origin；开关只接受网页会话。所有 API 调用追加 `api_audit.jsonl`。meta 保存 `started_by`、`stopped_by`、`forced` 和开始时健康快照。安全警告附在 status 和写操作响应中，但不阻止仿真。
- ROS 仍持续 spin、串行接收，保留原始 FIFO 和独立 latest 目标。增加按通道的实测频率、年龄、计数、解码错误；回放和 UDP 也提供实测健康。健康窗口为 2 s，超过 0.25 s 或从未收到消息告警。ROS 录制前在控制器实际复查；force 才可绕过。
- `ArmMapper` 保持数值计算顺序，增加 absolute/startup 两组七位布尔标志。schema 采用整数 **v4**：新读者必须理解该质量字段，旧 v2/v3 继续可读。最终目标回放不能推断原始限幅，使用 null。标志同时保存在命令及逐帧记录中。
- 停止后关闭 journal，严格检查由自有独立 CPU 进程执行，物理和推流继续。检查包括严格读回、独立接收边界计数、连续序号、行数/时长及时间边界。旧数据没有独立计数时不冒称检查通过。作废只写旁路 `discarded.json`。
- 实测旧 9000 行 episode 严格读回需 **3.0804 s**，支持将检查从物理循环移出的决定；该读回测量是 CPU 数据检查，不是新仿真实验。

入口、每个端点、退出码及语音操作规则见 [API 文档](../../usage/collection-api.md)；使用和 M2 边界见 [采集手册](../../usage/collection.md)。设计适配依据为 [aiohttp 服务](https://docs.aiohttp.org/en/stable/web_quickstart.html)、[Python spawn](https://docs.python.org/3.10/library/multiprocessing.html)、[MuJoCo 相机类型](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjvglcamera)。未引入 npm 或额外运行依赖。

## N1：已执行

数值见 [N1.json](N1.json)。计算节点全部 GPU 均有 CUDA 计算任务，因此浏览器使用 MuJoCo、free-space、中性最终目标 replay 和 CPU 软件渲染；数据、派生资源和输出留在配置工作区，没有占用忙 GPU。

| 项目 | 结果 |
|---|---|
| 计算节点全部单元测试 | **151 通过、4 跳过、0 失败**，2.61 s |
| 采集台全部单元测试 | **151 通过、4 跳过、0 失败**，0.87 s |
| 无头浏览器网页流程 | 启动→录制/备注→停止→检查→回看→作废→导出完成 |
| 仅 CLI 的控制流程 | 经远程 shell 启动会话、录制、停止等待检查、列表、作废、导出、停止会话完成 |
| 跨客户端 DOM 延迟 | 10 次开始/停止、共 **20** 样本；保守上界 P50 **33.2544 ms**、P95 **65.1230 ms**、最大 **68.8970 ms**；≤0.5 s，通过 |
| 最终新浏览器页面 JS 错误 | **0** |
| 真实导出数值读回 | **6** 份 Parquet，分别 63、41、39、39、39、40 行，与源 meta 行数一致 |
| 新只读 Codex 文档验收 | **3/3** 情景符合规则 |

4 个跳过项为缺少 gymnasium、torch、resnake_gym，以及未显式启用的真实 ROS 回归。新增采集/API 测试均实际执行。3 个 aiohttp 字符串 Request key 建议是警告，不是失败。Parquet 读回见 [export-readback.json](export-readback.json)，未将它冒称为本轮 LeRobot 原生完整数据集读回。

测试覆盖会话控制、正常关闭、取消初始化、独立检查完成、v4/v2/v3、原始输入边界、逐通道健康、作废前后原始文件哈希一致、损坏/截断/数量不一致/序号跳跃/时长不一致拒绝。API 契约覆盖各端点、409/403、幂等、同源与缺头、默认开关/网页开启/远程不能开启/服务重启复位、审计和 force meta。替身进程加入和退出检测集合也已测试；该测试没有覆盖下述终端包装进程反例。

### 延迟口径与测量修正

最初从 SSH 建连前计时，P95 为 **514.8 ms**，超门槛；这个值包含远程 shell 建连，保留在私有原始记录中。随后固定跨机墙钟偏移的方案出现随时间增长的偏差，无法作为有效延迟结果，未采用。

最终在 MutationObserver 确认 DOM 已更新后，立即发送只读 status 请求。以**同一个后端**的控制执行时间和收到该确认请求的时间作差。它包含 DOM 更新后的额外网络请求，因此是控制执行到 DOM 更新的保守上界，不依赖跨机校时。20 个值全部公开在 N1.json。没有把 514.8 ms 偷换成另一口径的同一指标。

探针曾有等待异步确认不足和读取空快照的问题；DevTools 也重放了旧页面探针异常。修正探针后使用全新浏览器 target 复测，最终产品页面 JS 错误 0。旧探针异常和无效计时结果保留，不计入最终新页面结果。

全新 `codex exec --ephemeral -s read-only` 仅取得 AGENTS 和 API 文档，未调用采集服务；问答原文及审查见 [codex-qna.md](codex-qna.md)。

## N2：失败后停止，未重跑

GPU 预检为 **356 MiB /4%**，无 CUDA 计算任务，满足准入。使用采集台已有 SAPIEN、浏览器环境及本机 localhost。远程写默认关闭被 CLI 正确拒绝，随后由无头浏览器操作网页开关。

替身使用 `/bin/sleep`，仅将 argv[0] 设为 `linker_hand_ros2_sdk_testdouble`；它被检测集合列出，横幅为红色，带警告的备注请求仍成功。**不能声称已验证正常绿色→红色的完整转换**：当时已有另一个常驻终端包装进程的命令行包含 `rokae_` 检查字符串，横幅本来就红。

替身退出后，网页等待恢复正常 **30 s 超时**。源代码以完整命令行子串匹配；实际匹配进程的 comm 是终端包装器，而非 SDK/珞石节点。这是已确认的误报反例。外部进程保持原样，没有结束它来改变结果。

异常时，私有测试驱动又在 Playwright 上下文退出后调用 browser.close，导致自动清理提前中断。随后只向确认属于本次运行的服务 PID 发送 SIGINT，保存录制并清理自有子进程。没有修正检测器或重跑 N2。

| N2 门槛 | 状态 |
|---|---|
| 连续 ≥300 s 的浏览器 SAPIEN 推流 | **未完成** |
| ≥2 个 episode 全通过 | **未完成**；只有 1 个保存后通过的 episode |
| 域 87 四路 100 Hz、≥60 s 原值无损 | **未执行** |
| 停一路后告警、record-start 409 | **未执行** |
| 台上 CLI 完整循环和本机导出 | **未执行** |
| 台上 20 次 DOM 延迟复验 | **未执行** |
| 替身加入/退出后恢复正常 | 加入被匹配，**恢复失败** |

[N2-failed.json](N2-failed.json)记录完整失败结果。人工清理前整段日志含失败后的遗留运行：**4863 编码帧 /146.6872 s，帧间隔口径 33.1454 fps**，应用→编码 P50 **43.5759 ms**、P95 **50.8788 ms**，分母 4863 帧。这不是有效的 300 s 验收窗口，也未测逐帧浏览器呈现 fps。P95 超过 50 ms，后端性能告警为 true。

保存的 v4 episode 为 **8690 行 /144.833 s**，17380 条 replay 接收事件，严格检查三项通过；started_by=web、stopped_by=service。它不能替代两段录制、真实 ROS 无损或完整网页验收。

### 截图与公开卫生

实际保留 **2 张原始诊断截图**：`recording.png` 和 `safety-red.png`，分别为录制约 5 s 和替身被匹配时的网页。**没有取得满足要求的“正常绿色横幅下录制中”截图**；长警告内容占据了画面，截图也未清楚呈现主画面。

两张图都包含另一进程命令行里的私有路径和地址，因此不进入公开仓库。采集台原图留在 `~/gamepad_vla/runs/collection-web-n2-evidence/`；本地私有副本在忽略的 `.local/collection-web/n2-diagnostic-private/`。未用生成或修改图片来制造正常截图。公开报告的两张合格截图交付仍未完成。

### 待验证的最小修正方案（本轮未实施）

候选修正是按真实可执行入口/节点参数识别 SDK 和珞石进程，而非扫描 shell 脚本文本。最小对照保持现场其他进程不变：只有终端参数含检查字符串的无害包装器应不告警；argv[0] 为 SDK/珞石替身的 sleep 应告警，退出后恢复。若包装器仍告警或替身漏检，则否定该实现。需要同时覆盖 ROS Python 入口。由于 N2 已失败，**本轮没有实现或执行这项对照**。

当前相机分辨率与 M1 旧预览不同，可能影响吞吐；没有执行同一状态、同一视角下的分辨率 A/B，对部分性能变化不作唯一原因判断。

## 环境、部署与交还

版本见 [environment.json](environment.json)。采集台 Python **3.10.12**、SAPIEN **3.0.3**、aiohttp **3.14.5**、手柄包 **0.3.1**；这些环境未升级。新增独立数据环境为 numpy **2.2.6**、PyArrow **25.0.1**，与计算节点 freeze 一致，固定于 `constraints/data-py310.txt`。

实际安装采用已有系统 Python 创建独立 venv，并显式固定两个现有依赖版本：

```bash
/usr/bin/python3 -m venv ~/gamepad_vla/envs/data-tools-py310
~/gamepad_vla/envs/data-tools-py310/bin/python -m pip install \
  --index-url https://mirror.sjtu.edu.cn/pypi/web/simple \
  numpy==2.2.6 pyarrow==25.0.1
env -u PYTHONPATH ~/gamepad_vla/envs/data-tools-py310/bin/python -m pip freeze \
  > ~/gamepad_vla/logs/n-data-freeze.txt
```

安装日志为 `~/gamepad_vla/logs/n-data-install.log`。清除 PYTHONPATH 后的 freeze 只有上述两个包；加载 ROS 后 pip 展示的额外 ROS metadata 不代表安装进该环境。私有 `.local/station.json` 的 `collection.export_python` 指向此解释器。**依赖已装好，台上实际导出验收未执行**，不能因 import 成功而标为导出通过。

站点只写安装根和允许的数据目录，缓存/临时目录也在安装根；不访问 CAN、串口或 USB，不启动手套、重定向、LinkerTA、SDK、珞石、相机节点，不调用软件急停脚本。训练和兄弟仓库未修改。未新增 BC/DAgger。

交还复查：采集台本任务进程 **0**、域 87 残留 **0**、CUDA 计算进程 **0**；显存 **356→1027→356 MiB**。首次清理后利用率 4%，期间复查 8%，最后复查 4%，均为独立采样；没有把利用率变化解释为已确认的残留。计算节点自有服务和子进程也已退出，浏览器已关闭；原始数据和日志保留。

| 交付或现场证据 | 位置 |
|---|---|
| 服务 / CLI 客户端 | `scripts/collection_server.py`、`scripts/collection_ctl.py` |
| 共享控制器 / 质量检查 | `src/gamepad_vla/workstation/collection_controller.py`、`collection_integrity.py` |
| API / 操作说明 | `docs/usage/collection-api.md`、`docs/usage/collection.md` |
| 采集台源码和私有配置 | `~/gamepad_vla/code/`、其 `.local/` |
| N2 原始日志、诊断图、失败统计 | `~/gamepad_vla/runs/collection-web-n2-evidence/` |
| 唯一 N2 episode | `~/gamepad_vla/runs/collection-web-n2/` 下该会话 |
| 台上测试、安装、freeze 日志 | `~/gamepad_vla/logs/n-tests.log`、`n-data-install.log`、`n-data-freeze.txt` |

## 修正与尝试记录

下表按独立故障记录修正次数；流程尝试与修正次数分别保留，不能把多次失败写成一次通过。

| 故障或证据 | 本轮处理 / 修正次数 |
|---|---|
| 初期视频连接依赖短超时性能查询 | 解除依赖并重试视频连接，1 次 |
| schema 断言仍为 v3；缺失导出环境用例依赖现场环境 | 分别更新断言、显式构造缺失环境，各 1 次 |
| cookie 测试从请求 jar 读安全属性 | 改查实际 Set-Cookie，1 次 |
| API 审计把列表响应当字典 | 修正，1 次；契约测试随后通过 |
| REST 浏览器流程第 1 轮：已关闭 viewer 的 final emit | 清除关闭的引用并加回归，1 次 |
| 第 2 轮：短回看首帧被收尾取消 | 等首帧及本机响应发送窗口，1 次 |
| 第 3 轮：stopped 到 OS 退出间再次 start 被拒绝 | 显式 stopping 状态并加回归，1 次 |
| 单独取消初始化时误报 viewer 退出失败 | 仅取消自有初始化进程并加回归，1 次 |
| 完整性读回实测阻塞 3.0804 s | 改独立 CPU 检查进程并复验，1 次 |
| 活跃列表使用未关闭 meta 产生负时长 | 改控制器实测值/未知值并加回归，1 次 |
| CLI --wait 摘要短暂仍显示保存中 | 等后端完成收尾并明确输出检查通过，1 次 |
| 计时包含 SSH；固定跨机偏移不成立 | 两个口径分别记录并弃用于 SSE 门槛；改同后端确认上界 |
| 浏览器确认探针异步等待、空状态 | 各修正 1 次；历史 DevTools 异常以全新 target 排除，原始记录保留 |
| N2 安全恢复超时、驱动清理异常 | **0 次修正、0 次重跑**；人工清理后停止报告 |

另有一次本地统计调用误用了没有 numpy 的系统解释器，改用现有项目 venv；没有因此安装或修改环境。

## 我自行做出的未授权选择

用户未逐项指定、由我自行决定且未另征求确认的实现选择：schema 使用整数 v4；SSE 50 ms、控制器状态约 100 ms；输入频率 2 s 窗口、推流性能 10 s 窗口；快捷键 R/S/N；episode id 组合会话名和局部编号；作废/检查分别使用 `discarded.json`、`integrity.json`；独立 CPU 检查进程；短回看收尾等待首帧和 100 ms 发送窗口；N1 用中性 CPU free-space replay；渲染副本接入 1280×720 标定 ego 而非原调试视角；DOM 测量使用同后端确认的保守上界。

其中完整命令行匹配导致了 N2 的实测误报；未将它合理化为通过。安装数据依赖有裁决 2 的明确授权。后续检测修正、N2 复跑与尚未执行的门槛仍需新的任务授权。
