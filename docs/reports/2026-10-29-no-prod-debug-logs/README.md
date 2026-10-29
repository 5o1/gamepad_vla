# 任务 U：生产采集不持续写调试日志

## 审阅者裁决 1（2026-10-29，用户追加）

用户要求：“当你需要诊断参考数据时才显式打开一次，否则都是不记录。”

诊断开关仅为单次会话启动参数（API `diagnostics: true` 或 CLI `--diagnostics`）；不保存到站点配置，不从配置读取默认开启，不沿用到下一次会话。网页不提供开关。关闭时不写诊断文件；显式开启时仍受上限约束，到期自动停止写入。

## 审阅者裁决 2（本地依赖）

不在本地安装 aiohttp；本地 `.venv` 保留最少依赖。API/SSE 测试在采集台已有 SAPIEN 环境执行，本地与采集台结果分别列出。

## 执行记录

已完成。基线 HEAD：`f3111d2`，未提交、未安装依赖。通过 `scripts/station/sync_tree.py` 整树部署并自动逐文件 SHA-256 比对。

## 修改与写盘审计

- `diagnostics` 只从本次会话启动请求读取。CLI 支持 `--diagnostics`，服务启动没有默认开启参数，网页没有诊断开关；站点或渲染配置不能开启下一次会话。view 的 `config.json` 仅作为本次子进程启动参数，绝不回写站点配置。
- 每个诊断文件先到任一上限即永久停止本次写入：900 秒、120000 行/H.264 access units、32 MiB。时间从 view 初始化起算，跨子进程共用起点。15 分钟容纳已有 10 分钟基准；视频可能先达字节上限。诊断文件惰性创建，未产生该类诊断时也不创建空文件。
- render/stream 的诊断行仅保存时序、帧号、命令号等小字段，不再包含仿真状态。网页所需 overlay、signals 与取帧候选标签仍通过原有 mmap/视频包实时传递，不写诊断。浏览器诊断只保存有限的标量时序字段。
- 网页 fps、P50/P95 沿用内存中的 `deque(maxlen=600)`，只统计最近 10 秒，不读取日志。
- 闲置时长默认 7200 秒，可用服务参数 `--idle-timeout-s` 或 `collection.idle_timeout_s` 修改。SSE/视频连接、网页请求、获准的写请求重置闲置计时；普通 CLI 成功 GET 不重置。录制、保存、检查或正在处理开始录制时不停止。闲置停止与开始录制共用锁，停止后向会话 `events.jsonl` 写入一次 `session/idle-stop`，并显示在最近控制事件中。附着于受管会话的内部 gateway 不另启闲置策略，避免外层有人连接时内部服务误停。

| 检查范围 / 写入 | 处理 |
| --- | --- |
| operator_view 五个逐帧 JSONL/H.264 文件 | 默认关闭，仅本次诊断有界写入 |
| `operator_command_*.jpg` | 仅诊断期间，最多三张 |
| `first_frame` | 原来每帧 touch，改为首帧一次 |
| renderer/gateway/encoder 与 camera/ego 子进程输出 | 每个启动最多 1 MiB；达到后继续消费管道并丢弃，避免堵塞子进程；同名文件重新启动时覆盖 |
| aiohttp access log | 服务与 gateway 均关闭，HTTP 轮询不产生日志 |
| `api_audit.jsonl` | 仅写请求与被拒绝的请求；成功只读/SSE 不追加 |
| `events.jsonl`、`controls.jsonl` | 保留按控制操作产生的生产审计/命令交接，不逐帧写 |
| snapshot/frame mmap | 原管线固定容量覆盖写，容量约 1 MiB / 2.7 MiB，不随时长增长 |
| config/spec/pids/ready/view/worker_exit/result、场景 XML/惯量修正 | 启停或配置操作时写一次/覆盖，无运行时间累计 |
| 相机 HTTP/JPEG/快照 ZIP 的 `.write` | 网络/内存输出，不是磁盘追加 |
| 标定 PNG/JSON、相机录制视频、episode 数据及完整性结果 | 用户操作产生的生产数据，保留 |
| `GAMEPAD_VLA_COLLECTION_PROFILE` 的旧 cProfile 分支 | 无仓库调用方，删除会随服务环境继承的诊断入口 |

未新增渲染管线、依赖、训练或硬件命令。实现参考 Python [单调时钟](https://docs.python.org/3/library/time.html#time.monotonic) 和 [有界 deque](https://docs.python.org/3/library/collections.html#collections.deque) 的原始文档。标准库机制的边界与实际运行结果分别验证。

## 测试

本地 `.venv`：`python -m pytest -q`，**195 passed, 8 skipped**。跳过项为 aiohttp（API 与相机路由）、torch、cv2、trimesh、显式 ROS2 集成条件、resnake_gym 与 xbox-gamepad-physics；按裁决没有安装。

采集台已有 SAPIEN Python：完整套件 **191 passed, 5 skipped**；API/SSE、四项任务 U 检查实际执行。跳过项为 gymnasium、torch、无 Node 的浏览器绑定、显式 ROS2 集成条件、resnake_gym；未以模拟输入测试冒充真机集成。aiohttp 的 AppKey/RequestKey 建议为警告，无失败。

任务 U 覆盖：默认短会话不创建五类逐帧文件；诊断时间/条数/字节截止（小上限、伪时钟）；诊断开关不从配置或上次会话继承；子进程日志满后仍能正常结束；成功只读请求/SSE 不进入审计，拒绝的 GET 仍保留；伪时钟下 SSE/网页活动保活、CLI 读轮询不保活、录制期间不停止、超时真正停止并只写一次控制事件。

测试修正预算：首次采集台运行的新夹具用默认 replay 却未提供路径，返回 400；仅将该夹具改为它支持的 ROS2 假源，第二轮通过（1/3 次修正）。

## 采集台冒烟与正式服务

实测数据见 [smoke-result.json](smoke-result.json)，可复现脚本见 [smoke.py](smoke.py)。临时端口 17881、replay、SAPIEN 3.0.3 / 手柄 0.4.1、60 Hz、gamepad-contact、原临时抬高场景、arm-startup none。临时测试服务在初次创建时开启 CLI 控制；没有修改正式服务的远程开关，没有启动输入或机器人节点。

| 实测 | 结果 |
| --- | --- |
| 未录制运行 | 180.0036 秒 |
| 全部测试目录文件逻辑大小增长 | **0 B** |
| 实际已分配磁盘空间增长 | **0 B** |
| 启动时会话文件（含固定 mmap） | 4109867 B 逻辑大小，3039232 B 实际分配 |
| 五类逐帧文件 | 均不存在 |
| 末尾推流内存统计 | 59.974 fps；apply→encode P50 44.257 ms / P95 46.206 ms |
| 随后录制 | 30.10 秒、1806 行、3612 条输入 |
| 完整性 | strict_readback、receiver_count_and_sequences、row_duration 全通过 |
| 审计 | 4 条写请求，成功只读请求 0 条 |
| 清理 | 已停止自有服务和仿真；临时目录（含 episode、物理工作目录及 view）已删除 |

录制通过 `collection_ctl.py record-stop --wait` 停止，并再次查询状态确认已保存、检查通过。

8780 已用最终代码通过 `scripts/station/start_collection_server.sh` 重启，PID 294114，新输出为 `collect-20261029-184333/sessions`（以站点私有 data root 为前缀）。参数沿用 ROS_DOMAIN_ID=88、source ros2、`station_g20_v4_4_raised_interim.json`、gamepad-contact、arm-startup none、空 ROS 前缀；远程控制仍默认关闭。启动后实查：HTTP 正常、session idle、无录制、safety safe，读取 status/config 后 `api_audit.jsonl` 仍为 0 B，磁盘 42%。没有自动启动仿真或硬件。

最终代码在本地和采集台均复跑通过，结果同上；最后的报告/实测 JSON 随整树同步。未安装依赖，未提交。
