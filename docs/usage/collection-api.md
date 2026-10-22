# 给 Codex 的采集控制说明

你在本项目中通过语音或文字接受采集指令。先读取本文；所有采集操作都使用 `collection_ctl.py`。它调用采集台回环 REST API，不连接硬件。

**N1 和采集台 N2 已通过，数值与限制见[验收报告](../reports/2026-10-22-collection-web/README.md)。** 验收服务已退出，使用前由现场人员启动。不要自行结束外部进程来消除横幅，也不要绕过远程控制开关。

## 连接与服务

通过站点远程 shell 在**采集台本机**执行客户端，不从本机访问采集台的局域网端口。具体连接命令、主机、环境和代码路径见忽略的 `.local/collection-web/station-control.md`；该文件属于私有配置，不得复制进公开报告。找不到私有说明时请用户提供站点入口，不猜地址。

由现场人员在采集台加载私有运行环境后启动：

```bash
mkdir -p ~/gamepad_vla/logs
python scripts/collection_server.py --host 127.0.0.1 --port 8780 \
  >> ~/gamepad_vla/logs/collection-server.log 2>&1
```

使用现有 SAPIEN 环境的 Python，不能使用 conda base 或覆盖共享环境。服务常驻到现场人员在终端 Ctrl+C。所有非回环绑定均被拒绝。默认值来自私有站点配置的 `collection`；网页设置与 CLI 使用相同控制器。

每次启动远程控制默认**关**，不记忆上次状态。现场人员在网页开关开启后，非网页客户端才能写入；也可由现场人员明确在服务启动命令加 `--enable-remote-control`。**Codex 不得为绕过关闭状态而重启服务、加这个参数、索取/复制网页 cookie、伪装网页或直接调用开关端点。** 只能请用户让现场人员开启。

网页由本次服务生成 HttpOnly、SameSite=Strict cookie 识别。所有写请求还必须有 JSON Content-Type 和非空 `X-Gamepad-VLA-Client`；有 Origin 时要求同源。这是防误操作的开关，不是安全边界，不能对抗刻意绕过。GET 状态、列表、SSE 始终可读。

## 标准操作流程

1. 经站点远程 shell 运行 `python scripts/collection_ctl.py status --json`。
2. 检查 `remote_control_enabled`。关闭时请用户让现场人员在网页开启，**不要尝试绕过**。
3. 检查会话、输入源和四路健康。会话不存在时，先明确后端、场景、输入；需要时按用户明确要求启动仿真会话。ROS 四路从未收到或年龄超过 0.25 s 时先恢复输入。不要自行 `--force`。
4. 按明确指令开始或停止。开始录制：`record-start`；停止并等检查：`record-stop --wait`。重复停止安全且幂等。
5. 再运行 `status --json`，用一句话复述实际结果，例如“已开始录制 episode 12，四路输入正常”，或“已停止，完整性检查通过”。未取得结果时不能声称成功。

安全横幅变红会附上匹配到的进程，但不阻止仿真操作。向用户报告警告；不要启动、结束或查询硬件进程/设备来消除警告。

## 语音场景规则

- 指令含糊时先确认，不猜用户意思。
- 不确定是否听清“停止”时，**宁可停止录制**：`record-stop --wait`，因为它安全且幂等；这不代表停电或硬件急停。
- “把刚才那段删了”不足以授权操作：先问 episode 编号和是否“标为作废”。接口没有删除原始数据功能。
- 作废、导出等不可逆或高成本操作，必须由用户明确说出 episode 编号，才可执行；不能自行猜“刚才”对应哪段。
- force 绕过 ROS 健康检查须用户明确授权；会把 forced 和当时四路健康写入 meta。

| 语音意图 | 命令 |
|---|---|
| 开始采集 | 先 status，再 record-start，最后 status 复述 |
| 停 / 停止采集 | record-stop --wait，再 status 复述 |
| 现在什么状态 | status |
| 给这段加备注“…” | note "…" |
| 指定编号标为作废 / 导出 | 核对明确编号后 void / export |

这里只映射采集控制意图，不提供采集动作清单。

## CLI 示例

以下命令都在采集台本机、代码目录执行。`--json` 输出原始 JSON，默认给简短英文摘要；agent 向用户复述结果时使用中文；`--url` 仅接受回环 URL。

```bash
python scripts/collection_ctl.py status --json
python scripts/collection_ctl.py session-start --backend sapien --scene-variant full \
  --source replay --replay "$GAMEPAD_VLA_REPLAY"
python scripts/collection_ctl.py record-start --note "Free-form note"
python scripts/collection_ctl.py note "Additional note"
python scripts/collection_ctl.py record-stop --wait --json
python scripts/collection_ctl.py episodes --json
python scripts/collection_ctl.py episodes "$EPISODE_ID" --json
python scripts/collection_ctl.py void "$EPISODE_ID" --reason "User-specified reason"
python scripts/collection_ctl.py export "$EPISODE_ID"
python scripts/collection_ctl.py session-stop
```

`EPISODE_ID` 使用列表返回的完整 id（包括会话部分）。默认不限制会话时长；`session-start --duration-s 300` 可限制。回看由网页同一 session/start 端点传 `{episode: id}`，使用现有 replay 源。导出调用已有工具；缺少 data-tools 环境时明确报错，不会安装依赖或静默失败。客户端退出码：0 成功，2 被拒绝（403/409），3 服务不可达，1 参数/导出/完整性等其他失败。`record-stop --wait` 默认最多等 120 s，超时后保留数据并请继续查询。

## 完整性结果按 episode 等待

`status.integrity` 和单段查询的 `integrity` 都包含 `episode_id`、`state`。状态为 `pending`、`running`、`passed`、`failed`；未完成时 `passed` 为 null。新录制开始后，当前字段立即变为新段的 pending，页面不沿用旧段的通过。停止时先保存，再显示检查中，最后显示带编号的检查通过或失败。

```json
{"episode_id":"session-example~episode-000002","state":"running","passed":null,"checks":{},"errors":[]}
```

程序必须保存 `recording/stop` 返回的 `episode_id`，轮询该段查询端点。只有结果的编号相同且 state 是 passed/failed 才结束；None、其他编号、pending、running 都继续等。`collection_ctl.py record-stop --wait` 已这样实现，超时退出 1，不会把上一段结果当作成功。`recording_status.last_saved` 属于历史记录，不能用它替代当前段检查。

## REST v1

所有写操作均返回 `state` 与 `safety`；所有错误为 `{error_code, message, state}` 并附 safety。JSON 示例中的字段值需换成实际用户指定值。非网页写调用需要现场开启远程控制。

| 方法 / 端点 | JSON 参数或结果 |
|---|---|
| GET `/api/v1/status` | 会话/后端/场景/源、四路健康、录制、episode_id、完整性、磁盘、安全、推流、远程开关、最近控制事件 |
| POST `/api/v1/session/start` | `{"backend":"sapien","scene_variant":"full","source":"replay","replay":"selected-file.jsonl","duration_s":null}` |
| POST `/api/v1/session/stop` | `{}`；重复停止成功 |
| POST `/api/v1/recording/start` | `{"note":"Free-form note","force":false}`；ROS 不健康拒绝，明确 force 可绕过并记录 |
| POST `/api/v1/recording/stop` | `{}`；立即返回 episode_id，检查由独立 CPU 进程异步执行 |
| POST `/api/v1/notes` | `{"text":"Free-form note"}` |
| GET `/api/v1/episodes` | episode 列表 |
| GET `/api/v1/episodes/{id}` | meta、integrity、voided |
| POST `/api/v1/episodes/{id}/void` | `{"reason":"Explicit reason"}`；旁路文件，不改原始数据 |
| POST `/api/v1/episodes/{id}/export` | `{}`；调用已有 LeRobot 导出工具 |
| GET `/api/v1/events` | SSE 状态流；网页自动重连，重连后先拉完整 status |
| GET `/api/v1/config` | 网页可选配置与站点默认值 |
| POST `/api/v1/remote-control` | `{"enabled":true}`；仅现场网页会话可调用，Codex 不调用 |

例如读取状态和停止录制（后者需已开启远程控制）：

```bash
curl http://127.0.0.1:8780/api/v1/status
curl -H 'Content-Type: application/json' -H 'X-Gamepad-VLA-Client: cli' \
  -d '{}' http://127.0.0.1:8780/api/v1/recording/stop
```

| error_code | HTTP / CLI exit | Message / next step |
|---|---|---|
| remote_control_disabled | 403 | Ask the station operator to enable remote control on the web page. Do not bypass it. |
| web_session_required | 403 | Only the local web page can toggle remote control. Ask the station operator. |
| cross_origin | 403 | Request origin does not match this page. Call the API locally on the station. |
| client_header_required | 403 | Write requests require JSON and a client header. Use collection_ctl.py or the local web page. |
| session_active | 409 | Simulation session already active. Stop it first if requested. |
| session_not_running | 409 | Simulation session not ready. Start it and wait until ready. |
| recording_active | 409 | Recording or saving already in progress. Check status first. |
| input_unhealthy | 409 | Unhealthy inputs: [channels]; restore inputs before retrying, or explicitly use force if authorized. |
| episode_recording | 409 | Episode is still recording. Stop and save it before marking it as void. |
| invalid_json / invalid_request | 400 | Invalid JSON or invalid parameters/data. Correct the request and retry. |
| not_found / http_error | 404 / other | Endpoint or method not found. Check the API v1 documentation. |
| operation_failed | 503 | Operation failed. Check the service log before retrying. |
| camera_busy | 409 | Camera is in use. Ask the station operator to resolve it. Owner processes are left running. |
| service_unreachable | CLI exit 3 | Service unreachable. Check that the service is running on the station. |
| integrity_failed | CLI exit 1 | Integrity check failed. Keep the original files and review the results. |
| integrity_pending | CLI exit 1 | Integrity check timed out. Passing is unconfirmed; query status or episodes. |

英文消息示例（`state`、`safety` 以实际结果为准）：

```json
{"error_code":"remote_control_disabled","message":"Ask the station operator to enable remote control on the web page.","state":{},"safety":{}}
```

CLI 默认摘要示例：

```text
Session running; Recording; episode session-example~episode-000012; Remote control On.
Stopped recording session-example~episode-000012; integrity: passed.
```

每次 API 调用追加 `api_audit.jsonl`，包含时间、端点、参数、结果和客户端；episode meta 保存 started_by/stopped_by、forced 与开始健康快照。日志和原始数据不进入公开仓库。

## 这个接口做不到的事

它不启动手套、重定向、LinkerTA、SDK、珞石或任何硬件节点；现场人员仍须按 [M2 手册](collection.md#采集台仅仿真联调)操作。它不能急停、不能下电、不能删除原始数据，也不能替 Codex 打开远程控制开关。`session-stop` 只停止本服务创建的仿真进程。页面提示的软件急停脚本不能下电，本接口不调用它。

## 采集画面与现场启动

画面通过 `operator_view` 的 teleop mmap、独立孪生渲染和 NVENC H.264 提供，浏览器使用 WebCodecs；需支持 H.264 的 Chrome/Edge。`POST /api/v1/view` 接受 `{"view":"camera_3"}` 或 `{"view":"operator"}`，遵守同一 JSON、客户端标识、同源和远程开关规则。status 的 `view` 来自实际编码帧。

现场采用临时抬高场景、gamepad-contact 和不做启动姿态限幅时：

```bash
cd ~/gamepad_vla/code
ROS_DOMAIN_ID=88 bash scripts/station/start_collection_server.sh \
  --host 127.0.0.1 --port 8780 --source ros2 \
  --spec configs/workcell/station_g20_v4_4_raised_interim.json \
  --scene-variant gamepad-contact --arm-startup none --ros-topic-prefix '' \
  --output-root ~/gamepad_vla_data/episodes
```

本机打开 `http://127.0.0.1:8780/`，确认参数后点“Start simulation”。远程写仍默认关闭。输入节点由现场人员用 `~/gamepad_vla/m2/start.sh` 启动；该服务和 Codex 不启动硬件节点。`gamepad-contact` 保留机器人与手柄接触，关闭桌面等环境碰撞；`--arm-startup none` 仅关闭启动姿态限幅，绝对关节限位保留。服务默认仍是 v4.4，以上命令显式选择临时抬高场景。

## 相机面板 API

相机生命周期独立于仿真；沿用上文所有写权限和审计规则。相机操作不发送手或臂命令。除相机子进程外，原有仿真进程所有权保持不变。

| 方法 / 端点 | 参数或结果 |
|---|---|
| POST `/api/v1/camera/start` | `{}`；启动预览，固定 1280×720 @ 30 fps |
| POST `/api/v1/camera/stop` | `{}`；停止相机及其录像子服务，释放 USB |
| GET `/api/v1/camera/status` | mode、serial、分辨率、K、fps、age_s、会话及质量报告 |
| GET `/api/v1/camera/preview.jpg` | 新鲜 JPEG，过期/未启动返回 503 |
| POST `/api/v1/camera/snapshot` | `{"label":"position_1"}`；保存 PNG + JSON，返回 name、path、metadata |
| GET `/api/v1/camera/photos` | 当前标定会话照片及原始元数据 |
| GET `/api/v1/camera/photos/{name}` | 当前会话单个 PNG 或 JSON，可下载 |
| POST `/api/v1/camera/record/start` | `{}`；停止预览，启动站点 ego 网页并转发 start/record/start |
| POST `/api/v1/camera/record/stop` | `{}`；转发 record/stop，停掉 ego 相机子服务，按需恢复预览 |
| GET `/api/v1/camera/record/state` | 站点 ego 网页的原始 state；结束后为最近一次原始录像结果 |

`camera_busy` 为 409，含英文占用进程提示；不会结束其他进程。录像三个代理端点直接返回上游 JSON，不包装成仿真 state；失败仍使用统一错误格式。原有“接口不启动硬件节点”的说明对相机端点有例外：这些端点按现场授权仅启动 RealSense，不启动手、臂或 CAN 节点。
