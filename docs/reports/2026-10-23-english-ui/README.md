# Task R: English collection UI and tools

基于 `fa0c94c`，未 commit。网页、服务端错误和 CLI 摘要已英文化，错误字段统一为 `message`，调用方、测试和 API 说明同步更新。状态使用 Idle / Recording / Saving / Checking / Saved；完整性显示 `Integrity: passed (episode ID)`。未更改路由、控制流程或 episode schema；第三方 overlay 和中文代码注释保留。已有英文帮助与输出无需改动。

| 验证 | 结果 |
|---|---|
| 本地 `.venv/bin/python -m pytest` | **188 passed, 8 skipped**；未安装依赖。跳过项涉及 aiohttp、torch、cv2、trimesh、ROS2 集成及外部包 |
| 采集台现有 SAPIEN Python：`"$P" -m pytest` | **183 passed, 5 skipped, 3 warnings**；API 与相机 API 测试通过。跳过 gymnasium、torch、Node、ROS2 集成、resnake_gym；警告为 aiohttp NotAppKeyWarning |
| Python 编译、两份改动 shell 脚本 `bash -n`、`git diff --check` | 通过 |
| `scripts/station/sync_tree.py` 整树部署 | SHA-256 校验通过 |
| 采集台 18792 临时服务 + 无头 Chromium | 采集页、operator 页标题及 lang 为英文，可见文字与辅助标签中文匹配 **0**，JS 异常 **0**；临时服务已退出 |
| 8780 正式服务 | 未停止、未重启；监听 PID、会话、revision、远程开关前后相同 |

截图：`.local/review/lang-r/browser/camera-panel-idle.png`、`operator-idle.png`。日志：`.local/review/lang-r/pytest.log`、`station-pytest.log`、`sync.log`，浏览器结果为 `browser/result.json`。验收在空闲页面进行，未启动相机、仿真或硬件节点；不将本次界面验收作为硬件功能验证。

中文残留检查命令（按范围排除第三方 overlay 及非用户可见代码注释；文档中文正文不参与源码扫描）：

```bash
rg -n "[\p{Han}]" \
  src/gamepad_vla/workstation/collection_web \
  src/gamepad_vla/workstation/operator_web \
  src/gamepad_vla/workstation/{collection_server,camera_panel,camera_service,collection_controller,operator_view}.py \
  scripts/collection_ctl.py scripts/run_collection.py scripts/station \
  -g '!**/overlay/**' -g '*.py' -g '*.sh' -g '*.html' -g '*.js' -g '*.mjs' \
  | rg -v '^[^:]+:[0-9]+:[[:space:]]*#'
```

输出：**无输出，0 行**（无匹配退出码为 1）。未排除注释的原始扫描为 19 行，全部是两份 shell 脚本原有中文注释，按用户要求未修改。原始与范围内结果保存在 `han-raw.txt`、`han-visible.txt`。

```bash
rg -n 'message_zh' src scripts tests docs/usage/collection-api.md AGENTS.md
```

输出：**无输出，0 行**。API 文档的错误表、代码示例与输出已使用英文；agent 仍用中文向用户复述结果。
