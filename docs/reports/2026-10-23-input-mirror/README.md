# 任务 T：采集台输入链镜像

仓库基线 `5c1431b`，未 commit。构建、启停脚本已入仓库工作树并部署，当前三个输入节点均由 `~/gamepad_vla/station_ws` 启动并保持运行。**双臂已验证；双手因手套没电关闭未验证，待充电后由用户在现场确认。** 按审阅者裁决 1 停止手话题排查，不再修改脚本或配置。

## 交付与构建

- [构建脚本](../../../scripts/station/build_input_mirror.sh)：首次复制、关闭 `manus_dex.thumb_ergo`、隔离用户 setuptools 后编译；`--linkerta-only` 仅修复已有镜像的 LinkerTA。
- [启动脚本](../../../scripts/station/start_input_chain.sh)：检查手 SDK／珞石节点、已有输入节点、镜像包前缀和镜像安装配置的 `calibration: 0`；固定域 88、loopback，retarget 使用已有 conda `workstation` 环境的 Python。
- [停止脚本](../../../scripts/station/stop_input_chain.sh)：只中断登记的输入进程组，检查实际退出结果；重复停止已验证。
- 路径由站点配置或参数提供，详见[采集手册](../../usage/collection.md#输入链镜像与构建)。站上 `m2/start.sh`、`m2/stop.sh`、`build_station_ws.sh` 已改为转调仓库入口，没有第二份业务实现。

已确认 LinkerTA 缓存原为 `USE_ROS2:BOOL=OFF`；其 CMake 默认 OFF，仅构建完成不会生成 ROS2 包注册信息。按上游 README，仅重编该包并加 `--cmake-args -DUSE_ROS2=ON` 后，`ros2 pkg prefix linkerta` 指向 `~/gamepad_vla/station_ws/install/linkerta`，`ros2 launch linkerta run.launch.py --show-args` 通过，实际启动的节点和参数文件也都位于镜像。

构建脚本还在独立的 `~/gamepad_vla/station_ws_t_buildcheck` 完成了一次从复制到编译的验证：其余三个 ROS 包及 LinkerTA 均 finished。该目录仅用于构建验证，当前运行仍使用原 `station_ws`。构建有上游 C++ 警告，无构建错误。未安装新依赖；`~/workstation` 未作修改。用 `diff -r` 核对镜像，排除生成的 Python 缓存和 egg-info 后，唯一源码差异为 `base_config.yml` 的 `manus_dex.thumb_ergo.enabled: true → false`；未制作镜像逐文件 SHA-256 清单。

仓库代码已用 `sync_tree.py` 整树部署并通过其传输哈希检查（399 个文件，本报告写入前）。本地 shell 语法检查通过，已有 `tests/test_site_config.py` 在仓库 `.venv` 中 4 项通过。

## 现场验证

最终测量时间：站点时间 2026-10-23 06:14:21（UTC+8）。使用真实 rclpy 订阅器，域 88、`ROS_LOCALHOST_ONLY=1`，预热 2 s 后连续测量 10.0099 s；双臂 QoS 为 BEST_EFFORT。频率按首末消息接收间隔 `(N−1)/(t_last−t_first)` 计算。

| 话题 | 消息数 | position 维度 | 实测频率 | 状态 |
|---|---:|---:|---:|---|
| `/left_arm_joint_control` | 1001 | 7 | 100.0014 Hz | 已验证，单一发布者 `linkerta_node` |
| `/right_arm_joint_control` | 1001 | 7 | 100.0023 Hz | 已验证，单一发布者 `linkerta_node` |
| `/cb_left_hand_control_cmd` | — | 未验证（期望 20） | 未验证（期望约 60 Hz） | 手套没电关闭，待现场确认 |
| `/cb_right_hand_control_cmd` | — | 未验证（期望 20） | 未验证（期望约 60 Hz） | 手套没电关闭，待现场确认 |

Manus 发布器已连接 Manus Core，目前仅接收器在线；输入进程保持运行，供手套充电开机后自动发布。未把旧节点缓存值的持续发布当作新手套数据，也未因手话题缺失继续排查或调整配置。

`thumb_ergo` 的记录来自启动脚本读取 retarget 实际使用的镜像源配置后写入 `retarget.log` 的首行；原生节点启动信息没有打印这个开关。下面将站上绝对路径前缀简写为 `<input_mirror>`：

```text
Input config: manus_dex.thumb_ergo.enabled: false (<input_mirror>/src/linkerhand_retarget/linkerhand_retarget/config/base_config.yml)
```

当前日志目录为 `~/gamepad_vla/logs/m2-20261023-061156-3662756`，也可通过 `logs/m2-current` 定位。此证据确认加载配置关闭覆盖，不代表手柄操控或手运动效果已经验证。

## 留存状态

Manus、retarget、LinkerTA 的登记 PID 分别为 3662787、3662834、3662835，均在镜像下运行；其 ROS 域和 loopback 环境已逐项核对。8780 采集服务 PID 始终为 3656161，原 session 保持 running、未录制、无服务错误，未重启。未启动手 SDK、珞石节点或相机。手话题验收留给用户充电后在现场完成。
