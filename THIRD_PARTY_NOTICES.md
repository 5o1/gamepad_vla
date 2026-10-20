# 来源与第三方许可

本文件说明随本项目分发的改编代码及外部资源的许可范围。外部项目和下载资产保留各自的许可，不因本项目的许可证而改变。

## 随仓库分发的改编代码

| 本项目文件 | 来源与改动 |
|---|---|
| `src/gamepad_vla/workstation/g20_mapping.py` | 采集台 `assist_teleop_local_policy` 的 `source/linkerhand_g20_isaaclab/linkerhand_g20_isaaclab/manus_mapping.py`，快照 `station_20261014`。保留映射表及换算公式，接入本项目统一关节顺序。 |
| `src/gamepad_vla/sim/g20.py` | 同一工程的 `assets.py` 和 `assets/usd/right/config.yaml`，快照 `station_20261014`。适配 URDF 转换、左右手资产准备及 PhysX mimic 修复。 |
| `src/gamepad_vla/workstation/operator_web/app.js` | [xbox_gamepad_physics v0.3.1](https://github.com/5o1/xbox_gamepad_physics/tree/v0.3.1) 的 `teleop/web/app.js`。沿用视频数据包及 WebCodecs 解码方式，适配操作者界面和遥测。上游采用 MIT，版权及全文见下。 |

上述两项工作站来源代码在本项目中的版本按 [MIT 许可证](LICENSE)分发，Copyright (c) 2026 5o1。此许可适用于本仓库中的版本，不改变原工作站工程其他文件的许可。

`src/gamepad_vla/signals.py` 的通道名称和排列对应 `resnake_gym` 的 `xinput-normalized-v1` 及 `xbox_gamepad_physics` 的 `sdl3-gamepad-core-v1` 接口，用于协议兼容；来源模块在文件头注明。

### xbox_gamepad_physics 许可

MIT License

Copyright (c) 2026 5o1

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## 下载资源

模型、手柄资源和外部标定求解器由脚本从上游获取，不随本仓库或 Python 发行包分发。固定版本、文件哈希和许可文件位置见[资源清单](references/external_resources.json)及[资产来源](references/robot_assets.md)。

- G20 对应的公开 L25 模型及 RealSense 相机描述保留上游 Apache-2.0 许可；相机描述同时保留上游 NOTICE。
- 手柄 wheel 和资源包遵循各自附带的许可及第三方声明；上述网页代码的 MIT 许可不代替资源包内其他内容的许可。
- AR5 0.8R 固定源的完整许可尚未确认；本项目提供来源和下载校验信息，不随仓库分发网格，也不为其另行授予许可。
- 标定求解器 `calib_board` 保留其上游许可；归档中的 `solve_four_holes.py` 是调用求解器的项目脚本。
