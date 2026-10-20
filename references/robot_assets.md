# 外部资源

机器可读清单：[external_resources.json](external_resources.json)。清单固定来源、修订、目标目录、文件大小和 SHA-256；下载文件保留在配置的工作目录，不随代码仓库分发。

| 资源 | 上游与固定版本 | 本项目用途 |
|---|---|---|
| 手柄 | [xbox_gamepad_physics v0.3.1](https://github.com/5o1/xbox_gamepad_physics/releases/tag/v0.3.1) | wheel、资源 ZIP 和发布校验文件；通过官方构建入口准备 SDF/可视化资源 |
| 双 0.8R | [linker-sim-assets](https://github.com/linker-bot/linker-sim-assets/tree/e6bcf44ff0203534e14500cb836f57e9f9492292/src/linker_robot_assets/assets/components/arms/ar5_08/variants/right)，`e6bcf44f` | 同一右型机械臂的两套实例；8 个真实 STL，保留上游关节、限位和惯性 |
| G20 对应几何 | [linkerhand-urdf / L25](https://github.com/linker-bot/linkerhand-urdf/tree/075cc7d42cc1e756bdcbece0fc069a0779fc5237/L25)，`075cc7d4` | 左右各 22 个 STL；对应侧网格与采用的站点版本同哈希，URDF 忽略 robot 名称和空白后相同 |
| D435 外壳 | [realsense-ros](https://github.com/realsenseai/realsense-ros/tree/7a87097c499e3b0b69518d54a8e7a9ac1924e87c/realsense2_description)，`7a87097c` | `meshes/d435.dae`，仅用于显示相机外壳，不安装相机驱动 |
| 标定求解器 | [calib_board](https://github.com/5o1/calib_board/tree/788696d54da4cb541b3baf6b645e591c7f4ecf20)，`788696d5` | 已归档三位置 ChArUco 外参求解；不需要站点登录或新拍照 |

`fetch_resources.py` 获取上面的完整输入，直接下载 LFS 的实际内容并核对字节数和哈希，因此该流程不需要安装 Git LFS。`prepare_resources.py` 将公开手模型映射到场景路径，生成两套唯一命名的机械臂并调用 `build_visual_split.py` 生成 64 个手部材质分片。分片只改视觉表面，碰撞和动力学继续使用原始 URDF。具体命令见[复现指南](../docs/reproduce.md)。

G20 与 L25 在这里的对应关系仅限已比对的几何、关节和惯性，不能据此推导 SDK 协议兼容。0.8R 的完整硬件后缀、实机安装零位和接触行为也未因此得到验证。

G20 对应源和相机描述保留下载的 Apache-2.0 许可文件；手柄遵循上游发布许可。0.8R 固定仓库的完整许可尚未确认，本项目仅记录下载来源，不重新分发其网格。

## 可选路径

- [IsaacLab v2.3.2](https://github.com/isaac-sim/IsaacLab/releases/tag/v2.3.2)：配合 Isaac Sim 5.1.0、Python 3.11；安装脚本为 `setup_isaaclab_env.sh`，不属于当前 SAPIEN 画面复现。
- [resnake_gym](https://github.com/5o1/resnake_gym)：上游信号集成。项目本地使用的修订尚不能通过匿名 GitHub API 获取，因此不声称可独立复现其训练结果。
- [openpi](https://github.com/Physical-Intelligence/openpi/tree/215abfb217dbac7d5f1273282331b9b1866c0479)：暂缓的 π0.5 适配，未提供预训练模型运行复现。
- [LinkerHand ROS2 SDK](https://github.com/linker-bot/linkerhand-ros2-sdk/tree/0035f236c98b644aba6d9e972359df3fa1af4342)：硬件参考；采集台自定义驱动与固件不随本仓库提供。
