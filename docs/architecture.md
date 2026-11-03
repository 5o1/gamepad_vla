# 模块边界

输入源负责 UDP、回放和脚本输入；命令层统一单位和关节表示；后端负责仿真或实机 I/O；采集循环按时间网格调度，并将观测交给记录器。图像渲染、编码和浏览器连接独立于物理循环。

场景文件保存机械几何、相机标定和任务参数。机器地址、设备序列号、部署目录及端口属于站点配置。前者可共享并参与版本管理，后者通过忽略的 `configs/site.local.json` 或 `GAMEPAD_VLA_SITE_CONFIG` 指定。

可视化复用手柄资产的 example runner 与 latest-state/NVENC 串流接口；不会修改兄弟仓库。Pi0.5 条件适配暂缓，不属于当前硬件控制路径。运动学与梯度检查只能证明相应数值行为，不能替代接触或实机验证。

`demogen/` 消费已结束且通过完整性检查的 v4 episode，负责单手流、控件片段库、手柄系参考、臂 IK、拼接与变化、物理筛选和数据溯源。它调用既有 `kinematics.SerialChain`、collection 后端和 `EpisodeWriter`；不会接入采集服务或创建第二套场景/画面。`scripts/demogen.py` 仅处理 CLI 参数，参数配置在 `configs/demogen/`，数据由站点数据根目录承载。只有实际物理回放通过的候选才晋升为输出 episode，目标信号不冒充观测；详见[使用说明](usage/demogen.md)。
