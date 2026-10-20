# 双 0.8R 场景视图

[合成图](render/multiview.png)从左到右依次为标定摄像机、侧面和摄像机对面。两侧均加载 0.8R 网格；相机外参与保留的驱动内参来自已归档标定。

这是静态场景审阅。安装位姿、home 关节角和手部安装参数仍需实机核验，图像不证明碰撞间隙或抓取成功。

当前配置为 [`station_g20_v4_4.json`](../../../configs/workcell/station_g20_v4_4.json)。使用 `scripts/render_workcell.py` 和 `configs/render/workcell_multiview_preview.yaml` 复现；站点参数由本地配置提供。原始运行日志和主机元数据保留在私有档案中。
