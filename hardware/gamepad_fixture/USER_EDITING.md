# 用 Fusion 编辑夹具零件

从 Windows `Downloads/gamepad_fixture_v3_1/assembly.step` 查看整机；实际编辑请打开 `edit_table_up/<零件名>.step`，例如 `cradle_lower_jaw.step`（下爪）或 `slider_upper_jaw.step`（上爪）。这些文件与整机共用桌面朝上坐标：单位 mm，Z 向上，原点在远端桌边中点。`print/` 是打印摆放方向，**不能作为交回的编辑源**。

在 Fusion 中保留导入的原点和坐标轴，不移动、旋转、缩放或“移到原点”。每次只导出一个实体零件为 STEP，保留装配位置，文件名保持 `<零件名>.step`；不要导出整个装配体或转换成网格。

编辑后可直接告诉审阅者：“我改好了 slider_upper_jaw，文件在 Downloads/…”。也可以在仓库根目录运行：

```bash
# DOWNLOADS 是 WSL 中 Windows 下载目录的路径，例如 /mnt/c/Users/<用户名>/Downloads
.local/cad-env/bin/python hardware/gamepad_fixture/user_steps.py \
  "$DOWNLOADS"/slider_upper_jaw.step --part slider_upper_jaw
.local/cad-env/bin/python hardware/gamepad_fixture/pipeline.py
```

导入命令先验证，再复制到 `hardware/gamepad_fixture/user_overrides/slider_upper_jaw.step`。也可将按上述坐标约定导出的文件直接放入该目录；生成时会再次验证。移走覆盖文件即可恢复参数化零件。导入不代表验收通过，必须等流水线 PASS；manifest 和场景 provenance 会列出覆盖零件与 SHA256。

坐标检查会拦截明显移动、旋转、错误单位、多个实体及过大的修改（中心偏差 >3 mm、包络尺寸偏差 >4 mm 或保留体积 <85%）。对称件绕对称轴转动可能无法从几何区分；不会尝试自动对齐。小修改也可能触发硬门：干涉/控件全行程/接口通道/握把背部空间、承压垫与刚性座接触、抗扭、齿盘锁紧、滑块行程、防脱、打印壁厚/缝隙/支撑计划、凸块误差与保护空间、camera_3 投影。流水线最后自动通过站点 compute 配置在计算节点执行 34 控件状态与 4 握姿的原生重放；连接或重放失败会停止验收。

请保留承压垫完整足迹、背托、导轨、M8 叉耳和 Hirth；打印前还需切片检查及实物载荷验证。坐标与门限拒绝时会报具体指标，不会悄悄移动零件或绕过检查。

计算环境：SAPIEN Python 使用站点 `compute.python`；MuJoCo 原生编译使用可选 `compute.mujoco_python`，未设置时使用同一 workspace 下现有 `envs/fixed-gamepad-mujoco-py311/bin/python`。均不安装或覆盖环境。
