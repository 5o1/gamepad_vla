# Xbox One S 1708 夹具

参数化实体源代码为 [fixture.py](fixture.py)，设计、BOM、打印和装配说明见[任务 V 报告](../../docs/reports/2026-10-29-gamepad-fixture-cad/README.md)。STEP/STL 使用 mm，仿真 GLB 使用 m；均位于手柄局部坐标系，`manifest.json` 分别给出两种单位的世界变换。

当前为 **v2 Z 形布局**：立柱脚在靠相机的内侧，外沿留下连续 220 × 60 mm 平整夹持带；打印底板厚 20 mm，配两把 C 型夹、压力鞋和背胶软垫。转轴轴心伸出桌边 14.62 mm，导轨端盖仍使夹具最外端伸出 80.01 mm。手柄目标位姿不变。

```bash
/usr/bin/python3.10 -m venv .local/cad-env
.local/cad-env/bin/pip install -r hardware/gamepad_fixture/requirements-cad.txt
XDG_CACHE_HOME="$PWD/.local/cad-env/.cache" .local/cad-env/bin/python hardware/gamepad_fixture/fixture.py
.local/cad-env/bin/python hardware/gamepad_fixture/check_fixture.py
.local/cad-env/bin/python hardware/gamepad_fixture/render.py
.local/cad-env/bin/python hardware/gamepad_fixture/design_sheet.py
```

默认 `out/` 含打印方向的 STEP/STL、装配 STEP、独立材质网格 GLB、完整夹持带切片修改器 `base_clamp_band.stl`、A4 软垫模板、`checks.json` 与七张 PNG。新增 `07_world_z_layout.png` 用世界 X/Z 标注相机方向和桌子外沿。`envelopes/` 单独保存 C 型夹及操作手包络，供复核，未混入物理装配。生成物不纳入 git；超过 5 MB 的文件存到 `.local/gamepad_fixture_large/`，输出目录保留相对符号链接。

铝板方案：`fixture.py --aluminum --out hardware/gamepad_fixture/out_aluminum`。可选 TPU：`--tpu-pads --out hardware/gamepad_fixture/out_tpu`；装配网格使用压缩后厚度，TPU 打印件使用未压缩厚度。两个开关可组合。`check_fixture.py` 和 `render.py` 均接受 `--out`。

```bash
.local/cad-env/bin/python hardware/gamepad_fixture/fixture.py --aluminum --out hardware/gamepad_fixture/out_aluminum
.local/cad-env/bin/python hardware/gamepad_fixture/check_fixture.py --out hardware/gamepad_fixture/out_aluminum
.local/cad-env/bin/python hardware/gamepad_fixture/render.py --out hardware/gamepad_fixture/out_aluminum
.local/cad-env/bin/python hardware/gamepad_fixture/fixture.py --tpu-pads --out hardware/gamepad_fixture/out_tpu
.local/cad-env/bin/python hardware/gamepad_fixture/check_fixture.py --out hardware/gamepad_fixture/out_tpu
.local/cad-env/bin/python hardware/gamepad_fixture/render.py --out hardware/gamepad_fixture/out_tpu
```

手部复核使用 `.local/two-hand-k0/{left,right}/{witnesses.json,kinematics.npz}` 和原有手部 URDF 碰撞网格，来源及覆盖范围记录于 `hand_states.json` 和报告。79 个离散状态包含抓握、已保存状态和诊断 IK 端点，不能证明全部控件的连续动作安全；缺失输入时检查明确失败。v1 世界包围盒的可追溯对照保存在 [reference/v1_fixture_world_bounds.json](reference/v1_fixture_world_bounds.json)。

CAD 验证不能确认实体 1708 的局部尺寸或打印材料性能。先核对报告中的卡尺项目，再按实测更新参数、重新导出并复测安装位姿。

[单页设计总图 PNG](design_sheet.png)（4960 × 3507，300 dpi）及 [A3 PDF](design_sheet.pdf)包含装配、爆炸、三视图、尺寸和剖面，按[材料表](materials.md)与[统一材料色](materials_palette.json)着色。`design_sheet.py`复用既有CPU渲染，不需要重新构建CAD。

实物验证使用 [VALIDATION.md](VALIDATION.md)。`fixture.py --coupons` 导出11个原实体小样到 `out_coupons/`；`--marker-card` 导出可拆ChArUco支架、CAD变换和两套1:1 PDF/SVG到 `out_marker/`，均不改默认 `out/` 或v2主体。[camera_3工具与精度](../../docs/reports/2026-10-30-fixture-validation/README.md)以标记卡作为验收主方案，轮廓仅粗调；临时ID90贴纸只测面内滑移，正式采集前须撕除；划线法作为独立低成本对照。
