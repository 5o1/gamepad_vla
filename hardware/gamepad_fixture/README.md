# Xbox One S 1708 夹具 V3.1

当前版采用竖直柱、对称叉耳与 M8 双剪金属轴、独立打印的恒定齿侧角 Hirth 盘，以及规则平面的两爪和背托。默认底板 240×90×16 mm；桌边 30 mm 带内除柱脚外，两侧各有 75 mm 可夹长度，不限定 C 夹中心。压力鞋和夹持带上的分压圈已取消，底部软垫由使用者粘贴，不画入装配。

[设计总图 PNG](out_v3_1/design_sheet.png) · [A3 PDF](out_v3_1/design_sheet.pdf) · [材料与五金](materials.md) · [实物验证单](VALIDATION.md) · [逐条要求、检查和场景报告](../../docs/reports/2026-11-04-gamepad-fixture-v3/README.md)

默认装配结果为手柄中心向桌内 **53.44 mm**、台面以上 **220.66 mm**、倾角 **45°**。这些数值由夹具几何和名义受压软垫厚度推出，位于 X/Z 的可行区域内。三处接触面来自视觉网格的稳健测量，保留平面拟合残差及 CoACD 对照；不再以网格作随形布尔切刀。尺寸是资产推得，须用卡尺核对；CAD 与打印筛查不认证实物强度、片材压缩或完整操作轨迹。

```bash
# 依赖只进入这个隔离环境；已有环境可直接使用。
/usr/bin/python3.10 -m venv .local/cad-env
.local/cad-env/bin/pip install -r hardware/gamepad_fixture/requirements-cad.txt
.local/cad-env/bin/python hardware/gamepad_fixture/fixture.py
.local/cad-env/bin/python hardware/gamepad_fixture/check_fixture.py
.local/cad-env/bin/python hardware/gamepad_fixture/render.py
.local/cad-env/bin/python hardware/gamepad_fixture/design_sheet.py
```

`out_v3_1/` 包含装配 STEP、打印方向 STEP/STL、以米为单位的分件 GLB、整条夹持带修改器、真实切除体审计、软垫 1:1 模板、检查 JSON 和预览图。`printability/` 对每个打印件执行 60k 随机表面点加全部三角形中心的壁厚、缝宽与悬垂筛查；**任何未豁免标记均失败**。局部支撑和边缘豁免由 [print_policy.py](print_policy.py) 明列，支撑计划必须在切片时落实。当前没有执行 Orca/Bambu CLI 切片。

```bash
.local/cad-env/bin/python hardware/gamepad_fixture/fixture.py --aluminum --out hardware/gamepad_fixture/out_aluminum
.local/cad-env/bin/python hardware/gamepad_fixture/fixture.py --tpu-pads --out hardware/gamepad_fixture/out_tpu
.local/cad-env/bin/python hardware/gamepad_fixture/fixture.py --coupons
.local/cad-env/bin/python hardware/gamepad_fixture/fixture.py --marker-card
# 对替代版、小样和标记卡同样运行 checker：
.local/cad-env/bin/python hardware/gamepad_fixture/check_fixture.py --out hardware/gamepad_fixture/out_tpu
```

卡尺数据用 `--parameters measurements.json` 覆盖 `Parameters().measured / installation / hardware / printing` 的对应字段。例如：

```json
{
  "installation": {"table_thickness": 25, "table_edge_to_apron": 30},
  "measured": {"jaw_pad_thickness": 2, "back_pad_thickness": 2, "pad_compression": 0.15}
}
```

上沿位置、上下跨度、背面平面和 Pair 边界的 `null` 表示重新从资产测量；卡尺值覆盖后实际参与 CAD。端口、肩键净距、前唇与软垫参数的核对清单见报告和验证单。改变尺寸后须重新生成并重跑检查。

完整流水线包含实际 CAD/打印检查、凸块测量、场景导出、camera_3 投影与计算节点原生手部重放：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .local/cad-env/bin/python hardware/gamepad_fixture/pipeline.py
```

运行网格先输出到 `out_v3_1/simulation/runtime/`，然后通过配置的 compute profile 复制到计算工作空间的 `assets/derived/gamepad_fixture_out_v3_1/`，场景以 workspace URI 引用。场景为 `configs/workcell/placement_trials/gamepad_fixture_v3_1.json`。v3 场景与资产保留，默认采集场景和8780服务未切换。

Hirth 适配参考 [Voith 原厂资料](https://www.voith.com/corp-en/products-services/connection-components-couplings/hirth-serrations.html)；基准职责参考 [Carr Lane 定位原则](https://www.carrlane.com/engineering-resources/fixture-design-principles/locating-clamping-principles/ctl)。这里采用塑料可制造的平顶、根部间隙及柔顺垫片，不能沿用工业硬化钢齿盘的精度和承载声明。齿盘先按验证单做小样扭矩与蠕变试验，再决定整机放行。

## V3.1 与 Fusion 编辑

默认新增 `installation.lower_lip_retreat_mm=2`（两侧 |X|=10…28 mm 的前唇高度从3降到1 mm）、`upper_outer_chamfer_mm=2`（仅前端 Z=22…26 mm 的上爪非承压外角）。承压垫、背托、导轨、M8 与 Hirth 保留。v3 的 `out/` 与试验场景保留，v3.1 使用 `out_v3_1/` 和 `gamepad_fixture_v3_1.json`。

[Fusion 编辑与导入说明](USER_EDITING.md) · [AF 验证报告](../../docs/reports/2026-11-05-gamepad-fixture-v3_1/README.md)。运行 `pipeline.py` 生成并验收实际替换后的零件、打印网格和仿真场景；覆盖件统一为桌面朝上装配坐标 mm，不能交回打印朝向 STEP。导入使用 build123d 的 [STEP 导入和装配接口](https://build123d.readthedocs.io/en/latest/assemblies.html)，经同一 export_all 和检查器处理。几何位置门限仅检测明显坐标错误，不认证 Fusion 编辑意图；实体、接触、打印和凸块检查是分别执行的。
