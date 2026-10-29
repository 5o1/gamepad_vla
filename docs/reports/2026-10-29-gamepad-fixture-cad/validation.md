# v2-Z 最终检查汇总

更新2026-10-30。三版已从当前v2源码重新生成；检查命令均exit 0。详细来源、控件缺口与原始手部状态见 [v2详细验证](v2-validation.md)。几何通过不代表已执行实物加载或完整机器人手动作。

| 检查项 | PETG/硅胶 | 铝板 | TPU |
|---|---|---|---|
| `assembly_pass` | 通过 | 通过 | 通过 |
| `exported_mesh_validity_pass` | 通过 | 通过 | 通过 |
| `control_travel_pass` | 通过 | 通过 | 通过 |
| `grip_back_pass` | 通过 | 通过 | 通过 |
| `nominal_pose_pass` | 通过 | 通过 | 通过 |
| `build_volume_pass` | 通过 | 通过 | 通过 |
| `span_pass` | 通过 | 通过 | 通过 |
| `slider_motion_pass` | 通过 | 通过 | 通过 |
| `slider_lock_backing_pass` | 通过 | 通过 | 通过 |
| `print_modifier_alignment_pass` | 通过 | 通过 | 通过 |
| `simulation_export_pass` | 通过 | 通过 | 通过 |
| `shell_clearance_pass` | 通过 | 通过 | 通过 |
| `analytic_pivot_pass` | 通过 | 通过 | 通过 |
| `mounting_band_pass` | 通过 | 通过 | 通过 |
| `root_unchanged_from_v1_pass` | 通过 | 通过 | 通过 |
| `hand_saved_states_pass` | 通过 | 通过 | 通过 |
| `hand_point_ik_diagnostics_pass` | 通过 | 通过 | 通过 |
| `clamp_hand_saved_states_pass` | 通过 | 通过 | 通过 |
| `clamp_hand_point_ik_diagnostics_pass` | 通过 | 通过 | 通过 |
| `clamp_fixture_pass` | 通过 | 通过 | 通过 |
| `cross_clamp_access_pass` | 通过 | 通过 | 通过 |
| `operator_own_clamp_access_pass` | 通过 | 通过 | 通过 |
| 完整控制动作认证 | **未认证** | **未认证** | **未认证** |
| 实物位姿认证 | **未执行** | **未执行** | **未执行** |

## 最小间隙与定位

| v2版本 | 14保存状态至夹具mm | 65诊断状态至夹具mm | 机器人手至C包络mm | 导出STL定位误差mm |
|---|---:|---:|---:|---:|
| out | 18.34211 | 3.30580 | 63.40993 | 0.00000710 |
| out_aluminum | 18.34211 | 3.30580 | 76.60993 | 0.00000884 |
| out_tpu | 18.34211 | 3.30580 | 63.40993 | 0.00000710 |

默认两C夹操作手对自身非操作结构、另一横杆旋转区分别约15mm；此处是暂定R75操作包络的几何结果。全宽220×60mm带平整检查通过，单个修改器对应258064mm³实料。

## 控件自身完整行程

| 控件 | 最小间隙mm | 扣采样余量后mm |
|---|---:|---:|
| a | 31.6406 | 31.2906 |
| b | 37.8868 | 37.5368 |
| left_shoulder | 13.1465 | 12.7965 |
| right_shoulder | 13.1462 | 12.7962 |
| pair | 1.2557 | 0.9057 |
| dpad | 8.9151 | 8.5651 |
| view | 20.5803 | 20.2303 |
| menu | 20.5886 | 20.2386 |
| left_stick | 19.2311 | 18.8811 |
| right_stick | 5.8571 | 5.5071 |
| left_trigger | 17.8208 | 17.4708 |
| right_trigger | 17.8208 | 17.4708 |
| x | 25.5459 | 25.1959 |
| guide | 3.2333 | 2.8833 |
| y | 28.1214 | 27.7714 |

采样圆弧误差解析上界最大约0.05153mm，实际统一保留至少0.35mm余量。以上是控件自身URDF限位，不是缺失的手指动作轨迹。

## 更新后的力学假设

30N×110mm=3.3Nm扰动，设计4Nm，齿根剪切估算9.16Nm，预紧/分离比约1.91。94mm完全无桌面支承条带模型：

| 版本 | 最大应力MPa | 挠度mm | 转角° |
|---|---:|---:|---:|
| out | 1.065625 | 0.676506 | 0.758781 |
| out_aluminum | 11.840278 | 0.217876 | 0.244374 |

打印版悬空转角可超0.5°，不能据此认证受载位姿；正常安装要求薄片与桌面连续贴合，并执行报告的载荷/蠕变试验。软垫差分压缩仍可产生1.2276°或组合1.4679mm上界，未获得物理精度认证。

## 输出与范围

轴线外伸14.6232mm、固定柱/齿盘外伸48.6041mm；全部夹具最远处仍由导轨盖限定为80.0096mm。40°刻线对应39.756021°，原root四元数与位置保持不变。

所有默认打印件包围盒（推荐打印方向）：

| 打印件 | mm |
|---|---|
| base_printed | 220.00 × 38.00 × 160.00 |
| column | 71.11 × 236.21 × 96.00 |
| cradle_lower_jaw | 137.00 × 142.00 × 42.00 |
| slider_upper_jaw | 32.00 × 75.00 × 63.05 |
| lock_screw_spacer | 10.00 × 10.00 × 3.00 |
| guide_cap | 46.00 × 20.00 × 31.60 |
| guide_rod_1 | 6.00 × 6.00 × 88.60 |
| guide_rod_2 | 6.00 × 6.00 × 88.60 |
| M6_hand_knob | 42.86 × 42.86 × 14.00 |
| pressure_shoe_1 | 44.00 × 16.00 × 40.00 |
| pressure_shoe_2 | 44.00 × 16.00 × 40.00 |

原始JSON分别为 [默认](../../../hardware/gamepad_fixture/out/checks.json)、[铝板](../../../hardware/gamepad_fixture/out_aluminum/checks.json)、[TPU](../../../hardware/gamepad_fixture/out_tpu/checks.json)。每版均另含hand_states.json、hand_clearances.json、材质/变换manifest及C夹包络。铝板和衬片的预期共面数值接触保留原始体积/法向厚度，未放宽其他0.05mm³通用阈值。
