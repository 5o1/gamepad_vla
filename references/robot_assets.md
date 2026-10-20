# 机器人资产来源

双臂使用 [Linkerbot 的 0.8R 资产](https://github.com/linker-bot/linker-sim-assets/tree/e6bcf44ff0203534e14500cb836f57e9f9492292/src/linker_robot_assets/assets/components/arms/ar5_08/variants/right)，固定提交 `e6bcf44ff0203534e14500cb836f57e9f9492292`。源 URDF 的 SHA-256 为 `867b951997eb0eaf06d726503208c72120f0b8cf67370a967cc8c79c8aac6c47`。

`scripts/prepare_ar5_08r_preview.py` 为左右实例生成唯一连杆和关节名称，保留原始网格、关节轴和限位，网格引用使用相对路径。派生资产位于 `workspace://assets/derived/ar5_08r_e6bcf44f/`。原仓库资产与实机完整型号后缀的对应关系仍待核验。

G20 几何来自采集台提供的左右 URDF 包；资产文件和详细来源清单保存在私有工作目录，不随代码仓库分发。左右手关节映射及 mimic 规则位于 `src/gamepad_vla/workstation/g20_mapping.py`。

手柄使用 `xbox_gamepad_physics` v0.3.1 的发布 wheel 和资源包。其 32 维信号通过控件名称映射至上游 20 维报告，不能按下标混用。
