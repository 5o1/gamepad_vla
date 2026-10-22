# Development guidance

Follow `AGENTS.md` for project boundaries and `README.md` for setup and maintenance.
Keep reusable logic in `src/gamepad_vla`; keep CLI orchestration in `scripts`.
Read machine-specific settings from the ignored site configuration, never from archived reports.
Do not commit private site configuration, raw station inventories, local handoff notes or unreviewed run output.

采集画面与手柄资产固定使用 xbox-gamepad-physics **0.4.1**、SAPIEN **3.0.3**；新建渲染场景调用公开 `robotwin.create_scene`，采集画面复用 `operator_view` 的 teleop mmap/NVENC 管线。
