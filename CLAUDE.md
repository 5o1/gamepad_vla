# Development guidance

Follow `AGENTS.md` for project boundaries and `README.md` for setup and maintenance.
Keep reusable logic in `src/gamepad_vla`; keep CLI orchestration in `scripts`.
Read machine-specific settings from the ignored site configuration, never from archived reports.
Do not commit private site configuration, raw station inventories, local handoff notes or unreviewed run output.
