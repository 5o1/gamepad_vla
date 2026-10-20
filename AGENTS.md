# Scope

This directory is ConCon's downstream part: dexterous-hand data collection,
generation and training. A hand controller, not necessarily a VLA, takes upstream
gamepad signals and physically actuates the gamepad to reproduce them. The pi0.5
conditioning path under `src/gamepad_vla/vla/` is deferred.

- Do not edit `resnake_gym`, which owns game logic and game RL, or
  `xbox_gamepad_physics`, which owns the gamepad asset. Copy their assets in and
  record provenance.
- Research new designs against primary sources before implementation. Record the
  adaptation and evidence limits in README.md.
- Human hand-motion data may be used, for example as an adversarial motion prior
  (user decision, 2026-10-14). Ask before introducing BC or DAgger training.
- Run training and other compute on the configured compute node and workspace,
  and only on GPUs that are free. Model weights, datasets, converted assets and
  run outputs stay there.
- Do not overwrite shared openpi, RoboTwin, Isaac or Python environments.
- The LinkerHand collection station is read-only by default and runs no
  training. Sending physical robot or hand commands, including CAN query
  frames, needs a separate hardware execution authorization.
- Passing gradient tests is not evidence of manipulation success.
