"""Gymnasium residual task: a fixed G20 thumb physically follows D-pad targets.

Reference: APRPianist e31c010, IK base + bounded PPO residual, structured state
and ten future targets. No human motion prior is enabled without gamepad data.
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces

from gamepad_vla.signals import DPAD_NAMES, SDL3_NAMES
from gamepad_vla.targets import TargetSequence, dpad_only
from gamepad_vla.tasks.fixed_gamepad import FixedGamepadPhysics, THUMB_JOINTS

DPAD_INDICES = np.array([SDL3_NAMES.index(n) for n in DPAD_NAMES])


def target_program(rng, duration_s, hold_seconds):
    """Balanced five-state random commands with explicit neutral holds."""
    times, reports, now = [], [], 0.0
    while now < duration_s:
        for label in rng.permutation(5):
            if now >= duration_s:
                break
            report = np.zeros(20, dtype=np.float32)
            if label:
                report[label - 1] = 1.0
            times.append(now); reports.append(report)
            now += float(rng.uniform(*hold_seconds))
    times.append(duration_s)
    reports.append(np.zeros(20, dtype=np.float32))
    return TargetSequence(np.array(times), np.array(reports), 'synthetic-relative')


def compute_signal_reward(values, target, available, threshold, specification):
    """Contact reward; the reference variant rejects digitally wrong presses.

    Centering/scaling 0.5 * (tolerance + no wrong key) gives tolerance -
    wrong_press. Unlike RoboPianist's requested-key-only tolerance, this task
    compares the full available signal vector and rewards a neutral match.
    """
    measured = values[DPAD_INDICES]
    on = target > .5
    if specification['type'] == 'continuous_product':
        positive = float(np.mean(np.clip(measured[on],0,1))) if np.any(on) else 1.
        negative = float(1-np.max(measured[~on])) if np.any(~on) else 1.
        return positive*negative
    if specification['type'] != 'key_match_no_wrong':
        raise ValueError('unknown signal reward type')
    desired = np.zeros(32)
    desired[DPAD_INDICES] = target
    undesired = np.asarray(available,dtype=bool).copy()
    undesired[DPAD_INDICES] &= ~on
    wrong = bool(np.any(np.abs(values[undesired]) > threshold))
    error = float(np.linalg.norm((values-desired)[available]))
    scaled = max(0.,error-specification['bound'])/specification['margin']
    tolerance = specification['value_at_margin'] ** (scaled*scaled)
    return float(tolerance-wrong)


class FixedDpadEnv(gym.Env):
    """Observation goal is the command for the upcoming control interval.

    Reward and logged target use that interval's command and its physical result.
    The returned observation advances to the next interval, including at switches.
    """
    metadata = {'render_modes': []}

    def __init__(self, config, *, sequence=None, work_dir=None):
        super().__init__()
        self.config = config
        self.physics = FixedGamepadPhysics(config, work_dir=work_dir)
        self.sequence = sequence
        if sequence is not None and not np.all(dpad_only(sequence.reports)):
            raise ValueError('this task accepts D-pad-only target reports')
        self.dt = config['control_timestep_s']
        self.horizon = round(config['episode_seconds'] / self.dt)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(len(THUMB_JOINTS),), dtype=np.float32)
        dimension = 42 + 32 + 4 * (config['lookahead_steps'] + 1) + 4 + 4
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(dimension,), dtype=np.float32)
        self.previous_action = np.zeros(4)
        self.index = 0
        self._last_label = 0
        self._retract = False

    def _goals(self):
        times = (self.index + np.arange(self.config['lookahead_steps'] + 1)) * self.dt
        return self.current_sequence.hold_at(times)[:, :4]

    def _label(self):
        goal = self._goals()[0]
        return int(np.argmax(goal) + 1) if np.any(goal) else 0

    def _base_command(self):
        label = self._label()
        if self.config.get('retract_before_switch', False):
            if label != self._last_label and self._last_label != 0 and label != 0:
                self._retract = True
            if label == 0:
                self._retract = False
            if self._retract:
                # Actual world fingertip height controls the phase. Advancing
                # the target or querying an observation cannot bypass contact.
                height_from_hover = float(np.dot(
                    self.physics.fingertip()-self.physics.ik.goal_points_world[0],
                    self.physics.ik.normal_world))
                clearance = self.config['retract_clearance_m']-self.config['hover_height_m']
                if height_from_hover >= clearance:
                    self._retract = False
        self._last_label = label
        return self.physics.ik.commands[0 if self._retract else label]

    def _observation(self):
        p = self.physics
        state = p.gamepad.gamepad_state()
        return np.concatenate((p.data.qpos[p.qidx], p.data.qvel[p.vidx], state.values,
                               self._goals().ravel(), self._base_command(),
                               self.previous_action)).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.physics.reset()
        warmup_peak = 0.0
        warmup_steps = round(self.config.get('warmup_seconds', 0.) / self.dt)
        for _ in range(warmup_steps):
            result = self.physics.step(self.physics.ik.commands[0])
            warmup_peak = max(warmup_peak, result['max_contact_force_n'])
        self.current_sequence = self.sequence or target_program(
            self.np_random, self.config['episode_seconds'], self.config['target_hold_seconds'])
        self.index = 0
        self._last_label = 0
        self._retract = False
        self.previous_action = np.zeros(4)
        return self._observation(), {'ik_error_m': self.physics.ik.errors,
            'warmup_seconds': warmup_steps*self.dt,'warmup_peak_contact_force_n': warmup_peak,
            'initial_gamepad_signal': np.asarray(self.physics.gamepad.gamepad_state().values).tolist()}

    def set_target(self, report):
        """Receive a causal full-20 goal, holding it until another arrives.

        Removes unreceived future goals and changes no physical state or clock.
        """
        report = np.asarray(report, dtype=np.float32)
        if report.shape != (20,) or not np.isfinite(report).all() or not dpad_only(report):
            raise ValueError('target must be a finite full-20 D-pad-only report')
        now = self.index * self.dt
        old = self.current_sequence
        keep = np.searchsorted(old.times_s, now, side='left')
        self.current_sequence = TargetSequence(np.r_[old.times_s[:keep],now],
                                              np.vstack([old.reports[:keep],report]),old.clock_id)
        return self._observation()

    def step(self, action):
        action = np.asarray(action, dtype=float)
        if action.shape != (4,) or not np.isfinite(action).all():
            raise ValueError('action must contain four finite thumb residuals')
        action = np.clip(action, -1., 1.)
        label = self._label()
        target = self._goals()[0].copy()
        base = self._base_command().copy()
        retracting = self._retract
        physical = self.physics.step(base + self.config['residual_scale_rad'] * action)
        state = self.physics.gamepad.gamepad_state()
        values = np.asarray(state.values)
        measured = values[DPAD_INDICES]
        signal_reward = compute_signal_reward(values,target,np.asarray(state.available_mask,dtype=bool),
            self.config.get('digital_threshold',.3),self.config.get('signal_reward',{'type':'continuous_product'}))
        tip_error = float(np.linalg.norm(self.physics.fingertip() - self.physics.ik.goal_points_world[label]))
        reference = self.physics.ik.goal_points_world[0 if retracting else label]
        reference_error = float(np.linalg.norm(self.physics.fingertip()-reference))
        finger_reward = float(np.exp(-.5 * (reference_error / .01) ** 2))
        weights = self.config['reward']
        reward = (weights['signal'] * signal_reward + weights['fingertip'] * finger_reward
                  - weights['energy'] * physical['energy_j'] / self.dt
                  - weights.get('force', 0.) * physical['max_contact_force_n']
                  - weights['action_change'] * float(np.sum((action-self.previous_action)**2)))
        info = dict(physical, target=target.tolist(), dpad=measured.tolist(),
                    gamepad_signal=values.tolist(), gamepad_mask=np.asarray(state.available_mask).tolist(),
                    label=label, base_command=base.tolist(), retracting=retracting, time_s=self.index * self.dt, result_time_s=(self.index+1)*self.dt,
                    signal_reward=signal_reward, fingertip_error_m=tip_error,
                    reference_error_m=reference_error)
        self.index += 1
        self.previous_action = action
        return self._observation(), float(reward), False, self.index >= self.horizon, info

    def close(self):
        self.physics.close()


def score_rollout(rows, threshold=.3):
    """Micro/per-direction F1, neutral accuracy, wrong controls and response delays."""
    target = np.array([r['target'] for r in rows]) > threshold
    actual = np.array([r['dpad'] for r in rows]) > threshold
    tp = (target & actual).sum(axis=0)
    fp = (~target & actual).sum(axis=0)
    fn = (target & ~actual).sum(axis=0)
    def ratio(a, b):
        return float(a/b) if b else None
    neutral = ~target.any(axis=1)
    non_dpad = np.ones(32, dtype=bool); non_dpad[DPAD_INDICES] = False
    signals = np.array([r['gamepad_signal'] for r in rows])
    masks = np.array([r['gamepad_mask'] for r in rows])
    delays = {'press': [], 'release': []}
    misses = {'press': 0, 'release': 0}
    already = {'press': 0, 'release': 0}
    for channel in range(4):
        previous = np.r_[False, target[:-1,channel]]
        changes = np.flatnonzero(previous != target[:,channel])
        for start in changes:
            desired = target[start,channel]
            subsequent = changes[changes > start]
            end = subsequent[0] if len(subsequent) else len(rows)
            matching = np.flatnonzero(actual[start:end,channel] == desired)
            key = 'press' if desired else 'release'
            if start > 0 and actual[start-1,channel] == desired:
                already[key] += 1
                continue
            if len(matching):
                first = start + matching[0]
                delays[key].append(rows[first]['result_time_s'] - rows[start]['time_s'])
            else:
                misses[key] += 1
    return {
        'digital_threshold': threshold, 'steps': len(rows), 'target_positive_samples': int(target.sum()),
        'precision': ratio(tp.sum(), (tp+fp).sum()), 'recall': ratio(tp.sum(), (tp+fn).sum()),
        'f1': ratio(2*tp.sum(), (2*tp+fp+fn).sum()),
        'per_direction': {name: {'tp': int(tp[i]), 'fp': int(fp[i]), 'fn': int(fn[i]),
                                'f1': ratio(2*tp[i], 2*tp[i]+fp[i]+fn[i])} for i,name in enumerate(DPAD_NAMES)},
        'exact_state_accuracy': float(np.mean(np.all(target == actual, axis=1))),
        'neutral_accuracy': float(np.mean(~actual[neutral].any(axis=1))) if neutral.any() else None,
        'non_dpad_false_press_rate': float(np.mean(np.any((np.abs(signals[:,non_dpad]) > threshold) & masks[:,non_dpad],axis=1))),
        'max_contact_force_n': max(r['max_contact_force_n'] for r in rows),
        'min_contact_distance_m': min(r['min_contact_distance_m'] for r in rows),
        'response': {key: {'events_measured': len(v), 'events_missed': misses[key],
                           'events_already_satisfied': already[key],
                           'median_delay_s': float(np.median(v)) if v else None,
                           'p95_delay_s': float(np.percentile(v,95)) if v else None} for key,v in delays.items()},
    }
