"""Task semantics: targets at switches, missed events and URDF import changes."""
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from gamepad_vla.tasks.fixed_gamepad import project_inertia_tensor
from gamepad_vla.targets import TargetSequence

gym = pytest.importorskip('gymnasium')
from gamepad_vla.tasks.dpad_env import FixedDpadEnv, DPAD_INDICES, score_rollout


def test_projection_corrects_triangle_with_smallest_norm_and_keeps_valid_inputs():
    original = np.diag([1.,2.,3.03])
    projected = project_inertia_tensor(original)
    np.testing.assert_allclose(np.diag(projected), [1.01,2.01,3.02])
    assert np.linalg.eigvalsh(projected)[-1] <= np.linalg.eigvalsh(projected)[:2].sum() + 1e-14
    valid = np.diag([2.,3.,4.])
    np.testing.assert_array_equal(project_inertia_tensor(valid),valid)
    np.testing.assert_array_equal(original,np.diag([1.,2.,3.03]))
    with pytest.raises(ValueError,match='non-positive'):
        project_inertia_tensor(np.diag([0.,1.,2.]))


class FakePhysics:
    def __init__(self,*args,**kwargs):
        self.qidx = self.vidx = np.arange(21)
        self.data = SimpleNamespace(qpos=np.zeros(21),qvel=np.zeros(21))
        self.ik = SimpleNamespace(commands=np.zeros((5,4)),errors=[0.]*5,
                                  goal_points_world=np.zeros((5,3)))
        self.values = np.zeros(32)
        self.gamepad = SimpleNamespace(gamepad_state=lambda: SimpleNamespace(
            values=self.values,available_mask=np.ones(32,dtype=bool)))
    def step(self,target):
        return dict(energy_j=0.,max_contact_force_n=0.,min_contact_distance_m=0.)
    def fingertip(self): return np.zeros(3)
    def reset(self): pass
    def close(self): pass


def test_switch_scores_previous_interval_and_observes_next_goal():
    reports=np.zeros((3,20));reports[1,0]=1
    sequence=TargetSequence(np.array([0.,.02,.04]),reports,'test')
    config=dict(control_timestep_s=.02,episode_seconds=.06,lookahead_steps=1,
                residual_scale_rad=.1,reward=dict(signal=1.,fingertip=0.,energy=0.,action_change=0.))
    with patch('gamepad_vla.tasks.dpad_env.FixedGamepadPhysics',FakePhysics):
        env=FixedDpadEnv(config,sequence=sequence)
    env.reset(seed=0)
    obs,reward,_,_,info=env.step(np.zeros(4))
    assert info['target']==[0,0,0,0] and reward==1.
    np.testing.assert_array_equal(obs[74:78],[1,0,0,0])
    _,reward,_,_,info=env.step(np.zeros(4))
    assert info['target']==[1,0,0,0] and reward==0.
    assert info['result_time_s']==.04
    env.reset(seed=0)
    assert env.index==0 and not env._goals()[0].any()


def test_neutral_cannot_hide_missing_presses_and_delay_includes_physics_interval():
    rows=[]
    for i,target in enumerate(([0,0,0,0],[1,0,0,0],[1,0,0,0],[0,0,0,0])):
        rows.append(dict(target=target,dpad=[0.]*4,gamepad_signal=[0.]*32,gamepad_mask=[True]*32,
                         time_s=i*.02,result_time_s=(i+1)*.02,max_contact_force_n=0.,min_contact_distance_m=0.))
    result=score_rollout(rows)
    assert result['f1']==0. and result['neutral_accuracy']==1.
    assert result['response']['press']['events_missed']==1
    assert result['response']['release']['median_delay_s'] is None
    assert result['response']['release']['events_already_satisfied']==1
    rows[2]['dpad'][0]=1.
    rows[2]['gamepad_signal'][DPAD_INDICES[0]]=1.
    result=score_rollout(rows)
    assert result['response']['press']['median_delay_s']==pytest.approx(.04)


def test_direction_change_retracts_until_measured_clearance_without_changing_goal():
    reports=np.zeros((3,20));reports[0,0]=1;reports[1:,1]=1
    sequence=TargetSequence(np.array([0.,.02,.04]),reports,'test')
    config=dict(control_timestep_s=.02,episode_seconds=.06,lookahead_steps=1,
                residual_scale_rad=.1,retract_before_switch=True,retract_clearance_m=.004,
                hover_height_m=.006,reward=dict(signal=1.,fingertip=0.,energy=0.,action_change=0.))
    with patch('gamepad_vla.tasks.dpad_env.FixedGamepadPhysics',FakePhysics):
        env=FixedDpadEnv(config,sequence=sequence)
    env.physics.ik.commands=np.arange(5)[:,None]*np.ones((5,4))
    env.physics.ik.goal_points_world[0]=[0,0,.006]
    env.physics.ik.normal_world=np.array([0,0,1])
    env.reset(seed=0)
    observation,_,_,_,info=env.step(np.zeros(4))
    assert info['base_command']==[1.]*4 and not info['retracting']
    # The next goal is down but base control lifts. It still earns zero task
    # reward when the actual gamepad is neutral rather than holding down.
    np.testing.assert_array_equal(observation[74:78],[0,1,0,0])
    _,reward,_,_,info=env.step(np.zeros(4))
    assert info['target']==[0,1,0,0] and info['base_command']==[0.]*4
    assert info['retracting'] and reward==0.
    env.physics.fingertip=lambda:np.array([0.,0.,.0041])
    np.testing.assert_array_equal(env._base_command(),[2.]*4)
    assert not env._retract


def test_causal_target_update_holds_current_report_without_physics_or_future_leakage():
    reports=np.zeros((3,20));reports[1,0]=1;reports[2,1]=1
    sequence=TargetSequence(np.array([0.,.02,.04]),reports,'test')
    config=dict(control_timestep_s=.02,episode_seconds=.06,lookahead_steps=3,
                residual_scale_rad=.1,reward=dict(signal=1.,fingertip=0.,energy=0.,action_change=0.))
    with patch('gamepad_vla.tasks.dpad_env.FixedGamepadPhysics',FakePhysics):
        env=FixedDpadEnv(config,sequence=sequence)
    env.reset(seed=0)
    actual_before=env.physics.data.qpos.copy()
    goal=np.zeros(20);goal[3]=1
    env.set_target(goal)
    assert env.index==0
    np.testing.assert_array_equal(env.physics.data.qpos,actual_before)
    np.testing.assert_array_equal(env._goals(),np.tile([0,0,0,1],(4,1)))
    env.step(np.zeros(4))
    goal[:4]=[1,0,0,0]
    env.set_target(goal)
    assert env.index==1
    np.testing.assert_array_equal(env._goals(),np.tile([1,0,0,0],(4,1)))


def test_reset_preparation_is_separate_from_target_clock_and_reported():
    config=dict(control_timestep_s=.02,episode_seconds=.1,lookahead_steps=1,
                residual_scale_rad=.03,warmup_seconds=.06,
                target_hold_seconds=[.3,.8],reward=dict(signal=1.,fingertip=0.,energy=0.,action_change=0.))
    with patch('gamepad_vla.tasks.dpad_env.FixedGamepadPhysics',FakePhysics):
        env=FixedDpadEnv(config)
    with patch.object(env.physics,'step',wraps=env.physics.step) as step:
        _,info=env.reset(seed=0)
        assert step.call_count==3
    assert env.index==0 and info['warmup_seconds']==pytest.approx(.06)
    assert info['warmup_peak_contact_force_n']==0.


def test_reference_key_reward_prefers_release_to_conflicting_digital_press():
    from gamepad_vla.tasks.dpad_env import compute_signal_reward
    target=np.array([1.,0,0,0]);mask=np.ones(32,dtype=bool)
    neutral=np.zeros(32)
    correct=neutral.copy();correct[DPAD_INDICES[0]]=1
    conflicting=correct.copy();conflicting[DPAD_INDICES[2]]=.31
    reference=dict(type='key_match_no_wrong',bound=.05,margin=.5,value_at_margin=.1)
    rewards=[compute_signal_reward(v,target,mask,.3,reference) for v in (correct,neutral,conflicting)]
    assert rewards[0]==1. and rewards[0]>rewards[1]>rewards[2]
    assert rewards[2]<0.
    # This is the measured objective mismatch in the original product reward:
    # it rates the conflicting digital press above a clean release.
    product=dict(type='continuous_product')
    assert compute_signal_reward(conflicting,target,mask,.3,product)>compute_signal_reward(neutral,target,mask,.3,product)
    non_task=correct.copy();non_task[6]=1.
    assert compute_signal_reward(non_task,target,mask,.3,reference)<0.


def test_digital_hold_reward_does_not_incentivize_bottoming_out():
    from gamepad_vla.tasks.dpad_env import compute_signal_reward
    target=np.array([1.,0,0,0]);mask=np.ones(32,dtype=bool)
    partial=np.zeros(32);partial[DPAD_INDICES[0]]=.5
    full=partial.copy();full[DPAD_INDICES[0]]=1.
    missing=partial.copy();missing[DPAD_INDICES[0]]=.29
    wrong=partial.copy();wrong[DPAD_INDICES[2]]=.31
    spec=dict(type='digital_hold_no_wrong',bound=.05,margin=.5,value_at_margin=.1,on_margin=.1)
    partial_reward=compute_signal_reward(partial,target,mask,.3,spec)
    assert partial_reward==compute_signal_reward(full,target,mask,.3,spec)==1.
    assert compute_signal_reward(missing,target,mask,.3,spec)<partial_reward
    assert compute_signal_reward(wrong,target,mask,.3,spec)<0.


def test_composed_scene_keeps_child_asset_paths_reloadable(tmp_path):
    import xml.etree.ElementTree as ET
    from gamepad_vla.tasks.fixed_gamepad import resolve_asset_files
    resources=tmp_path/'resources'
    artifact=resources/'mujoco/full_controls';artifact.mkdir(parents=True)
    (artifact/'manifest.json').write_text('{}')
    original='<mujoco><asset><mesh name="pad" file="meshes/pad.obj"/><texture name="paint" file="textures/rgb.png"/><mesh name="arm" file="/already/absolute/arm.stl"/></asset></mujoco>'
    for base in (resources,artifact):
        xml=ET.fromstring(resolve_asset_files(original,base))
        assert xml.find('asset/mesh[@name="pad"]').get('file')==str(artifact/'meshes/pad.obj')
        assert xml.find('asset/texture').get('file')==str(artifact/'textures/rgb.png')
        assert xml.find('asset/mesh[@name="arm"]').get('file')=='/already/absolute/arm.stl'
