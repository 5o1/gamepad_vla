"""Shared view publishes measured state without touching collection physics."""
import json
from types import SimpleNamespace
import pytest
from gamepad_vla.workstation.operator_view import OperatorView
from gamepad_vla.workstation.collection_sapien import CollectionSapienBackend
from gamepad_vla.workstation.collection_mujoco import CollectionMujocoBackend


def test_collection_view_publishes_completed_bilateral_state_and_view_control(tmp_path):
    published=[]
    view=OperatorView.__new__(OperatorView)
    view.children=[];view.step_id=0;view.snapshot=SimpleNamespace(write=published.append)
    view.run=tmp_path;view._event_offset=0
    row=dict(hand_q=[[1]*21,[2]*21],arm_q=[[3]*7,[4]*7],gamepad_signal=[.9]+[0]*31,
             control_joint_positions={'button_a_joint':.003},applied_monotonic_ns=123,
             overlay={'values':[.9]+[0]*31})
    view.publish(row)
    result=json.loads(published[0]);assert result['hand_q']==row['hand_q'] and result['arm_q']==row['arm_q']
    assert result['signals']==row['gamepad_signal'] and result['applied_ns']==123
    view.set_view('operator');assert json.loads((tmp_path/'view.json').read_text())=={'view':'operator'}
    with pytest.raises(ValueError):view.set_view('other')
    (tmp_path/'controls.jsonl').write_text('{"action":"stop"}\n')
    assert view.events()==[{'action':'stop'}] and view.events()==[]


@pytest.mark.parametrize('backend',[CollectionSapienBackend,CollectionMujocoBackend])
def test_physics_backends_refuse_the_removed_render_path(backend):
    with pytest.raises(ValueError,match='OperatorView'):backend(None,render=True)
