import runpy
from pathlib import Path

import numpy as np
import pytest


def test_replay_selects_actions_by_name_not_state_or_column(tmp_path):
    load = runpy.run_path(Path(__file__).parents[1]/'scripts/glove_link/replay_sender.py')['load_actions']
    names = [f'left_hand_motor_{i}' for i in reversed(range(20))] + ['arm']
    action = np.tile(np.r_[np.arange(19,-1,-1),0],(3,1))
    path = tmp_path/'input.npz'
    np.savez(path,names=names,units=['motor_0_255']*20+['rad'],
             timestamp=[5,5.0333,5.0666],action=action,**{'observation.state':action+100})
    t, values = load(path)
    np.testing.assert_allclose(t,[0,.0333,.0666])
    np.testing.assert_array_equal(values,np.tile(np.arange(20),(3,1)))
    np.savez(path,names=names,units=['rad']*21,timestamp=[0,1],action=action[:2])
    with pytest.raises(ValueError,match='units'):
        load(path)
