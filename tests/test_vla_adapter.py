from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")
nn = torch.nn

from gamepad_vla.vla.adapter import ConditionedPi05, TargetTokens, block_mask


class TinyExpert(nn.Module):
    """Contract test double, not PI weights or a manipulation policy."""
    def __init__(self, width):
        super().__init__()
        self.qkv = nn.Linear(width, 3 * width)

    def forward(self, *, attention_mask, inputs_embeds, **kwargs):
        prefix, suffix = inputs_embeds
        tokens = torch.cat((prefix, suffix), 1)
        q, k, v = self.qkv(tokens).chunk(3, -1)
        scores = q @ k.transpose(-1, -2) / q.shape[-1] ** .5
        scores = scores.masked_fill(~attention_mask, -1e9)
        output = scores.softmax(-1) @ v
        return (output[:, :prefix.shape[1]], output[:, prefix.shape[1]:]), None


class TinyBase(nn.Module):
    def __init__(self):
        super().__init__()
        self.pi05 = True
        self.config = SimpleNamespace(action_dim=7, action_horizon=3)
        self.action_in_proj = nn.Linear(7, 16)
        self.action_out_proj = nn.Linear(16, 7)
        self.paligemma_with_expert = TinyExpert(16)

    def embed_suffix(self, state, actions, time):
        valid = torch.ones(actions.shape[:2], dtype=torch.bool)
        blocks = torch.zeros_like(valid)
        blocks[:, 0] = True
        return self.action_in_proj(actions), valid, blocks, None

    def _prepare_attention_masks_4d(self, mask):
        return mask


def inputs():
    return dict(targets=torch.rand(2, 5, 20, requires_grad=True),
                target_seconds=torch.arange(5.).expand(2, -1) / 10,
                target_valid=torch.tensor([[True, True, False, False, False]] * 2),
                robot_state=torch.randn(2, 9, requires_grad=True),
                feedback=torch.zeros(2, 20))


def run(model, data):
    return model.sample_actions(torch.ones(2, 2, 16), torch.ones(2, 2, dtype=torch.bool),
                                torch.zeros(2, 2, dtype=torch.bool),
                                torch.ones(2, 3, 7), steps=2, **data)


def test_frozen_base_still_passes_gradient_to_game_and_proprio():
    model = ConditionedPi05(TinyBase(), 9)
    model.base.requires_grad_(False)
    data = inputs()
    run(model, data).square().mean().backward()
    assert data["targets"].grad[:, :2].abs().sum() > 0
    assert data["targets"].grad[:, 2:].abs().sum() == 0
    assert data["robot_state"].grad.abs().sum() > 0
    assert model.condition.target[0].weight.grad.abs().sum() > 0
    assert all(p.grad is None for p in model.base.parameters())


def test_padding_values_cannot_change_output():
    model = ConditionedPi05(TinyBase(), 9)
    data = inputs()
    before = run(model, data)
    changed = dict(data, targets=data["targets"].detach().clone())
    changed["targets"][:, 2:] = float("nan")
    torch.testing.assert_close(run(model, changed), before)


def test_condition_changes_actions():
    model = ConditionedPi05(TinyBase(), 9)
    data = inputs()
    before = run(model, data)
    assert not torch.allclose(before, run(model, dict(data, targets=data["targets"] + .3)))


def test_prefix_cannot_read_conditions_or_actions():
    valid = torch.tensor([[True, True, True, True, False, True, True]])
    boundaries = torch.tensor([[False, False, True, False, False, True, False]])
    mask = block_mask(valid, boundaries)
    assert not mask[0, :2, 2:].any()
    assert not mask[0, 2:5, 5:].any()
    assert mask[0, 5:, :4].all()
    assert not mask[0, :, 4].any()


def test_empty_target_rejected():
    data = inputs()
    with pytest.raises(ValueError):
        TargetTokens(9, 16)(data["targets"], data["target_seconds"],
                            torch.zeros(2, 5, dtype=torch.bool),
                            data["robot_state"], data["feedback"])
