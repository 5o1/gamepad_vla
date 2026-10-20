"""Differentiable target tokens for the official openpi PI0Pytorch (pi05=True).

Attention block semantics follow openpi/pi0_pytorch.py at 215abfb2.
This wrapper owns only new projections; it does not patch upstream files.
"""

import torch
from torch import nn


def block_mask(valid, boundaries):
    blocks = boundaries.long().cumsum(-1)
    return (blocks[:, None, :] <= blocks[:, :, None]) & (
        valid[:, None, :] & valid[:, :, None]
    )


class TargetTokens(nn.Module):
    def __init__(self, state_dim, width):
        super().__init__()
        self.state = nn.Linear(state_dim, width)
        self.feedback = nn.Linear(20, width)
        self.target = nn.Sequential(nn.Linear(21, width), nn.SiLU(), nn.Linear(width, width))

    def forward(self, targets, seconds, valid, state, feedback):
        if targets.ndim != 3 or targets.shape[-1] != 20:
            raise ValueError("targets must have shape [B,H,20]")
        if seconds.shape != targets.shape[:2] or valid.shape != seconds.shape:
            raise ValueError("target times and validity must have shape [B,H]")
        if valid.dtype != torch.bool or not valid.any(-1).all():
            raise ValueError("each sample needs at least one valid target")
        if state.shape != (targets.shape[0], self.state.in_features):
            raise ValueError("robot_state shape does not match configured state_dim")
        if feedback.shape != (targets.shape[0], 20):
            raise ValueError("feedback must have shape [B,20]")
        # Mask before projection: arbitrary padding cannot leak through NaNs.
        target_input = torch.cat((targets, seconds[..., None]), -1)
        target_input = torch.where(valid[..., None], target_input, 0.)
        if not torch.isfinite(target_input).all():
            raise ValueError("valid target values and times must be finite")
        dtype = self.state.weight.dtype
        tokens = torch.cat((self.state(state.to(dtype))[:, None],
                            self.feedback(feedback.to(dtype))[:, None],
                            self.target(target_input.to(dtype))), 1)
        mask = torch.cat((torch.ones(valid.shape[0], 2, dtype=torch.bool,
                                     device=valid.device), valid), 1)
        return tokens, mask


class ConditionedPi05(nn.Module):
    def __init__(self, base, state_dim):
        super().__init__()
        if not base.pi05:
            raise ValueError("this adapter requires pi05=True")
        self.base = base
        self.condition = TargetTokens(state_dim, base.action_in_proj.out_features)

    def velocity(self, prefix, prefix_valid, prefix_blocks, noisy_actions, timestep,
                 *, targets, target_seconds, target_valid, robot_state, feedback):
        batch, horizon, _ = noisy_actions.shape
        # pi05 ignores legacy state here; our continuous state is an explicit token.
        suffix, suffix_valid, suffix_blocks, adarms = self.base.embed_suffix(
            robot_state, noisy_actions, timestep)
        condition, condition_valid = self.condition(
            targets, target_seconds, target_valid, robot_state, feedback)
        condition = condition.to(suffix.dtype)
        cblocks = torch.zeros_like(condition_valid)
        cblocks[:, 0] = True
        suffix = torch.cat((condition, suffix), 1)
        valid = torch.cat((prefix_valid, condition_valid, suffix_valid), 1)
        blocks = torch.cat((prefix_blocks, cblocks, suffix_blocks), 1)
        mask = self.base._prepare_attention_masks_4d(block_mask(valid, blocks))
        positions = valid.long().cumsum(1) - 1
        (_, output), _ = self.base.paligemma_with_expert.forward(
            attention_mask=mask, position_ids=positions, past_key_values=None,
            inputs_embeds=[prefix, suffix], use_cache=False,
            adarms_cond=[None, adarms])
        return self.base.action_out_proj(output[:, -horizon:].to(
            self.base.action_out_proj.weight.dtype))

    def sample_actions(self, prefix, prefix_valid, prefix_blocks, noise, *, steps=4,
                       **condition):
        """Fixed-step differentiable Euler solve; no implicit detach/no_grad."""
        if not isinstance(steps, int) or steps < 1:
            raise ValueError("steps must be a positive integer")
        actions = noise
        for i in range(steps):
            t = torch.full((noise.shape[0],), 1 - i / steps,
                           dtype=noise.dtype, device=noise.device)
            velocity = self.velocity(prefix, prefix_valid, prefix_blocks,
                                     actions, t, **condition)
            actions = actions - velocity / steps
        return actions
