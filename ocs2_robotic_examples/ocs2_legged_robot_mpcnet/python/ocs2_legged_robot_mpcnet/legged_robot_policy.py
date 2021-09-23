import torch

from ocs2_mpcnet import policy
from ocs2_mpcnet.helper import bmv, bmm

from ocs2_legged_robot_mpcnet import legged_robot_config as config


input_scaling = torch.tensor(config.input_scaling, device=config.device, dtype=config.dtype).diag().unsqueeze(dim=0)
input_bias = torch.tensor(config.input_bias, device=config.device, dtype=config.dtype).unsqueeze(dim=0)
input_bias_stacked = torch.stack([input_bias for i in range(config.EXPERT_NUM)], dim=2)


def u_transform(u):
    return bmv(input_scaling, u) + input_bias


def U_transform(U):
    return bmm(input_scaling, U) + input_bias_stacked


class LeggedRobotLinearPolicy(policy.LinearPolicy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = 'LeggedRobotLinearPolicy'

    def forward(self, t, x):
        u, p, U = super().forward(t, x)
        return u_transform(u), p, U_transform(U)


class LeggedRobotNonlinearPolicy(policy.NonlinearPolicy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = 'LeggedRobotNonlinearPolicy'

    def forward(self, t, x):
        u, p, U = super().forward(t, x)
        return u_transform(u), p, U_transform(U)


class LeggedRobotMixtureOfLinearExpertsPolicy(policy.MixtureOfLinearExpertsPolicy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = 'LeggedRobotMixtureOfLinearExpertsPolicy'

    def forward(self, t, x):
        u, p, U = super().forward(t, x)
        return u_transform(u), p, U_transform(U)


class LeggedRobotMixtureOfNonlinearExpertsPolicy(policy.MixtureOfNonlinearExpertsPolicy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = 'LeggedRobotMixtureOfNonlinearExpertsPolicy'

    def forward(self, t, x):
        u, p, U = super().forward(t, x)
        return u_transform(u), p, U_transform(U)
