import torch

from ocs2_mpcnet import policy 

from ocs2_legged_robot_mpcnet import legged_robot_config as config


def input_transform(u):
    input_bias = torch.tensor(config.input_bias, device=config.device, dtype=config.dtype)
    input_scaling = torch.tensor(config.input_scaling, device=config.device, dtype=config.dtype).diag()
    return torch.add(input_bias, torch.matmul(u, input_scaling))


class LeggedRobotLinearPolicy(policy.LinearPolicy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = 'LeggedRobotLinearPolicy'

    def forward(self, t, x):
        p, u = super().forward(t, x)
        return p, input_transform(u)


class LeggedRobotNonlinearPolicy(policy.NonlinearPolicy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = 'LeggedRobotNonlinearPolicy'

    def forward(self, t, x):
        p, u = super().forward(t, x)
        return p, input_transform(u)


class LeggedRobotMixtureOfLinearExpertsPolicy(policy.MixtureOfLinearExpertsPolicy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = 'LeggedRobotMixtureOfLinearExpertsPolicy'

    def forward(self, t, x):
        p, u = super().forward(t, x)
        return p, input_transform(u)


class LeggedRobotMixtureOfNonlinearExpertsPolicy(policy.MixtureOfNonlinearExpertsPolicy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = 'LeggedRobotMixtureOfNonlinearExpertsPolicy'

    def forward(self, t, x):
        p, u = super().forward(t, x)
        return p, input_transform(u)
