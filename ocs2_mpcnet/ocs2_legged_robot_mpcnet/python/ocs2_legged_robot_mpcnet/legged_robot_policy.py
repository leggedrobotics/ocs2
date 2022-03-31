###############################################################################
# Copyright (c) 2022, Farbod Farshidian. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#  * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

"""Legged robot policy classes.

Provides robot-specific classes for different neural network policies for legged robot.

Todo:
    * Delete this file as part of refactoring, as it will be become obsolete.
"""

import torch

from ocs2_mpcnet_core.policy import linear, mixture_of_linear_experts, mixture_of_nonlinear_experts, nonlinear
from ocs2_mpcnet_core.helper import bmv, bmm

from ocs2_legged_robot_mpcnet import legged_robot_config as config


input_scaling = torch.tensor(config.INPUT_SCALING, device=config.DEVICE, dtype=config.DTYPE).diag().unsqueeze(dim=0)
input_bias = torch.tensor(config.INPUT_BIAS, device=config.DEVICE, dtype=config.DTYPE).unsqueeze(dim=0)


def u_transform(u: torch.Tensor) -> torch.Tensor:
    """Control input transformation.

    Transforms the predicted control input by scaling and adding a bias.

    Args:
        u: A (B,U) tensor with the predicted control inputs.

    Returns:
        u: A (B,U) tensor with the transformed control inputs.
    """
    return bmv(input_scaling, u) + input_bias


class LeggedRobotLinearPolicy(linear.LinearPolicy):
    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = "LeggedRobotLinearPolicy"

    def forward(self, t, x):
        u = super().forward(t, x)
        return u_transform(u)


class LeggedRobotNonlinearPolicy(nonlinear.NonlinearPolicy):
    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = "LeggedRobotNonlinearPolicy"

    def forward(self, t, x):
        u = super().forward(t, x)
        return u_transform(u)


class LeggedRobotMixtureOfLinearExpertsPolicy(mixture_of_linear_experts.MixtureOfLinearExpertsPolicy):
    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = "LeggedRobotMixtureOfLinearExpertsPolicy"

    def forward(self, t, x):
        u, p = super().forward(t, x)
        return u_transform(u), p


class LeggedRobotMixtureOfNonlinearExpertsPolicy(mixture_of_nonlinear_experts.MixtureOfNonlinearExpertsPolicy):
    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = "LeggedRobotMixtureOfNonlinearExpertsPolicy"

    def forward(self, t, x):
        u, p = super().forward(t, x)
        return u_transform(u), p
