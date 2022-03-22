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

import torch

from ocs2_mpcnet import policy
from ocs2_mpcnet.helper import bmv, bmm

from ocs2_legged_robot_mpcnet import legged_robot_config as config


input_scaling = torch.tensor(config.input_scaling, device=config.device, dtype=config.dtype).diag().unsqueeze(dim=0)
input_bias = torch.tensor(config.input_bias, device=config.device, dtype=config.dtype).unsqueeze(dim=0)


def u_transform(u):
    return bmv(input_scaling, u) + input_bias


class LeggedRobotLinearPolicy(policy.LinearPolicy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = 'LeggedRobotLinearPolicy'

    def forward(self, t, x):
        u = super().forward(t, x)
        return u_transform(u)


class LeggedRobotNonlinearPolicy(policy.NonlinearPolicy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t, dim_x, dim_u)
        self.name = 'LeggedRobotNonlinearPolicy'

    def forward(self, t, x):
        u = super().forward(t, x)
        return u_transform(u)


class LeggedRobotMixtureOfLinearExpertsPolicy(policy.MixtureOfLinearExpertsPolicy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = 'LeggedRobotMixtureOfLinearExpertsPolicy'

    def forward(self, t, x):
        u, p = super().forward(t, x)
        return u_transform(u), p


class LeggedRobotMixtureOfNonlinearExpertsPolicy(policy.MixtureOfNonlinearExpertsPolicy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t, dim_x, dim_u, num_experts)
        self.name = 'LeggedRobotMixtureOfNonlinearExpertsPolicy'

    def forward(self, t, x):
        u, p = super().forward(t, x)
        return u_transform(u), p
