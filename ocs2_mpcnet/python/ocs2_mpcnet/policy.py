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

from ocs2_mpcnet import config
from ocs2_mpcnet.helper import bmv


class Policy(torch.nn.Module):

    def __init__(self, dim_in, dim_out):
        super().__init__()
        self.name = 'Policy'
        self.dim_in = dim_in
        self.dim_out = dim_out


class LinearPolicy(Policy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t + dim_x, dim_u)
        self.name = 'LinearPolicy'
        self.linear = torch.nn.Linear(self.dim_in, self.dim_out)

    def forward(self, t, x):
        u = self.linear(torch.cat((t, x), dim=1))
        return u, torch.ones(len(u), 1, device=config.device, dtype=config.dtype), u.unsqueeze(dim=2)


class NonlinearPolicy(Policy):

    def __init__(self, dim_t, dim_x, dim_u):
        super().__init__(dim_t + dim_x, dim_u)
        self.name = 'NonlinearPolicy'
        self.dim_hidden = int((self.dim_in + dim_u) / 2)
        self.linear1 = torch.nn.Linear(self.dim_in, self.dim_hidden)
        self.activation = torch.nn.Tanh()
        self.linear2 = torch.nn.Linear(self.dim_hidden, self.dim_out)

    def forward(self, t, x):
        z = self.activation(self.linear1(torch.cat((t, x), dim=1)))
        u = self.linear2(z)
        return u, torch.ones(len(u), 1, device=config.device, dtype=config.dtype), u.unsqueeze(dim=2)


class MixtureOfLinearExpertsPolicy(Policy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t + dim_x, dim_u)
        self.name = 'MixtureOfLinearExpertsPolicy'
        self.num_experts = num_experts
        # gating
        self.gating_net = torch.nn.Sequential(
            torch.nn.Linear(self.dim_in, self.num_experts),
            torch.nn.Softmax(dim=1)
        )
        # experts
        self.expert_nets = torch.nn.ModuleList(
            [LinearExpert(i, self.dim_in, self.dim_out) for i in range(self.num_experts)]
        )

    def forward(self, t, x):
        p = self.gating_net(torch.cat((t, x), dim=1))
        U = torch.stack([self.expert_nets[i](torch.cat((t, x), dim=1)) for i in range(self.num_experts)], dim=2)
        u = bmv(U, p)
        return u, p, U


class MixtureOfNonlinearExpertsPolicy(Policy):

    def __init__(self, dim_t, dim_x, dim_u, num_experts):
        super().__init__(dim_t + dim_x, dim_u)
        self.name = 'MixtureOfNonlinearExpertsPolicy'
        self.num_experts = num_experts
        self.dim_hidden_expert = int((self.dim_in + dim_u) / 2)
        self.dim_hidden_gating = int((self.dim_in + num_experts) / 2)
        # gating
        self.gating_net = torch.nn.Sequential(
            torch.nn.Linear(self.dim_in, self.dim_hidden_gating),
            torch.nn.Tanh(),
            torch.nn.Linear(self.dim_hidden_gating, self.num_experts),
            torch.nn.Softmax(dim=1)
        )
        # experts
        self.expert_nets = torch.nn.ModuleList(
            [NonlinearExpert(i, self.dim_in, self.dim_hidden_expert, self.dim_out) for i in range(self.num_experts)]
        )

    def forward(self, t, x):
        p = self.gating_net(torch.cat((t, x), dim=1))
        U = torch.stack([self.expert_nets[i](torch.cat((t, x), dim=1)) for i in range(self.num_experts)], dim=2)
        u = bmv(U, p)
        return u, p, U


class LinearExpert(torch.nn.Module):

    def __init__(self, id, dim_in, dim_out):
        super().__init__()
        self.name = 'LinearExpert' + str(id)
        self.dim_in = dim_in
        self.dim_out = dim_out
        self.linear = torch.nn.Linear(self.dim_in, self.dim_out)

    def forward(self, input):
        return self.linear(input)


class NonlinearExpert(torch.nn.Module):

    def __init__(self, id, dim_in, dim_hidden, dim_out):
        super().__init__()
        self.name = 'NonlinearExpert' + str(id)
        self.dim_in = dim_in
        self.dim_hidden = dim_hidden
        self.dim_out = dim_out
        self.linear1 = torch.nn.Linear(self.dim_in, self.dim_hidden)
        self.activation = torch.nn.Tanh()
        self.linear2 = torch.nn.Linear(self.dim_hidden, self.dim_out)

    def forward(self, input):
        return self.linear2(self.activation(self.linear1(input)))
