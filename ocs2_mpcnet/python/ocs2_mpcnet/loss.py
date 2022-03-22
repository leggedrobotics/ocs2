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

from ocs2_mpcnet_core import config
from ocs2_mpcnet.helper import bdot, bmv


class Hamiltonian:

    # Uses the linear quadratic approximation of the Hamiltonian as loss
    # H(x,u) = 1/2 dx' dHdxx dx + du' dHdux dx + 1/2 du' dHduu du + dHdx' dx + dHdu' du + H

    @staticmethod
    def compute_sample(x_inquiry, x_nominal, u_inquiry, u_nominal, dHdxx, dHdux, dHduu, dHdx, dHdu, H):
        if torch.equal(x_inquiry, x_nominal):
            du = torch.sub(u_inquiry, u_nominal)
            return 0.5 * torch.dot(du, torch.mv(dHduu, du)) + torch.dot(dHdu, du) + H
        elif torch.equal(u_inquiry, u_nominal):
            dx = torch.sub(x_inquiry, x_nominal)
            return 0.5 * torch.dot(dx, torch.mv(dHdxx, dx)) + torch.dot(dHdx, dx) + H
        else:
            dx = torch.sub(x_inquiry, x_nominal)
            du = torch.sub(u_inquiry, u_nominal)
            return (
                0.5 * torch.dot(dx, torch.mv(dHdxx, dx))
                + torch.dot(du, torch.mv(dHdux, dx))
                + 0.5 * torch.dot(du, torch.mv(dHduu, du))
                + torch.dot(dHdx, dx)
                + torch.dot(dHdu, du)
                + H
            )

    @staticmethod
    def compute_batch(x_inquiry, x_nominal, u_inquiry, u_nominal, dHdxx, dHdux, dHduu, dHdx, dHdu, H):
        if torch.equal(x_inquiry, x_nominal):
            du = torch.sub(u_inquiry, u_nominal)
            return 0.5 * bdot(du, bmv(dHduu, du)) + bdot(dHdu, du) + H
        elif torch.equal(u_inquiry, u_nominal):
            dx = torch.sub(x_inquiry, x_nominal)
            return 0.5 * bdot(dx, bmv(dHdxx, dx)) + bdot(dHdx, dx) + H
        else:
            dx = torch.sub(x_inquiry, x_nominal)
            du = torch.sub(u_inquiry, u_nominal)
            return (
                0.5 * bdot(dx, bmv(dHdxx, dx))
                + bdot(du, bmv(dHdux, dx))
                + 0.5 * bdot(du, bmv(dHduu, du))
                + bdot(dHdx, dx)
                + bdot(dHdu, du)
                + H
            )


class BehavioralCloning:

    # Uses a simple quadratic function as loss
    # BC(u) = du' R du

    def __init__(self, R):
        self.R_sample = torch.tensor(R, device=config.device, dtype=config.dtype)
        self.R_batch = self.R_sample.unsqueeze(dim=0)

    def compute_sample(self, u_predicted, u_target):
        du = torch.sub(u_predicted, u_target)
        return torch.dot(du, torch.mv(self.R_sample, du))

    def compute_batch(self, u_predicted, u_target):
        du = torch.sub(u_predicted, u_target)
        return bdot(du, bmv(self.R_batch, du))


class CrossEntropy:

    # Uses the cross entropy between two probability distributions as loss
    # CE(p_target, p_predicted) = - sum(p_target * log(p_predicted))

    def __init__(self, epsilon):
        self.epsilon = torch.tensor(epsilon, device=config.device, dtype=config.dtype)

    def compute_sample(self, p_target, p_predicted):
        return -torch.dot(p_target, torch.log(p_predicted + self.epsilon))

    def compute_batch(self, p_target, p_predicted):
        return -bdot(p_target, torch.log(p_predicted + self.epsilon))
