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

"""Loss classes.

Provides classes with loss functions for MPC-Net, such as the Hamiltonian loss and the cross entropy loss for training
the gating network of a mixture of experts network. Additionally, a simple behavioral cloning loss is implemented.
"""

import torch

from ocs2_mpcnet_core import config
from ocs2_mpcnet.helper import bdot, bmv


class Hamiltonian:
    """Hamiltonian loss.

    Uses the linear quadratic approximation of the Hamiltonian as loss:
        H(x,u) = 1/2 dx' dHdxx dx + du' dHdux dx + 1/2 du' dHduu du + dHdx' dx + dHdu' du + H,
    where the state x is of dimension X and the input u is of dimension U.
    """

    @staticmethod
    def compute_sample(x_inquiry, x_nominal, u_inquiry, u_nominal, dHdxx, dHdux, dHduu, dHdx, dHdu, H):
        """Computes the Hamiltonian for one sample.

        Computes the Hamiltonian for one sample using the provided linear quadratic approximation.

        Args:
            x_inquiry: A (X) tensor with the state the Hamiltonian should be computed for.
            x_nominal: A (X) tensor with the state that was used as development/expansion point.
            u_inquiry: A (U) tensor with the input the Hamiltonian should be computed for.
            u_nominal: A (U) tensor with the input that was used as development/expansion point.
            dHdxx: A (X,X) tensor with the state-state Hessian of the approximation.
            dHdux: A (U,X) tensor with the input-state Hessian of the approximation.
            dHduu: A (U,U) tensor with the input-input Hessian of the approximation.
            dHdx: A (X) tensor with the state gradient of the approximation.
            dHdu: A (U) tensor with the input gradient of the approximation.
            H: A (1) tensor with the Hamiltonian at the development/expansion point.

        Returns:
            A (1) tensor containing the computed Hamiltonian.
        """
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
        """Computes the Hamiltonians for a batch.

        Computes the Hamiltonians for a batch of size B using the provided linear quadratic approximations.

        Args:
            x_inquiry: A (B,X) tensor with the states the Hamiltonians should be computed for.
            x_nominal: A (B,X) tensor with the states that were used as development/expansion points.
            u_inquiry: A (B,U) tensor with the inputs the Hamiltonians should be computed for.
            u_nominal: A (B,U) tensor with the inputs that were used as development/expansion point.
            dHdxx: A (B,X,X) tensor with the state-state Hessians of the approximations.
            dHdux: A (B,U,X) tensor with the input-state Hessians of the approximations.
            dHduu: A (B,U,U) tensor with the input-input Hessians of the approximations.
            dHdx: A (B,X) tensor with the state gradients of the approximations.
            dHdu: A (B,U) tensor with the input gradients of the approximations.
            H: A (B) tensor with the Hamiltonians at the development/expansion points.

        Returns:
            A (B) tensor containing the computed Hamiltonians.
        """
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
    """Behavioral cloning loss.

    Uses a simple quadratic function as loss:
        BC(u) = du' R du,
    where the input u is of dimension U.

    Attributes:
        R_sample: A (U,U) tensor with the input cost matrix R for one sample.
        R_batch: A (1,U,U) tensor with the input cost matrix for a batch.
    """

    def __init__(self, R):
        """Initializes the BehavioralCloning class.

        Initializes the BehavioralCloning class by setting fixed attributes.

        Args:
            R: A NumPy array of shape (U, U) with the input cost matrix.
        """
        self.R_sample = torch.tensor(R, device=config.device, dtype=config.dtype)
        self.R_batch = self.R_sample.unsqueeze(dim=0)

    def compute_sample(self, u_predicted, u_target):
        """Computes the behavioral cloning loss for one sample.

        Computes the behavioral cloning loss for one sample using the cost matrix.

        Args:
            u_predicted: A (U) tensor with the predicted input.
            u_target: A (U) tensor with the target input.

        Returns:
            A (1) tensor containing the behavioral cloning loss.
        """
        du = torch.sub(u_predicted, u_target)
        return torch.dot(du, torch.mv(self.R_sample, du))

    def compute_batch(self, u_predicted, u_target):
        """Computes the behavioral cloning loss for a batch.

        Computes the behavioral cloning loss for a batch of size B using the cost matrix.

        Args:
            u_predicted: A (B, U) tensor with the predicted inputs.
            u_target: A (B, U) tensor with the target inputs.

        Returns:
            A (B) tensor containing the behavioral cloning losses.
        """
        du = torch.sub(u_predicted, u_target)
        return bdot(du, bmv(self.R_batch, du))


class CrossEntropy:
    """Cross entropy loss.

    Uses the cross entropy between two discrete probability distributions as loss:
        CE(p_target, p_predicted) = - sum(p_target * log(p_predicted)),
    where the sample space is the set of P individually identified items.

    Attributes:
        epsilon: A (1) tensor with a small epsilon used to stabilize the logarithm.
    """

    def __init__(self, epsilon):
        """Initializes the CrossEntropy class.

        Initializes the CrossEntropy class by setting fixed attributes.

        Args:
            epsilon: A float used to stabilize the logarithm.
        """
        self.epsilon = torch.tensor(epsilon, device=config.device, dtype=config.dtype)

    def compute_sample(self, p_target, p_predicted):
        """Computes the cross entropy loss for one sample.

        Computes the cross entropy loss for one sample, where the logarithm is stabilized by a small epsilon.

        Args:
            p_target: A (P) tensor with the target discrete probability distribution.
            p_predicted: A (P) tensor with the predicted discrete probability distribution.

        Returns:
            A (1) tensor containing the cross entropy loss.
        """
        return -torch.dot(p_target, torch.log(p_predicted + self.epsilon))

    def compute_batch(self, p_target, p_predicted):
        """Computes the cross entropy loss for a batch.

        Computes the cross entropy loss for a batch, where the logarithm is stabilized by a small epsilon.

        Args:
            p_target: A (B,P) tensor with the target discrete probability distributions.
            p_predicted: A (B,P) tensor with the predicted discrete probability distributions.

        Returns:
            A (B) tensor containing the cross entropy losses.
        """
        return -bdot(p_target, torch.log(p_predicted + self.epsilon))
