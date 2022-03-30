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
import numpy as np

from ocs2_mpcnet import config
from ocs2_mpcnet.helper import bdot, bmv


class HamiltonianLoss:
    r"""Hamiltonian loss.

    Uses the linear quadratic approximation of the Hamiltonian as loss:

    .. math::

        H(x,u) = \frac{1}{2} \delta x^T \partial H_{xx} \delta x +\delta u^T \partial H_{ux} \delta x + \frac{1}{2}
        \delta u^T \partial H_{uu} \delta u + \partial H_{x}^T \delta x + \partial H_{u}^T \delta u + H,

    where the state x is of dimension X and the input u is of dimension U.
    """

    def __call__(
        self,
        x_query: torch.Tensor,
        x_nominal: torch.Tensor,
        u_query: torch.Tensor,
        u_nominal: torch.Tensor,
        dHdxx: torch.Tensor,
        dHdux: torch.Tensor,
        dHduu: torch.Tensor,
        dHdx: torch.Tensor,
        dHdu: torch.Tensor,
        H: torch.Tensor,
    ) -> torch.Tensor:
        """Computes the mean Hamiltonian loss.

        Computes the mean Hamiltonian loss for a batch of size B using the provided linear quadratic approximations.

        Args:
            x_query: A (B,X) tensor with the states the Hamiltonian loss should be computed for.
            x_nominal: A (B,X) tensor with the states that were used as development/expansion points.
            u_query: A (B,U) tensor with the inputs the Hamiltonian loss should be computed for.
            u_nominal: A (B,U) tensor with the inputs that were used as development/expansion point.
            dHdxx: A (B,X,X) tensor with the state-state Hessians of the approximations.
            dHdux: A (B,U,X) tensor with the input-state Hessians of the approximations.
            dHduu: A (B,U,U) tensor with the input-input Hessians of the approximations.
            dHdx: A (B,X) tensor with the state gradients of the approximations.
            dHdu: A (B,U) tensor with the input gradients of the approximations.
            H: A (B) tensor with the Hamiltonians at the development/expansion points.

        Returns:
            A (1) tensor containing the mean Hamiltonian loss.
        """
        return torch.mean(self.compute(x_query, x_nominal, u_query, u_nominal, dHdxx, dHdux, dHduu, dHdx, dHdu, H))

    @staticmethod
    def compute(
        x_query: torch.Tensor,
        x_nominal: torch.Tensor,
        u_query: torch.Tensor,
        u_nominal: torch.Tensor,
        dHdxx: torch.Tensor,
        dHdux: torch.Tensor,
        dHduu: torch.Tensor,
        dHdx: torch.Tensor,
        dHdu: torch.Tensor,
        H: torch.Tensor,
    ) -> torch.Tensor:
        """Computes the Hamiltonian losses for a batch.

        Computes the Hamiltonian losses for a batch of size B using the provided linear quadratic approximations.

        Args:
            x_query: A (B,X) tensor with the states the Hamiltonian loss should be computed for.
            x_nominal: A (B,X) tensor with the states that were used as development/expansion points.
            u_query: A (B,U) tensor with the inputs the Hamiltonian loss should be computed for.
            u_nominal: A (B,U) tensor with the inputs that were used as development/expansion point.
            dHdxx: A (B,X,X) tensor with the state-state Hessians of the approximations.
            dHdux: A (B,U,X) tensor with the input-state Hessians of the approximations.
            dHduu: A (B,U,U) tensor with the input-input Hessians of the approximations.
            dHdx: A (B,X) tensor with the state gradients of the approximations.
            dHdu: A (B,U) tensor with the input gradients of the approximations.
            H: A (B) tensor with the Hamiltonians at the development/expansion points.

        Returns:
            A (B) tensor containing the computed Hamiltonian losses.
        """
        if torch.equal(x_query, x_nominal):
            du = torch.sub(u_query, u_nominal)
            return 0.5 * bdot(du, bmv(dHduu, du)) + bdot(dHdu, du) + H
        elif torch.equal(u_query, u_nominal):
            dx = torch.sub(x_query, x_nominal)
            return 0.5 * bdot(dx, bmv(dHdxx, dx)) + bdot(dHdx, dx) + H
        else:
            dx = torch.sub(x_query, x_nominal)
            du = torch.sub(u_query, u_nominal)
            return (
                0.5 * bdot(dx, bmv(dHdxx, dx))
                + bdot(du, bmv(dHdux, dx))
                + 0.5 * bdot(du, bmv(dHduu, du))
                + bdot(dHdx, dx)
                + bdot(dHdu, du)
                + H
            )


class BehavioralCloningLoss:
    r"""Behavioral cloning loss.

    Uses a simple quadratic function as loss:

    .. math::

        BC(u) = \delta u^T R \, \delta u,

    where the input u is of dimension U.

    Attributes:
        R: A (1,U,U) tensor with the input cost matrix.
    """

    def __init__(self, R: np.ndarray) -> None:
        """Initializes the BehavioralCloningLoss class.

        Initializes the BehavioralCloningLoss class by setting fixed attributes.

        Args:
            R: A NumPy array of shape (U, U) with the input cost matrix.
        """
        self.R = torch.tensor(R, device=config.DEVICE, dtype=config.DTYPE).unsqueeze(dim=0)

    def __call__(self, u_predicted: torch.Tensor, u_target: torch.Tensor) -> torch.Tensor:
        """Computes the mean behavioral cloning loss.

        Computes the mean behavioral cloning loss for a batch of size B using the cost matrix.

        Args:
            u_predicted: A (B, U) tensor with the predicted inputs.
            u_target: A (B, U) tensor with the target inputs.

        Returns:
            A (1) tensor containing the mean behavioral cloning loss.
        """
        return torch.mean(self.compute(u_predicted, u_target))

    def compute(self, u_predicted: torch.Tensor, u_target: torch.Tensor) -> torch.Tensor:
        """Computes the behavioral cloning losses for a batch.

        Computes the behavioral cloning losses for a batch of size B using the cost matrix.

        Args:
            u_predicted: A (B, U) tensor with the predicted inputs.
            u_target: A (B, U) tensor with the target inputs.

        Returns:
            A (B) tensor containing the behavioral cloning losses.
        """
        du = torch.sub(u_predicted, u_target)
        return bdot(du, bmv(self.R, du))


class CrossEntropyLoss:
    r"""Cross entropy loss.

    Uses the cross entropy between two discrete probability distributions as loss:

    .. math::

        CE(p_{target}, p_{predicted}) = - \sum_{i=1}^{P} (p_{target,i} \log(p_{predicted,i} + \varepsilon)),

    where the sample space is the set of P individually identified items.

    Attributes:
        epsilon: A (1) tensor with a small epsilon used to stabilize the logarithm.
    """

    def __init__(self, epsilon: float) -> None:
        """Initializes the CrossEntropyLoss class.

        Initializes the CrossEntropyLoss class by setting fixed attributes.

        Args:
            epsilon: A float used to stabilize the logarithm.
        """
        self.epsilon = torch.tensor(epsilon, device=config.DEVICE, dtype=config.DTYPE)

    def __call__(self, p_target: torch.Tensor, p_predicted: torch.Tensor) -> torch.Tensor:
        """Computes the mean cross entropy loss.

        Computes the mean cross entropy loss for a batch, where the logarithm is stabilized by a small epsilon.

        Args:
            p_target: A (B,P) tensor with the target discrete probability distributions.
            p_predicted: A (B,P) tensor with the predicted discrete probability distributions.

        Returns:
            A (1) tensor containing the mean cross entropy loss.
        """
        return torch.mean(self.compute(p_target, p_predicted))

    def compute(self, p_target: torch.Tensor, p_predicted: torch.Tensor) -> torch.Tensor:
        """Computes the cross entropy losses for a batch.

        Computes the cross entropy losses for a batch, where the logarithm is stabilized by a small epsilon.

        Args:
            p_target: A (B,P) tensor with the target discrete probability distributions.
            p_predicted: A (B,P) tensor with the predicted discrete probability distributions.

        Returns:
            A (B) tensor containing the cross entropy losses.
        """
        return -bdot(p_target, torch.log(p_predicted + self.epsilon))
