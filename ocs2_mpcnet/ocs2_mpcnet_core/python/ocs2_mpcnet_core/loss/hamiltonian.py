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

"""Hamiltonian loss.

Provides a class that implements the Hamiltonian loss for MPC-Net.
"""

import torch

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.loss.base import BaseLoss
from ocs2_mpcnet_core.helper import bdot, bmv


class HamiltonianLoss(BaseLoss):
    r"""Hamiltonian loss.

    Uses the linear quadratic approximation of the Hamiltonian as loss:

    .. math::

        H(x,u) = \frac{1}{2} \delta x^T \partial H_{xx} \delta x +\delta u^T \partial H_{ux} \delta x + \frac{1}{2}
        \delta u^T \partial H_{uu} \delta u + \partial H_{x}^T \delta x + \partial H_{u}^T \delta u + H,

    where the state x is of dimension X and the input u is of dimension U.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the HamiltonianLoss class.

        Initializes the HamiltonianLoss class.

        Args:
            config: An instance of the configuration class.
        """
        super().__init__(config)

    def __call__(
        self,
        x_query: torch.Tensor,
        x_nominal: torch.Tensor,
        u_query: torch.Tensor,
        u_nominal: torch.Tensor,
        p_query: torch.Tensor,
        p_nominal: torch.Tensor,
        dHdxx: torch.Tensor,
        dHdux: torch.Tensor,
        dHduu: torch.Tensor,
        dHdx: torch.Tensor,
        dHdu: torch.Tensor,
        H: torch.Tensor,
    ) -> torch.Tensor:
        """Computes the loss.

        Computes the mean loss for a batch.

        Args:
            x_query: A (B,X) tensor with the query (e.g. predicted) states.
            x_nominal: A (B,X) tensor with the nominal (e.g. target) states.
            u_query: A (B,U) tensor with the query (e.g. predicted) inputs.
            u_nominal: A (B,U) tensor with the nominal (e.g. target) inputs.
            p_query: A (B,P) tensor with the query (e.g. predicted) discrete probability distributions.
            p_nominal: A (B,P) tensor with the nominal (e.g. target) discrete probability distributions.
            dHdxx: A (B,X,X) tensor with the state-state Hessians of the Hamiltonian approximations.
            dHdux: A (B,U,X) tensor with the input-state Hessians of the Hamiltonian approximations.
            dHduu: A (B,U,U) tensor with the input-input Hessians of the Hamiltonian approximations.
            dHdx: A (B,X) tensor with the state gradients of the Hamiltonian approximations.
            dHdu: A (B,U) tensor with the input gradients of the Hamiltonian approximations.
            H: A (B) tensor with the Hamiltonians at the nominal points.

        Returns:
            A (1) tensor containing the mean loss.
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
            dHdxx: A (B,X,X) tensor with the state-state Hessians of the Hamiltonian approximations.
            dHdux: A (B,U,X) tensor with the input-state Hessians of the Hamiltonian approximations.
            dHduu: A (B,U,U) tensor with the input-input Hessians of the Hamiltonian approximations.
            dHdx: A (B,X) tensor with the state gradients of the Hamiltonian approximations.
            dHdu: A (B,U) tensor with the input gradients of the Hamiltonian approximations.
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
