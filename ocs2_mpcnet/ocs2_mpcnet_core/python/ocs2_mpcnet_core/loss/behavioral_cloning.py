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

"""Behavioral cloning loss.

Provides a class that implements a simple behavioral cloning loss for benchmarking MPC-Net.
"""

import torch

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.loss.base import BaseLoss
from ocs2_mpcnet_core.helper import bdot, bmv


class BehavioralCloningLoss(BaseLoss):
    r"""Behavioral cloning loss.

    Uses a simple quadratic function as loss:

    .. math::

        BC(u) = \delta u^T R \, \delta u,

    where the input u is of dimension U.

    Attributes:
        R: A (1,U,U) tensor with the input cost matrix.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the BehavioralCloningLoss class.

        Initializes the BehavioralCloningLoss class by setting fixed attributes.

        Args:
            config: An instance of the configuration class.
        """
        super().__init__(config)
        self.R = torch.tensor(config.R, device=config.DEVICE, dtype=config.DTYPE).diag().unsqueeze(dim=0)

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
        return torch.mean(self.compute(u_query, u_nominal))

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
