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

"""Base loss.

Provides a base class for all loss classes.
"""

import torch
from abc import ABCMeta, abstractmethod

from ocs2_mpcnet_core.config import Config


class BaseLoss(metaclass=ABCMeta):
    """Base loss.

    Provides the interface to all loss classes.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the BaseLoss class.

        Initializes the BaseLoss class.

        Args:
            config: An instance of the configuration class.
        """
        pass

    @abstractmethod
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
        pass
