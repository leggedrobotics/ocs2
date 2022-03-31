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
import numpy as np

from ocs2_mpcnet_core import config
from ocs2_mpcnet_core.helper import bdot, bmv


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
