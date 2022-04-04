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

"""Cross entropy loss.

Provides a class that implements the cross entropy loss for training a gating network of a mixture of experts network.
"""

import torch

from ocs2_mpcnet_core import config
from ocs2_mpcnet_core.helper import bdot, bmv


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
