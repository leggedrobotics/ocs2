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

"""Base policy.

Provides a base class for all policy classes.
"""

import torch
from typing import Tuple
from abc import ABCMeta, abstractmethod

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.helper import bmv


class BasePolicy(torch.nn.Module, metaclass=ABCMeta):
    """Base policy.

    Provides the interface to all policy classes.

    Attributes:
        observation_scaling: A (1,O,O) tensor for the observation scaling.
        action_scaling: A (1,A,A) tensor for the action scaling.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the BasePolicy class.

        Initializes the BasePolicy class.

        Args:
            config: An instance of the configuration class.
        """
        super().__init__()
        self.observation_scaling = (
            torch.tensor(config.OBSERVATION_SCALING, device=config.DEVICE, dtype=config.DTYPE).diag().unsqueeze(dim=0)
        )
        self.action_scaling = (
            torch.tensor(config.ACTION_SCALING, device=config.DEVICE, dtype=config.DTYPE).diag().unsqueeze(dim=0)
        )

    @abstractmethod
    def forward(self, observation: torch.Tensor) -> Tuple[torch.Tensor, ...]:
        """Forward method.

        Defines the computation performed at every call. Computes the output tensors from the input tensors.

        Args:
            observation: A (B,O) tensor with the observations.

        Returns:
            tuple: A tuple with the predictions, e.g. containing a (B,A) tensor with the predicted actions.
        """
        pass

    def scale_observation(self, observation: torch.Tensor) -> torch.Tensor:
        """Scale observation.

        Scale the observation with a fixed diagonal matrix.

        Args:
            observation: A (B,O) tensor with the observations.

        Returns:
            scaled_observation: A (B,O) tensor with the scaled observations.
        """
        return bmv(self.observation_scaling, observation)

    def scale_action(self, action: torch.Tensor) -> torch.Tensor:
        """Scale action.

        Scale the action with a fixed diagonal matrix.

        Args:
            action: A (B,A) tensor with the actions.

        Returns:
            scaled_action: A (B,A) tensor with the scaled actions.
        """
        return bmv(self.action_scaling, action)
