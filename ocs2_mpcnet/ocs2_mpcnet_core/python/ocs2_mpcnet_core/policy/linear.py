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

"""Linear policy.

Provides a class that implements a linear policy.
"""

import torch
from typing import Tuple

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.policy.base import BasePolicy


class LinearPolicy(BasePolicy):
    """Linear policy.

    Class for a simple linear neural network policy.

    Attributes:
        name: A string with the name of the policy.
        observation_dimension: An integer defining the observation (i.e. input) dimension of the policy.
        action_dimension: An integer defining the action (i.e. output) dimension of the policy.
        linear: The linear neural network layer.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the LinearPolicy class.

        Initializes the LinearPolicy class by setting fixed and variable attributes.

        Args:
            config: An instance of the configuration class.
        """
        super().__init__(config)
        self.name = "LinearPolicy"
        self.observation_dimension = config.OBSERVATION_DIM
        self.action_dimension = config.ACTION_DIM
        self.linear = torch.nn.Linear(self.observation_dimension, self.action_dimension)

    def forward(self, observation: torch.Tensor) -> Tuple[torch.Tensor]:
        """Forward method.

        Defines the computation performed at every call. Computes the output tensors from the input tensors.

        Args:
            observation: A (B,O) tensor with the observations.

        Returns:
            action: A (B,A) tensor with the predicted actions.
        """
        scaled_observation = self.scale_observation(observation)
        unscaled_action = self.linear(scaled_observation)
        action = self.scale_action(unscaled_action)
        return (action,)
