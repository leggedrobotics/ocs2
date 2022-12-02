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

"""Mixture of nonlinear experts policy.

Provides classes that implement a mixture of nonlinear experts policy.
"""

import torch
from typing import Tuple

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.policy.base import BasePolicy
from ocs2_mpcnet_core.helper import bmv


class MixtureOfNonlinearExpertsPolicy(BasePolicy):
    """Mixture of nonlinear experts policy.

    Class for a mixture of experts neural network policy with nonlinear experts, where the hidden layer dimension is the
    mean of the input and output dimensions.

    Attributes:
        name: A string with the name of the policy.
        observation_dimension: An integer defining the observation (i.e. input) dimension of the policy.
        gating_hidden_dimension: An integer defining the dimension of the hidden layer for the gating network.
        expert_hidden_dimension: An integer defining the dimension of the hidden layer for the expert networks.
        action_dimension: An integer defining the action (i.e. output) dimension of the policy.
        expert_number: An integer defining the number of experts.
        gating_net: The gating network.
        expert_nets: The expert networks.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the MixtureOfNonlinearExpertsPolicy class.

        Initializes the MixtureOfNonlinearExpertsPolicy class by setting fixed and variable attributes.

        Args:
            config: An instance of the configuration class.
        """
        super().__init__(config)
        self.name = "MixtureOfNonlinearExpertsPolicy"
        self.observation_dimension = config.OBSERVATION_DIM
        self.gating_hidden_dimension = int((config.OBSERVATION_DIM + config.EXPERT_NUM) / 2)
        self.expert_hidden_dimension = int((config.OBSERVATION_DIM + config.ACTION_DIM) / 2)
        self.action_dimension = config.ACTION_DIM
        self.expert_number = config.EXPERT_NUM
        # gating
        self.gating_net = torch.nn.Sequential(
            torch.nn.Linear(self.observation_dimension, self.gating_hidden_dimension),
            torch.nn.Tanh(),
            torch.nn.Linear(self.gating_hidden_dimension, self.expert_number),
            torch.nn.Softmax(dim=1),
        )
        # experts
        self.expert_nets = torch.nn.ModuleList(
            [
                _NonlinearExpert(i, self.observation_dimension, self.expert_hidden_dimension, self.action_dimension)
                for i in range(self.expert_number)
            ]
        )

    def forward(self, observation: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Forward method.

        Defines the computation performed at every call. Computes the output tensors from the input tensors.

        Args:
            observation: A (B,O) tensor with the observations.

        Returns:
            action: A (B,A) tensor with the predicted actions.
            expert_weights: A (B,E) tensor with the predicted expert weights.
        """
        scaled_observation = self.scale_observation(observation)
        expert_weights = self.gating_net(scaled_observation)
        expert_actions = torch.stack(
            [self.expert_nets[i](scaled_observation) for i in range(self.expert_number)], dim=2
        )
        unscaled_action = bmv(expert_actions, expert_weights)
        action = self.scale_action(unscaled_action)
        return action, expert_weights


class _NonlinearExpert(torch.nn.Module):
    """Nonlinear expert.

    Class for a simple nonlinear neural network expert, where the hidden layer dimension is the mean of the input and
    output dimensions.

    Attributes:
        name: A string with the name of the expert.
        input_dimension: An integer defining the input dimension of the expert.
        hidden_dimension: An integer defining the dimension of the hidden layer.
        output_dimension: An integer defining the output dimension of the expert.
        linear1: The first linear neural network layer.
        activation: The activation to get the hidden layer.
        linear2: The second linear neural network layer.
    """

    def __init__(self, index: int, input_dimension: int, hidden_dimension: int, output_dimension: int) -> None:
        """Initializes the _NonlinearExpert class.

        Initializes the _NonlinearExpert class by setting fixed and variable attributes.

        Args:
            index: An integer with the index of the expert.
            input_dimension: An integer defining the input dimension of the expert.
            hidden_dimension: An integer defining the dimension of the hidden layer.
            output_dimension: An integer defining the output dimension of the expert.
        """
        super().__init__()
        self.name = "NonlinearExpert" + str(index)
        self.input_dimension = input_dimension
        self.hidden_dimension = hidden_dimension
        self.output_dimension = output_dimension
        self.linear1 = torch.nn.Linear(self.input_dimension, self.hidden_dimension)
        self.activation = torch.nn.Tanh()
        self.linear2 = torch.nn.Linear(self.hidden_dimension, self.output_dimension)

    def forward(self, input: torch.Tensor) -> torch.Tensor:
        """Forward method.

        Defines the computation performed at every call. Computes the output tensors from the input tensors.

        Args:
            input: A (B,I) tensor with the inputs.

        Returns:
            output: A (B,O) tensor with the outputs.
        """
        return self.linear2(self.activation(self.linear1(input)))
