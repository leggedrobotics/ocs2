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


class LinearPolicy(torch.nn.Module):
    """Linear policy.

    Class for a simple linear neural network policy.

    Attributes:
        name: A string with the name of the policy.
        dim_in: An integer defining the input dimension of the policy.
        dim_out: An integer defining the output dimension of the policy.
        linear: The linear neural network layer.
    """

    def __init__(self, dim_t: int, dim_x: int, dim_u: int) -> None:
        """Initializes the LinearPolicy class.

        Initializes the LinearPolicy class by setting fixed and variable attributes.

        Args:
            dim_t: An integer defining the generalized time dimension.
            dim_x: An integer defining the relative state dimension.
            dim_u: An integer defining the control input dimension.
        """
        super().__init__()
        self.name = "LinearPolicy"
        self.dim_in = dim_t + dim_x
        self.dim_out = dim_u
        self.linear = torch.nn.Linear(self.dim_in, self.dim_out)

    def forward(self, t: torch.Tensor, x: torch.Tensor) -> torch.Tensor:
        """Forward method.

        Defines the computation performed at every call. Computes the output tensors from the input tensors.

        Args:
            t: A (B,T) tensor with the generalized times.
            x: A (B,X) tensor with the relative states.

        Returns:
            u: A (B,U) tensor with the predicted control inputs.
        """
        return self.linear(torch.cat((t, x), dim=1))
