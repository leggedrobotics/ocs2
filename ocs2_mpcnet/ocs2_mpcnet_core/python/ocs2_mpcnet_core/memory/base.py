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

"""Base memory.

Provides a base class for all memory classes.
"""

import torch
import numpy as np
from typing import Tuple, List
from abc import ABCMeta, abstractmethod

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core import ScalarFunctionQuadraticApproximation


class BaseMemory(metaclass=ABCMeta):
    """Base memory.

    Provides the interface to all memory classes.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the BaseMemory class.

        Initializes the BaseMemory class.

        Args:
            config: An instance of the configuration class.
        """
        pass

    @abstractmethod
    def push(
        self,
        t: float,
        x: np.ndarray,
        u: np.ndarray,
        p: np.ndarray,
        observation: np.ndarray,
        action_transformation: List[np.ndarray],
        hamiltonian: ScalarFunctionQuadraticApproximation,
    ) -> None:
        """Pushes data into the memory.

        Pushes one data sample into the memory.

        Args:
            t: A float with the time.
            x: A NumPy array of shape (X) with the observed state.
            u: A NumPy array of shape (U) with the optimal input.
            p: A NumPy array of shape (P) tensor for the observed discrete probability distributions of the modes.
            observation: A NumPy array of shape (O) with the generalized times.
            action_transformation: A list containing NumPy arrays of shape (U,A) and (U) with the action transformation.
            hamiltonian: An OCS2 scalar function quadratic approximation representing the Hamiltonian around x and u.
        """
        pass

    @abstractmethod
    def sample(self, batch_size: int) -> Tuple[torch.Tensor, ...]:
        """Samples data from the memory.

        Samples a batch of data from the memory.

        Args:
            batch_size: An integer defining the batch size B.

        Returns:
            A tuple containing the sampled batch of data.
            - t_batch: A (B) tensor with the times.
            - x_batch: A (B,X) tensor with the observed states.
            - u_batch: A (B,U) tensor with the optimal inputs.
            - p_batch: A (B,P) tensor with the observed discrete probability distributions of the modes.
            - observation_batch: A (B,O) tensor with the observations.
            - action_transformation_matrix_batch: A (B,U,A) tensor with the action transformation matrices.
            - action_transformation_vector_batch: A (B,U) tensor with the action transformation vectors.
            - dHdxx_batch: A (B,X,X) tensor with the state-state Hessians of the Hamiltonian approximations.
            - dHdux_batch: A (B,U,X) tensor with the input-state Hessians of the Hamiltonian approximations.
            - dHduu_batch: A (B,U,U) tensor with the input-input Hessians of the Hamiltonian approximations.
            - dHdx_batch: A (B,X) tensor with the state gradients of the Hamiltonian approximations.
            - dHdu_batch: A (B,U) tensor with the input gradients of the Hamiltonian approximations.
            - H_batch: A (B) tensor with the Hamiltonians at the development/expansion points.
        """
        pass

    @abstractmethod
    def __len__(self) -> int:
        """The length of the memory.

        Return the length of the memory given by the current size.

        Returns:
            An integer describing the length of the memory.
        """
        pass
