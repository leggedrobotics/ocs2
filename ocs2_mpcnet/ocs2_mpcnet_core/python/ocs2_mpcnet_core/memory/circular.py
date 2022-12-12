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

"""Circular memory.

Provides a class that implements a circular memory.
"""

import torch
import numpy as np
from typing import Tuple, List

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.memory.base import BaseMemory
from ocs2_mpcnet_core import ScalarFunctionQuadraticApproximation


class CircularMemory(BaseMemory):
    """Circular memory.

    Stores data in a circular memory that overwrites old data if the size of the memory reaches its capacity.

    Attributes:
        capacity: An integer defining the capacity of the memory.
        size: An integer giving the current size of the memory.
        position: An integer giving the current position in the memory.
        t: A (C) tensor for the times.
        x: A (C,X) tensor for the observed states.
        u: A (C,U) tensor for the optimal inputs.
        p: A (C,P) tensor for the observed discrete probability distributions of the modes.
        observation: A (C,O) tensor for the observations.
        action_transformation_matrix: A (C,U,A) tensor for the action transformation matrices.
        action_transformation_vector: A (C,U) tensor for the action transformation vectors.
        dHdxx: A (C,X,X) tensor for the state-state Hessians of the Hamiltonian approximations.
        dHdux: A (C,U,X) tensor for the input-state Hessians of the Hamiltonian approximations.
        dHduu: A (C,U,U) tensor for the input-input Hessians of the Hamiltonian approximations.
        dHdx: A (C,X) tensor for the state gradients of the Hamiltonian approximations.
        dHdu: A (C,U) tensor for the input gradients of the Hamiltonian approximations.
        H: A (C) tensor for the Hamiltonians at the development/expansion points.
    """

    def __init__(self, config: Config) -> None:
        """Initializes the CircularMemory class.

        Initializes the CircularMemory class by setting fixed attributes, initializing variable attributes and
        pre-allocating memory.

        Args:
            config: An instance of the configuration class.
        """
        # init variables
        self.device = config.DEVICE
        self.capacity = config.CAPACITY
        self.size = 0
        self.position = 0
        # pre-allocate memory
        self.t = torch.zeros(config.CAPACITY, device=config.DEVICE, dtype=config.DTYPE)
        self.x = torch.zeros(config.CAPACITY, config.STATE_DIM, device=config.DEVICE, dtype=config.DTYPE)
        self.u = torch.zeros(config.CAPACITY, config.INPUT_DIM, device=config.DEVICE, dtype=config.DTYPE)
        self.p = torch.zeros(config.CAPACITY, config.EXPERT_NUM, device=config.DEVICE, dtype=config.DTYPE)
        self.observation = torch.zeros(
            config.CAPACITY, config.OBSERVATION_DIM, device=config.DEVICE, dtype=config.DTYPE
        )
        self.action_transformation_matrix = torch.zeros(
            config.CAPACITY, config.INPUT_DIM, config.ACTION_DIM, device=config.DEVICE, dtype=config.DTYPE
        )
        self.action_transformation_vector = torch.zeros(
            config.CAPACITY, config.INPUT_DIM, device=config.DEVICE, dtype=config.DTYPE
        )
        self.dHdxx = torch.zeros(
            config.CAPACITY, config.STATE_DIM, config.STATE_DIM, device=config.DEVICE, dtype=config.DTYPE
        )
        self.dHdux = torch.zeros(
            config.CAPACITY, config.INPUT_DIM, config.STATE_DIM, device=config.DEVICE, dtype=config.DTYPE
        )
        self.dHduu = torch.zeros(
            config.CAPACITY, config.INPUT_DIM, config.INPUT_DIM, device=config.DEVICE, dtype=config.DTYPE
        )
        self.dHdx = torch.zeros(config.CAPACITY, config.STATE_DIM, device=config.DEVICE, dtype=config.DTYPE)
        self.dHdu = torch.zeros(config.CAPACITY, config.INPUT_DIM, device=config.DEVICE, dtype=config.DTYPE)
        self.H = torch.zeros(config.CAPACITY, device=config.DEVICE, dtype=config.DTYPE)

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
        # push data into memory
        # note: - torch.as_tensor: no copy as data is a ndarray of the corresponding dtype and the device is the cpu
        #       - torch.Tensor.copy_: copy performed together with potential dtype and device change
        self.t[self.position].copy_(torch.as_tensor(t, dtype=None, device=torch.device("cpu")))
        self.x[self.position].copy_(torch.as_tensor(x, dtype=None, device=torch.device("cpu")))
        self.u[self.position].copy_(torch.as_tensor(u, dtype=None, device=torch.device("cpu")))
        self.p[self.position].copy_(torch.as_tensor(p, dtype=None, device=torch.device("cpu")))
        self.observation[self.position].copy_(torch.as_tensor(observation, dtype=None, device=torch.device("cpu")))
        self.action_transformation_matrix[self.position].copy_(
            torch.as_tensor(action_transformation[0], dtype=None, device=torch.device("cpu"))
        )
        self.action_transformation_vector[self.position].copy_(
            torch.as_tensor(action_transformation[1], dtype=None, device=torch.device("cpu"))
        )
        self.dHdxx[self.position].copy_(torch.as_tensor(hamiltonian.dfdxx, dtype=None, device=torch.device("cpu")))
        self.dHdux[self.position].copy_(torch.as_tensor(hamiltonian.dfdux, dtype=None, device=torch.device("cpu")))
        self.dHduu[self.position].copy_(torch.as_tensor(hamiltonian.dfduu, dtype=None, device=torch.device("cpu")))
        self.dHdx[self.position].copy_(torch.as_tensor(hamiltonian.dfdx, dtype=None, device=torch.device("cpu")))
        self.dHdu[self.position].copy_(torch.as_tensor(hamiltonian.dfdu, dtype=None, device=torch.device("cpu")))
        self.H[self.position].copy_(torch.as_tensor(hamiltonian.f, dtype=None, device=torch.device("cpu")))
        # update size and position
        self.size = min(self.size + 1, self.capacity)
        self.position = (self.position + 1) % self.capacity

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
        indices = torch.randint(low=0, high=self.size, size=(batch_size,), device=self.device)
        t_batch = self.t[indices]
        x_batch = self.x[indices]
        u_batch = self.u[indices]
        p_batch = self.p[indices]
        observation_batch = self.observation[indices]
        action_transformation_matrix_batch = self.action_transformation_matrix[indices]
        action_transformation_vector_batch = self.action_transformation_vector[indices]
        dHdxx_batch = self.dHdxx[indices]
        dHdux_batch = self.dHdux[indices]
        dHduu_batch = self.dHduu[indices]
        dHdx_batch = self.dHdx[indices]
        dHdu_batch = self.dHdu[indices]
        H_batch = self.H[indices]
        return (
            t_batch,
            x_batch,
            u_batch,
            p_batch,
            observation_batch,
            action_transformation_matrix_batch,
            action_transformation_vector_batch,
            dHdxx_batch,
            dHdux_batch,
            dHduu_batch,
            dHdx_batch,
            dHdu_batch,
            H_batch,
        )

    def __len__(self) -> int:
        """The length of the memory.

        Return the length of the memory given by the current size.

        Returns:
            An integer describing the length of the memory.
        """
        return self.size
