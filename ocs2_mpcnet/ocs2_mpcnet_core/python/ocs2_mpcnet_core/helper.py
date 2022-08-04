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

"""Helper functions.

Provides helper functions, such as convenience functions for batch-wise operations or access to OCC2 types.
"""

import torch
import numpy as np
from typing import Tuple, Dict

from ocs2_mpcnet_core import (
    size_array,
    scalar_array,
    vector_array,
    SystemObservation,
    SystemObservationArray,
    ModeSchedule,
    ModeScheduleArray,
    TargetTrajectories,
    TargetTrajectoriesArray,
)


def bdot(bv1: torch.Tensor, bv2: torch.Tensor) -> torch.Tensor:
    """Batch-wise dot product.

    Performs a batch-wise dot product between two batches of vectors with batch size B and dimension N. Supports
    broadcasting for the batch dimension.

    Args:
        bv1: A (B,N) tensor containing a batch of vectors.
        bv2: A (B,N) tensor containing a batch of vectors.

    Returns:
        A (B) tensor containing the batch-wise dot product.
    """
    return torch.sum(torch.mul(bv1, bv2), dim=1)


def bmv(bm: torch.Tensor, bv: torch.Tensor) -> torch.Tensor:
    """Batch-wise matrix-vector product.

    Performs a batch-wise matrix-vector product between a batch of MxN matrices and a batch of vectors of dimension N,
    each with batch size B. Supports broadcasting for the batch dimension.

    Args:
        bm: A (B,M,N) tensor containing a batch of matrices.
        bv: A (B,N) tensor containing a batch of vectors.

    Returns:
        A (B,M) tensor containing the batch-wise matrix-vector product.
    """
    return torch.matmul(bm, bv.unsqueeze(dim=2)).squeeze(dim=2)


def bmm(bm1: torch.Tensor, bm2: torch.Tensor) -> torch.Tensor:
    """Batch-wise matrix-matrix product.

    Performs a batch-wise matrix-matrix product between a batch of MxK matrices and a batch of KxN matrices, each with
    batch size B. Supports broadcasting for the batch dimension (unlike torch.bmm).

    Args:
        bm1: A (B,M,K) tensor containing a batch of matrices.
        bm2: A (B,K,N) tensor containing a batch of matrices.

    Returns:
        A (B,M,N) tensor containing the batch-wise matrix-matrix product.
    """
    return torch.matmul(bm1, bm2)


def get_size_array(data: np.ndarray) -> size_array:
    """Get an OCS2 size array.

    Creates an OCS2 size array and fills it with integer data from a NumPy array.

    Args:
        data: A NumPy array of shape (N) containing integers.

    Returns:
        An OCS2 size array of length N.
    """
    my_size_array = size_array()
    my_size_array.resize(len(data))
    for i in range(len(data)):
        my_size_array[i] = data[i]
    return my_size_array


def get_scalar_array(data: np.ndarray) -> scalar_array:
    """Get an OCS2 scalar array.

    Creates an OCS2 scalar array and fills it with float data from a NumPy array.

    Args:
        data: A NumPy array of shape (N) containing floats.

    Returns:
        An OCS2 scalar array of length N.
    """
    my_scalar_array = scalar_array()
    my_scalar_array.resize(len(data))
    for i in range(len(data)):
        my_scalar_array[i] = data[i]
    return my_scalar_array


def get_vector_array(data: np.ndarray) -> vector_array:
    """Get an OCS2 vector array.

    Creates an OCS2 vector array and fills it with float data from a NumPy array.

    Args:
        data: A NumPy array of shape (M,N) containing floats.

    Returns:
        An OCS2 vector array of length M with vectors of dimension N.
    """
    my_vector_array = vector_array()
    my_vector_array.resize(len(data))
    for i in range(len(data)):
        my_vector_array[i] = data[i]
    return my_vector_array


def get_system_observation(mode: int, time: float, state: np.ndarray, input: np.ndarray) -> SystemObservation:
    """Get an OCS2 system observation object.

    Creates an OCS2 system observation object and fills it with data.

    Args:
        mode: The observed mode given by an integer.
        time: The observed time given by a float.
        state: The observed state given by a NumPy array of shape (M) containing floats.
        input: The observed input given by a NumPy array of shape (N) containing floats.

    Returns:
        An OCS2 system observation object.
    """
    system_observation = SystemObservation()
    system_observation.mode = mode
    system_observation.time = time
    system_observation.state = state
    system_observation.input = input
    return system_observation


def get_system_observation_array(length: int) -> SystemObservationArray:
    """Get an OCS2 system observation array.

    Creates an OCS2 system observation array but does not fill it with data.

    Args:
        length: The length that the array should have given by an integer.

    Returns:
        An OCS2 system observation array of the desired length.
    """
    system_observation_array = SystemObservationArray()
    system_observation_array.resize(length)
    return system_observation_array


def get_target_trajectories(
    time_trajectory: np.ndarray, state_trajectory: np.ndarray, input_trajectory: np.ndarray
) -> TargetTrajectories:
    """Get an OCS2 target trajectories object.

    Creates an OCS2 target trajectories object and fills it with data.

    Args:
        time_trajectory: The target time trajectory given by a NumPy array of shape (K) containing floats.
        state_trajectory: The target state trajectory given by a NumPy array of shape (K,M) containing floats.
        input_trajectory: The target input trajectory given by a NumPy array of shape (K,N) containing floats.

    Returns:
        An OCS2 target trajectories object.
    """
    time_trajectory_array = get_scalar_array(time_trajectory)
    state_trajectory_array = get_vector_array(state_trajectory)
    input_trajectory_array = get_vector_array(input_trajectory)
    return TargetTrajectories(time_trajectory_array, state_trajectory_array, input_trajectory_array)


def get_target_trajectories_array(length: int) -> TargetTrajectoriesArray:
    """Get an OCS2 target trajectories array.

    Creates an OCS2 target trajectories array but does not fill it with data.

    Args:
        length: The length that the array should have given by an integer.

    Returns:
        An OCS2 target trajectories array of the desired length.
    """
    target_trajectories_array = TargetTrajectoriesArray()
    target_trajectories_array.resize(length)
    return target_trajectories_array


def get_mode_schedule(event_times: np.ndarray, mode_sequence: np.ndarray) -> ModeSchedule:
    """Get an OCS2 mode schedule object.

    Creates an OCS2 mode schedule object and fills it with data.

    Args:
        event_times: The event times given by a NumPy array of shape (K-1) containing floats.
        mode_sequence: The mode sequence given by a NumPy array of shape (K) containing integers.

    Returns:
        An OCS2 mode schedule object.
    """
    event_times_array = get_scalar_array(event_times)
    mode_sequence_array = get_size_array(mode_sequence)
    return ModeSchedule(event_times_array, mode_sequence_array)


def get_mode_schedule_array(length: int) -> ModeScheduleArray:
    """Get an OCS2 mode schedule array.

    Creates an OCS2 mode schedule array but does not fill it with data.

    Args:
        length: The length that the array should have given by an integer.

    Returns:
        An OCS2 mode schedule array of the desired length.
    """
    mode_schedule_array = ModeScheduleArray()
    mode_schedule_array.resize(length)
    return mode_schedule_array


def get_event_times_and_mode_sequence(
    default_mode: int, duration: float, event_times_template: np.ndarray, mode_sequence_template: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Get the event times and mode sequence describing a mode schedule.

    Creates the event times and mode sequence for a certain time duration from a template (e.g. a gait).

    Args:
        default_mode: The default mode prepended and appended to the mode schedule and given by an integer.
        duration: The duration of the mode schedule given by a float.
        event_times_template: The event times template given by a NumPy array of shape (T) containing floats.
        mode_sequence_template: The mode sequence template given by a NumPy array of shape (T) containing integers.

    Returns:
        A tuple containing the components of the mode schedule.
            - event_times: The event times given by a NumPy array of shape (K-1) containing floats.
            - mode_sequence: The mode sequence given by a NumPy array of shape (K) containing integers.
    """
    gait_cycle_duration = event_times_template[-1]
    num_gait_cycles = int(np.floor(duration / gait_cycle_duration))
    event_times = np.array([0.0], dtype=np.float64)
    mode_sequence = np.array([default_mode], dtype=np.uintp)
    for _ in range(num_gait_cycles):
        event_times = np.append(
            event_times, event_times[-1] * np.ones(len(event_times_template)) + event_times_template
        )
        mode_sequence = np.append(mode_sequence, mode_sequence_template)
    mode_sequence = np.append(mode_sequence, np.array([default_mode], dtype=np.uintp))
    return event_times, mode_sequence


def get_one_hot(mode: int, expert_number: int, expert_for_mode: Dict[int, int]) -> np.ndarray:
    """Get one hot encoding of mode.

    Get a one hot encoding of a mode represented by a discrete probability distribution, where the sample space is the
    set of P individually identified items given by the set of E individually identified experts.

    Args:
        mode: The mode of the system given by an integer.
        expert_number: The number of experts given by an integer.
        expert_for_mode: A dictionary that assigns modes to experts.

    Returns:
        p: Discrete probability distribution given by a NumPy array of shape (P) containing floats.
    """
    one_hot = np.zeros(expert_number)
    one_hot[expert_for_mode[mode]] = 1.0
    return one_hot
