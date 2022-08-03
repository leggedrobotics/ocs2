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

"""Legged robot helper functions.

Provides robot-specific helper functions for legged robot.
"""

import numpy as np
from typing import Tuple, List

from ocs2_mpcnet_core import helper
from ocs2_mpcnet_core import SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray

from ocs2_legged_robot_mpcnet import config


def get_stance(duration: float) -> Tuple[np.ndarray, np.ndarray]:
    """Get the stance gait.

    Creates the stance event times and mode sequence for a certain time duration:
        - contact schedule: STANCE
        - swing schedule: -

    Args:
        duration: The duration of the mode schedule given by a float.

    Returns:
        A tuple containing the components of the mode schedule.
            - event_times: The event times given by a NumPy array of shape (K-1) containing floats.
            - mode_sequence: The mode sequence given by a NumPy array of shape (K) containing integers.
    """
    event_times_template = np.array([1.0], dtype=np.float64)
    mode_sequence_template = np.array([15], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(15, duration, event_times_template, mode_sequence_template)


def get_random_initial_state_stance() -> np.ndarray:
    """Get a random initial state for stance.

    Samples a random initial state for the robot in the stance gait.

    Returns:
        x: A random initial state given by a NumPy array of shape (X) containing floats.
    """
    max_normalized_linear_momentum_x = 0.1
    max_normalized_linear_momentum_y = 0.1
    max_normalized_linear_momentum_z = 0.1
    max_normalized_angular_momentum_x = config.NORMALIZED_INERTIA[0] * 30.0 * np.pi / 180.0
    max_normalized_angular_momentum_y = config.NORMALIZED_INERTIA[1] * 30.0 * np.pi / 180.0
    max_normalized_angular_momentum_z = config.NORMALIZED_INERTIA[2] * 30.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[0] = np.random.uniform(-max_normalized_linear_momentum_x, max_normalized_linear_momentum_x)
    random_deviation[1] = np.random.uniform(-max_normalized_linear_momentum_y, max_normalized_linear_momentum_y)
    random_deviation[2] = np.random.uniform(-max_normalized_linear_momentum_z, max_normalized_linear_momentum_z / 2.0)
    random_deviation[3] = np.random.uniform(-max_normalized_angular_momentum_x, max_normalized_angular_momentum_x)
    random_deviation[4] = np.random.uniform(-max_normalized_angular_momentum_y, max_normalized_angular_momentum_y)
    random_deviation[5] = np.random.uniform(-max_normalized_angular_momentum_z, max_normalized_angular_momentum_z)
    return np.array(config.DEFAULT_STATE) + random_deviation


def get_random_target_state_stance() -> np.ndarray:
    """Get a random target state for stance.

    Samples a random target state for the robot in the stance gait.

    Returns:
        x: A random target state given by a NumPy array of shape (X) containing floats.
    """
    max_position_z = 0.075
    max_orientation_z = 25.0 * np.pi / 180.0
    max_orientation_y = 15.0 * np.pi / 180.0
    max_orientation_x = 25.0 * np.pi / 180.0
    random_deviation = np.zeros(config.TARGET_STATE_DIM)
    random_deviation[8] = np.random.uniform(-max_position_z, max_position_z)
    random_deviation[9] = np.random.uniform(-max_orientation_z, max_orientation_z)
    random_deviation[10] = np.random.uniform(-max_orientation_y, max_orientation_y)
    random_deviation[11] = np.random.uniform(-max_orientation_x, max_orientation_x)
    return np.array(config.DEFAULT_STATE) + random_deviation


def get_trot_1(duration: float) -> Tuple[np.ndarray, np.ndarray]:
    """Get the first trot gait.

    Creates the first trot event times and mode sequence for a certain time duration:
        - contact schedule: LF_RH, RF_LH
        - swing schedule: RF_LH, LF_RH

    Args:
        duration: The duration of the mode schedule given by a float.

    Returns:
        A tuple containing the components of the mode schedule.
            - event_times: The event times given by a NumPy array of shape (K-1) containing floats.
            - mode_sequence: The mode sequence given by a NumPy array of shape (K) containing integers.
    """
    event_times_template = np.array([0.35, 0.7], dtype=np.float64)
    mode_sequence_template = np.array([9, 6], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(15, duration, event_times_template, mode_sequence_template)


def get_trot_2(duration: float) -> Tuple[np.ndarray, np.ndarray]:
    """Get the second trot gait.

    Creates the second trot event times and mode sequence for a certain time duration:
        - contact schedule: RF_LH, LF_RH
        - swing schedule: LF_RH, RF_LH

    Args:
        duration: The duration of the mode schedule given by a float.

    Returns:
        A tuple containing the components of the mode schedule.
            - event_times: The event times given by a NumPy array of shape (K-1) containing floats.
            - mode_sequence: The mode sequence given by a NumPy array of shape (K) containing integers.
    """
    event_times_template = np.array([0.35, 0.7], dtype=np.float64)
    mode_sequence_template = np.array([6, 9], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(15, duration, event_times_template, mode_sequence_template)


def get_random_initial_state_trot() -> np.ndarray:
    """Get a random initial state for trot.

    Samples a random initial state for the robot in a trot gait.

    Returns:
        x: A random initial state given by a NumPy array of shape (X) containing floats.
    """
    max_normalized_linear_momentum_x = 0.5
    max_normalized_linear_momentum_y = 0.25
    max_normalized_linear_momentum_z = 0.25
    max_normalized_angular_momentum_x = config.NORMALIZED_INERTIA[0] * 60.0 * np.pi / 180.0
    max_normalized_angular_momentum_y = config.NORMALIZED_INERTIA[1] * 60.0 * np.pi / 180.0
    max_normalized_angular_momentum_z = config.NORMALIZED_INERTIA[2] * 35.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[0] = np.random.uniform(-max_normalized_linear_momentum_x, max_normalized_linear_momentum_x)
    random_deviation[1] = np.random.uniform(-max_normalized_linear_momentum_y, max_normalized_linear_momentum_y)
    random_deviation[2] = np.random.uniform(-max_normalized_linear_momentum_z, max_normalized_linear_momentum_z / 2.0)
    random_deviation[3] = np.random.uniform(-max_normalized_angular_momentum_x, max_normalized_angular_momentum_x)
    random_deviation[4] = np.random.uniform(-max_normalized_angular_momentum_y, max_normalized_angular_momentum_y)
    random_deviation[5] = np.random.uniform(-max_normalized_angular_momentum_z, max_normalized_angular_momentum_z)
    return np.array(config.DEFAULT_STATE) + random_deviation


def get_random_target_state_trot() -> np.ndarray:
    """Get a random target state for trot.

    Samples a random target state for the robot in a trot gait.

    Returns:
        x: A random target state given by a NumPy array of shape (X) containing floats.
    """
    max_position_x = 0.3
    max_position_y = 0.15
    max_orientation_z = 30.0 * np.pi / 180.0
    random_deviation = np.zeros(config.TARGET_STATE_DIM)
    random_deviation[6] = np.random.uniform(-max_position_x, max_position_x)
    random_deviation[7] = np.random.uniform(-max_position_y, max_position_y)
    random_deviation[9] = np.random.uniform(-max_orientation_z, max_orientation_z)
    return np.array(config.DEFAULT_STATE) + random_deviation


def get_tasks(
    n_tasks: int, duration: float, choices: List[str]
) -> Tuple[SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray]:
    """Get tasks.

    Get a random set of task that should be executed by the data generation or policy evaluation.

    Args:
        n_tasks: Number of tasks given by an integer.
        duration: Duration of each task given by a float.
        choices: Different gaits for the tasks given by a list containing strings with the gait names.

    Returns:
        A tuple containing the components of the task.
            - initial_observations: The initial observations given by an OCS2 system observation array.
            - mode_schedules: The desired mode schedules given by an OCS2 mode schedule array.
            - target_trajectories: The desired target trajectories given by an OCS2 target trajectories array.
    """
    initial_observations = helper.get_system_observation_array(n_tasks)
    mode_schedules = helper.get_mode_schedule_array(n_tasks)
    target_trajectories = helper.get_target_trajectories_array(n_tasks)
    for i in range(n_tasks):
        if choices[i] == "stance":
            initial_observations[i] = helper.get_system_observation(
                15, 0.0, get_random_initial_state_stance(), np.zeros(config.INPUT_DIM)
            )
            mode_schedules[i] = helper.get_mode_schedule(*get_stance(duration))
            target_trajectories[i] = helper.get_target_trajectories(
                duration * np.ones((1, 1)),
                get_random_target_state_stance().reshape((1, config.TARGET_STATE_DIM)),
                np.zeros((1, config.TARGET_INPUT_DIM)),
            )
        elif choices[i] == "trot_1":
            initial_observations[i] = helper.get_system_observation(
                15, 0.0, get_random_initial_state_trot(), np.zeros(config.INPUT_DIM)
            )
            mode_schedules[i] = helper.get_mode_schedule(*get_trot_1(duration))
            target_trajectories[i] = helper.get_target_trajectories(
                duration * np.ones((1, 1)),
                get_random_target_state_trot().reshape((1, config.TARGET_STATE_DIM)),
                np.zeros((1, config.TARGET_INPUT_DIM)),
            )
        elif choices[i] == "trot_2":
            initial_observations[i] = helper.get_system_observation(
                15, 0.0, get_random_initial_state_trot(), np.zeros(config.INPUT_DIM)
            )
            mode_schedules[i] = helper.get_mode_schedule(*get_trot_2(duration))
            target_trajectories[i] = helper.get_target_trajectories(
                duration * np.ones((1, 1)),
                get_random_target_state_trot().reshape((1, config.TARGET_STATE_DIM)),
                np.zeros((1, config.TARGET_INPUT_DIM)),
            )
    return initial_observations, mode_schedules, target_trajectories


def get_one_hot(mode: int) -> np.ndarray:
    """Get one hot encoding of mode.

    Get a one hot encoding of a mode represented by a discrete probability distribution, where the sample space is the
    set of P individually identified items given by the set of E individually identified experts.

    Args:
        mode: The mode of the system given by an integer.

    Returns:
        p: Discrete probability distribution given by a NumPy array of shape (P) containing floats.
    """
    one_hot = np.zeros(config.EXPERT_NUM)
    one_hot[config.EXPERT_FOR_MODE[mode]] = 1.0
    return one_hot
