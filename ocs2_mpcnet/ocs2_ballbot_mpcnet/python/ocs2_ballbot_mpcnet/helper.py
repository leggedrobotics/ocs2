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

"""Ballbot helper functions.

Provides robot-specific helper functions for ballbot.
"""

import numpy as np
from typing import Tuple

from ocs2_mpcnet_core import config
from ocs2_mpcnet_core import helper
from ocs2_mpcnet_core import SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray


def get_default_event_times_and_mode_sequence(duration: float) -> Tuple[np.ndarray, np.ndarray]:
    """Get the event times and mode sequence describing the default mode schedule.

    Creates the default event times and mode sequence for a certain time duration.

    Args:
        duration: The duration of the mode schedule given by a float.

    Returns:
        A tuple containing the components of the mode schedule.
            - event_times: The event times given by a NumPy array of shape (K-1) containing floats.
            - mode_sequence: The mode sequence given by a NumPy array of shape (K) containing integers.
    """
    event_times_template = np.array([1.0], dtype=np.float64)
    mode_sequence_template = np.array([0], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(0, duration, event_times_template, mode_sequence_template)


def get_random_initial_state(state_dimension: int, default_state: [float]) -> np.ndarray:
    """Get a random initial state.

    Samples a random initial state for the robot.

    Args:
        state_dimension: The dimension of the state given by an integer.
        default_state: The default state given by a Python array containing floats.

    Returns:
        x: A random initial state given by a NumPy array containing floats.
    """
    max_linear_velocity_x = 0.5
    max_linear_velocity_y = 0.5
    max_euler_angle_derivative_z = 45.0 / 180.0 * np.pi
    max_euler_angle_derivative_y = 45.0 / 180.0 * np.pi
    max_euler_angle_derivative_x = 45.0 / 180.0 * np.pi
    random_deviation = np.zeros(state_dimension)
    random_deviation[5] = np.random.uniform(-max_linear_velocity_x, max_linear_velocity_x)
    random_deviation[6] = np.random.uniform(-max_linear_velocity_y, max_linear_velocity_y)
    random_deviation[7] = np.random.uniform(-max_euler_angle_derivative_z, max_euler_angle_derivative_z)
    random_deviation[8] = np.random.uniform(-max_euler_angle_derivative_y, max_euler_angle_derivative_y)
    random_deviation[9] = np.random.uniform(-max_euler_angle_derivative_x, max_euler_angle_derivative_x)
    return np.array(default_state) + random_deviation


def get_random_target_state(target_state_dimension: int, default_target_state: [float]) -> np.ndarray:
    """Get a random target state.

    Samples a random target state for the robot.

    Args:
        target_state_dimension: The dimension of the target state given by an integer.
        default_target_state: The default target state given by a Python array containing floats.

    Returns:
        x: A random target state given by a NumPy array containing floats.
    """
    max_position_x = 1.0
    max_position_y = 1.0
    max_orientation_z = 45.0 / 180.0 * np.pi
    random_deviation = np.zeros(target_state_dimension)
    random_deviation[0] = np.random.uniform(-max_position_x, max_position_x)
    random_deviation[1] = np.random.uniform(-max_position_y, max_position_y)
    random_deviation[2] = np.random.uniform(-max_orientation_z, max_orientation_z)
    return np.array(default_target_state) + random_deviation


def get_tasks(
    config: config.Config, tasks_number: int, duration: float
) -> Tuple[SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray]:
    """Get tasks.

    Get a random set of task that should be executed by the data generation or policy evaluation.

    Args:
        config: An instance of the configuration class.
        tasks_number: Number of tasks given by an integer.
        duration: Duration of each task given by a float.

    Returns:
        A tuple containing the components of the task.
            - initial_observations: The initial observations given by an OCS2 system observation array.
            - mode_schedules: The desired mode schedules given by an OCS2 mode schedule array.
            - target_trajectories: The desired target trajectories given by an OCS2 target trajectories array.
    """
    initial_mode = 0
    initial_time = 0.0
    initial_observations = helper.get_system_observation_array(tasks_number)
    mode_schedules = helper.get_mode_schedule_array(tasks_number)
    target_trajectories = helper.get_target_trajectories_array(tasks_number)
    for i in range(tasks_number):
        initial_observations[i] = helper.get_system_observation(
            initial_mode,
            initial_time,
            get_random_initial_state(config.STATE_DIM, config.DEFAULT_STATE),
            np.zeros(config.INPUT_DIM),
        )
        mode_schedules[i] = helper.get_mode_schedule(*get_default_event_times_and_mode_sequence(duration))
        target_trajectories[i] = helper.get_target_trajectories(
            duration * np.ones((1, 1)),
            get_random_target_state(config.TARGET_STATE_DIM, config.DEFAULT_TARGET_STATE).reshape(
                (1, config.TARGET_STATE_DIM)
            ),
            np.zeros((1, config.TARGET_INPUT_DIM)),
        )
    return initial_observations, mode_schedules, target_trajectories
