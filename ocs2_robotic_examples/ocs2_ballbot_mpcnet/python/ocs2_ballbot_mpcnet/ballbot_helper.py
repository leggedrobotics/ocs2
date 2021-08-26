import numpy as np

from ocs2_mpcnet import helper
from ocs2_ballbot_mpcnet import ballbot_config as config


def get_default_event_times_and_mode_sequence(duration):
    # contact schedule: -
    # swing schedule: -
    event_times_template = np.array([1.0], dtype=np.float64)
    mode_sequence_template = np.array([0], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(0, duration, event_times_template, mode_sequence_template)


def get_random_initial_state():
    random_state = np.zeros(config.STATE_DIM)
    random_state[0] = np.random.uniform(-0.5, 0.5)  # base x
    random_state[1] = np.random.uniform(-0.5, 0.5)  # base y
    random_state[2] = np.random.uniform(-0.5, 0.5)  # base yaw
    random_state[3] = np.random.uniform(-0.1, 0.1)  # base pitch
    random_state[4] = np.random.uniform(-0.1, 0.1)  # base roll
    return random_state


def get_random_target_state():
    random_state = np.zeros(config.STATE_DIM)
    random_state[0] = np.random.uniform(-0.5, 0.5)  # base x
    random_state[1] = np.random.uniform(-0.5, 0.5)  # base y
    random_state[2] = np.random.uniform(-0.5, 0.5)  # base yaw
    return random_state


def get_tasks(n_tasks, duration):
    initial_observations = helper.get_system_observation_array(n_tasks)
    mode_schedules = helper.get_mode_schedule_array(n_tasks)
    target_trajectories = helper.get_target_trajectories_array(n_tasks)
    for i in range(n_tasks):
        initial_observations[i] = helper.get_system_observation(0, 0.0, get_random_initial_state(), np.zeros(config.INPUT_DIM))
        mode_schedules[i] = helper.get_mode_schedule(*get_default_event_times_and_mode_sequence(duration))
        target_trajectories[i] = helper.get_target_trajectories(duration * np.ones((1, 1)),
                                                                get_random_target_state().reshape((1, config.STATE_DIM)),
                                                                np.zeros((1, config.INPUT_DIM)))
    return initial_observations, mode_schedules, target_trajectories
