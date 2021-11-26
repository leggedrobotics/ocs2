import numpy as np

from ocs2_mpcnet import helper
from ocs2_legged_robot_mpcnet import legged_robot_config as config


def get_stance(duration):
    # contact schedule: STANCE
    # swing schedule: -
    event_times_template = np.array([1.0], dtype=np.float64)
    mode_sequence_template = np.array([15], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(15, duration, event_times_template, mode_sequence_template)


def get_random_initial_state_stance():
    max_normalized_linear_momentum_x = 0.1
    max_normalized_linear_momentum_y = 0.1
    max_normalized_linear_momentum_z = 0.1
    max_normalized_angular_momentum_x = config.normalized_inertia[0] * 30.0 * np.pi / 180.0
    max_normalized_angular_momentum_y = config.normalized_inertia[1] * 30.0 * np.pi / 180.0
    max_normalized_angular_momentum_z = config.normalized_inertia[2] * 30.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[0] = np.random.uniform(-max_normalized_linear_momentum_x, max_normalized_linear_momentum_x)
    random_deviation[1] = np.random.uniform(-max_normalized_linear_momentum_y, max_normalized_linear_momentum_y)
    random_deviation[2] = np.random.uniform(-max_normalized_linear_momentum_z, max_normalized_linear_momentum_z / 2.0)
    random_deviation[3] = np.random.uniform(-max_normalized_angular_momentum_x, max_normalized_angular_momentum_x)
    random_deviation[4] = np.random.uniform(-max_normalized_angular_momentum_y, max_normalized_angular_momentum_y)
    random_deviation[5] = np.random.uniform(-max_normalized_angular_momentum_z, max_normalized_angular_momentum_z)
    return np.array(config.default_state) + random_deviation


def get_random_target_state_stance():
    max_position_z = 0.075
    max_orientation_z = 25.0 * np.pi / 180.0
    max_orientation_y = 15.0 * np.pi / 180.0
    max_orientation_x = 25.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[8] = np.random.uniform(-max_position_z, max_position_z)
    random_deviation[9] = np.random.uniform(-max_orientation_z, max_orientation_z)
    random_deviation[10] = np.random.uniform(-max_orientation_y, max_orientation_y)
    random_deviation[11] = np.random.uniform(-max_orientation_x, max_orientation_x)
    return np.array(config.default_state) + random_deviation


def get_trot_1(duration):
    # contact schedule: LF_RH, RF_LH
    # swing schedule: RF_LH, LF_RH
    event_times_template = np.array([0.35, 0.7], dtype=np.float64)
    mode_sequence_template = np.array([9, 6], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(15, duration, event_times_template, mode_sequence_template)


def get_trot_2(duration):
    # contact schedule: RF_LH, LF_RH
    # swing schedule: LF_RH, RF_LH
    event_times_template = np.array([0.35, 0.7], dtype=np.float64)
    mode_sequence_template = np.array([6, 9], dtype=np.uintp)
    return helper.get_event_times_and_mode_sequence(15, duration, event_times_template, mode_sequence_template)


def get_random_initial_state_trot():
    max_normalized_linear_momentum_x = 0.5
    max_normalized_linear_momentum_y = 0.25
    max_normalized_linear_momentum_z = 0.25
    max_normalized_angular_momentum_x = config.normalized_inertia[0] * 60.0 * np.pi / 180.0
    max_normalized_angular_momentum_y = config.normalized_inertia[1] * 60.0 * np.pi / 180.0
    max_normalized_angular_momentum_z = config.normalized_inertia[2] * 35.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[0] = np.random.uniform(-max_normalized_linear_momentum_x, max_normalized_linear_momentum_x)
    random_deviation[1] = np.random.uniform(-max_normalized_linear_momentum_y, max_normalized_linear_momentum_y)
    random_deviation[2] = np.random.uniform(-max_normalized_linear_momentum_z, max_normalized_linear_momentum_z / 2.0)
    random_deviation[3] = np.random.uniform(-max_normalized_angular_momentum_x, max_normalized_angular_momentum_x)
    random_deviation[4] = np.random.uniform(-max_normalized_angular_momentum_y, max_normalized_angular_momentum_y)
    random_deviation[5] = np.random.uniform(-max_normalized_angular_momentum_z, max_normalized_angular_momentum_z)
    return np.array(config.default_state) + random_deviation


def get_random_target_state_trot():
    max_position_x = 0.3
    max_position_y = 0.15
    max_orientation_z = 30.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[6] = np.random.uniform(-max_position_x, max_position_x)
    random_deviation[7] = np.random.uniform(-max_position_y, max_position_y)
    random_deviation[9] = np.random.uniform(-max_orientation_z, max_orientation_z)
    return np.array(config.default_state) + random_deviation


def get_tasks(n_tasks, duration, choices):
    initial_observations = helper.get_system_observation_array(n_tasks)
    mode_schedules = helper.get_mode_schedule_array(n_tasks)
    target_trajectories = helper.get_target_trajectories_array(n_tasks)
    for i in range(n_tasks):
        if choices[i] == "stance":
            initial_observations[i] = helper.get_system_observation(15, 0.0, get_random_initial_state_stance(), np.zeros(config.INPUT_DIM))
            mode_schedules[i] = helper.get_mode_schedule(*get_stance(duration))
            target_trajectories[i] = helper.get_target_trajectories(duration * np.ones((1, 1)),
                                                                    get_random_target_state_stance().reshape((1, config.STATE_DIM)),
                                                                    np.zeros((1, config.INPUT_DIM)))
        elif choices[i] == "trot_1":
            initial_observations[i] = helper.get_system_observation(15, 0.0, get_random_initial_state_trot(), np.zeros(config.INPUT_DIM))
            mode_schedules[i] = helper.get_mode_schedule(*get_trot_1(duration))
            target_trajectories[i] = helper.get_target_trajectories(duration * np.ones((1, 1)),
                                                                    get_random_target_state_trot().reshape((1, config.STATE_DIM)),
                                                                    np.zeros((1, config.INPUT_DIM)))
        elif choices[i] == "trot_2":
            initial_observations[i] = helper.get_system_observation(15, 0.0, get_random_initial_state_trot(), np.zeros(config.INPUT_DIM))
            mode_schedules[i] = helper.get_mode_schedule(*get_trot_2(duration))
            target_trajectories[i] = helper.get_target_trajectories(duration * np.ones((1, 1)),
                                                                    get_random_target_state_trot().reshape((1, config.STATE_DIM)),
                                                                    np.zeros((1, config.INPUT_DIM)))
    return initial_observations, mode_schedules, target_trajectories


def get_one_hot(mode):
    one_hot = np.zeros(config.EXPERT_NUM)
    one_hot[config.expert_for_mode[mode]] = 1.0
    return one_hot
