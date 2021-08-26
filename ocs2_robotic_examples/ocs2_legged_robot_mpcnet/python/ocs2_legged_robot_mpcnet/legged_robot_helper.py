import numpy as np

from ocs2_legged_robot_mpcnet import size_array, scalar_array, vector_array, SystemObservation, SystemObservationArray,\
    ModeSchedule, ModeScheduleArray, TargetTrajectories, TargetTrajectoriesArray
from ocs2_legged_robot_mpcnet import legged_robot_config as config


def get_size_array(data):
    my_size_array = size_array()
    my_size_array.resize(len(data))
    for i in range(len(data)):
        my_size_array[i] = data[i]
    return my_size_array


def get_scalar_array(data):
    my_scalar_array = scalar_array()
    my_scalar_array.resize(len(data))
    for i in range(len(data)):
        my_scalar_array[i] = data[i]
    return my_scalar_array


def get_vector_array(data):
    my_vector_array = vector_array()
    my_vector_array.resize(len(data))
    for i in range(len(data)):
        my_vector_array[i] = data[i]
    return my_vector_array


def get_system_observation(time, state):
    system_observation = SystemObservation()
    system_observation.mode = 15
    system_observation.time = time
    system_observation.state = state
    system_observation.input = np.zeros(config.STATE_DIM)
    return system_observation


def get_system_observation_array(length):
    system_observation_array = SystemObservationArray()
    system_observation_array.resize(length)
    return system_observation_array


def get_target_trajectories(time_trajectory, state_trajectory):
    time_trajectory_array = get_scalar_array(time_trajectory)
    state_trajectory_array = get_vector_array(state_trajectory)
    input_trajectory_array = get_vector_array(np.zeros((len(time_trajectory), config.INPUT_DIM)))
    return TargetTrajectories(time_trajectory_array, state_trajectory_array, input_trajectory_array)


def get_target_trajectories_array(length):
    target_trajectories_array = TargetTrajectoriesArray()
    target_trajectories_array.resize(length)
    return target_trajectories_array


def get_mode_schedule(event_times, mode_sequence):
    event_times_array = get_scalar_array(event_times)
    mode_sequence_array = get_size_array(mode_sequence)
    return ModeSchedule(event_times_array, mode_sequence_array)


def get_mode_schedule_array(length):
    mode_schedule_array = ModeScheduleArray()
    mode_schedule_array.resize(length)
    return mode_schedule_array


def get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template):
    gait_cycle_duration = event_times_template[-1]
    num_gait_cycles = int(np.floor(duration / gait_cycle_duration))
    event_times = np.array([0.0], dtype=np.float64)
    mode_sequence = np.array([15], dtype=np.uintp)
    for _ in range(num_gait_cycles):
        event_times = np.append(event_times, event_times[-1] * np.ones(len(event_times_template)) + event_times_template)
        mode_sequence = np.append(mode_sequence, mode_sequence_template)
    mode_sequence = np.append(mode_sequence, np.array([15], dtype=np.uintp))
    return event_times, mode_sequence


def get_stance(duration):
    # contact schedule: STANCE
    # swing schedule: -
    event_times_template = np.array([1.0], dtype=np.float64)
    mode_sequence_template = np.array([15], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


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
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


def get_trot_2(duration):
    # contact schedule: RF_LH, LF_RH
    # swing schedule: LF_RH, RF_LH
    event_times_template = np.array([0.35, 0.7], dtype=np.float64)
    mode_sequence_template = np.array([6, 9], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


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


def get_dynamic_diagonal_walk_1(duration):
    # contact schedule: RF_LH_RH, RF_LH, LF_RF_LH, LF_LH_RH, LF_RH, LF_RF_RH
    # swing schedule: LF, LF_RH, RH, RF, RF_LH, LH
    event_times_template = np.array([0.15, 0.3, 0.45, 0.6, 0.75, 0.9], dtype=np.float64)
    mode_sequence_template = np.array([7, 6, 14, 11, 9, 13], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


def get_dynamic_diagonal_walk_2(duration):
    # contact schedule: LF_LH_RH, LF_RH LF_RF_RH, RF_LH_RH, RF_LH, LF_RF_LH
    # swing schedule: RF, RF_LH, LH, LF, LF_RH, RH
    event_times_template = np.array([0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9], dtype=np.float64)
    mode_sequence_template = np.array([11, 9, 13, 7, 6, 14], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


def get_random_initial_state_dynamic_diagonal_walk():
    max_normalized_linear_momentum_x = 0.4
    max_normalized_linear_momentum_y = 0.2
    max_normalized_linear_momentum_z = 0.2
    max_normalized_angular_momentum_x = config.normalized_inertia[0] * 52.5 * np.pi / 180.0
    max_normalized_angular_momentum_y = config.normalized_inertia[1] * 52.5 * np.pi / 180.0
    max_normalized_angular_momentum_z = config.normalized_inertia[2] * 30.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[0] = np.random.uniform(-max_normalized_linear_momentum_x / 2.0, max_normalized_linear_momentum_x)
    random_deviation[1] = np.random.uniform(-max_normalized_linear_momentum_y, max_normalized_linear_momentum_y)
    random_deviation[2] = np.random.uniform(-max_normalized_linear_momentum_z, max_normalized_linear_momentum_z / 2.0)
    random_deviation[3] = np.random.uniform(-max_normalized_angular_momentum_x, max_normalized_angular_momentum_x)
    random_deviation[4] = np.random.uniform(-max_normalized_angular_momentum_y, max_normalized_angular_momentum_y)
    random_deviation[5] = np.random.uniform(-max_normalized_angular_momentum_z, max_normalized_angular_momentum_z)
    return np.array(config.default_state) + random_deviation


def get_random_target_state_dynamic_diagonal_walk():
    max_position_x = 0.275
    max_position_y = 0.1375
    max_orientation_z = 25.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[6] = np.random.uniform(-max_position_x / 2.0, max_position_x)
    random_deviation[7] = np.random.uniform(-max_position_y, max_position_y)
    random_deviation[9] = np.random.uniform(-max_orientation_z, max_orientation_z)
    return np.array(config.default_state) + random_deviation


def get_static_walk_1(duration):
    # contact schedule: LF_RF_RH, RF_LH_RH, LF_RF_LH, LF_LH_RH
    # swing schedule: LH, LF, RH, RF
    event_times_template = np.array([0.3, 0.6, 0.9, 1.2], dtype=np.float64)
    mode_sequence_template = np.array([13, 7, 14, 11], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


def get_static_walk_2(duration):
    # contact schedule: RF_LH_RH, LF_RF_LH, LF_LH_RH, LF_RF_RH
    # swing schedule: LF, RH, RF, LH
    event_times_template = np.array([0.3, 0.6, 0.9, 1.2], dtype=np.float64)
    mode_sequence_template = np.array([7, 14, 11, 13], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


def get_static_walk_3(duration):
    # contact schedule: LF_RF_LH, LF_LH_RH, LF_RF_RH, RF_LH_RH
    # swing schedule: RH, RF, LH, LF
    event_times_template = np.array([0.3, 0.6, 0.9, 1.2], dtype=np.float64)
    mode_sequence_template = np.array([14, 11, 13, 7], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


def get_static_walk_4(duration):
    # contact schedule: LF_LH_RH, LF_RF_RH, RF_LH_RH, LF_RF_LH
    # swing schedule: RF, LH, LF, RH
    event_times_template = np.array([0.3, 0.6, 0.9, 1.2], dtype=np.float64)
    mode_sequence_template = np.array([11, 13, 7, 14], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


def get_random_initial_state_static_walk():
    max_normalized_linear_momentum_x = 0.25
    max_normalized_linear_momentum_y = 0.125
    max_normalized_linear_momentum_z = 0.125
    max_normalized_angular_momentum_x = config.normalized_inertia[0] * 45.0 * np.pi / 180.0
    max_normalized_angular_momentum_y = config.normalized_inertia[1] * 45.0 * np.pi / 180.0
    max_normalized_angular_momentum_z = config.normalized_inertia[2] * 25.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[0] = np.random.uniform(-max_normalized_linear_momentum_x / 2.0, max_normalized_linear_momentum_x)
    random_deviation[1] = np.random.uniform(-max_normalized_linear_momentum_y, max_normalized_linear_momentum_y)
    random_deviation[2] = np.random.uniform(-max_normalized_linear_momentum_z, max_normalized_linear_momentum_z / 2.0)
    random_deviation[3] = np.random.uniform(-max_normalized_angular_momentum_x, max_normalized_angular_momentum_x)
    random_deviation[4] = np.random.uniform(-max_normalized_angular_momentum_y, max_normalized_angular_momentum_y)
    random_deviation[5] = np.random.uniform(-max_normalized_angular_momentum_z, max_normalized_angular_momentum_z)
    return np.array(config.default_state) + random_deviation


def get_random_target_state_static_walk():
    max_position_x = 0.25
    max_position_y = 0.125
    max_orientation_z = 20.0 * np.pi / 180.0
    random_deviation = np.zeros(config.STATE_DIM)
    random_deviation[6] = np.random.uniform(-max_position_x / 2.0, max_position_x)
    random_deviation[7] = np.random.uniform(-max_position_y, max_position_y)
    random_deviation[9] = np.random.uniform(-max_orientation_z, max_orientation_z)
    return np.array(config.default_state) + random_deviation


def get_tasks(n_tasks, duration, choices):
    initial_observations = get_system_observation_array(n_tasks)
    mode_schedules = get_mode_schedule_array(n_tasks)
    target_trajectories = get_target_trajectories_array(n_tasks)
    for i in range(n_tasks):
        if choices[i] == "stance":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_stance())
            mode_schedules[i] = get_mode_schedule(*get_stance(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_stance().reshape((1, config.STATE_DIM)))
        elif choices[i] == "trot_1":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_trot())
            mode_schedules[i] = get_mode_schedule(*get_trot_1(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_trot().reshape((1, config.STATE_DIM)))
        elif choices[i] == "trot_2":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_trot())
            mode_schedules[i] = get_mode_schedule(*get_trot_2(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_trot().reshape((1, config.STATE_DIM)))
        elif choices[i] == "dynamic_diagonal_walk_1":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_dynamic_diagonal_walk())
            mode_schedules[i] = get_mode_schedule(*get_dynamic_diagonal_walk_1(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_dynamic_diagonal_walk().reshape((1, config.STATE_DIM)))
        elif choices[i] == "dynamic_diagonal_walk_2":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_dynamic_diagonal_walk())
            mode_schedules[i] = get_mode_schedule(*get_dynamic_diagonal_walk_2(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_dynamic_diagonal_walk().reshape((1, config.STATE_DIM)))
        elif choices[i] == "static_walk_1":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_static_walk())
            mode_schedules[i] = get_mode_schedule(*get_static_walk_1(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_static_walk().reshape((1, config.STATE_DIM)))
        elif choices[i] == "static_walk_2":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_static_walk())
            mode_schedules[i] = get_mode_schedule(*get_static_walk_2(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_static_walk().reshape((1, config.STATE_DIM)))
        elif choices[i] == "static_walk_3":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_static_walk())
            mode_schedules[i] = get_mode_schedule(*get_static_walk_3(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_static_walk().reshape((1, config.STATE_DIM)))
        elif choices[i] == "static_walk_4":
            initial_observations[i] = get_system_observation(0.0, get_random_initial_state_static_walk())
            mode_schedules[i] = get_mode_schedule(*get_static_walk_4(duration))
            target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                             get_random_target_state_static_walk().reshape((1, config.STATE_DIM)))
    return initial_observations, mode_schedules, target_trajectories


def get_one_hot(mode):
    one_hot = np.zeros(config.EXPERT_NUM)
    one_hot[config.expert_for_mode[mode]] = 1.0
    return one_hot
