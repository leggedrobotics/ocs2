import numpy as np

from ocs2_ballbot_mpcnet import size_array, scalar_array, vector_array, SystemObservation, SystemObservationArray,\
    ModeSchedule, ModeScheduleArray, TargetTrajectories, TargetTrajectoriesArray
from ocs2_ballbot_mpcnet import ballbot_config as config


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
    system_observation.mode = 0
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
    mode_sequence = np.array([0], dtype=np.uintp)
    for _ in range(num_gait_cycles):
        event_times = np.append(event_times, event_times[-1] * np.ones(len(event_times_template)) + event_times_template)
        mode_sequence = np.append(mode_sequence, mode_sequence_template)
    mode_sequence = np.append(mode_sequence, np.array([0], dtype=np.uintp))
    return event_times, mode_sequence


def get_default_mode_schedule(duration):
    # contact schedule: -
    # swing schedule: -
    event_times_template = np.array([1.0], dtype=np.float64)
    mode_sequence_template = np.array([0], dtype=np.uintp)
    return get_event_times_and_mode_sequence(duration, event_times_template, mode_sequence_template)


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
    initial_observations = get_system_observation_array(n_tasks)
    mode_schedules = get_mode_schedule_array(n_tasks)
    target_trajectories = get_target_trajectories_array(n_tasks)
    for i in range(n_tasks):
        initial_observations[i] = get_system_observation(0.0, get_random_initial_state())
        mode_schedules[i] = get_mode_schedule(*get_default_mode_schedule(duration))
        target_trajectories[i] = get_target_trajectories(duration * np.ones((1, 1)),
                                                         get_random_target_state().reshape((1, config.STATE_DIM)))
    return initial_observations, mode_schedules, target_trajectories
