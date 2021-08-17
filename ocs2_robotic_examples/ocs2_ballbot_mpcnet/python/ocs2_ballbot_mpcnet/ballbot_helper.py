import numpy as np

from ocs2_ballbot_mpcnet import size_array, scalar_array, vector_array, SystemObservation, SystemObservationArray, ModeSchedule, ModeScheduleArray, TargetTrajectories, TargetTrajectoriesArray


def get_system_observation(time, state):
    system_observation = SystemObservation()
    system_observation.mode = 0
    system_observation.time = time
    system_observation.state = state
    system_observation.input = np.zeros(3)
    return system_observation


def get_system_observation_array(times, states):
    system_observation_array = SystemObservationArray()
    system_observation_array.resize(len(times))
    for i in range(len(times)):
        system_observation_array[i] = get_system_observation(times[i], states[i])
    return system_observation_array


def get_mode_schedule():
    event_times_np = np.array([0.0], dtype=np.float64)
    mode_sequence_np = np.array([0, 0], dtype=np.uintp)
    event_times = scalar_array()
    event_times.resize(len(event_times_np))
    for i in range(len(event_times_np)):
        event_times[i] = event_times_np[i]
    mode_sequence = size_array()
    mode_sequence.resize(len(mode_sequence_np))
    for i in range(len(mode_sequence_np)):
        mode_sequence[i] = mode_sequence_np[i]
    return ModeSchedule(event_times, mode_sequence)


def get_mode_schedule_array(length):
    mode_schedule_array = ModeScheduleArray()
    mode_schedule_array.resize(length)
    for i in range(length):
        mode_schedule_array[i] = get_mode_schedule()
    return mode_schedule_array


def get_target_trajectories(time, state, input):
    # time
    time_trajectory = scalar_array()
    time_trajectory.resize(1)
    time_trajectory[0] = time
    # state
    state_trajectory = vector_array()
    state_trajectory.resize(1)
    state_trajectory[0] = state
    # input
    input_trajectory = vector_array()
    input_trajectory.resize(1)
    input_trajectory[0] = input
    return TargetTrajectories(time_trajectory, state_trajectory, input_trajectory)


def get_target_trajectories_array(times, states, inputs):
    target_trajectories_array = TargetTrajectoriesArray()
    target_trajectories_array.resize(len(times))
    for i in range(len(times)):
        target_trajectories_array[i] = get_target_trajectories(times[i], states[i], inputs[i])
    return target_trajectories_array


def get_random_initial_state():
    random_state = np.zeros(10)
    random_state[0] = np.random.uniform(-0.5, 0.5)  # base x
    random_state[1] = np.random.uniform(-0.5, 0.5)  # base y
    random_state[2] = np.random.uniform(-0.5, 0.5)  # base yaw
    random_state[3] = np.random.uniform(-0.1, 0.1)  # base pitch
    random_state[4] = np.random.uniform(-0.1, 0.1)  # base roll
    return random_state


def get_random_target_state():
    random_state = np.zeros(10)
    random_state[0] = np.random.uniform(-0.5, 0.5)  # base x
    random_state[1] = np.random.uniform(-0.5, 0.5)  # base y
    random_state[2] = np.random.uniform(-0.5, 0.5)  # base yaw
    return random_state
