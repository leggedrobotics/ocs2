import os
import time
import datetime
import torch
import numpy as np

from torch.utils.tensorboard import SummaryWriter

from ocs2_mpcnet.loss import BehavioralCloning as Loss
from ocs2_mpcnet.memory import ReplayMemory as Memory
from ocs2_mpcnet.policy import LinearPolicy as Policy

import ballbot_config as config
from ballbot_helper import get_system_observation_array, get_mode_schedule_array, get_target_trajectories_array, get_random_initial_state, get_random_target_state

from ocs2_ballbot_mpcnet import MpcnetInterface

# settings for data generation by applying behavioral policy
data_generation_time_step = 0.1
data_generation_duration = 3.0
data_generation_data_decimation = 1
data_generation_n_threads = 2
data_generation_n_tasks = 10
data_generation_n_samples = 2
data_generation_sampling_covariance = np.zeros((10, 10), order='F')
for i in range(10):
    data_generation_sampling_covariance[i, i] = 0.01

# settings for computing metrics by applying learned policy
policy_evaluation_time_step = 0.1
policy_evaluation_duration = 3.0
policy_evaluation_n_threads = 2
policy_evaluation_n_tasks = 10

# mpcnet interface
mpcnet_interface = MpcnetInterface(data_generation_n_threads, policy_evaluation_n_threads)

# logging
description = "description"
folder = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_" + config.name + "_" + description
writer = SummaryWriter("runs/" + folder)
os.makedirs(name="policies/" + folder)

# loss
loss = Loss(torch.tensor(config.R, device=config.device, dtype=config.dtype).diag(), np.diag(config.R))

# memory
memory_capacity = 1000000
memory = Memory(memory_capacity)

# policy
policy = Policy(config.TIME_DIM, config.STATE_DIM, config.INPUT_DIM)
policy.to(config.device)
print("Initial policy parameters:")
print(list(policy.named_parameters()))
dummy_input = (torch.randn(config.TIME_DIM, device=config.device, dtype=config.dtype),
               torch.randn(config.STATE_DIM, device=config.device, dtype=config.dtype))
print("Saving initial policy.")
save_path = "policies/" + folder + "/initial_policy"
torch.onnx.export(model=policy, args=dummy_input, f=save_path + ".onnx")
torch.save(obj=policy, f=save_path + ".pt")

# optimizer
learning_rate = 1e-2
learning_iterations = 100000
optimizer = torch.optim.Adam(policy.parameters(), lr=learning_rate)
batch_size = 2 ** 5


def start_data_generation(alpha, policy):
    policy_file_path = "/tmp/data_generation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
    torch.onnx.export(model=policy, args=dummy_input, f=policy_file_path)
    initial_times = 0.0 * np.ones(data_generation_n_tasks)
    initial_states = np.zeros((data_generation_n_tasks, config.STATE_DIM))
    for i in range(data_generation_n_tasks):
        initial_states[i, :] = get_random_initial_state()
    target_times = data_generation_duration * np.ones(data_generation_n_tasks)
    target_states = np.zeros((data_generation_n_tasks, config.STATE_DIM))
    target_inputs = np.zeros((data_generation_n_tasks, config.INPUT_DIM))
    for i in range(data_generation_n_tasks):
        target_states[i, :] = get_random_target_state()
    mpcnet_interface.startDataGeneration(alpha, policy_file_path, data_generation_time_step, data_generation_data_decimation,
                                         data_generation_n_samples, data_generation_sampling_covariance,
                                         get_system_observation_array(initial_times, initial_states),
                                         get_mode_schedule_array(data_generation_n_tasks),
                                         get_target_trajectories_array(target_times, target_states, target_inputs))


def start_policy_evaluation(policy):
    policy_file_path = "/tmp/policy_evaluation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
    torch.onnx.export(model=policy, args=dummy_input, f=policy_file_path)
    initial_times = 0.0 * np.ones(policy_evaluation_n_tasks)
    initial_states = np.zeros((policy_evaluation_n_tasks, config.STATE_DIM))
    for i in range(policy_evaluation_n_tasks):
        initial_states[i, :] = get_random_initial_state()
    target_times = policy_evaluation_duration * np.ones(policy_evaluation_n_tasks)
    target_states = np.zeros((policy_evaluation_n_tasks, config.STATE_DIM))
    target_inputs = np.zeros((policy_evaluation_n_tasks, config.INPUT_DIM))
    for i in range(policy_evaluation_n_tasks):
        target_states[i, :] = get_random_target_state()
    mpcnet_interface.startPolicyEvaluation(policy_file_path, policy_evaluation_time_step,
                                          get_system_observation_array(initial_times, initial_states),
                                          get_mode_schedule_array(policy_evaluation_n_tasks),
                                          get_target_trajectories_array(target_times, target_states, target_inputs))


try:
    print("==============\nWaiting for first data.\n==============")
    start_data_generation(alpha=1.0, policy=policy)
    start_policy_evaluation(policy=policy)
    while not mpcnet_interface.isDataGenerationDone():
        time.sleep(1.0)

    print("==============\nStarting training.\n==============")
    for iteration in range(learning_iterations):
        alpha = 1.0 - 1.0 * iteration / learning_iterations

        # data generation
        if mpcnet_interface.isDataGenerationDone():
            # get generated data
            data = mpcnet_interface.getGeneratedData()
            for i in range(len(data)):
                # push t, x, u, generalized time, relative state, Hamiltonian into memeory
                memory.push(data[i].t, data[i].x, data[i].u, data[i].generalized_time, data[i].relative_state, data[i].hamiltonian)
            # logging
            writer.add_scalar('data/new_data_points', len(data), iteration)
            writer.add_scalar('data/total_data_points', memory.size, iteration)
            print("iteration", iteration, "received data points", len(data), "requesting with alpha", alpha)
            # start new data generation
            start_data_generation(alpha=alpha, policy=policy)

        # policy evaluation
        if mpcnet_interface.isPolicyEvaluationDone():
            # get computed metrics
            metrics = mpcnet_interface.getComputedMetrics()
            survival_time = np.mean([metrics[i].survival_time for i in range(len(metrics))])
            incurred_hamiltonian = np.mean([metrics[i].incurred_hamiltonian for i in range(len(metrics))])
            # logging
            writer.add_scalar('metric/survival_time', survival_time, iteration)
            writer.add_scalar('metric/incurred_hamiltonian', incurred_hamiltonian, iteration)
            print("iteration", iteration, "received metrics:", "incurred_hamiltonian", incurred_hamiltonian, "survival_time", survival_time)
            # start new policy evaluation
            start_policy_evaluation(policy=policy)

        # intermediate policies
        if (iteration % 1000 == 0) and (iteration > 0):
            print("Saving intermediate policy for iteration", iteration)
            save_path = "policies/" + folder + "/intermediate_policy_" + str(iteration)
            torch.onnx.export(model=policy, args=dummy_input, f=save_path + ".onnx")
            torch.save(obj=policy, f=save_path + ".pt")

        # extract batch of samples from replay memory
        samples = memory.sample(batch_size)

        # take an optimization step
        def closure():
            # clear the gradients
            optimizer.zero_grad()
            # compute the empirical loss
            empiricial_loss = torch.zeros(1, dtype=config.dtype, device=config.device)
            for sample in samples:
                # torch
                t = torch.tensor([sample.t], dtype=config.dtype, device=config.device)
                x = torch.tensor(sample.x, dtype=config.dtype, device=config.device)
                u_target = torch.tensor(sample.u, dtype=config.dtype, device=config.device)
                generalized_time = torch.tensor(sample.generalized_time, dtype=config.dtype, device=config.device)
                relative_state = torch.tensor(sample.relative_state, dtype=config.dtype, device=config.device)
                p, U = policy(generalized_time, relative_state)
                u_predicted = torch.matmul(p, U)
                # empirical loss
                empiricial_loss = empiricial_loss + loss.compute_torch(u_predicted, u_target)
            # compute the gradients
            empiricial_loss.backward()
            # log metrics
            writer.add_scalar('objective/empirical_loss', empiricial_loss.item() / batch_size, iteration)
            # return empiricial loss
            return empiricial_loss
        optimizer.step(closure)

        # let data generation and policy evaluation finish in last iteration (to avoid a segmentation fault)
        if iteration == learning_iterations - 1:
            while (not mpcnet_interface.isDataGenerationDone()) or (not mpcnet_interface.isPolicyEvaluationDone()):
                time.sleep(1.0)

    print("==============\nTraining completed.\n==============")

except KeyboardInterrupt:
    # let data generation and policy evaluation finish (to avoid a segmentation fault)
    while (not mpcnet_interface.isDataGenerationDone()) or (not mpcnet_interface.isPolicyEvaluationDone()):
        time.sleep(1.0)
    print("==============\nTraining interrupted.\n==============")
    pass

print("Final policy parameters:")
print(list(policy.named_parameters()))

print("Saving final policy.")
save_path = "policies/" + folder + "/final_policy"
torch.onnx.export(model=policy, args=dummy_input, f=save_path + ".onnx")
torch.save(obj=policy, f=save_path + ".pt")

writer.close()

print("Done. Exiting now.")
