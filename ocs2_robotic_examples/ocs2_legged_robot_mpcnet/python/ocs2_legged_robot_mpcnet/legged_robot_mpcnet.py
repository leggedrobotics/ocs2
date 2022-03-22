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

import os
import time
import datetime
import random
import torch
import numpy as np

from torch.utils.tensorboard import SummaryWriter

from ocs2_mpcnet.helper import bmv, bmm
from ocs2_mpcnet.loss import Hamiltonian as ExpertsLoss
from ocs2_mpcnet.loss import CrossEntropy as GatingLoss
from ocs2_mpcnet.memory import CircularMemory as Memory

from ocs2_legged_robot_mpcnet.legged_robot_policy import LeggedRobotMixtureOfNonlinearExpertsPolicy as Policy
from ocs2_legged_robot_mpcnet import legged_robot_config as config
from ocs2_legged_robot_mpcnet import legged_robot_helper as helper
from ocs2_legged_robot_mpcnet import MpcnetInterface

# settings for data generation by applying behavioral policy
data_generation_time_step = 0.0025
data_generation_duration = 4.0
data_generation_data_decimation = 4
data_generation_n_threads = 12
data_generation_n_tasks = 12
data_generation_n_samples = 2
data_generation_sampling_covariance = np.zeros((config.STATE_DIM, config.STATE_DIM), order='F')
for i in range(0, 3):
    data_generation_sampling_covariance[i, i] = 0.05 ** 2  # normalized linear momentum
for i in range(3, 6):
    data_generation_sampling_covariance[i, i] = (config.normalized_inertia[i - 3] * 2.5 * np.pi / 180.0) ** 2  # normalized angular momentum
for i in range(6, 9):
    data_generation_sampling_covariance[i, i] = 0.01 ** 2  # position
for i in range(9, 12):
    data_generation_sampling_covariance[i, i] = (0.5 * np.pi / 180.0) ** 2  # orientation
for i in range(12, 24):
    data_generation_sampling_covariance[i, i] = (0.5 * np.pi / 180.0) ** 2  # joint positions

# settings for computing metrics by applying learned policy
policy_evaluation_time_step = 0.0025
policy_evaluation_duration = 4.0
policy_evaluation_n_threads = 3
policy_evaluation_n_tasks = 3

# rollout settings for data generation and policy evaluation
raisim = True

# mpcnet interface
mpcnet_interface = MpcnetInterface(data_generation_n_threads, policy_evaluation_n_threads, raisim)

# logging
description = "description"
folder = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_" + config.name + "_" + description
writer = SummaryWriter("runs/" + folder)
os.makedirs(name="policies/" + folder)

# loss
epsilon = 1e-8  # epsilon to improve numerical stability of logs and denominators
my_lambda = 10.0  # parameter to control the relative importance of both loss types
experts_loss = ExpertsLoss()
gating_loss = GatingLoss(epsilon)

# memory
memory_capacity = 500000
memory = Memory(memory_capacity, config.TIME_DIM, config.STATE_DIM, config.INPUT_DIM, config.EXPERT_NUM)

# policy
policy = Policy(config.TIME_DIM, config.STATE_DIM, config.INPUT_DIM, config.EXPERT_NUM)
policy.to(config.device)
print("Initial policy parameters:")
print(list(policy.named_parameters()))
dummy_input = (torch.randn(1, config.TIME_DIM, device=config.device, dtype=config.dtype),
               torch.randn(1, config.STATE_DIM, device=config.device, dtype=config.dtype))
print("Saving initial policy.")
save_path = "policies/" + folder + "/initial_policy"
torch.onnx.export(model=policy, args=dummy_input, f=save_path + ".onnx")
torch.save(obj=policy, f=save_path + ".pt")

# optimizer
batch_size = 2 ** 7
learning_iterations = 100000
learning_rate_default = 1e-3
learning_rate_gating_net = learning_rate_default
learning_rate_expert_nets = learning_rate_default
optimizer = torch.optim.Adam([{'params': policy.gating_net.parameters(), 'lr': learning_rate_gating_net},
                              {'params': policy.expert_nets.parameters(), 'lr': learning_rate_expert_nets}],
                             lr=learning_rate_default)

# weights for ["stance", "trot_1", "trot_2"]
weights = [1, 2, 2]


def start_data_generation(policy, alpha=1.0):
    policy_file_path = "/tmp/data_generation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
    torch.onnx.export(model=policy, args=dummy_input, f=policy_file_path)
    choices = random.choices(["stance", "trot_1", "trot_2"], k=data_generation_n_tasks, weights=weights)
    initial_observations, mode_schedules, target_trajectories = helper.get_tasks(data_generation_n_tasks, data_generation_duration, choices)
    mpcnet_interface.startDataGeneration(alpha, policy_file_path, data_generation_time_step, data_generation_data_decimation,
                                         data_generation_n_samples, data_generation_sampling_covariance,
                                         initial_observations, mode_schedules, target_trajectories)


def start_policy_evaluation(policy, alpha=0.0):
    policy_file_path = "/tmp/policy_evaluation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
    torch.onnx.export(model=policy, args=dummy_input, f=policy_file_path)
    choices = random.choices(["stance", "trot_1", "trot_2"], k=policy_evaluation_n_tasks, weights=weights)
    initial_observations, mode_schedules, target_trajectories = helper.get_tasks(policy_evaluation_n_tasks, policy_evaluation_duration, choices)
    mpcnet_interface.startPolicyEvaluation(alpha, policy_file_path, policy_evaluation_time_step,
                                           initial_observations, mode_schedules, target_trajectories)


try:
    print("==============\nWaiting for first data.\n==============")
    start_data_generation(policy)
    start_policy_evaluation(policy)
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
                # push t, x, u, p, generalized time, relative state, input_transformation, Hamiltonian into memory
                memory.push(data[i].t, data[i].x, data[i].u, helper.get_one_hot(data[i].mode), data[i].generalized_time,
                            data[i].relative_state, data[i].input_transformation, data[i].hamiltonian)
            # logging
            writer.add_scalar('data/new_data_points', len(data), iteration)
            writer.add_scalar('data/total_data_points', len(memory), iteration)
            print("iteration", iteration, "received data points", len(data), "requesting with alpha", alpha)
            # start new data generation
            start_data_generation(policy, alpha)

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
            start_policy_evaluation(policy)

        # intermediate policies
        if (iteration % 10000 == 0) and (iteration > 0):
            print("Saving intermediate policy for iteration", iteration)
            save_path = "policies/" + folder + "/intermediate_policy_" + str(iteration)
            torch.onnx.export(model=policy, args=dummy_input, f=save_path + ".onnx")
            torch.save(obj=policy, f=save_path + ".pt")

        # extract batch from memory
        t, x, u, p, generalized_time, relative_state, input_transformation, dHdxx, dHdux, dHduu, dHdx, dHdu, H = memory.sample(batch_size)

        # take an optimization step
        def closure():
            # clear the gradients
            optimizer.zero_grad()
            # prediction
            u_predicted, p_predicted = policy(generalized_time, relative_state)
            u_predicted = bmv(input_transformation, u_predicted)
            # compute the empirical loss
            empirical_experts_loss = experts_loss.compute_batch(x, x, u_predicted, u, dHdxx, dHdux, dHduu, dHdx, dHdu, H).sum() / batch_size
            empirical_gating_loss = gating_loss.compute_batch(p, p_predicted).sum() / batch_size
            empirical_loss = empirical_experts_loss + my_lambda * empirical_gating_loss
            # compute the gradients
            empirical_loss.backward()
            # logging
            writer.add_scalar('objective/empirical_experts_loss', empirical_experts_loss.item(), iteration)
            writer.add_scalar('objective/empirical_gating_loss', empirical_gating_loss.item(), iteration)
            writer.add_scalar('objective/empirical_loss', empirical_loss.item(), iteration)
            # return empirical loss
            return empirical_loss
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
