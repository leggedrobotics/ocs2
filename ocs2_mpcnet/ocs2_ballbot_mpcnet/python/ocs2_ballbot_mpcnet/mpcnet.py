#!/usr/bin/env python3

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

"""Ballbot MPC-Net.

Main script for training an MPC-Net policy for ballbot.
"""

import os
import sys
import time
import datetime
import torch
import numpy as np

from torch.utils.tensorboard import SummaryWriter

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.helper import bmv, bmm
from ocs2_mpcnet_core.loss.hamiltonian import HamiltonianLoss as Loss
from ocs2_mpcnet_core.memory.circular import CircularMemory as Memory
from ocs2_mpcnet_core.policy.linear import LinearPolicy as Policy

from ocs2_ballbot_mpcnet import helper
from ocs2_ballbot_mpcnet import MpcnetInterface


def main(config_file_path: str) -> None:
    # config
    config = Config(config_file_path)

    # mpcnet interface
    mpcnet_interface = MpcnetInterface(config.DATA_GENERATION_THREADS, config.POLICY_EVALUATION_THREADS, config.RAISIM)

    # logging
    folder = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_" + config.NAME + "_" + config.DESCRIPTION
    writer = SummaryWriter("runs/" + folder)
    os.makedirs(name="policies/" + folder)

    # loss
    loss = Loss()

    # memory
    memory = Memory(config)

    # policy
    policy = Policy(config)
    policy.to(config.DEVICE)
    print("Initial policy parameters:")
    print(list(policy.named_parameters()))
    dummy_observation = torch.randn(1, config.OBSERVATION_DIM, device=config.DEVICE, dtype=config.DTYPE)
    print("Saving initial policy.")
    save_path = "policies/" + folder + "/initial_policy"
    torch.onnx.export(model=policy, args=dummy_observation, f=save_path + ".onnx")
    torch.save(obj=policy, f=save_path + ".pt")

    # optimizer
    optimizer = torch.optim.Adam(policy.parameters(), lr=config.LEARNING_RATE)

    def start_data_generation(policy, alpha=1.0):
        policy_file_path = "/tmp/data_generation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
        torch.onnx.export(model=policy, args=dummy_observation, f=policy_file_path)
        initial_observations, mode_schedules, target_trajectories = helper.get_tasks(
            config,
            config.DATA_GENERATION_TASKS,
            config.DATA_GENERATION_DURATION,
        )
        mpcnet_interface.startDataGeneration(
            alpha,
            policy_file_path,
            config.DATA_GENERATION_TIME_STEP,
            config.DATA_GENERATION_DATA_DECIMATION,
            config.DATA_GENERATION_SAMPLES,
            np.diag(np.power(np.array(config.DATA_GENERATION_SAMPLING_VARIANCE), 2)),
            initial_observations,
            mode_schedules,
            target_trajectories,
        )

    def start_policy_evaluation(policy, alpha=0.0):
        policy_file_path = "/tmp/policy_evaluation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
        torch.onnx.export(model=policy, args=dummy_observation, f=policy_file_path)
        initial_observations, mode_schedules, target_trajectories = helper.get_tasks(
            config,
            config.POLICY_EVALUATION_TASKS,
            config.POLICY_EVALUATION_DURATION,
        )
        mpcnet_interface.startPolicyEvaluation(
            alpha,
            policy_file_path,
            config.POLICY_EVALUATION_TIME_STEP,
            initial_observations,
            mode_schedules,
            target_trajectories,
        )

    try:
        print("==============\nWaiting for first data.\n==============")
        start_data_generation(policy)
        start_policy_evaluation(policy)
        while not mpcnet_interface.isDataGenerationDone():
            time.sleep(1.0)

        print("==============\nStarting training.\n==============")
        for iteration in range(config.LEARNING_ITERATIONS):
            alpha = 1.0 - 1.0 * iteration / config.LEARNING_ITERATIONS

            # data generation
            if mpcnet_interface.isDataGenerationDone():
                # get generated data
                data = mpcnet_interface.getGeneratedData()
                for i in range(len(data)):
                    # push t, x, u, p, observation, action transformation, Hamiltonian into memory
                    memory.push(
                        data[i].t,
                        data[i].x,
                        data[i].u,
                        torch.ones(1, device=config.DEVICE, dtype=config.DTYPE),
                        data[i].observation,
                        data[i].actionTransformation,
                        data[i].hamiltonian,
                    )
                # logging
                writer.add_scalar("data/new_data_points", len(data), iteration)
                writer.add_scalar("data/total_data_points", len(memory), iteration)
                print("iteration", iteration, "received data points", len(data), "requesting with alpha", alpha)
                # start new data generation
                start_data_generation(policy, alpha)

            # policy evaluation
            if mpcnet_interface.isPolicyEvaluationDone():
                # get computed metrics
                metrics = mpcnet_interface.getComputedMetrics()
                survival_time = np.mean([metrics[i].survivalTime for i in range(len(metrics))])
                incurred_hamiltonian = np.mean([metrics[i].incurredHamiltonian for i in range(len(metrics))])
                # logging
                writer.add_scalar("metric/survival_time", survival_time, iteration)
                writer.add_scalar("metric/incurred_hamiltonian", incurred_hamiltonian, iteration)
                print(
                    "iteration",
                    iteration,
                    "received metrics:",
                    "incurred_hamiltonian",
                    incurred_hamiltonian,
                    "survival_time",
                    survival_time,
                )
                # start new policy evaluation
                start_policy_evaluation(policy)

            # intermediate policies
            if (iteration % 1000 == 0) and (iteration > 0):
                print("Saving intermediate policy for iteration", iteration)
                save_path = "policies/" + folder + "/intermediate_policy_" + str(iteration)
                torch.onnx.export(model=policy, args=dummy_observation, f=save_path + ".onnx")
                torch.save(obj=policy, f=save_path + ".pt")

            # extract batch from memory
            (
                t,
                x,
                u,
                p,
                observation,
                action_transformation_matrix,
                action_transformation_vector,
                dHdxx,
                dHdux,
                dHduu,
                dHdx,
                dHdu,
                H,
            ) = memory.sample(config.BATCH_SIZE)

            # take an optimization step
            def closure():
                # clear the gradients
                optimizer.zero_grad()
                # prediction
                action = policy(observation)
                input = bmv(action_transformation_matrix, action) + action_transformation_vector
                # compute the empirical loss
                empirical_loss = loss(x, x, input, u, dHdxx, dHdux, dHduu, dHdx, dHdu, H)
                # compute the gradients
                empirical_loss.backward()
                # logging
                writer.add_scalar("objective/empirical_loss", empirical_loss.item(), iteration)
                # return empirical loss
                return empirical_loss

            optimizer.step(closure)

            # let data generation and policy evaluation finish in last iteration (to avoid a segmentation fault)
            if iteration == config.LEARNING_ITERATIONS - 1:
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
    torch.onnx.export(model=policy, args=dummy_observation, f=save_path + ".onnx")
    torch.save(obj=policy, f=save_path + ".pt")

    writer.close()

    print("Done. Exiting now.")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        main(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config/ballbot.yaml"))
