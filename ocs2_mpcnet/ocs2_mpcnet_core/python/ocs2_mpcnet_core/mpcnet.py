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

"""MPC-Net class.

Provides a class that handles the MPC-Net training.
"""

import os
import time
import datetime
import torch
import numpy as np
from typing import Optional, Tuple
from abc import ABCMeta, abstractmethod
from torch.utils.tensorboard import SummaryWriter


from ocs2_mpcnet_core import helper
from ocs2_mpcnet_core import SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray
from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.loss import BaseLoss
from ocs2_mpcnet_core.memory import BaseMemory
from ocs2_mpcnet_core.policy import BasePolicy


class Mpcnet(metaclass=ABCMeta):
    """MPC-Net.

    Implements the main methods for the MPC-Net training.

    Takes a specific configuration, interface, memory, policy and loss function(s).
    The task formulation has to be implemented in a robot-specific class derived from this class.
    Provides the main training loop for MPC-Net.
    """

    def __init__(
        self,
        root_dir: str,
        config: Config,
        interface: object,
        memory: BaseMemory,
        policy: BasePolicy,
        experts_loss: BaseLoss,
        gating_loss: Optional[BaseLoss] = None,
    ) -> None:
        """Initializes the Mpcnet class.

        Initializes the Mpcnet class by setting fixed and variable attributes.

        Args:
            root_dir: The absolute path to the root directory.
            config: An instance of the configuration class.
            interface: An instance of the interface class.
            memory: An instance of a memory class.
            policy: An instance of a policy class.
            experts_loss: An instance of a loss class used as experts loss.
            gating_loss: An instance of a loss class used as gating loss.
        """
        # config
        self.config = config
        # interface
        self.interface = interface
        # logging
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = os.path.join(root_dir, "runs", f"{timestamp}_{config.NAME}_{config.DESCRIPTION}")
        self.writer = SummaryWriter(self.log_dir)
        # loss
        self.experts_loss = experts_loss
        self.gating_loss = gating_loss
        # memory
        self.memory = memory
        # policy
        self.policy = policy
        self.policy.to(config.DEVICE)
        self.dummy_observation = torch.randn(1, config.OBSERVATION_DIM, device=config.DEVICE, dtype=config.DTYPE)
        # optimizer
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=config.LEARNING_RATE)

    @abstractmethod
    def get_tasks(
        self, tasks_number: int, duration: float
    ) -> Tuple[SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray]:
        """Get tasks.

        Get a random set of task that should be executed by the data generation or policy evaluation.

        Args:
            tasks_number: Number of tasks given by an integer.
            duration: Duration of each task given by a float.

        Returns:
            A tuple containing the components of the task.
                - initial_observations: The initial observations given by an OCS2 system observation array.
                - mode_schedules: The desired mode schedules given by an OCS2 mode schedule array.
                - target_trajectories: The desired target trajectories given by an OCS2 target trajectories array.
        """
        pass

    def start_data_generation(self, policy: BasePolicy, alpha: float = 1.0):
        """Start data generation.

        Start the data generation rollouts to receive new data.

        Args:
            policy: The current learned policy.
            alpha: The weight of the MPC policy in the rollouts.
        """
        policy_file_path = "/tmp/data_generation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
        torch.onnx.export(model=policy, args=self.dummy_observation, f=policy_file_path)
        initial_observations, mode_schedules, target_trajectories = self.get_tasks(
            self.config.DATA_GENERATION_TASKS, self.config.DATA_GENERATION_DURATION
        )
        self.interface.startDataGeneration(
            alpha,
            policy_file_path,
            self.config.DATA_GENERATION_TIME_STEP,
            self.config.DATA_GENERATION_DATA_DECIMATION,
            self.config.DATA_GENERATION_SAMPLES,
            np.diag(np.power(np.array(self.config.DATA_GENERATION_SAMPLING_VARIANCE), 2)),
            initial_observations,
            mode_schedules,
            target_trajectories,
        )

    def start_policy_evaluation(self, policy: BasePolicy, alpha: float = 0.0):
        """Start policy evaluation.

        Start the policy evaluation rollouts to validate the current performance.

        Args:
            policy: The current learned policy.
            alpha: The weight of the MPC policy in the rollouts.
        """
        policy_file_path = "/tmp/policy_evaluation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
        torch.onnx.export(model=policy, args=self.dummy_observation, f=policy_file_path)
        initial_observations, mode_schedules, target_trajectories = self.get_tasks(
            self.config.POLICY_EVALUATION_TASKS, self.config.POLICY_EVALUATION_DURATION
        )
        self.interface.startPolicyEvaluation(
            alpha,
            policy_file_path,
            self.config.POLICY_EVALUATION_TIME_STEP,
            initial_observations,
            mode_schedules,
            target_trajectories,
        )

    def train(self) -> None:
        """Train.

        Run the main training loop of MPC-Net.
        """
        try:
            # save initial policy
            save_path = self.log_dir + "/initial_policy"
            torch.onnx.export(model=self.policy, args=self.dummy_observation, f=save_path + ".onnx")
            torch.save(obj=self.policy, f=save_path + ".pt")

            print("==============\nWaiting for first data.\n==============")
            self.start_data_generation(self.policy)
            self.start_policy_evaluation(self.policy)
            while not self.interface.isDataGenerationDone():
                time.sleep(1.0)

            print("==============\nStarting training.\n==============")
            for iteration in range(self.config.LEARNING_ITERATIONS):
                alpha = 1.0 - 1.0 * iteration / self.config.LEARNING_ITERATIONS

                # data generation
                if self.interface.isDataGenerationDone():
                    # get generated data
                    data = self.interface.getGeneratedData()
                    for i in range(len(data)):
                        # push t, x, u, p, observation, action transformation, Hamiltonian into memory
                        self.memory.push(
                            data[i].t,
                            data[i].x,
                            data[i].u,
                            helper.get_one_hot(data[i].mode, self.config.EXPERT_NUM, self.config.EXPERT_FOR_MODE),
                            data[i].observation,
                            data[i].actionTransformation,
                            data[i].hamiltonian,
                        )
                    # logging
                    self.writer.add_scalar("data/new_data_points", len(data), iteration)
                    self.writer.add_scalar("data/total_data_points", len(self.memory), iteration)
                    print("iteration", iteration, "received data points", len(data), "requesting with alpha", alpha)
                    # start new data generation
                    self.start_data_generation(self.policy, alpha)

                # policy evaluation
                if self.interface.isPolicyEvaluationDone():
                    # get computed metrics
                    metrics = self.interface.getComputedMetrics()
                    survival_time = np.mean([metrics[i].survivalTime for i in range(len(metrics))])
                    incurred_hamiltonian = np.mean([metrics[i].incurredHamiltonian for i in range(len(metrics))])
                    # logging
                    self.writer.add_scalar("metric/survival_time", survival_time, iteration)
                    self.writer.add_scalar("metric/incurred_hamiltonian", incurred_hamiltonian, iteration)
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
                    self.start_policy_evaluation(self.policy)

                # save intermediate policy
                if (iteration % int(0.1 * self.config.LEARNING_ITERATIONS) == 0) and (iteration > 0):
                    save_path = self.log_dir + "/intermediate_policy_" + str(iteration)
                    torch.onnx.export(model=self.policy, args=self.dummy_observation, f=save_path + ".onnx")
                    torch.save(obj=self.policy, f=save_path + ".pt")

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
                ) = self.memory.sample(self.config.BATCH_SIZE)

                # normal closure only evaluating the experts loss
                def normal_closure():
                    # clear the gradients
                    self.optimizer.zero_grad()
                    # prediction
                    action = self.policy(observation)[0]
                    input = helper.bmv(action_transformation_matrix, action) + action_transformation_vector
                    # compute the empirical loss
                    empirical_loss = self.experts_loss(x, x, input, u, p, p, dHdxx, dHdux, dHduu, dHdx, dHdu, H)
                    # compute the gradients
                    empirical_loss.backward()
                    # clip the gradients
                    if self.config.GRADIENT_CLIPPING:
                        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.config.GRADIENT_CLIPPING_VALUE)
                    # logging
                    self.writer.add_scalar("objective/empirical_loss", empirical_loss.item(), iteration)
                    # return empirical loss
                    return empirical_loss

                # cheating closure also adding the gating loss (only relevant for mixture of experts networks)
                def cheating_closure():
                    # clear the gradients
                    self.optimizer.zero_grad()
                    # prediction
                    action, weights = self.policy(observation)[:2]
                    input = helper.bmv(action_transformation_matrix, action) + action_transformation_vector
                    # compute the empirical loss
                    empirical_experts_loss = self.experts_loss(x, x, input, u, p, p, dHdxx, dHdux, dHduu, dHdx, dHdu, H)
                    empirical_gating_loss = self.gating_loss(x, x, u, u, weights, p, dHdxx, dHdux, dHduu, dHdx, dHdu, H)
                    empirical_loss = empirical_experts_loss + self.config.LAMBDA * empirical_gating_loss
                    # compute the gradients
                    empirical_loss.backward()
                    # clip the gradients
                    if self.config.GRADIENT_CLIPPING:
                        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.config.GRADIENT_CLIPPING_VALUE)
                    # logging
                    self.writer.add_scalar("objective/empirical_experts_loss", empirical_experts_loss.item(), iteration)
                    self.writer.add_scalar("objective/empirical_gating_loss", empirical_gating_loss.item(), iteration)
                    self.writer.add_scalar("objective/empirical_loss", empirical_loss.item(), iteration)
                    # return empirical loss
                    return empirical_loss

                # take an optimization step
                if self.config.CHEATING:
                    self.optimizer.step(cheating_closure)
                else:
                    self.optimizer.step(normal_closure)

                # let data generation and policy evaluation finish in last iteration (to avoid a segmentation fault)
                if iteration == self.config.LEARNING_ITERATIONS - 1:
                    while (not self.interface.isDataGenerationDone()) or (not self.interface.isPolicyEvaluationDone()):
                        time.sleep(1.0)

            print("==============\nTraining completed.\n==============")

            # save final policy
            save_path = self.log_dir + "/final_policy"
            torch.onnx.export(model=self.policy, args=self.dummy_observation, f=save_path + ".onnx")
            torch.save(obj=self.policy, f=save_path + ".pt")

        except KeyboardInterrupt:
            # let data generation and policy evaluation finish (to avoid a segmentation fault)
            while (not self.interface.isDataGenerationDone()) or (not self.interface.isPolicyEvaluationDone()):
                time.sleep(1.0)
            print("==============\nTraining interrupted.\n==============")
            pass

        self.writer.close()
