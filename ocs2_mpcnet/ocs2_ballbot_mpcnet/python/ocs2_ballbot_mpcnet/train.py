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

import sys
import os

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.loss import HamiltonianLoss
from ocs2_mpcnet_core.memory import CircularMemory
from ocs2_mpcnet_core.policy import LinearPolicy

from ocs2_ballbot_mpcnet import BallbotMpcnet
from ocs2_ballbot_mpcnet import MpcnetInterface


def main(root_dir: str, config_file_name: str) -> None:
    # config
    config = Config(os.path.join(root_dir, "config", config_file_name))
    # interface
    interface = MpcnetInterface(config.DATA_GENERATION_THREADS, config.POLICY_EVALUATION_THREADS, config.RAISIM)
    # loss
    loss = HamiltonianLoss(config)
    # memory
    memory = CircularMemory(config)
    # policy
    policy = LinearPolicy(config)
    # mpcnet
    mpcnet = BallbotMpcnet(root_dir, config, interface, memory, policy, loss)
    # train
    mpcnet.train()


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 1:
        main(root_dir, sys.argv[1])
    else:
        main(root_dir, "ballbot.yaml")
