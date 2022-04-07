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

"""Legged robot configuration variables.

Sets robot-specific configuration variables for legged robot.
"""

from ocs2_mpcnet_core import config

#
# config
#

# data type for tensor elements
DTYPE = config.DTYPE

# device on which tensors will be allocated
DEVICE = config.DEVICE

#
# legged_robot_config
#

# name of the robot
NAME = "legged_robot"

# (generalized) time dimension
TIME_DIM = 12

# state dimension
STATE_DIM = 24

# input dimension
INPUT_DIM = 24

# target trajectories state dimension
TARGET_STATE_DIM = STATE_DIM

# target trajectories input dimension
TARGET_INPUT_DIM = INPUT_DIM

# observation dimension
OBSERVATION_DIM = 12 + STATE_DIM

# action dimension
ACTION_DIM = INPUT_DIM

# expert number
EXPERT_NUM = 3

# default state
# fmt: off
DEFAULT_STATE = [
    0.0, 0.0, 0.0,      # normalized linear momentum
    0.0, 0.0, 0.0,      # normalized angular momentum
    0.0, 0.0, 0.575,    # position
    0.0, 0.0, 0.0,      # orientation
    -0.25, 0.6, -0.85,  # joint positions LF
    -0.25, -0.6, 0.85,  # joint positions LH
    0.25, 0.6, -0.85,   # joint positions RF
    0.25, -0.6, 0.85    # joint positions RH
]
# fmt: on

# input bias
# fmt: off
INPUT_BIAS = [
    0.0, 0.0, 127.861,  # contact forces LF
    0.0, 0.0, 127.861,  # contact forces LH
    0.0, 0.0, 127.861,  # contact forces RF
    0.0, 0.0, 127.861,  # contact forces RH
    0.0, 0.0, 0.0,      # joint velocities LF
    0.0, 0.0, 0.0,      # joint velocities LH
    0.0, 0.0, 0.0,      # joint velocities RF
    0.0, 0.0, 0.0       # joint velocities RH
]
# fmt: on

# input scaling
# fmt: off
INPUT_SCALING = [
    100.0, 100.0, 100.0,  # contact forces LF
    100.0, 100.0, 100.0,  # contact forces LH
    100.0, 100.0, 100.0,  # contact forces RF
    100.0, 100.0, 100.0,  # contact forces RH
    10.0, 10.0, 10.0,     # joint velocities LF
    10.0, 10.0, 10.0,     # joint velocities LH
    10.0, 10.0, 10.0,     # joint velocities RF
    10.0, 10.0, 10.0,     # joint velocities RH
]
# fmt: on

# (diagonally dominant) nominal centroidal inertia normalized by robot mass
NORMALIZED_INERTIA = [1.62079 / 52.1348, 4.83559 / 52.1348, 4.72382 / 52.1348]

# input cost for behavioral cloning
# fmt: off
R = [
    0.001, 0.001, 0.001,  # contact forces LF
    0.001, 0.001, 0.001,  # contact forces LH
    0.001, 0.001, 0.001,  # contact forces RF
    0.001, 0.001, 0.001,  # contact forces RH
    5.0, 5.0, 5.0,        # joint velocities LF
    5.0, 5.0, 5.0,        # joint velocities LH
    5.0, 5.0, 5.0,        # joint velocities RF
    5.0, 5.0, 5.0,        # joint velocities RH
]
# fmt: on

# dictionary for cheating with the gating loss
# assigns each of the OCS2 modes to an expert that is responsible for covering the corresponding contact configuration
EXPERT_FOR_MODE = dict([(i, None) for i in range(16)])
EXPERT_FOR_MODE[15] = 0  # stance
EXPERT_FOR_MODE[6] = 1  # trot
EXPERT_FOR_MODE[9] = 2  # trot
