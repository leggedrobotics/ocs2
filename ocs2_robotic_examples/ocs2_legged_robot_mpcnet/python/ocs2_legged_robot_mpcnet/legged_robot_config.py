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

from ocs2_mpcnet import config

#
# config
#

# data type for tensor elements
dtype = config.dtype

# device on which tensors will be allocated
device = config.device

#
# legged_robot_config
#

# name of the robot
name = "legged_robot"

# (generalized) time dimension
TIME_DIM = 12

# state dimension
STATE_DIM = 24

# input dimension
INPUT_DIM = 24

# expert number
EXPERT_NUM = 3

# default state
default_state = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.575,
    0.0,
    0.0,
    0.0,
    -0.25,
    0.6,
    -0.85,
    -0.25,
    -0.6,
    0.85,
    0.25,
    0.6,
    -0.85,
    0.25,
    -0.6,
    0.85,
]

# input bias
input_bias = [
    0.0,
    0.0,
    127.861,
    0.0,
    0.0,
    127.861,
    0.0,
    0.0,
    127.861,
    0.0,
    0.0,
    127.861,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
]

# input scaling
input_scaling = [
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    100.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
]

# (diagonally dominant) nominal centroidal inertia normalized by robot mass
normalized_inertia = [1.62079 / 52.1348, 4.83559 / 52.1348, 4.72382 / 52.1348]

# input cost for behavioral cloning
R = [
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    0.001,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
    5.0,
]

# dictionary for cheating
expert_for_mode = dict([(i, None) for i in range(16)])
expert_for_mode[15] = 0
expert_for_mode[6] = 1
expert_for_mode[9] = 2
