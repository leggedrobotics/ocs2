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
EXPERT_NUM = 8

# default state
default_state = [0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.575,
                 0.0, 0.0, 0.0,
                 -0.25, 0.6, -0.85,
                 -0.25, -0.6, 0.85,
                 0.25, 0.6, -0.85,
                 0.25, -0.6, 0.85]

# input bias
input_bias = [0.0, 0.0, 127.861,
              0.0, 0.0, 127.861,
              0.0, 0.0, 127.861,
              0.0, 0.0, 127.861,
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0]

# input scaling
input_scaling = [100.0, 100.0, 100.0,
                 100.0, 100.0, 100.0,
                 100.0, 100.0, 100.0,
                 100.0, 100.0, 100.0,
                 10.0, 10.0, 10.0,
                 10.0, 10.0, 10.0,
                 10.0, 10.0, 10.0,
                 10.0, 10.0, 10.0]

# (diagonally dominant) nominal centroidal inertia normalized by robot mass
normalized_inertia = [1.62079 / 52.1348, 4.83559 / 52.1348, 4.72382 / 52.1348]

# input cost for behavioral cloning
R = [0.001, 0.001, 0.001,
     0.001, 0.001, 0.001,
     0.001, 0.001, 0.001,
     0.001, 0.001, 0.001,
     5.0, 5.0, 5.0,
     5.0, 5.0, 5.0,
     5.0, 5.0, 5.0,
     5.0, 5.0, 5.0]

# dictionary for cheating
expert_for_mode = dict([(i, None) for i in range(16)])
expert_for_mode[15] = 0
expert_for_mode[6] = 1
expert_for_mode[9] = 2
expert_for_mode[7] = 3
expert_for_mode[11] = 4
expert_for_mode[13] = 5
expert_for_mode[14] = 6
