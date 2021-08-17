from ocs2_mpcnet import config

#
# config
#

# data type for tensor elements
dtype = config.dtype

# device on which tensors will be allocated
device = config.device

#
# ballbot_config
#

# name of the robot
name = "ballbot"

# (generalized) time dimension
TIME_DIM = 1

# state dimension
STATE_DIM = 10

# input dimension
INPUT_DIM = 3

# input cost for behavioral cloning
R = [2.0, 2.0, 2.0]