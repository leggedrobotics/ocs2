

#include <ocs2_mpc/MPC_PI.h>
#include "ocs2_ballbot_example/definitions.h"

// PI
template class ocs2::Solver_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
template class ocs2::PiSolver<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;

// MPC
template class ocs2::MPC_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
template class ocs2::MPC_PI<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;