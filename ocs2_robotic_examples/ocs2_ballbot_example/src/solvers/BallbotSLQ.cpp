

#include <ocs2_mpc/MPC_SLQ.h>
#include "ocs2_ballbot_example/definitions.h"

// SLQ
template class ocs2::Solver_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
template class ocs2::DDP_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
template class ocs2::SLQ<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;

// MPC
template class ocs2::MPC_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
template class ocs2::MPC_SLQ<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;