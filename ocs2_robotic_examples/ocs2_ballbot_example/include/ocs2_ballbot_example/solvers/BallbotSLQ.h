

#pragma once

#include <ocs2_mpc/MPC_SLQ.h>
#include "ocs2_ballbot_example/definitions.h"

// SLQ
extern template class ocs2::Solver_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::DDP_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::SLQ<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;

// MPC
extern template class ocs2::MPC_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::MPC_SLQ<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;