//
// Created by rgrandia on 19.08.19.
//

#pragma once

#include "ocs2_ballbot_example/definitions.h"
#include <ocs2_mpc/MPC_SLQ.h>

// SLQ
extern template class ocs2::Solver_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::DDP_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::SLQ_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::SLQ<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::SLQ_MP<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;

// MPC
extern template class ocs2::MPC_BASE<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
extern template class ocs2::MPC_SLQ<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;