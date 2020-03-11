//
// Created by rgrandia on 18.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedSlqMpc.h"
#include "ocs2_quadruped_interface/QuadrupedInterface.h"

/**  Explicit instantiation of MPC and SLQ functions */
namespace switched_model {
extern template std::unique_ptr<ocs2::SLQ<STATE_DIM, INPUT_DIM>> getSlq<QuadrupedInterface>(const QuadrupedInterface& quadrupedInterface, const ocs2::SLQ_Settings& slqSettings);
extern template std::unique_ptr<ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>> getMpc<QuadrupedInterface>(const QuadrupedInterface& quadrupedInterface, const ocs2::MPC_Settings& mpcSettings,
                                                            const ocs2::SLQ_Settings& slqSettings);
}  // namespace switched_model

/**  Explicit instantiation of MPC and SLQ classes */
namespace ocs2 {
template class MPC_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class MPC_SLQ<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class Solver_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class DDP_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class SLQ<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
}  // namespace ocs2
