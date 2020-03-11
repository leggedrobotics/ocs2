//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedMpcNode.h"
#include <ocs2_quadruped_interface/QuadrupedInterface.h>

namespace switched_model {
extern template void quadrupedMpcNode<QuadrupedInterface>(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::MPC_Settings& mpcSettings,
                      const ocs2::SLQ_Settings& slqSettings);
}  // namespace switched_model
