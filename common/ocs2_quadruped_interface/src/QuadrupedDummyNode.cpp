//
// Created by rgrandia on 17.02.20.
//
#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>
#include <ocs2_quadruped_interface/QuadrupedInterface.h>

namespace switched_model {
extern template void quadrupedDummyNode<QuadrupedInterface>(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface,
                                                            const QuadrupedInterface::rollout_base_t* rolloutPtr,
                                                            double mrtDesiredFrequency, double mpcDesiredFrequency);
}  // namespace switched_model
