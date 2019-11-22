//
// Created by ruben on 01.12.18.
//

#include <ocs2_anymal_bear_loopshaping/asConstraint/definitions.h>
#include <ocs2_anymal_bear_loopshaping/implementation/OCS2AnymalAugmentedInterface.tpp>
#include <ocs2_anymal_bear_loopshaping/implementation/MRT_ROS_Anymal_Augmented.tpp>

namespace anymal{
    template class OCS2AnymalAugmentedInterface<
        anymal_loopshaping::STATE_DIM_,
        anymal_loopshaping::INPUT_DIM_,
        anymal_loopshaping::SYSTEM_STATE_DIM_,
        anymal_loopshaping::SYSTEM_INPUT_DIM_,
        anymal_loopshaping::FILTER_STATE_DIM_,
        anymal_loopshaping::FILTER_INPUT_DIM_,
        anymal_loopshaping::JOINT_COORD_SIZE_>;
    template class MRT_ROS_Anymal_Augmented<OCS2AnymalLoopshapingInterface>;
}

