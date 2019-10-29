//
// Created by ruben on 24.09.18.
//

#ifndef OCS2_ANYMAL_LOOPSHAPING_DEFINITIONS_ASCONSTRAINT_H
#define OCS2_ANYMAL_LOOPSHAPING_DEFINITIONS_ASCONSTRAINT_H

#include <cstddef>
#include "ocs2_anymal_bear_loopshaping/OCS2AnymalAugmentedInterface.h"
#include <ocs2_quadruped_interface/MPC_ROS_Quadruped.h>
#include "ocs2_anymal_bear_loopshaping/MRT_ROS_Anymal_Augmented.h"

namespace anymal {

    enum anymal_loopshaping : size_t
    {
        SYSTEM_STATE_DIM_ = 24,
        SYSTEM_INPUT_DIM_ = 24,
        FILTER_STATE_DIM_ = 24,
        FILTER_INPUT_DIM_ = 24,
        STATE_DIM_ = 24 + 24,
        INPUT_DIM_ = 24 + 24,
        JOINT_COORD_SIZE_ = 12
    };

    using OCS2AnymalLoopshapingInterface = OCS2AnymalAugmentedInterface<
        anymal_loopshaping::STATE_DIM_,
        anymal_loopshaping::INPUT_DIM_,
        anymal_loopshaping::SYSTEM_STATE_DIM_,
        anymal_loopshaping::SYSTEM_INPUT_DIM_,
        anymal_loopshaping::FILTER_STATE_DIM_,
        anymal_loopshaping::FILTER_INPUT_DIM_,
        anymal_loopshaping::JOINT_COORD_SIZE_>;
    using MPC_ROS_Anymal_Loopshaping = switched_model::MPC_ROS_Quadruped<
        anymal_loopshaping::JOINT_COORD_SIZE_,
        anymal_loopshaping::STATE_DIM_,
        anymal_loopshaping::INPUT_DIM_>;
    using MRT_ROS_Anymal_Loopshaping = MRT_ROS_Anymal_Augmented<OCS2AnymalLoopshapingInterface>;


} // namespace anymal

#endif //OCS2_ANYMAL_LOOPSHAPING_DEFINITIONS_ASCONSTRAINT_H
