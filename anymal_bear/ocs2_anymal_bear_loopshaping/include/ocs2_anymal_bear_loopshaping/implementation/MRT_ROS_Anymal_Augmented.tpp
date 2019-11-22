//
// Created by ruben on 25.07.18.
//

#include <ocs2_anymal_bear_loopshaping/MRT_ROS_Anymal_Augmented.h>

namespace anymal {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class ANYMAL_INTERFACE>
MRT_ROS_Anymal_Augmented<ANYMAL_INTERFACE>::MRT_ROS_Anymal_Augmented(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
                                                                     const std::string& pathToConfigFolder)
    : BASE(ocs2QuadrupedInterfacePtr, "anymal") {}

}  // end of namespace anymal
