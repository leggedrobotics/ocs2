//
// Created by ruben on 25.07.18.
//

#ifndef MRT_ROS_ANYMAL_AUGMENTED_H_
#define MRT_ROS_ANYMAL_AUGMENTED_H_

#include <ocs2_quadruped_interface/MRT_ROS_Quadruped.h>

namespace anymal {

template <class ANYMAL_INTERFACE>
class MRT_ROS_Anymal_Augmented : public switched_model::MRT_ROS_Quadruped<ANYMAL_INTERFACE::JOINT_COORD_SIZE_, ANYMAL_INTERFACE::STATE_DIM_,
                                                                          ANYMAL_INTERFACE::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<MRT_ROS_Anymal_Augmented> Ptr;

  using BASE =
      switched_model::MRT_ROS_Quadruped<ANYMAL_INTERFACE::JOINT_COORD_SIZE_, ANYMAL_INTERFACE::STATE_DIM_, ANYMAL_INTERFACE::INPUT_DIM_>;

  using typename BASE::quadruped_interface_ptr_t;

  typedef ANYMAL_INTERFACE ocs2_anymal_interface_t;

  MRT_ROS_Anymal_Augmented(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr, const std::string& pathToConfigFolder);

  ~MRT_ROS_Anymal_Augmented() = default;
};

}  // end of namespace anymal

//#include "implementation/MRT_ROS_Anymal_Augmented.tpp"

#endif /* MRT_ROS_ANYMAL_AUGMENTED_H_ */
