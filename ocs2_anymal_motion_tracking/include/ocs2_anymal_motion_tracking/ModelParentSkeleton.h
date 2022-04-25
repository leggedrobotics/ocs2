//
// Created by rgrandia on 25.03.22.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_anymal_motion_tracking/MotionTracking.h>

namespace switched_model {

template <typename SCALAR_T>
ocs2::PinocchioInterfaceTpl<SCALAR_T> getPinocchioInterface(const std::string& urdf);

template <typename SCALAR_T>
std::vector<std::string> getAllFrames(const ocs2::PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface);

template <typename SCALAR_T>
class ModelParentSkeleton {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ModelParentSkeleton(ocs2::PinocchioInterfaceTpl<SCALAR_T> pinocchioInterface, const std::vector<std::string>& frames);

  const std::vector<MotionTarget<SCALAR_T>>& update(const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& state,
                                                    const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input);

 private:
  switched_model::joint_coordinate_s_t<SCALAR_T> mapJointConfigurationOcs2ToPinocchio(
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  ocs2::PinocchioInterfaceTpl<SCALAR_T> pinocchioInterface_;
  std::vector<size_t> frameIds_;
  std::vector<MotionTarget<SCALAR_T>> motionTargets_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class ModelParentSkeleton<scalar_t>;
extern template class ModelParentSkeleton<ad_scalar_t>;

}  // namespace switched_model