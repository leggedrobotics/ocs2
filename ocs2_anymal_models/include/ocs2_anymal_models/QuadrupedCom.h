#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_anymal_models/QuadrupedPinocchioMapping.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class QuadrupedCom : public switched_model::ComModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PinocchioInterface = ocs2::PinocchioInterfaceTpl<SCALAR_T>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic>;

  QuadrupedCom(const ocs2::PinocchioInterface& pinocchioInterfaceconst);
  ~QuadrupedCom() = default;

  QuadrupedCom<SCALAR_T>* clone() const override;

  SCALAR_T totalMass() const override { return totalMass_; }

  switched_model::base_coordinate_s_t<SCALAR_T> calculateBaseLocalAccelerations(
      const switched_model::base_coordinate_s_t<SCALAR_T>& basePose,
      const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
      const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const override;

 private:
  QuadrupedCom(const QuadrupedCom& rhs);

  template <typename T = SCALAR_T, typename std::enable_if<std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface.toCppAd();
  }

  template <typename T = SCALAR_T, typename std::enable_if<!std::is_same<T, ocs2::ad_scalar_t>::value, bool>::type = true>
  PinocchioInterface castPinocchioInterface(const ocs2::PinocchioInterface& pinocchioInterface) {
    return pinocchioInterface;
  }

  SCALAR_T totalMass_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  QuadrupedPinocchioMappingTpl<SCALAR_T> pinocchioMapping_;
};

}  // namespace tpl

ocs2::PinocchioInterface createQuadrupedPinocchioInterface(const std::string& urdfFilePath);

using QuadrupedCom = tpl::QuadrupedCom<ocs2::scalar_t>;
using QuadrupedComAd = tpl::QuadrupedCom<ocs2::ad_scalar_t>;

}  // namespace anymal

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class anymal::tpl::QuadrupedCom<ocs2::scalar_t>;
extern template class anymal::tpl::QuadrupedCom<ocs2::ad_scalar_t>;
