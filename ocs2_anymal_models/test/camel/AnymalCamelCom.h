/*
 * AnymalCamelCom.h
 *
 *  Created on: Aug 11, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class AnymalCamelCom : public switched_model::ComModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Constructor needed for initialization
   */
  AnymalCamelCom();

  /**
   * Default destructor
   */
  ~AnymalCamelCom() = default;

  AnymalCamelCom<SCALAR_T>* clone() const override;

  SCALAR_T totalMass() const override { return totalMass_; }

  switched_model::base_coordinate_s_t<SCALAR_T> calculateBaseLocalAccelerations(
      const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
      const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
      const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const override;

 private:
  SCALAR_T totalMass_;
};

}  // namespace tpl

using AnymalCamelCom = tpl::AnymalCamelCom<ocs2::scalar_t>;
using AnymalCamelComAd = tpl::AnymalCamelCom<ocs2::ad_scalar_t>;

}  // namespace anymal

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class anymal::tpl::AnymalCamelCom<ocs2::scalar_t>;
extern template class anymal::tpl::AnymalCamelCom<ocs2::ad_scalar_t>;
