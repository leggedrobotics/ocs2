/*
 * AnymalCom.h
 *
 *  Created on: Aug 11, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class AnymalCom : public switched_model::ComModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = switched_model::ComModelBase<SCALAR_T>;
  using typename BASE::joint_coordinate_t;
  using typename BASE::matrix3s_t;
  using typename BASE::matrix6s_t;
  using typename BASE::vector3s_t;

  /**
   * Constructor needed for initialization
   */
  AnymalCom();

  /**
   * Default destructor
   */
  ~AnymalCom() = default;

  AnymalCom<SCALAR_T>* clone() const override;

  void setJointConfiguration(const joint_coordinate_t& q) override;

  vector3s_t comPositionBaseFrame() const override { return comPositionBaseFrame_; }

  SCALAR_T totalMass() const override { return totalMass_; }

  Eigen::Matrix<SCALAR_T, 6, 6> comInertia() const override { return comInertia_; }

 private:
  // cached values for current default joint configuration
  vector3s_t comPositionBaseFrame_;
  matrix6s_t comInertia_;
  SCALAR_T totalMass_;
};

}  // namespace tpl

using AnymalCom = tpl::AnymalCom<double>;
using AnymalComAd = tpl::AnymalCom<ocs2::CppAdInterface<double>::ad_scalar_t>;

}  // namespace anymal

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class anymal::tpl::AnymalCom<double>;
extern template class anymal::tpl::AnymalCom<ocs2::CppAdInterface<double>::ad_scalar_t>;
