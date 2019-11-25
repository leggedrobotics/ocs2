//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_anymal_wheels_switched_model/core/AnymalKinematics.h"

#include <iit/rbd/traits/TraitSelector.h>

#include "ocs2_anymal_wheels_switched_model/generated/jacobians.h"
#include "ocs2_anymal_wheels_switched_model/generated/transforms.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalKinematics<SCALAR_T>* AnymalKinematics<SCALAR_T>::clone() const {
  return new AnymalKinematics<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> AnymalKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT fr_trunk_X_fr_LF_foot_;
      return fr_trunk_X_fr_LF_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case RF: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_FOOT fr_trunk_X_fr_RF_foot_;
      return fr_trunk_X_fr_RF_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case LH: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_FOOT fr_trunk_X_fr_LH_foot_;
      return fr_trunk_X_fr_LH_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case RH: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_FOOT fr_trunk_X_fr_RH_foot_;
      return fr_trunk_X_fr_RH_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    default:
      std::runtime_error("Not defined foot index.");
      break;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename AnymalKinematics<SCALAR_T>::joint_jacobian_t AnymalKinematics<SCALAR_T>::baseToFootJacobianInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  joint_jacobian_t footJacobian;
  footJacobian.setZero();

  switch (footIndex) {
    case LF: {
      typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LF_FOOT fr_trunk_J_fr_LF_foot_;
      footJacobian.template block<6, 3>(0, 0) = fr_trunk_J_fr_LF_foot_(jointPositions);
      break;
    }
    case RF: {
      typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RF_FOOT fr_trunk_J_fr_RF_foot_;
      footJacobian.template block<6, 3>(0, 3) = fr_trunk_J_fr_RF_foot_(jointPositions);
      break;
    }
    case LH: {
      typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LH_FOOT fr_trunk_J_fr_LH_foot_;
      footJacobian.template block<6, 3>(0, 6) = fr_trunk_J_fr_LH_foot_(jointPositions);
      break;
    }
    case RH: {
      typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RH_FOOT fr_trunk_J_fr_RH_foot_;
      footJacobian.template block<6, 3>(0, 9) = fr_trunk_J_fr_RH_foot_(jointPositions);
      break;
    }
    default: {
      std::runtime_error("Not defined foot index.");
      break;
    }
  }

  return footJacobian;
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalKinematics<double>;
template class anymal::tpl::AnymalKinematics<ocs2::CppAdInterface<double>::ad_scalar_t>;
