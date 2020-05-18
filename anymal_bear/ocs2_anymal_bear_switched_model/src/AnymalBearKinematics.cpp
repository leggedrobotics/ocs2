//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_anymal_bear_switched_model/core/AnymalBearKinematics.h"

#include <iit/rbd/traits/TraitSelector.h>

#include "ocs2_anymal_bear_switched_model/generated/jacobians.h"
#include "ocs2_anymal_bear_switched_model/generated/transforms.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalBearKinematics<SCALAR_T>* AnymalBearKinematics<SCALAR_T>::clone() const {
  return new AnymalBearKinematics<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> AnymalBearKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
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
      throw std::runtime_error("Not defined foot index.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> AnymalBearKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT fr_base_X_fr_LF_FOOT;
      return fr_base_X_fr_LF_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case RF: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_FOOT fr_base_X_fr_RF_FOOT;
      return fr_base_X_fr_RF_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case LH: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_FOOT fr_base_X_fr_LH_FOOT;
      return fr_base_X_fr_LH_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case RH: {
      typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_FOOT fr_base_X_fr_RH_FOOT;
      return fr_base_X_fr_RH_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    default:
      throw std::runtime_error("Undefined endeffector index.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
typename AnymalBearKinematics<SCALAR_T>::joint_jacobian_t AnymalBearKinematics<SCALAR_T>::baseToFootJacobianInBaseFrame(
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
      throw std::runtime_error("Not defined foot index.");
    }
  }

  return footJacobian;
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalBearKinematics<double>;
template class anymal::tpl::AnymalBearKinematics<ocs2::CppAdInterface<double>::ad_scalar_t>;
