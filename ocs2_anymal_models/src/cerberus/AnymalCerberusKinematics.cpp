//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_anymal_models/cerberus/AnymalCerberusKinematics.h"

#include <iit/rbd/traits/TraitSelector.h>

#include "ocs2_anymal_models/cerberus/generated/jacobians.h"
#include "ocs2_anymal_models/cerberus/generated/transforms.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalCerberusKinematics<SCALAR_T>* AnymalCerberusKinematics<SCALAR_T>::clone() const {
  return new AnymalCerberusKinematics<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> AnymalCerberusKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT fr_trunk_X_fr_LF_foot_;
      return fr_trunk_X_fr_LF_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case RF: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_FOOT fr_trunk_X_fr_RF_foot_;
      return fr_trunk_X_fr_RF_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case LH: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_FOOT fr_trunk_X_fr_LH_foot_;
      return fr_trunk_X_fr_LH_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case RH: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_FOOT fr_trunk_X_fr_RH_foot_;
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
switched_model::matrix3_s_t<SCALAR_T> AnymalCerberusKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT fr_base_X_fr_LF_FOOT;
      return fr_base_X_fr_LF_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case RF: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_FOOT fr_base_X_fr_RF_FOOT;
      return fr_base_X_fr_RF_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case LH: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_FOOT fr_base_X_fr_LH_FOOT;
      return fr_base_X_fr_LH_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case RH: {
      typename iit::cerberus::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_FOOT fr_base_X_fr_RH_FOOT;
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
typename AnymalCerberusKinematics<SCALAR_T>::joint_jacobian_block_t AnymalCerberusKinematics<SCALAR_T>::baseToFootJacobianBlockInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::cerberus::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LF_FOOT fr_trunk_J_fr_LF_foot_;
      return fr_trunk_J_fr_LF_foot_(jointPositions);
    }
    case RF: {
      typename iit::cerberus::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RF_FOOT fr_trunk_J_fr_RF_foot_;
      return fr_trunk_J_fr_RF_foot_(jointPositions);
    }
    case LH: {
      typename iit::cerberus::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LH_FOOT fr_trunk_J_fr_LH_foot_;
      return fr_trunk_J_fr_LH_foot_(jointPositions);
    }
    case RH: {
      typename iit::cerberus::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RH_FOOT fr_trunk_J_fr_RH_foot_;
      return fr_trunk_J_fr_RH_foot_(jointPositions);
    }
    default: {
      throw std::runtime_error("Not defined foot index.");
    }
  }
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalCerberusKinematics<ocs2::scalar_t>;
template class anymal::tpl::AnymalCerberusKinematics<ocs2::ad_scalar_t>;
