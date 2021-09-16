//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_anymal_models/chimera/AnymalChimeraKinematics.h"

#include <iit/rbd/traits/TraitSelector.h>

#include "ocs2_anymal_models/chimera/generated/jacobians.h"
#include "ocs2_anymal_models/chimera/generated/transforms.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalChimeraKinematics<SCALAR_T>* AnymalChimeraKinematics<SCALAR_T>::clone() const {
  return new AnymalChimeraKinematics<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> AnymalChimeraKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT fr_trunk_X_fr_LF_foot_;
      return fr_trunk_X_fr_LF_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case RF: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_FOOT fr_trunk_X_fr_RF_foot_;
      return fr_trunk_X_fr_RF_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case LH: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_FOOT fr_trunk_X_fr_LH_foot_;
      return fr_trunk_X_fr_LH_foot_(jointPositions).template topRightCorner<3, 1>();
    }
    case RH: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_FOOT fr_trunk_X_fr_RH_foot_;
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
switched_model::matrix3_s_t<SCALAR_T> AnymalChimeraKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT fr_base_X_fr_LF_FOOT;
      return fr_base_X_fr_LF_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case RF: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_FOOT fr_base_X_fr_RF_FOOT;
      return fr_base_X_fr_RF_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case LH: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_FOOT fr_base_X_fr_LH_FOOT;
      return fr_base_X_fr_LH_FOOT(jointPositions).template topLeftCorner<3, 3>();
    }
    case RH: {
      typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_FOOT fr_base_X_fr_RH_FOOT;
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
typename AnymalChimeraKinematics<SCALAR_T>::joint_jacobian_block_t AnymalChimeraKinematics<SCALAR_T>::baseToFootJacobianBlockInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LF_FOOT fr_trunk_J_fr_LF_foot_;
      return fr_trunk_J_fr_LF_foot_(jointPositions);
    }
    case RF: {
      typename iit::chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RF_FOOT fr_trunk_J_fr_RF_foot_;
      return fr_trunk_J_fr_RF_foot_(jointPositions);
    }
    case LH: {
      typename iit::chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LH_FOOT fr_trunk_J_fr_LH_foot_;
      return fr_trunk_J_fr_LH_foot_(jointPositions);
    }
    case RH: {
      typename iit::chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RH_FOOT fr_trunk_J_fr_RH_foot_;
      return fr_trunk_J_fr_RH_foot_(jointPositions);
    }
    default: {
      throw std::runtime_error("Not defined foot index.");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
std::vector<typename AnymalChimeraKinematics<SCALAR_T>::CollisionSphere> AnymalChimeraKinematics<SCALAR_T>::collisionSpheresInBaseFrame(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  const SCALAR_T kneeRadius(0.08);
  const switched_model::vector3_s_t<SCALAR_T> kneeOffsetInKneeFrame{SCALAR_T(0.0), SCALAR_T(0.0), SCALAR_T(0.055)};

  std::vector<CollisionSphere> collisionSpheres;

  {
    typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    fr_base_X_fr_LF_KFE.update(jointPositions);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_LF_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position -= fr_base_X_fr_LF_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  {
    typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    fr_base_X_fr_RF_KFE.update(jointPositions);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_RF_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position += fr_base_X_fr_RF_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  {
    typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    fr_base_X_fr_LH_KFE.update(jointPositions);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_LH_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position -= fr_base_X_fr_LH_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  {
    typename iit::chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    fr_base_X_fr_RH_KFE.update(jointPositions);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_RH_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position += fr_base_X_fr_RH_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  return collisionSpheres;
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalChimeraKinematics<ocs2::scalar_t>;
template class anymal::tpl::AnymalChimeraKinematics<ocs2::ad_scalar_t>;
