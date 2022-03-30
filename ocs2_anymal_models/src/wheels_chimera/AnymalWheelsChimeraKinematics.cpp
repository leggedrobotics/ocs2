/*
 * AnymalWheelsChimeraKinematics.cpp
 *
 *  Created on: Nov 25, 2019
 *      Author: Marko Bjelonic
 */

#include "ocs2_anymal_models/wheels_chimera/AnymalWheelsChimeraKinematics.h"
#include <ocs2_anymal_models/wheels_chimera/WheelsChimeraSwitchedModel.h>

#include <iit/rbd/traits/TraitSelector.h>

#include "ocs2_anymal_models/wheels_chimera/generated/jacobians.h"
#include "ocs2_anymal_models/wheels_chimera/generated/transforms.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
AnymalWheelsChimeraKinematics<SCALAR_T>* AnymalWheelsChimeraKinematics<SCALAR_T>::clone() const {
  return new AnymalWheelsChimeraKinematics<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> AnymalWheelsChimeraKinematics<SCALAR_T>::baseToLegRootInBaseFrame(size_t footIndex) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  switch (footIndex) {
    case LF: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_HAA fr_trunk_X_fr_LF_haa_;
      return fr_trunk_X_fr_LF_haa_.template topRightCorner<3, 1>();
    }
    case RF: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_HAA fr_trunk_X_fr_RF_haa_;
      return fr_trunk_X_fr_RF_haa_.template topRightCorner<3, 1>();
    }
    case LH: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_HAA fr_trunk_X_fr_LH_haa_;
      return fr_trunk_X_fr_LH_haa_.template topRightCorner<3, 1>();
    }
    case RH: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_HAA fr_trunk_X_fr_RH_haa_;
      return fr_trunk_X_fr_RH_haa_.template topRightCorner<3, 1>();
    }
    default:
      throw std::runtime_error("Not defined foot index.");
  }
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> AnymalWheelsChimeraKinematics<SCALAR_T>::positionBaseToWheelAxisInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  const auto q = wheels_chimera::getExtendedJointCoordinates(jointPositions);

  switch (footIndex) {
    case LF: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_WHEEL_L p;
      return p(q).template topRightCorner<3, 1>();
    }
    case RF: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_WHEEL_L p;
      return p(q).template topRightCorner<3, 1>();
    }
    case LH: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_WHEEL_L p;
      return p(q).template topRightCorner<3, 1>();
    }
    case RH: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_WHEEL_L p;
      return p(q).template topRightCorner<3, 1>();
    }
    default:
      throw std::runtime_error("Undefined endeffector index.");
  }
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> AnymalWheelsChimeraKinematics<SCALAR_T>::positionBaseToFootInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  SCALAR_T haa = jointPositions(footIndex * 3);  // HAA angle for the requested leg
  switched_model::vector3_s_t<SCALAR_T> wheelOffset;
  wheelOffset.x() = 0;
  wheelOffset.y() = wheelRadius_ * sin(haa);
  wheelOffset.z() = -wheelRadius_ * cos(haa);

  return positionBaseToWheelAxisInBaseFrame(footIndex, jointPositions) + wheelOffset;
}

template <typename SCALAR_T>
typename AnymalWheelsChimeraKinematics<SCALAR_T>::joint_jacobian_block_t
AnymalWheelsChimeraKinematics<SCALAR_T>::baseToFootJacobianBlockInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  Eigen::Matrix<SCALAR_T, 6, 3> wheelOffsetJacobian;
  wheelOffsetJacobian.setZero();

  SCALAR_T haa = jointPositions(footIndex * 3);  // HAA angle for the requested leg
  wheelOffsetJacobian(4, 0) = wheelRadius_ * cos(haa);
  wheelOffsetJacobian(5, 0) = wheelRadius_ * sin(haa);

  const auto q = wheels_chimera::getExtendedJointCoordinates(jointPositions);

  switch (footIndex) {
    case LF: {
      typename iit::wheels_chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LF_WHEEL_L fr_base_J_fr_LF_foot_;
      return fr_base_J_fr_LF_foot_(q).template leftCols<3>() + wheelOffsetJacobian;
    }
    case RF: {
      typename iit::wheels_chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RF_WHEEL_L fr_base_J_fr_RF_foot_;
      return fr_base_J_fr_RF_foot_(q).template leftCols<3>() + wheelOffsetJacobian;
    }
    case LH: {
      typename iit::wheels_chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LH_WHEEL_L fr_base_J_fr_LH_foot_;
      return fr_base_J_fr_LH_foot_(q).template leftCols<3>() + wheelOffsetJacobian;
    }
    case RH: {
      typename iit::wheels_chimera::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RH_WHEEL_L fr_base_J_fr_RH_foot_;
      return fr_base_J_fr_RH_foot_(q).template leftCols<3>() + wheelOffsetJacobian;
    }
    default: {
      throw std::runtime_error("Undefined endeffector index.");
    }
  }
}

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> AnymalWheelsChimeraKinematics<SCALAR_T>::wheelAxisOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  const auto q = wheels_chimera::getExtendedJointCoordinates(jointPositions);
  switch (footIndex) {
    case LF: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_WHEEL_L fr_base_X_fr_LF_WHEEL_L;
      return fr_base_X_fr_LF_WHEEL_L(q).template topLeftCorner<3, 3>();
    }
    case RF: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_WHEEL_L fr_base_X_fr_RF_WHEEL_L;
      return fr_base_X_fr_RF_WHEEL_L(q).template topLeftCorner<3, 3>();
    }
    case LH: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_WHEEL_L fr_base_X_fr_LH_WHEEL_L;
      return fr_base_X_fr_LH_WHEEL_L(q).template topLeftCorner<3, 3>();
    }
    case RH: {
      typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_WHEEL_L fr_base_X_fr_RH_WHEEL_L;
      return fr_base_X_fr_RH_WHEEL_L(q).template topLeftCorner<3, 3>();
    }
    default: {
      throw std::runtime_error("Undefined endeffector index.");
    }
  }
}

template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> AnymalWheelsChimeraKinematics<SCALAR_T>::footOrientationInBaseFrame(
    size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  // clang-format off
  switched_model::matrix3_s_t<SCALAR_T> wheelAxis_R_foot;
  wheelAxis_R_foot<< SCALAR_T(1), SCALAR_T(0), SCALAR_T(0),
                       SCALAR_T(0), SCALAR_T(0), SCALAR_T(-1),
                       SCALAR_T(0), SCALAR_T(1), SCALAR_T(0);
  // clang-format on

  return wheelAxisOrientationInBaseFrame(footIndex, jointPositions) * wheelAxis_R_foot;
}

template <typename SCALAR_T>
std::vector<typename AnymalWheelsChimeraKinematics<SCALAR_T>::CollisionSphere>
AnymalWheelsChimeraKinematics<SCALAR_T>::collisionSpheresInBaseFrame(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;

  const auto q = wheels_chimera::getExtendedJointCoordinates(jointPositions);

  const SCALAR_T kneeRadius(0.08);
  const switched_model::vector3_s_t<SCALAR_T> kneeOffsetInKneeFrame{SCALAR_T(0.0), SCALAR_T(0.0), SCALAR_T(0.055)};

  std::vector<CollisionSphere> collisionSpheres;

  {
    typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    fr_base_X_fr_LF_KFE.update(q);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_LF_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position -= fr_base_X_fr_LF_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  {
    typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    fr_base_X_fr_RF_KFE.update(q);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_RF_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position += fr_base_X_fr_RF_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  {
    typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    fr_base_X_fr_LH_KFE.update(q);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_LH_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position -= fr_base_X_fr_LH_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  {
    typename iit::wheels_chimera::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    fr_base_X_fr_RH_KFE.update(q);
    collisionSpheres.push_back(CollisionSphere{fr_base_X_fr_RH_KFE.template topRightCorner<3, 1>(), kneeRadius});
    collisionSpheres.back().position += fr_base_X_fr_RH_KFE.template topLeftCorner<3, 3>() * kneeOffsetInKneeFrame;
  }

  return collisionSpheres;
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalWheelsChimeraKinematics<ocs2::scalar_t>;
template class anymal::tpl::AnymalWheelsChimeraKinematics<ocs2::ad_scalar_t>;
