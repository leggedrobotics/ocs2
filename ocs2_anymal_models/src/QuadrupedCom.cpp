#include "ocs2_anymal_models/QuadrupedCom.h"
#include "ocs2_anymal_models/DynamicsHelpers.h"

#include <ocs2_switched_model_interface/core/Rotations.h>

#include <ocs2_pinocchio_interface/urdf.h>

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace anymal {

ocs2::PinocchioInterface createQuadrupedPinocchioInterface(const std::string& urdfFilePath) {
  // add 6 DoF for the floating base
  return ocs2::getPinocchioInterfaceFromUrdfFile(urdfFilePath, pinocchio::JointModelFreeFlyer());
}

namespace tpl {
template <typename SCALAR_T>
QuadrupedCom<SCALAR_T>::QuadrupedCom(const ocs2::PinocchioInterface& pinocchioInterface)
    : pinocchioMapping_({0, 2, 1, 3}), pinocchioInterfacePtr_(new PinocchioInterface(castPinocchioInterface(pinocchioInterface))) {
  const auto& model = pinocchioInterfacePtr_->getModel();
  totalMass_ = pinocchio::computeTotalMass(model);
}

template <typename SCALAR_T>
QuadrupedCom<SCALAR_T>::QuadrupedCom(const QuadrupedCom& rhs)
    : pinocchioInterfacePtr_(new PinocchioInterface(*rhs.pinocchioInterfacePtr_)),
      totalMass_(rhs.totalMass_),
      pinocchioMapping_(rhs.pinocchioMapping_) {}

template <typename SCALAR_T>
QuadrupedCom<SCALAR_T>* QuadrupedCom<SCALAR_T>::clone() const {
  return new QuadrupedCom<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::base_coordinate_s_t<SCALAR_T> QuadrupedCom<SCALAR_T>::calculateBaseLocalAccelerations(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto jointOcs2ToPinocchio = [this](vector_t joint) -> vector_t { return pinocchioMapping_.mapJointOcs2ToPinocchio(joint); };
  /**
   * pinocchio state = [basePos(0-2) baseQuad(3-6) q(7-18)]
   *
   * basePos: Base position in Origin frame (3x1)
   * baseQuad: (x,y,z,w) (4x1)
   * q: Joint angles per leg [HAA, HFE, KFE] (3x1) [4x]
   *
   * pinocchio velocity = [v(0-2) w(3-5) qj(6-17)]
   *
   * v: Base linear velocity in Base Frame (3x1)
   * w: Base angular velocity in Base Frame (3x1)
   * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
   */
  vector_t configuration = vector_t::Zero(model.nq);
  // Leave basePos empty here - Not necessary to fill
  // baseQuad
  const Eigen::Quaternion<SCALAR_T> baseQuat = switched_model::quaternionBaseToOrigin<SCALAR_T>(switched_model::getOrientation(basePose));
  configuration.template segment<4>(3) = baseQuat.coeffs();
  // JointsPos
  configuration.template segment<switched_model::JOINT_COORDINATE_SIZE>(7) = jointOcs2ToPinocchio(jointPositions);

  // Calculate joint space inertial matrix
  pinocchio::crba(model, data, configuration);

  vector_t velocity = vector_t::Zero(model.nv);
  // Base linear velocity in Base frame
  velocity.template head<3>() = switched_model::getLinearVelocity(baseLocalVelocities);
  // Base angular velocity in Base frame
  velocity.template segment<3>(3) = switched_model::getAngularVelocity(baseLocalVelocities);
  // Joint velocity
  velocity.template segment<switched_model::JOINT_COORDINATE_SIZE>(6) = jointOcs2ToPinocchio(jointVelocities);

  const vector_t dynamicBias = pinocchio::nonLinearEffects(model, data, configuration, velocity);

  switched_model::base_coordinate_s_t<SCALAR_T> pinocchioBaseForces;
  // Force
  pinocchioBaseForces.template head<3>() = forcesOnBaseInBaseFrame.template tail<3>();
  // Wrench
  pinocchioBaseForces.template tail<3>() = forcesOnBaseInBaseFrame.template head<3>();

  switched_model::vector6_s_t<SCALAR_T> baseForcesInBaseFrame = pinocchioBaseForces - dynamicBias.head(6);
  baseForcesInBaseFrame.noalias() -= data.M.template block<6, 12>(0, 6) * jointOcs2ToPinocchio(jointAccelerations);

  // M are symmetric but pinocchio only fills in the upper triangle.
  switched_model::matrix6_s_t<SCALAR_T> Mb = data.M.topLeftCorner(6, 6).template selfadjointView<Eigen::Upper>();
  vector_t baseAcceleration = inertiaTensorSolveLinearAngular(Mb, baseForcesInBaseFrame);

  vector_t ocs2baseAcceleration(6);
  // Angular
  ocs2baseAcceleration.template head<3>() = baseAcceleration.template tail<3>();
  // Linear
  ocs2baseAcceleration.template tail<3>() = baseAcceleration.template head<3>();

  return ocs2baseAcceleration;
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::QuadrupedCom<ocs2::scalar_t>;
template class anymal::tpl::QuadrupedCom<ocs2::ad_scalar_t>;
