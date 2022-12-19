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

ocs2::PinocchioInterface createQuadrupedPinocchioInterfaceFromUrdfString(const std::string& urdfString) {
  // add 6 DoF for the floating base
  return ocs2::getPinocchioInterfaceFromUrdfString(urdfString, pinocchio::JointModelFreeFlyer());
}

namespace tpl {
template <typename SCALAR_T>
QuadrupedCom<SCALAR_T>::QuadrupedCom(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface)
    : pinocchioInterfacePtr_(new PinocchioInterface_s_t(castPinocchioInterface(pinocchioInterface))),
      pinocchioMapping_(frameDeclaration, pinocchioInterface) {
  const auto& model = pinocchioInterfacePtr_->getModel();
  totalMass_ = pinocchio::computeTotalMass(model);
}

template <typename SCALAR_T>
QuadrupedCom<SCALAR_T>::QuadrupedCom(const QuadrupedCom& rhs)
    : pinocchioInterfacePtr_(new PinocchioInterface_s_t(*rhs.pinocchioInterfacePtr_)),
      pinocchioMapping_(rhs.pinocchioMapping_),
      totalMass_(rhs.totalMass_) {}

template <typename SCALAR_T>
QuadrupedCom<SCALAR_T>* QuadrupedCom<SCALAR_T>::clone() const {
  return new QuadrupedCom<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedCom<SCALAR_T>::centerOfMassInBaseFrame(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto configuration = getPinnochioConfiguration(switched_model::base_coordinate_s_t<SCALAR_T>::Zero(), jointPositions);
  pinocchio::centerOfMass(model, data, configuration);
  return data.com[1];  // CoM of the full robot in the free-flyer frame, i.e. base frame.
}

template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> QuadrupedCom<SCALAR_T>::centerOfMassInWorldFrame(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto configuration = getPinnochioConfiguration(basePose, jointPositions);
  return pinocchio::centerOfMass(model, data, configuration);
}

template <typename SCALAR_T>
switched_model::base_coordinate_s_t<SCALAR_T> QuadrupedCom<SCALAR_T>::calculateBaseLocalAccelerations(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;

  auto& data = pinocchioInterfacePtr_->getData();
  const auto& model = pinocchioInterfacePtr_->getModel();
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
  const auto configuration = getPinnochioConfiguration(basePose, jointPositions);
  const auto velocity = getPinnochioVelocity(baseLocalVelocities, jointVelocities);

  // Calculate joint space inertial matrix
  pinocchio::crba(model, data, configuration);

  const vector_t dynamicBias = pinocchio::nonLinearEffects(model, data, configuration, velocity);

  switched_model::base_coordinate_s_t<SCALAR_T> pinocchioBaseForces;
  // Force
  pinocchioBaseForces.template head<3>() = forcesOnBaseInBaseFrame.template tail<3>();
  // Wrench
  pinocchioBaseForces.template tail<3>() = forcesOnBaseInBaseFrame.template head<3>();

  switched_model::vector6_s_t<SCALAR_T> baseForcesInBaseFrame = pinocchioBaseForces - dynamicBias.head(6);
  baseForcesInBaseFrame.noalias() -= data.M.template block<6, 12>(0, 6) * pinocchioMapping_.getPinocchioJointVector(jointAccelerations);

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

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> QuadrupedCom<SCALAR_T>::getPinnochioConfiguration(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  const auto& model = pinocchioInterfacePtr_->getModel();

  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> configuration(model.nq);
  // basePost
  configuration.template head<3>() = switched_model::getPositionInOrigin(basePose);
  // baseQuad
  const Eigen::Quaternion<SCALAR_T> baseQuat = switched_model::quaternionBaseToOrigin<SCALAR_T>(switched_model::getOrientation(basePose));
  configuration.template segment<4>(3) = baseQuat.coeffs();
  // JointsPos
  configuration.template segment<switched_model::JOINT_COORDINATE_SIZE>(7) = pinocchioMapping_.getPinocchioJointVector(jointPositions);
  return configuration;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> QuadrupedCom<SCALAR_T>::getPinnochioVelocity(
    const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities) const {
  const auto& model = pinocchioInterfacePtr_->getModel();

  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> velocity(model.nv);
  // Base linear velocity in Base frame
  velocity.template head<3>() = switched_model::getLinearVelocity(baseLocalVelocities);
  // Base angular velocity in Base frame
  velocity.template segment<3>(3) = switched_model::getAngularVelocity(baseLocalVelocities);
  // Joint velocity
  velocity.template segment<switched_model::JOINT_COORDINATE_SIZE>(6) = pinocchioMapping_.getPinocchioJointVector(jointVelocities);

  return velocity;
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::QuadrupedCom<ocs2::scalar_t>;
template class anymal::tpl::QuadrupedCom<ocs2::ad_scalar_t>;
