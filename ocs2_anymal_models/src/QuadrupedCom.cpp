#include "ocs2_anymal_models/QuadrupedCom.h"

#include <ocs2_pinocchio_interface/urdf.h>

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Cholesky>

// Test
#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include <ocs2_anymal_models/camel/generated/inverse_dynamics.h>
#include "ocs2_anymal_models/camel/generated/inertia_properties.h"
#include "ocs2_anymal_models/camel/generated/jsim.h"
#include "ocs2_anymal_models/camel/generated/miscellaneous.h"
#include "ocs2_anymal_models/camel/generated/transforms.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace anymal {

ocs2::PinocchioInterface createQuadrupedPinocchioInterface(const std::string& urdfFilePath) {
  // add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelSpherical());
  jointComposite.addJoint(pinocchio::JointModelTranslation());

  return ocs2::getPinocchioInterfaceFromUrdfFile(urdfFilePath, jointComposite);
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
  const Eigen::Quaternion<SCALAR_T> baseQuat(Eigen::AngleAxis<SCALAR_T>(basePose(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX()) *
                                             Eigen::AngleAxis<SCALAR_T>(basePose(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
                                             Eigen::AngleAxis<SCALAR_T>(basePose(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()));

  auto baseToOrigin = [&baseQuat](vector_t v) -> vector_t { return baseQuat.toRotationMatrix() * v; };
  auto originToBase = [&baseQuat](vector_t v) -> vector_t { return baseQuat.toRotationMatrix().transpose() * v; };
  auto jointOcs2ToPinocchio = [this](vector_t joint) -> vector_t { return pinocchioMapping_.mapJointOcs2ToPinocchio(joint); };
  /**
   * pinocchio state = [baseQuad(0-3) basePos(5-6) q(7-18)]
   *
   * baseQuad: (x,y,z,w) (4x1)
   * basePos: Base position in Origin frame (3x1)
   * q: Joint angles per leg [HAA, HFE, KFE] (3x1) [4x]
   *
   * pinocchio velocity = [w(0-2) v(3-5) qj(6-17)]
   *
   * w: Base angular velocity in Origin Frame (3x1)
   * v: Base linear velocity in Origin Frame (3x1)
   * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
   */
  vector_t configuration = vector_t::Zero(model.nq);
  // baseQuad
  configuration.template segment<4>(0) = baseQuat.coeffs();
  // Leave basePos empty here - Not necessary
  // JointsPos
  configuration.template segment<switched_model::JOINT_COORDINATE_SIZE>(7) = jointOcs2ToPinocchio(jointPositions);

  // Calculate joint space inertial matrix
  pinocchio::crba(model, data, configuration);
  // M are symmetric but pinocchio only fills in the upper triangle.
  data.M.template triangularView<Eigen::StrictlyLower>() = data.M.transpose().template triangularView<Eigen::StrictlyLower>();

  vector_t velocity = vector_t::Zero(model.nv);
  // // Base angular velocity in Origin frame
  // velocity.template head<3>() = baseToOrigin(switched_model::getAngularVelocity(baseLocalVelocities));
  // // Base linear velocity
  // velocity.template segment<3>(3) = baseToOrigin(switched_model::getLinearVelocity(baseLocalVelocities)) +
  //                                   velocity.template head<3>().cross(switched_model::getPositionInOrigin(basePose));
  // Base angular velocity in Origin frame
  velocity.template head<3>() = switched_model::getAngularVelocity(baseLocalVelocities);
  // Base linear velocity
  velocity.template segment<3>(3) = switched_model::getLinearVelocity(baseLocalVelocities);
  // Joint velocity
  velocity.template segment<switched_model::JOINT_COORDINATE_SIZE>(6) = jointOcs2ToPinocchio(jointVelocities);
  const vector_t dynamicBias = pinocchio::nonLinearEffects(model, data, configuration, velocity);

  switched_model::base_coordinate_s_t<SCALAR_T> pinocchioBaseForces;
  // forcesOnBaseInBaseFrame  { torque on base in Base Frame (3x1), linear forces on the base in Base Frame (3x1) }.
  // // Force
  // pinocchioBaseForces.template tail<3>() = baseToOrigin(forcesOnBaseInBaseFrame.template tail<3>());
  // // Wrench
  // pinocchioBaseForces.template head<3>() = baseToOrigin(forcesOnBaseInBaseFrame.template head<3>()) +
  //                                          pinocchioBaseForces.template tail<3>().cross(switched_model::getPositionInOrigin(basePose));

  // Force
  pinocchioBaseForces.template tail<3>() = forcesOnBaseInBaseFrame.template tail<3>();
  // Wrench
  pinocchioBaseForces.template head<3>() = forcesOnBaseInBaseFrame.template head<3>());

  vector_t pinocchioJointAccelerations = jointOcs2ToPinocchio(jointAccelerations);
  vector_t baseForces = pinocchioBaseForces - dynamicBias.head(6) - data.M.template block<6, 12>(0, 6) * pinocchioJointAccelerations;

  vector_t baseAcceleration = data.M.topLeftCorner(6, 6).llt().solve(baseForces);

  // vector_t ocs2baseAcceleration(6);
  // // Angular
  // ocs2baseAcceleration.template head<3>() = originToBase(baseAcceleration.template head<3>());
  // // Linear
  // ocs2baseAcceleration.template tail<3>() = originToBase(baseAcceleration.template tail<3>()) +
  //                                           ocs2baseAcceleration.template head<3>().cross(switched_model::getPositionInOrigin(basePose));

  // ocs2baseAcceleration.template tail<3>() +=
  //     switched_model::getAngularVelocity(baseLocalVelocities).cross(switched_model::getLinearVelocity(baseLocalVelocities));

  {
    using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
    iit::camel::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
    iit::camel::tpl::ForceTransforms<trait_t> forceTransforms_;
    iit::camel::tpl::MotionTransforms<trait_t> motionTransforms_;
    iit::camel::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
    iit::camel::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

    return anymal::robcogen_helpers::computeBaseAcceleration(inverseDynamics_, jointSpaceInertiaMatrix_, basePose, baseLocalVelocities,
                                                             jointPositions, jointVelocities, jointAccelerations, forcesOnBaseInBaseFrame);

    const auto baseDynamics = anymal::robcogen_helpers::getBaseDynamicsTermsImpl(inverseDynamics_, jointSpaceInertiaMatrix_, basePose,
                                                                                 baseLocalVelocities, jointPositions, jointVelocities);

    vector_t G = computeGeneralizedGravity(model, data, configuration);
    std::cerr << "Robo:\n"
              << baseDynamics.Mb << "\nMj:\n"
              << "\nPino:\n"
              << data.M.template block<6, 6>(0, 0) << "\n";

    std::cerr << "\nC+g Robo:\n"
              << (baseDynamics.C + baseDynamics.Mb * baseDynamics.invMbG).transpose() << "\n"
              << "\nPino :\n " << dynamicBias.topRows(6).transpose() << "\n";

    std::cerr << "G Robo:\n"
              << (baseDynamics.Mb * baseDynamics.invMbG).transpose() << "\n"
              << "\nPino :\n " << G.transpose().head(6) << "\n";

    std::cerr << "\n\nM*ddqj Robo:  " << (baseDynamics.Mj * jointAccelerations).transpose()
              << "\nPino :        " << (data.M.template block<6, 12>(0, 6) * pinocchioJointAccelerations).transpose();

    // std::cerr << "\n\nBase force Robo:\n"
    //           << forcesOnBaseInBaseFrame.transpose() << "\nPino:\n"
    //           << pinocchioBaseForces.transpose() << "\n\n";

    switched_model::base_coordinate_s_t<SCALAR_T> baseForces = forcesOnBaseInBaseFrame - baseDynamics.C;
    baseForces.noalias() -= baseDynamics.Mj * jointAccelerations;

    std::cerr << "\nacc  Robo:\n"
              << anymal::robcogen_helpers::inertiaTensorSolve<SCALAR_T>(baseDynamics.Mb, baseForces - baseDynamics.Mb * baseDynamics.invMbG)
                     .transpose()
              << "\n"
              << (anymal::robcogen_helpers::inertiaTensorSolve<SCALAR_T>(baseDynamics.Mb, baseForces) - baseDynamics.invMbG).transpose()
              << "\nPino:\n"
              << ocs2baseAcceleration.transpose() << "\n\n";

    std::cerr << "\n\nBase force Pino:\n" << baseForces.transpose() << "\n\n";

    return anymal::robcogen_helpers::testAcceleration(inverseDynamics_, jointSpaceInertiaMatrix_, basePose, baseLocalVelocities,
                                                      jointPositions, jointVelocities, jointAccelerations, forcesOnBaseInBaseFrame);

    // std::cerr << "\n" << std::boolalpha << data.M.template block<6, 6>(0, 0).isApprox(baseDynamics.Mb) << "\n";

    // std::cerr << *pinocchioInterfacePtr_ << "\n";
    // std::cerr << model.inertias[model.getBodyId("base")] << "\n";
  }

  return baseAcceleration;
}

// template <typename SCALAR_T>
// switched_model::base_coordinate_s_t<SCALAR_T> QuadrupedCom<SCALAR_T>::calculateBaseLocalAccelerations(
//     const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>&
//     baseLocalVelocities, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions, const
//     switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities, const switched_model::joint_coordinate_s_t<SCALAR_T>&
//     jointAccelerations, const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
//   using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
//   iit::camel::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
//   iit::camel::tpl::ForceTransforms<trait_t> forceTransforms_;
//   iit::camel::tpl::MotionTransforms<trait_t> motionTransforms_;
//   iit::camel::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
//   iit::camel::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

//   const auto baseDynamics = anymal::robcogen_helpers::getBaseDynamicsTermsImpl(inverseDynamics_, jointSpaceInertiaMatrix_, basePose,
//                                                                                baseLocalVelocities, jointPositions, jointVelocities);

//   switched_model::base_coordinate_s_t<SCALAR_T> baseForces = forcesOnBaseInBaseFrame - baseDynamics.C;
//   baseForces.noalias() -= baseDynamics.Mj * jointAccelerations;
//   std::cerr << "C:\n" << baseDynamics.C.transpose() << "\ninvMbG:\n" << baseDynamics.invMbG.transpose() << "\n\n";
//   return anymal::robcogen_helpers::computeBaseAcceleration(inverseDynamics_, jointSpaceInertiaMatrix_, basePose, baseLocalVelocities,
//                                                            jointPositions, jointVelocities, jointAccelerations, forcesOnBaseInBaseFrame);
// }

// template <typename SCALAR_T>
// switched_model::base_coordinate_s_t<SCALAR_T> QuadrupedCom<SCALAR_T>::calculateBaseLocalAccelerations(
//     const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>&
//     baseLocalVelocities, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions, const
//     switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities, const switched_model::joint_coordinate_s_t<SCALAR_T>&
//     jointAccelerations, const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
//   auto& data = pinocchioInterfacePtr_->getData();
//   const auto& model = pinocchioInterfacePtr_->getModel();
//   const Eigen::Quaternion<SCALAR_T> baseQuat(Eigen::AngleAxis<SCALAR_T>(basePose(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX()) *
//                                              Eigen::AngleAxis<SCALAR_T>(basePose(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
//                                              Eigen::AngleAxis<SCALAR_T>(basePose(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()));

//   auto baseToOrigin = [&baseQuat](vector_t v) -> vector_t { return baseQuat.toRotationMatrix() * v; };
//   auto worldToBase = [&baseQuat](vector_t v) -> vector_t { return baseQuat.toRotationMatrix().transpose() * v; };
//   auto jointOcs2ToPinocchio = [this](vector_t joint) -> vector_t { return pinocchioMapping_.mapJointOcs2ToPinocchio(joint); };
//   /**
//    * pinocchio state = [baseQuad(0-3) basePos(5-6) q(7-18)]
//    *
//    * baseQuad: (x,y,z,w) (4x1)
//    * basePos: Base position in Origin frame (3x1)
//    * q: Joint angles per leg [HAA, HFE, KFE] (3x1) [4x]
//    *
//    * pinocchio velocity = [w(0-2) v(3-5) qj(6-17)]
//    *
//    * w: Base angular velocity in Origin Frame (3x1)
//    * v: Base linear velocity in Origin Frame (3x1)
//    * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
//    */
//   vector_t configuration = vector_t::Zero(model.nq);
//   // baseQuad
//   configuration.template segment<4>(0) = baseQuat.coeffs();
//   // Leave basePos empty here - Not necessary
//   configuration.template segment<3>(4) = switched_model::getPositionInOrigin(basePose);
//   // JointsPos
//   configuration.template segment<switched_model::JOINT_COORDINATE_SIZE>(7) = jointOcs2ToPinocchio(jointPositions);

//   vector_t velocity = vector_t::Zero(model.nv);
//   // Base angular velocity in Origin frame
//   velocity.template head<3>() = switched_model::getAngularVelocity(baseLocalVelocities);
//   // Base linear velocity
//   velocity.template segment<3>(3) = switched_model::getLinearVelocity(baseLocalVelocities);
//   // Joint velocity
//   velocity.template segment<switched_model::JOINT_COORDINATE_SIZE>(6) = jointOcs2ToPinocchio(jointVelocities);

//   vector_t acceleration = vector_t::Zero(model.nv);
//   // Base angular velocity in Origin frame
//   acceleration.template head<3>() = switched_model::getAngularAcceleration(baseLocalVelocities));
//   // Base linear velocity
//   velocity.template segment<3>(3) = baseToOrigin(switched_model::getLinearVelocity(baseLocalVelocities));
//   // Joint velocity
//   velocity.template segment<switched_model::JOINT_COORDINATE_SIZE>(6) = jointOcs2ToPinocchio(jointVelocities);

//   pinocchio::forwardKinematics(model, data, configuration, velocity, const Eigen::MatrixBase<TangentVectorType2>& a);
//   switched_model::base_coordinate_s_t<SCALAR_T> pinocchioBaseForces;
//   // forcesOnBaseInBaseFrame  { torque on base in Base Frame (3x1), linear forces on the base in Base Frame (3x1) }.
//   // Force
//   pinocchioBaseForces.template tail<3>() = baseToOrigin(forcesOnBaseInBaseFrame.template tail<3>());
//   // Wrench
//   pinocchioBaseForces.template head<3>() = baseToOrigin(forcesOnBaseInBaseFrame.template head<3>()) +
//                                            pinocchioBaseForces.template tail<3>().cross(switched_model::getPositionInOrigin(basePose));

//   vector_t pinocchioJointAccelerations = jointOcs2ToPinocchio(jointAccelerations);

//   return ocs2baseAcceleration;
// }

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::QuadrupedCom<ocs2::scalar_t>;
// template class anymal::tpl::QuadrupedCom<ocs2::ad_scalar_t>;
