//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h"

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 bool recompileModel)
    : Base(), adKinematicModelPtr_(adKinematicModel.clone()), adComModelPtr_(adComModel.clone()) {
  std::string libName = "anymal_dynamics";
  std::string libFolder = "/tmp/ocs2";
  const bool verbose = recompileModel;
  this->initialize(libName, libFolder, recompileModel, verbose);
}

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs)
    : Base(rhs), adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()), adComModelPtr_(rhs.adComModelPtr_->clone()) {}

ComKinoSystemDynamicsAd* ComKinoSystemDynamicsAd::clone() const {
  return new ComKinoSystemDynamicsAd(*this);
}

void ComKinoSystemDynamicsAd::systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                            ad_dynamic_vector_t& stateDerivative) const {
  // Extract elements from state
  Vector3Ad baseEulerAngles = state.segment(0, 3);
  Vector3Ad o_comPosition = state.segment(3, 3);            // in origin frame
  Vector3Ad com_baseAngularVelocity = state.segment(6, 3);  // in com frame
  Vector3Ad com_comLinearVelocity = state.segment(9, 3);    // in com frame
  ad_joint_coordinate_t qJoints = state.segment(12, 12);
  ad_joint_coordinate_t dqJoints = input.segment(12, 12);

  Vector3Ad com_base2CoM = adComModelPtr_->comPositionBaseFrame();
  Matrix3Ad o_R_b = RotationMatrixBasetoOrigin<ad_scalar_t>(baseEulerAngles);
  ad_base_coordinate_t basePose;
  basePose << baseEulerAngles, o_comPosition - o_R_b * com_base2CoM;

  // Inertia matrix in the CoM frame and its derivatives
  Matrix6Ad M = adComModelPtr_->comInertia();
  Matrix3Ad rotationalInertia = M.template topLeftCorner<3, 3>();
  Matrix3Ad rotationMInverse = rotationalInertia.inverse();
  Matrix6Ad MInverse;
  MInverse << rotationMInverse, Matrix3Ad::Zero(), Matrix3Ad::Zero(), Matrix3Ad::Identity() / M(5, 5);

  // gravity effect on CoM in CoM coordinate
  Vector3Ad o_gravityVector;
  o_gravityVector << ad_scalar_t(0), ad_scalar_t(0), ad_scalar_t(-9.81);
  Vector6Ad MInverseG;
  MInverseG << Vector3Ad::Zero(), -o_R_b.transpose() * o_gravityVector;

  // Coriolis and centrifugal forces
  ad_base_coordinate_t C;
  C.head(3) = com_baseAngularVelocity.cross(rotationalInertia * com_baseAngularVelocity);
  C.tail(3).setZero();

  // contact JacobianTransposeLambda
  adKinematicModelPtr_->update(basePose, qJoints);
  ad_base_coordinate_t JcTransposeLambda;
  JcTransposeLambda.setZero();
  for (size_t i = 0; i < 4; i++) {
    Vector3Ad com_base2StanceFeet = adKinematicModelPtr_->footPositionBaseFrame(i);
    Vector3Ad com_comToFoot = com_base2StanceFeet - com_base2CoM;
    JcTransposeLambda.head(3) += com_comToFoot.cross(input.template segment<3>(3 * i));
    JcTransposeLambda.tail(3) += input.template segment<3>(3 * i);
  }

  // angular velocities to Euler angle derivatives transformation
  Matrix3Ad transformAngVel2EulerAngDev = switched_model::AngularVelocitiesToEulerAngleDerivativesMatrix(baseEulerAngles);

  // CoM dynamics
  Eigen::Matrix<ad_scalar_t, 12, 1> stateDerivativeCoM;
  stateDerivativeCoM.segment(0, 3) = transformAngVel2EulerAngDev * com_baseAngularVelocity;
  stateDerivativeCoM.segment(3, 3) = o_R_b * com_comLinearVelocity;
  stateDerivativeCoM.segment(6, 6) = MInverse * (-C + JcTransposeLambda) - MInverseG;

  // extended state time derivatives
  stateDerivative << stateDerivativeCoM, dqJoints;
}

}  // namespace switched_model