//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h"

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 const SwitchedModelModeScheduleManager& modeScheduleManager, bool recompileModel)
    : Base(STATE_DIM, INPUT_DIM),
      adKinematicModelPtr_(adKinematicModel.clone()),
      adComModelPtr_(adComModel.clone()),
      modeScheduleManagerPtr_(&modeScheduleManager) {
  std::string libName = "anymal_dynamics";
  std::string libFolder = "/tmp/ocs2";
  const bool verbose = recompileModel;
  this->initialize(libName, libFolder, recompileModel, verbose);
}

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs)
    : Base(rhs),
      adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()),
      adComModelPtr_(rhs.adComModelPtr_->clone()),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

ComKinoSystemDynamicsAd* ComKinoSystemDynamicsAd::clone() const {
  return new ComKinoSystemDynamicsAd(*this);
}

ocs2::ad_vector_t ComKinoSystemDynamicsAd::systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                                         const ocs2::ad_vector_t& input, const ocs2::ad_vector_t& parameters) const {
  const comkino_state_ad_t comkinoState = state;
  const comkino_input_ad_t comkinoInput = input;
  ocs2::ad_vector_t stateDerivative(state.rows());

  const joint_coordinate_ad_t dqJoints = getJointVelocities(comkinoInput);
  const com_state_ad_t comStateDerivative =
      computeComStateDerivative(*adComModelPtr_, *adKinematicModelPtr_, comkinoState, comkinoInput, ad_parameters_t(parameters));

  // extended state time derivatives
  stateDerivative << comStateDerivative, dqJoints;
  return stateDerivative;
}

template <typename SCALAR_T>
com_state_s_t<SCALAR_T> ComKinoSystemDynamicsAd::computeComStateDerivative(const ComModelBase<SCALAR_T>& comModel,
                                                                           const KinematicsModelBase<SCALAR_T>& kinematicsModel,
                                                                           const comkino_state_s_t<SCALAR_T>& comKinoState,
                                                                           const comkino_input_s_t<SCALAR_T>& comKinoInput,
                                                                           const ComKinoSystemDynamicsParameters<SCALAR_T>& parameters) {
  // Extract elements from state
  const base_coordinate_s_t<SCALAR_T> comPose = getComPose(comKinoState);
  const base_coordinate_s_t<SCALAR_T> com_comTwist = getComLocalVelocities(comKinoState);
  const joint_coordinate_s_t<SCALAR_T> qJoints = getJointPositions(comKinoState);

  const vector3_s_t<SCALAR_T> baseEulerAngles = getOrientation(comPose);

  const vector3_s_t<SCALAR_T> com_comAngularVelocity = getAngularVelocity(com_comTwist);
  const vector3_s_t<SCALAR_T> com_comLinearVelocity = getLinearVelocity(com_comTwist);

  // Inertia matrix in the CoM frame and its derivatives
  const matrix6_s_t<SCALAR_T> MInverse = comModel.comInertiaInverse();

  // gravity effect on CoM in CoM coordinate
  const vector3_s_t<SCALAR_T> o_gravityVector(SCALAR_T(0.0), SCALAR_T(0.0), SCALAR_T(-9.81));
  vector6_s_t<SCALAR_T> MInverseG;
  MInverseG << vector3_s_t<SCALAR_T>::Zero(), -rotateVectorOriginToBase(o_gravityVector, baseEulerAngles);

  // Coriolis and centrifugal forces
  base_coordinate_s_t<SCALAR_T> C;
  C.head(3) = com_comAngularVelocity.cross(comModel.rotationalInertia() * com_comAngularVelocity);
  C.tail(3).setZero();

  // contact JacobianTransposeLambda
  const vector3_s_t<SCALAR_T> com_base2CoM = comModel.comPositionBaseFrame();
  base_coordinate_s_t<SCALAR_T> JcTransposeLambda;
  JcTransposeLambda.setZero();
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    vector3_s_t<SCALAR_T> com_base2StanceFeet = kinematicsModel.positionBaseToFootInBaseFrame(i, qJoints);
    vector3_s_t<SCALAR_T> com_comToFoot = com_base2StanceFeet - com_base2CoM;
    JcTransposeLambda.head(3) += com_comToFoot.cross(comKinoInput.template segment<3>(3 * i));
    JcTransposeLambda.tail(3) += comKinoInput.template segment<3>(3 * i);
  }

  // External forces
  const vector3_s_t<SCALAR_T> externalForceInBase = rotateVectorOriginToBase(parameters.externalForceInOrigin, baseEulerAngles);
  JcTransposeLambda.head(3) += parameters.externalTorqueInBase - com_base2CoM.cross(externalForceInBase);  // += T + com_com2Base.cross(F)
  JcTransposeLambda.tail(3) += externalForceInBase;

  // CoM dynamics
  com_state_s_t<SCALAR_T> stateDerivativeCoM;
  stateDerivativeCoM.segment(0, 3) = switched_model::angularVelocitiesToEulerAngleDerivatives(com_comAngularVelocity, baseEulerAngles);
  stateDerivativeCoM.segment(3, 3) = rotateVectorBaseToOrigin(com_comLinearVelocity, baseEulerAngles);
  stateDerivativeCoM.segment(6, 6) = MInverse * (-C + JcTransposeLambda) - MInverseG;
  return stateDerivativeCoM;
}

template com_state_t ComKinoSystemDynamicsAd::computeComStateDerivative(const ComModelBase<scalar_t>& comModel,
                                                                        const KinematicsModelBase<scalar_t>& kinematicsModel,
                                                                        const comkino_state_t& comKinoState,
                                                                        const comkino_input_t& comKinoInput,
                                                                        const parameters_t& parameters);
template com_state_ad_t ComKinoSystemDynamicsAd::computeComStateDerivative(const ComModelBase<ad_scalar_t>& comModel,
                                                                           const KinematicsModelBase<ad_scalar_t>& kinematicsModel,
                                                                           const comkino_state_ad_t& comKinoState,
                                                                           const comkino_input_ad_t& comKinoInput,
                                                                           const ad_parameters_t& parameters);

}  // namespace switched_model
