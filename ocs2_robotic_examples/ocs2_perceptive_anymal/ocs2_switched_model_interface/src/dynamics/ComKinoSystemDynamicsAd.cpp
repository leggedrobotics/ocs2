//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h"

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

namespace {
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> computeExternalForcesInBaseFrame(const KinematicsModelBase<SCALAR_T>& kinematicsModel,
                                                               const comkino_state_s_t<SCALAR_T>& comKinoState,
                                                               const comkino_input_s_t<SCALAR_T>& comKinoInput,
                                                               const ComKinoSystemDynamicsParameters<SCALAR_T>& parameters) {
  // Extract elements from state
  const base_coordinate_s_t<SCALAR_T> basePose = getBasePose(comKinoState);
  const joint_coordinate_s_t<SCALAR_T> qJoints = getJointPositions(comKinoState);
  const vector3_s_t<SCALAR_T> baseEulerAngles = getOrientation(basePose);

  // contact JacobianTransposeLambda
  base_coordinate_s_t<SCALAR_T> JcTransposeLambda = base_coordinate_s_t<SCALAR_T>::Zero();
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    const vector3_s_t<SCALAR_T> baseToFootInBase = kinematicsModel.positionBaseToFootInBaseFrame(i, qJoints);
    const vector3_s_t<SCALAR_T> contactForce = comKinoInput.template segment<3>(3 * i);
    JcTransposeLambda.head(3) += baseToFootInBase.cross(contactForce);
    JcTransposeLambda.tail(3) += contactForce;
  }

  // External forces
  const vector3_s_t<SCALAR_T> externalForceInBase = rotateVectorOriginToBase(parameters.externalForceInOrigin, baseEulerAngles);
  JcTransposeLambda.head(3) += parameters.externalTorqueInBase;
  JcTransposeLambda.tail(3) += externalForceInBase;

  return JcTransposeLambda;
};
}  // namespace

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 const DynamicsParametersSynchronizedModule& dynamicsParametersModule,
                                                 ModelSettings settings)
    : adKinematicModelPtr_(adKinematicModel.clone()),
      adComModelPtr_(adComModel.clone()),
      dynamicsParametersModulePtr_(&dynamicsParametersModule),
      settings_(settings) {
  const std::string libName = settings_.robotName_ + "_dynamics";
  this->initialize(STATE_DIM, INPUT_DIM, libName, settings_.autodiffLibraryFolder_, settings_.recompileLibraries_);
}

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs)
    : Base(rhs),
      adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()),
      adComModelPtr_(rhs.adComModelPtr_->clone()),
      dynamicsParametersModulePtr_(rhs.dynamicsParametersModulePtr_),
      settings_(rhs.settings_) {}

ComKinoSystemDynamicsAd* ComKinoSystemDynamicsAd::clone() const {
  return new ComKinoSystemDynamicsAd(*this);
}

ocs2::ad_vector_t ComKinoSystemDynamicsAd::systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                                         const ocs2::ad_vector_t& input, const ocs2::ad_vector_t& parameters) const {
  const comkino_state_ad_t comkinoState = state;
  const comkino_input_ad_t comkinoInput = input;

  const joint_coordinate_ad_t dqJoints = getJointVelocities(comkinoInput);

  const com_state_ad_t comStateDerivative =
      computeComStateDerivative(*adComModelPtr_, *adKinematicModelPtr_, comkinoState, comkinoInput, ad_parameters_t(parameters));

  // extended state time derivatives
  ocs2::ad_vector_t stateDerivative(state.rows());
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
  const base_coordinate_s_t<SCALAR_T> basePose = getBasePose(comKinoState);
  const vector3_s_t<SCALAR_T> baseEulerAngles = getOrientation(basePose);
  const base_coordinate_s_t<SCALAR_T> baseLocalVelocities = getBaseLocalVelocities(comKinoState);
  const vector3_s_t<SCALAR_T> baseLocalAngularVelocity = getAngularVelocity(baseLocalVelocities);
  const vector3_s_t<SCALAR_T> baseLocalLinearVelocity = getLinearVelocity(baseLocalVelocities);
  const joint_coordinate_s_t<SCALAR_T> qJoints = getJointPositions(comKinoState);
  const joint_coordinate_s_t<SCALAR_T> qdJoints = getJointVelocities(comKinoInput);

  const base_coordinate_s_t<SCALAR_T> extForce = computeExternalForcesInBaseFrame(kinematicsModel, comKinoState, comKinoInput, parameters);

  // pose dynamics
  com_state_s_t<SCALAR_T> stateDerivativeCoM;
  stateDerivativeCoM.segment(0, 3) = switched_model::angularVelocitiesToEulerAngleDerivatives(baseLocalAngularVelocity, baseEulerAngles);
  stateDerivativeCoM.segment(3, 3) = rotateVectorBaseToOrigin(baseLocalLinearVelocity, baseEulerAngles);

  /*
   * Base dynamics with the following assumptions:
   *  - Zero joint acceleration
   */
  stateDerivativeCoM.segment(6, 6) = comModel.calculateBaseLocalAccelerations(basePose, baseLocalVelocities, qJoints, qdJoints,
                                                                              joint_coordinate_s_t<SCALAR_T>::Zero(), extForce);
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
