//
// Created by rgrandia on 30.04.21.
//

#include "ocs2_switched_model_interface/cost/MotionTrackingCost.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_switched_model_interface/core/Rotations.h"
#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/cost/CostElements.h"

namespace switched_model {

namespace {

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, -1, 1> computeMotionTargets(const comkino_state_s_t<SCALAR_T>& x, const comkino_input_s_t<SCALAR_T>& u,
                                                    const KinematicsModelBase<SCALAR_T>& kinematics) {
  // Extract elements from reference
  const auto basePose = getBasePose(x);
  const auto base_baseTwist = getBaseLocalVelocities(x);
  const auto eulerAngles = getOrientation(basePose);
  const auto qJoints = getJointPositions(x);
  const auto dqJoints = getJointVelocities(u);

  CostElements<SCALAR_T> motionTarget;
  motionTarget.eulerXYZ = eulerAngles;
  motionTarget.comPosition = getPositionInOrigin(basePose);
  motionTarget.comAngularVelocity = rotateVectorBaseToOrigin(getAngularVelocity(base_baseTwist), eulerAngles);
  motionTarget.comLinearVelocity = rotateVectorBaseToOrigin(getLinearVelocity(base_baseTwist), eulerAngles);
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    motionTarget.jointPosition[leg] = qJoints.template segment<3>(3 * leg);
    motionTarget.footPosition[leg] = kinematics.footPositionInOriginFrame(leg, basePose, qJoints);
    motionTarget.jointVelocity[leg] = dqJoints.template segment<3>(3 * leg);
    motionTarget.footVelocity[leg] = kinematics.footVelocityInOriginFrame(leg, basePose, base_baseTwist, qJoints, dqJoints);
    motionTarget.contactForce[leg] = u.template segment<3>(3 * leg);
  }
  return motionTarget.asVector();
}

}  // namespace

MotionTrackingCost::MotionTrackingCost(const Weights& settings, const ad_kinematic_model_t& adKinematicModel,
                                       const ModelSettings& modelSettings)
    : adKinematicModelPtr_(adKinematicModel.clone()), modelSettings_(modelSettings) {
  // Weights are sqrt of settings
  CostElements<ocs2::scalar_t> weightStruct;
  weightStruct.eulerXYZ = settings.eulerXYZ.cwiseSqrt();
  weightStruct.comPosition = settings.comPosition.cwiseSqrt();
  weightStruct.comAngularVelocity = settings.comAngularVelocity.cwiseSqrt();
  weightStruct.comLinearVelocity = settings.comLinearVelocity.cwiseSqrt();
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    weightStruct.jointPosition[leg] = settings.jointPosition[leg].cwiseSqrt();
    weightStruct.footPosition[leg] = settings.footPosition[leg].cwiseSqrt();
    weightStruct.jointVelocity[leg] = settings.jointVelocity[leg].cwiseSqrt();
    weightStruct.footVelocity[leg] = settings.footVelocity[leg].cwiseSqrt();
    weightStruct.contactForce[leg] = settings.contactForce[leg].cwiseSqrt();
  }
  sqrtWeights_ = weightStruct.asVector();

  initialize(STATE_DIM, INPUT_DIM, CostElements<ocs2::scalar_t>::Size() + sqrtWeights_.size(),
             modelSettings_.robotName_ + "_MotionTrackingCost", modelSettings_.autodiffLibraryFolder_, modelSettings_.recompileLibraries_);
};

ocs2::vector_t MotionTrackingCost::getParameters(ocs2::scalar_t time, const ocs2::TargetTrajectories& targetTrajectories,
                                                 const ocs2::PreComputation& preComputation) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComputation);

  ocs2::vector_t parameters(CostElements<ocs2::scalar_t>::Size() + sqrtWeights_.size());
  parameters << switchedModelPreComp.getMotionReference().asVector(), sqrtWeights_;
  return parameters;
}

MotionTrackingCost::MotionTrackingCost(const MotionTrackingCost& other)
    : ocs2::StateInputCostGaussNewtonAd(other),
      adKinematicModelPtr_(other.adKinematicModelPtr_->clone()),
      sqrtWeights_(other.sqrtWeights_),
      modelSettings_(other.modelSettings_) {}

ocs2::ad_vector_t MotionTrackingCost::costVectorFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                                         const ocs2::ad_vector_t& input, const ocs2::ad_vector_t& parameters) const {
  auto referenceTargets = parameters.head(CostElements<ocs2::scalar_t>::Size());
  auto sqrtWeights = parameters.tail(sqrtWeights_.size());

  const auto currentTargets = computeMotionTargets<ocs2::ad_scalar_t>(state, input, *adKinematicModelPtr_);
  ocs2::ad_vector_t errors = (currentTargets - referenceTargets).cwiseProduct(sqrtWeights);

  // Rotation error, expressed in base frame
  auto rotationErrorInBase = rotationErrorInLocalEulerXYZ<ocs2::ad_scalar_t>(currentTargets.head<3>(), parameters.head<3>());

  // For the orientation, we replace the error in Euler angles coordinates with a proper rotation error.
  errors.head<3>() = rotationErrorInBase.cwiseProduct(sqrtWeights.head<3>());
  return errors;
}

MotionTrackingCost::Weights loadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose) {
  MotionTrackingCost::Weights weights;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### Tacking Cost Weights:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  ocs2::loadData::loadPtreeValue(pt, weights.eulerXYZ.x(), fieldname + ".roll", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.eulerXYZ.y(), fieldname + ".pitch", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.eulerXYZ.z(), fieldname + ".yaw", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comPosition.x(), fieldname + ".base_position_x", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comPosition.y(), fieldname + ".base_position_y", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comPosition.z(), fieldname + ".base_position_z", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comAngularVelocity.x(), fieldname + ".base_angular_vel_x", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comAngularVelocity.y(), fieldname + ".base_angular_vel_y", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comAngularVelocity.z(), fieldname + ".base_angular_vel_z", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comLinearVelocity.x(), fieldname + ".base_linear_vel_x", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comLinearVelocity.y(), fieldname + ".base_linear_vel_y", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.comLinearVelocity.z(), fieldname + ".base_linear_vel_z", verbose);
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    bool legVerbose = verbose && (leg == 0);
    ocs2::loadData::loadPtreeValue(pt, weights.jointPosition[leg].x(), fieldname + ".joint_position_HAA", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.jointPosition[leg].y(), fieldname + ".joint_position_HFE", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.jointPosition[leg].z(), fieldname + ".joint_position_KFE", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.footPosition[leg].x(), fieldname + ".foot_position_x", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.footPosition[leg].y(), fieldname + ".foot_position_y", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.footPosition[leg].z(), fieldname + ".foot_position_z", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.jointVelocity[leg].x(), fieldname + ".joint_velocity_HAA", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.jointVelocity[leg].y(), fieldname + ".joint_velocity_HFE", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.jointVelocity[leg].z(), fieldname + ".joint_velocity_KFE", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.footVelocity[leg].x(), fieldname + ".foot_velocity_x", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.footVelocity[leg].y(), fieldname + ".foot_velocity_y", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.footVelocity[leg].z(), fieldname + ".foot_velocity_z", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.contactForce[leg].x(), fieldname + ".contact_force_x", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.contactForce[leg].y(), fieldname + ".contact_force_y", legVerbose);
    ocs2::loadData::loadPtreeValue(pt, weights.contactForce[leg].z(), fieldname + ".contact_force_z", legVerbose);
  }

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return weights;
}

}  // namespace switched_model
