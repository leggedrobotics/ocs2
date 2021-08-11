//
// Created by rgrandia on 30.04.21.
//

#include "ocs2_switched_model_interface/cost/MotionTrackingCost.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace switched_model {

namespace {
constexpr size_t baseTargets = 12;
constexpr size_t legTargets = 15;
constexpr size_t costVectorLength = baseTargets + NUM_CONTACT_POINTS * legTargets;

template <typename SCALAR_T>
struct CostElements {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_s_t<SCALAR_T> eulerXYZ{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comPosition{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comAngularVelocity{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comLinearVelocity{vector3_s_t<SCALAR_T>::Zero()};
  feet_array_t<vector3_s_t<SCALAR_T>> jointPosition{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> footPosition{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> jointVelocity{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> footVelocity{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> contactForce{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
};

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, -1, 1> costElementsToVector(const CostElements<SCALAR_T>& asStruct) {
  Eigen::Matrix<SCALAR_T, -1, 1> v(costVectorLength);

  // Base
  v.head(baseTargets) << asStruct.eulerXYZ, asStruct.comPosition, asStruct.comAngularVelocity, asStruct.comLinearVelocity;

  // Legs
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    v.segment(baseTargets + leg * legTargets, legTargets) << asStruct.jointPosition[leg], asStruct.footPosition[leg],
        asStruct.jointVelocity[leg], asStruct.footVelocity[leg], asStruct.contactForce[leg];
  }
  return v;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, -1, 1> computeMotionTargets(const comkino_state_s_t<SCALAR_T>& x, const comkino_input_s_t<SCALAR_T>& u,
                                                    const KinematicsModelBase<SCALAR_T>& kinematics,
                                                    const ComModelBase<SCALAR_T>& comModel) {
  // Extract elements from reference
  const auto comPose = getComPose(x);
  const auto basePose = comModel.calculateBasePose(comPose);
  const auto com_comTwist = getComLocalVelocities(x);
  const auto base_baseTwist = comModel.calculateBaseLocalVelocities(com_comTwist);
  const auto eulerAngles = getOrientation(comPose);
  const auto qJoints = getJointPositions(x);
  const auto dqJoints = getJointVelocities(u);

  CostElements<SCALAR_T> motionTarget;
  motionTarget.eulerXYZ = eulerAngles;
  motionTarget.comPosition = getPositionInOrigin(comPose);
  motionTarget.comAngularVelocity = rotateVectorBaseToOrigin(getAngularVelocity(com_comTwist), eulerAngles);
  motionTarget.comLinearVelocity = rotateVectorBaseToOrigin(getLinearVelocity(com_comTwist), eulerAngles);
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    motionTarget.jointPosition[leg] = qJoints.template segment<3>(3 * leg);
    motionTarget.footPosition[leg] = kinematics.footPositionInOriginFrame(leg, basePose, qJoints);
    motionTarget.jointVelocity[leg] = dqJoints.template segment<3>(3 * leg);
    motionTarget.footVelocity[leg] = kinematics.footVelocityInOriginFrame(leg, basePose, base_baseTwist, qJoints, dqJoints);
    motionTarget.contactForce[leg] = u.template segment<3>(3 * leg);
  }
  return costElementsToVector(motionTarget);
}

vector_t computeMotionReferences(scalar_t time, const comkino_state_t& x, const comkino_input_t& u,
                                 const SwingTrajectoryPlanner& swingTrajectoryPlanner) {
  // Extract elements from reference
  const auto comPose = getComPose(x);
  const auto com_comTwist = getComLocalVelocities(x);
  const auto eulerAngles = getOrientation(comPose);
  const auto qJoints = getJointPositions(x);
  const auto dqJoints = getJointVelocities(u);

  CostElements<scalar_t> motionTarget;
  motionTarget.eulerXYZ = eulerAngles;
  motionTarget.comPosition = getPositionInOrigin(comPose);
  motionTarget.comAngularVelocity = rotateVectorBaseToOrigin(getAngularVelocity(com_comTwist), eulerAngles);
  motionTarget.comLinearVelocity = rotateVectorBaseToOrigin(getLinearVelocity(com_comTwist), eulerAngles);
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto& footPhase = swingTrajectoryPlanner.getFootPhase(leg, time);
    motionTarget.jointPosition[leg] = qJoints.template segment<3>(3 * leg);
    motionTarget.footPosition[leg] = footPhase.getPositionInWorld(time);
    motionTarget.jointVelocity[leg] = dqJoints.template segment<3>(3 * leg);
    motionTarget.footVelocity[leg] = footPhase.getVelocityInWorld(time);
    motionTarget.contactForce[leg] = u.template segment<3>(3 * leg);
  }
  return costElementsToVector(motionTarget);
}

}  // namespace

MotionTrackingCost::MotionTrackingCost(const Weights& settings, const SwitchedModelModeScheduleManager& modeScheduleManager,
                                       const SwingTrajectoryPlanner& swingTrajectoryPlanner, const kinematic_model_t& kinematicModel,
                                       const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                       const ad_com_model_t& adComModel, bool recompile)
    : modeScheduleManagerPtr_(&modeScheduleManager),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      kinematicModelPtr_(kinematicModel.clone()),
      adKinematicModelPtr_(adKinematicModel.clone()),
      comModelPtr_(comModel.clone()),
      adComModelPtr_(adComModel.clone()) {
  // Weights are sqrt of settings
  CostElements<ocs2::ad_scalar_t> weightStruct;
  weightStruct.eulerXYZ = settings.eulerXYZ.cwiseSqrt().cast<ocs2::ad_scalar_t>();
  weightStruct.comPosition = settings.comPosition.cwiseSqrt().cast<ocs2::ad_scalar_t>();
  weightStruct.comAngularVelocity = settings.comAngularVelocity.cwiseSqrt().cast<ocs2::ad_scalar_t>();
  weightStruct.comLinearVelocity = settings.comLinearVelocity.cwiseSqrt().cast<ocs2::ad_scalar_t>();
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    weightStruct.jointPosition[leg] = settings.jointPosition.cwiseSqrt().cast<ocs2::ad_scalar_t>();
    weightStruct.footPosition[leg] = settings.footPosition.cwiseSqrt().cast<ocs2::ad_scalar_t>();
    weightStruct.jointVelocity[leg] = settings.jointVelocity.cwiseSqrt().cast<ocs2::ad_scalar_t>();
    weightStruct.footVelocity[leg] = settings.footVelocity.cwiseSqrt().cast<ocs2::ad_scalar_t>();
    weightStruct.contactForce[leg] = settings.contactForce.cwiseSqrt().cast<ocs2::ad_scalar_t>();
  }
  sqrtWeights_ = costElementsToVector(weightStruct);

  initialize(STATE_DIM, INPUT_DIM, costVectorLength, "MotionTrackingCost", "/tmp/ocs2", recompile);
};

ocs2::vector_t MotionTrackingCost::getParameters(ocs2::scalar_t time, const ocs2::TargetTrajectories& targetTrajectories) const {
  // Interpolate reference
  const comkino_state_t xRef = targetTrajectories.getDesiredState(time);
  comkino_input_t uRef = targetTrajectories.getDesiredInput(time);

  // If the input has zero values, overwrite it.
  if (uRef.isZero()) {
    // Get stance configuration
    const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(time);
    uRef = weightCompensatingInputs(*comModelPtr_, contactFlags, getOrientation(getComPose(xRef)));
  }

  return computeMotionReferences(time, xRef, uRef, *swingTrajectoryPlannerPtr_);
}

MotionTrackingCost::MotionTrackingCost(const MotionTrackingCost& other)
    : ocs2::StateInputCostGaussNewtonAd(other),
      sqrtWeights_(other.sqrtWeights_),
      modeScheduleManagerPtr_(other.modeScheduleManagerPtr_),
      swingTrajectoryPlannerPtr_(other.swingTrajectoryPlannerPtr_),
      kinematicModelPtr_(other.kinematicModelPtr_->clone()),
      adKinematicModelPtr_(other.adKinematicModelPtr_->clone()),
      comModelPtr_(other.comModelPtr_->clone()),
      adComModelPtr_(other.adComModelPtr_->clone()) {}

ocs2::ad_vector_t MotionTrackingCost::costVectorFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                                         const ocs2::ad_vector_t& input, const ocs2::ad_vector_t& parameters) const {
  const auto currentTargets = computeMotionTargets<ocs2::ad_scalar_t>(state, input, *adKinematicModelPtr_, *adComModelPtr_);
  return (currentTargets - parameters).cwiseProduct(sqrtWeights_);
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
  ocs2::loadData::loadPtreeValue(pt, weights.jointPosition.x(), fieldname + ".joint_position_HAA", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.jointPosition.y(), fieldname + ".joint_position_HFE", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.jointPosition.z(), fieldname + ".joint_position_KFE", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.footPosition.x(), fieldname + ".foot_position_x", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.footPosition.y(), fieldname + ".foot_position_y", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.footPosition.z(), fieldname + ".foot_position_z", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.jointVelocity.x(), fieldname + ".joint_velocity_HAA", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.jointVelocity.y(), fieldname + ".joint_velocity_HFE", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.jointVelocity.z(), fieldname + ".joint_velocity_KFE", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.footVelocity.x(), fieldname + ".foot_velocity_x", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.footVelocity.y(), fieldname + ".foot_velocity_y", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.footVelocity.z(), fieldname + ".foot_velocity_z", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.contactForce.x(), fieldname + ".contact_force_x", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.contactForce.y(), fieldname + ".contact_force_y", verbose);
  ocs2::loadData::loadPtreeValue(pt, weights.contactForce.z(), fieldname + ".contact_force_z", verbose);

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return weights;
}

}  // namespace switched_model
