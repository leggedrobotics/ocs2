/*
 * MRT_ROS_Quadruped.h
 *
 *  Created on: May 25, 2018
 *      Author: farbod
 */

#include <ocs2_robotic_tools/rbd_libraries/robcogen/iit/rbd/rbd.h>

#include "ocs2_anymal_switched_model/generated/transforms.h"
#include "ocs2_anymal_switched_model/generated/inertia_properties.h"
#include "ocs2_anymal_switched_model/generated/inverse_dynamics.h"
#include "ocs2_anymal_switched_model/generated/jsim.h"

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::MRT_ROS_Quadruped(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
                                                                             const std::string& robotName /*robot*/)

    : BASE(robotName), ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr), modelSettings_(ocs2QuadrupedInterfacePtr->modelSettings()) {
  // take a copy of the logic rules to make sure we don't share it with the MPC node
  logic_rules_mrt_.reset(new logic_rules_t(dynamic_cast<logic_rules_t&>(*ocs2QuadrupedInterfacePtr->getLogicRulesPtr())));

  // share logic rules with the Base
  BASE::setLogicRules(logic_rules_mrt_);

  // set up the rollout
  if (ocs2QuadrupedInterfacePtr->mpcSettings().useFeedbackPolicy_) {
    BASE::initRollout(ocs2QuadrupedInterfacePtr_->getDynamics(), ocs2QuadrupedInterfacePtr_->slqSettings().rolloutSettings_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::findsIndicesEventTimes(const scalar_array_t& eventTimes,
                                                                                       const scalar_array_t& timeTrajectory,
                                                                                       std::vector<int>& eventsIndices) const {
  // vector of (partition, index). -1 means end()
  eventsIndices.clear();
  eventsIndices.resize(eventTimes.size(), -1);

  for (size_t j = 0; j < eventTimes.size(); j++) {
    const scalar_t te = eventTimes[j];
    int& ie = eventsIndices[j];

    // skip if the controller is empty
    if (timeTrajectory.empty()) {
      continue;
    }

    // if not the first event, use the index of the previous event in order to be more efficient.
    // subjected to that they are in the same partition
    typename scalar_array_t::const_iterator beginItr = timeTrajectory.begin();
    if (j > 0) {
      beginItr += eventsIndices[j - 1];
    }

    auto lower = std::lower_bound(beginItr, timeTrajectory.end(), te);

    // if the lower bound found
    if (lower != timeTrajectory.end()) {
      ie = lower - timeTrajectory.begin();
    } else {
      break;
    }

  }  // end of j loop

  // no event at the final time
  for (int& ind : eventsIndices) {
    if (ind == timeTrajectory.size() - 1) {
      ind = -1;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::modifyBufferPolicy(const CommandData& commandBuffer, PolicyData& policyBuffer) {
  const auto& mpcInitObservationBuffer = commandBuffer.mpcInitObservation_;
  const auto& mpcTimeTrajectoryBuffer = policyBuffer.mpcTimeTrajectory_;
  const auto& mpcStateTrajectoryBuffer = policyBuffer.mpcStateTrajectory_;
  const auto& eventTimesBuffer = policyBuffer.eventTimes_;
  const auto& subsystemsSequenceBuffer = policyBuffer.subsystemsSequence_;
  auto& mpcControllerBuffer = policyBuffer.mpcController_;

  // only continue if we are using feedforward policy
  if (mpcControllerBuffer->getType() != ocs2::ControllerType::FEEDFORWARD) {
    return;
  }

  const size_t NE = subsystemsSequenceBuffer.size();
  touchdownTimeStockBuffer_.clear();
  touchdownTimeStockBuffer_.reserve(NE + 1);
  touchdownStateStockBuffer_.clear();
  touchdownStateStockBuffer_.reserve(NE + 1);
  touchdownInputStockBuffer_.clear();
  touchdownInputStockBuffer_.reserve(NE + 1);

  // for the first point
  scalar_t t_init = mpcTimeTrajectoryBuffer.front();
  state_vector_t x_init = mpcStateTrajectoryBuffer.front();
  touchdownTimeStockBuffer_.push_back(t_init);
  touchdownStateStockBuffer_.push_back(x_init);
  touchdownInputStockBuffer_.push_back(mpcControllerBuffer->computeInput(t_init, x_init));
  // making the reference and the measured EE velocity the same
  touchdownInputStockBuffer_.front().template segment<JOINT_COORD_SIZE>(12) =
      mpcInitObservationBuffer.input().template segment<JOINT_COORD_SIZE>(12);

  // find event indices
  std::vector<int> eventsIndices;
  findsIndicesEventTimes(eventTimesBuffer, mpcTimeTrajectoryBuffer, eventsIndices);

  // save touch-down information
  for (int eventsIndice : eventsIndices) {
    // skip if it is before the controller time
    if (eventsIndice == 0) {
      continue;
    }
    // stop if it is after the controller time
    if (eventsIndice == -1) {
      break;
    }

    const auto& t = mpcTimeTrajectoryBuffer[eventsIndice + 1];
    const auto& x = mpcStateTrajectoryBuffer[eventsIndice + 1];
    touchdownTimeStockBuffer_.push_back(t);
    touchdownStateStockBuffer_.push_back(x);
    touchdownInputStockBuffer_.push_back(mpcControllerBuffer->computeInput(t, x));
  }

  // for the last point
  const auto& t_final = mpcTimeTrajectoryBuffer.back();
  const auto& x_final = mpcStateTrajectoryBuffer.back();
  touchdownTimeStockBuffer_.push_back(t_final);
  touchdownStateStockBuffer_.push_back(x_final);
  touchdownInputStockBuffer_.push_back(mpcControllerBuffer->computeInput(t_final, x_final));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::modifyPolicy(const CommandData& command, PolicyData& policy) {
  const auto& mpcController = policy.mpcController_;
  const auto& eventTimes = policy.eventTimes_;
  const auto& subsystemsSequence = policy.subsystemsSequence_;

  // only continue if we are using feedforward policy
  if (mpcController->getType() != ocs2::ControllerType::FEEDFORWARD) {
    return;
  }

  // display
  if (ocs2QuadrupedInterfacePtr_->mpcSettings().debugPrint_) {
    std::cerr << "touchdownTimeStock: {";
    for (const auto& t : touchdownTimeStockBuffer_) {
      std::cerr << t << ", ";
    }
    std::cerr << "\b\b}" << std::endl;
    std::cerr << "touchdownTimeStock.size: " << touchdownTimeStockBuffer_.size() << std::endl;

    std::cerr << "eventTimes: {";
    for (const auto& t : eventTimes) {
      std::cerr << t << ", ";
    }
    std::cerr << "\b\b}" << std::endl;
    std::cerr << "eventTimes.size: " << eventTimes.size() << std::endl;

    std::cerr << "subsystemsSequence: {";
    for (const auto& t : subsystemsSequence) {
      std::cerr << t << ", ";
    }
    std::cerr << "\b\b}" << std::endl;
    std::cerr << "subsystemsSequence.size: " << subsystemsSequence.size() << std::endl;

    BASE::logicMachinePtr_->display();
  }

  // for feet trajectory filtering
  if (ocs2QuadrupedInterfacePtr_->modelSettings().useFeetTrajectoryFiltering_) {
    updateFeetTrajectories(eventTimes, subsystemsSequence, touchdownTimeStockBuffer_, touchdownStateStockBuffer_,
                           touchdownInputStockBuffer_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::updateFeetTrajectories(const scalar_array_t& eventTimes,
                                                                                       const size_array_t& subsystemsSequence,
                                                                                       const scalar_array_t& touchdownTimeStock,
                                                                                       const state_vector_array_t& touchdownStateStock,
                                                                                       const input_vector_array_t& touchdownInputStock) {
  // compute feet
  const size_t n = touchdownTimeStock.size();
  vector_3d_array_t o_contactForcesTemp;
  std::vector<vector_3d_array_t> o_feetPositionStock(n);
  std::vector<vector_3d_array_t> o_feetVelocityStock(n);
  for (size_t i = 0; i < n; i++) {
    computeFeetState(touchdownStateStock[i], touchdownInputStock[i], o_feetPositionStock[i], o_feetVelocityStock[i], o_contactForcesTemp);

    if (0 < i && i < n - 1) {
      o_feetVelocityStock[i].fill(vector_3d_t::Zero());
    }

  }  // end of i loop

  const size_t numPhaseSequence = subsystemsSequence.size();
  size_t initActiveSubsystem = BASE::findActiveSubsystemFnc_(touchdownTimeStock.front());
  size_t finalActiveSubsystem = BASE::findActiveSubsystemFnc_(touchdownTimeStock.back() - ocs2::OCS2NumericTraits<scalar_t>::weakEpsilon());

  // XY plan
  feetXPlanPtrStock_.resize(numPhaseSequence);
  feetYPlanPtrStock_.resize(numPhaseSequence);
  for (size_t i = 0; i < numPhaseSequence; i++) {
    if (i < initActiveSubsystem || finalActiveSubsystem < i) {
      continue;
    }

    const size_t startIndex = i - initActiveSubsystem;

    for (size_t j = 0; j < 4; j++) {
      feetXPlanPtrStock_[i][j] = std::make_shared<cubic_spline_t>();
      feetYPlanPtrStock_[i][j] = std::make_shared<cubic_spline_t>();

      feetXPlanPtrStock_[i][j]->set(touchdownTimeStock[startIndex] /*t0*/, o_feetPositionStock[startIndex][j](0) /*p0*/,
                                    o_feetVelocityStock[startIndex][j](0) /*v0*/, touchdownTimeStock[startIndex + 1] /*t1*/,
                                    o_feetPositionStock[startIndex + 1][j](0) /*p1*/, o_feetVelocityStock[startIndex + 1][j](0) /*v1*/);

      feetYPlanPtrStock_[i][j]->set(touchdownTimeStock[startIndex] /*t0*/, o_feetPositionStock[startIndex][j](1) /*p0*/,
                                    o_feetVelocityStock[startIndex][j](1) /*v0*/, touchdownTimeStock[startIndex + 1] /*t1*/,
                                    o_feetPositionStock[startIndex + 1][j](1) /*p1*/, o_feetVelocityStock[startIndex + 1][j](1) /*v1*/);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeFeetState(const state_vector_t& state, const input_vector_t& input,
                                                                                 vector_3d_array_t& o_feetPosition,
                                                                                 vector_3d_array_t& o_feetVelocity,
                                                                                 vector_3d_array_t& o_contactForces) {
  base_coordinate_t comPose = state.template head<6>();
  base_coordinate_t comLocalVelocities = state.template segment<6>(6);
  joint_coordinate_t qJoints = state.template segment<JOINT_COORD_SIZE>(12);
  joint_coordinate_t dqJoints = input.template segment<JOINT_COORD_SIZE>(12);

  base_coordinate_t basePose;
  ocs2QuadrupedInterfacePtr_->getComModel().calculateBasePose(qJoints, comPose, basePose);
  base_coordinate_t baseLocalVelocities;
  ocs2QuadrupedInterfacePtr_->getComModel().calculateBaseLocalVelocities(qJoints, dqJoints, comLocalVelocities, baseLocalVelocities);

  ocs2QuadrupedInterfacePtr_->getKinematicModel().update(basePose, qJoints);
  Eigen::Matrix3d o_R_b = ocs2QuadrupedInterfacePtr_->getKinematicModel().rotationMatrixOrigintoBase().transpose();

  for (size_t i = 0; i < 4; i++) {
    // calculates foot position in the base frame
    vector_3d_t b_footPosition;
    ocs2QuadrupedInterfacePtr_->getKinematicModel().footPositionBaseFrame(i, b_footPosition);

    // calculates foot position in the origin frame
    o_feetPosition[i] = o_R_b * b_footPosition + basePose.template tail<3>();

    // calculates foot velocity in the base frame
    Eigen::Matrix<scalar_t, 6, JOINT_COORD_SIZE> b_footJacobain;
    ocs2QuadrupedInterfacePtr_->getKinematicModel().footJacobainBaseFrame(i, b_footJacobain);
    vector_3d_t b_footVelocity = (b_footJacobain * dqJoints).template tail<3>();

    // calculates foot velocity in the origin frame
    ocs2QuadrupedInterfacePtr_->getKinematicModel().FromBaseVelocityToInertiaVelocity(o_R_b, baseLocalVelocities, b_footPosition,
                                                                                      b_footVelocity, o_feetVelocity[i]);

    // calculates contact forces in the origin frame
    o_contactForces[i] = o_R_b * input.template segment<3>(3 * i);

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::evaluatePolicy(
    const scalar_t& time, const state_vector_t& state, vector_3d_array_t& o_feetPositionRef, vector_3d_array_t& o_feetVelocityRef,
    vector_3d_array_t& o_feetAccelerationRef, base_coordinate_t& o_comPoseRef, base_coordinate_t& o_comVelocityRef,
    base_coordinate_t& o_comAccelerationRef, contact_flag_t& stanceLegs) {
  // optimal switched model state and input
  state_vector_t stateRef;
  input_vector_t inputRef;
  size_t subsystem;
  BASE::evaluatePolicy(time, state, stateRef, inputRef, subsystem);
  const auto index = logic_rules_mrt_->getEventTimeCount(time);
  logic_rules_mrt_->getMotionPhaseLogics(index, stanceLegs, feetZPlanPtr_);

  // computes swing phase progress
  computeSwingPhaseProgress(index, stanceLegs, time, swingPhaseProgress_);

  if (!ocs2QuadrupedInterfacePtr_->modelSettings().useFeetTrajectoryFiltering_) {
    // calculates nominal position, velocity, and contact forces of the feet in the origin frame.
    // This also updates the kinematic model.
    vector_3d_array_t o_contactForces;
    computeFeetState(stateRef, inputRef, o_feetPositionRef, o_feetVelocityRef, o_contactForces);

    // Look one dt ahead to obtain acceleration
    const scalar_t dt = 1.0 / ocs2QuadrupedInterfacePtr_->modelSettings().feetFilterFrequency_;
    state_vector_t stateRef_ahead;
    input_vector_t inputRef_ahead;
    size_t subsystem_ahead;
    BASE::evaluatePolicy(time + dt, state, stateRef_ahead, inputRef_ahead, subsystem_ahead);
    vector_3d_array_t o_feetPositionRef_ahead, o_feetVelocityRef_ahead;
    vector_3d_array_t o_contactForces_ahead;
    computeFeetState(stateRef_ahead, inputRef_ahead, o_feetPositionRef_ahead, o_feetVelocityRef_ahead, o_contactForces_ahead);

    for (size_t j = 0; j < 4; j++) {
      o_feetAccelerationRef[j] = (o_feetVelocityRef_ahead[j] - o_feetVelocityRef[j]) / dt;
    }

  } else {
    // filter swing leg trajectory
    for (size_t j = 0; j < 4; j++) {
      o_feetPositionRef[j] << feetXPlanPtrStock_[index][j]->evaluateSplinePosition(time),
          feetYPlanPtrStock_[index][j]->evaluateSplinePosition(time), feetZPlanPtr_[j]->calculatePosition(time);

      o_feetVelocityRef[j] << feetXPlanPtrStock_[index][j]->evaluateSplineVelocity(time),
          feetYPlanPtrStock_[index][j]->evaluateSplineVelocity(time), feetZPlanPtr_[j]->calculateVelocity(time);

      o_feetAccelerationRef[j] << feetXPlanPtrStock_[index][j]->evaluateSplineAcceleration(time),
          feetYPlanPtrStock_[index][j]->evaluateSplineAcceleration(time), feetZPlanPtr_[j]->calculateAcceleration(time);
    }
  }

  // calculate CoM pose, velocity, and acceleration in the origin frame.
  ocs2QuadrupedInterfacePtr_->computeComStateInOrigin(stateRef, inputRef, o_comPoseRef, o_comVelocityRef, o_comAccelerationRef);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::rolloutPolicy(
    const scalar_t& time, const state_vector_t& state, vector_3d_array_t& o_feetPositionRef, vector_3d_array_t& o_feetVelocityRef,
    vector_3d_array_t& o_feetAccelerationRef, base_coordinate_t& o_comPoseRef, base_coordinate_t& o_comVelocityRef,
    base_coordinate_t& o_comAccelerationRef, contact_flag_t& stanceLegs) {
  // optimal switched model state and input
  state_vector_t stateRef;
  input_vector_t inputRef;
  size_t subsystem;
  BASE::evaluatePolicy(time, state, stateRef, inputRef, subsystem);
  const auto index = logic_rules_mrt_->getEventTimeCount(time);
  logic_rules_mrt_->getMotionPhaseLogics(index, stanceLegs, feetZPlanPtr_);

  // computes swing phase progress
  computeSwingPhaseProgress(index, stanceLegs, time, swingPhaseProgress_);

  if (!ocs2QuadrupedInterfacePtr_->modelSettings().useFeetTrajectoryFiltering_) {
    // calculates nominal position, velocity, and contact forces of the feet in the origin frame.
    // This also updates the kinematic model.
    vector_3d_array_t o_contactForces;
    computeFeetState(stateRef, inputRef, o_feetPositionRef, o_feetVelocityRef, o_contactForces);

    // Look one dt ahead to obtain acceleration
    const scalar_t dt = 1.0 / ocs2QuadrupedInterfacePtr_->modelSettings().feetFilterFrequency_;
    state_vector_t stateRef_ahead;
    input_vector_t inputRef_ahead;
    size_t subsystem_ahead;
    BASE::rolloutPolicy(time, state, dt, stateRef_ahead, inputRef_ahead, subsystem_ahead);
    vector_3d_array_t o_feetPositionRef_ahead, o_feetVelocityRef_ahead;
    vector_3d_array_t o_contactForces_ahead;
    computeFeetState(stateRef_ahead, inputRef_ahead, o_feetPositionRef_ahead, o_feetVelocityRef_ahead, o_contactForces_ahead);

    for (size_t j = 0; j < 4; j++) {
      o_feetAccelerationRef[j] = (o_feetVelocityRef_ahead[j] - o_feetVelocityRef[j]) / dt;
    }

  } else {
    // filter swing leg trajectory
    for (size_t j = 0; j < 4; j++) {
      o_feetPositionRef[j] << feetXPlanPtrStock_[index][j]->evaluateSplinePosition(time),
          feetYPlanPtrStock_[index][j]->evaluateSplinePosition(time), feetZPlanPtr_[j]->calculatePosition(time);

      o_feetVelocityRef[j] << feetXPlanPtrStock_[index][j]->evaluateSplineVelocity(time),
          feetYPlanPtrStock_[index][j]->evaluateSplineVelocity(time), feetZPlanPtr_[j]->calculateVelocity(time);

      o_feetAccelerationRef[j] << feetXPlanPtrStock_[index][j]->evaluateSplineAcceleration(time),
          feetYPlanPtrStock_[index][j]->evaluateSplineAcceleration(time), feetZPlanPtr_[j]->calculateAcceleration(time);
    }
  }

  // calculate CoM pose, velocity, and acceleration in the origin frame.
  ocs2QuadrupedInterfacePtr_->computeComStateInOrigin(stateRef, inputRef, o_comPoseRef, o_comVelocityRef, o_comAccelerationRef);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
//void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::rolloutPolicy(
//	scalar_t time, const state_vector_t& state, rbd_state_vector_t& rbdState, joint_coordinate_t& rbdInput, size_t& subsystem) {
//
//  // optimal switched model state and input
//  state_vector_t stateRef;
//  input_vector_t inputRef;
//  BASE::evaluatePolicy(time, state, stateRef, inputRef, subsystem);
//
//  // calculate rbd state.
//  ocs2QuadrupedInterfacePtr_->computeRbdModelState(stateRef, inputRef, rbdState);
//  base_coordinate_t qBase     = rbdState.template segment<6>(0);
//  joint_coordinate_t qJoints  = rbdState.template segment<12>(6);
//  base_coordinate_t qdBase    = rbdState.template segment<6>(18);
//  joint_coordinate_t qdJoints = rbdState.template segment<12>(24);
//
//  // look one dt ahead to obtain acceleration
//  const scalar_t dt = 1.0 / ocs2QuadrupedInterfacePtr_->modelSettings().feetFilterFrequency_;
//  state_vector_t stateRef_ahead;
//  input_vector_t inputRef_ahead;
//  size_t subsystem_ahead;
//  BASE::rolloutPolicy(time, state, dt, stateRef_ahead, inputRef_ahead, subsystem_ahead);
//  vector_3d_array_t o_feetPositionRef_ahead, o_feetVelocityRef_ahead;
//  vector_3d_array_t o_contactForces_ahead;
//  joint_coordinate_t qddJoints = (inputRef_ahead.template tail<JOINT_COORD_SIZE>() - inputRef.template tail<JOINT_COORD_SIZE>()) / dt;
//
//  // inverse dynamics
//  // RBD homogeneous transforms of feet
//  ocs2QuadrupedInterfacePtr_->getKinematicModel().update(qBase, qJoints);
//  // force transformed in the lowerLeg coordinate
//  iit::rbd::ForceVector extForceBase[4];
//  for (size_t j=0; j<4; j++) {
//	  vector_3d_t b_footPosition;
//	  ocs2QuadrupedInterfacePtr_->getKinematicModel().footPositionBaseFrame(j, b_footPosition);
//	  extForceBase[j].head<3>() = b_footPosition.cross(inputRef.template segment<3>(3*j));
//	  extForceBase[j].tail<3>() = inputRef.template segment<3>(3*j);
//  }
//  iit::ANYmal::ForceTransforms forceTransforms;
//  iit::ANYmal::LinkDataMap<iit::rbd::ForceVector> extForces;
//  for (size_t j=0; j<iit::ANYmal::linksCount; j++)  {
//	  extForces[iit::ANYmal::orderedLinkIDs[j]] = iit::rbd::ForceVector::Zero();
//  }
//  extForces[iit::ANYmal::LF_SHANK] = forceTransforms.fr_LF_SHANK_COM_X_fr_base(qJoints) * extForceBase[0];
//  extForces[iit::ANYmal::RF_SHANK] = forceTransforms.fr_RF_SHANK_COM_X_fr_base(qJoints) * extForceBase[1];
//  extForces[iit::ANYmal::LH_SHANK] = forceTransforms.fr_LH_SHANK_COM_X_fr_base(qJoints) * extForceBase[2];
//  extForces[iit::ANYmal::RH_SHANK] = forceTransforms.fr_RH_SHANK_COM_X_fr_base(qJoints) * extForceBase[3];
//
//  // gravity vetor in the base frame
//  iit::rbd::Vector6D g;
//  Eigen::Matrix3d b_R_o = ocs2QuadrupedInterfacePtr_->getKinematicModel().rotationMatrixOrigintoBase();
//  g << vector_3d_t::Zero(), b_R_o*vector_3d_t(0.0,0.0,-9.81);
//
//  iit::ANYmal::dyn::InertiaProperties inertias;
//  iit::ANYmal::MotionTransforms transforms;
//  base_coordinate_t qddBase;
//  iit::ANYmal::dyn::InverseDynamics inverseDynamics(inertias, transforms);
//  inverseDynamics.id(rbdInput, qddBase, g, qdBase, qJoints, qdJoints, qddJoints, extForces);
//
//  for (size_t i=0; i<4; i++) {
//	  Eigen::Matrix<double,6,JOINT_COORD_SIZE> footJacobain;
//	  ocs2QuadrupedInterfacePtr_->getKinematicModel().footJacobainBaseFrame(i, footJacobain);
//	  joint_coordinate_t tau = footJacobain.template bottomRows<3>().transpose() * inputRef.template segment<3>(3*i);
//	  std::cout << "True torque " << i << ":      " << rbdInput.template segment<3>(3*i).transpose() << std::endl;
//	  std::cout << "Estimated torque " << i << ": " << tau.template segment<3>(3*i).transpose() << std::endl;
//	  std::cout << "extForceBase " << i << ": " << extForceBase[i].transpose() << std::endl;
//	  switch (i) {
//	  case 0: std::cout << "extForces " << i << ": " << extForces[iit::ANYmal::LF_SHANK].transpose() << std::endl << std::endl; break;
//	  case 1: std::cout << "extForces " << i << ": " << extForces[iit::ANYmal::RF_SHANK].transpose() << std::endl << std::endl; break;
//	  case 2: std::cout << "extForces " << i << ": " << extForces[iit::ANYmal::LH_SHANK].transpose() << std::endl << std::endl; break;
//	  case 3: std::cout << "extForces " << i << ": " << extForces[iit::ANYmal::RH_SHANK].transpose() << std::endl << std::endl; break;
//	  }
//  }
//}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::rolloutPolicy(
	scalar_t time, const state_vector_t& state, rbd_state_vector_t& rbdState, joint_coordinate_t& rbdInput, size_t& subsystem) {

  // optimal switched model state and input
  state_vector_t stateRef;
  input_vector_t inputRef;
  BASE::evaluatePolicy(time, state, stateRef, inputRef, subsystem);

  // calculate rbd state.
  ocs2QuadrupedInterfacePtr_->computeRbdModelState(stateRef, inputRef, rbdState);
  base_coordinate_t qBase     = rbdState.template segment<6>(0);
  joint_coordinate_t qJoints  = rbdState.template segment<12>(6);
  base_coordinate_t qdBase    = rbdState.template segment<6>(18);
  joint_coordinate_t qdJoints = rbdState.template segment<12>(24);

  // look one dt ahead to obtain acceleration
  const scalar_t dt = 1.0 / ocs2QuadrupedInterfacePtr_->modelSettings().feetFilterFrequency_;
  state_vector_t stateRef_ahead;
  input_vector_t inputRef_ahead;
  size_t subsystem_ahead;
  BASE::rolloutPolicy(time, state, dt, stateRef_ahead, inputRef_ahead, subsystem_ahead);
  vector_3d_array_t o_feetPositionRef_ahead, o_feetVelocityRef_ahead;
  vector_3d_array_t o_contactForces_ahead;
  joint_coordinate_t qddJoints = (inputRef_ahead.template tail<JOINT_COORD_SIZE>() - inputRef.template tail<JOINT_COORD_SIZE>()) / dt;

  // inverse dynamics
  // RBD homogeneous transforms of feet
  ocs2QuadrupedInterfacePtr_->getKinematicModel().update(qBase, qJoints);
  // force transformed in the lowerLeg coordinate
  iit::rbd::ForceVector extForceBase[4];
  for (size_t j=0; j<4; j++) {
	  vector_3d_t b_footPosition;
	  ocs2QuadrupedInterfacePtr_->getKinematicModel().footPositionBaseFrame(j, b_footPosition);
	  extForceBase[j].head<3>() = b_footPosition.cross(inputRef.template segment<3>(3*j));
	  extForceBase[j].tail<3>() = inputRef.template segment<3>(3*j);
  }
  joint_coordinate_t extForceJoint[4];
  for (size_t j=0; j<4; j++) {
	  Eigen::Matrix<double,6,JOINT_COORD_SIZE> b_footJacobain;
	  ocs2QuadrupedInterfacePtr_->getKinematicModel().footJacobainBaseFrame(j, b_footJacobain);
	  extForceJoint[j] = b_footJacobain.template bottomRows<3>().transpose() * inputRef.template segment<3>(3*j);
  }

  iit::ANYmal::dyn::InertiaProperties inertias;
  iit::ANYmal::MotionTransforms MotionTransforms;
  iit::ANYmal::ForceTransforms forceTransforms;

  iit::ANYmal::dyn::JSIM Mm(inertias, forceTransforms);
  Mm.update(qJoints);

  iit::ANYmal::dyn::InverseDynamics inverseDynamics(inertias, MotionTransforms);
  inverseDynamics.setJointStatus(qJoints);

  generalized_coordinate_t Gv;
  {
	  // gravity vetor in the base frame
	  iit::rbd::Vector6D gravity;
	  Eigen::Matrix3d b_R_o = ocs2QuadrupedInterfacePtr_->getKinematicModel().rotationMatrixOrigintoBase();
	  gravity << vector_3d_t::Zero(), b_R_o*vector_3d_t(0.0,0.0,-9.81);

	  iit::rbd::Vector6D baseWrench;
	  joint_coordinate_t jForces;
	  inverseDynamics.G_terms_fully_actuated(baseWrench, jForces, gravity);
	  Gv << baseWrench, jForces;
  }

  generalized_coordinate_t Cv;
  {
	  iit::rbd::Vector6D baseWrench;
	  joint_coordinate_t jForces;
	  inverseDynamics.C_terms_fully_actuated(baseWrench, jForces, qdBase, qdJoints);
	  Cv << baseWrench, jForces;
  }

  // compute Base local acceleration about Base frame
  base_coordinate_t comLocalAcceleration = -Mm. template topRightCorner<6,JOINT_COORD_SIZE>()*qddJoints
		  - Cv.template head<6>() - Gv.template head<6>();
  for (size_t j=0; j<4; j++) { comLocalAcceleration += extForceBase[j]; }
  comLocalAcceleration = Mm. template topLeftCorner<6,6>().ldlt().solve(comLocalAcceleration);

  // compute joint torque
  rbdInput = Mm. template bottomLeftCorner<JOINT_COORD_SIZE,6>()*comLocalAcceleration
		  + Mm. template bottomRightCorner<JOINT_COORD_SIZE,JOINT_COORD_SIZE>()*qddJoints
		  + Cv.template tail<JOINT_COORD_SIZE>() + Gv.template tail<JOINT_COORD_SIZE>();
  for (size_t j=0; j<4; j++) { rbdInput -= extForceJoint[j]; }

//  for (size_t i=0; i<4; i++) {
//	  Eigen::Matrix<double,6,JOINT_COORD_SIZE> footJacobain;
//	  ocs2QuadrupedInterfacePtr_->getKinematicModel().footJacobainBaseFrame(i, footJacobain);
//	  joint_coordinate_t tau = footJacobain.template bottomRows<3>().transpose() * inputRef.template segment<3>(3*i);
//	  std::cout << "True torque " << i << ":      " << rbdInput.template segment<3>(3*i).transpose() << std::endl;
//	  std::cout << "Estimated torque " << i << ": " << -tau.template segment<3>(3*i).transpose() << std::endl;
//  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
const typename MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::scalar_t&
MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getsSwingPhaseProgress(const size_t& legIndex) const {
  return swingPhaseProgress_.at(legIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeSwingPhaseProgress(
    const size_t& activeSubsystemIndex, const contact_flag_t& stanceLegs, const scalar_t& time,
    std::array<scalar_t, 4>& swingPhaseProgress) const {
  using int_array_t = std::vector<int>;

  const scalar_array_t& eventTimes = logic_rules_mrt_->eventTimes();

  std::array<int_array_t, 4> startTimesIndices;
  logic_rules_mrt_->getFeetPlanner().getStartTimesIndices(startTimesIndices);

  std::array<int_array_t, 4> finalTimesIndices;
  logic_rules_mrt_->getFeetPlanner().getFinalTimesIndices(finalTimesIndices);

  // for each leg
  for (size_t j = 0; j < 4; j++) {
    const auto swingStartIndex = startTimesIndices[j][activeSubsystemIndex];
    const auto swingFinalIndex = finalTimesIndices[j][activeSubsystemIndex];

    if (swingStartIndex == -1) {
      throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(j) + " is not defined.");
    }
    if (swingFinalIndex == eventTimes.size()) {
      throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(j) + " is not defined.");
    }

    const auto swingStartTime = eventTimes[swingStartIndex];
    const auto swingFinalTime = eventTimes[swingFinalIndex];

    if (stanceLegs[j] == 0) {
      swingPhaseProgress[j] = (time - swingStartTime) / (swingFinalTime - swingStartTime);
    } else {
      swingPhaseProgress[j] = 0.0;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
bool MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::updateNodes(const contact_flag_t& contactFlag, const scalar_t& time,
                                                                            const rbd_state_vector_t& rbdState) {
  // check callback queue
  ::ros::spinOnce();

  system_observation_t currentObservation;
  // time
  currentObservation.time() = time;
  // state
  ocs2QuadrupedInterfacePtr_->computeSwitchedModelState(rbdState, currentObservation.state());
  // input
  currentObservation.input().template segment<12>(0).setZero();
  currentObservation.input().template segment<JOINT_COORD_SIZE>(12) = rbdState.template tail<JOINT_COORD_SIZE>();
  // mode
  currentObservation.subsystem() = switched_model::stanceLeg2ModeNumber(contactFlag);

  // publish the current observation
  BASE::setCurrentObservation(currentObservation);

  // check for a new controller update from the MPC node
  bool policyUpdated = BASE::updatePolicy();

  return policyUpdated;
}

}  // end of namespace switched_model
