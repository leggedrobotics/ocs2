/*
 * MRT_ROS_Quadruped.h
 *
 *  Created on: May 25, 2018
 *      Author: farbod
 */

#include "ocs2_quadruped_interface/MRT_ROS_Quadruped.h"

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::MRT_ROS_Quadruped(
    std::shared_ptr<quadruped_interface_t> ocs2QuadrupedInterfacePtr, std::string robotName)

    : BASE(std::move(robotName)),
      ocs2QuadrupedInterfacePtr_(std::move(ocs2QuadrupedInterfacePtr)),
      switchedModelStateEstimator_(ocs2QuadrupedInterfacePtr_->getComModel()),
      modelSettings_(ocs2QuadrupedInterfacePtr_->modelSettings()) {
  // take a copy of the logic rules to make sure we don't share it with the MPC node
  logic_rules_mrt_.reset(new logic_rules_t(dynamic_cast<logic_rules_t&>(*ocs2QuadrupedInterfacePtr_->getLogicRulesPtr())));

  // share logic rules with the Base
  BASE::setLogicRules(logic_rules_mrt_);

  // set up the rollout
  BASE::initRollout(&ocs2QuadrupedInterfacePtr_->getRollout());
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
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::modifyBufferPolicy(const command_data_t& commandBuffer,
                                                                                   primal_solution_t& primalSolutionBuffer) {
  const auto& mpcTimeTrajectoryBuffer = primalSolutionBuffer.timeTrajectory_;
  const auto& mpcStateTrajectoryBuffer = primalSolutionBuffer.stateTrajectory_;
  const auto& eventTimesBuffer = primalSolutionBuffer.eventTimes_;
  auto& mpcControllerBuffer = primalSolutionBuffer.controllerPtr_;

  // only continue if we are using feedforward primalSolution
  if (mpcControllerBuffer->getType() != ocs2::ControllerType::FEEDFORWARD) {
    return;
  }

  const size_t NE = primalSolutionBuffer.subsystemsSequence_.size();
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
      commandBuffer.mpcInitObservation_.input().template segment<JOINT_COORD_SIZE>(12);

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
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::modifyPolicy(const command_data_t& command,
                                                                             primal_solution_t& primalSolution) {
  const auto& mpcController = primalSolution.controllerPtr_;
  const auto& eventTimes = primalSolution.eventTimes_;
  const auto& subsystemsSequence = primalSolution.subsystemsSequence_;

  // only continue if we are using feedforward primalSolution
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
  base_coordinate_t comPose = getComPose(state);
  base_coordinate_t comLocalVelocities = getComLocalVelocities(state);
  joint_coordinate_t qJoints = getJointPositions(state);
  joint_coordinate_t dqJoints = getJointVelocities(input);

  base_coordinate_t basePose = ocs2QuadrupedInterfacePtr_->getComModel().calculateBasePose(comPose);
  base_coordinate_t baseLocalVelocities = ocs2QuadrupedInterfacePtr_->getComModel().calculateBaseLocalVelocities(comLocalVelocities);

  Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin<scalar_t>(state.template head<3>());

  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    o_feetPosition[i] = ocs2QuadrupedInterfacePtr_->getKinematicModel().footPositionInOriginFrame(i, basePose, qJoints);
    o_feetVelocity[i] =
        ocs2QuadrupedInterfacePtr_->getKinematicModel().footVelocityInOriginFrame(i, basePose, baseLocalVelocities, qJoints, dqJoints);
    o_contactForces[i] = o_R_b * input.template segment<3>(3 * i);
  }
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
      if (subsystem_ahead == subsystem) {
        o_feetAccelerationRef[j] = (o_feetVelocityRef_ahead[j] - o_feetVelocityRef[j]) / dt;
      } else {  // When subsystem changes jumps occur in the velocity reference.
        o_feetAccelerationRef[j].setZero();
      }
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

  // calculate CoM velocity in CoM frame.
  const auto comLocalVelocities = getComLocalVelocities(stateRef);

  // compute CoM local acceleration about CoM frame
  base_coordinate_t comLocalAcceleration;
  com_state_t comStateDerivative = ComKinoSystemDynamicsAd::computeComStateDerivative(
      ocs2QuadrupedInterfacePtr_->getComModel(), ocs2QuadrupedInterfacePtr_->getKinematicModel(),
      stateRef.template head<switched_model::STATE_DIM>(), inputRef.template head<switched_model::INPUT_DIM>());
  // CoM acceleration about CoM frame
  comLocalAcceleration = comStateDerivative.template tail<6>();

  // CoM pose in the origin frame
  o_comPoseRef = getComPose(stateRef);

  // Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
  Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin<scalar_t>(getOrientation(o_comPoseRef));

  // CoM velocity in the origin frame
  o_comVelocityRef.template head<3>() = o_R_b * getAngularVelocity(comLocalVelocities);
  o_comVelocityRef.template tail<3>() = o_R_b * getLinearVelocity(comLocalVelocities);

  // CoM acceleration in the origin frame
  o_comAccelerationRef.template head<3>() = o_R_b * comLocalAcceleration.template head<3>();
  o_comAccelerationRef.template tail<3>() = o_R_b * comLocalAcceleration.template tail<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::rolloutPolicy(
    const scalar_t& time, const state_vector_t& state, vector_3d_array_t& o_feetPositionRef, vector_3d_array_t& o_feetVelocityRef,
    vector_3d_array_t& o_feetAccelerationRef, base_coordinate_t& o_comPoseRef, base_coordinate_t& o_comVelocityRef,
    base_coordinate_t& o_comAccelerationRef, contact_flag_t& stanceLegs) {
  // FIX ME: For some reason the implementation was a 1-1 copy of evaluate policy. Removed code duplication for now, but there should be a
  // difference.
  evaluatePolicy(time, state, o_feetPositionRef, o_feetVelocityRef, o_feetAccelerationRef, o_comPoseRef, o_comVelocityRef,
                 o_comAccelerationRef, stanceLegs);
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

    if (!stanceLegs[j]) {
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
  currentObservation.state() = switchedModelStateEstimator_.estimateComkinoModelState(rbdState);
  // input
  currentObservation.input().template segment<12>(0).setZero();
  currentObservation.input().template segment<JOINT_COORD_SIZE>(12) = getJointVelocities(rbdState);
  // mode
  currentObservation.subsystem() = switched_model::stanceLeg2ModeNumber(contactFlag);

  // publish the current observation
  BASE::setCurrentObservation(currentObservation);

  // check for a new controller update from the MPC node
  bool policyUpdated = BASE::updatePolicy();

  return policyUpdated;
}

}  // end of namespace switched_model
