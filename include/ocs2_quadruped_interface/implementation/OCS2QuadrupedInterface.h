/*
 * OCS2QuadrupedInterface.h
 *
 *  Created on: Feb 14, 2018
 *      Author: farbod
 */

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"
#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::OCS2QuadrupedInterface(const kinematic_model_t& kinematicModel,
                                                                                       const com_model_t& comModel,
                                                                                       const std::string& pathToConfigFolder)

    : kinematicModelPtr_(kinematicModel.clone()), comModelPtr_(comModel.clone()), switchedModelStateEstimator_(comModel) {
  // load setting from loading file
  std::string pathToConfigFile = pathToConfigFolder + "/task.info";
  loadSettings(pathToConfigFile);

  // logic rule
  feet_z_planner_ptr_t feetZPlannerPtr(new feet_z_planner_t(modelSettings_.swingLegLiftOff_, 1.0 /*swingTimeScale*/,
                                                            modelSettings_.liftOffVelocity_, modelSettings_.touchDownVelocity_));

  logicRulesPtr_ = logic_rules_ptr_t(new logic_rules_t(feetZPlannerPtr, modelSettings_.phaseTransitionStanceTime_));

  logicRulesPtr_->setMotionConstraints(initSwitchingModes_, initEventTimes_, gapIndicatorPtrs_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::loadSettings(const std::string& pathToConfigFile) {
  // load SLQ settings
  slqSettings_.loadSettings(pathToConfigFile, "slq", true);

  // load MPC settings
  BASE::mpcSettings().loadSettings(pathToConfigFile, true);

  // load switched model settings
  modelSettings_.loadSettings(pathToConfigFile, true);

  std::cerr << std::endl;

  // partitioning times
  BASE::definePartitioningTimes(pathToConfigFile, timeHorizon_, numPartitions_, partitioningTimes_, true);
  // display
  std::cerr << "Time Partition: {";
  for (const auto& timePartition : partitioningTimes_) std::cerr << timePartition << ", ";
  std::cerr << "\b\b}" << std::endl;

  initTime_ = 0.0;
  finalTime_ = timeHorizon_;

  // initial state of the switched system
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "initialRobotState", initRbdState_);
  computeSwitchedModelState(initRbdState_, initialState_);

  // cost function components
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q", Q_);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "R", R_);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q_final", QFinal_);
  // costs over Cartesian velocities
  Eigen::Matrix<double, 12, 12> J_allFeet;
  kinematicModelPtr_->update(initRbdState_.template segment<18>(0));
  for (int leg = 0; leg < 4; ++leg) {
    Eigen::Matrix<double, 6, 12> J_thisfoot;
    kinematicModelPtr_->footJacobainBaseFrame(leg, J_thisfoot);
    J_allFeet.block<3, 12>(3 * leg, 0) = J_thisfoot.bottomRows<3>();
  }

  const double alpha = modelSettings_.torqueMixingFactor_;
  R_.template block<12, 12>(0, 0) =
      ((1.0 - alpha) * R_.template block<12, 12>(0, 0) + alpha * J_allFeet * R_.template block<12, 12>(0, 0) * J_allFeet.transpose())
          .eval();
  R_.template block<12, 12>(12, 12) = (J_allFeet.transpose() * R_.template block<12, 12>(12, 12) * J_allFeet).eval();

  if (INPUT_DIM == 4 * JOINT_COORD_SIZE) {
    R_.template block<12, 12>(24, 24) =
        ((1.0 - alpha) * R_.template block<12, 12>(24, 24) + alpha * J_allFeet * R_.template block<12, 12>(24, 24) * J_allFeet.transpose())
            .eval();
    R_.template block<12, 12>(36, 36) = (J_allFeet.transpose() * R_.template block<12, 12>(36, 36) * J_allFeet).eval();
  }

  // target state
  base_coordinate_t comFinalPose;
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "CoM_final_pose", comFinalPose);
  xFinal_ = initialState_;
  xFinal_.template head<6>() += comFinalPose;

  // load the mode sequence template
  std::cerr << std::endl;
  loadModeSequenceTemplate(pathToConfigFile, "initialModeSequenceTemplate", initialModeSequenceTemplate_, false);
  std::cerr << std::endl;
  if (initialModeSequenceTemplate_.templateSubsystemsSequence_.size() == 0){
    throw std::runtime_error("initialModeSequenceTemplate.templateSubsystemsSequence should have at least one entry.");
  }
  if (initialModeSequenceTemplate_.templateSwitchingTimes_.size() != initialModeSequenceTemplate_.templateSubsystemsSequence_.size() + 1) {
    throw std::runtime_error(
        "initialModeSequenceTemplate.templateSwitchingTimes size should be 1 + "
        "size_of(initialModeSequenceTemplate.templateSubsystemsSequence).");
  }

  initSwitchingModes_ = initialModeSequenceTemplate_.templateSubsystemsSequence_;
  initSwitchingModes_.push_back(string2ModeNumber("STANCE"));
  initNumSubsystems_ = initSwitchingModes_.size();

  auto& templateSwitchingTimes = initialModeSequenceTemplate_.templateSwitchingTimes_;
  initEventTimes_ = scalar_array_t(templateSwitchingTimes.begin() + 1, templateSwitchingTimes.end());

  // stanceLeg sequence
  initStanceLegSequene_.resize(initNumSubsystems_);
  for (size_t i = 0; i < initNumSubsystems_; i++) initStanceLegSequene_[i] = modeNumber2StanceLeg(initSwitchingModes_[i]);

  // display
  std::cerr << "Initial Switching Modes: {";
  for (const auto& switchingMode : initSwitchingModes_) {
    std::cerr << switchingMode << ", ";
  }
  std::cerr << "\b\b}" << std::endl;
  std::cerr << "Initial Event Times:     {";
  for (const auto& switchingtime : initEventTimes_) {
    std::cerr << switchingtime << ", ";
  }
  if (!initEventTimes_.empty()) {
    std::cerr << "\b\b}" << std::endl;
  } else {
    std::cerr << "}" << std::endl;
  }

  // load the mode sequence template
  std::cerr << std::endl;
  loadModeSequenceTemplate(pathToConfigFile, "defaultModeSequenceTemplate", defaultModeSequenceTemplate_, true);
  std::cerr << std::endl;

  // Gap Indicators
  loadGaps(pathToConfigFile, gapIndicatorPtrs_, true);

  switchingModes_ = initSwitchingModes_;
  stanceLegSequene_ = initStanceLegSequene_;
  switchingTimes_ = initEventTimes_;

  // Ground profile class
  scalar_t groundHight;
  estimateFlatGround(initRbdState_, contact_flag_t{1, 1, 1, 1}, groundHight);
  std::cerr << "Ground Profile Height: " << groundHight << std::endl << std::endl;
  groundProfilePtr_ = flat_ground_profile_ptr_t(new flat_ground_profile_t(groundHight));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
ocs2::SLQ_Settings& OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::slqSettings() {
  return slqSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeSwitchedModelState(const rbd_state_vector_t& rbdState,
                                                                                               state_vector_t& comkinoState) {
  typename state_estimator_t::comkino_model_state_t comKinoState_truesize;
  switchedModelStateEstimator_.estimateComkinoModelState(rbdState, comKinoState_truesize);
  comkinoState.template segment<12 + JOINT_COORD_SIZE>(0) = comKinoState_truesize;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeRbdModelState(const state_vector_t& comkinoState,
                                                                                          const input_vector_t& comkinoInput,
                                                                                          rbd_state_vector_t& rbdState) {
  switchedModelStateEstimator_.estimateRbdModelState(comkinoState.template segment<12 + JOINT_COORD_SIZE>(0),
                                                     comkinoInput.template segment<JOINT_COORD_SIZE>(12), rbdState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeComLocalAcceleration(const state_vector_t& comkinoState,
                                                                                                 const input_vector_t& comkinoInput,
                                                                                                 base_coordinate_t& comLocalAcceleration) {
  typedef Eigen::Matrix<scalar_t, 3, 3> matrix_3_t;
  typedef Eigen::Matrix<scalar_t, 3, 1> vector_3_t;
  typedef Eigen::Matrix<scalar_t, 6, 6> matrix_6_t;
  typedef Eigen::Matrix<scalar_t, 6, 1> vector_6_t;

  static vector_3_t o_gravityVector = vector_3_t(0.0, 0.0, -modelSettings_.gravitationalAcceleration_);

  Eigen::VectorBlock<const state_vector_t, 6> xCOM = comkinoState.template segment<6>(0);
  Eigen::VectorBlock<const state_vector_t, 3> b_W_com = comkinoState.template segment<3>(6);
  Eigen::VectorBlock<const state_vector_t, 3> b_V_com = comkinoState.template segment<3>(9);
  Eigen::VectorBlock<const state_vector_t, 12> qJoints = comkinoState.template segment<12>(12);
  Eigen::VectorBlock<const input_vector_t, 12> dqJoints = comkinoInput.template segment<12>(12);
  Eigen::VectorBlock<const input_vector_t, 3 * 4> lambda = comkinoInput.template head<12>();

  // Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
  Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(xCOM.template head<3>());

  // base to CoM displacement in the CoM frame
  vector_3_t b_base2CoM = comModelPtr_->comPositionBaseFrame(qJoints);

  // base coordinate
  base_coordinate_t xBase;
  xBase.template head<3>() = xCOM.template head<3>();
  xBase.template tail<3>() = xCOM.template tail<3>() - o_R_b * b_base2CoM;

  // update kinematic model
  kinematicModelPtr_->update(xBase, qJoints);

  // base to stance feet displacement in the CoM frame
  std::array<vector_3_t, 4> b_base2StanceFeet;
  for (size_t i = 0; i < 4; i++) kinematicModelPtr_->footPositionBaseFrame(i, b_base2StanceFeet[i]);

  // Inertia matrix in the CoM frame and its derivatives
  matrix_6_t M = comModelPtr_->comInertia(qJoints);
  matrix_6_t dMdt = comModelPtr_->comInertiaDerivative(qJoints, dqJoints);
  matrix_3_t rotationMInverse = M.template topLeftCorner<3, 3>().inverse();
  matrix_6_t MInverse;
  MInverse << rotationMInverse, matrix_3_t::Zero(), matrix_3_t::Zero(), (1 / M(5, 5)) * matrix_3_t::Identity();

  // Coriolis and centrifugal forces
  vector_6_t C;
  C.template head<3>() = b_W_com.cross(M.template topLeftCorner<3, 3>() * b_W_com) + dMdt.template topLeftCorner<3, 3>() * b_W_com;
  C.template tail<3>().setZero();

  // gravity effect on CoM in CoM coordinate
  vector_6_t MInverseG;
  MInverseG << vector_3_t::Zero(), -o_R_b.transpose() * o_gravityVector;

  // contact JacobianTransposeLambda
  vector_3_t b_comToFoot;
  vector_6_t JcTransposeLambda = vector_6_t::Zero();
  for (size_t i = 0; i < 4; i++) {
    b_comToFoot = b_base2StanceFeet[i] - b_base2CoM;

    JcTransposeLambda.template head<3>() += b_comToFoot.cross(lambda.template segment<3>(3 * i));
    JcTransposeLambda.template tail<3>() += lambda.template segment<3>(3 * i);
  }

  // CoM acceleration about CoM frame
  comLocalAcceleration = MInverse * (-C + JcTransposeLambda) - MInverseG;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeComStateInOrigin(const state_vector_t& comkinoState,
                                                                                             const input_vector_t& comkinoInput,
                                                                                             base_coordinate_t& o_comPose,
                                                                                             base_coordinate_t& o_comVelocity,
                                                                                             base_coordinate_t& o_comAcceleration) {
  Eigen::VectorBlock<const state_vector_t, 3> o_r_com = comkinoState.template segment<3>(3);
  Eigen::VectorBlock<const state_vector_t, 3> b_W_com = comkinoState.template segment<3>(6);
  Eigen::VectorBlock<const state_vector_t, 3> b_V_com = comkinoState.template segment<3>(9);

  // compute CoM local acceleration about CoM frame
  base_coordinate_t comLocalAcceleration;
  computeComLocalAcceleration(comkinoState, comkinoInput, comLocalAcceleration);

  // Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
  Eigen::Matrix3d o_R_b = kinematicModelPtr_->rotationMatrixOrigintoBase().transpose();

  // CoM pose in the origin frame
  o_comPose << comkinoState.template head<3>(), o_r_com;

  // CoM velocity in the origin frame
  o_comVelocity.template head<3>() = o_R_b * b_W_com;
  o_comVelocity.template tail<3>() = o_R_b * b_V_com;

  // CoM acceleration in the origin frame
  o_comAcceleration.template head<3>() = o_R_b * comLocalAcceleration.template head<3>();
  o_comAcceleration.template tail<3>() = o_R_b * comLocalAcceleration.template tail<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::estimateFlatGround(const rbd_state_vector_t& rbdState,
                                                                                        const contact_flag_t& contactFlag,
                                                                                        scalar_t& groundHight) const {
  kinematicModelPtr_->update(rbdState.template head<6 + JOINT_COORD_SIZE>());

  std::array<Eigen::Vector3d, 4> feetPositions;
  kinematicModelPtr_->feetPositionsOriginFrame(feetPositions);

  scalar_t totalHeight = 0.0;
  int feetInContact = 0;
  for (size_t j = 0; j < 4; j++)
    if (contactFlag[j] == true) {
      feetInContact++;
      totalHeight += feetPositions[j](2);
    }
  groundHight = totalHeight / scalar_t(feetInContact);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::kinematic_model_t&
OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getKinematicModel() {
  return *kinematicModelPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::com_model_t&
OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getComModel() {
  return *comModelPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::slq_base_t&
OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getSLQ() {
  return *slqPtr_;
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::slq_base_ptr_t&
OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getSLQPtr() {
  return slqPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getPerformanceIndeces(scalar_t& costFunction, scalar_t& constriantISE1,
                                                                                           scalar_t& constriantISE2) const {
  costFunction = costFunction_;
  constriantISE1 = constriantISE1_;
  constriantISE2 = constriantISE2_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getOptimizedControllerPtr(
    linear_controller_ptr_array_t& controllersPtrStock) const {
  controllersPtrStock.clear();
  for (const auto& controllerPtr : controllersPtrStock_) {
    controllersPtrStock.emplace_back(dynamic_cast<linear_controller_t*>(controllerPtr));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getOptimizedTrajectoriesPtr(
    const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr, const state_vector_array2_t*& stateTrajectoriesStockPtr,
    const input_vector_array2_t*& inputTrajectoriesStockPtr) const {
  timeTrajectoriesStockPtr = timeTrajectoriesStockPtr_;
  stateTrajectoriesStockPtr = stateTrajectoriesStockPtr_;
  inputTrajectoriesStockPtr = inputTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getEventTimesPtr(const scalar_array_t*& eventTimesPtr) const {
  eventTimesPtr = &eventTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getSubsystemsSequencePtr(
    const size_array_t*& subsystemsSequencePtr) const {
  subsystemsSequencePtr = &subsystemsSequence_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getContactFlagsSequencePtr(
    const std::vector<contact_flag_t>*& contactFlagsSequencePtr) const {
  contactFlagsSequencePtr = &contactFlagsSequence_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getGapIndicatorPtrs(
    std::vector<EndEffectorConstraintBase::ConstPtr>& gapIndicatorPtrs) const {
  gapIndicatorPtrs = gapIndicatorPtrs_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getIterationsLog(eigen_scalar_array_t& iterationCost,
                                                                                      eigen_scalar_array_t& iterationISE1,
                                                                                      eigen_scalar_array_t& iterationISE2) const {
  iterationCost = iterationCost_;
  iterationISE1 = iterationISE1_;
  iterationISE2 = ocs2Iterationcost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
Model_Settings& OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::modelSettings() {
  return modelSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getLoadedInitialState(rbd_state_vector_t& initRbdState) const {
  initRbdState = initRbdState_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getLoadedTimeHorizon(scalar_t& timeHorizon) const {
  timeHorizon = timeHorizon_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::concatenate() {
  timeTrajectory_ = timeTrajectoriesStockPtr_->at(0);
  stateTrajectory_ = stateTrajectoriesStockPtr_->at(0);
  inputTrajectory_ = inputTrajectoriesStockPtr_->at(0);
  for (size_t i = 1; i < numSubsystems_; i++) {
    timeTrajectory_.insert(timeTrajectory_.end(), timeTrajectoriesStockPtr_->at(i).begin(), timeTrajectoriesStockPtr_->at(i).end());
    stateTrajectory_.insert(stateTrajectory_.end(), stateTrajectoriesStockPtr_->at(i).begin(), stateTrajectoriesStockPtr_->at(i).end());
    inputTrajectory_.insert(inputTrajectory_.end(), inputTrajectoriesStockPtr_->at(i).begin(), inputTrajectoriesStockPtr_->at(i).end());
  }

  controllerTimeTrajectory_.clear();
  controllerFBTrajectory_.clear();
  controllerFFTrajector_.clear();
  for (size_t i = 0; i < numSubsystems_; i++) {
    auto linearCtrlPtr = dynamic_cast<linear_controller_t*>(controllersPtrStock_[i]);
    controllerTimeTrajectory_.insert(controllerTimeTrajectory_.end(), linearCtrlPtr->timeStamp_.begin(), linearCtrlPtr->timeStamp_.end());
    controllerFBTrajectory_.insert(controllerFBTrajectory_.end(), linearCtrlPtr->gainArray_.begin(), linearCtrlPtr->gainArray_.end());
    controllerFFTrajector_.insert(controllerFFTrajector_.end(), linearCtrlPtr->biasArray_.begin(), linearCtrlPtr->biasArray_.end());
  }

  linInterpolateState_.setTimeStamp(&timeTrajectory_);
  linInterpolateState_.setData(&stateTrajectory_);

  linInterpolateInput_.setTimeStamp(&timeTrajectory_);
  linInterpolateInput_.setData(&inputTrajectory_);

  linInterpolateUff_.setTimeStamp(&controllerTimeTrajectory_);
  linInterpolateUff_.setData(&controllerFFTrajector_);

  linInterpolateK_.setTimeStamp(&controllerTimeTrajectory_);
  linInterpolateK_.setData(&controllerFBTrajectory_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::runSLQ(
    const scalar_t& initTime, const rbd_state_vector_t& initRbdState, const scalar_t& finalTime,
    const linear_controller_ptr_array_t& initialControllersStock /*=linear_controller_ptr_array_t()*/) {
  // reference trajectories
  input_vector_t uNominalForWeightCompensation;
  designWeightCompensatingInput(initialState_, uNominalForWeightCompensation);

  // reference time
  costDesiredTrajectories_.desiredTimeTrajectory().resize(2);
  costDesiredTrajectories_.desiredTimeTrajectory().at(0) = initEventTimes_.front();
  costDesiredTrajectories_.desiredTimeTrajectory().at(1) = initEventTimes_.back();
  // reference state
  costDesiredTrajectories_.desiredStateTrajectory().resize(2);
  costDesiredTrajectories_.desiredStateTrajectory().at(0) = initialState_;
  costDesiredTrajectories_.desiredStateTrajectory().at(1) = xFinal_;
  // reference inputs for weight compensation
  costDesiredTrajectories_.desiredInputTrajectory().resize(2);
  costDesiredTrajectories_.desiredInputTrajectory().at(0) = uNominalForWeightCompensation;
  costDesiredTrajectories_.desiredInputTrajectory().at(1) = uNominalForWeightCompensation;

  slqPtr_->setCostDesiredTrajectories(costDesiredTrajectories_);

  // TODO: numSubsystems_ is not set
  //	scalar_array_t costAnnealingStartTimes(numSubsystems_, switchingTimes_.front());
  //	scalar_array_t costAnnealingFinalTimes(numSubsystems_, switchingTimes_.back());

  // run slqp
  if (initialControllersStock.empty() == true) {
    std::cerr << "Cold initialization." << std::endl;
    slqPtr_->run(initTime_, initialState_, finalTime_, partitioningTimes_);

  } else {
    std::cerr << "Warm initialization." << std::endl;
    controller_ptr_array_t initialControllersPtrStockTemp;
    for (auto& controllerPtr_i : initialControllersStock) initialControllersPtrStockTemp.push_back(controllerPtr_i);
    slqPtr_->run(initTime_, initialState_, finalTime_, partitioningTimes_, initialControllersPtrStockTemp);
  }

  // get the optimizer outputs
  ocs2Iterationcost_.clear();
  slqPtr_->getIterationsLog(iterationCost_, iterationISE1_, iterationISE2_);
  slqPtr_->getPerformanceIndeces(costFunction_, constriantISE1_, constriantISE2_);

  controllersPtrStock_ = slqPtr_->getController();
  slqPtr_->getNominalTrajectoriesPtr(timeTrajectoriesStockPtr_, stateTrajectoriesStockPtr_, inputTrajectoriesStockPtr_);

  // get gait sequence (should be copied since it might be overridden in the next iteration)
  eventTimes_ = logicRulesPtr_->eventTimes();
  subsystemsSequence_ = logicRulesPtr_->subsystemsSequence();
  contactFlagsSequence_ = logicRulesPtr_->getContactFlagsSequence();

  //	concatenate();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
bool OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::runMPC(const scalar_t& initTime, const rbd_state_vector_t& initState) {
  initTime_ = initTime;
  computeSwitchedModelState(initState, initialState_);

  // update controller
  bool controllerIsUpdated = mpcPtr_->run(initTime_, initialState_);

  // get the optimizer outputs
  mpcPtr_->getOptimizedControllerPtr(controllersPtrStock_);

  mpcPtr_->getOptimizedTrajectoriesPtr(timeTrajectoriesStockPtr_, stateTrajectoriesStockPtr_, inputTrajectoriesStockPtr_);

  // get gait sequence (should be copied since it might be overridden in the next iteration)
  eventTimes_ = logicRulesPtr_->eventTimes();
  subsystemsSequence_ = logicRulesPtr_->subsystemsSequence();
  //	contactFlagsSequence_ = mpcPtr_->getLogicRulesPtr()->getContactFlagsSequence();

  return controllerIsUpdated;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::runOCS2(const scalar_t& initTime,
                                                                             const rbd_state_vector_t& initHyQState,
                                                                             const scalar_array_t& switchingTimes /*=scalar_array_t()*/) {
  //	if (switchingTimes.empty()==true)
  //		switchingTimes_ = initEventTimes_;
  //	else
  //		switchingTimes_ = switchingTimes;
  //
  //	initTime_ = initTime;
  //	computeSwitchedModelState(initHyQState, initSwitchedState_);
  //
  //	// run ocs2
  //	ocs2Ptr_->run(initTime_, initSwitchedState_, switchingTimes_.back(), initSystemStockIndexes_, switchingTimes_,
  //			controller_array_t(),
  //			desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
  //
  //	// get the optimizer outputs
  //	ocs2Ptr_->getOCS2IterationsLog(ocs2Iterationcost_);
  //	ocs2Ptr_->getSLQIterationsLog(iterationCost_, iterationISE1_);
  //	ocs2Ptr_->getCostFunction(costFunction_);
  //	constriantISE_ = iterationISE1_.back()(0);
  //	ocs2Ptr_->getSwitchingTimes(switchingTimes_);
  //	ocs2Ptr_->getController(controllersStock_);
  //	ocs2Ptr_->getNominalTrajectories(timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::loadSimulationSettings(const std::string& filename, scalar_t& dt,
                                                                                            scalar_t& tFinal, scalar_t& initSettlingTime) {
  const scalar_t defaultInitSettlingTime = 1.5;
  boost::property_tree::ptree pt;

  try {
    boost::property_tree::read_info(filename, pt);
    dt = pt.get<scalar_t>("simulationSettings.dt");
    tFinal = pt.get<scalar_t>("simulationSettings.tFinal");
    initSettlingTime = pt.get<scalar_t>("simulationSettings.initSettlingTime", defaultInitSettlingTime);
  } catch (const std::exception& e) {
    std::cerr << "Tried to open file " << filename << " but failed: " << std::endl;
    std::cerr << "Error in loading simulation settings: " << e.what() << std::endl;
    throw;
  }
  std::cerr << "Simulation Settings: " << std::endl;
  std::cerr << "=====================================" << std::endl;
  std::cerr << "Simulation time step ......... " << dt << std::endl;
  std::cerr << "Controller simulation time ... [0, " << tFinal << "]" << std::endl;
  std::cerr << "initial settling time ......... " << initSettlingTime << std::endl << std::endl;
}

}  // end of namespace switched_model
